#!/usr/bin/env python3
"""Throwaway driver proving the control channel closes the loop.

Not the harness -- just enough of a fake vehicle to show that the firmware's
own decisions come back out over the wire:

  1. GFI self-test: the firmware pulses GFITEST; we answer by raising GFI, and
     it should clear postcode 09 and stop reporting the self-test failure.
  2. Plug in: hold PILOT_SENSE at the state B band and the firmware should
     start PWM at the configured ampacity.
  3. Charge: drop to the state C band and the firmware should close CHARGING.
  4. Fault: trip GFI mid-charge and the firmware should open CHARGING again.

Run:  drive.py <socket-path> <rapi-in-fifo> <binary-stdout-log>
"""
import os
import socket
import sys
import time

# Pilot sense ADC counts, per board mode. The firmware compares the positive
# peak against m_ThreshData {A/B, B/C, C/D, D, DS}; we sit clear of each
# boundary. NEG is the negative half of the pilot square wave: it must be below
# m_ThreshDS or the firmware fails its diode check once PWM starts.
BANDS = {
    "oev6": {"A": 950, "B": 830, "C": 730, "NEG": 50},    # thresh 875/780/690, DS 260
    "nxt":  {"A": 4000, "B": 3700, "C": 3350, "NEG": 90},  # thresh 3932/3517/3225, DS 492
}


class Driver:
    def __init__(self, path):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        for _ in range(100):
            try:
                self.sock.connect(path)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.05)
        else:
            raise SystemExit(f"could not connect to {path}")
        self.sock.setblocking(False)
        self.buf = ""
        self.out = {}
        self.pilot = None
        self.board = None
        self.log = []
        self.gfi_pulses = 0
        self.answer_gfi = True   # model the GFI test circuit as always present

    def pump(self, seconds=0.2):
        end = time.time() + seconds
        while time.time() < end:
            try:
                data = self.sock.recv(4096).decode()
                if not data:
                    return
                self.buf += data
            except BlockingIOError:
                time.sleep(0.005)
                continue
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                self._handle(line.strip())

    def _handle(self, line):
        if not line:
            return
        self.log.append(line)
        parts = line.split()
        if parts[0] == "OUT":
            self.out[parts[1]] = int(parts[2])
            # The GFI test coil is wired through the detector on real hardware,
            # so the detector follows the coil: energised -> tripped, and it
            # answers every self-test, not just the one at boot. Following the
            # coil state also leaves GFI asserted long enough for the firmware
            # to poll it -- raising and clearing it in one batch would put both
            # levels in the same read, and the edge would never be seen.
            # Load-side AC sense follows the contactor. The AC-sense lines are
            # active low -- "voltage detected at the pin" pulls it low -- so a
            # closed relay puts mains on ACLINE1 and drives it to 0.
            if parts[1] == "CHARGING":
                self.send("IN ACLINE1 %d" % (0 if int(parts[2]) else 1))
            if parts[1] == "GFITEST" and self.answer_gfi:
                if int(parts[2]) == 1:
                    self.gfi_pulses += 1
                    self.send("IN GFI 1")
                else:
                    self.send("IN GFI 0")
        elif parts[0] == "PILOT":
            self.pilot = (parts[1], int(parts[2]), int(parts[3]))
        elif parts[0] == "HELLO":
            kv = dict(p.split("=", 1) for p in parts[1:] if "=" in p)
            self.board = kv.get("board")
            print(f"   HELLO {kv}")

    def send(self, line):
        self.sock.sendall((line + "\n").encode())

    def set_pilot(self, band):
        """Present the pilot as the square wave it physically is: the state's
        positive plateau paired with the -12V negative half."""
        b = BANDS[self.board]
        self.send(f"ADC PILOT_SENSE {b[band]} {b['NEG']}")


def rapi(cmd):
    """Frame a RAPI command with its additive checksum (sum of all chars
    before '*', the '$' included)."""
    return "%s*%02X\r" % (cmd, sum(cmd.encode()) & 0xFF)


def check(label, ok):
    print(f"   [{'PASS' if ok else 'FAIL'}] {label}")
    return ok


def main():
    sock_path, rapi_fifo = sys.argv[1], sys.argv[2]
    d = Driver(sock_path)
    while d.board is None:               # wait for HELLO before using bands
        d.pump(0.1)

    # Energise the bench: ground present (active low), relay open so no load
    # voltage yet, no current, pilot idling at +12V.
    d.send("IN ACLINE2 0")
    d.send("IN ACLINE1 1")
    d.set_pilot("A")
    d.pump(0.2)

    results = []

    # --- 1. answer the GFI self-test -------------------------------------
    # The firmware pulses GFITEST and waits for the trip. Watch for the pulse
    # and answer it, which is exactly what the real GFI circuit does.
    deadline = time.time() + 8
    while time.time() < deadline and d.gfi_pulses == 0:
        d.pump(0.05)
    results.append(check("firmware ran its GFI self-test", d.gfi_pulses > 0))

    # --- 2. plug the car in ----------------------------------------------
    with open(rapi_fifo, "w") as f:               # enable, clear boot lock
        f.write(rapi("$FE") + rapi("$SB"))
        f.flush()
    d.pump(0.5)

    d.set_pilot("B")
    d.pump(2.0)
    results.append(check("pilot went to PWM on state B",
                         d.pilot is not None and d.pilot[0] == "PWM"))
    if d.pilot:
        print(f"        pilot={d.pilot[0]} duty={d.pilot[1]/10:.1f}% amps={d.pilot[2]}")

    # --- 3. car starts drawing -------------------------------------------
    d.set_pilot("C")
    d.pump(3.0)
    results.append(check("CHARGING relay closed on state C",
                         d.out.get("CHARGING") == 1))

    # --- 4. GFI trips mid-charge -----------------------------------------
    d.answer_gfi = False   # stop auto-answering; this trip is a real fault
    # Only meaningful if the relay actually closed: asserting it is open when
    # it was never closed passes without testing anything.
    if d.out.get("CHARGING") == 1:
        d.send("IN GFI 1")
        d.pump(2.0)
        results.append(check("CHARGING relay opened on GFI trip",
                             d.out.get("CHARGING") == 0))
    else:
        results.append(check("GFI trip test (skipped, relay never closed)", False))

    print(f"\n   channel lines received: {len(d.log)}, "
          f"GFI self-tests answered: {d.gfi_pulses}")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
