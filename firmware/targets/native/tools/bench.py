#!/usr/bin/env python3
"""A bench for the natively-built OpenEVSE safety firmware.

Launches the firmware binary, drives its hardware control channel, and speaks
RAPI to it, so tests can put the real firmware in a situation and assert on
what it decides.

The bench models the board around the controller -- the GFI detector, the
contactor's load-side sense, the pilot as a square wave -- because that is
what the firmware is wired to and what it reacts to. It deliberately models
no vehicle policy beyond the pilot level a test asks for: deciding what a
pilot voltage *means* is the firmware's job, and the moment the bench starts
deciding too, the test stops proving anything.

Reactions run on their own thread. They have to: the firmware pulses the GFI
test coil for about a second and waits for the trip inside that loop, so a
bench that only reacted when a test happened to poll would miss it.
"""

import os
import selectors
import socket
import subprocess
import threading
import time

# --- EVSE states, from J1772EvseController.h -------------------------------
STATE_UNKNOWN         = 0x00
STATE_A               = 0x01  # not connected
STATE_B               = 0x02  # connected, ready
STATE_C               = 0x03  # charging
STATE_D               = 0x04  # vent required
STATE_DIODE_CHK_FAILED = 0x05
STATE_GFCI_FAULT      = 0x06
STATE_NO_GROUND       = 0x07
STATE_STUCK_RELAY     = 0x08
STATE_GFI_TEST_FAILED = 0x09
STATE_OVER_TEMPERATURE = 0x0A
STATE_OVER_CURRENT    = 0x0B
STATE_SLEEPING        = 0xFE
STATE_DISABLED        = 0xFF

STATE_NAMES = {v: k for k, v in list(globals().items()) if k.startswith("STATE_")}

# Pilot sense ADC counts per board. The firmware compares the positive peak
# against m_ThreshData {A/B, B/C, C/D, D, DS}; these sit clear of each
# boundary. NEG is the negative half of the pilot square wave and must stay
# below m_ThreshDS, or the firmware fails its own diode check once PWM starts.
PILOT_BANDS = {
    #        A     B     C     D    NEG      thresholds        DS
    "oev6": {"A": 950,  "B": 830,  "C": 730,  "D": 500,  "NEG": 50},   # 875/780/690  260
    "nxt":  {"A": 4000, "B": 3700, "C": 3350, "D": 2200, "NEG": 90},   # 3932/3517/3225 492
}


class Timeout(AssertionError):
    """Raised when the firmware does not reach an expected condition."""


def rapi_frame(cmd):
    """Frame a RAPI command with its additive checksum: the 8-bit sum of every
    character before the '*', the leading '$' included."""
    return "%s*%02X\r" % (cmd, sum(cmd.encode()) & 0xFF)


class Bench:
    def __init__(self, binary, board=None, eeprom=None, socket_path=None,
                 wait_ms=10000):
        self.binary = binary
        self.board = board
        self.eeprom = eeprom
        # AF_UNIX paths are capped near 108 bytes, and a pytest tmp_path can
        # exceed that on its own, so keep the socket somewhere short.
        self.socket_path = socket_path or "/tmp/oevse-bench-%d.sock" % os.getpid()
        self.wait_ms = wait_ms

        self.proc = None
        self.sock = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._threads = []

        # observed state
        self.out = {}          # digital outputs, by signal name
        self.pilot = None      # (state, duty_tenths_pct, amps)
        self.hello = {}
        self.evse_state = None
        self.pilot_state = None
        self.boot_postcode = None
        self.rapi_lines = []
        self.channel_lines = []
        self._responses = []   # $OK / $NK lines, in order

        # bench behaviour, switchable per test
        self.gfi_detector = True     # answer the self-test coil
        self.ground_present = True   # ACLINE2 low == ground present
        self.relay_welded = False    # load-side sense stays live when open

    # -- lifecycle ---------------------------------------------------------

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    def start(self):
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)

        env = dict(os.environ)
        env["OPENEVSE_HW_SOCKET"] = self.socket_path
        env["OPENEVSE_HW_WAIT_MS"] = str(self.wait_ms)
        if self.eeprom:
            env["OPENEVSE_EEPROM"] = self.eeprom

        self.proc = subprocess.Popen(
            [self.binary],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, env=env, bufsize=0,
        )

        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = time.time() + 10
        while True:
            try:
                self.sock.connect(self.socket_path)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.time() > deadline:
                    raise Timeout("firmware never opened %s" % self.socket_path)
                time.sleep(0.02)

        self._spawn(self._channel_loop)
        self._spawn(self._rapi_loop)

        self._wait(lambda: self.hello, 10, "no HELLO from firmware")
        if self.board is None:
            self.board = self.hello.get("board")

        # Energise the bench before the firmware leaves its self tests:
        # ground present, contactor open, pilot idling at +12V.
        self._send_ground()
        self._send_aclines(charging=False)
        self.set_pilot("A")

    def stop(self):
        self._stop.set()
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)
        if self.sock:
            self.sock.close()
        for t in self._threads:
            t.join(timeout=2)
        if os.path.exists(self.socket_path):
            try:
                os.unlink(self.socket_path)
            except OSError:
                pass

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    # -- channel -----------------------------------------------------------

    def _channel_loop(self):
        sel = selectors.DefaultSelector()
        sel.register(self.sock, selectors.EVENT_READ)
        buf = ""
        while not self._stop.is_set():
            if not sel.select(timeout=0.05):
                continue
            try:
                data = self.sock.recv(4096).decode()
            except OSError:
                return
            if not data:
                return
            buf += data
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                self._on_channel(line.strip())

    def _on_channel(self, line):
        if not line:
            return
        parts = line.split()
        with self._lock:
            self.channel_lines.append(line)
            if parts[0] == "HELLO":
                self.hello = dict(p.split("=", 1) for p in parts[1:] if "=" in p)
                return
            if parts[0] == "PILOT":
                self.pilot = (parts[1], int(parts[2]), int(parts[3]))
                return
            if parts[0] != "OUT":
                return
            name, val = parts[1], int(parts[2])
            self.out[name] = val

        # React outside the lock; these send on the socket.
        if name == "GFITEST" and self.gfi_detector:
            # The test coil is wired through the detector, so the detector
            # follows the coil. Holding GFI for as long as the coil is
            # energised also leaves the firmware time to poll the level --
            # raising and clearing it together would put both levels in one
            # read and the edge would never be seen.
            self.send("IN GFI %d" % val)
        elif name == "CHARGING":
            self._send_aclines(charging=bool(val))

    def send(self, line):
        try:
            self.sock.sendall((line + "\n").encode())
        except OSError:
            pass

    # -- bench wiring ------------------------------------------------------

    def _send_ground(self):
        self._send_aclines()

    def _send_aclines(self, charging=None):
        """Drive both AC-sense lines. They are active low: voltage at the pin
        pulls it low.

        Losing earth takes both lines with it. The sense circuits are
        ground-referenced, so a lost earth reads as both pins open -- which is
        exactly what a non-CGMI board looks for: it calls bad ground only on
        acpinstate == ACPINS_OPEN, both pins, and never on ACLINE2 alone.
        """
        if charging is None:
            charging = bool(self.out.get("CHARGING", 0))

        if not self.ground_present:
            self.send("IN ACLINE1 1")
            self.send("IN ACLINE2 1")
            return

        live = charging or self.relay_welded
        self.send("IN ACLINE1 %d" % (0 if live else 1))
        self.send("IN ACLINE2 0")

    def set_pilot(self, band):
        """Present the pilot as the square wave it physically is: the band's
        positive plateau paired with the negative half."""
        b = PILOT_BANDS[self.board]
        self.send("ADC PILOT_SENSE %d %d" % (b[band], b["NEG"]))

    def set_current(self, counts_peak):
        """Ammeter CT, sampled peak-to-peak by readAmmeter()."""
        mid = int(self.hello.get("adc_max", 1023)) // 2
        self.send("ADC CURRENT %d %d" % (mid + counts_peak, mid - counts_peak))

    def gfi_trip(self):
        """A real earth fault, as opposed to the self-test coil."""
        self.gfi_detector = False
        self.send("IN GFI 1")

    def gfi_clear(self):
        self.send("IN GFI 0")
        self.gfi_detector = True

    def open_ground(self):
        self.ground_present = False
        self._send_aclines()

    def weld_relay(self):
        """Contactor stuck closed: load-side sense stays live with the relay
        commanded open."""
        self.relay_welded = True
        self._send_aclines(charging=True)

    # -- RAPI --------------------------------------------------------------

    def _rapi_loop(self):
        buf = ""
        while not self._stop.is_set():
            chunk = self.proc.stdout.read(1)
            if not chunk:
                return
            ch = chunk.decode(errors="replace")
            if ch in ("\r", "\n"):
                if buf:
                    self._on_rapi(buf)
                    buf = ""
            else:
                buf += ch

    def _on_rapi(self, line):
        with self._lock:
            self.rapi_lines.append(line)
        parts = line.split()
        if not parts:
            return
        # $AT evsestate pilotstate currentcapacity vflags
        if parts[0] == "$AT" and len(parts) >= 3:
            with self._lock:
                self.evse_state = int(parts[1], 16)
                self.pilot_state = int(parts[2], 16)
        # $AB postcode fwrev
        elif parts[0] == "$AB" and len(parts) >= 2:
            with self._lock:
                self.boot_postcode = int(parts[1], 16)
        elif line.startswith("$OK") or line.startswith("$NK"):
            # n.b. prefix, not an exact token match: a reply with no parameters
            # has its checksum hard against the code, as in "$OK^20".
            with self._lock:
                self._responses.append(line)

    def rapi(self, cmd, wait=True, timeout=10):
        """Send a RAPI command and, by default, wait for its response.

        Waiting matters more than it looks: a command that changes what the
        firmware advertises -- $SC, say -- takes effect when the firmware gets
        round to reading it, so a test that fires and moves on can sample the
        old value and fail intermittently.
        """
        with self._lock:
            before = len(self._responses)
        self.proc.stdin.write(rapi_frame(cmd).encode())
        self.proc.stdin.flush()
        if not wait:
            return None
        self._wait(lambda: len(self._responses) > before, timeout,
                   "response to %s" % cmd)
        with self._lock:
            return self._responses[-1]

    def enable(self):
        """Enable the EVSE and clear the boot lock, which BOOTLOCK builds
        require before they will charge."""
        self.rapi("$FE")
        self.rapi("$SB")

    # -- waiting -----------------------------------------------------------

    def _wait(self, pred, timeout, what):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if pred():
                    return
            time.sleep(0.02)
        raise Timeout("timed out after %.1fs waiting for %s" % (timeout, what))

    def wait_boot(self, timeout=20):
        self._wait(lambda: self.boot_postcode is not None, timeout,
                   "boot notification")
        return self.boot_postcode

    def wait_state(self, state, timeout=20):
        self._wait(lambda: self.evse_state == state, timeout,
                   "EVSE state %s, last seen %s" % (
                       STATE_NAMES.get(state, hex(state)),
                       STATE_NAMES.get(self.evse_state, self.evse_state)))

    def wait_relay(self, closed, timeout=20):
        want = 1 if closed else 0
        self._wait(lambda: self.out.get("CHARGING") == want, timeout,
                   "CHARGING relay %s" % ("closed" if closed else "open"))

    def wait_pilot_pwm(self, timeout=20):
        self._wait(lambda: self.pilot and self.pilot[0] == "PWM", timeout,
                   "pilot PWM")
        with self._lock:
            return self.pilot
