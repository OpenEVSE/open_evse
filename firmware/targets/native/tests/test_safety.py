"""Safety behaviour of the OpenEVSE firmware, run natively.

These are the cases that are awkward or destructive to stage on real
hardware: making the GFI self-test fail, welding a contactor, opening an
earth mid-charge. Each drives the firmware through its hardware channel and
asserts on what the firmware itself decides, reported over RAPI.
"""

import pytest

from bench import (
    STATE_B, STATE_C, STATE_GFCI_FAULT, STATE_GFI_TEST_FAILED,
    STATE_NO_GROUND, STATE_STUCK_RELAY,
)

POSTCODE_OK = 0x00


# --- power-on self test ----------------------------------------------------

def test_gfi_self_test_passes_when_detector_answers(bench):
    """The firmware pulses the test coil and requires the trip back before it
    will run. With the detector wired, POST comes up clean."""
    assert bench.wait_boot() == POSTCODE_OK
    assert bench.out.get("GFITEST") == 0, "test coil left energised after POST"


def test_gfi_self_test_fails_when_detector_is_dead(firmware, tmp_path):
    """A GFI detector that never answers must fail POST rather than charge.

    This is the case worth having: on hardware it means sabotaging the GFI
    circuit on a live board, so it tends not to get tested at all.
    """
    from bench import Bench

    with Bench(firmware, eeprom=str(tmp_path / "eeprom.bin")) as b:
        b.gfi_detector = False          # coil energises, nothing trips
        assert b.wait_boot() == 0x09    # postcode 09, GFI self-test failure
        b.wait_state(STATE_GFI_TEST_FAILED)
        assert b.out.get("CHARGING") == 0, "relay closed after a failed POST"


# --- a normal session ------------------------------------------------------

def test_charging_session_reaches_state_c(bench):
    assert bench.wait_boot() == POSTCODE_OK
    bench.enable()

    bench.set_pilot("B")
    bench.wait_state(STATE_B)
    assert bench.out.get("CHARGING") == 0, "relay closed before state C"

    bench.set_pilot("C")
    bench.wait_state(STATE_C)
    bench.wait_relay(closed=True)


@pytest.mark.parametrize("amps,duty_tenths", [
    (6, 100),    # J1772: amps / 0.6
    (16, 266),
    (24, 400),
    (32, 533),
])
def test_pilot_duty_matches_ampacity(bench, amps, duty_tenths):
    """Duty cycle advertises the ampacity to the vehicle, so getting the
    scaling wrong silently offers the car the wrong current."""
    assert bench.wait_boot() == POSTCODE_OK
    bench.enable()
    bench.rapi("$SC %d" % amps)

    bench.set_pilot("B")
    state, duty, reported = bench.wait_pilot_pwm()

    assert state == "PWM"
    assert duty == duty_tenths, (
        "%dA advertised %.1f%%, expected %.1f%%" % (amps, duty / 10, duty_tenths / 10))
    assert reported == amps


# --- faults during a session ----------------------------------------------

def test_gfi_trip_opens_the_relay(charging):
    """An earth fault while charging must drop the contactor."""
    charging.gfi_trip()
    charging.wait_state(STATE_GFCI_FAULT)
    charging.wait_relay(closed=False)


def test_lost_ground_opens_the_relay(charging):
    """Losing earth mid-charge must drop the contactor.

    The firmware only runs this check GROUND_CHK_DELAY (1s) after the relay
    closes and only while it is closed, so the wait here is the firmware's
    own settling time rather than slack in the test.
    """
    charging.open_ground()
    charging.wait_state(STATE_NO_GROUND, timeout=15)
    charging.wait_relay(closed=False)


def test_welded_relay_is_detected(charging):
    """A contactor that will not open is the fault the load-side sense line
    exists to catch: the firmware commands the relay open, still sees mains
    downstream, and must report a stuck relay."""
    charging.weld_relay()
    charging.set_pilot("A")          # unplug, so the firmware opens the relay
    charging.wait_state(STATE_STUCK_RELAY)
