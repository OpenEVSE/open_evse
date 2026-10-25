import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from bench import Bench  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def binary_for(env):
    return os.path.join(REPO_ROOT, ".pio", "build", env, "program")


def pytest_addoption(parser):
    parser.addoption(
        "--env", default="native_oev6",
        help="PlatformIO environment to test (native_oev6 or native_nxt)",
    )


@pytest.fixture(scope="session")
def firmware(request):
    env = request.config.getoption("--env")
    path = binary_for(env)
    if not os.path.exists(path):
        pytest.skip("%s not built -- run: pio run -e %s" % (path, env))
    return path


@pytest.fixture
def bench(firmware, tmp_path):
    """A fresh bench per test, with its own EEPROM so state cannot leak
    between tests. The firmware persists settings and trip counters, so a
    shared EEPROM would make results depend on execution order."""
    with Bench(firmware, eeprom=str(tmp_path / "eeprom.bin")) as b:
        yield b


@pytest.fixture
def charging(bench):
    """A bench already delivering charge: booted clean, enabled, EV plugged in
    and drawing. The starting point for the fault tests."""
    from bench import STATE_C

    assert bench.wait_boot() == 0, "expected a clean POST"
    bench.enable()
    bench.set_pilot("B")
    bench.wait_pilot_pwm()
    bench.set_pilot("C")
    bench.wait_state(STATE_C)
    bench.wait_relay(closed=True)
    return bench
