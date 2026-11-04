"""Exercise the actual relay methods with simulated interrupt delivery.

Run with: python3 -m unittest discover -s tests
Requires a C++ compiler. This is a host simulation, not a hardware test.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'firmware/open_evse/J1772EvseController.cpp').read_text()
METHODS = (SOURCE[SOURCE.index('void J1772EVSEController::chargingOn()'):
                  SOURCE.index('#ifdef PP_AUTO_AMPACITY\nvoid J1772EVSEController::DoPPError')]
           + SOURCE[SOURCE.index('void J1772EVSEController::SetGfiTripped()'):
                    SOURCE.index('#endif // GFI\n\nvoid J1772EVSEController::EnableDiodeCheck')]
           + SOURCE[SOURCE.index('uint8_t J1772EVSEController::doPost()'):
                    SOURCE.index('#endif // ADVPWR\n\nvoid J1772EVSEController::Init()')])

HARNESS = r'''
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#define GFI
#define RELAY_ZC_SWITCH
#define AUTOSVCLEVEL
#define CHARGING_REG
#define CHARGING2_REG
#define CHARGINGAC_REG
#define HIGH 1
#define LOW 0
#define V6_CHARGING_PIN 0
#define V6_CHARGING_PIN2 1
#define ECVF_CHARGING_ON 1
#define ECVF_GFI_TRIPPED 2
#define ECVF_SESSION_ENDED 4
#define PILOT_STATE_P12 1
#define PILOT_STATE_PWM 2
#define PILOT_STATE_N12 3
#define WDT_RESET() do {} while (0)
#define RelaySettlingTime 1
#define UD 0
#define L1 1
#define L2 2
#define OG 3
#define SR 4
#define FG 5
#define both 0
#define L1on 1
#define L2on 2
#define none 3
#define RLY_TEST_PIN_OPEN 1
#define ACPINS_OPEN 3
using std::uint8_t;
using std::uint16_t;
static bool irq_enabled = true, pending = false, in_isr = false, tripped = false;
static int trip_at = -1, event_count = 0, writes_after_trip = 0;
static bool outputs[3];
void deliver();
void event() {
  if (in_isr) return;
  if (event_count++ == trip_at) {
    if (irq_enabled) deliver(); else pending = true;
  }
}
struct AutoCriticalSection {
  bool previous;
  AutoCriticalSection() : previous(irq_enabled) { irq_enabled = false; }
  ~AutoCriticalSection() {
    irq_enabled = previous;
    if (irq_enabled && pending) { pending = false; deliver(); }
  }
};
void output(int pin, bool state) {
  if (state) { event(); if (tripped) ++writes_after_trip; }
  outputs[pin] = state;
}
void digitalWrite(int pin, int state) { output(pin, state); }
void analogWrite(int pin, int state) { output(pin, state != 0); }
void delay(int) { event(); }
void wdt_delay(int) { event(); }
unsigned long millis() { event(); return 10; }
struct Pin { int n; void write(int v) { output(n,v); } };
struct Gfi { bool fault = false; bool Fault() { return fault; } void SetFault() { fault = true; } };
struct Pilot { void SetState(int) {} };
struct Display { void SetRedLed(int) {} void SetGreenLed(int) {} } g_OBD;
struct Adc { int read() { event(); return 100; } };
struct J1772EVSEController {
  Gfi m_Gfi; Pilot m_Pilot;
  Adc adcPilot;
  struct { int m_ThreshAB = 50; } m_ThreshData;
  Pin pinCharging{0}, pinCharging2{1}, pinChargingAC{2};
  uint16_t flags = 0;
  int m_relayCloseMs = 10, m_relayHoldPwm = 100;
  unsigned long m_ChargeOnTimeMS = 0, m_ChargeOffTimeMS = 0;
  unsigned long m_AccumulatedChargeTime = 0, m_ElapsedChargeTime = 0;
  bool v6 = true, enabled[3] = {true,true,true};
  bool isV6() { return v6; }
  bool RelayDC1Enabled() { return enabled[0]; }
  bool RelayDC2Enabled() { return enabled[1]; }
  bool RelayACEnabled() { return enabled[2]; }
  bool hasCGMI() { return true; }
  bool AutoSvcLevelEnabled() { return true; }
  bool StuckRelayChkEnabled() { return true; }
  bool GndChkEnabled() { return true; }
  void SetAutoSvcLvlSkipped(int) {}
  uint8_t ReadACPins() { event(); return none; }
  bool RelayZCSwitchEnabled() { return true; }
  void zcWaitRelayClose() { event(); }
  void zcWaitRelayOpen() {}
  bool chargingIsOn() { return flags & ECVF_CHARGING_ON; }
  void setVFlags(int f) { flags |= f; }
  void clrVFlags(int f) { flags &= ~f; }
  bool vFlagIsSet(int f) { return flags & f; }
  void chargingOn(); void chargingOff(uint8_t emergency = 0); void SetGfiTripped(); uint8_t doPost();
} controller;
void deliver() {
  in_isr = true; tripped = true;
  controller.SetGfiTripped();
  in_isr = false;
}
''' + METHODS + r'''
void reset() {
  controller = J1772EVSEController();
  irq_enabled = true; pending = in_isr = tripped = false;
  event_count = writes_after_trip = 0; trip_at = -1;
  for (bool &output : outputs) output = false;
}
void check(bool value, const char *message, int point) {
  if (!value) { std::fprintf(stderr, "%s at event %d\n", message, point); std::exit(1); }
}
int main() {
  for (int legacy = 0; legacy < 2; ++legacy) {
    reset(); controller.v6 = !legacy; controller.chargingOn();
    int events = event_count;
    check(outputs[0] && outputs[1] && outputs[2] && controller.chargingIsOn(), "normal close", legacy);
    for (int point = 0; point < events; ++point) {
      reset(); controller.v6 = !legacy; trip_at = point; controller.chargingOn();
      check(tripped && controller.m_Gfi.Fault(), "fault latched", point);
      check(!outputs[0] && !outputs[1] && !outputs[2], "outputs open", point);
      check(!controller.chargingIsOn(), "charging flag cleared", point);
      check(!writes_after_trip, "output driven after interrupt", point);
    }
    reset(); controller.v6 = !legacy; controller.m_Gfi.SetFault(); controller.chargingOn();
    check(!outputs[0] && !outputs[1] && !outputs[2] && !controller.chargingIsOn(), "existing fault", legacy);
    reset(); controller.v6 = !legacy; controller.enabled[1] = false; controller.chargingOn();
    check(outputs[0] && !outputs[1] && outputs[2] && controller.chargingIsOn(), "disabled relay", legacy);

    reset(); controller.v6 = !legacy; controller.doPost();
    int post_events = event_count;
    for (int point = 0; point < post_events; ++point) {
      reset(); controller.v6 = !legacy; trip_at = point; controller.doPost();
      check(tripped && controller.m_Gfi.Fault(), "POST fault latched", point);
      check(!outputs[0] && !outputs[1] && !outputs[2], "POST outputs open", point);
      check(!writes_after_trip, "POST output driven after interrupt", point);
    }
    reset(); controller.v6 = !legacy; controller.m_Gfi.SetFault(); controller.doPost();
    check(!outputs[0] && !outputs[1] && !outputs[2], "POST existing fault", legacy);
  }
}
'''

GFI_HARNESS = r'''
#include <cstdint>
#include <cstdlib>
using std::uint8_t;
#define GFI
#define GFI_SELFTEST
#define OPENEVSE_2
#define GFI_TEST_CYCLES 2
#define GFI_PULSE_ON_US 1
#define GFI_PULSE_OFF_US 1
#define WDT_RESET() do {} while (0)
static bool irq_enabled = true, pending = false, trip_on_read = false, input_high = false;
void trip();
struct AutoCriticalSection {
  bool previous;
  AutoCriticalSection() : previous(irq_enabled) { irq_enabled = false; }
  ~AutoCriticalSection() {
    irq_enabled = previous;
    if (irq_enabled && pending) { pending = false; trip(); }
  }
};
void test_signal();
struct DigitalPin {
  int read() {
    bool sample = input_high;
    if (trip_on_read) {
      trip_on_read = false;
      if (irq_enabled) trip(); else pending = true;
    }
    return sample;
  }
  void write(int state) { if (state) test_signal(); }
};
void delay(int) {}
void delayMicroseconds(int) {}
void wdt_delay(int) {}
'''


class GfiRelayTest(unittest.TestCase):
    def test_interrupt_interleavings(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'relay.cpp'
            source.write_text(HARNESS)
            for defines in ([], ['-DOEV6'], ['-DOEV6', '-DRELAY_PWM']):
                with self.subTest(defines=defines):
                    binary = Path(temp) / 'relay'
                    subprocess.run(['c++', '-std=c++11', '-Wall', '-Werror', *defines,
                                    str(source), '-o', str(binary)], check=True)
                    subprocess.run([str(binary)], check=True)

    def test_reset_and_self_test_preserve_real_trips(self):
        for target in ('m328p', 'samd'):
            with self.subTest(target=target), tempfile.TemporaryDirectory() as temp:
                path = ROOT / 'firmware/targets' / target
                header = (path / 'Gfi.h').read_text().replace('#pragma once', '')
                code = (path / 'Gfi.cpp').read_text()
                methods = code[code.index('void Gfi::Reset()'):code.rindex('#endif // GFI')]
                main = r'''
Gfi gfi;
void trip() { gfi.SetFault(); }
void test_signal() { gfi.SetTestSuccess(); }
int main() {
  input_high = false; gfi.Reset();
  if (gfi.Fault()) return 1;
  input_high = true; gfi.Reset();
  if (!gfi.Fault()) return 2;
  input_high = false; gfi.Reset();
  if (gfi.Fault()) return 3;
  trip_on_read = true; gfi.Reset();
  if (!gfi.Fault()) return 4;
  if (gfi.SelfTest() || !gfi.Fault()) return 5;
  gfi.Reset();
  if (gfi.SelfTest() || gfi.Fault()) return 6;
}
'''
                source = Path(temp) / 'gfi.cpp'
                source.write_text(GFI_HARNESS + header + methods + main)
                binary = Path(temp) / 'gfi'
                subprocess.run(['c++', '-std=c++11', '-Wall', '-Werror',
                                str(source), '-o', str(binary)], check=True)
                subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
