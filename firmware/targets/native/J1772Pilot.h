// -*- C++ -*-
//
// J1772 pilot for the native (host) target.
//
// There is no timer peripheral here. SetPWM() records the duty the controller
// asked for and SetState() records P12/N12; both are published to whatever is
// driving the hardware, which decides what the vehicle does about it. The
// pilot voltage the firmware then reads back arrives through PILOT_SENSE_PIN
// like any other ADC input.

#pragma once

typedef enum {
  PILOT_STATE_P12, PILOT_STATE_PWM, PILOT_STATE_N12
}
PILOT_STATE;

class J1772Pilot {
  PILOT_STATE m_State;
  DigitalPin pin;
  int m_Amps;      // last requested ampacity, -1 when not in PWM
  uint32_t m_Duty; // 0..1000, tenths of a percent

public:
  J1772Pilot() : m_State(PILOT_STATE_N12), m_Amps(-1), m_Duty(0) {}
  void Init();
  void SetState(PILOT_STATE pstate); // P12/N12
  PILOT_STATE GetState() {
    return m_State;
  }
  int SetPWM(int amps); // 12V 1KHz PWM

  // Native-only accessors, for the control channel.
  int Amps() { return m_Amps; }
  uint32_t DutyTenthsPercent() { return m_Duty; }
};
