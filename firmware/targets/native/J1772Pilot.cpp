#include "open_evse.h"

void J1772Pilot::Init()
{
  pin.init(PILOT_REG, PILOT_IDX, DigitalPin::OUT);
  SetState(PILOT_STATE_P12);
}

// steady state -12V or +12V
void J1772Pilot::SetState(PILOT_STATE pstate)
{
  pin.write(pstate == PILOT_STATE_P12 ? 1 : 0);

  m_State = pstate;
  m_Amps = -1;
  m_Duty = (pstate == PILOT_STATE_P12) ? 1000 : 0;
}

// set EVSE current capacity in Amperes
int J1772Pilot::SetPWM(int amps)
{
  // Duty cycle per J1772, in tenths of a percent, using the same two ranges
  // the hardware targets use:
  //   6..51A : duty% = amps / 0.6
  //   52..80A: duty% = 64 + amps / 2.5
  uint32_t duty;
  if ((amps >= 6) && (amps <= 51)) {
    duty = ((uint32_t)amps * 1000u) / 6u;   // amps/0.6  -> 32A = 533 (53.3%)
  }
  else if ((amps > 51) && (amps <= 80)) {
    duty = 640u + ((uint32_t)amps * 4u);    // 64%+amps/2.5 -> 80A = 960 (96%)
  }
  else {
    return 1;
  }

  m_Duty = duty;
  m_Amps = amps;
  m_State = PILOT_STATE_PWM;

  return 0;
}
