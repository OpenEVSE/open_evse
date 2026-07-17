#include "open_evse.h"

ExternalEEPROM g_eeprom;

//                                               A/B  B/C  C/D  D DS
THRESH_DATA J1772EVSEController::m_ThreshData = {3932,3517,3225,0,492};

#ifdef MCU_ID_LEN
// mcuid *must* be of size MCU_ID_LEN
void getMcuId(uint8_t *mcuid)
{
  for (uint8_t word = 0; word < 4; word++) {
    uint32_t value;
    switch (word) {
      case 0:
        value = *((volatile uint32_t *)0x0080A00C);
        break;
      case 1:
        value = *((volatile uint32_t *)0x0080A040);
        break;
      case 2:
        value = *((volatile uint32_t *)0x0080A044);
        break;
      default:
        value = *((volatile uint32_t *)0x0080A048);
        break;
    }
    mcuid[(word * 4) + 0] = (uint8_t)(value & 0xFF);
    mcuid[(word * 4) + 1] = (uint8_t)((value >> 8) & 0xFF);
    mcuid[(word * 4) + 2] = (uint8_t)((value >> 16) & 0xFF);
    mcuid[(word * 4) + 3] = (uint8_t)((value >> 24) & 0xFF);
  }
}
#endif // MCU_ID_LEN


#ifdef RELAY_ZC_SWITCH
// --- GMI zero-cross ADC on PA09 / AIN[17] -----------------------------------
//
// The AC voltage zero-cross ("GMI") line is on PA09.  analogRead() cannot read
// it, for two independent reasons:
//
//   1. Pin-number remap: analogRead(pin) does `if (pin < A0) pin += A0;`
//      (wiring_analog.c).  GMI_ADC_PIN is Arduino pin 3 (PA09), and 3 < A0(14),
//      so analogRead(3) actually samples pin 3+14 = 17 = A3 = PA04 — an
//      unconnected, floating pin.
//   2. Even without the remap, the arduino_zero variant descriptor for PA09
//      carries No_ADC_Channel, so analogRead() has no ADC channel to select for
//      it.  PA09's real ADC input is AIN[17] (INPUTCTRL.MUXPOS = 0x11), which
//      the variant table simply cannot express.
//
// So we drive the ADC directly on MUXPOS = 0x11 here, following the same
// enable / discard-first-conversion / read / disable sequence the core's
// analogRead() uses, so it coexists cleanly with analogRead() on the other
// AdcPins (which enable/disable the ADC per call and leave it disabled).
//
// PA09 is *also* the digital ACLINE2 ground-monitor input (pinAC2, INP_PU),
// polled by ReadACPins().  gmiAdcBegin() switches it to the analog mux with the
// pull-up OFF (an enabled pull-up would bias the sample); gmiAdcEnd() restores
// it to a digital input with pull-up, faithfully reproducing DigitalPin INP_PU.
//
// Do NOT "simplify" any of this back to analogRead() — it would read PA04.

#define GMI_ADC_MUXPOS 0x11u // AIN[17] = PA09

// Wait for ADC register synchronization across clock domains (SAMD21 requires
// this after CTRLA.ENABLE, INPUTCTRL and before SWTRIG writes).
static inline void gmiSyncADC()
{
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

void gmiAdcBegin()
{
  EPortType port = (EPortType)g_APinDescription[GMI_ADC_PIN].ulPort;
  uint32_t   pin  = g_APinDescription[GMI_ADC_PIN].ulPin; // PA09 -> 9

  // Route PA09 to peripheral function B (analog) in the PORT pin-mux.  PA09 is
  // odd, so write PMUXO while preserving the even-nibble (PA08) muxing.
  uint32_t pmux = PORT->Group[port].PMUX[pin >> 1].reg & PORT_PMUX_PMUXE(0xF);
  PORT->Group[port].PMUX[pin >> 1].reg = pmux | PORT_PMUX_PMUXO(PIO_ANALOG);
  // Enable the mux and clear INEN + PULLEN: no digital input buffer, and no
  // pull-up to bias the analog reading.
  PORT->Group[port].PINCFG[pin].reg = PORT_PINCFG_PMUXEN;

  // Select AIN[17] as the positive input.  MUXNEG stays at GND (core default).
  gmiSyncADC();
  ADC->INPUTCTRL.bit.MUXPOS = GMI_ADC_MUXPOS;

  gmiSyncADC();
  ADC->CTRLA.bit.ENABLE = 0x01; // enable ADC
  gmiSyncADC();

  // The first conversion after a MUXPOS change is not valid (SAMD21 datasheet);
  // trigger one and discard it.
  ADC->SWTRIG.bit.START = 1;
  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; // clear ready flag
}

uint16_t gmiAdcRead()
{
  gmiSyncADC();
  ADC->SWTRIG.bit.START = 1;
  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;
  return (uint16_t)ADC->RESULT.reg; // 12-bit result; reading RESULT clears RESRDY
}

void gmiAdcEnd()
{
  gmiSyncADC();
  ADC->CTRLA.bit.ENABLE = 0x00; // disable ADC (leave it as analogRead() expects)
  gmiSyncADC();

  // Restore PA09 to the digital ACLINE2 ground-monitor input with pull-up.
  // pinMode() rewrites PINCFG (clearing PMUXEN, setting INEN|PULLEN), does
  // DIRCLR, and drives OUT high for the pull-up — exactly DigitalPin INP_PU.
  pinMode(GMI_ADC_PIN, INPUT_PULLUP);
}
#endif // RELAY_ZC_SWITCH


void DigitalPin::init(uint32_t pinnum,int idxjunk,PinMode mode)
{
  _pinNum = pinnum;
  if (mode == INP) {
    _pinMode = INPUT;
  }
  else if (mode == INP_PU) {
    _pinMode = INPUT_PULLUP;
  }
  else {
    _pinMode = OUTPUT;
  }
  pinMode(_pinNum,_pinMode);
}


// platform-specific init
void initTarget()
{
  g_hasCGMI = true;

  Wire.begin();
  g_eeprom.setMemoryType(512);
  if (g_eeprom.begin() == false) {
    g_EvseController.SetState(EVSE_STATE_EEPROM_FAILURE);
    RapiInit();
    RapiSendBootNotification();
    while(1);
  }

  //n.b. set BOD via fuses, not this code, as fuses may lock out BOD from being
  // manipulated in code, anyway
  // default factory fuse setting is 1.78V BOD enabled
#ifdef DONTUSE
    // 1. Prepare the configuration value level 7 = ~1.78V
    uint32_t bod33_config = SYSCTRL_BOD33_LEVEL(7)     | 
                            SYSCTRL_BOD33_HYST      | 
                            SYSCTRL_BOD33_ACTION_RESET | 
                            SYSCTRL_BOD33_ENABLE;

    // 2. Wait for the SYSCTRL to be ready/synchronized 
    // (BOD33 is often in a different clock domain)
    while (SYSCTRL->PCLKSR.bit.BOD33RDY == 0);

    // 3. Disable before configuring if it was already running 
    // (Optional, but ensures Level change takes effect cleanly)
    SYSCTRL->BOD33.bit.ENABLE = 0;
    while (SYSCTRL->PCLKSR.bit.BOD33RDY == 0);

    // 4. Write the full configuration
    SYSCTRL->BOD33.reg = bod33_config;

    // 5. Wait for synchronization again to ensure it's active
    while (SYSCTRL->PCLKSR.bit.BOD33RDY == 0);
#endif // DONTUSE
}
