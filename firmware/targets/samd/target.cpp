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


// Helper function to wait for ADC register synchronization
static inline void adcSync() {
  while (ADC->STATUS.bit.SYNCBUSY);
}

bool AdcPin::_adcInitDone = false;

void AdcPin::_adcInit() {
  // 1. Enable the APB clock for the ADC so the CPU can talk to it
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  // 2. Set up the Peripheral Clock (GCLK) for the ADC
  // We attach GCLK0 (48MHz) to the ADC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
                      GCLK_CLKCTRL_GEN_GCLK0 | 
                      GCLK_CLKCTRL_ID_ADC;
  while (GCLK->STATUS.bit.SYNCBUSY); // Wait for clock sync

  // 3. Load factory calibration values from NVM (Non-Volatile Memory)
  // This maintains ADC accuracy even at high speeds
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  
  adcSync();
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  // 4. Configure CTRLB: 12-bit resolution & a faster Prescaler
  // DIV64 runs the ADC clock at 48MHz / 64 = 750kHz (well within datasheet specs for high speed)
  // For maximum speed, change DIV64 to DIV32 (1.5MHz clock, slightly noisier)
  adcSync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 | 
                   ADC_CTRLB_RESSEL_12BIT;

  // 5. Configure Reference and Input settings
  // Set reference to internal VCC/2 (approx 1.65V) and half gain so we can measure the full 0-3.3V scale
  adcSync();
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1; // VCC/2
  
  adcSync();
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | 
                       ADC_INPUTCTRL_MUXNEG_GND; // Single-ended to GND

  // 6. Set sample time to minimum (0)
  adcSync();
  ADC->SAMPCTRL.reg = 0x00; 

  // 7. Enable the ADC
  adcSync();
  ADC->CTRLA.bit.ENABLE = 1;
  adcSync();
}

// WARNING: ONLY WORKS ON A0-A5, just like analogRead()
uint32_t AdcPin::read() {
  if (!_adcInitDone) {
    _adcInit();
    _adcInitDone = true;
  }
  else {
    // Enable the ADC in case someone else turned it off
    adcSync();
    ADC->CTRLA.bit.ENABLE = 1;
    adcSync();
  }

  // Map Arduino pins to SAMD21 ADC channel (MUXPOS)
  uint32_t pinMux = 0;
  
  // Safe mapping for standard Arduino Zero analog pins (A0 to A5)
  switch(_pinNum) {
    case A0: pinMux = ADC_INPUTCTRL_MUXPOS_PIN0;  break; // AIN0
    case A1: pinMux = ADC_INPUTCTRL_MUXPOS_PIN2;  break; // AIN2
    case A2: pinMux = ADC_INPUTCTRL_MUXPOS_PIN3;  break; // AIN3
    case A3: pinMux = ADC_INPUTCTRL_MUXPOS_PIN4;  break; // AIN4
    case A4: pinMux = ADC_INPUTCTRL_MUXPOS_PIN5;  break; // AIN5
    case A5: pinMux = ADC_INPUTCTRL_MUXPOS_PIN10; break; // AIN10
    default: return -1; // Invalid pin
  }

  // 1. Select the correct analog pin (MUXPOS)
  adcSync();
  ADC->INPUTCTRL.bit.MUXPOS = pinMux;

  // 2. Trigger conversion using software trigger
  adcSync();
  ADC->SWTRIG.bit.START = 1;

  // 3. Clear the Result Ready flag to prepare for reading
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // 4. Wait for the conversion to complete
  while (ADC->INTFLAG.bit.RESRDY == 0);

  // 5. Return the 12-bit result (0 to 4095)
  return ADC->RESULT.reg;
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
