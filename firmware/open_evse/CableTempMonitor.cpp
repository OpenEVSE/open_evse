/*
 * Open EVSE Firmware - cable NTC thermistor temperature monitoring
 *
 * This file is part of Open EVSE.

 * Open EVSE is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * Open EVSE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "open_evse.h"

#ifdef CABLE_TEMPERATURE_MONITORING

CableTempMonitor g_CableTempMonitor;

//
// Normalized NTC curve: R/R25 * 2000, tabulated every 5C from -40C to 150C,
// generated for beta = CABLE_TEMP_CURVE_BETA (3443) using
//   R/R25 = exp(B * (1/T - 1/298.15))
//
// Storing the *ratio* rather than absolute ohms is what makes R25
// configurable: any NTC of the same curve shape is handled by dividing its
// measured resistance by its own R25 before the lookup. A different beta is
// then corrected arithmetically in RatioToTempC10(), so no logarithm - and
// therefore no floating point - is needed anywhere.
//
// Linear interpolation between 5C steps costs under 0.1C of curve-fit error
// across the range that matters (25C..120C).
//
#define CABLE_TEMP_CURVE_MIN_C     -40
#define CABLE_TEMP_CURVE_STEP_C      5
#define CABLE_TEMP_CURVE_RATIO_SCALE 2000UL
#define CABLE_TEMP_CURVE_CNT        39

static const uint16_t s_ntcCurveX2000[CABLE_TEMP_CURVE_CNT] PROGMEM = {
  50028, //  -40C  R/R25=25.0141
  36691, //  -35C  R/R25=18.3457
  27255, //  -30C  R/R25=13.6277
  20490, //  -25C  R/R25=10.2450
  15579, //  -20C  R/R25=7.78932
  11971, //  -15C  R/R25=5.98543
   9291, //  -10C  R/R25=4.64557
   7280, //   -5C  R/R25=3.63988
   5755, //    0C  R/R25=2.87749
   4588, //    5C  R/R25=2.29409
   3687, //   10C  R/R25=1.84366
   2986, //   15C  R/R25=1.49296
   2435, //   20C  R/R25=1.21770
   2000, //   25C  R/R25=1.00000
   1653, //   30C  R/R25=0.82657
   1375, //   35C  R/R25=0.68746
   1150, //   40C  R/R25=0.57514
    968, //   45C  R/R25=0.48387
    819, //   50C  R/R25=0.40927
    696, //   55C  R/R25=0.34794
    594, //   60C  R/R25=0.29725
    510, //   65C  R/R25=0.25512
    440, //   70C  R/R25=0.21995
    381, //   75C  R/R25=0.19043
    331, //   80C  R/R25=0.16555
    289, //   85C  R/R25=0.14448
    253, //   90C  R/R25=0.12657  (1265.7 ohms for a 10k NTC - the datasheet point)
    223, //   95C  R/R25=0.11128
    196, //  100C  R/R25=0.09817
    174, //  105C  R/R25=0.08690
    154, //  110C  R/R25=0.07716
    137, //  115C  R/R25=0.06873
    123, //  120C  R/R25=0.06140
    110, //  125C  R/R25=0.05500
     99, //  130C  R/R25=0.04941
     89, //  135C  R/R25=0.04450
     80, //  140C  R/R25=0.04018
     73, //  145C  R/R25=0.03637
     66, //  150C  R/R25=0.03300
};

// Below this ratio the reading is called a short rather than a temperature.
// Deliberately set as low as it can usefully go - R/R25 < 0.001, i.e. under
// 10 ohms on a 10k NTC - because everything between here and the bottom of
// the curve table clamps to the table's top temperature and therefore trips
// the over-temperature check. Widening this band would carve a hole in that
// protection: a thermistor hot enough to fall through it would report
// "shorted", which does not fault, and would silently clear an
// over-temperature fault raised on the way up.
#define CABLE_TEMP_SHORT_RATIO_X2000 2

// 273.15K and 298.15K in 10ths of a kelvin, for the beta correction
#define CABLE_TEMP_KELVIN_OFFSET_K10 2732
#define CABLE_TEMP_T25_K10           2982
// 1/T fixed-point scale: invT = CABLE_TEMP_INVT_SCALE / Tk10. one count is
// worth ~0.013C at 90C, so the correction adds no meaningful error.
#define CABLE_TEMP_INVT_SCALE        100000000L


void CableTempMonitor::Init()
{
  m_AdcPP.init(PP_PIN);
  m_AdcPP2.init(PP2_PIN);

  m_LastUpdate = 0;

  uint8_t flags = eeprom_read_byte((uint8_t*)EOFS_CABLE_TEMP_FLAGS);
  if (flags == 0xff) flags = 0; // unformatted eeprom - feature off
  m_Flags = flags & CTMF_ENABLED; // OVERTEMPERATURE is volatile

  for (uint8_t i=0;i < CABLE_TEMP_SENSOR_CNT;i++) {
    LoadCfg(i);
    m_TempC10[i] = CABLE_TEMP_NOT_INSTALLED;
  }

#ifdef PP_AUTO_AMPACITY
  // Corruption guard only: SetPin()/SetCfg() and EnablePPAutoAmpacity()
  // enforce exclusivity at the point of change, so a saved configuration
  // should never claim PP_READ for both functions. If one somehow does,
  // proximity pilot keeps the pin.
  if (g_EvseController.PPAutoAmpacityIsEnabled()) {
    ReleasePPPin();
  }
#endif // PP_AUTO_AMPACITY
}


void CableTempMonitor::LoadCfg(uint8_t idx)
{
  uintptr_t ofs = EOFS_CABLE_TEMP_CFG + (uintptr_t)idx * CABLE_TEMP_CFG_EEPROM_SIZE;
  CABLE_TEMP_CFG *c = &m_Cfg[idx];

  c->r25    = eeprom_read_word((uint16_t*)(ofs));
  c->beta   = eeprom_read_word((uint16_t*)(ofs+2));
  c->offset = (int16_t)eeprom_read_word((uint16_t*)(ofs+4));
  c->panic  = (int16_t)eeprom_read_word((uint16_t*)(ofs+6));
  c->pin    = eeprom_read_byte((uint8_t*)(ofs+8));

  // fall back to the Phoenix Contact NACS defaults for anything unformatted
  // or out of range
  if ((c->r25 < CABLE_TEMP_MIN_R25) || (c->r25 == 0xffff)) c->r25 = CABLE_TEMP_DEFAULT_R25;
  if ((c->beta < CABLE_TEMP_MIN_BETA) || (c->beta > CABLE_TEMP_MAX_BETA)) c->beta = CABLE_TEMP_DEFAULT_BETA;
  if ((c->offset < CABLE_TEMP_MIN_OFFSET) || (c->offset > CABLE_TEMP_MAX_OFFSET)) c->offset = CABLE_TEMP_DEFAULT_OFFSET;
  if ((c->panic < CABLE_TEMP_MIN_PANIC) || (c->panic > CABLE_TEMP_MAX_PANIC)) c->panic = CABLE_TEMP_DEFAULT_PANIC;
  if (c->pin > CABLE_TEMP_PIN_MAX) c->pin = CABLE_TEMP_PIN_NONE;
}


void CableTempMonitor::SaveCfg(uint8_t idx)
{
  uintptr_t ofs = EOFS_CABLE_TEMP_CFG + (uintptr_t)idx * CABLE_TEMP_CFG_EEPROM_SIZE;
  CABLE_TEMP_CFG *c = &m_Cfg[idx];

  eeprom_write_word((uint16_t*)(ofs),  c->r25);
  eeprom_write_word((uint16_t*)(ofs+2),c->beta);
  eeprom_write_word((uint16_t*)(ofs+4),(uint16_t)c->offset);
  eeprom_write_word((uint16_t*)(ofs+6),(uint16_t)c->panic);
  eeprom_write_byte((uint8_t*)(ofs+8), c->pin);
}


void CableTempMonitor::Enable(int8_t tf)
{
  if (tf) m_Flags |= CTMF_ENABLED;
  else {
    m_Flags &= ~(CTMF_ENABLED|CTMF_OVERTEMPERATURE);
    for (uint8_t i=0;i < CABLE_TEMP_SENSOR_CNT;i++) {
      m_TempC10[i] = CABLE_TEMP_NOT_INSTALLED;
    }
  }
  eeprom_write_byte((uint8_t*)EOFS_CABLE_TEMP_FLAGS,m_Flags & CTMF_ENABLED);
}


int8_t CableTempMonitor::UsesPPPin()
{
  for (uint8_t i=0;i < CABLE_TEMP_SENSOR_CNT;i++) {
    if (m_Cfg[i].pin == CABLE_TEMP_PIN_PP) return 1;
  }
  return 0;
}


void CableTempMonitor::ReleasePPPin()
{
  for (uint8_t i=0;i < CABLE_TEMP_SENSOR_CNT;i++) {
    if (m_Cfg[i].pin == CABLE_TEMP_PIN_PP) {
      m_Cfg[i].pin = CABLE_TEMP_PIN_NONE;
      m_TempC10[i] = CABLE_TEMP_NOT_INSTALLED;
      SaveCfg(i);
    }
  }
}


int8_t CableTempMonitor::SetPin(int32_t idx,int32_t pin)
{
  if ((idx < 0) || (idx >= CABLE_TEMP_SENSOR_CNT) ||
      (pin < 0) || (pin > CABLE_TEMP_PIN_MAX)) return 1;

  m_Cfg[idx].pin = (uint8_t)pin;
  m_TempC10[idx] = CABLE_TEMP_NOT_INSTALLED;
  SaveCfg(idx);

#ifdef PP_AUTO_AMPACITY
  // PP_READ has one owner. Claiming it for temperature monitoring turns
  // proximity-pilot auto-ampacity off; $FF P 1 takes it back the same way.
  if ((pin == CABLE_TEMP_PIN_PP) && g_EvseController.PPAutoAmpacityIsEnabled()) {
    g_EvseController.EnablePPAutoAmpacity(0);
  }
#endif // PP_AUTO_AMPACITY

  return 0;
}


int8_t CableTempMonitor::SetCfg(int32_t idx,int32_t pin,int32_t r25,int32_t beta,int32_t offset,int32_t panic)
{
  if ((idx < 0) || (idx >= CABLE_TEMP_SENSOR_CNT) ||
      (pin < 0) || (pin > CABLE_TEMP_PIN_MAX)) return 1;
  if ((r25 < CABLE_TEMP_MIN_R25) || (r25 > (int32_t)CABLE_TEMP_MAX_R25) ||
      (beta < CABLE_TEMP_MIN_BETA) || (beta > CABLE_TEMP_MAX_BETA) ||
      (offset < CABLE_TEMP_MIN_OFFSET) || (offset > CABLE_TEMP_MAX_OFFSET) ||
      (panic < CABLE_TEMP_MIN_PANIC) || (panic > CABLE_TEMP_MAX_PANIC)) return 1;

  CABLE_TEMP_CFG *c = &m_Cfg[idx];
  c->r25 = (uint16_t)r25;
  c->beta = (uint16_t)beta;
  c->offset = (int16_t)offset;
  c->panic = (int16_t)panic;

  return SetPin(idx,pin); // saves and enforces the PP interlock
}


uint16_t CableTempMonitor::ReadAdcAvg(uint8_t pin)
{
  AdcPin *ap = (pin == CABLE_TEMP_PIN_PP) ? &m_AdcPP : &m_AdcPP2;

  uint32_t acc = 0;
  for (uint8_t i=0;i < CABLE_TEMP_ADC_SAMPLES;i++) {
    acc += ap->read();
  }
  return (uint16_t)(acc / CABLE_TEMP_ADC_SAMPLES);
}


//
// Map a normalized resistance R/R25 (scaled by 2000) to a temperature in
// 10ths of a degree C, correcting for a thermistor beta that differs from
// the curve table's.
//
// The correction is exact, not an approximation. For any beta,
//   ln(R/R25) = B * (1/T - 1/T25)
// so reading the table (which assumes B0) yields a Tref satisfying
//   ln(R/R25) = B0 * (1/Tref - 1/T25)
// and equating the two gives
//   1/T = 1/T25 + (B0/B) * (1/Tref - 1/T25)
// which is plain integer arithmetic - no logarithm required.
//
int16_t CableTempMonitor::RatioToTempC10(uint32_t ratioX2000,uint16_t beta)
{
  if (ratioX2000 < CABLE_TEMP_SHORT_RATIO_X2000) return CABLE_TEMP_SHORTED;

  uint16_t hi = pgm_read_word(&s_ntcCurveX2000[0]);
  // colder than the bottom of the table. At this end the divider is within a
  // couple of ADC counts of the rail, so a genuine sub- -40C reading and a
  // disconnected thermistor are not distinguishable - report it as open.
  if (ratioX2000 > hi) return CABLE_TEMP_OPEN;

  uint16_t lo = pgm_read_word(&s_ntcCurveX2000[CABLE_TEMP_CURVE_CNT-1]);
  // Hotter than the top of the table - clamp rather than error out, so the
  // over-temperature check still trips. Reported as a flat 150.0C, which
  // means "at or above the top of the measurable range" rather than a
  // measurement; for a thermistor whose beta differs from the curve's, the
  // true temperature at the clamp point can be a good deal lower. Clamping
  // high is the fail-safe direction - a beta-corrected (and therefore lower)
  // clamp value could sit under a high panic threshold and never trip it.
  if (ratioX2000 <= lo) {
    return (int16_t)((CABLE_TEMP_CURVE_MIN_C + (CABLE_TEMP_CURVE_CNT-1)*CABLE_TEMP_CURVE_STEP_C) * 10);
  }

  // the table descends with temperature, so walk down until ratioX2000
  // brackets between entry i and i+1
  uint8_t i = 0;
  uint16_t a = hi;
  uint16_t b = a;
  for (i=0;i < (CABLE_TEMP_CURVE_CNT-1);i++) {
    a = pgm_read_word(&s_ntcCurveX2000[i]);
    b = pgm_read_word(&s_ntcCurveX2000[i+1]);
    if (ratioX2000 > b) break; // a >= ratioX2000 > b
  }

  int32_t tC10 = (int32_t)(CABLE_TEMP_CURVE_MIN_C + (int16_t)i*CABLE_TEMP_CURVE_STEP_C) * 10;
  // linear interpolation within the 5C step
  tC10 += ((int32_t)(a - ratioX2000) * (CABLE_TEMP_CURVE_STEP_C*10)) / (int32_t)(a - b);

  if (beta != CABLE_TEMP_CURVE_BETA) {
    int32_t tK10 = tC10 + CABLE_TEMP_KELVIN_OFFSET_K10;
    if (tK10 < 1) tK10 = 1;
    int32_t invT0  = CABLE_TEMP_INVT_SCALE / CABLE_TEMP_T25_K10;
    int32_t invRef = CABLE_TEMP_INVT_SCALE / tK10;
    int32_t invT   = invT0 + ((invRef - invT0) * (int32_t)CABLE_TEMP_CURVE_BETA) / (int32_t)beta;
    if (invT < 1) invT = 1;
    tC10 = (CABLE_TEMP_INVT_SCALE / invT) - CABLE_TEMP_KELVIN_OFFSET_K10;
  }

  return (int16_t)tC10;
}


int16_t CableTempMonitor::ReadSensor(uint8_t idx)
{
  CABLE_TEMP_CFG *c = &m_Cfg[idx];
  if (c->pin == CABLE_TEMP_PIN_NONE) return CABLE_TEMP_NOT_INSTALLED;

  uint32_t adc = ReadAdcAvg(c->pin);

  // divider pinned at the rail: nothing pulling the node down, so either no
  // cable is plugged in or the thermistor circuit is open
  if (adc >= ADC_MAX) return CABLE_TEMP_OPEN;

  // Rntc = Rpullup * adc / (ADC_MAX - adc)
  // adc*Rpullup peaks at 4095*1000, well inside 32 bits
  uint32_t rOhms = (adc * CABLE_TEMP_PULLUP_OHMS) / (uint32_t)(ADC_MAX - adc);

  // cap before scaling by 2000 so the multiply cannot overflow; anything
  // this cold is off the bottom of the curve anyway
  uint32_t rMax = ((uint32_t)pgm_read_word(&s_ntcCurveX2000[0]) * c->r25) / CABLE_TEMP_CURVE_RATIO_SCALE;
  if (rOhms > rMax) return CABLE_TEMP_OPEN;

  uint32_t ratioX2000 = (rOhms * CABLE_TEMP_CURVE_RATIO_SCALE) / (uint32_t)c->r25;

  int16_t t = RatioToTempC10(ratioX2000,c->beta);
  if (!CABLE_TEMP_IS_VALID(t)) return t; // sentinel - don't apply calibration to it

  return t + c->offset;
}


void CableTempMonitor::Read()
{
  if (!(m_Flags & CTMF_ENABLED)) return;

  unsigned long curms = millis();
  if ((curms - m_LastUpdate) < CABLE_TEMPMONITOR_UPDATE_INTERVAL) return;
  m_LastUpdate = curms;

  uint8_t overtemp = 0;
  for (uint8_t i=0;i < CABLE_TEMP_SENSOR_CNT;i++) {
    int16_t t = ReadSensor(i);
    m_TempC10[i] = t;
    if (CABLE_TEMP_IS_VALID(t) && (t >= m_Cfg[i].panic)) overtemp = 1;
  }

  if (overtemp) m_Flags |= CTMF_OVERTEMPERATURE;
  else m_Flags &= ~CTMF_OVERTEMPERATURE;
}

#endif // CABLE_TEMPERATURE_MONITORING
