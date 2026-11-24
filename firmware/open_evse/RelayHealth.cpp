#include "open_evse.h"

#ifdef RELAY_HEALTH

RelayHealth g_RelayHealth;

// Above this the (I/I_rated)^2 term would overflow the 64-bit numerator in
// OneOpenDamageX1e6() below; a hot open this large has already used up a
// big share of the electrical budget, so clamping it costs nothing.
#define RELAY_HEALTH_DAMAGE_MAX_MA 1000000L // 1000 A

// Miner's-rule electrical-damage contribution of a single hot open, in
// millionths of rated electrical life (N_e) consumed:
//   d = (1/N_e) * (I/I_rated)^2 * alpha_pf * alpha_T
// alpha_T is derived live from the MCP9808 ambient reading when
// TEMPERATURE_MONITORING is enabled, else held at 1.0 (neutral).
//
// Computed in one integer division of the full product, from milliamps, so
// nothing is truncated part-way. The fraction of a millionth that is left
// over is carried in m_ElecDamageRem and added to the next hot open, so
// frequent small hot opens (which individually cost well under a millionth
// - a 5 A open is ~0.3) still add up instead of each rounding to zero. The
// remainder is RAM-only: a reboot drops less than one millionth.
// Safe to call from ISR context: arithmetic only.
uint32_t RelayHealth::OneOpenDamageX1e6(int32_t currentMa)
{
  if (currentMa <= 0) return 0; // known zero current: no arc, no wear
  if (currentMa > RELAY_HEALTH_DAMAGE_MAX_MA) currentMa = RELAY_HEALTH_DAMAGE_MAX_MA;

  uint32_t alphaTX100 = 100;
#ifdef TEMPERATURE_MONITORING
  int16_t tempC10 = g_TempMonitor.m_MCP9808_temperature;
  if ((tempC10 != TEMPERATURE_NOT_INSTALLED) && (tempC10 > RELAY_HEALTH_TEMP_KNEE_C10)) {
    uint32_t overC10 = (uint32_t)(tempC10 - RELAY_HEALTH_TEMP_KNEE_C10);
    alphaTX100 = 100 + (overC10 * RELAY_HEALTH_TEMP_SLOPE_X100_PER_C10);
    if (alphaTX100 > RELAY_HEALTH_TEMP_ALPHA_MAX_X100) alphaTX100 = RELAY_HEALTH_TEMP_ALPHA_MAX_X100;
  }
#endif // TEMPERATURE_MONITORING

  //   d x1e6 = (mA/IrmA)^2 / N_e * (pf/100) * (aT/100) * 1e6
  //          = mA^2 * pf * aT * 100 / (IrmA^2 * N_e)
  // worst case numerator: (1e6)^2 * 120 * 300 * 100 = 3.6e18 < 2^64
  const uint64_t irMa = (uint64_t)RELAY_RATED_CURRENT_A * 1000ULL;
  const uint64_t den = irMa * irMa * (uint64_t)RELAY_RATED_ELECTRICAL_LIFE_CYCLES;
  uint64_t num = (uint64_t)currentMa * (uint64_t)currentMa
    * (uint64_t)RELAY_HEALTH_LOAD_ALPHA_X100 * (uint64_t)alphaTX100 * 100ULL;

  num += m_ElecDamageRem; // m_ElecDamageRem < den, so this cannot overflow either
  uint64_t dX1e6 = num / den;
  m_ElecDamageRem = num - dX1e6 * den;

  return (dX1e6 > 0xffffffffULL) ? 0xffffffffUL : (uint32_t)dX1e6;
}

// a learned-baseline sample count as stored in EEPROM, or 0 (restart
// learning) if it isn't tagged with the current baseline encoding
static uint8_t loadBaselineN(uint16_t eofs,uint8_t maxN)
{
  uint8_t raw = eeprom_read_byte((uint8_t*)eofs);
  if ((raw & ~RELAY_HEALTH_BASELINE_N_MASK) != RELAY_HEALTH_BASELINE_N_TAG) return 0;
  raw &= RELAY_HEALTH_BASELINE_N_MASK;
  return (raw > maxN) ? 0 : raw;
}

static void saveBaselineN(uint16_t eofs,uint8_t n)
{
  eeprom_write_byte((uint8_t*)eofs,(uint8_t)(RELAY_HEALTH_BASELINE_N_TAG | n));
}

void RelayHealth::Init()
{
  m_ElecDamageX1e6 = eeprom_read_dword((uint32_t*)EOFS_RELAY_ELEC_DAMAGE_X1E6);
  if (m_ElecDamageX1e6 == 0xffffffffUL) m_ElecDamageX1e6 = 0; // unformatted eeprom
  m_ElecDamageRem = 0;
  m_ElecDamageDirty = 0;

  m_ColdOpenCnt = eeprom_read_word((uint16_t*)EOFS_RELAY_COLD_OPEN_CNT);
  if (m_ColdOpenCnt == 0xffff) m_ColdOpenCnt = 0;

  m_TransitBaselineN = loadBaselineN(EOFS_RELAY_TRANSIT_BASELINE_N,RELAY_HEALTH_TRANSIT_BASELINE_SAMPLES);
  m_TransitBaselineX100 = m_TransitBaselineN ? eeprom_read_word((uint16_t*)EOFS_RELAY_TRANSIT_BASELINE_X100) : 0;
  m_LastOpenTransitMs = 0;
  m_TransitBaselineDirty = 0;

#ifdef TEMPERATURE_MONITORING
  m_ThermalBaselineN = loadBaselineN(EOFS_RELAY_THERMAL_BASELINE_N,RELAY_HEALTH_THERMAL_BASELINE_SAMPLES);
  m_ThermalBaselineX10000 = m_ThermalBaselineN ? eeprom_read_word((uint16_t*)EOFS_RELAY_THERMAL_BASELINE_X10000) : 0;
  m_SessionBaselineValid = 0;
  m_SessionBaselineTempC10 = TEMPERATURE_NOT_INSTALLED;
  m_LastThermalSampleMs = 0;
  m_LastThermalX10000 = 0xffff;
#endif // TEMPERATURE_MONITORING
}

void RelayHealth::OnRelayClose()
{
#ifdef TEMPERATURE_MONITORING
  // capture this session's ambient baseline before load heating accumulates,
  // and start its deltaT/I^2 sampling window
  int16_t t = g_TempMonitor.m_MCP9808_temperature;
  if (t != TEMPERATURE_NOT_INSTALLED) {
    m_SessionBaselineTempC10 = t;
    m_SessionBaselineValid = 1;
  }
  else {
    m_SessionBaselineValid = 0;
  }
  m_LastThermalSampleMs = millis();
#endif // TEMPERATURE_MONITORING
}

// May be called in ISR context (gfi_isr -> SetGfiTripped -> chargingOff(1)),
// so it must not touch EEPROM: the hot-open damage and transit-baseline
// updates are made in RAM and persisted later by FlushDeferredEepromWrites().
// (Cold opens are never emergency opens, so never ISR context, and keep
// their batched direct write.)
void RelayHealth::OnRelayOpen(uint8_t hotSwitch,int32_t lastOpenCurrentMa,uint16_t openTransitMs)
{
  if (hotSwitch) {
    uint32_t d = OneOpenDamageX1e6(lastOpenCurrentMa);
    if (d) {
      uint32_t newDamage = m_ElecDamageX1e6 + d;
      if (newDamage < m_ElecDamageX1e6) newDamage = 0xffffffffUL; // overflow guard, saturate
      m_ElecDamageX1e6 = newDamage;
      m_ElecDamageDirty = 1;
    }
  }
  else if (m_ColdOpenCnt < 0xfffe) {
    m_ColdOpenCnt++;
    // batched write - see RELAY_HEALTH_COLD_CNT_EEPROM_BATCH
    if ((m_ColdOpenCnt % RELAY_HEALTH_COLD_CNT_EEPROM_BATCH) == 0) {
      eeprom_write_word((uint16_t*)EOFS_RELAY_COLD_OPEN_CNT,m_ColdOpenCnt);
    }
  }

  if (openTransitMs < RELAY_TRANSIT_TIMEOUT_MS) { // a real measurement, not a timeout/unsupported sentinel
    m_LastOpenTransitMs = openTransitMs;
    if (m_TransitBaselineN < RELAY_HEALTH_TRANSIT_BASELINE_SAMPLES) {
      // running mean kept at 0.01 ms resolution, so a whole-ms sample that
      // differs from the mean by under 1 ms still moves it (at whole-ms
      // resolution 4,6,6,6,6,6,6,6 learned 5 instead of 5.75)
      m_TransitBaselineX100 = (uint16_t)(((uint32_t)m_TransitBaselineX100 * m_TransitBaselineN + (uint32_t)openTransitMs * 100UL) / (m_TransitBaselineN + 1));
      m_TransitBaselineN++;
      m_TransitBaselineDirty = 1;
    }
  }
}

void RelayHealth::FlushDeferredEepromWrites()
{
  if (m_ElecDamageDirty) {
    // 32-bit value an ISR can update: take a consistent copy
    noInterrupts();
    uint32_t damage = m_ElecDamageX1e6;
    m_ElecDamageDirty = 0;
    interrupts();
    eeprom_write_dword((uint32_t*)EOFS_RELAY_ELEC_DAMAGE_X1E6,damage);
  }
  if (m_TransitBaselineDirty) {
    m_TransitBaselineDirty = 0;
    eeprom_write_word((uint16_t*)EOFS_RELAY_TRANSIT_BASELINE_X100,m_TransitBaselineX100);
    saveBaselineN(EOFS_RELAY_TRANSIT_BASELINE_N,m_TransitBaselineN);
  }
}

void RelayHealth::Update()
{
#ifdef TEMPERATURE_MONITORING
  if (!g_EvseController.RelayIsClosed() || !m_SessionBaselineValid) return;

  unsigned long curms = millis();
  if ((curms - m_LastThermalSampleMs) < RELAY_HEALTH_THERMAL_WINDOW_MS) return;
  m_LastThermalSampleMs = curms;

  int32_t currentMa = g_EvseController.GetChargingCurrent();
  if (currentMa < (int32_t)RELAY_HEALTH_THERMAL_MIN_CURRENT_MA) return; // I^2 too small to be meaningful

  int16_t t = g_TempMonitor.m_MCP9808_temperature;
  if (t == TEMPERATURE_NOT_INSTALLED) return;

  int32_t deltaTC10 = (int32_t)t - (int32_t)m_SessionBaselineTempC10;
  if (deltaTC10 < 0) deltaTC10 = 0; // ambient can drift down too; only heating is meaningful here

  // H x10000 = deltaT(C) * 10000 / I(A)^2, from milliamps so the current
  // isn't truncated to whole amps either:
  //   = (deltaTC10/10) * 10000 * 1e6 / mA^2 = deltaTC10 * 1e9 / mA^2
  // At x100 (the earlier scale) a typical healthy contact - a few degrees
  // of rise at 32 A - computed to 0, which left the baseline at 0 and the
  // warning permanently disarmed. 0xffff is the "no sample" sentinel, so
  // saturate one below it (6 A floor => only past ~236C of rise).
  uint64_t hX10000 = ((uint64_t)deltaTC10 * 1000000000ULL) / ((uint64_t)currentMa * (uint64_t)currentMa);
  if (hX10000 > 0xfffe) hX10000 = 0xfffe;
  m_LastThermalX10000 = (uint16_t)hX10000;

  if (m_ThermalBaselineN < RELAY_HEALTH_THERMAL_BASELINE_SAMPLES) {
    m_ThermalBaselineX10000 = (uint16_t)(((uint32_t)m_ThermalBaselineX10000 * m_ThermalBaselineN + (uint32_t)hX10000) / (m_ThermalBaselineN + 1));
    m_ThermalBaselineN++;
    eeprom_write_word((uint16_t*)EOFS_RELAY_THERMAL_BASELINE_X10000,m_ThermalBaselineX10000);
    saveBaselineN(EOFS_RELAY_THERMAL_BASELINE_N,m_ThermalBaselineN);
  }
#endif // TEMPERATURE_MONITORING
}

uint8_t RelayHealth::GetLifeRemainingPct()
{
  uint64_t mechDamageX1e6 = ((uint64_t)m_ColdOpenCnt * 1000000ULL) / RELAY_RATED_MECHANICAL_LIFE_CYCLES;
  uint64_t totalX1e6 = (uint64_t)m_ElecDamageX1e6 + mechDamageX1e6;
  if (totalX1e6 >= 1000000ULL) return 0;
  return (uint8_t)(100UL - (uint32_t)(totalX1e6 / 10000ULL));
}

uint8_t RelayHealth::TransitDriftWarning()
{
  if (m_TransitBaselineN < RELAY_HEALTH_TRANSIT_BASELINE_SAMPLES) return 0; // baseline not established yet
  if ((m_TransitBaselineX100 == 0) || (m_LastOpenTransitMs == 0)) return 0; // no live measurement yet / guard divide-by-zero
  // live (ms) / baseline (ms x100), as a x100 ratio
  uint32_t ratioX100 = ((uint32_t)m_LastOpenTransitMs * 10000UL) / m_TransitBaselineX100;
  return (ratioX100 >= RELAY_HEALTH_TRANSIT_WARN_RATIO_X100) ? 1 : 0;
}

#ifdef TEMPERATURE_MONITORING
// x10000 -> the x100 scale $GL documents, rounded, keeping the sentinel
static uint16_t thermalX10000ToX100(uint16_t v)
{
  return (v == 0xffff) ? 0xffff : (uint16_t)((v + 50U) / 100U);
}

uint16_t RelayHealth::GetThermalIndexX100() { return thermalX10000ToX100(m_LastThermalX10000); }
uint16_t RelayHealth::GetThermalBaselineX100() { return m_ThermalBaselineN >= RELAY_HEALTH_THERMAL_BASELINE_SAMPLES ? thermalX10000ToX100(m_ThermalBaselineX10000) : 0xffff; }

uint8_t RelayHealth::ThermalWarningLevel()
{
  if (m_ThermalBaselineN < RELAY_HEALTH_THERMAL_BASELINE_SAMPLES) return 0; // baseline not established yet
  if ((m_ThermalBaselineX10000 == 0) || (m_LastThermalX10000 == 0xffff)) return 0;
  // both x10000, so the ratio (and its x100 thresholds) is scale-free
  uint32_t ratioX100 = ((uint32_t)m_LastThermalX10000 * 100UL) / m_ThermalBaselineX10000;
  if (ratioX100 >= RELAY_HEALTH_THERMAL_WARN_RATIO_X100) return 2;
  if (ratioX100 >= RELAY_HEALTH_THERMAL_WATCH_RATIO_X100) return 1;
  return 0;
}
#else // !TEMPERATURE_MONITORING
uint16_t RelayHealth::GetThermalIndexX100() { return 0xffff; }
uint16_t RelayHealth::GetThermalBaselineX100() { return 0xffff; }
uint8_t RelayHealth::ThermalWarningLevel() { return 0; }
#endif // TEMPERATURE_MONITORING

void RelayHealth::ResetLifeEstimate()
{
  noInterrupts();
  m_ElecDamageX1e6 = 0;
  m_ElecDamageRem = 0;
  m_ElecDamageDirty = 0;
  interrupts();
  eeprom_write_dword((uint32_t*)EOFS_RELAY_ELEC_DAMAGE_X1E6,0);
  m_ColdOpenCnt = 0;
  eeprom_write_word((uint16_t*)EOFS_RELAY_COLD_OPEN_CNT,0);

  m_TransitBaselineX100 = 0;
  m_TransitBaselineN = 0;
  m_LastOpenTransitMs = 0;
  m_TransitBaselineDirty = 0;
  eeprom_write_word((uint16_t*)EOFS_RELAY_TRANSIT_BASELINE_X100,0);
  saveBaselineN(EOFS_RELAY_TRANSIT_BASELINE_N,0);

#ifdef TEMPERATURE_MONITORING
  m_ThermalBaselineX10000 = 0;
  m_ThermalBaselineN = 0;
  m_LastThermalX10000 = 0xffff;
  eeprom_write_word((uint16_t*)EOFS_RELAY_THERMAL_BASELINE_X10000,0);
  saveBaselineN(EOFS_RELAY_THERMAL_BASELINE_N,0);
#endif // TEMPERATURE_MONITORING
}

#endif // RELAY_HEALTH
