#include "open_evse.h"

#ifdef RELAY_HEALTH

RelayHealth g_RelayHealth;

// Miner's-rule electrical-damage contribution of a single hot open, in
// millionths of rated electrical life (N_e) consumed:
//   d = (1/N_e) * (I/I_rated)^2 * alpha_pf * alpha_T
// alpha_T is derived live from the MCP9808 ambient reading when
// TEMPERATURE_MONITORING is enabled, else held at 1.0 (neutral).
uint32_t RelayHealth::OneOpenDamageX1e6(int32_t currentMa)
{
  if (currentMa < 0) currentMa = 0;
  uint32_t Ia = (uint32_t)currentMa / 1000UL; // amps, truncated - fine for a health estimate
  if (Ia == 0) return 0; // no meaningful arc energy without current

  uint32_t Ir = RELAY_RATED_CURRENT_A;
  // (I/Ir)^2 * 1e6, via a 64-bit intermediate: a fault-current hot open can
  // be a few hundred amps, which overflows 32 bits once squared and scaled
  uint64_t ratioSqX1e6 = ((uint64_t)Ia * Ia * 1000000ULL) / ((uint64_t)Ir * Ir);

  uint64_t dX1e6 = ratioSqX1e6 / RELAY_RATED_ELECTRICAL_LIFE_CYCLES;

  uint32_t alphaTX100 = 100;
#ifdef TEMPERATURE_MONITORING
  int16_t tempC10 = g_TempMonitor.m_MCP9808_temperature;
  if ((tempC10 != TEMPERATURE_NOT_INSTALLED) && (tempC10 > RELAY_HEALTH_TEMP_KNEE_C10)) {
    uint32_t overC10 = (uint32_t)(tempC10 - RELAY_HEALTH_TEMP_KNEE_C10);
    alphaTX100 = 100 + (overC10 * RELAY_HEALTH_TEMP_SLOPE_X100_PER_C10);
    if (alphaTX100 > RELAY_HEALTH_TEMP_ALPHA_MAX_X100) alphaTX100 = RELAY_HEALTH_TEMP_ALPHA_MAX_X100;
  }
#endif // TEMPERATURE_MONITORING

  dX1e6 = (dX1e6 * RELAY_HEALTH_LOAD_ALPHA_X100 * alphaTX100) / 10000ULL;

  return (dX1e6 > 0xffffffffULL) ? 0xffffffffUL : (uint32_t)dX1e6;
}

void RelayHealth::Init()
{
  m_ElecDamageX1e6 = eeprom_read_dword((uint32_t*)EOFS_RELAY_ELEC_DAMAGE_X1E6);
  if (m_ElecDamageX1e6 == 0xffffffffUL) m_ElecDamageX1e6 = 0; // unformatted eeprom

  m_ColdOpenCnt = eeprom_read_word((uint16_t*)EOFS_RELAY_COLD_OPEN_CNT);
  if (m_ColdOpenCnt == 0xffff) m_ColdOpenCnt = 0;

  m_TransitBaselineMs = eeprom_read_word((uint16_t*)EOFS_RELAY_TRANSIT_BASELINE_MS);
  if (m_TransitBaselineMs == 0xffff) m_TransitBaselineMs = 0;
  m_TransitBaselineN = eeprom_read_byte((uint8_t*)EOFS_RELAY_TRANSIT_BASELINE_N);
  if ((m_TransitBaselineN == 0xff) || (m_TransitBaselineN > RELAY_HEALTH_TRANSIT_BASELINE_SAMPLES)) m_TransitBaselineN = 0;
  m_LastOpenTransitMs = 0;

#ifdef TEMPERATURE_MONITORING
  m_ThermalBaselineX100 = eeprom_read_word((uint16_t*)EOFS_RELAY_THERMAL_BASELINE_X100);
  if (m_ThermalBaselineX100 == 0xffff) m_ThermalBaselineX100 = 0;
  m_ThermalBaselineN = eeprom_read_byte((uint8_t*)EOFS_RELAY_THERMAL_BASELINE_N);
  if ((m_ThermalBaselineN == 0xff) || (m_ThermalBaselineN > RELAY_HEALTH_THERMAL_BASELINE_SAMPLES)) m_ThermalBaselineN = 0;
  m_SessionBaselineValid = 0;
  m_SessionBaselineTempC10 = TEMPERATURE_NOT_INSTALLED;
  m_LastThermalSampleMs = 0;
  m_LastThermalX100 = 0xffff;
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

void RelayHealth::OnRelayOpen(uint8_t hotSwitch,int32_t lastOpenCurrentMa,uint16_t openTransitMs)
{
  if (hotSwitch) {
    uint32_t d = OneOpenDamageX1e6(lastOpenCurrentMa);
    uint32_t newDamage = m_ElecDamageX1e6 + d;
    if (newDamage < m_ElecDamageX1e6) newDamage = 0xffffffffUL; // overflow guard, saturate
    m_ElecDamageX1e6 = newDamage;
    eeprom_write_dword((uint32_t*)EOFS_RELAY_ELEC_DAMAGE_X1E6,m_ElecDamageX1e6);
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
      m_TransitBaselineMs = (uint16_t)(((uint32_t)m_TransitBaselineMs * m_TransitBaselineN + openTransitMs) / (m_TransitBaselineN + 1));
      m_TransitBaselineN++;
      eeprom_write_word((uint16_t*)EOFS_RELAY_TRANSIT_BASELINE_MS,m_TransitBaselineMs);
      eeprom_write_byte((uint8_t*)EOFS_RELAY_TRANSIT_BASELINE_N,m_TransitBaselineN);
    }
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

  uint32_t Ia = (uint32_t)currentMa / 1000UL;
  if (Ia == 0) return;

  // H x100 = deltaT(C) * 100 / I(A)^2; deltaTC10 is already deltaT in 0.1C
  // units, so this is (deltaTC10/10)*100/Ia^2 = deltaTC10*10/Ia^2
  uint32_t hX100 = (uint32_t)(((uint64_t)deltaTC10 * 10UL) / ((uint64_t)Ia * Ia));
  if (hX100 > 0xffff) hX100 = 0xffff;
  m_LastThermalX100 = (uint16_t)hX100;

  if (m_ThermalBaselineN < RELAY_HEALTH_THERMAL_BASELINE_SAMPLES) {
    m_ThermalBaselineX100 = (uint16_t)(((uint32_t)m_ThermalBaselineX100 * m_ThermalBaselineN + hX100) / (m_ThermalBaselineN + 1));
    m_ThermalBaselineN++;
    eeprom_write_word((uint16_t*)EOFS_RELAY_THERMAL_BASELINE_X100,m_ThermalBaselineX100);
    eeprom_write_byte((uint8_t*)EOFS_RELAY_THERMAL_BASELINE_N,m_ThermalBaselineN);
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
  if ((m_TransitBaselineMs == 0) || (m_LastOpenTransitMs == 0)) return 0; // no live measurement yet / guard divide-by-zero
  uint32_t ratioX100 = ((uint32_t)m_LastOpenTransitMs * 100UL) / m_TransitBaselineMs;
  return (ratioX100 >= RELAY_HEALTH_TRANSIT_WARN_RATIO_X100) ? 1 : 0;
}

#ifdef TEMPERATURE_MONITORING
uint16_t RelayHealth::GetThermalIndexX100() { return m_LastThermalX100; }
uint16_t RelayHealth::GetThermalBaselineX100() { return m_ThermalBaselineN >= RELAY_HEALTH_THERMAL_BASELINE_SAMPLES ? m_ThermalBaselineX100 : 0xffff; }

uint8_t RelayHealth::ThermalWarningLevel()
{
  if (m_ThermalBaselineN < RELAY_HEALTH_THERMAL_BASELINE_SAMPLES) return 0; // baseline not established yet
  if ((m_ThermalBaselineX100 == 0) || (m_LastThermalX100 == 0xffff)) return 0;
  uint32_t ratioX100 = ((uint32_t)m_LastThermalX100 * 100UL) / m_ThermalBaselineX100;
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
  m_ElecDamageX1e6 = 0;
  eeprom_write_dword((uint32_t*)EOFS_RELAY_ELEC_DAMAGE_X1E6,0);
  m_ColdOpenCnt = 0;
  eeprom_write_word((uint16_t*)EOFS_RELAY_COLD_OPEN_CNT,0);

  m_TransitBaselineMs = 0;
  m_TransitBaselineN = 0;
  m_LastOpenTransitMs = 0;
  eeprom_write_word((uint16_t*)EOFS_RELAY_TRANSIT_BASELINE_MS,0);
  eeprom_write_byte((uint8_t*)EOFS_RELAY_TRANSIT_BASELINE_N,0);

#ifdef TEMPERATURE_MONITORING
  m_ThermalBaselineX100 = 0;
  m_ThermalBaselineN = 0;
  m_LastThermalX100 = 0xffff;
  eeprom_write_word((uint16_t*)EOFS_RELAY_THERMAL_BASELINE_X100,0);
  eeprom_write_byte((uint8_t*)EOFS_RELAY_THERMAL_BASELINE_N,0);
#endif // TEMPERATURE_MONITORING
}

#endif // RELAY_HEALTH
