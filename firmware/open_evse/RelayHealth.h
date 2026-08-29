// -*- C++ -*-
#pragma once

#ifdef RELAY_HEALTH

// Relay contact-life estimation, built on the RELAY_ZC_SWITCH diagnostics
// already captured by J1772EVSEController (hot/cold open classification, the
// ammeter reading immediately before each open, and coil close/open transit
// timing via the load-side AC-sense pin), plus the MCP9808 ambient sensor
// when TEMPERATURE_MONITORING is enabled. Diagnostic only: never gates or
// alters charging logic. Surfaced via RAPI $GL, reset via $FH.
//
// Two independent signals:
//
// 1) Cumulative-damage (Miner's rule) cycle-life estimate. Every relay open
//    consumes a fraction of a life budget. Cold opens (current already at
//    zero) draw down the mechanical budget, which is normally so large
//    relative to a station's lifetime cycle count that its contribution
//    rounds to ~0. Hot opens (current still flowing: fault interrupts,
//    e-stops, or ZC switching disabled/skipped) draw down the much smaller
//    electrical budget, weighted by (I/I_rated)^2 plus load-character and
//    temperature multipliers - this is normally the entire signal. Reported
//    as a 0-100% life-remaining estimate.
//
// 2) Coil drop-out (open) transit-time drift: a self-learned baseline for
//    how long the relay takes to physically open after being commanded to,
//    and a live/baseline ratio. A lengthening drop-out time is an early
//    symptom of contact welding, ahead of the hard stuck-relay check
//    (EVSE_STATE_STUCK_RELAY) that ultimately catches it - that hard check
//    is the authoritative weld detector; this is just an earlier warning.
//
// If TEMPERATURE_MONITORING is also enabled, a third, independent signal is
// tracked: a thermal index H = deltaT / I^2, proportional to contact
// resistance (heating at the contact is I^2*R; normalizing by I^2 removes
// the load dependence, leaving a number that only rises as the contact
// erodes/oxidizes). Self-baselined per station and reported as a live/
// baseline ratio, since enclosure thermals vary station to station.
//
// None of this replaces direct contact-resistance measurement (closed-
// contact voltage drop at a known current) - that requires a differential
// voltage sensor across the contacts that this hardware does not have.

class RelayHealth {
  uint32_t m_ElecDamageX1e6; // Miner's-rule electrical-damage accumulator (millionths of rated electrical life consumed); EEPROM-backed
  uint16_t m_ColdOpenCnt;    // cumulative cold (non-arced) relay opens; EEPROM-backed, saturating

  uint16_t m_TransitBaselineMs; // learned open-transit-time baseline (ms); EEPROM-backed
  uint8_t  m_TransitBaselineN;  // # samples averaged into the baseline so far; EEPROM-backed, locks at RELAY_HEALTH_TRANSIT_BASELINE_SAMPLES
  uint16_t m_LastOpenTransitMs; // most recent measured open-transit time, for the live/baseline ratio; not persisted

#ifdef TEMPERATURE_MONITORING
  uint16_t m_ThermalBaselineX100;    // self-learned H0 (deltaT/I^2 x100); EEPROM-backed, locks after RELAY_HEALTH_THERMAL_BASELINE_SAMPLES
  uint8_t  m_ThermalBaselineN;       // # samples averaged into H0 so far; EEPROM-backed
  int16_t  m_SessionBaselineTempC10; // ambient temp captured at relay close, this session's deltaT reference
  uint8_t  m_SessionBaselineValid;
  unsigned long m_LastThermalSampleMs;
  uint16_t m_LastThermalX100; // most recent completed-window sample; 0xffff = none yet this boot
#endif

  static uint32_t OneOpenDamageX1e6(int32_t currentMa);

public:
  RelayHealth() {}

  void Init();
  void Update(); // call every main loop iteration

  // call from J1772EVSEController::chargingOn() once the relay is confirmed closed
  void OnRelayClose();
  // call from J1772EVSEController::chargingOff() when the relay was on, right
  // after m_RelayOpenTransitMs is measured
  void OnRelayOpen(uint8_t hotSwitch,int32_t lastOpenCurrentMa,uint16_t openTransitMs);

  uint8_t  GetLifeRemainingPct();
  uint32_t GetElecDamageX1e6() { return m_ElecDamageX1e6; }
  uint16_t GetColdOpenCnt() { return m_ColdOpenCnt; }

  uint16_t GetTransitBaselineMs() { return m_TransitBaselineN >= RELAY_HEALTH_TRANSIT_BASELINE_SAMPLES ? m_TransitBaselineMs : 0xffff; }
  uint8_t  TransitDriftWarning();

  // thermal signal: always declared so RAPI ($GL) has a stable response
  // shape regardless of build config. Without TEMPERATURE_MONITORING these
  // report the "not available" sentinel (0xffff / 0).
  uint16_t GetThermalIndexX100();    // 0xffff = not available/not yet sampled
  uint16_t GetThermalBaselineX100(); // 0xffff = not available/not yet established
  uint8_t  ThermalWarningLevel();    // 0=ok/not available 1=watch 2=warn

  void ResetLifeEstimate(); // $FH - clear all accumulators/baselines, e.g. after relay replacement
};

extern RelayHealth g_RelayHealth;

#endif // RELAY_HEALTH
