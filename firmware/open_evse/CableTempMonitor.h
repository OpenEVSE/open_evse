// -*- C++ -*-
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
#pragma once

#ifdef CABLE_TEMPERATURE_MONITORING

//
// Monitors NTC thermistors embedded in the EV (output) cable and the input
// (supply) cable, using the two proximity-pilot-style analog inputs.
//
// Hardware (per channel):
//
//   VCC (5V, or 3.3V on SAMD)
//    |
//   [1k]                          <- CABLE_TEMP_PULLUP_OHMS
//    |
//    +---- connector pin (PP / PP2) ---- [NTC in cable] ---- GND
//    |
//   [10k]                         <- series protection only; the ADC input is
//    |                               high-impedance so it drops ~0V
//   PP_READ / PP2_READ  (ADC2/ADC3 on 328P, PB09/PA04 on SAMD)
//    |
//   [diode to GND]                <- negative clamp
//
// so the ADC sees a plain divider:  Vadc/Vcc = Rntc / (Rpullup + Rntc)
//
// Four logical sources are provided - EV1, EV2, IN1, IN2 - each with its own
// thermistor parameters, calibration offset and shutdown threshold. Because
// there are only two physical inputs, at most two sources can be assigned at
// any one time; the rest sit unassigned (CABLE_TEMP_PIN_NONE).
//
// IMPORTANT - PP_READ is shared with the proximity pilot. Assigning any
// source to CABLE_TEMP_PIN_PP automatically disables PP auto-ampacity, and
// enabling PP auto-ampacity automatically unassigns any source on PP_READ.
// The two functions cannot both own that pin. See RAPI $SN / $FF P.
//
// Configured and read over RAPI:
//   $FF C 0|1                          enable/disable the whole feature
//   $SN idx pin                        (re)assign a source's input pin
//   $SN idx pin r25 beta offset panic  full per-source configuration
//   $GN                                read all four temperatures
//   $GN idx                            read one source's configuration
//

// source indices
#define CABLE_TEMP_EV1 0
#define CABLE_TEMP_EV2 1
#define CABLE_TEMP_IN1 2
#define CABLE_TEMP_IN2 3
#define CABLE_TEMP_SENSOR_CNT 4

// per-source pin assignment
#define CABLE_TEMP_PIN_NONE 0
#define CABLE_TEMP_PIN_PP   1 // PP_READ  - ADC2 (328P) / PB09 (SAMD) - shared w/ proximity pilot
#define CABLE_TEMP_PIN_PP2  2 // PP2_READ - ADC3 (328P) / PA04 (SAMD)
#define CABLE_TEMP_PIN_MAX  2

// top-of-divider pullup, in ohms. fixed by the hardware; override at build
// time if your board is stuffed differently
#ifndef CABLE_TEMP_PULLUP_OHMS
#define CABLE_TEMP_PULLUP_OHMS 1000UL
#endif

// Defaults: Phoenix Contact NACS cable, 10k NTC.
// beta is derived from the two published points, 10000 ohms @ 25C and
// 1266 ohms @ 90C:
//   B = ln(R25/R90) / (1/T25 - 1/T90) = ln(10000/1266) / (1/298.15 - 1/363.15)
//     = 3442.6  -> 3443, which reproduces 1265.7 ohms @ 90C
#define CABLE_TEMP_DEFAULT_R25   10000
#define CABLE_TEMP_DEFAULT_BETA   3443
#define CABLE_TEMP_DEFAULT_OFFSET    0
// 90.0C - the cable manufacturer's recommended shut-off temperature
#define CABLE_TEMP_DEFAULT_PANIC   900

// the reference beta the built-in R/R25 curve was generated at. a source
// configured with a different beta is corrected arithmetically - see
// CableTempMonitor::RatioToTempC10()
#define CABLE_TEMP_CURVE_BETA 3443

// sentinel readings (all in 10ths of a degree C, like the rest of the
// temperature code, and all below any real reading so a naive ">= panic"
// comparison can never trip on one)
#define CABLE_TEMP_NOT_INSTALLED -2560 // source unassigned, or feature disabled
#define CABLE_TEMP_OPEN          -2561 // divider at the rail: no cable/thermistor connected, or open circuit
#define CABLE_TEMP_SHORTED       -2562 // divider at ground: thermistor or wiring shorted

// a reading is a real temperature (not a sentinel) if it is above this
#define CABLE_TEMP_IS_VALID(t) ((t) > CABLE_TEMP_NOT_INSTALLED)

// how often to re-read the thermistors
#define CABLE_TEMPMONITOR_UPDATE_INTERVAL 1000ul

// oversampling per update - the divider is a slow, high-impedance source, so
// a handful of averaged samples costs nothing and settles the LSBs
#define CABLE_TEMP_ADC_SAMPLES 8

// clamps for user-supplied configuration
#define CABLE_TEMP_MIN_R25    100
#define CABLE_TEMP_MAX_R25  65535UL
#define CABLE_TEMP_MIN_BETA  1000
#define CABLE_TEMP_MAX_BETA  6000
#define CABLE_TEMP_MIN_PANIC  300 //  30.0C
#define CABLE_TEMP_MAX_PANIC 1500 // 150.0C - top of the curve table
#define CABLE_TEMP_MIN_OFFSET -2000
#define CABLE_TEMP_MAX_OFFSET  2000

// CableTempMonitor::m_Flags
#define CTMF_ENABLED         0x01
#define CTMF_OVERTEMPERATURE 0x02

typedef struct cable_temp_cfg {
  uint16_t r25;    // NTC nominal resistance at 25C, ohms
  uint16_t beta;   // NTC beta coefficient (B25/85-ish; see CABLE_TEMP_CURVE_BETA)
  int16_t  offset; // calibration offset added to the computed reading, 10ths of a degree C
  int16_t  panic;  // shutdown threshold, 10ths of a degree C
  uint8_t  pin;    // CABLE_TEMP_PIN_xxx
} CABLE_TEMP_CFG;

// EEPROM image is written/read as this many bytes per source. the struct is
// serialized field by field rather than block-copied so alignment padding
// can't change the on-EEPROM layout between targets.
#define CABLE_TEMP_CFG_EEPROM_SIZE 9

class CableTempMonitor {
  uint8_t m_Flags;
  unsigned long m_LastUpdate;
  CABLE_TEMP_CFG m_Cfg[CABLE_TEMP_SENSOR_CNT];
  int16_t m_TempC10[CABLE_TEMP_SENSOR_CNT];
  AdcPin m_AdcPP;
  AdcPin m_AdcPP2;

  void LoadCfg(uint8_t idx);
  void SaveCfg(uint8_t idx);
  uint16_t ReadAdcAvg(uint8_t pin);
  static int16_t RatioToTempC10(uint32_t ratioX2000,uint16_t beta);
  int16_t ReadSensor(uint8_t idx);

public:
  CableTempMonitor() {}

  void Init();
  void Read(); // call every main loop iteration; self-rate-limits

  int8_t IsEnabled() { return (m_Flags & CTMF_ENABLED) ? 1 : 0; }
  void Enable(int8_t tf);

  // most recent reading for a source, 10ths of a degree C, or one of the
  // CABLE_TEMP_xxx sentinels
  int16_t GetTempC10(uint8_t idx) { return (idx < CABLE_TEMP_SENSOR_CNT) ? m_TempC10[idx] : CABLE_TEMP_NOT_INSTALLED; }
  const CABLE_TEMP_CFG *GetCfg(uint8_t idx) { return (idx < CABLE_TEMP_SENSOR_CNT) ? &m_Cfg[idx] : 0; }

  // Returns 0 on success, non-zero if any argument is out of range.
  // Arguments are taken as int32_t so an out-of-range RAPI value is rejected
  // rather than silently wrapping into range when narrowed.
  // Assigning a source to CABLE_TEMP_PIN_PP disables PP auto-ampacity.
  int8_t SetPin(int32_t idx,int32_t pin);
  int8_t SetCfg(int32_t idx,int32_t pin,int32_t r25,int32_t beta,int32_t offset,int32_t panic);

  // 1 if any assigned source is at or above its configured shutdown
  // threshold. only ever true when the feature is enabled.
  int8_t OverTemperature() { return (m_Flags & CTMF_OVERTEMPERATURE) ? 1 : 0; }

  // 1 if any source is currently assigned to PP_READ (i.e. proximity pilot
  // cannot use that pin)
  int8_t UsesPPPin();
  // unassign every source sitting on PP_READ - called when PP auto-ampacity
  // takes the pin back
  void ReleasePPPin();
};

extern CableTempMonitor g_CableTempMonitor;

#endif // CABLE_TEMPERATURE_MONITORING
