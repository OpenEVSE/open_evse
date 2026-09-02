// -*- C++ -*-
//
// Native (host) target for the OpenEVSE safety firmware.
//
// This target models nothing. DigitalPin and AdcPin read and write a small
// block of pin state, and that block is all a driver -- the emulator, or a
// test harness -- needs to see in order to present the firmware with a world.
// The vehicle, the pilot response and the electrical model live on the far
// side of that boundary, not here.
//
// Two board modes are supported, selected by NATIVE_BOARD_NXT:
//   unset  - OpenEVSE v3 / OEV6, 10-bit ADC, 10-byte MCU id
//   set    - OpenEVSE NXT (SAMD), 12-bit ADC, 16-byte MCU id
// Everything that differs between them is a constant in this file plus the
// threshold table in target.cpp.

#pragma once

#include <stdint.h>
#include <string.h>
#include "Arduino.h"

#ifdef NATIVE_BOARD_NXT

#define MCU_ID_LEN 16
#define ADC_RESOLUTION_BITS 12
#define ADC_MAX  4095
#define ADC_HALF 2048
#define DEFAULT_CURRENT_SCALE_FACTOR    37
#define DEFAULT_AMMETER_CURRENT_OFFSET -135
#define PILOT_LOOP_CNT 825

#else // OEV6

#define MCU_ID_LEN 10
#define ADC_RESOLUTION_BITS 10
#define ADC_MAX  1023
#define ADC_HALF  512
#define DEFAULT_CURRENT_SCALE_FACTOR   220
#define DEFAULT_AMMETER_CURRENT_OFFSET   0
#define PILOT_LOOP_CNT 100

#endif // NATIVE_BOARD_NXT

#define GetVerStr(s) strcpy(s,VERSION)

//
// Watchdog
//
// There is nothing to kick on the host, but WDT_RESET() is the firmware's
// periodic housekeeping call -- it appears inside every bare millis() wait
// loop in the shared code. That makes it the natural place to service the
// control channel, so it is a real function rather than an empty macro.
// See nativeServiceIo() in target.cpp.
//
void nativeServiceIo();

#define WDT_RESET()    nativeServiceIo()
#define WDT_ENABLE()
#define WDT_ENABLE_1S()
#define WDT_DISABLE()
#define wdt_reset()    nativeServiceIo()
#define wdt_enable(sec)

inline void wdt_disable() {}

//
// DigitalPin
//
// The first constructor argument is an index into the pin block (see
// pindefs.h); the second is the bit index on a real port and is unused here,
// kept only so the shared code's DIGITAL_PIN call sites compile unchanged.
//
class DigitalPin {
  uint8_t _pin;
  uint8_t _mode;

public:
  enum PinMode { INP, INP_PU, OUT };

  DigitalPin() : _pin(0xFF), _mode(INP) {}
  DigitalPin(uint32_t pinnum, int idxUnused, PinMode mode) {
    init(pinnum, idxUnused, mode);
  }

  void init(uint32_t pinnum, int idxUnused, PinMode mode);
  void mode(PinMode mode);

  uint8_t read();
  void write(uint32_t state);
  void writeAtomic(uint32_t state) { write(state); }
};

//
// AdcPin
//
class AdcPin {
  uint8_t _pin;

public:
  AdcPin() : _pin(0xFF) {}
  AdcPin(uint32_t pinnum) { init(pinnum); }

  void init(uint32_t pinnum) { _pin = (uint8_t)pinnum; }
  uint32_t read();

  static void referenceMode(uint8_t) {}
};

#ifdef RELAY_ZC_SWITCH
// AC zero-cross sampling. On SAMD these bracket a direct ADC burst on the
// GMI line; here they simply mark the burst so a driver can follow along.
void gmiAdcBegin();
uint16_t gmiAdcRead();
void gmiAdcEnd();
#endif // RELAY_ZC_SWITCH

void getMcuId(uint8_t *mcuid);

//
// EEPROM, backed by a file so settings survive a restart the way they do on
// hardware. Path is taken from $OPENEVSE_EEPROM, defaulting to
// ./openevse_eeprom.bin.
//
uint8_t  eeprom_read_byte(const uint8_t *ofs);
uint16_t eeprom_read_word(const uint16_t *ofs);
uint32_t eeprom_read_dword(const uint32_t *ofs);
void eeprom_write_byte(uint8_t *ofs, uint8_t val);
void eeprom_write_word(uint16_t *ofs, uint16_t val);
void eeprom_write_dword(uint32_t *ofs, uint32_t val);

void initTarget();
