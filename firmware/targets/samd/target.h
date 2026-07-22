#pragma once


// enable $GI
#define MCU_ID_LEN 16 // SAMD ID is 128-bit

#define ADC_RESOLUTION_BITS 12
#define ADC_MAX 4095
#define ADC_HALF 2048

#define DEFAULT_CURRENT_SCALE_FACTOR    37
#define DEFAULT_AMMETER_CURRENT_OFFSET -135


// for J1772.ReadPilot()
// analogRead() 15 = ~13ms too slow!!!
// adcPin::read() 825 ~ 12-13ms
#define PILOT_LOOP_CNT 825


#define GetVerStr(s) strcpy(s,VERSION)

void wdt_disable();

#ifdef WATCHDOG
#include <Adafruit_SleepyDog.h>

inline void wdt_enable(int sec)
{
  Watchdog.enable(sec * 1000);
}

inline void wdt_disable()
{
  Watchdog.disable();
}

inline void wdt_reset()
{
  Watchdog.reset();
}

#define WDT_RESET() wdt_reset()
#define WDT_ENABLE() wdt_enable(2)
#define WDT_ENABLE_1S() wdt_enable(1)
#define WDT_DISABLE() wdt_disable()
#else
#define WDT_RESET()
#define WDT_ENABLE()
#define WDT_ENABLE_1S()
#define WDT_DISABLE()
#define wdt_reset()
#define wdt_enable(sec)
#endif // WATCHDOG

class DigitalPin {
  uint32_t _pinNum;
  uint32_t _pinMode;
  uint32_t _port;
  uint32_t _mask;
  
public:
  enum PinMode { INP,INP_PU,OUT };

  DigitalPin() {}
  DigitalPin(uint32_t pinnum,int idxjunk,PinMode mode) {
    init(pinnum,idxjunk,mode);
  }

  void init(uint32_t pinnum,int idxjunk,PinMode mode);

  uint8_t read() {
    return (PORT->Group[_port].IN.reg & _mask) != 0;
  }
  void write(uint32_t state) {
    if (_pinMode != OUTPUT) {
      return;
    }

    if (state)
        PORT->Group[_port].OUTSET.reg = _mask;
    else
        PORT->Group[_port].OUTCLR.reg = _mask;
  }
};

class AdcPin {
  static bool _adcInitDone;
  uint32_t _pinNum;

  void  _adcInit();
public:

  AdcPin() {}
  AdcPin(uint32_t pinnum) {
    init(pinnum);
  }

  void init(uint32_t pinnum) {
    _pinNum = pinnum;
  }

  // NB pin-number remap trap: the SAMD Arduino core's analogRead() does
  // `if (pin < A0) pin += A0;` (wiring_analog.c), so an AdcPin built with a
  // low digital pin number silently samples a *different*, A0-offset pin.
  // Safe here only because every AdcPin instance uses A0..A5 (>= A0):
  // PILOT_SENSE_PIN, CURRENT_PIN, PP_PIN.  The GMI zero-cross line on PA09
  // (pin 3) cannot use analogRead() at all — see gmiAdc*() below.
  uint32_t read();
};

#ifdef RELAY_ZC_SWITCH
// Read the AC zero-cross signal on the GMI line (PA09 / AIN[17]) via a direct
// SAMD21 ADC register access — analogRead() cannot reach it (see target.cpp).
// gmiAdcBegin()/gmiAdcEnd() bracket a sampling burst: they switch PA09 from its
// digital ACLINE2 ground-monitor input (INP_PU) to the analog mux and back,
// restoring the digital pull-up afterwards.  Do NOT call these per-sample.
void gmiAdcBegin();
uint16_t gmiAdcRead();
void gmiAdcEnd();
#endif // RELAY_ZC_SWITCH

void getMcuId(uint8_t *mcuid);

#include "SparkFun_External_EEPROM.h"
extern ExternalEEPROM g_eeprom;

static uint8_t eeprom_read_byte (const uint8_t *ofs) {
  uint8_t ret = 0xFF;
  g_eeprom.read((uint32_t) ofs,(uint8_t *)&ret,sizeof(ret));
  return ret;
}
static uint16_t eeprom_read_word (const uint16_t *ofs) {
  uint16_t ret = 0xFFFF;
  g_eeprom.read((uint32_t) ofs,(uint8_t *)&ret,sizeof(ret));
  return ret;
}
static uint32_t eeprom_read_dword (const uint32_t *ofs) {
  uint32_t ret = 0xFFFFFFFF;
  g_eeprom.read((uint32_t) ofs,(uint8_t *)&ret,sizeof(ret));
  return ret;
}
static void eeprom_write_byte (uint8_t *ofs, uint8_t val) {
  g_eeprom.write((uint32_t)ofs, (const uint8_t *)&val, sizeof(val));
}
static void eeprom_write_word (uint16_t *ofs, uint16_t val) {
  g_eeprom.write((uint32_t)ofs, (const uint8_t *)&val, sizeof(val));
}
static void eeprom_write_dword (uint32_t *ofs, uint32_t val) {
  g_eeprom.write((uint32_t)ofs, (const uint8_t *)&val, sizeof(val));
}



void initTarget();
