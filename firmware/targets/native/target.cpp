#include "open_evse.h"

#include <stdio.h>
#include <stdlib.h>

//                                                A/B   B/C   C/D  D  DS
#ifdef NATIVE_BOARD_NXT
THRESH_DATA J1772EVSEController::m_ThreshData = {3932, 3517, 3225, 0, 492};
#else
THRESH_DATA J1772EVSEController::m_ThreshData = { 875,  780,  690, 0, 260};
#endif

// ---------------------------------------------------------------------------
// Pin state block
//
// The whole of the firmware's observable hardware, in two arrays. Phase 4
// replaces the accessors below with reads and writes that cross the control
// channel; until then inputs hold whatever they were last set to and outputs
// are simply recorded.
// ---------------------------------------------------------------------------

namespace {

uint8_t  g_digitalIn[NATIVE_DIGITAL_PIN_COUNT];
uint8_t  g_digitalOut[NATIVE_DIGITAL_PIN_COUNT];
uint16_t g_adc[NATIVE_ADC_PIN_COUNT];

} // namespace

// Service the outside world. Called from WDT_RESET() and from every pin read,
// which between them cover every wait loop in the shared firmware -- see the
// pump-point audit in the native target notes. A no-op until phase 4 gives it
// a channel to service.
void nativeServiceIo()
{
}

void DigitalPin::init(uint32_t pinnum, int /*idxUnused*/, PinMode m)
{
  _pin = (uint8_t)pinnum;
  mode(m);
}

void DigitalPin::mode(PinMode m)
{
  _mode = m;
  if (_pin >= NATIVE_DIGITAL_PIN_COUNT) return;

  // An input with a pull-up reads high until something drives it low, which
  // is what the firmware expects of an unconnected ACLINE or lock pin.
  if (m == INP_PU) g_digitalIn[_pin] = HIGH;
}

uint8_t DigitalPin::read()
{
  nativeServiceIo();
  if (_pin >= NATIVE_DIGITAL_PIN_COUNT) return LOW;
  return (_mode == OUT) ? g_digitalOut[_pin] : g_digitalIn[_pin];
}

void DigitalPin::write(uint32_t state)
{
  if (_mode != OUT) return;
  if (_pin >= NATIVE_DIGITAL_PIN_COUNT) return;
  g_digitalOut[_pin] = state ? HIGH : LOW;
}

uint32_t AdcPin::read()
{
  nativeServiceIo();
  if (_pin >= NATIVE_ADC_PIN_COUNT) return 0;
  return g_adc[_pin];
}

#ifdef RELAY_ZC_SWITCH
void gmiAdcBegin() {}
uint16_t gmiAdcRead() { nativeServiceIo(); return ADC_HALF; }
void gmiAdcEnd() {}
#endif // RELAY_ZC_SWITCH

// ---------------------------------------------------------------------------
// Test access to the pin block
//
// Deliberately kept separate from the DigitalPin/AdcPin interface the firmware
// sees, so a driver cannot accidentally write an input through the same path
// the firmware reads it. This is the seam the control channel plugs into.
// ---------------------------------------------------------------------------

void nativeSetDigitalIn(uint8_t pin, uint8_t val)
{
  if (pin < NATIVE_DIGITAL_PIN_COUNT) g_digitalIn[pin] = val ? HIGH : LOW;
}

uint8_t nativeGetDigitalOut(uint8_t pin)
{
  return (pin < NATIVE_DIGITAL_PIN_COUNT) ? g_digitalOut[pin] : LOW;
}

void nativeSetAdc(uint8_t pin, uint16_t counts)
{
  if (pin < NATIVE_ADC_PIN_COUNT) {
    g_adc[pin] = (counts > ADC_MAX) ? ADC_MAX : counts;
  }
}

// ---------------------------------------------------------------------------
// MCU id
// ---------------------------------------------------------------------------

#ifdef MCU_ID_LEN
// mcuid *must* be of size MCU_ID_LEN
void getMcuId(uint8_t *mcuid)
{
  // Stable and obviously synthetic, so a host build is never mistaken for a
  // real unit by anything keying off $GI.
  for (uint8_t i = 0; i < MCU_ID_LEN; i++) {
    mcuid[i] = 0xE0 + i;
  }
}
#endif // MCU_ID_LEN

// ---------------------------------------------------------------------------
// EEPROM, backed by a file
// ---------------------------------------------------------------------------

namespace {

// Large enough for every EOFS_* offset the firmware uses.
constexpr size_t kEepromSize = 4096;

uint8_t g_eeprom[kEepromSize];
bool    g_eepromLoaded = false;
bool    g_eepromDirty  = false;

const char *eepromPath()
{
  const char *env = getenv("OPENEVSE_EEPROM");
  return (env && *env) ? env : "openevse_eeprom.bin";
}

void eepromLoad()
{
  if (g_eepromLoaded) return;
  g_eepromLoaded = true;

  // Unwritten EEPROM cells read as 0xFF on both real targets, and the
  // firmware relies on that to detect "never configured".
  memset(g_eeprom, 0xFF, sizeof(g_eeprom));

  FILE *f = fopen(eepromPath(), "rb");
  if (!f) return;
  if (fread(g_eeprom, 1, sizeof(g_eeprom), f) == 0 && ferror(f)) {
    fprintf(stderr, "native: failed to read %s\n", eepromPath());
  }
  fclose(f);
}

void eepromStore()
{
  if (!g_eepromDirty) return;

  FILE *f = fopen(eepromPath(), "wb");
  if (!f) {
    fprintf(stderr, "native: cannot write %s\n", eepromPath());
    return;
  }
  if (fwrite(g_eeprom, 1, sizeof(g_eeprom), f) != sizeof(g_eeprom)) {
    fprintf(stderr, "native: short write to %s\n", eepromPath());
  }
  fclose(f);
  g_eepromDirty = false;
}

bool eepromRange(size_t ofs, size_t len)
{
  return (ofs + len) <= kEepromSize;
}

} // namespace

uint8_t eeprom_read_byte(const uint8_t *ofs)
{
  eepromLoad();
  const size_t o = (size_t)(uintptr_t)ofs;
  if (!eepromRange(o, 1)) return 0xFF;
  return g_eeprom[o];
}

uint16_t eeprom_read_word(const uint16_t *ofs)
{
  eepromLoad();
  const size_t o = (size_t)(uintptr_t)ofs;
  if (!eepromRange(o, 2)) return 0xFFFF;
  uint16_t v;
  memcpy(&v, &g_eeprom[o], sizeof(v));
  return v;
}

uint32_t eeprom_read_dword(const uint32_t *ofs)
{
  eepromLoad();
  const size_t o = (size_t)(uintptr_t)ofs;
  if (!eepromRange(o, 4)) return 0xFFFFFFFF;
  uint32_t v;
  memcpy(&v, &g_eeprom[o], sizeof(v));
  return v;
}

void eeprom_write_byte(uint8_t *ofs, uint8_t val)
{
  eepromLoad();
  const size_t o = (size_t)(uintptr_t)ofs;
  if (!eepromRange(o, 1)) return;
  g_eeprom[o] = val;
  g_eepromDirty = true;
  eepromStore();
}

void eeprom_write_word(uint16_t *ofs, uint16_t val)
{
  eepromLoad();
  const size_t o = (size_t)(uintptr_t)ofs;
  if (!eepromRange(o, 2)) return;
  memcpy(&g_eeprom[o], &val, sizeof(val));
  g_eepromDirty = true;
  eepromStore();
}

void eeprom_write_dword(uint32_t *ofs, uint32_t val)
{
  eepromLoad();
  const size_t o = (size_t)(uintptr_t)ofs;
  if (!eepromRange(o, 4)) return;
  memcpy(&g_eeprom[o], &val, sizeof(val));
  g_eepromDirty = true;
  eepromStore();
}

// ---------------------------------------------------------------------------

void initTarget()
{
  eepromLoad();

  // CGMI (combined ground monitor / weld detect) is hardwired on NXT.
#ifdef NATIVE_BOARD_NXT
  g_hasCGMI = true;
#else
  g_hasCGMI = false;
#endif

  // Start from a plausible idle bench: no EV, no current, both AC test pins
  // reading mains present, GFI clear.
  nativeSetAdc(PILOT_SENSE_PIN, ADC_MAX);
  nativeSetAdc(CURRENT_PIN, ADC_HALF);
  nativeSetAdc(PP_PIN, ADC_MAX);
  nativeSetDigitalIn(GFI_REG, LOW);
  nativeSetDigitalIn(ACLINE1_REG, LOW);
  nativeSetDigitalIn(ACLINE2_REG, LOW);
}
