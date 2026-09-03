#include "open_evse.h"
#include "channel.h"

#include <stdio.h>
#include <stdlib.h>

//                                                A/B   B/C   C/D  D  DS
#ifdef NATIVE_BOARD_NXT
THRESH_DATA J1772EVSEController::m_ThreshData = {3932, 3517, 3225, 0, 492};
#else
THRESH_DATA J1772EVSEController::m_ThreshData = { 875,  780,  690, 0, 260};
#endif

// ---------------------------------------------------------------------------
// Pin state
//
// The whole of the firmware's observable hardware. Reads and writes here are
// what the control channel publishes and drives; see channel.cpp.
// ---------------------------------------------------------------------------

namespace {

// Only ADC values are held here. Digital pin state lives in EpoxyDuino's own
// store, reached through digitalRead/digitalReadValue/digitalWrite, so that a
// driver raising an input also feeds checkInterrupts() -- which is what makes
// a GFI edge dispatch gfi_isr() the way the hardware interrupt would. Keeping
// a second private array here would have left the interrupt path dead.
//
// Each ADC channel carries two levels rather than one, because the firmware
// samples two of its inputs as waveforms and not as values. ReadPilot() takes
// the min and max over PILOT_LOOP_CNT samples and rejects the reading unless
// the negative excursion is there (plow < m_ThreshDS, else DIODE_CHK_FAILED);
// readAmmeter() derives RMS from peak-to-peak the same way. A single level
// cannot express either, so a channel that only set one could never get the
// firmware past its own diode check.
//
// Alternating successive reads is the whole of the emulation: which levels to
// present, and what they mean, stays with the driver.
struct AdcChannel {
  uint16_t high;
  uint16_t low;
  bool     phase;
};

AdcChannel g_adc[NATIVE_ADC_PIN_COUNT];

} // namespace

// Service the outside world. Called from WDT_RESET() and from every pin read,
// which between them cover every wait loop in the shared firmware.
void nativeServiceIo()
{
  channelService();
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
  if (m == INP_PU) digitalReadValue(_pin, HIGH);
}

uint8_t DigitalPin::read()
{
  nativeServiceIo();
  if (_pin >= NATIVE_DIGITAL_PIN_COUNT) return LOW;
  return (_mode == OUT) ? digitalWriteValue(_pin) : (uint8_t)digitalRead(_pin);
}

void DigitalPin::write(uint32_t state)
{
  if (_mode != OUT) return;
  if (_pin >= NATIVE_DIGITAL_PIN_COUNT) return;
  digitalWrite(_pin, state ? HIGH : LOW);
}

uint32_t AdcPin::read()
{
  nativeServiceIo();
  if (_pin >= NATIVE_ADC_PIN_COUNT) return 0;

  AdcChannel &ch = g_adc[_pin];
  if (ch.high == ch.low) return ch.high;

  ch.phase = !ch.phase;
  return ch.phase ? ch.high : ch.low;
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
  if (pin < NATIVE_DIGITAL_PIN_COUNT) digitalReadValue(pin, val ? HIGH : LOW);
}

uint8_t nativeGetDigitalOut(uint8_t pin)
{
  return (pin < NATIVE_DIGITAL_PIN_COUNT) ? digitalWriteValue(pin) : LOW;
}

void nativeSetAdc(uint8_t pin, uint16_t high, uint16_t low)
{
  if (pin >= NATIVE_ADC_PIN_COUNT) return;
  if (high > ADC_MAX) high = ADC_MAX;
  if (low  > ADC_MAX) low  = ADC_MAX;
  g_adc[pin].high = high;
  g_adc[pin].low  = low;
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
  channelBegin();

  // CGMI (combined ground monitor / weld detect) is hardwired on NXT.
#ifdef NATIVE_BOARD_NXT
  g_hasCGMI = true;
#else
  g_hasCGMI = false;
#endif

  // Start from a plausible idle bench: no EV, no current, both AC test pins
  // reading mains present, GFI clear.
  // Idle bench: pilot at a steady +12V (state A, no PWM), no current, PP open.
  nativeSetAdc(PILOT_SENSE_PIN, ADC_MAX, ADC_MAX);
  nativeSetAdc(CURRENT_PIN, ADC_HALF, ADC_HALF);
  nativeSetAdc(PP_PIN, ADC_MAX, ADC_MAX);
  // Digital inputs are deliberately not preset here. J1772EVSEController::Init
  // runs after this and calls DigitalPin::init(..., INP_PU) on the AC-sense and
  // GFI lines, which sets them to the pull-up state -- anything written here
  // would simply be overwritten. Until a driver drives them they read as an
  // unconnected input does on hardware: high, which the firmware reads as "no
  // voltage at the pin" (the AC-sense lines are active low).
}
