# Cable temperature monitoring (NTC thermistors on PP / PP2)

Monitors NTC thermistors embedded in the EV (output) cable and the input
(supply) cable via the two proximity-pilot-style analog inputs, and faults the
EVSE into `EVSE_STATE_OVER_TEMPERATURE` when a cable gets too hot.

Build flag: `CABLE_TEMPERATURE_MONITORING` (enabled in the common
`platformio.ini` flags). Runtime-disabled by default — nothing reads the ADC
and nothing can fault until `$FF C 1` is sent and at least one source is
assigned a pin.

Source: [`firmware/open_evse/CableTempMonitor.h`](../firmware/open_evse/CableTempMonitor.h),
[`.cpp`](../firmware/open_evse/CableTempMonitor.cpp).

## Hardware

Per channel, the divider is the same shape as the existing PP ampacity
circuit:

```
  VCC (5V, or 3.3V on SAMD)
   |
  [1k]                       <- CABLE_TEMP_PULLUP_OHMS
   |
   +---- connector pin (PP / PP2) ---- [NTC in cable] ---- GND
   |
  [10k]                      <- series protection only; the ADC input is
   |                            high-impedance, so it drops ~0 V
  PP_READ / PP2_READ
   |
  [diode to GND]             <- negative clamp
```

so the ADC sees `Vadc/Vcc = Rntc / (Rpullup + Rntc)`, and because the ADC
reference is the supply on both targets (AVR `AVCC`, SAMD `INTVCC1` with
`GAIN_DIV2`) the measurement is ratiometric — supply variation cancels.

| | ATmega328P | SAMD21 |
|---|---|---|
| `PP_PIN` (PP_READ) | ADC2 | PB09 (A2) |
| `PP2_PIN` (PP2_READ) | ADC3 | PA04 (A3) |
| ADC resolution | 10-bit | 12-bit |

`PP2_PIN` was added to both targets' `pindefs.h`; `PP_PIN` already existed.

## Sources

Four logical sources, each with independent thermistor parameters,
calibration offset and shutdown threshold:

| idx | name | intended use |
|---|---|---|
| 0 | EV1 | EV / output cable |
| 1 | EV2 | EV / output cable |
| 2 | IN1 | input / supply cable |
| 3 | IN2 | input / supply cable |

Each is assigned to an input pin — `0` unassigned, `1` PP_READ, `2` PP2_READ.
There are only two physical inputs, so at most two sources can be reading at
once; the naming is about which cable the thermistor is in, not about how many
ADCs exist. Nothing stops two sources sharing an input — they simply read the
same voltage through their own calibration.

## PP_READ is shared with the proximity pilot

`PP_READ` cannot serve both the proximity pilot and a temperature sensor. The
interlock is enforced at the point of change, in both directions:

- `$SN idx 1 …` (assigning a source to PP_READ) clears `ECF_PP_AUTO_AMPACITY`,
  exactly as `$FF P 0` would.
- `$FF P 1` (enabling PP auto-ampacity) unassigns every source sitting on
  PP_READ, exactly as `$SN idx 0` would.

Both are silent — read back with `$GE` (flags) and `$GN idx` to see the
result. `CableTempMonitor::Init()` re-checks at boot as a corruption guard
only; if a saved configuration somehow claims the pin for both, proximity
pilot keeps it.

`PP2_READ` has no such conflict.

## Temperature conversion

No floating point and no logarithm, on either target.

1. `Rntc = Rpullup * adc / (ADC_MAX - adc)`
2. normalize: `ratio = Rntc / R25`, scaled by 2000
3. look the ratio up in a 39-entry `PROGMEM` table of `R/R25` tabulated every
   5 °C from −40 °C to 150 °C, linearly interpolating within the step
4. correct for a thermistor beta differing from the table's, if needed
5. add the calibration offset

Storing the *normalized* curve rather than absolute ohms is what makes `R25`
configurable — any NTC of the same curve shape works by dividing out its own
`R25` before the lookup.

The beta correction (step 4) is exact rather than an approximation. For any
beta, `ln(R/R25) = B·(1/T − 1/T25)`. Reading the table (which assumes `B0`)
yields a `Tref` satisfying `ln(R/R25) = B0·(1/Tref − 1/T25)`, and equating the
two gives

```
1/T = 1/T25 + (B0/B) · (1/Tref − 1/T25)
```

which is plain integer arithmetic on a fixed-point reciprocal. It is skipped
entirely when `beta == B0`.

### Curve reference point

`B0 = 3443` is derived from the two Phoenix Contact NACS published points,
10000 Ω @ 25 °C and 1266 Ω @ 90 °C:

```
B = ln(10000/1266) / (1/298.15 − 1/363.15) = 3442.6
```

3443 reproduces 1265.7 Ω @ 90 °C. Both datasheet points come back out of the
integer pipeline exactly (25.0 °C and 90.0 °C).

### Accuracy

Modelled against exact floating-point ground truth through the whole chain
(true temperature → true resistance → truncating ADC → integer algorithm):

| | 328P (10-bit) | SAMD (12-bit) |
|---|---|---|
| worst \|error\|, 20–120 °C, stock 10k B3443 | 0.5 °C | 0.4 °C |
| worst \|error\|, 20–120 °C, across 10k B3950 / 10k B3380 / 5k B3450 | 0.6 °C | 0.4 °C |
| error at 70/90/110 °C | +0.0 … +0.3 °C | +0.0 … +0.3 °C |

Error in the band that matters is small and biased hot, which is the
fail-safe direction. With the default 90.0 °C threshold the fault fires at a
true 89.8 °C (10-bit) / 89.9 °C (12-bit).

Accuracy degrades below about 0 °C — a 1 kΩ pull-up against a 10 kΩ NTC puts
the divider within a few ADC counts of the rail down there (about 1.5 °C
worst case at −20 °C on the 328P). Irrelevant for over-temperature
protection, but don't treat a cold reading as precise.

## Sentinel readings

All in 10ths of a degree C, all below any real reading, so a `>= panic`
comparison can never trip on one:

| value | meaning |
|---|---|
| −2560 | source unassigned, or the feature is disabled |
| −2561 | open circuit — no cable plugged in, or a disconnected/broken thermistor. Also reported below about −40 °C, where a very cold cable and an open one are not distinguishable. |
| −2562 | shorted thermistor or wiring |

**Neither open nor short faults the EVSE.** An unplugged connector reads open
during entirely normal operation, so open cannot be treated as an error. A
hard short is reported for diagnosis rather than shutdown — see the caveat
below.

## Over-temperature behaviour

`CableTempMonitor::Read()` re-reads every 1000 ms (8 averaged ADC samples per
source) and sets an internal over-temperature flag if any assigned source is
at or above its own `panic` threshold. `J1772EVSEController::Update()` then
drops into `EVSE_STATE_OVER_TEMPERATURE`, the same fault state the enclosure
sensors use.

Gated on the cable feature's own enable (`$FF C`), not on `$FF T` — these are
separate sensors with separate thresholds, and one shouldn't silently switch
off the other.

Like the existing ambient panic check, there is no hysteresis: the fault
clears when the reading drops back below the threshold.

### Two deliberate design decisions worth knowing

**A reading above the top of the table clamps to 150.0 °C.** That value means
"at or above the top of the measurable range", not a measurement — for a
thermistor whose beta differs from the curve's, the true temperature at the
clamp point can be meaningfully lower (about 128 °C for a 10k B3950). Clamping
high is the fail-safe direction: a beta-corrected, and therefore lower, clamp
value could sit under a high `panic` threshold and never trip it.

**The short-detection band is deliberately very narrow** — `R/R25 < 0.001`,
under 10 Ω on a 10k NTC. Everything between there and the bottom of the curve
table clamps to 150.0 °C and therefore trips. A wider short band would carve a
hole in the protection: a thermistor hot enough to fall into it would report
"shorted", which does not fault, and would silently clear an over-temperature
fault raised on the way up. Verified over the full ADC range that there is no
reading between 90 °C and 300 °C that fails to trip.

**Open/short do not fault.** A failed or disconnected sensor on an assigned
source means the cable is unprotected, and the firmware currently reports that
rather than acting on it — visible through `$GN`, so the ESP32 layer can
decide. Worth revisiting if you want sensor-integrity enforcement in the
controller itself.

## RAPI

Full reference is inline in
[`rapi_proc.h`](../firmware/open_evse/rapi_proc.h). Summary:

### `$FF C 0|1` — enable/disable

Saved to EEPROM. Disabling also clears all readings back to −2560 and drops
any over-temperature condition.

### `$SN` — configure a source

```
$SN idx pin                        reassign the input pin only
$SN idx pin r25 beta offset panic  full configuration
```

| arg | range | default | meaning |
|---|---|---|---|
| `idx` | 0–3 | — | 0=EV1 1=EV2 2=IN1 3=IN2 |
| `pin` | 0–2 | 0 | 0=unassigned 1=PP_READ 2=PP2_READ |
| `r25` | 100–65535 | 10000 | NTC nominal resistance at 25 °C, Ω |
| `beta` | 1000–6000 | 3443 | NTC beta coefficient |
| `offset` | −2000…2000 | 0 | calibration offset, 10ths °C |
| `panic` | 300–1500 | 900 | shutdown threshold, 10ths °C |

Saved to EEPROM. `$NK` on any out-of-range argument — values are validated as
`int32_t` before narrowing, so an out-of-range argument is rejected rather
than wrapping into range.

```
$SN 0 2 10000 3443 0 900   EV1 on PP2_READ, stock Phoenix Contact NACS cable
$SN 0 2                    move EV1 to PP2_READ, keep its calibration
$SN 0 0                    unassign EV1
```

### `$GN` — read

```
$GN       -> $OK ev1 ev2 in1 in2      temperatures, 10ths °C or a sentinel
$GN idx   -> $OK pin r25 beta offset panic
```

## EEPROM

37 bytes at offsets 58–94; next free offset is 95.

| offset | size | contents |
|---|---|---|
| 58 | 1 | `CTMF_ENABLED`; `0xff` = unformatted → feature off |
| 59 + 9·idx | 9 | per source: `r25`(2) `beta`(2) `offset`(2) `panic`(2) `pin`(1) |

Serialized field by field rather than block-copied, so struct alignment
padding cannot change the on-EEPROM layout between targets. Anything
unformatted or out of range falls back to the Phoenix Contact NACS defaults.

## Cost

| build | flash before | flash after | RAM before | RAM after |
|---|---|---|---|---|
| `m328p_core` | 23008 (70.2%) | 25254 (77.1%) | 940 | 1015 |
| `m328p_LCD_WIFI` | 27182 (83.0%) | 29356 (89.6%) | 1046 | 1121 |
| `samd` | — | 48884 (18.6%) | — | 4764 |

`m328p_LCD_WIFI` is the tight one at 89.6% — about 3.4 kB of flash left. If
that becomes a problem, drop `-D CABLE_TEMPERATURE_MONITORING` from that env
specifically.

Two shared buffers grew to fit the new commands, on AVR only:

- `TMP_BUF_SIZE` 34 → 36: `$GN`'s worst-case response (four 5-character
  sentinels) is 2 longer than `$GS`, previously the longest AVR reply.
- `ESRAPI_BUFLEN` 32 → 40: the 6-argument `$SN` with a sequence id and
  checksum is 36 characters + NUL. Commands longer than the buffer are
  silently dropped, so this had to grow.

Both are conditional on `CABLE_TEMPERATURE_MONITORING`; builds without the
feature keep their previous footprint exactly.

## Not implemented

- **No throttling.** Over-temperature is a hard shutdown at the threshold.
  `TEMPERATURE_THROTTLING`'s half/quarter-current staging applies to the
  enclosure sensors only. Adding a cable-temperature throttle knee would be a
  natural follow-up.
- **No LCD display of cable temperatures.** The fault shows through the
  existing `EVSE_STATE_OVER_TEMPERATURE` screen; the per-source readings are
  RAPI-only.
- **No `$AT`/async notification** specific to cable temperature — the state
  change to `EVSE_STATE_OVER_TEMPERATURE` is reported through the normal
  `$AT` path.
