# PR #60 (Release 9.4.3) — response to CodeRabbit review

Review: CodeRabbit run `fc6e345c`, 18 findings (16 actionable, 2 minor),
covering `843596c..40f61c3`.

We checked every finding against the code. 17 were valid and are fixed as
suggested or close to it. 1 was valid, but the suggested fix would have broken
the shipped build, so we fixed it differently (see *PC3 conflict* below). No
finding was rejected.

Build check after the fixes: `m328p_core_lcd`, `samd` and `samd_ice_dev` all
build. ATmega328P usage is now RAM 1166 / 2048 (56.9%) and flash 30060 / 32768
(91.7%), up 43 B and 572 B. No hardware testing yet.

## Safety

| Finding | Verdict | Fix |
|---|---|---|
| `J1772EvseController.cpp:616` — `waitRelayTransit()` runs inside `gfi_isr()` | **Valid, severe.** The loop waits on `millis()`, which doesn't advance in ISR context, and it calls `WDT_RESET()`. If the relay never reports open (for example, welded), the controller hangs forever and the watchdog never fires. | `chargingOff(1)` (emergency) no longer waits for transit confirmation. It records `RELAY_TRANSIT_TIMEOUT_MS` ("not measured"), and RelayHealth ignores that value. Non-emergency opens still measure transit. |
| `J1772EvseController.cpp:2216` — EV can connect during the blocking stuck-relay recovery | **Valid.** The caller checks `EvConnected()` once, but recovery can run for tens of seconds. | `AttemptStuckRelayRecovery()` calls `ReadPilot()` before every `chargingOn()`. If an EV is connected, it returns "not recovered" with the relay open. |
| `samd/Gfi.cpp:175-178` — GFI input not re-read after the settle delay | **Valid.** If the output goes high during the settle, `testInProgress` absorbs the edge. After the test it stays high with no new rising edge, so a real fault would never trip. | After the settle, if the pin still reads high, `SelfTest()` clears `testInProgress` and returns 3 (failed). |

## RAPI

| Finding | Verdict | Fix |
|---|---|---|
| `rapi_proc.cpp:999` — `$GL` can overflow | **Valid, worse than reported.** The worst-case text is 48 chars and `ESRAPI_BUFLEN` was 40. On AVR, `g_sTmp` was 36 bytes, so even a typical `$OK …` reply (~40 bytes) overflowed it. | Added `RAPI_GL_MAX_LEN` (48). With `RELAY_HEALTH`, `ESRAPI_BUFLEN = 49` and `TMP_BUF_SIZE = 61`, with `#error` guards. The reply is written with `snprintf(buffer, ESRAPI_BUFLEN, …)`. |
| `rapi_proc.cpp:626` — `$SN`/`$GN` arguments outside int32 wrap into range | **Valid.** | New `dtoi32Checked()` rejects empty strings, non-digits and values outside int32. `$SN` and `$GN idx` reply `$NK` when it fails. Negative offsets still work. |

## Relay health (`RelayHealth.cpp`)

| Finding | Verdict | Fix |
|---|---|---|
| `:90` — EEPROM writes in `OnRelayOpen()` (reachable from `gfi_isr`) | **Valid.** | Hot-open damage and transit-baseline updates now change RAM only and set dirty flags. `RelayHealth::FlushDeferredEepromWrites()` saves them from the main loop through the controller's existing `FlushDeferredEepromWrites()`. It reads the 32-bit damage value with interrupts disabled. |
| `:14-16` — `RELAY_OPEN_CURRENT_UNKNOWN` counted as zero wear | **Valid.** Every emergency open (GFI, overcurrent) added no electrical damage, though those opens are the main source of wear. | The emergency path charges damage at the cached `m_ChargingCurrent`, a RAM read that adds no trip latency. `$GW lastopenma` still reports -1 ("not measured"). A known zero current still adds zero wear. |
| `:23` — damage truncated below 10 A | **Valid.** | Damage is computed from mA as one 64-bit product divided once: `mA²·pf·αT·100 / (IrmA²·N_e)`. The sub-millionth remainder carries over to the next hot open, in RAM only, so a reboot loses less than 1e-6. A 9 A open now adds 1.08 instead of 0, and repeated 5 A opens (about 0.33 each) add up. Current is capped at 1000 A to rule out overflow. |
| `:134` — thermal index too coarse | **Valid.** At x100 and whole amps, a few °C of rise at 32 A came out as 0. The learned baseline then stayed 0 and `ThermalWarningLevel()` could never fire. | H is now computed from mA at x10000 (`deltaTC10·1e9/mA²`) and capped at 0xfffe. The warning ratios stay valid because both values use the same scale. |
| `:103` — transit baseline truncates while learning | **Valid.** | The baseline is learned at 0.01 ms (`x100`, at most 30000 in a uint16). For the example sequence 4,6,…,6 it now learns 5.75 instead of 5. |

**Storage and compatibility:** the EEPROM layout is unchanged, but the two
baseline words now use finer scales. The count bytes (offsets 52 and 55) are
now marked with `RELAY_HEALTH_BASELINE_N_TAG` (0x40). A count byte without
that mark is treated as unlearned and learning restarts. That covers both
erased EEPROM and values written by earlier dev builds. Accumulated damage and
cold-open counts are kept.

**`$GL` wire format is unchanged:** `transitbaselinems` is still whole ms and
the thermal fields are still x100 (rounded), so the ESP32 firmware and GUI
need no changes.

## Cable temperature

| Finding | Verdict | Fix |
|---|---|---|
| `CableTempMonitor.cpp:143` — `r25=65535` reloads as 10000 | **Valid.** 0xffff is the erased-EEPROM value. | `CABLE_TEMP_MAX_R25` is now 65534, so `$SN` rejects 65535. Docs updated. |
| `CableTempMonitor.cpp:339` — a negative offset can hide the hot-end clamp | **Valid.** A reading clamped at 150.0 °C plus a negative offset could fall below `panic` (at most 1500). | Readings at or above `CABLE_TEMP_HOT_CLAMP_C10` skip the calibration offset. In-range readings still apply it. |
| `m328p/pindefs.h:29` — PC3 shared by the button and PP2 | **Valid, but the premise was wrong.** The review assumed `m328p_core_lcd` doesn't enable `CABLE_TEMPERATURE_MONITORING`. It does, through `[common]`, so the suggested `#error` would break the release build. | `PP2_PIN` is no longer defined for the 328P, where PC3 is the pulled-up button input. Without `PP2_PIN`, `CABLE_TEMP_PIN_MAX` becomes `CABLE_TEMP_PIN_PP`: `$SN idx 2 …` replies `$NK`, and a saved PP2 assignment loads as unassigned. SAMD is unchanged. |

## SAMD target

| Finding | Verdict | Fix |
|---|---|---|
| `Gfi.h:40` — `diagResult` starts at 0 ("pass") | **Valid.** The global is zero-initialised, so `$GQ` reported a pass before any test had run. | `Gfi::Init()` sets `diagResult = 0xff` and the other diagnostic markers. It runs once at boot, before any self-test. |
| `platformio.ini:59` — `samd_ice_dev` / `CALIBRATE` doesn't compile | **Valid.** A `;` was missing in `Calibrate()`. | Added the semicolon. That exposed an existing error: `FlushDeferredEepromWrites()` was declared only under `RELAY_ZC_SWITCH`, which `samd_ice_dev` doesn't set. Moved the declaration out of that block. `samd_ice_dev` builds again. |
| `target.cpp:168` — only 5 of 8 linearity calibration bits loaded | **Valid.** | Now ORs in `LINEARITY_1` shifted left by 5, matching the core's `wiring.c`. |
| `target.cpp:235` — `RESRDY` cleared after `START` | **Valid.** If the conversion finished first, its flag was wiped and the wait never ended. | `RESRDY` is now cleared before `SWTRIG.START`. |
| `target.cpp:228` — pin never routed to the ADC | **Valid.** The datasheet requires PMUX function B. | `AdcPin::read()` calls `pinPeripheral(pin, PIO_ANALOG)` once per instance, on the first read. That can't happen in `init()`, which may run from a global constructor before the core's `init()`. |

## Files touched

`firmware/open_evse/{CableTempMonitor.cpp,CableTempMonitor.h,J1772EvseController.cpp,J1772EvseController.h,RelayHealth.cpp,RelayHealth.h,open_evse.h,rapi_proc.cpp,rapi_proc.h}`,
`firmware/targets/m328p/pindefs.h`,
`firmware/targets/samd/{Gfi.cpp,target.cpp,target.h}`,
`firmware/CHANGELOG`, `docs/cable-temperature-monitoring.md`.
