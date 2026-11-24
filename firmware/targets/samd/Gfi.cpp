/*
 * This file is part of Open EVSE.
 *
 * Copyright (c) 2011-2019 Sam C. Lin
 *
 * Open EVSE is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * Open EVSE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Open EVSE; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
#include "open_evse.h"

#ifdef GFI
// interrupt service routing
void gfi_isr()
{
#ifndef BYPASS_GFI
  g_EvseController.SetGfiTripped();
#endif
}

#include <Arduino.h>

#ifdef GFI_SELFTEST
// Strong drive (~7 mA vs ~2 mA at 3.3 V) on the GFI test pin, for more base
// drive into the test transistor. pinMode() rewrites PINCFG and clears
// DRVSTR, so call this after every pinMode() on the pin -- including the one
// tone() does internally on its first call.
static inline void gfiTestPinStrongDrive()
{
  PORT->Group[g_APinDescription[GFITEST_REG].ulPort]
    .PINCFG[g_APinDescription[GFITEST_REG].ulPin].bit.DRVSTR = 1;
}
#endif // GFI_SELFTEST

void Gfi::Init(uint8_t v6)
{
#ifdef GFI_SELFTEST_DIAG
  // $GQ must read "never run" (255) until a SelfTest() actually runs - e.g.
  // when the self-test is disabled - not the zero-initialised 0 ("pass").
  // Init() runs once, at boot (EvseReset() from setup()), before any
  // SelfTest(), so this never discards a completed result.
  diagResult = 0xff;
  diagPinBefore = 0;
  diagEdgeCnt = 0;
  diagFirstEdgeMs = 0xffff;
  diagStartMs = 0;
  diagHighSamples = 0;
  diagSamples = 0;
  diagClearMs = 0xffff;
#endif // GFI_SELFTEST_DIAG
#ifdef GFI_SELFTEST
  // Drive the test pin low from boot. Otherwise PA06 floats (reset state)
  // until the first tone() call, leaving the test transistor's base undriven.
  digitalWrite(GFITEST_REG,LOW);
  pinMode(GFITEST_REG,OUTPUT);
  gfiTestPinStrongDrive();
#endif // GFI_SELFTEST

  pin.init(GFI_REG,GFI_IDX,DigitalPin::INP);
  // GFI triggers on rising edge
  attachInterrupt(digitalPinToInterrupt(GFI_REG),gfi_isr,RISING);

  Reset();
}


//RESET GFI LOGIC
void Gfi::Reset()
{
  WDT_RESET();

#ifdef GFI_SELFTEST
  testInProgress = 0;
  testSuccess = 0;
#endif // GFI_SELFTEST

  if (pin.read()) m_GfiFault = 1; // if interrupt pin is high, set fault
  else m_GfiFault = 0;
}

#ifdef GFI_SELFTEST
uint8_t Gfi::SelfTest()
{
#ifdef BYPASS_GFI
  return 0;
#endif
  int i;
#ifdef GFI_SELFTEST_DIAG
  diagResult = 0xff; // in progress / didn't finish
  diagEdgeCnt = 0;
  diagFirstEdgeMs = 0xffff;
  diagHighSamples = 0;
  diagSamples = 0;
  diagClearMs = 0xffff;
  diagPinBefore = pin.read();
#endif
  // wait for GFI pin to clear
  for (i=0;i < 20;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
  if (i == 20) {
#ifdef GFI_SELFTEST_DIAG
    diagResult = 2;
#endif
    return 2;
  }

  testSuccess = 0;
  testInProgress = 1;

  WDT_RESET();
  unsigned long waitms = GFI_TEST_CYCLES/60*1000ul;
  unsigned long curms,startms = millis();
#ifdef GFI_SELFTEST_DIAG
  diagStartMs = startms;
#endif
  tone(GFITEST_REG,60,waitms);
  // tone()'s first call does pinMode(OUTPUT), which clears DRVSTR. Re-assert
  // it; the first toggle is a half-period (~8 ms) away, so no pulse is weak.
  gfiTestPinStrongDrive();

  unsigned long dogms = startms;
#ifdef GFI_SELFTEST_DIAG
  unsigned long samplems = startms;
#endif
  while (((curms=millis())-startms) < waitms) {
    if (testSuccess) break;
#ifdef GFI_SELFTEST_DIAG
    // poll the GFI output once per ms: if it goes HIGH but the ISR never
    // fires, the problem is interrupt delivery, not the GFI circuit
    if (curms != samplems) {
      samplems = curms;
      diagSamples++;
      if (pin.read()) diagHighSamples++;
    }
#endif
    if ((curms-dogms) >= (WATCHDOG_TIMEOUT_SEC*500ul)) {
      WDT_RESET();
      dogms = curms;
    }
  }

  // Stop the test current NOW, as the AVR pulse loop does. tone() otherwise
  // keeps driving the test winding for the rest of waitms (~1 s) after the
  // trip. The pin-clear wait below can then see the GFI output low *between*
  // test pulses, return with testInProgress cleared, and the next pulse trips
  // the real gfi_isr() path -> spurious GFCI fault right after a "pass".
  // IRQ masked so a pending TC5 toggle can't re-raise the pin after noTone().
  NVIC_DisableIRQ(TC5_IRQn);
  noTone(GFITEST_REG);
  NVIC_ClearPendingIRQ(TC5_IRQn);

  // wait for GFI pin to clear
  unsigned long stopms = millis();
  for (i=0;i < 40;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
#ifdef GFI_SELFTEST_DIAG
  if (i < 40) diagClearMs = (uint16_t)(millis() - stopms);
#endif
  if (i == 40) {
    // MUST clear testInProgress on every exit: while it is set, gfi_isr()
    // treats a real ground fault as "test success" and does NOT trip.
    testInProgress = 0;
#ifdef GFI_SELFTEST_DIAG
    diagResult = 3;
#endif
    return 3;
  }


  // sometimes getting spurious GFI faults when testing just before closing
  // relay.
  // wait a little more for everything to settle down (CT/filter ringing after
  // the test current stops). testInProgress is still set, so any trailing
  // edges are absorbed as test success rather than tripping a real fault.
  // Same as the AVR target.
  wdt_delay(1000);

  // Re-check before standing the self-test down. If the output went high
  // during the settle and is still high, clearing testInProgress now would
  // leave it high with no rising edge left to fire gfi_isr() - a real fault
  // that started in this window would never trip. Fail the test instead.
  if (pin.read()) {
    testInProgress = 0;
#ifdef GFI_SELFTEST_DIAG
    diagResult = 3;
#endif
    return 3;
  }

  m_GfiFault = 0;

  testInProgress = 0;

#ifdef GFI_SELFTEST_DIAG
  diagResult = !testSuccess;
#endif
  return !testSuccess;
}
#endif // GFI_SELFTEST

#ifdef GFI_SELFTEST_DIAG
void Gfi::GetDiag(char *buf)
{
  sprintf(buf,"%u %u %u %u %u %u %u",
          (unsigned)diagResult,(unsigned)diagPinBefore,
          (unsigned)diagEdgeCnt,(unsigned)diagFirstEdgeMs,
          (unsigned)diagHighSamples,(unsigned)diagSamples,
          (unsigned)diagClearMs);
}
#endif // GFI_SELFTEST_DIAG
#endif // GFI
