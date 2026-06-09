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
  g_EvseController.SetGfiTripped();
}


void Gfi::Init(uint8_t v6)
{
  is_v6 = v6;
  pin.init(GFI_REG,GFI_IDX,DigitalPin::INP);
  // GFI triggers on rising edge
  attachInterrupt(GFI_INTERRUPT,gfi_isr,RISING);

#ifdef GFI_SELFTEST
  volatile uint8_t *reg = GFITEST_REG;
  volatile uint8_t idx  = GFITEST_IDX;
#ifdef OEV6
  if (is_v6) {
    reg = V6_GFITEST_REG;
    idx = V6_GFITEST_IDX;
  }
#endif // OEV6
  pinTest.init(reg,idx,DigitalPin::OUT);
#ifdef DC_GFI_TEST
#ifdef OEV6
  if (v6) {
    pinMode(V6_DC_TEST_PIN,OUTPUT);
  } else {
#endif // OEV6
    volatile uint8_t *dcreg = DC_TEST_REG;
    volatile uint8_t dcidx  = DC_TEST_IDX;
    pinDCTest.init(dcreg,dcidx,DigitalPin::OUT);
#ifdef OEV6
  }
#endif // OEV6
#endif //DC_GFI_TEST
#endif //GFI_SELFTEST

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
  int i;
  // wait for GFI pin to clear
  for (i=0;i < 20;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
  if (i == 20) return 2;

  testInProgress = 1;
  testSuccess = 0;
  for(int i=0; !testSuccess && (i < GFI_TEST_CYCLES); i++) {
    pinTest.write(1);
    delayMicroseconds(GFI_PULSE_ON_US);
    pinTest.write(0);
    delayMicroseconds(GFI_PULSE_OFF_US);
  }

  // wait for GFI pin to clear
  for (i=0;i < 40;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
  if (i == 40) return 3;

#ifdef DC_GFI_TEST
  if (testSuccess) {  //no point continuing unless AC test was successful
    testSuccess = 0;
    // turn on DC test pin
#ifdef OEV6
    if (is_v6) {
#ifdef V6_DC_TEST_PIN
      digitalWrite(V6_DC_TEST_PIN,HIGH);
#endif
    } else {
#endif //OEV6
    pinDCTest.write(1);
#ifdef OEV6
    }
#endif //OEV6
    for(int i=0; !testSuccess && (i < GFI_TEST_CYCLES); i++) {
      //just wait this time...
      delayMicroseconds(GFI_PULSE_ON_US);
      delayMicroseconds(GFI_PULSE_OFF_US);
    }
    //turn off the DC test pin
#ifdef OEV6
    if (is_v6) {
#ifdef V6_DC_TEST_PIN
      digitalWrite(V6_DC_TEST_PIN,LOW);
#endif
    } else {
#endif //OEV6
    pinDCTest.write(0);
#ifdef OEV6
    }
#endif //OEV6
    // wait for GFI pin to clear
    for (i=0;i < 40;i++) {
      WDT_RESET();
      if (!pin.read()) break;
      delay(50);
    }
    if (i == 40) return 4;
  }
  //if testSuccess == 1 at this point then both tests were successful
#endif //DC_GFI_TEST

#ifndef OPENEVSE_2
  // sometimes getting spurious GFI faults when testing just before closing
  // relay.
  // wait a little more for everything to settle down
  // this delay is needed only if 10uF cap is in the circuit, which makes the circuit
  // temporarily overly sensitive to trips until it discharges
  wdt_delay(1000);
#endif // OPENEVSE_2

  m_GfiFault = 0;
  testInProgress = 0;

  return !testSuccess;
}
#endif // GFI_SELFTEST
#endif // GFI
