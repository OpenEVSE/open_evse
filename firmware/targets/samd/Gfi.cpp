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


void Gfi::Init(uint8_t v6)
{
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
  //    RAPI_SERIAL_PORT.print("GST ");
  //    uint32_t sms = millis();
  int i;
  // wait for GFI pin to clear
  for (i=0;i < 20;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
  if (i == 20) return 2;
  //    RAPI_SERIAL_PORT.print(millis()-sms);

  testInProgress = 1;
  testSuccess = 0;

  WDT_RESET();
  unsigned long waitms = GFI_TEST_CYCLES/60*1000ul;
  tone(GFITEST_REG,60,waitms);

  unsigned long curms,startms = millis();
  unsigned long dogms = startms;
  while (((curms=millis())-startms) < waitms) {
    if (testSuccess) break;
    else if ((curms-dogms) >= (WATCHDOG_TIMEOUT_SEC*500ul)) {
      WDT_RESET();
      dogms = curms;
    }
  }

  //  RAPI_SERIAL_PORT.print(" ST ");RAPI_SERIAL_PORT.print(millis()-sms);
  //  RAPI_SERIAL_PORT.print("tss ");RAPI_SERIAL_PORT.print(testSuccess);

  // wait for GFI pin to clear
  for (i=0;i < 40;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
  if (i == 40) return 3;
  //  RAPI_SERIAL_PORT.print(" w ");RAPI_SERIAL_PORT.print(i);
  //  RAPI_SERIAL_PORT.print(" s ");RAPI_SERIAL_PORT.println(testSuccess);


#ifdef dontneed
  // sometimes getting spurious GFI faults when testing just before closing
  // relay.
  // wait a little more for everything to settle down
  // this delay is needed only if 10uF cap is in the circuit, which makes the circuit
  // temporarily overly sensitive to trips until it discharges
  wdt_delay(1000);
#endif 

  m_GfiFault = 0;

  testInProgress = 0;

  return !testSuccess;
}
#endif // GFI_SELFTEST
#endif // GFI
