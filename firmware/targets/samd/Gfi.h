/*
 * Open EVSE Firmware
 *
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
#pragma once

#ifdef GFI_SELFTEST
// Self-test diagnostics, read back via RAPI $GQ, test run on demand via $FQ
#define GFI_SELFTEST_DIAG
#endif

class Gfi {
  DigitalPin pin;
  uint8_t m_GfiFault;
#ifdef GFI_SELFTEST
  volatile uint8_t testSuccess;
  volatile uint8_t testInProgress;
  void gfiPWM60Hz(uint8_t pin,unsigned long waitms);
#endif // GFI_SELFTEST
#ifdef GFI_SELFTEST_DIAG
  // filled by SelfTest(); see $GQ in rapi_proc.h for the meaning of each
  uint8_t  diagResult;      // last SelfTest() return code
  uint8_t  diagPinBefore;   // GFI pin level just before test current starts
  volatile uint16_t diagEdgeCnt; // rising edges seen by the ISR during the test
  volatile uint16_t diagFirstEdgeMs; // ms from test start to first edge
  unsigned long diagStartMs;
  uint16_t diagHighSamples; // polls of the GFI pin that read HIGH while test current flowed
  uint16_t diagSamples;     // total polls while test current flowed
  uint16_t diagClearMs;     // ms for GFI pin to go low after test current stopped
#endif // GFI_SELFTEST_DIAG

public:

  Gfi() {}

  void Init(uint8_t v6=0);
  void Reset();
  void SetFault() { m_GfiFault = 1; }
  uint8_t Fault() { return m_GfiFault; }
#ifdef GFI_SELFTEST
  uint8_t SelfTest();
#ifdef GFI_SELFTEST_DIAG
  // called from ISR context (gfi_isr -> SetGfiTripped)
  void SetTestSuccess() {
    if (!testSuccess) diagFirstEdgeMs = (uint16_t)(millis() - diagStartMs);
    if (diagEdgeCnt < 0xffff) diagEdgeCnt++;
    testSuccess = 1;
  }
  // "$OK result pinbefore edges firstedgems highsamples samples clearms"
  void GetDiag(char *buf);
#else
  void SetTestSuccess() { testSuccess = 1; }
#endif
  uint8_t SelfTestSuccess() { return testSuccess; }
  uint8_t SelfTestInProgress() { return testInProgress; }
#endif
};
