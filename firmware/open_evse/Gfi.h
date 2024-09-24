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

class Gfi {
  DigitalPin pin;
  uint8_t m_GfiFault;
  uint8_t is_v6;
#ifdef GFI_SELFTEST
  uint8_t testSuccess;
  uint8_t testInProgress;
#endif // GFI_SELFTEST
public:
#ifdef GFI_SELFTEST
  DigitalPin pinTest;
#ifdef DC_GFI_TEST
  DigitalPin pinDCTest;
#endif  //DC_GFI_TEST
#endif  //GFI_SELFTEST

  Gfi() {}

  void Init(uint8_t v6=0);
  void Reset();
  void SetFault() { m_GfiFault = 1; }
  uint8_t Fault() { return m_GfiFault; }
#ifdef GFI_SELFTEST
  uint8_t SelfTest();
  void SetTestSuccess() { testSuccess = 1; }
  uint8_t SelfTestSuccess() { return testSuccess; }
  uint8_t SelfTestInProgress() { return testInProgress; }
#endif
};
