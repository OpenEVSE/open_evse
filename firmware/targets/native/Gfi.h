// -*- C++ -*-
//
// GFI for the native (host) target.
//
// Shaped after the m328p target rather than SAMD. SAMD drives the test coil
// with a single tone() call and then spins on millis() for a second; tone()
// is a no-op on the host and that spin offers nothing a driver can respond
// to. The m328p form toggles a real pin GFI_TEST_CYCLES times with a wait
// between every edge, so each edge is an observable event and the loop exits
// as soon as the trip arrives.

#pragma once

class Gfi {
  DigitalPin pin;
  uint8_t m_GfiFault;
#ifdef GFI_SELFTEST
  volatile uint8_t testSuccess;
  uint8_t testInProgress;
#endif // GFI_SELFTEST

public:
#ifdef GFI_SELFTEST
  // public: shared code drives it directly under FT_GFI_RETRY
  DigitalPin pinTest;
#endif

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
