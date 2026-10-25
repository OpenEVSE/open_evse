#include "open_evse.h"

#ifdef GFI

// Interrupt service routine. On the host there is no hardware interrupt; a
// driver raising the GFI line is picked up by EpoxyDuino's polled edge
// detection in checkInterrupts(), which then calls this.
void gfi_isr()
{
#ifndef BYPASS_GFI
  g_EvseController.SetGfiTripped();
#endif
}

void Gfi::Init(uint8_t /*v6*/)
{
  pin.init(GFI_REG, GFI_IDX, DigitalPin::INP);
  attachInterrupt(GFI_REG, gfi_isr, RISING);

#ifdef GFI_SELFTEST
  pinTest.init(GFITEST_REG, GFITEST_IDX, DigitalPin::OUT);
  pinTest.write(0);
#endif

  Reset();
}

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
  // wait for GFI pin to clear
  for (i=0;i < 20;i++) {
    WDT_RESET();
    if (!pin.read()) break;
    delay(50);
  }
  if (i == 20) return 2;

  testInProgress = 1;
  testSuccess = 0;

  WDT_RESET();
  // Pulse the test coil at ~60Hz. Every edge is a write a driver can see and
  // every gap is a delay that services the trip coming back, so the loop ends
  // as soon as testSuccess is set rather than always running to completion.
  for (i=0; !testSuccess && (i < GFI_TEST_CYCLES); i++) {
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

  m_GfiFault = 0;
  testInProgress = 0;

  return !testSuccess;
}
#endif // GFI_SELFTEST
#endif // GFI
