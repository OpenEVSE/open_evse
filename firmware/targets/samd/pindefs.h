#include "samd_pins.h"

#define GFI_REG ZERO_PA14

// pin is supposed to be wrapped around the GFI CT 5+ times
#define GFITEST_REG ZERO_PA06

#define PILOT_REG ZERO_PA18 // PWM

// analog pins
#define CURRENT_PIN     ZERO_PA02 // analog current reading pin A0
#define PILOT_SENSE_PIN ZERO_PB08 // analog pilot voltage reading pin A1
#define PP_PIN          ZERO_PB09 // PP_READ - A2
#define PP2_PIN         ZERO_PA04 // PP2_READ - A3 (cable NTC thermistor input, see CableTempMonitor.h)


#define ACLINE1_REG ZERO_PA08 // WELD_DETECT
#define ACLINE2_REG ZERO_PA09 // GMI_LINE

// digital Relay trigger pin
#define CHARGING_REG   ZERO_PA15 // DC_RELAY_1
#define CHARGING2_REG  ZERO_PA20 // DC_RELAY_2
#define CHARGINGAC_REG ZERO_PA07 // AC_RELAY

// requires external 12V H-bridge driver such as Polulu 1451
#define MENNEKES_LOCK_PINA_REG ZERO_PA16 // LOCK
#define MENNEKES_LOCK_PINB_REG ZERO_PA19 // UNLOCK



#ifdef RELAY_ZC_SWITCH
#define GMI_ADC_PIN ZERO_PA09  // GMI_LINE — ADC-capable, for voltage ZC detection
#endif

// dummies - unused
#define PILOT_IDX 0
#define CHARGING_IDX 0
#define CHARGING2_IDX 0
#define CHARGINGAC_IDX 0
#define ACLINE1_IDX 0
#define ACLINE2_IDX 0

#define GFI_IDX 0
#define GFITEST_IDX 0

#define MENNEKES_LOCK_PINA_IDX 0
#define MENNEKES_LOCK_PINB_IDX 0
