// -*- C++ -*-
//
// Pin assignments for the native (host) target.
//
// There is no silicon here, so a "pin" is just an index into the pin state
// block in target.cpp. The numbering is arbitrary and local to this target;
// only uniqueness matters. DigitalPin's second constructor argument (the bit
// index on a real port) is unused, so every *_IDX is 0 -- the same convention
// the SAMD target uses.

#pragma once

// digital inputs
#define GFI_REG        0  // GFI trip line (interrupt source)
#define GFI_IDX        0
#define ACLINE1_REG    1  // TEST PIN 1 - L1/L2, ground and stuck relay
#define ACLINE1_IDX    0
#define ACLINE2_REG    2  // TEST PIN 2 - ground monitor
#define ACLINE2_IDX    0

// digital outputs
#define GFITEST_REG    3  // GFI self-test coil drive
#define GFITEST_IDX    0
#define PILOT_REG      4  // J1772 pilot (PWM)
#define PILOT_IDX      0
#define CHARGING_REG   5  // DC relay 1
#define CHARGING_IDX   0
#define CHARGING2_REG  6  // DC relay 2
#define CHARGING2_IDX  0
#define CHARGINGAC_REG 7  // AC relay
#define CHARGINGAC_IDX 0

// OEV6 drives its relays through the Arduino API rather than DigitalPin, so
// these are plain pin numbers rather than indices into the block above.
#define V6_CHARGING_PIN  10
#define V6_CHARGING_PIN2 11

#ifdef MENNEKES_LOCK
#define MENNEKES_LOCK_PINA_REG 8  // LOCK
#define MENNEKES_LOCK_PINA_IDX 0
#define MENNEKES_LOCK_PINB_REG 9  // UNLOCK
#define MENNEKES_LOCK_PINB_IDX 0
#endif // MENNEKES_LOCK

// analog inputs
#define CURRENT_PIN     0 // ammeter CT
#define PILOT_SENSE_PIN 1 // pilot voltage feedback
#define PP_PIN          2 // proximity pilot

// Deliberately left undefined, as on the SAMD target: RED_LED_REG,
// GREEN_LED_REG, BTN_REG, SLEEP_STATUS_REG, AUTH_LOCK_REG, VOLTMETER_PIN.
// The shared code compiles these out when they are absent.

// Total number of digital pins the block must hold. Keep in step with the
// highest *_REG above.
#define NATIVE_DIGITAL_PIN_COUNT 12
#define NATIVE_ADC_PIN_COUNT      3
