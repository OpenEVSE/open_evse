//#define PZEM_RX_PIN MISO // ICSP 1
//#define PZEM_TX_PIN MOSI // ICSP 4
//#define PZEM_RX_PIN 9 // AUX_B = PB1
//#define PZEM_TX_PIN A3 // AUX_A = PC3

#define GFI_INTERRUPT 0 // interrupt number 0 = PD2, 1 = PD3
// interrupt number 0 = PD2, 1 = PD3
#define GFI_REG &PIND
#define GFI_IDX 2

// pin is supposed to be wrapped around the GFI CT 5+ times
// pin is supposed to be wrapped around the GFI CT 5+ times
#define GFITEST_REG &PIND
#define GFITEST_IDX 6
// V6 GFI test pin PB0
#define V6_GFITEST_REG &PINB
#define V6_GFITEST_IDX 0


// button sensing pin
#define BTN_REG &PINC
#define BTN_IDX 3


//J1772EVSEController
#define CURRENT_PIN 0 // analog current reading pin ADCx
#define PILOT_SENSE_PIN 1 // analog pilot voltage reading pin ADCx
#define PP_PIN 2 // PP_READ - ADC2
#define PP2_PIN 3 // PP2_READ - ADC3 (cable NTC thermistor input, see CableTempMonitor.h)

 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE1_REG &PIND
#define ACLINE1_IDX 3
// TEST PIN 2 for L1/L2, ground and stuck relay
#define ACLINE2_REG &PIND
#define ACLINE2_IDX 4

#ifdef VOLTMETER
// N.B. Note, ADC2 is already used as PP_PIN so beware of potential clashes
// voltmeter pin is ADC2 on OPENEVSE_2
#define VOLTMETER_PIN 2 // analog AC Line voltage voltmeter pin ADCx
#endif // VOLTMETER

#ifdef OPENEVSE_2
// This pin must match the last write to CHARGING_PIN, modulo a delay. If
// it is low when CHARGING_PIN is high, that's a missing ground.
// If it's high when CHARGING_PIN is low, that's a stuck relay.
// Auto L1/L2 is done with the voltmeter.
#define ACLINE1_REG &PIND // OpenEVSE II has only one AC test pin.
#define ACLINE1_IDX 3

#define CHARGING_REG &PIND // OpenEVSE II has just one relay pin.
#define CHARGING_IDX 7 // OpenEVSE II has just one relay pin.
#else // !OPENEVSE_2

 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE1_REG &PIND
#define ACLINE1_IDX 3
 // TEST PIN 2 for L1/L2, ground and stuck relay
#define ACLINE2_REG &PIND
#define ACLINE2_IDX 4

#define V6_CHARGING_PIN  5
#define V6_CHARGING_PIN2 6

// digital Relay trigger pin
#define CHARGING_REG &PINB
#define CHARGING_IDX 0
// digital Relay trigger pin for second relay
#define CHARGING2_REG &PIND
#define CHARGING2_IDX 7
//digital Charging pin for AC relay
#define CHARGINGAC_REG &PINB
#define CHARGINGAC_IDX 1

// obsolete LED pin
//#define RED_LED_REG &PIND
//#define RED_LED_IDX 5
// obsolete LED pin
//#define GREEN_LED_REG &PINB
//#define GREEN_LED_IDX 5
#endif // OPENEVSE_2

// N.B. if PAFC_PWM is enabled, then pilot pin can be PB1 or PB2
// if using fast PWM (PAFC_PWM disabled) pilot pin *MUST* be PB2
#define PILOT_REG &PINB
#define PILOT_IDX 2

#ifdef MENNEKES_LOCK
// requires external 12V H-bridge driver such as Polulu 1451

//D11 - MOSI
#define MENNEKES_LOCK_PINA_REG &PINB
#define MENNEKES_LOCK_PINA_IDX 3

//D12 - MISO
#define MENNEKES_LOCK_PINB_REG &PINB
#define MENNEKES_LOCK_PINB_IDX 4
#endif // MENNEKES_LOCK


// if defined, this pin goes HIGH when the EVSE is sleeping, and LOW otherwise
//#define SLEEP_STATUS_REG &PINB
//#define SLEEP_STATUS_IDX 4

#ifdef AUTH_LOCK
// AUTH_LOCK_REG/IDX - use an input pin to control AUTH_LOCK instead of
// manual function calls
// digital pin is configured as input with internal pull-up enabled
// EVSE is locked when input HIGH and unlocked when input LOW
//#define AUTH_LOCK_REG &PINC
//#define AUTH_LOCK_IDX 2
#endif // AUTH_LOCK

