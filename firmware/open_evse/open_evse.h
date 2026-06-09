// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * This file is part of Open EVSE.

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
 
// UK/EU specific settings (by OpenEnergyMonitor):
// - Disable AUTOSVCLEVEL (autodetection is designed for split-phase)
// - Charging level default to L2
// - Set MAX_CURRENT_CAPACITY_L2 32 (European Limit)
// - Add '.EU' to version number
// - Enable LCD Re-draw every couple of min (required for EMC/CE)

#pragma once

#define OPEN_EVSE

#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <pins_arduino.h>
#include "./Wire.h"
#include "avrstuff.h"
#include "i2caddr.h"

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO

#define setBits(flags,bits) (flags |= (bits))
#define clrBits(flags,bits) (flags &= ~(bits))

#ifndef VERSION
#define VERSION "8.2.3"
#endif // !VERSION

#include "Language_default.h"   //Default language should always be included as bottom layer

typedef unsigned long time_t;

//Language preferences: Add your custom languagefile here. See Language_default.h for more info.
//#include "Language_norwegian.h"

//-- begin features

#ifndef PLATFORMIO
//#define OCPP
// support V6 hardware
#define OEV6
#ifdef OEV6
//#define INVERT_V6_DETECTION // DO NOT USE: ONLY FOR lincomatic's BETA V6 board
#define RELAY_PWM
#define RELAY_HOLD_DELAY_TUNING // enable Z0
#endif // OEV6

// enable CGMI support
//#define ENABLE_CGMI

// auto detect L1/L2
#ifndef NO_AUTOSVCLEVEL
#define AUTOSVCLEVEL
#endif

// on boot, EVSE locked until receives $SB 
//#define BOOTLOCK

// show disabled tests before POST
#define SHOW_DISABLED_TESTS

// current measurement
#define AMMETER

// Enable three-phase energy calculation
// Note: three-phase energy will always be calculated even if EV is only using singe-phase. Ony enable if always charging 3-phase EV and aware of this limitation.
//#define THREEPHASE

// charging access control - if defined, enables RAPI G4/S4 commands
//  to enable/disable charging function
// if AUTH_LOCK_REG/IDX are also defined (see below), then a hardware pin is
//  used to control access, rather than RAPI
// defining AUTH_LOCK enables locking functionality
// AUTH_LOCK = 1 -> default to locked, automatically lock whenever transition to state A
// AUTH_LOCK = 0 -> only locks when RAPI command sent to lock
//#define AUTH_LOCK 1

// serial remote api
#define RAPI

// RAPI over serial
#define RAPI_SERIAL

// RAPI $WF support
#define RAPI_WF

// RAPI $AN support
#define RAPI_BTN

// RAPI over I2C
//#define RAPI_I2C

// enable sending of RAPI commands
//#define RAPI_SENDER

// EVSE must call state transition function for permission to change states
//#define STATE_TRANSITION_REQ_FUNC

// auto detect ampacity by PP pin resistor
// #define PP_AUTO_AMPACITY

// charge for a specified amount of time and then stop
#define TIME_LIMIT


// stop charging after a certain kWh reached - requires KWH_RECORDING
#define CHARGE_LIMIT

// support Mennekes (IEC 62196) type 2 locking pin
#define MENNEKES_LOCK

// Support for Nick Sayer's OpenEVSE II board, which has alternate hardware for ground check/stuck relay check and a voltmeter for L1/L2.
//#define OPENEVSE_2

#endif // !PLATFORMIO

// enable $GI
#define MCU_ID_LEN 10

// enable watchdog timer
#define WATCHDOG

#ifdef PP_AUTO_AMPACITY
#define STATE_TRANSITION_REQ_FUNC

#include "AutoCurrentCapacityController.h"

extern AutoCurrentCapacityController g_ACCController;
#endif

#ifdef OPENEVSE_2
// If the AC voltage is > 150,000 mV, then it's L2. Else, L1.
#define L2_VOLTAGE_THRESHOLD (150000)
#define VOLTMETER
// 35 ms is just a bit longer than 1.5 cycles at 50 Hz
#define VOLTMETER_POLL_INTERVAL (35)
// This is just a wild guess
// #define VOLTMETER_SCALE_FACTOR (266)     // original guess
//#define DEFAULT_VOLT_SCALE_FACTOR (262)        // calibrated for Craig K OpenEVSE II build
#define DEFAULT_VOLT_SCALE_FACTOR (298)        // calibrated for lincomatic's OEII
// #define VOLTMETER_OFFSET_FACTOR (40000)  // original guess
//#define DEFAULT_VOLT_OFFSET (46800)     // calibrated for Craig K OpenEVSE II build
#define DEFAULT_VOLT_OFFSET (12018)     // calibrated for lincomatic's OEII
#endif // OPENEVSE_2

// GFI support
#define GFI

// If you loop a wire from the third GFI pin through the CT a few times and then to ground,
// enable this. ADVPWR must also be defined.
#define GFI_SELFTEST
// If you loop a single turn of wire from the 12V supply through a 2k resistor to the
// second DC relay pin, enable this for DC fault test
//#define DC_GFI_TEST

// behavior specified by UL
// 1) if enabled, POST failure will cause a hard fault until power cycled.
//    disabled, will retry POST continuously until it passes
// 2) if enabled, any a fault occurs immediately after charge is initiated,
//    hard fault until power cycled. Otherwise, do the standard delay/retry sequence
#define UL_COMPLIANT

#ifdef UL_COMPLIANT
#define ADVPWR
#define GFI
// if enabled, do GFI self test before closing relay
#define UL_GFI_SELFTEST
#define GFI_SELFTEST
#endif //UL_COMPLIANT

#define TEMPERATURE_MONITORING  // Temperature monitoring support

#define HEARTBEAT_SUPERVISION // Heartbeat Supervision support

#ifdef AMMETER

// if OVERCURRENT_THRESHOLD is defined, then EVSE will hard fault in
// the event that the EV is pulling more current than it's allowed to
// declare overcurrent when charging amps > pilot amps + OVERCURRENT_THRESHOLD
//#define OVERCURRENT_THRESHOLD 5 // A
// go to error state overcurrent by OVERCURRENT_THRESHOLD amps
// for OVERCURRENT_TIMEOUT ms
//#define OVERCURRENT_TIMEOUT 10000UL // ms

// if there's no accurate voltmeter, hardcode voltages
#ifndef MV_FOR_L1
#define MV_FOR_L1 120000L       // conventional for North America
#endif
#ifndef MV_FOR_L2
#define MV_FOR_L2 240000L       // conventional for North America
//  #define MV_FOR_L2 230000L   // conventional for most of the world
#endif

// kWh Recording feature depends upon #AMMETER support
// comment out KWH_RECORDING to have the elapsed time and time of day displayed on the second line of the LCD
#define KWH_RECORDING
#ifdef KWH_RECORDING

// update interval in ms - code assumes that current/voltage are constant
// during this interval
#define KWH_CALC_INTERVAL_MS (250UL)

#include "EnergyMeter.h"
#endif // KWH_RECORDING


#endif //AMMETER

//Adafruit RGBLCD (MCP23017) - can have RGB or monochrome backlight
#define RGBLCD

//select default LCD backlight mode. can be overridden w/CLI/RAPI
#define BKL_TYPE_MONO 0
#define BKL_TYPE_RGB  1
#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_RGB
//#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_MONO

// Adafruit LCD backpack in I2C mode (MCP23008)
//#define I2CLCD

// Support PCF8574* based I2C backpack using F. Malpartida's library
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// *requires* I2CLCD enabled and RGBLCD disabled
//#define I2CLCD_PCF8574
#ifdef I2CLCD_PCF8574
#define I2CLCD
#undef RGBLCD
#endif // I2CLCD_PCF8574

// Advanced Powersupply... Ground check, stuck relay, L1/L2 detection.
#define ADVPWR

// single button menus (needs LCD enabled)
// connect an SPST-NO button between pin defined by BTN_REG/BTN_IDX and GND or enable ADAFRUIT_BTN to use the
// select button of the Adafruit RGB LCD
// How to use 1-button menu
// Long press activates menu
// When within menus, short press cycles menu items, long press selects and exits current submenu
#define BTN_MENU

// take out basic setup stuff that the user really shouldn't be changing,
// which can be set via RAPI/WiFi module.. reclaims a lot of code space
#define NOSETUP_MENU

// When not in menus, short press instantly stops the EVSE - another short press resumes.  Long press activates menus
// also allows menus to be manipulated even when in State B/C
#define BTN_ENABLE_TOGGLE

#ifdef BTN_MENU
// use Adafruit RGB LCD select button
#ifdef RGBLCD
#define ADAFRUIT_BTN
#endif // RGBLCD
#endif // BTN_MENU

// Option for RTC and DelayTime
// REQUIRES HARDWARE RTC: DS1307 or DS3231 connected via I2C
#define RTC // enable RTC & timer functions

#ifdef RTC
// Option for Delay Timer - GoldServe
#define DELAYTIMER

#if defined(DELAYTIMER) && defined(BTN_MENU)
#define DELAYTIMER_MENU
#endif

#else // !RTC
// this weird error comes out if RTC not defined, due to a bug in g++
//D:\git\open_evse\firmware\open_evse\open_evse.ino: In function 'ProcessInputs'//:
//
//open_evse:2384: error: unable to find a register to spill in class 'NO_REGS'
//
// }
//
// ^
//
//open_evse:2384: error: this is the insn:
//
//(insn 884 881 887 135 (set (mem:QI (post_dec:HI (reg/f:HI 32 __SP_L__)) [0  S1// A8])
//
////        (subreg:QI (reg/f:HI 1065) 1)) C:\Users\Geek\AppData\Local\Temp\arduino_build_853681\sketch\rapi_proc.cpp:418 1 {pushqi1}
#define GPPBUGKLUDGE
#endif // RTC

#ifdef OCPP
#define AUTH_LOCK 1
#define RAPI_SERIAL
#endif // OCPP


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



// for stability testing - shorter timeout/higher retry count
//#define GFI_TESTING

// phase and frequency correct PWM 1/8000 resolution
// when not defined, use fast PWM -> 1/250 resolution
#define PAFC_PWM

// glynhudson reports that LCD gets corrupted by EMC testing during CE
// certification.. redraw display periodically when enabled
//#define PERIODIC_LCD_REFRESH_MS 120000UL

// when closing DC relay set to HIGH for m_relayCloseMs, then
// switch to m_relayHoldPwm
// ONLY WORKS PWM-CAPABLE PINS!!!
// use Arduino pin number PD5 = 5, PD6 = 6
//#define RELAY_PWM
#define DEFAULT_RELAY_CLOSE_MS 100
#define DEFAULT_RELAY_HOLD_PWM 255 // (0-255, where 0=0%, 255=100%
// enables RAPI $Z0 for tuning PWM (see rapi_proc.h for $Z0 syntax)
// PWM parameters written to/loaded from EEPROM
#define RELAY_HOLD_DELAY_TUNING // enable Z0

//-- end features

#ifndef DEFAULT_LCD_BKL_TYPE
#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_MONO
#endif

#if defined(RGBLCD) || defined(I2CLCD)
#define LCD16X2
//If LCD is not defined, undef BTN_MENU - requires LCD
#else
#undef BTN_MENU
#endif // RGBLCD || I2CLCD

#if defined(OPENEVSE_2) && !defined(ADVPWR)
#error INVALID CONFIG - OPENEVSE_2 implies/requires ADVPWR
#endif

#if defined(UL_COMPLIANT) && !defined(GFI_SELFTEST)
#error INVALID CONFIG - GFI SELF TEST NEEDED FOR UL COMPLIANCE
#endif

#if defined(GFI_SELFTEST) && !defined(GFI)
#error INVALID_CONFIG - GFI NEEDED FOR GFI SELF TEST
#endif

// for testing print various diagnostic messages to the UART
//#define SERDBG

//
// begin functional tests
//
//
// for debugging ONLY - turns off all safety checks
//#define NOCHECKS
// DO NOT USE FT_xxx. FOR FUNCTIONAL TESTING ONLY
//
// Test for GFI fault lockout
// immediately GFI fault when entering STATE C -> should hard fault
// there will be a delay of a few seconds before the fault because we
// need to loop to cause the GFI fault
// right after GFI fault generated, will flash Closing/Relay on LCD
// -> should hard GFCI fault instantly when relay closes
//#define FT_GFI_LOCKOUT

// Test for auto reclose after GFI fault. Attach EVSIM in STATE C
// 10 sec after charging starts will induce fault. 1 minute after fault
// induced, should clear the fault and resume normal operation
//#define FT_GFI_RETRY

//
// test delay between start of sleep mode and opening relay
// connect EV and put into charging state C
// relay should open after pilot goes back to state B or higher
// or after 3 sec, whichever happens first
// LCD will display SLEEP OPEN/THRESH if opened due to EV response
//              or  SLEEP OPEN/TIMEOUT if due to timeout
//
//#define FT_SLEEP_DELAY
//
// endurance test
// alternate state B 9 sec/state C 1 sec forever
// top line displays cycle count
// bottom line displays ac pin state
//#define FT_ENDURANCE

// just read AC pins and display on line 1
//#define FT_READ_AC_PINS

//
// end functional tests
//

//-- begin configuration

// WARNING: ALL DELAYS *MUST* BE SHORTER THAN THIS TIMER OR WE WILL GET INTO
// AN INFINITE RESET LOOP
#define WATCHDOG_TIMEOUT WDTO_2S

#define LCD_MAX_CHARS_PER_LINE 16

#define TMP_BUF_SIZE ((LCD_MAX_CHARS_PER_LINE+1)*2)


// n.b. DEFAULT_SERVICE_LEVEL is ignored if ADVPWR defined, since it's autodetected
#ifndef DEFAULT_SERVICE_LEVEL
#define DEFAULT_SERVICE_LEVEL 2 // 1=L1, 2=L2
#endif

// current capacity in amps
#ifndef DEFAULT_CURRENT_CAPACITY_L1
#define DEFAULT_CURRENT_CAPACITY_L1 12
#endif
#ifndef DEFAULT_CURRENT_CAPACITY_L2
#define DEFAULT_CURRENT_CAPACITY_L2 24
#endif

// minimum allowable current in amps
#ifndef MIN_CURRENT_CAPACITY_J1772
#define MIN_CURRENT_CAPACITY_J1772 6
#endif

// maximum allowable current in amps
#ifndef MAX_CURRENT_CAPACITY_L1
#define MAX_CURRENT_CAPACITY_L1 24 // J1772 Max for L1 on a 20A circuit = 16, 15A circuit = 12 Some Vehicles suppoty 24A on 30A RV outlet TT-30
#endif
#ifndef MAX_CURRENT_CAPACITY_L2
#define MAX_CURRENT_CAPACITY_L2 80 // J1772 Max for L2 = 80
#endif

//J1772EVSEController

#define CURRENT_PIN 0 // analog current reading pin ADCx
#define PILOT_PIN 1 // analog pilot voltage reading pin ADCx
#define PP_PIN 2 // PP_READ - ADC2
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

// digital Relay trigger pin
#define V6_CHARGING_PIN  5
#define CHARGING_REG &PINB
#define CHARGING_IDX 0

#ifdef DC_GFI_TEST
// 6ma DC leakage test when this pin is high
#define V6_DC_TEST_PIN 6
#define DC_TEST_REG &PIND
#define DC_TEST_IDX 7
#else
// digital Relay trigger pin for second relay
#define V6_CHARGING_PIN2 6
#define CHARGING2_REG &PIND
#define CHARGING2_IDX 7
#endif //DC_GFI_TEST

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
#include "MennekesLock.h"
#endif // MENNEKES_LOCK




#define SERIAL_BAUD 115200

// EEPROM offsets for settings
#define EOFS_CURRENT_CAPACITY_L1 0 // 1 byte
#define EOFS_CURRENT_CAPACITY_L2 1 // 1 byte
#define EOFS_FLAGS               2 // 2 bytes

// EEPROM offsets for Delay Timer function - GoldServe
#define EOFS_TIMER_FLAGS         4 // 1 byte
#define EOFS_TIMER_START_HOUR    5 // 1 byte
#define EOFS_TIMER_START_MIN     6 // 1 byte
#define EOFS_TIMER_STOP_HOUR     7 // 1 byte
#define EOFS_TIMER_STOP_MIN      8 // 1 byte

// AMMETER stuff
#define EOFS_CURRENT_SCALE_FACTOR 9 // 2 bytes
#define EOFS_AMMETER_CURR_OFFSET  11 // 2 bytes
#define EOFS_KWH_ACCUMULATED 13 // 4 bytes

// fault counters
#define EOFS_GFI_TRIP_CNT      17 // 1 byte
#define EOFS_NOGND_TRIP_CNT    18 // 1 byte
#define EOFS_STUCK_RELAY_TRIP_CNT 19 // 1 byte

#define EOFS_VOLT_OFFSET 20 // 4 bytes
#define EOFS_VOLT_SCALE_FACTOR 24 // 2 bytes
#define EOFS_THRESH_AMBIENT 26 // 2 bytes
#define EOFS_THRESH_IR 28 // 2 bytes

// for I2C RAPI
#define EOFS_LOCAL_I2C_ADDR 30 // 1 byte
//
// for DUOSHARE
//
// for shared power pool
#define EOFS_GROUP_CURRENT_CAPACITY 31 // 1 byte
// non-volatile flags
#define EOFS_DUO_NVFLAGS 32 // 1 byte
#define EOFS_DUO_SHARED_AMPS 33 // 1 byte
//
// Reserved for HEARTBEAT_SUPERVISION (3 Bytes)
//
// Duration in seconds:
#define EOFS_HEARTBEAT_SUPERVISION_INTERVAL 34 // 2 bytes (zero if infinite)
// Fallback Current in quarter Amperes:
#define EOFS_HEARTBEAT_SUPERVISION_CURRENT 36 // 1 byte

#define EOFS_RELAY_CLOSE_MS 37 // 1 byte
#define EOFS_RELAY_HOLD_PWM 38 // 1 byte

#define EOFS_MAX_HW_CURRENT_CAPACITY 511 // 1 byte



// must stay within thresh for this time in ms before switching states
#define DELAY_STATE_TRANSITION 250
// must transition to state A from contacts closed in < 100ms according to spec
// but Leaf sometimes bounces from 3->1 so we will debounce it a little anyway
#define DELAY_STATE_TRANSITION_A 25

// for ADVPWR
#define GROUND_CHK_DELAY  1000 // delay after charging started to test, ms
#define STUCK_RELAY_DELAY 1000 // delay after charging opened to test, ms
#define RelaySettlingTime  250 // time for relay to settle in post, ms

// ACPINS sample interval - max number of ms to sample
// used only when ADVPWR - for rectified MID400 chips which block
// half cycle
#define AC_SAMPLE_MS 20 // 1 cycle @ 60Hz = 16.6667ms @ 50Hz = 20ms


// V6 has PD7 tied to ground
#define V6_ID_REG D
#define V6_ID_IDX 7

#ifdef GFI
#define GFI_INTERRUPT 0 // interrupt number 0 = PD2, 1 = PD3
// interrupt number 0 = PD2, 1 = PD3
#define GFI_REG &PIND
#define GFI_IDX 2

#ifdef GFI_SELFTEST
// pin is supposed to be wrapped around the GFI CT 5+ times
#define GFITEST_REG &PIND
#define GFITEST_IDX 6
// V6 GFI test pin PB0
#define V6_GFITEST_REG &PINB
#define V6_GFITEST_IDX 0



#define GFI_TEST_CYCLES 60
// GFI pulse should be 50% duty cycle
#define GFI_PULSE_ON_US 8333 // 1/2 of roughly 60 Hz.
#define GFI_PULSE_OFF_US 8334 // 1/2 of roughly 60 Hz.
#endif
#endif // GFI

#ifdef GFI_TESTING
#define GFI_TIMEOUT ((unsigned long)(15*1000))
#define GFI_RETRY_COUNT  255
#else // !GFI_TESTING
#define GFI_TIMEOUT ((unsigned long)(5*60000)) // 15*60*1000 doesn't work. go figure
// number of times to retry tests before giving up. 255 = retry indefinitely
#define GFI_RETRY_COUNT  6
#endif // GFI_TESTING

// for RGBLCD
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define BLUE 0x4
#define TEAL 0x6
#define VIOLET 0x5
#define WHITE 0x7

#if defined(RGBLCD) || defined(I2CLCD)
// Using LiquidTWI2 for both types of I2C LCD's
// see http://blog.lincomatic.com/?p=956 for installation instructions
#include "./Wire.h"
#ifdef I2CLCD_PCF8574
#include "./LiquidCrystal_I2C.h"
#define LCD_I2C_ADDR 0x27
#else
#ifdef RGBLCD
#define MCP23017 // Adafruit RGB LCD (PANELOLU2 is now supported without additional define)
#else
#define MCP23008 // Adafruit I2C Backpack
#endif
#include "./LiquidTWI2.h"
#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack
#endif // I2CLCD_PCF8574
#endif // RGBLCD || I2CLCD

// button sensing pin
#define BTN_REG &PINC
#define BTN_IDX 3
#define BTN_PRESS_SHORT 50  // ms
#define BTN_PRESS_LONG 500 // ms
#define BTN_PRESS_VERYLONG 10000


#ifdef RTC
// Default start/stop timers for un-initialized EEPROMs.
// Makes it easy to compile in default time without need to set it up the first time.
#define DEFAULT_START_HOUR    0x00 //Start time: 00:05
#define DEFAULT_START_MIN     0x05
#define DEFAULT_STOP_HOUR     0x06 //End time: 6:55
#define DEFAULT_STOP_MIN      0x37
#endif // RTC

// for J1772.ReadPilot()
// 1x = 114us 20x = 2.3ms 100x = 11.3ms
#define PILOT_LOOP_CNT 100

#ifdef AMMETER
// This multiplier is the number of milliamps per A/d converter unit.

// First, you need to select the burden resistor for the CT. You choose the largest value possible such that
// the maximum peak-to-peak voltage for the current range is 5 volts. To obtain this value, divide the maximum
// outlet current by the Te. That value is the maximum CT current RMS. You must convert that to P-P, so multiply
// by 2*sqrt(2). Divide 5 by that value and select the next lower standard resistor value. For the reference
// design, Te is 1018 and the outlet maximum is 30. 5/((30/1018)*2*sqrt(2)) = 59.995, so a 56 ohm resistor
// is called for. Call this value Rb (burden resistor).

// Next, one must use Te and Rb to determine the volts-per-amp value. Note that the readCurrent()
// method calculates the RMS value before the scaling factor, so RMS need not be taken into account.
// (1 / Te) * Rb = Rb / Te = Volts per Amp. For the reference design, that's 55.009 mV.
// Each count of the A/d converter is 4.882 mV (5/1024). V/A divided by V/unit is unit/A. For the reference
// design, that's 11.26. But we want milliamps per unit, so divide that into 1000 to get 88.7625558. Round near...
//#define DEFAULT_CURRENT_SCALE_FACTOR 106 // for RB = 47 - recommended for 30A max
//#define DEFAULT_CURRENT_SCALE_FACTOR 184 // for RB = 27 - recommended for 50A max
// Craig K, I arrived at 213 by scaling my previous multiplier of 225 down by the ratio of my panel meter reading of 28 with the OpenEVSE uncalibrated reading of 29.6
// then upped the scale factor to 220 after fixing the zero offset by subtracing 900ma
//#define DEFAULT_CURRENT_SCALE_FACTOR 220 // for RB = 22 - measured by Craig on his new OpenEVSE V3
// NOTE: setting DEFAULT_CURRENT_SCALE_FACTOR TO 0 will disable the ammeter
// until it is overridden via RAPI
//#define DEFAULT_CURRENT_SCALE_FACTOR 220   // Craig K, average of three OpenEVSE controller calibrations
#ifdef OPENEVSE_2
#define DEFAULT_CURRENT_SCALE_FACTOR 186   // OpenEVSE II with a 27 Ohm burden resistor, after a 2-point calibration at 12.5A and 50A
#else
#define DEFAULT_CURRENT_SCALE_FACTOR 220   // OpenEVSE v2.5 and v3 with a 22 Ohm burden resistor (note that the schematic may say 28 Ohms by mistake)
#endif

// subtract this from ammeter current reading to correct zero offset
#ifdef OPENEVSE_2
#define DEFAULT_AMMETER_CURRENT_OFFSET 230 // OpenEVSE II with a 27 Ohm burden resistor, after a 2-point calibration at 12.5A and 50A
#else
#define DEFAULT_AMMETER_CURRENT_OFFSET 0   // OpenEVSE v2.5 and v3 with a 22 Ohm burden resistor.  Could use a more thorough calibration exercise to nails this down.
#endif

// The maximum number of milliseconds to sample an ammeter pin in order to find three zero-crossings.
// one and a half cycles at 50 Hz is 30 ms.
#define CURRENT_SAMPLE_INTERVAL 35

// Once we detect a zero-crossing, we should not look for one for another quarter cycle or so. 1/4 // cycle at 50 Hz is 5 ms.
#define CURRENT_ZERO_DEBOUNCE_INTERVAL 5

#endif // AMMETER

#ifdef TEMPERATURE_MONITORING

#define MCP9808_IS_ON_I2C    // Use the MCP9808 connected to I2C
//#define TMP007_IS_ON_I2C     // Use the TMP007 IR sensor on I2C
#define TEMPERATURE_DISPLAY_ALWAYS 0     // Set this flag to 1 to always show temperatures on the bottom line of the 16X2 LCD
                                         // Set to it 0 to only display when temperatures become elevated
// #define TESTING_TEMPERATURE_OPERATION // Set this flag to play with very low sensor thresholds or to evaluate the code.
                                         // Leave it commented out instead to run with normal temperature thresholds.

// Temperature thresholds below are expressed as 520 meaning 52.0C to save from needing floating point library
// Keep any adjustments that you make at least 2C apart, giving things some hysterisis.  (example is 580 is 3C apart from 550)
// The RESTORE_AMPERAGE value must be lower than the THROTTLE_DOWN value
// The THROTTLE_DOWN value must be lower than the SHUTDOWN value
// The SHUTDOWN value must be lower than the PANIC value
#ifndef TESTING_TEMPERATURE_OPERATION
// normal oerational thresholds just below
// This is the temperature in the enclosure where we tell the car to draw 1/2 amperage.
#ifdef OPENEVSE_2
#define TEMPERATURE_AMBIENT_THROTTLE_DOWN 650
#else
#define TEMPERATURE_AMBIENT_THROTTLE_DOWN 650
#endif

// If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
// recover to this level we can kick the current back up to the user's original amperage setting.
#ifdef OPENEVSE_2
#define TEMPERATURE_AMBIENT_RESTORE_AMPERAGE 620
#else
#define TEMPERATURE_AMBIENT_RESTORE_AMPERAGE 620
#endif

// This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.
#ifdef OPENEVSE_2
#define TEMPERATURE_AMBIENT_SHUTDOWN 680
#else
#define TEMPERATURE_AMBIENT_SHUTDOWN 680
#endif

//  At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in
//  an over temperature error state.  The EVSE can be restart from the button or unplugged.
//  If temperatures get to this level it is advised to open the enclosure to look for trouble.
#ifdef OPENEVSE_2
#define TEMPERATURE_AMBIENT_PANIC 710
#else
#define TEMPERATURE_AMBIENT_PANIC 710
#endif

#define TEMPERATURE_INFRARED_THROTTLE_DOWN 650    // This is the temperature seen  by the IR sensor where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_INFRARED_RESTORE_AMPERAGE 600 // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_INFRARED_SHUTDOWN 700         // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.

#define TEMPERATURE_INFRARED_PANIC 750            //  At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in
                                                  //  an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  //  If temperatures get to this level it is advised to open the enclosure to look for trouble.
#else  //TESTING_TEMPERATURE_OPERATION

// Below are good values for testing purposes at room temperature with an EV simulator and no actual high current flowing
#define TEMPERATURE_AMBIENT_THROTTLE_DOWN 290     // This is the temperature in the enclosure where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_AMBIENT_RESTORE_AMPERAGE 270  // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_AMBIENT_SHUTDOWN 310          // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.

#define TEMPERATURE_AMBIENT_PANIC 330             //  At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in
                                                  //  an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  //  If temperatures get to this level it is advised to open the enclosure to look for trouble.

#define TEMPERATURE_INFRARED_THROTTLE_DOWN 330    // This is the temperature seen  by the IR sensor where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_INFRARED_RESTORE_AMPERAGE 270 // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_INFRARED_SHUTDOWN 360         // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.

#define TEMPERATURE_INFRARED_PANIC 400            // At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in
                                                  // an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  // If temperatures get to this level it is advised to open the enclosure to look for trouble.

#endif // TESTING_TEMPERATURE_OPERATION

#endif // TEMPERATURE_MONITORING

// how long to show each disabled test on LCD
#define SHOW_DISABLED_DELAY 1500

//-- end configuration

typedef union union4b {
  int8_t i8;
  uint8_t u8;
  int16_t i16;
  uint16_t u16;
  int32_t i32;
  uint32_t u32;
  unsigned u;
  int i;
} UNION4B,*PUNION4B;


//-- begin class definitions

#ifdef WATCHDOG
#define WDT_RESET() wdt_reset() // pat the dog
#define WDT_ENABLE() wdt_enable(WATCHDOG_TIMEOUT)
#else
#define WDT_RESET()
#define WDT_ENABLE()
#endif // WATCHDOG

// OnboardDisplay.m_bFlags
#define OBDF_MONO_BACKLIGHT 0x01
#define OBDF_AMMETER_DIRTY  0x80
#define OBDF_UPDATE_DISABLED 0x40

// OnboardDisplay::Update()
#define OBD_UPD_NORMAL    0
#define OBD_UPD_FORCE     1 // update even if no state transition
#define OBD_UPD_HARDFAULT 2 // update w/ hard fault
class OnboardDisplay
{
#ifdef RED_LED_REG
  DigitalPin pinRedLed;
#endif
#ifdef GREEN_LED_REG
  DigitalPin pinGreenLed;
#endif
#if defined(RGBLCD) || defined(I2CLCD)
#ifdef I2CLCD_PCF8574
  LiquidCrystal_I2C m_Lcd;
#else
  LiquidTWI2 m_Lcd;
#endif // I2CLCD_PCF8574
#endif // defined(RGBLCD) || defined(I2CLCD)
  uint8_t m_bFlags;
  char m_strBuf[LCD_MAX_CHARS_PER_LINE+1];
  unsigned long m_LastUpdateMs;

  int8_t updateDisabled() { return  m_bFlags & OBDF_UPDATE_DISABLED; }

  void MakeChar(uint8_t n, PGM_P bytes);
public:
  OnboardDisplay();
  void Init();

  void SetGreenLed(uint8_t state) {
#ifdef GREEN_LED_REG
    pinGreenLed.write(state);
#endif
  }

  void SetRedLed(uint8_t state) {
#ifdef RED_LED_REG
  pinRedLed.write(state);
#endif
  }
#ifdef LCD16X2
  void LcdBegin(int x,int y) {
#ifdef I2CLCD
#ifndef I2CLCD_PCF8574
    m_Lcd.setMCPType(LTI_TYPE_MCP23008);
#endif
    m_Lcd.begin(x,y);
    m_Lcd.setBacklight(HIGH);
#elif defined(RGBLCD)
    m_Lcd.setMCPType(LTI_TYPE_MCP23017);
    m_Lcd.begin(x,y,2);
    m_Lcd.setBacklight(WHITE);
#endif // I2CLCD
  }
  void LcdPrint(const char *s) {
    m_Lcd.print(s);
  }
  void LcdPrint_P(PGM_P s);
  void LcdPrint(int y,const char *s);
  void LcdPrint_P(int y,PGM_P s);
  void LcdPrint(int x,int y,const char *s);
  void LcdPrint_P(int x,int y,PGM_P s);
  void LcdPrint(int i) {
    m_Lcd.print(i);
  }
  void LcdSetCursor(int x,int y) {
    m_Lcd.setCursor(x,y);
  }
  void LcdClearLine(int y) {
    m_Lcd.setCursor(0,y);
    for (uint8_t i=0;i < LCD_MAX_CHARS_PER_LINE;i++) {
      m_Lcd.write(' ');
    }
    m_Lcd.setCursor(0,y);
  }
  void LcdClear() {
    m_Lcd.clear();
  }
  void LcdWrite(uint8_t data) {
    m_Lcd.write(data);
  }
  void LcdMsg(const char *l1,const char *l2);
  void LcdMsg_P(PGM_P l1,PGM_P l2);
  void LcdSetBacklightType(uint8_t t,uint8_t update=OBD_UPD_FORCE) { // BKL_TYPE_XXX
#ifdef RGBLCD
    if (t == BKL_TYPE_RGB) m_bFlags &= ~OBDF_MONO_BACKLIGHT;
    else m_bFlags |= OBDF_MONO_BACKLIGHT;
    Update(update);
#endif // RGBLCD
  }
  uint8_t IsLcdBacklightMono() {
#ifdef RGBLCD
    return (m_bFlags & OBDF_MONO_BACKLIGHT) ? 1 : 0;
#else
    return 1;
#endif // RGBLCD
  }
  void LcdSetBacklightColor(uint8_t c) {
#ifdef RGBLCD
    if (IsLcdBacklightMono()) {
      if (c) c = WHITE;
    }
    m_Lcd.setBacklight(c);
#endif // RGBLCD
  }
#ifdef RGBLCD
  uint8_t readButtons() { return m_Lcd.readButtons(); }
#endif // RGBLCD
#endif // LCD16X2

#ifdef AMMETER
  void SetAmmeterDirty(uint8_t tf) {
    if (tf) m_bFlags |= OBDF_AMMETER_DIRTY;
    else m_bFlags &= ~OBDF_AMMETER_DIRTY;
  }
  int8_t AmmeterIsDirty() { return (m_bFlags & OBDF_AMMETER_DIRTY) ? 1 : 0; }
#endif // AMMETER

  void DisableUpdate(int8_t on) {
    if (on) m_bFlags |= OBDF_UPDATE_DISABLED;
    else m_bFlags &= ~OBDF_UPDATE_DISABLED;
  }
  int8_t UpdatesDisabled() { return (m_bFlags & OBDF_UPDATE_DISABLED) ? 1 : 0; }
  void Update(int8_t updmode=OBD_UPD_NORMAL); // OBD_UPD_xxx
};

#ifdef GFI
#include "Gfi.h"
#endif // GFI

#ifdef TEMPERATURE_MONITORING
#include "./MCP9808.h"  //  adding the ambient temp sensor to I2C
#include "./Adafruit_TMP007.h"   //  adding the TMP007 IR I2C sensor


#define TEMPERATURE_NOT_INSTALLED -2560 // fake temp to return when hardware not installed
#define TEMPMONITOR_UPDATE_INTERVAL 1000ul
// TempMonitor.m_Flags
#define TMF_OVERTEMPERATURE          0x01
#define TMF_OVERTEMPERATURE_SHUTDOWN 0x02
#define TMF_BLINK_ALARM              0x04
#define TMF_OVERTEMPERATURE_LOGGED   0x08
class TempMonitor {
  uint8_t m_Flags;
  unsigned long m_LastUpdate;
public:
#ifdef MCP9808_IS_ON_I2C
  MCP9808 m_tempSensor;
#endif  //MCP9808_IS_ON_I2C
#ifdef TMP007_IS_ON_I2C
  Adafruit_TMP007 m_tmp007;
#endif  //TMP007_IS_ON_I2C
  // these three temperatures need to be signed integers
  int16_t m_MCP9808_temperature;  // 230 means 23.0C  Using an integer to save on floating point library use
  int16_t m_DS3231_temperature;   // the DS3231 RTC has a built in temperature sensor
  int16_t m_TMP007_temperature;

  TempMonitor() {}
  void Init();
  void Read();

  void SetBlinkAlarm(int8_t tf) {
    if (tf) m_Flags |= TMF_BLINK_ALARM;
    else m_Flags &= ~TMF_BLINK_ALARM;
  }
  int8_t BlinkAlarm() { return (m_Flags & TMF_BLINK_ALARM) ? 1 : 0; }
  void SetOverTemperature(int8_t tf) {
    if (tf) m_Flags |= (TMF_OVERTEMPERATURE|TMF_OVERTEMPERATURE_LOGGED);
    else m_Flags &= ~TMF_OVERTEMPERATURE;
  }
  int8_t OverTemperature() { return (m_Flags & TMF_OVERTEMPERATURE) ? 1 : 0; }
  void SetOverTemperatureShutdown(int8_t tf) {
    if (tf) m_Flags |= (TMF_OVERTEMPERATURE_SHUTDOWN|TMF_OVERTEMPERATURE_LOGGED);
    else m_Flags &= ~TMF_OVERTEMPERATURE_SHUTDOWN;
  }
  int8_t OverTemperatureShutdown() { return (m_Flags & TMF_OVERTEMPERATURE_SHUTDOWN) ? 1 : 0; }
  uint8_t OverTemperatureLogged() { return (m_Flags & TMF_OVERTEMPERATURE_LOGGED) ? 1 : 0; }
  void ClrOverTemperatureLogged() { m_Flags &= ~TMF_OVERTEMPERATURE_LOGGED; }
#ifdef TEMPERATURE_MONITORING_NY
  void LoadThresh();
  void SaveThresh();
#endif //TEMPERATURE_MONITORING_NY
};
#endif // TEMPERATURE_MONITORING

#include "J1772Pilot.h"
#include "J1772EvseController.h"

#ifdef BTN_MENU
#define BTN_STATE_OFF   0
#define BTN_STATE_SHORT 1 // short press
#define BTN_STATE_LONG  2 // long press
class Btn {
#ifdef BTN_REG
  DigitalPin pinBtn;
#endif
  uint8_t buttonState;
  unsigned long lastDebounceTime;  // the last time the output pin was toggled
  unsigned long vlongDebounceTime;  // for verylong press

public:
  Btn();
  void init();

  void read();
  uint8_t shortPress();
  uint8_t longPress();
};


class Menu {
public:
  PGM_P m_Title;
  uint8_t m_CurIdx;

  void init(const char *firstitem);

  Menu();

  virtual void Init() = 0;
  virtual void Next() = 0;
  virtual Menu *Select() = 0;
};

class SettingsMenu : public Menu {
  uint8_t m_menuCnt;
#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
  uint8_t m_skipLimits;
#endif
public:
  SettingsMenu();
  void Init();
  void Next();
  Menu *Select();
#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
  void CheckSkipLimits();
#endif
};

class SetupMenu : public Menu {
  uint8_t m_menuCnt;
public:
  SetupMenu();
  void Init();
  void Next();
  Menu *Select();
};

class SvcLevelMenu : public Menu {
public:
  SvcLevelMenu();
  void Init();
  void Next();
  Menu *Select();
};

class MaxCurrentMenu  : public Menu {
  uint8_t m_MinCurrent;
  uint8_t m_MaxCurrent;
public:
  MaxCurrentMenu();
  void Init();
  void Next();
  Menu *Select();
};

class DiodeChkMenu : public Menu {
public:
  DiodeChkMenu();
  void Init();
  void Next();
  Menu *Select();
};

#ifdef GFI_SELFTEST
class GfiTestMenu : public Menu {
public:
  GfiTestMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif

#ifdef TEMPERATURE_MONITORING
class TempOnOffMenu : public Menu {
public:
  TempOnOffMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif  // TEMPERATURE_MONITORING

class VentReqMenu : public Menu {
public:
  VentReqMenu();
  void Init();
  void Next();
  Menu *Select();
};


#ifdef ADVPWR
class GndChkMenu : public Menu {
public:
  GndChkMenu();
  void Init();
  void Next();
  Menu *Select();
};

class RlyChkMenu : public Menu {
public:
  RlyChkMenu();
  void Init();
  void Next();
  Menu *Select();
};

#endif // ADVPWR
class ResetMenu : public Menu {
public:
  ResetMenu();
  void Init();
  void Next();
  Menu *Select();
};

#ifdef RGBLCD
class BklTypeMenu : public Menu {
public:
  BklTypeMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif // RGBLCD

#if defined(DELAYTIMER)
class RTCMenu : public Menu {
public:
  RTCMenu();
  void Init();
  void Next();
  Menu *Select();
};

class RTCMenuMonth : public Menu {
public:
  RTCMenuMonth();
  void Init();
  void Next();
  Menu *Select();
};

class RTCMenuDay : public Menu {
public:
  RTCMenuDay();
  void Init();
  void Next();
  Menu *Select();
};
class RTCMenuYear : public Menu {
public:
  RTCMenuYear();
  void Init();
  void Next();
  Menu *Select();
};
class RTCMenuHour : public Menu {
public:
  RTCMenuHour();
  void Init();
  void Next();
  Menu *Select();
};
class RTCMenuMinute : public Menu {
public:
  RTCMenuMinute();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenu : public Menu {
public:
  DelayMenu();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuEnableDisable : public Menu {
public:
  DelayMenuEnableDisable();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStartHour : public Menu {
public:
  DelayMenuStartHour();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStartMin : public Menu {
public:
  DelayMenuStartMin();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStopHour : public Menu {
public:
  DelayMenuStopHour();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStopMin : public Menu {
public:
  DelayMenuStopMin();
  void Init();
  void Next();
  Menu *Select();
};
#endif // DELAYTIMER

#ifdef CHARGE_LIMIT
class ChargeLimitMenu  : public Menu {
  void showCurSel(uint8_t plus=0);
public:
  ChargeLimitMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif // CHARGE_LIMIT

#ifdef TIME_LIMIT
class TimeLimitMenu  : public Menu {
  void showCurSel(uint8_t plus=0);
public:
  TimeLimitMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif // TIME_LIMIT

class BtnHandler {
  Btn m_Btn;
  Menu *m_CurMenu;
  uint8_t m_SavedLcdMode;

public:
  BtnHandler();
  void init() { m_Btn.init(); }
  void ChkBtn();
  uint8_t GetSavedLcdMode() { return m_SavedLcdMode; }
  void SetSavedLcdMode(uint8_t mode ) { m_SavedLcdMode = mode; }
  int8_t DoShortPress(int8_t infaultstate);
};

#endif // BTN_MENU

#ifdef DELAYTIMER
// Start Delay Timer class definition - GoldServe
class DelayTimer {
  uint8_t m_DelayTimerEnabled;
  uint8_t m_StartTimerHour;
  uint8_t m_StartTimerMin;
  uint8_t m_StopTimerHour;
  uint8_t m_StopTimerMin;
  uint8_t m_ManualOverride;
  unsigned long m_LastCheck;
public:
  DelayTimer(){
    m_LastCheck = - (60ul * 1000ul);
  };
  void Init();
  void CheckTime();
  void Enable();
  void Disable();

  void SetManualOverride() {
    if (IsTimerEnabled()) {
      m_ManualOverride = 1;
    }
  }
  void ClrManualOverride() { m_ManualOverride = 0; }
  uint8_t ManualOverrideIsSet() { return m_ManualOverride; }

  uint8_t IsTimerEnabled(){
    return m_DelayTimerEnabled;
  };
  uint8_t GetStartTimerHour(){
    return m_StartTimerHour;
  };
  uint8_t GetStartTimerMin(){
    return m_StartTimerMin;
  };
  uint8_t GetStopTimerHour(){
    return m_StopTimerHour;
  };
  uint8_t GetStopTimerMin(){
    return m_StopTimerMin;
  };
  void SetStartTimer(uint8_t hour, uint8_t min){
    m_StartTimerHour = hour;
    m_StartTimerMin = min;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_HOUR, m_StartTimerHour);
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_MIN, m_StartTimerMin);
    //    g_EvseController.SaveSettings();
  };
  void SetStopTimer(uint8_t hour, uint8_t min){
    m_StopTimerHour = hour;
    m_StopTimerMin = min;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_MIN, m_StopTimerMin);
    //    g_EvseController.SaveSettings();
  };
  uint8_t IsInAwakeTimeInterval(); //
  uint8_t IsTimerValid(){
     if (m_StartTimerHour || m_StartTimerMin || m_StopTimerHour || m_StopTimerMin){ // Check not all equal 0
       if ((m_StartTimerHour == m_StopTimerHour) && (m_StartTimerMin == m_StopTimerMin)){ // Check start time not equal to stop time
         return 0;
       } else {
         return 1;
       }
     } else {
       return 0;
     }
  };
  uint8_t IsTimerOn();

  void PrintTimerIcon();
};

#endif //ifdef DELAYTIMER

// -- end class definitions

char *u2a(unsigned long x,int8_t digits=0);
void ProcessInputs();

/*
extern char g_sPlus[];
extern char g_sSlash[];
extern char g_sColon[];
extern char g_sSpace[];
*/
#define g_sPlus "+"
#define g_sSlash "/"
#define g_sColon ":"
#define g_sSpace " "


#ifdef DELAYTIMER
extern DelayTimer g_DelayTimer;
#endif
#ifdef BTN_MENU
extern BtnHandler g_BtnHandler;
extern SettingsMenu g_SettingsMenu;
#endif // BTN_MENU
extern OnboardDisplay g_OBD;
extern char g_sTmp[TMP_BUF_SIZE];

#ifdef KWH_RECORDING
extern unsigned long g_WattHours_accumulated;
extern unsigned long g_WattSeconds;
#endif // KWH_RECORDING
#ifdef TEMPERATURE_MONITORING
extern TempMonitor g_TempMonitor;
#endif // TEMPERATURE_MONITORING

char *GetFirmwareVersion(char *str);
void wdt_delay(uint32_t ms);


#include "strings.h"
#include "rapi_proc.h"
