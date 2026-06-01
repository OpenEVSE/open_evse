// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2011-2023 Sam C. Lin
 * Copyright (c) 2011-2014 Chris Howell <chris1howell@msn.com>
 * timer code Copyright (c) 2013 Kevin L <goldserve1@hotmail.com>
 * portions Copyright (c) 2014-2015 Nick Sayer <nsayer@kfu.com>
 * portions Copyright (c) 2015 Craig Kirkpatrick
 * portions Copyright (c) 2015 William McBrine
 * portions Copyright (c) 2019 Tim Kuechler BowElectric at gmail

 Revised  Ver	By		Reason
 6/21/13  20b3	Scott Rubin	fixed LCD display bugs with RTC enabled
 6/25/13  20b4	Scott Rubin	fixed LCD display bugs, CLI fixes, when RTC disabled
 6/30/13  20b5	Scott Rubin	added LcdDetected() function, prevents hang if LCD not installed
 7/06/13  20b5	Scott Rubin	rewrote power detection in POST function for 1 or 2 relays
 7/11/13  20b5	Scott Rubin	skips POST if EV is connected, won't charge if open ground or stuck relay
 8/12/13  20b5b Scott Rubin    fix GFI error - changed gfi.Reset() to check for constant GFI signal
 8/26/13  20b6 Scott Rubin     add Stuck Relay State delay, fix Stuck Relay state exit (for Active E)
 9/20/13  20b7 Chris Howell    updated/tweaked/shortened CLI messages   
 10/25/14      Craig K         add smoothing to the Amperage readout
 3/1/15        Craig K         add TEMPERATURE_MONITORING
 3/7/15        Craig K         add KWH_RECORDING
 10/28/2019    Tim Kuechler    add HEARTBEAT_SUPERVISION
  
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
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include "./Wire.h"
#include "./RTClib.h"
#include "open_evse.h"

// if using I2CLCD_PCF8574 uncomment below line  and comment out LiquidTWI2.h above
//#include "./LiquidCrystal_I2C.h"
#ifdef TEMPERATURE_MONITORING
  #ifdef MCP9808_IS_ON_I2C
  #include "MCP9808.h"  //  adding the ambient temp sensor to I2C
  #endif 
  #ifdef TMP007_IS_ON_I2C
  #include "./Adafruit_TMP007.h"   //  adding the TMP007 IR I2C sensor
  #endif 
#endif // TEMPERATURE_MONITORING


#ifdef BTN_MENU
SettingsMenu g_SettingsMenu;
SetupMenu g_SetupMenu;
MaxCurrentMenu g_MaxCurrentMenu;
#ifndef NOSETUP_MENU
SvcLevelMenu g_SvcLevelMenu;
DiodeChkMenu g_DiodeChkMenu;
#ifdef RGBLCD
BklTypeMenu g_BklTypeMenu;
#endif // RGBLCD
#ifdef GFI_SELFTEST
GfiTestMenu g_GfiTestMenu;
#endif
#ifdef TEMPERATURE_MONITORING
TempOnOffMenu g_TempOnOffMenu;
#endif // TEMPERATURE_MONITORING
VentReqMenu g_VentReqMenu;
#ifdef ADVPWR
GndChkMenu g_GndChkMenu;
RlyChkMenu g_RlyChkMenu;
#endif // ADVPWR
#endif // NOSETUP_MENU
ResetMenu g_ResetMenu;
// Instantiate additional Menus - GoldServe
#if defined(DELAYTIMER_MENU)
RTCMenu g_RTCMenu;
RTCMenuMonth g_RTCMenuMonth;
RTCMenuDay g_RTCMenuDay;
RTCMenuYear g_RTCMenuYear;
RTCMenuHour g_RTCMenuHour;
RTCMenuMinute g_RTCMenuMinute;
DelayMenu g_DelayMenu;
DelayMenuEnableDisable g_DelayMenuEnableDisable;
DelayMenuStartHour g_DelayMenuStartHour;
DelayMenuStopHour g_DelayMenuStopHour;
DelayMenuStartMin g_DelayMenuStartMin;
DelayMenuStopMin g_DelayMenuStopMin;
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
ChargeLimitMenu g_ChargeLimitMenu;
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
TimeLimitMenu g_TimeLimitMenu;
#endif // TIME_LIMIT


Menu *g_SettingsMenuList[] = {
#ifdef TIME_LIMIT
  &g_TimeLimitMenu,
#endif // TIME_LIMIT
#ifdef CHARGE_LIMIT
  &g_ChargeLimitMenu,
#endif // CHARGE_LIMIT
#ifdef DELAYTIMER_MENU
  &g_DelayMenu,
#endif // DELAYTIMER_MENU
  &g_SetupMenu,
  &g_ResetMenu,
  NULL
};

Menu *g_SetupMenuList[] = {
#ifdef NOSETUP_MENU
  &g_MaxCurrentMenu,
  &g_RTCMenu,
#else // !NOSETUP_MENU
#ifdef DELAYTIMER_MENU
  &g_RTCMenu,
#endif // DELAYTIMER_MENU
#ifdef RGBLCD
  &g_BklTypeMenu,
#endif // RGBLCD
  &g_SvcLevelMenu,
  &g_DiodeChkMenu,
  &g_VentReqMenu,
#ifdef ADVPWR
  &g_GndChkMenu,
  &g_RlyChkMenu,
#endif // ADVPWR
#ifdef GFI_SELFTEST
  &g_GfiTestMenu,
#endif // GFI_SELFTEST
#ifdef TEMPERATURE_MONITORING
  &g_TempOnOffMenu,
#endif // TEMPERATURE_MONITORING
#endif // NOSETUP_MENU
  NULL
};

BtnHandler g_BtnHandler;
#endif // BTN_MENU

#define g_sHHMMfmt "%02d:%02d"

//-- begin global variables

char g_sTmp[TMP_BUF_SIZE];

OnboardDisplay g_OBD;

// Instantiate RTC and Delay Timer - GoldServe
#ifdef RTC
RTC_DS1307 g_RTC;

#if defined(RAPI)
void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s) {
  g_RTC.adjust(DateTime(y,m,d,h,mn,s));
}
void GetRTC(char *buf) {
  DateTime t = g_RTC.now();
  sprintf(buf,"%d %d %d %d %d %d",t.year()-2000,t.month(),t.day(),t.hour(),t.minute(),t.second());
}
#endif // RAPI
#endif // RTC
#ifdef DELAYTIMER
DelayTimer g_DelayTimer;
#ifdef DELAYTIMER_MENU
// Start variables to support RTC and Delay Timer - GoldServe
uint16_t g_year;
uint8_t g_month;
uint8_t g_day;
uint8_t g_hour;
uint8_t g_min;
#endif // DELAYTIMER_MENU
#endif // DELAYTIMER


#ifdef TEMPERATURE_MONITORING
TempMonitor g_TempMonitor;
#endif // TEMPERATURE_MONITORING

#ifdef PP_AUTO_AMPACITY
AutoCurrentCapacityController g_ACCController;
#endif

unsigned long OnboardDisplay::lastBacklightChangeMillis = 0;



//-- end global variables

// watchdog-safe delay - use this when delay is longer than watchdog
// *do not* call this before WDT_ENABLE() is called
void wdt_delay(uint32_t ms)
{
  do {
    WDT_RESET();
    if (ms > WATCHDOG_TIMEOUT/2) {
      delay(WATCHDOG_TIMEOUT/2);
      ms -= WATCHDOG_TIMEOUT/2;
    }
    else {
      delay(ms);
      ms = 0;
    }
  } while(ms);
}

static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

// if digits > 0, zero pad w/ to # digits
// WARNING: This function uses the *end* of g_sTmp as its buffer
char *u2a(unsigned long x,int8_t digits)
{
  char *s = g_sTmp + sizeof(g_sTmp);

  *--s = 0;

  do {
    *--s = '0'+ x % 10;
    x /= 10;
    --digits;
  } while (x || digits > 0);

  return s;
}

// wdt_init turns off the watchdog timer after we use it
// to reboot

void wdt_init(void) __attribute__((naked,used)) __attribute__((section(".init3")));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();

  return;
}


#ifdef TEMPERATURE_MONITORING

void TempMonitor::Init()
{
  m_Flags = 0;
  m_MCP9808_temperature = TEMPERATURE_NOT_INSTALLED;  // 230 means 23.0C  Using an integer to save on floating point library use
  m_DS3231_temperature = TEMPERATURE_NOT_INSTALLED;   // the DS3231 RTC has a built in temperature sensor
  m_TMP007_temperature = TEMPERATURE_NOT_INSTALLED;

#ifdef TEMPERATURE_MONITORING_NY
  LoadThresh();
#endif

#ifdef MCP9808_IS_ON_I2C
  m_tempSensor.begin();
#endif // MCP9808_IS_ON_I2C

#ifdef TMP007_IS_ON_I2C
  m_tmp007.begin();
#endif // TMP007_IS_ON_I2C
}

void TempMonitor::Read()
{
  unsigned long curms = millis();
  if ((curms - m_LastUpdate) >= TEMPMONITOR_UPDATE_INTERVAL) {
#ifdef TMP007_IS_ON_I2C
    m_TMP007_temperature = m_tmp007.readObjTempC10();   //  using the TI TMP007 IR sensor
#endif
#ifdef MCP9808_IS_ON_I2C
    m_MCP9808_temperature = m_tempSensor.readAmbient();  // for the MCP9808
#endif

       
#ifdef RTC
#ifdef OPENEVSE_2
    m_DS3231_temperature = TEMPERATURE_NOT_INSTALLED;  // OpenEVSE II does not use the DS3231
#else // !OPENEVSE_2
    // This code chunk below reads the DS3231 RTC's internal temperature sensor            
    Wire.beginTransmission(DS1307_ADDRESS);
    wiresend(uint8_t(0x0e));
    wiresend( 0x20 );               // write bit 5 to initiate conversion of temperature
    Wire.endTransmission();            
             
    Wire.beginTransmission(DS1307_ADDRESS);
    wiresend(uint8_t(0x11));
    Wire.endTransmission();
	  
    if(Wire.requestFrom(DS1307_ADDRESS, 2)) {                           // detect presence of DS3231 on I2C while addressing to read from it
    m_DS3231_temperature = (((int16_t)wirerecv()) << 8) | (wirerecv()); // read upper and lower byte
    m_DS3231_temperature = m_DS3231_temperature >> 6;                   // lower 6 bits always zero, ignore them
    if (m_DS3231_temperature & 0x0200) m_DS3231_temperature |= 0xFE00;  // sign extend if a negative number since we shifted over by 6 bits
    m_DS3231_temperature = (m_DS3231_temperature * 10) / 4;             // handle this as 0.25C resolution
                                                                        // Note that the device's sign bit only pertains to the device's upper byte.
                                                                        // The small side effect is that -0.25, -0.5, and -0.75C actual temperatues
                                                                        // will be read from the device without negative sign, thus appearing as +0.25C...
                                                                        // There is no software workaround to this small hardware shortcoming.
                                                                        // Temperatures outside of these values work perfectly with 1/4 degree resolution.
                                                                        // I wrote this note so nobody wastes time trying to "fix" this in software
                                                                        // since fundamentally it is a hardware limitaion of the DS3231.
      }                                                                    
    else                                                                    
      m_DS3231_temperature = TEMPERATURE_NOT_INSTALLED;
    
#endif // OPENEVSE_2
#endif // RTC

    m_LastUpdate = curms;
  }
}
#endif // TEMPERATURE_MONITORING


OnboardDisplay::OnboardDisplay()
#if defined(I2CLCD) || defined(RGBLCD)
#ifdef I2CLCD_PCF8574
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
  : m_Lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE)
#else
  : m_Lcd(LCD_I2C_ADDR,1)
#endif // I2CLCD_PCF8574
#endif // defined(I2CLCD) || defined(RGBLCD)
{
}


#if defined(DELAYTIMER)
const char CustomChar_6[8] PROGMEM = {0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0}; // clock
#endif
#ifdef DELAYTIMER
const char CustomChar_1[8] PROGMEM = {0x0,0x0,0xe,0xe,0xe,0x0,0x0,0x0}; // stop (cube)
const char CustomChar_2[8] PROGMEM = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0}; // play 
#endif // DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
const char CustomChar_3[8] PROGMEM = {0x0,0xe,0xc,0x1f,0x3,0x6,0xc,0x8}; // lightning
#endif
#ifdef AUTH_LOCK
const char CustomChar_4[8] PROGMEM = { // padlock
	0b00000,
	0b01110,
	0b01010,
	0b11111,
	0b11011,
	0b11011,
	0b01110,
	0b00000
};
#endif // AUTH_LOCK
#ifdef TIME_LIMIT
const char CustomChar_5[8] PROGMEM = { // time limit clock
	0b00000,
	0b01110,
	0b10001,
	0b11101,
	0b10101,
	0b01110,
	0b00000,
	0b00000
};
#endif // TIME_LIMIT

#ifdef LCD16X2
void OnboardDisplay::MakeChar(uint8_t n, PGM_P bytes)
{
  memcpy_P(g_sTmp, bytes, 8);
  m_Lcd.createChar(n, (uint8_t*)g_sTmp);
}
#endif // LCD16X2

void OnboardDisplay::Init()
{
  WDT_RESET();
#ifdef RGBLCD
  m_bFlags = 0;
#else
  m_bFlags = OBDF_MONO_BACKLIGHT;
#endif // RGBLCD

#ifdef GREEN_LED_REG
  pinGreenLed.init(GREEN_LED_REG,GREEN_LED_IDX,DigitalPin::OUT);
  SetGreenLed(0);
#endif
#ifdef RED_LED_REG
  pinRedLed.init(RED_LED_REG,RED_LED_IDX,DigitalPin::OUT);
  SetRedLed(0);
#endif

#ifdef LCD16X2
  LcdBegin(LCD_MAX_CHARS_PER_LINE, 2);
  LcdSetBacklightColor(WHITE);

#if defined(DELAYTIMER)
  MakeChar(0,CustomChar_6);
#endif
#ifdef DELAYTIMER
  MakeChar(1,CustomChar_1);
  MakeChar(2,CustomChar_2);
#endif //#ifdef DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
  MakeChar(3,CustomChar_3);
#endif
#ifdef AUTH_LOCK
  MakeChar(4,CustomChar_4);
#endif
#ifdef TIME_LIMIT
  MakeChar(5,CustomChar_5);
#endif // TIME_LIMIT
  m_Lcd.clear();

#ifdef OPENEVSE_2
  LcdPrint_P(0,PSTR("Open EVSE II"));
#else
  LcdPrint_P(0,PSTR("Open EVSE"));
#endif
  LcdPrint_P(0,1,PSTR("Ver. "));
  LcdPrint_P(VERSTR);
  wdt_delay(1500);
  WDT_RESET();
#endif //#ifdef LCD16X2
}

#ifdef LCD16X2
void OnboardDisplay::LcdPrint(int x,int y,const char *s)
{ 
  m_Lcd.setCursor(x,y);
  m_Lcd.print(s); 
}

void OnboardDisplay::LcdPrint_P(PGM_P s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  m_Lcd.print(m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int y,PGM_P s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  LcdPrint(y,m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int x,int y,PGM_P s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  m_Lcd.setCursor(x,y);
  m_Lcd.print(m_strBuf);
}

void OnboardDisplay::LcdMsg_P(PGM_P l1,PGM_P l2)
{
  LcdPrint_P(0,l1);
  LcdPrint_P(1,l2);
}


// print at (0,y), filling out the line with trailing spaces
void OnboardDisplay::LcdPrint(int y,const char *s)
{
  m_Lcd.setCursor(0,y);
  uint8_t i,len = strlen(s);
  if (len > LCD_MAX_CHARS_PER_LINE)
    len = LCD_MAX_CHARS_PER_LINE;
  for (i=0;i < len;i++) {
    m_Lcd.write(s[i]);
  }
  for (i=len;i < LCD_MAX_CHARS_PER_LINE;i++) {
    m_Lcd.write(' ');
  }
}

void OnboardDisplay::LcdMsg(const char *l1,const char *l2)
{
  LcdPrint(0,l1);
  LcdPrint(1,l2);
}
#endif // LCD16X2


void OnboardDisplay::Update(int8_t updmode)
{
  if (updateDisabled() && !g_EvseController.InFaultState()) return;

  uint8_t curstate = g_EvseController.GetState();
#ifdef LCD16X2
  uint8_t svclvl = g_EvseController.GetCurSvcLevel();
  int currentcap = g_EvseController.GetCurrentCapacity();
#endif
  unsigned long curms = millis();

  if (g_EvseController.StateTransition() || (updmode != OBD_UPD_NORMAL)) {
    curms += 1000; // trigger periodic update code below

    if (g_EvseController.InHardFault()) {
      // need this in case we're called outside of J1772EvseController.HardFault()
      // during a hard fault
      updmode = OBD_UPD_HARDFAULT;
    }

#ifdef LCD16X2
    sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,currentcap);
#endif
    switch(curstate) {
    case EVSE_STATE_A: // not connected
      SetGreenLed(1);
      SetRedLed(0);
#ifdef LCD16X2
      // Display Timer and Stop Icon - GoldServe
      LcdClear();
      LcdSetCursor(0,0);
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {
	LcdSetBacklightColor(TEAL);
	LcdWrite(4); 
      }
      else {
	LcdSetBacklightColor(GREEN);
      }
#else
      LcdSetBacklightColor(GREEN);
#endif // AUTH_LOCK
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psReady);
      LcdPrint(10,0,g_sTmp);
      
#ifdef KWH_RECORDING 
      sprintf(g_sTmp,STRF_WH,(g_EnergyMeter.GetSessionWs() / 3600) );
      LcdPrint(0,1,g_sTmp);
      
      sprintf(g_sTmp,STRF_KWH,(g_EnergyMeter.GetTotkWh() / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
#endif // KWH_RECORDING
      
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_B: // connected/not charging
      SetGreenLed(1);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdClear();
      LcdSetCursor(0,0);
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {
	LcdWrite(4); 
	LcdSetBacklightColor(TEAL);
      }
      else {
	LcdSetBacklightColor(YELLOW);
      }
#else
      LcdSetBacklightColor(YELLOW);
#endif // AUTH_LOCK
#ifdef CHARGE_LIMIT
      if (g_EvseController.GetChargeLimitkWh()) {
	LcdWrite(3); // lightning
      }
#endif
#ifdef TIME_LIMIT
      if (g_EvseController.GetTimeLimit15()) {
	LcdWrite(5); // time limit clock
      }
#endif
      // Display Timer and Stop Icon - GoldServe
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psEvConnected);
      LcdPrint(10,0,g_sTmp);
      
#ifdef KWH_RECORDING
      sprintf(g_sTmp,STRF_WH,(g_EnergyMeter.GetSessionWs() / 3600) );
      LcdPrint(0,1,g_sTmp);
      
      sprintf(g_sTmp,STRF_KWH,(g_EnergyMeter.GetTotkWh() / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
#endif // KWH_RECORDING
      
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_C: // charging
      SetGreenLed(0);
      SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(TEAL);
      LcdClear();
      LcdSetCursor(0,0);
      // Display Timer and Stop Icon - GoldServe
#ifdef CHARGE_LIMIT
      if (g_EvseController.GetChargeLimitTotWs()) {
	LcdWrite(3); // lightning
      }
#endif
#ifdef TIME_LIMIT
      if (g_EvseController.GetTimeLimit15()) {
	LcdWrite(5); // clock
      }
#endif
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psCharging);
#endif //Adafruit RGB LCD
      // n.b. blue LED is on
#ifdef AMMETER
      SetAmmeterDirty(1); // force ammeter update code below
#endif // AMMETER
      break;
    case EVSE_STATE_D: // vent required
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psVentReq);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DIODE_CHK_FAILED:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psDiodeChkFailed);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_GFCI_FAULT:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      if (updmode == OBD_UPD_HARDFAULT) {
        LcdMsg_P(g_psSvcReq,g_psGfciFault);
      }
      else {
	// 2nd line will be updated below with auto retry count
        LcdPrint_P(0,g_psGfciFault);
      }
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
#ifdef TEMPERATURE_MONITORING      
    case EVSE_STATE_OVER_TEMPERATURE:    // overtemp message in Red on the RGB LCD
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psSvcReq,g_psTemperatureFault);  //  SERVICE REQUIRED     OVER TEMPERATURE 
#endif
      break;
#endif //TEMPERATURE_MONITORING        
#ifdef OVERCURRENT_THRESHOLD
    case EVSE_STATE_OVER_CURRENT:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdPrint_P(0,g_psSvcReq);
      strcpy_P(g_sTmp,g_psOverCurrent);
      sprintf(g_sTmp+strlen(g_sTmp)," %dA",(int)(g_EvseController.GetChargingCurrent()/1000-g_EvseController.GetCurrentCapacity()));
      LcdPrint(1,g_sTmp);
#endif
      break;
#endif // OVERCURRENT_THRESHOLD   
    case EVSE_STATE_NO_GROUND:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      if (updmode == OBD_UPD_HARDFAULT) {
        LcdMsg_P(g_EvseController.CGMIisEnabled() ? g_psSvcReq : g_psEvseError,g_psNoGround);
      }
      else {
	// 2nd line will be updated below with auto retry count
        LcdPrint_P(0,g_psNoGround);
      }
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_STUCK_RELAY:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(updmode == OBD_UPD_HARDFAULT ? g_psSvcReq : g_psEvseError,g_psStuckRelay);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_RELAY_CLOSURE_FAULT:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psSvcReq,g_psRelayClosureFault);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DISABLED:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2
      LcdSetBacklightColor(VIOLET);
      LcdClear();
      LcdSetCursor(0,0);
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {
	LcdWrite(4); 
      }
#endif // AUTH_LOCK
      LcdPrint_P(g_psDisabled);
      LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
      break;
#ifdef GFI_SELFTEST
    case EVSE_STATE_GFI_TEST_FAILED:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psTestFailed,g_psGfci);
#endif
      break;
#endif // GFI_SELFTEST
    case EVSE_STATE_SLEEPING:
      SetGreenLed(1);
      SetRedLed(1);
#ifdef LCD16X2
      LcdSetBacklightColor(g_EvseController.EvConnected() ? WHITE : VIOLET);
      LcdClear();
      LcdSetCursor(0,0);
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {
	LcdWrite(4); 
      }
#endif // AUTH_LOCK
      LcdPrint_P(g_psSleeping);
      LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
      break;
    default:
      SetGreenLed(0);
      SetRedLed(1);
      // n.b. blue LED is off
    }
#ifdef TEMPERATURE_MONITORING
    if ((g_TempMonitor.OverTemperature() || g_TempMonitor.OverTemperatureLogged()) && !g_EvseController.InHardFault()) {
#ifdef LCD16X2
      LcdSetBacklightColor(RED);
#endif
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2
      LcdPrint_P(0,g_psHighTemp);
#endif
    }
#endif // TEMPERATURE_MONITORING
  }

  //
  // put anything that needs to be updated periodically here
  // the code below will only run once per second
  //
  if (((curms-m_LastUpdateMs) >= 1000) || (updmode == OBD_UPD_FORCE)) {
    m_LastUpdateMs = curms;
    
#ifdef GFI
    if (!g_EvseController.InHardFault() &&
	((curstate == EVSE_STATE_GFCI_FAULT) || (curstate == EVSE_STATE_NO_GROUND))) {
#ifdef LCD16X2
      strcpy(g_sTmp,g_sRetryIn);
      int resetsec = (int)(g_EvseController.GetResetMs() / 1000ul);
      if (resetsec >= 0) {
	sprintf(g_sTmp+sizeof(g_sTmp)-6,g_sHHMMfmt,resetsec / 60,resetsec % 60);
	strcat(g_sTmp,g_sTmp+sizeof(g_sTmp)-6);
	LcdPrint(1,g_sTmp);
      }
#endif // LCD16X2
      return;
    }
#endif // GFI

#ifdef RTC
    DateTime currentTime = g_RTC.now();
#endif

#ifdef LCD16X2
#if defined(AMMETER)
    if (((curstate == EVSE_STATE_C)
#ifdef ECVF_AMMETER_CAL
         || g_EvseController.AmmeterCalEnabled()
#endif 
         ) && AmmeterIsDirty()) {
      SetAmmeterDirty(0);

      uint32_t current = g_EvseController.GetChargingCurrent();

#if defined(PP_AUTO_AMPACITY)
      int a = current / 1000;
      int ma = (current % 1000) / 100;
      if (ma >= 5) {
	a++;
      }
      sprintf(g_sTmp,"%d:%dA",a,g_EvseController.GetCurrentCapacity());

      LcdPrint(9,0,"       ");
      LcdPrint(LCD_MAX_CHARS_PER_LINE-strlen(g_sTmp),0,g_sTmp);
#else //!PP_AUTO_AMPACITY
      if (current >= 1000) { // display only if > 1000
	int a = current / 1000;
	int ma = (current % 1000) / 100;
	if (ma > 9) {
	  ma = 0;
	  a++;
	}
	sprintf(g_sTmp,"%3d.%dA",a,ma);
      }
      else {
	strcpy_P(g_sTmp,PSTR("    0A"));
      }
      LcdPrint(10,0,g_sTmp);
#endif // PP_AUTO_AMPACITY
    }
#endif // AMMETER

    if (curstate == EVSE_STATE_C) {
#ifndef KWH_RECORDING
      time_t elapsedTime = g_EvseController.GetElapsedChargeTime();
#endif
   
#ifdef KWH_RECORDING
      sprintf(g_sTmp,STRF_WH,(g_EnergyMeter.GetSessionWs() / 3600) );
      LcdPrint(0,1,g_sTmp);

#ifdef VOLTMETER
      sprintf(g_sTmp,STRF_VOLT,(g_EvseController.GetVoltage() / 1000));  // Display voltage from OpenEVSE II
      LcdPrint(11,1,g_sTmp);
#else
      sprintf(g_sTmp,STRF_KWH,(g_EnergyMeter.GetTotkWh() / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
#endif // VOLTMETER
#endif // KWH_RECORDING

#ifdef TEMPERATURE_MONITORING
      if ((g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS)  {
	g_OBD.LcdClearLine(1);
	const char *tempfmt = "%2d.%1dC";
#ifdef MCP9808_IS_ON_I2C
	if ( g_TempMonitor.m_MCP9808_temperature != TEMPERATURE_NOT_INSTALLED) {   
	  sprintf(g_sTmp,tempfmt,g_TempMonitor.m_MCP9808_temperature/10, abs(g_TempMonitor.m_MCP9808_temperature % 10));  //  Ambient sensor near or on the LCD
	  LcdPrint(0,1,g_sTmp);
	}
#endif

#ifdef RTC	
	if ( g_TempMonitor.m_DS3231_temperature != TEMPERATURE_NOT_INSTALLED) {
	  sprintf(g_sTmp,tempfmt,g_TempMonitor.m_DS3231_temperature/10, abs(g_TempMonitor.m_DS3231_temperature % 10));      //  sensor built into the DS3231 RTC Chip
	  LcdPrint(5,1,g_sTmp);
	}
#endif
	
#ifdef TMP007_IS_ON_I2C
	if ( g_TempMonitor.m_TMP007_temperature != TEMPERATURE_NOT_INSTALLED ) {
	  sprintf(g_sTmp,tempfmt,g_TempMonitor.m_TMP007_temperature/10, abs(g_TempMonitor.m_TMP007_temperature % 10));  //  Infrared sensor probably looking at 30A fuses
	  LcdPrint(11,1,g_sTmp);
	}
#endif

	if (g_TempMonitor.BlinkAlarm() && g_TempMonitor.OverTemperatureShutdown()) { // Blink Red off-and-on while over the temperature shutdown limit, zero current should flow to the EV
	  g_TempMonitor.SetBlinkAlarm(0);                                            // toggle the alarm flag so we can blink
	  SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
	  LcdSetBacklightColor(RED);
#endif //Adafruit RGB LCD            
	}
	else if (g_TempMonitor.BlinkAlarm() == 0) { // If baclkight was left RED while last blinking
	  g_TempMonitor.SetBlinkAlarm(1);           // toggle the alarm flag so we can blink
	  SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
	  LcdSetBacklightColor(TEAL);
#endif
	}           
      }  // (g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS) 
      else if (g_TempMonitor.BlinkAlarm() == 0) { // If baclkight was left RED while last blinking
	g_TempMonitor.SetBlinkAlarm(1); // reset the alarm flag
	SetRedLed(0);                   // restore the normal TEAL backlight
#ifdef LCD16X2 //Adafruit RGB LCD
	LcdSetBacklightColor(TEAL);
#endif
      }
      if (!(g_TempMonitor.OverTemperature() || TEMPERATURE_DISPLAY_ALWAYS)) { 
#endif // TEMPERATURE_MONITORING
#ifndef KWH_RECORDING
      int h = elapsedTime / 3600;
      int m = (elapsedTime % 3600) / 60;
      int s = elapsedTime % 60;
      sprintf(g_sTmp,"%02d:%02d:%02d",h,m,s);
#ifdef RTC
      g_sTmp[8]=' ';
      g_sTmp[9]=' ';
      g_sTmp[10]=' ';
      sprintf(g_sTmp+11,g_sHHMMfmt,currentTime.hour(),currentTime.minute());
#endif //RTC
      LcdPrint(1,g_sTmp);
#endif // KWH_RECORDING
#ifdef TEMPERATURE_MONITORING
      }
#endif // TEMPERATURE_MONITORING
    } // curstate == EVSE_STATE_C
    // Display a new stopped LCD screen with Delay Timers enabled - GoldServe
#ifdef DELAYTIMER
    else if (curstate == EVSE_STATE_SLEEPING) {
      LcdSetCursor(0,0);
      g_DelayTimer.PrintTimerIcon();
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {
	LcdWrite(4); 
      }
#endif // AUTH_LOCK
      LcdPrint_P(g_psSleeping);
      sprintf(g_sTmp,"%02d:%02d:%02d",currentTime.hour(),currentTime.minute(),currentTime.second());
      LcdPrint(0,1,g_sTmp);
      if (g_DelayTimer.IsTimerEnabled()){
	LcdSetCursor(9,0);
	LcdWrite(2);
	LcdWrite(6);
	sprintf(g_sTmp,g_sHHMMfmt,g_DelayTimer.GetStartTimerHour(),g_DelayTimer.GetStartTimerMin());
	LcdPrint(11,0,g_sTmp);
	LcdSetCursor(9,1);
	LcdWrite(1);
	LcdWrite(6);
	sprintf(g_sTmp,g_sHHMMfmt,g_DelayTimer.GetStopTimerHour(),g_DelayTimer.GetStopTimerMin());
	LcdPrint(11,1,g_sTmp);
      } else {
	sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,currentcap);
	LcdPrint(10,0,g_sTmp);
      }
    }
#endif // DELAYTIMER

  if ( g_OBD.lastBacklightChangeMillis < millis() + LCD_BL_TIMEOUT) {
    //turn backlight off after 5 secs
      LcdSetBacklightColor(0);
  }
#endif // LCD16X2
  }

#ifdef FT_ENDURANCE
  LcdSetCursor(0,0);
  sprintf(g_sTmp,"%d %d",g_CycleCnt,(int)g_EvseController.ReadACPins());
  LcdPrint(g_sTmp);
#endif // FT_ENDURANCE
}




#ifdef BTN_MENU
Btn::Btn()
{
  buttonState = BTN_STATE_OFF;
  lastDebounceTime = 0;
  vlongDebounceTime = 0;
}

void Btn::init()
{
#ifdef BTN_REG
  pinBtn.init(BTN_REG,BTN_IDX,DigitalPin::INP_PU);
#endif
}

void Btn::read()
{
  uint8_t sample;
  unsigned long delta;
#ifdef ADAFRUIT_BTN
  sample = (g_OBD.readButtons() & BUTTON_SELECT) ? 1 : 0;
#else //!ADAFRUIT_BTN
  sample = pinBtn.read() ? 0 : 1;
#endif // ADAFRUIT_BTN
  if (!sample && (buttonState == BTN_STATE_LONG) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
  }
  if ((buttonState == BTN_STATE_OFF) ||
      ((buttonState == BTN_STATE_SHORT) && lastDebounceTime)) {
    if (sample) {
      if (!lastDebounceTime && (buttonState == BTN_STATE_OFF)) {
	lastDebounceTime = millis();
      }
      delta = millis() - lastDebounceTime;

      if (buttonState == BTN_STATE_OFF) {
	if (delta >= BTN_PRESS_SHORT) {
	  buttonState = BTN_STATE_SHORT;
	}
      }
      else if (buttonState == BTN_STATE_SHORT) {
	if (delta >= BTN_PRESS_LONG) {
	  buttonState = BTN_STATE_LONG;
	}
      }
    }
    else { //!sample
      lastDebounceTime = 0;
    }
  }
#ifdef RAPI_WF
  else if (sample && vlongDebounceTime && (buttonState == BTN_STATE_LONG)) {
    if ((millis() - vlongDebounceTime) >= BTN_PRESS_VERYLONG) {
      vlongDebounceTime = 0;
      RapiSetWifiMode(WIFI_MODE_AP_DEFAULT);
    }
  }
#endif // RAPI_WF
}

uint8_t Btn::shortPress()
{
  if ((buttonState == BTN_STATE_SHORT) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t Btn::longPress()
{
  if ((buttonState == BTN_STATE_LONG) && lastDebounceTime) {
    vlongDebounceTime = lastDebounceTime;
    lastDebounceTime = 0;
    return 1;
  }
  else {
    return 0;
  }
}


Menu::Menu()
{
}

void Menu::init(const char *firstitem)
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint(1,firstitem);
}

SettingsMenu::SettingsMenu()
{
  m_Title = g_psSettings;

  m_menuCnt = 0;
  while (g_SettingsMenuList[m_menuCnt]) {
    m_menuCnt++;
  }
}

#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
void SettingsMenu::CheckSkipLimits()
{
  // only allow Charge Limit menu item if car connected and no error
  m_skipLimits = !g_EvseController.LimitsAllowed();
}
#endif // CHARGE_LIMIT

void SettingsMenu::Init()
{
  m_CurIdx = 0;

#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
  while (m_skipLimits && (
#if defined(CHARGE_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_ChargeLimitMenu) ||
#endif
#if defined(TIME_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_TimeLimitMenu)
#else
	 0
#endif
			  )) {
    m_CurIdx++;
  }
#endif // CHARGE_LIMIT || TIME_LIMIT

  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint_P(1,g_SettingsMenuList[m_CurIdx]->m_Title);
}

void SettingsMenu::Next()
{
  if (++m_CurIdx > m_menuCnt) {
    m_CurIdx = 0;
  }

#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
  while (m_skipLimits && (
#if defined(CHARGE_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_ChargeLimitMenu) ||
#endif
#if defined(TIME_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_TimeLimitMenu)
#else
	 0
#endif
			  )) {
    m_CurIdx++;
  }
#endif // CHARGE_LIMIT || TIME_LIMIT

  PGM_P title;
  if (m_CurIdx < m_menuCnt) {
    title = g_SettingsMenuList[m_CurIdx]->m_Title;
  }
  else {
    title = g_psExit;
  }

  g_OBD.LcdPrint_P(1,title);
}

Menu *SettingsMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {
    return g_SettingsMenuList[m_CurIdx];
  }
  else {
    g_OBD.LcdClear();
    return NULL;
  }
}

SetupMenu::SetupMenu()
{
  m_Title = g_psSetup;

  m_menuCnt = 0;
  while (g_SetupMenuList[m_menuCnt]) {
    m_menuCnt++;
  }
}

void SetupMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint_P(1,g_SetupMenuList[0]->m_Title);
}

void SetupMenu::Next()
{
  if (++m_CurIdx > m_menuCnt) {
    m_CurIdx = 0;
  }

  PGM_P title;
  if (m_CurIdx < m_menuCnt) {
    title = g_SetupMenuList[m_CurIdx]->m_Title;
  }
  else {
    title = g_psExit;
  }

  g_OBD.LcdPrint_P(1,title);
}

Menu *SetupMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {
    return g_SetupMenuList[m_CurIdx];
  }
  else {
    m_CurIdx = 0;
    g_EvseController.Reboot();
    return &g_SetupMenu;
  }
}

#ifndef NOSETUP_MENU

#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
#define SVC_LVL_MNU_ITEMCNT 3
#else
#define SVC_LVL_MNU_ITEMCNT 2
#endif // ADVPWR && AUTOSVCLEVEL
const char *g_SvcLevelMenuItems[] = {
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
  STR_AUTO,
#endif // ADVPWR && AUTOSVCLEVEL
  STR_LEVEL_1,
  STR_LEVEL_2
};

#ifdef RGBLCD
BklTypeMenu::BklTypeMenu()
{
  m_Title = g_psBklType;
}
void BklTypeMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = (g_BtnHandler.GetSavedLcdMode() == BKL_TYPE_RGB) ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_BklMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void BklTypeMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  g_OBD.LcdPrint(g_sPlus);
  g_OBD.LcdPrint(g_BklMenuItems[m_CurIdx]);
}

Menu *BklTypeMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_BklMenuItems[m_CurIdx]);

  g_EvseController.SetBacklightType((m_CurIdx == 0) ? BKL_TYPE_RGB : BKL_TYPE_MONO);
  g_BtnHandler.SetSavedLcdMode((m_CurIdx == 0) ? BKL_TYPE_RGB : BKL_TYPE_MONO);

  return &g_SetupMenu;

}
#endif // RGBLCD


SvcLevelMenu::SvcLevelMenu()
{
  m_Title = g_psSvcLevel;
}

void SvcLevelMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
  if (g_EvseController.AutoSvcLevelEnabled()) {
    m_CurIdx = 0;
  }
  else {
    m_CurIdx = g_EvseController.GetCurSvcLevel();
  }
#else
  m_CurIdx = (g_EvseController.GetCurSvcLevel() == 1) ? 0 : 1;
#endif // ADVPWR && AUTOSVCLEVEL
  //sprintf(g_sTmp,"+%s",g_SvcLevelMenuItems[m_CurIdx]);
  //g_OBD.LcdPrint(1,g_sTmp);
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);
}

void SvcLevelMenu::Next()
{
  if (++m_CurIdx >= SVC_LVL_MNU_ITEMCNT) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
#ifdef AUTOSVCLEVEL
  if ((g_EvseController.AutoSvcLevelEnabled() && !m_CurIdx) ||
      (!g_EvseController.AutoSvcLevelEnabled() && (g_EvseController.GetCurSvcLevel() == m_CurIdx))) {
    g_OBD.LcdPrint(g_sPlus);
  }
#else
  if (g_EvseController.GetCurSvcLevel() == (m_CurIdx+1)) {
    g_OBD.LcdPrint(g_sPlus);
  }
#endif //AUTOSVCLEVEL
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);
}

Menu *SvcLevelMenu::Select()
{
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
  if (m_CurIdx == 0) {
    g_EvseController.EnableAutoSvcLevel(1);
  }
  else {
    g_EvseController.SetSvcLevel(m_CurIdx);
    g_EvseController.EnableAutoSvcLevel(0);
  }
#else
  g_EvseController.SetSvcLevel(m_CurIdx+1);
#endif // ADVPWR && AUTOSVCLEVEL
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);

  g_EvseController.SaveEvseFlags();

  delay(500);

  return &g_SetupMenu;
}

DiodeChkMenu::DiodeChkMenu()
{
  m_Title = g_psDiodeCheck;
}

void DiodeChkMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.DiodeCheckEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void DiodeChkMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.DiodeCheckEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *DiodeChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableDiodeCheck((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}

#ifdef GFI_SELFTEST
GfiTestMenu::GfiTestMenu()
{
  m_Title = g_psGfiTest;
}

void GfiTestMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.GfiSelfTestEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void GfiTestMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.GfiSelfTestEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *GfiTestMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableGfiSelfTest((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
#endif // GFI_SELFTEST

#ifdef TEMPERATURE_MONITORING
TempOnOffMenu::TempOnOffMenu()
{
  m_Title = g_psTempChk;
}

void TempOnOffMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.TempChkEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void TempOnOffMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.TempChkEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *TempOnOffMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableTempChk((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
#endif // TEMPERATURE_MONITORING

VentReqMenu::VentReqMenu()
{
  m_Title = g_psVentReqChk;
}

void VentReqMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.VentReqEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void VentReqMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.VentReqEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *VentReqMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableVentReq((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}

#ifdef ADVPWR
GndChkMenu::GndChkMenu()
{
  m_Title = g_psGndChk;
}

void GndChkMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.GndChkEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void GndChkMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.GndChkEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *GndChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableGndChk((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
RlyChkMenu::RlyChkMenu()
{
  m_Title = g_psRlyChk;
}

void RlyChkMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.StuckRelayChkEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void RlyChkMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.StuckRelayChkEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *RlyChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableStuckRelayChk((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
#endif // ADVPWR

#endif // NOSETUP_MENU

MaxCurrentMenu::MaxCurrentMenu()
{
  m_Title = g_psMaxCurrent;
}


void MaxCurrentMenu::Init()
{
  uint8_t cursvclvl = g_EvseController.GetCurSvcLevel();
  m_MinCurrent = MIN_CURRENT_CAPACITY_J1772;
  if (cursvclvl == 1) {
    m_MaxCurrent = MAX_CURRENT_CAPACITY_L1;
  }
  else {
    m_MaxCurrent = g_EvseController.GetMaxHwCurrentCapacity();
  }
  
  sprintf(g_sTmp,g_sMaxCurrentFmt,(cursvclvl == 1) ? "L1" : "L2");
  g_OBD.LcdPrint(0,g_sTmp);
  m_CurIdx = g_EvseController.GetCurrentCapacity();
  if (m_CurIdx < m_MinCurrent) m_CurIdx = m_MinCurrent;
  sprintf(g_sTmp,"+%dA",m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
}

void MaxCurrentMenu::Next()
{
  if (++m_CurIdx > m_MaxCurrent) {
    m_CurIdx = m_MinCurrent;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  if (g_EvseController.GetCurrentCapacity() == m_CurIdx) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(m_CurIdx);
  g_OBD.LcdPrint("A");
}

Menu *MaxCurrentMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(m_CurIdx);
  g_OBD.LcdPrint("A");
  delay(500);
  eeprom_write_byte((uint8_t*)((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2),m_CurIdx);  
  g_EvseController.SetCurrentCapacity(m_CurIdx);
  return &g_SetupMenu;
}


ResetMenu::ResetMenu()
{
  m_Title = g_psReset;
}


void ResetMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,g_psResetNow);
  g_OBD.LcdPrint(1,g_YesNoMenuItems[0]);
}

void ResetMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
}

Menu *ResetMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    g_EvseController.Reboot();
  }
  return NULL;
}

#ifdef DELAYTIMER_MENU
// pluspos = -1 = suppress "+"
void DtsStrPrint1(uint16_t y,uint8_t mo,uint8_t d,uint8_t h,uint8_t mn,int8_t pluspos)
{
  *g_sTmp = 0;
  if (pluspos == 0) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(mo,2));
  strcat(g_sTmp,g_sSlash);
  if (pluspos == 1) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(d,2));
  strcat(g_sTmp,g_sSlash);
  if (pluspos == 2) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(y,2));
  strcat(g_sTmp,g_sSpace);
  if (pluspos == 3) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(h,2));
  strcat(g_sTmp,g_sColon);
  if (pluspos == 4) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(mn,2));

  g_OBD.LcdPrint(1,g_sTmp);
}


RTCMenu::RTCMenu()
{
  m_Title = g_psRTC;
}

void RTCMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,g_psSetDateTime);
  g_OBD.LcdPrint(1,g_YesNoMenuItems[0]);
}

void RTCMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
}
Menu *RTCMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    return &g_RTCMenuMonth;
  } else {
    return &g_SetupMenu; 
  }
}
RTCMenuMonth::RTCMenuMonth()
{
}
void RTCMenuMonth::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Month);
  DateTime t = g_RTC.now();
  g_month = t.month();
  g_day = t.day();
  g_year = t.year() - 2000;
  g_hour = t.hour();
  g_min = t.minute();
  m_CurIdx = g_month;

  DtsStrPrint1(g_year,g_month,g_day,g_hour,g_min,0);
}
void RTCMenuMonth::Next()
{
  if (++m_CurIdx >= 13) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,m_CurIdx,g_day,g_hour,g_min,0);
}
Menu *RTCMenuMonth::Select()
{
  g_month = m_CurIdx;
  DtsStrPrint1(g_year,m_CurIdx,g_day,g_hour,g_min,0);
  delay(500);
  return &g_RTCMenuDay;
}
RTCMenuDay::RTCMenuDay()
{
  m_Title = g_psRTC;
}
void RTCMenuDay::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Day);
  m_CurIdx = g_day;
  DtsStrPrint1(g_year,g_month,m_CurIdx,g_hour,g_min,1);
}
void RTCMenuDay::Next()
{
  if (++m_CurIdx >= 32) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,g_month,m_CurIdx,g_hour,g_min,1);
}
Menu *RTCMenuDay::Select()
{
  g_day = m_CurIdx;
  DtsStrPrint1(g_year,g_month,m_CurIdx,g_hour,g_min,1);
  delay(500);
  return &g_RTCMenuYear;
}
RTCMenuYear::RTCMenuYear()
{
}
#define YEAR_MIN 23
#define YEAR_MAX 33
void RTCMenuYear::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Year);
  m_CurIdx = g_year;
  if (m_CurIdx < YEAR_MIN || m_CurIdx > YEAR_MAX){
    m_CurIdx = YEAR_MIN;
    g_year = YEAR_MIN;
  }
  DtsStrPrint1(m_CurIdx,g_month,g_day,g_hour,g_min,2);
}
void RTCMenuYear::Next()
{
  if (++m_CurIdx > YEAR_MAX) {
    m_CurIdx = YEAR_MIN;
  }
  DtsStrPrint1(m_CurIdx,g_month,g_day,g_hour,g_min,2);
}
Menu *RTCMenuYear::Select()
{
  g_year = m_CurIdx;
  DtsStrPrint1(m_CurIdx,g_month,g_day,g_hour,g_min,2);
  delay(500);
  return &g_RTCMenuHour;
}
RTCMenuHour::RTCMenuHour()
{
}
void RTCMenuHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Hour);
  m_CurIdx = g_hour;
  DtsStrPrint1(g_year,g_month,g_day,m_CurIdx,g_min,3);
}
void RTCMenuHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,g_month,g_day,m_CurIdx,g_min,3);
}
Menu *RTCMenuHour::Select()
{
  g_hour = m_CurIdx;
  DtsStrPrint1(g_year,g_month,g_day,m_CurIdx,g_min,3);
  delay(500);
  return &g_RTCMenuMinute;
}
RTCMenuMinute::RTCMenuMinute()
{
}
void RTCMenuMinute::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Minute);
  m_CurIdx = g_min;
  DtsStrPrint1(g_year,g_month,g_day,g_hour,m_CurIdx,4);
}
void RTCMenuMinute::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,g_month,g_day,g_hour,m_CurIdx,4);
}
Menu *RTCMenuMinute::Select()
{
  g_min = m_CurIdx;
  DtsStrPrint1(g_year,g_month,g_day,g_hour,m_CurIdx,4);
  g_RTC.adjust(DateTime(g_year, g_month, g_day, g_hour, g_min, 0));
  delay(500);
  return &g_SetupMenu;
}

void HsStrPrint1(uint8_t h,uint8_t m,int8_t pluspos=-1)
{
  *g_sTmp = 0;
  if (pluspos == 0) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(h,2));
  strcat(g_sTmp,g_sColon);
  if (pluspos == 1) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(m,2));
  g_OBD.LcdPrint(1,g_sTmp);
}


DelayMenu::DelayMenu()
{
  m_Title = g_psDelayTimer;
}
void DelayMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,g_psDelayTimer);
  g_OBD.LcdPrint(1,g_DelayMenuItems[0]);
}
void DelayMenu::Next()
{
  if (++m_CurIdx >= 3) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_DelayMenuItems[m_CurIdx]);
}
Menu *DelayMenu::Select()
{
  delay(500);
  if (m_CurIdx == 0) {
    return &g_DelayMenuEnableDisable;
  } else if (m_CurIdx == 1) {
    return &g_DelayMenuStartHour;
  } else {
    return &g_DelayMenuStopHour;
  }
}
DelayMenuEnableDisable::DelayMenuEnableDisable()
{
  m_Title = g_psDelayTimer;
}
void DelayMenuEnableDisable::Init()
{
  m_CurIdx = !g_DelayTimer.IsTimerEnabled();
  g_OBD.LcdPrint_P(0,g_psDelayTimer);
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}
void DelayMenuEnableDisable::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == !g_DelayTimer.IsTimerEnabled()){
    g_OBD.LcdPrint(0,1,g_sPlus);
    g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  } else {
    g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
  }
}
Menu *DelayMenuEnableDisable::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    g_DelayTimer.Enable();
  } else {
    g_DelayTimer.Disable();
  }
  return &g_SettingsMenu;
}
DelayMenuStartHour::DelayMenuStartHour()
{
}
void DelayMenuStartHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStartHour);
  g_hour = g_DelayTimer.GetStartTimerHour();
  g_min = g_DelayTimer.GetStartTimerMin();
  m_CurIdx = g_hour;
  HsStrPrint1(m_CurIdx,g_min,0);
}

void DelayMenuStartHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  HsStrPrint1(m_CurIdx,g_min,0);
}
Menu *DelayMenuStartHour::Select()
{
  g_hour = m_CurIdx;
  HsStrPrint1(m_CurIdx,g_min,0);
  delay(500);
  return &g_DelayMenuStartMin;
}
DelayMenuStopHour::DelayMenuStopHour()
{
}
void DelayMenuStopHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStopHour);
  g_hour = g_DelayTimer.GetStopTimerHour();
  g_min = g_DelayTimer.GetStopTimerMin();
  m_CurIdx = g_hour;
  HsStrPrint1(m_CurIdx,g_min,0);
}
void DelayMenuStopHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  HsStrPrint1(m_CurIdx,g_min,0);
}
Menu *DelayMenuStopHour::Select()
{
  g_hour = m_CurIdx;
  HsStrPrint1(m_CurIdx,g_min,0);
  delay(500);
  return &g_DelayMenuStopMin;
}
DelayMenuStartMin::DelayMenuStartMin()
{
}
void DelayMenuStartMin::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStartMin);
  m_CurIdx = g_min;
  HsStrPrint1(g_hour,m_CurIdx,1);
}

void DelayMenuStartMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  HsStrPrint1(g_hour,m_CurIdx,1);
}
Menu *DelayMenuStartMin::Select()
{
  g_min = m_CurIdx;
  HsStrPrint1(g_hour,m_CurIdx,1);
  g_DelayTimer.SetStartTimer(g_hour, g_min);
  delay(500);
  return &g_SettingsMenu;
}
DelayMenuStopMin::DelayMenuStopMin()
{
}
void DelayMenuStopMin::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStopMin);
  m_CurIdx = g_min;
  HsStrPrint1(g_hour,m_CurIdx,1);
}

void DelayMenuStopMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  HsStrPrint1(g_hour,m_CurIdx,1);
}
Menu *DelayMenuStopMin::Select()
{
  g_min = m_CurIdx;
  HsStrPrint1(g_hour,m_CurIdx,1);
  g_DelayTimer.SetStopTimer(g_hour, g_min);  // Set Timers
  delay(500);
  return &g_SettingsMenu;
}
#endif // DELAYTIMER_MENU

#ifdef CHARGE_LIMIT
#define MAX_CHARGE_LIMIT 40

ChargeLimitMenu::ChargeLimitMenu()
{
  m_Title = g_psChargeLimit;
}

void ChargeLimitMenu::showCurSel(uint8_t plus)
{
  *g_sTmp = 0;
  if (plus) strcpy(g_sTmp,g_sPlus);
  if (m_CurIdx == 0) {
    strcat(g_sTmp,"off");
  }
  else {
    strcat(g_sTmp,u2a(m_CurIdx));
    strcat(g_sTmp," kWh");
  }
  g_OBD.LcdPrint(1,g_sTmp);
}


void ChargeLimitMenu::Init()
{
  m_CurIdx = g_EvseController.GetChargeLimitkWh();

  g_OBD.LcdPrint_P(0,g_psChargeLimit);
  showCurSel(1);
}

void ChargeLimitMenu::Next()
{
  if (m_CurIdx < 5) m_CurIdx++;
  else m_CurIdx += 5;
  if (m_CurIdx > MAX_CHARGE_LIMIT) {
    m_CurIdx = 0;
  }
  showCurSel((g_EvseController.GetChargeLimitkWh() == m_CurIdx) ? 1 : 0);
}

Menu *ChargeLimitMenu::Select()
{
  showCurSel(1);
  g_EvseController.SetChargeLimitkWh(m_CurIdx);
  delay(500);
  return m_CurIdx ? NULL : &g_SettingsMenu;
}

#endif // CHARGE_LIMIT


#ifdef TIME_LIMIT
// max time limit = MAX_TIME_LIMIT_D15 * 15 minutes
#define MAX_TIME_LIMIT_D15 32

TimeLimitMenu::TimeLimitMenu()
{
  m_Title = g_psTimeLimit;
}

void TimeLimitMenu::showCurSel(uint8_t plus)
{
  uint16_t limit = m_CurIdx * 15;
  *g_sTmp = 0;
  if (plus) strcpy(g_sTmp,g_sPlus);
  if (limit == 0) {
    strcat(g_sTmp,"off");
  }
  else {
    if (limit < 60) {
      strcat(g_sTmp,u2a(limit));
      strcat(g_sTmp," min");
    }
    else {
      strcat(g_sTmp,u2a(limit / 60));
      if (limit % 60) { // assume == 30
	strcat(g_sTmp,".5");
      }
      strcat(g_sTmp," hr");
    }
  }
  g_OBD.LcdPrint(1,g_sTmp);
}


void TimeLimitMenu::Init()
{
  m_CurIdx = g_EvseController.GetTimeLimit15();

  g_OBD.LcdPrint_P(0,g_psTimeLimit);
  showCurSel(1);
}

void TimeLimitMenu::Next()
{
  if (m_CurIdx < 4) m_CurIdx++;
  else if (m_CurIdx < 12) m_CurIdx += 2;
  else m_CurIdx += 4;
  if (m_CurIdx > MAX_TIME_LIMIT_D15) {
    m_CurIdx = 0;
  }
  showCurSel((g_EvseController.GetTimeLimit15() == m_CurIdx) ? 1 : 0);
}

Menu *TimeLimitMenu::Select()
{
  showCurSel(1);
  g_EvseController.SetTimeLimit15(m_CurIdx);
  delay(500);
  return m_CurIdx ? NULL : &g_SettingsMenu;
}
#endif // TIME_LIMIT

BtnHandler::BtnHandler()
{
  m_CurMenu = NULL;
}

int8_t BtnHandler::DoShortPress(int8_t infaultstate)
{
#ifdef TEMPERATURE_MONITORING
    g_TempMonitor.ClrOverTemperatureLogged();
#endif
    if (m_CurMenu) {
      m_CurMenu->Next();
    }
    else {
      // force into setup menu when in fault
    if (infaultstate) return 1; // triggers longpress action
      else {
	if ((g_EvseController.GetState() == EVSE_STATE_DISABLED) ||
	    (g_EvseController.GetState() == EVSE_STATE_SLEEPING)) {
	  g_EvseController.Enable();
	}
	else {
	  g_EvseController.Sleep();
	}

#ifdef DELAYTIMER
	if (g_DelayTimer.IsTimerEnabled()) {
	  uint8_t intimeinterval = g_DelayTimer.IsInAwakeTimeInterval();
	  uint8_t sleeping = (g_EvseController.GetState() == EVSE_STATE_SLEEPING) ? 1 : 0;
	  if ((intimeinterval && sleeping) || (!intimeinterval && !sleeping)) {
	    g_DelayTimer.SetManualOverride();
          }
	  else {
	    g_DelayTimer.ClrManualOverride();
   	  }
	}
#endif // DELAYTIMER
      }
    }

  return 0;
}

void BtnHandler::ChkBtn()
{
  WDT_RESET();
  m_Btn.read();

  if (!g_EvseController.ButtonIsEnabled()) {
#ifdef RAPI_BTN
    if (m_Btn.shortPress()) {
      RapiSendButtonPress(0);
    } else if (m_Btn.longPress()) {
      RapiSendButtonPress(1);
    }
#endif // RAPI_BTN
    return;
  }

  int8_t infaultstate = g_EvseController.InFaultState();

  if (m_Btn.shortPress()) {
    if (DoShortPress(infaultstate)) {
      goto longpress;
    }
  }
  else if (m_Btn.longPress()) {
#ifdef TEMPERATURE_MONITORING
    g_TempMonitor.ClrOverTemperatureLogged();
#endif
  longpress:
    if (m_CurMenu) {
      m_CurMenu = m_CurMenu->Select();
      if (m_CurMenu) {
	uint8_t curidx = 0;
	if ((m_CurMenu == &g_SettingsMenu)||(m_CurMenu == &g_SetupMenu)) {
	  curidx = m_CurMenu->m_CurIdx;
	}
	m_CurMenu->Init();
	if ((m_CurMenu == &g_SettingsMenu)||(m_CurMenu == &g_SetupMenu)) {
	  if (curidx > 0) {
	    // restore prev menu item
	    m_CurMenu->m_CurIdx = curidx-1;
	    m_CurMenu->Next();
	  }
	}

      }
      else { // exit
	g_EvseController.Enable();
	g_OBD.DisableUpdate(0);
	g_OBD.LcdSetBacklightType(m_SavedLcdMode); // exiting menus - restore LCD mode
	g_OBD.Update(OBD_UPD_FORCE);
	g_EvseController.ClrInMenu();
      }
    }
    else {
      g_EvseController.SetInMenu();
#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
      g_SettingsMenu.CheckSkipLimits();
#endif // CHARGE_LIMIT
      m_SavedLcdMode = g_OBD.IsLcdBacklightMono() ? BKL_TYPE_MONO : BKL_TYPE_RGB;
      g_OBD.LcdSetBacklightColor(WHITE);
      g_OBD.DisableUpdate(1);
      if (infaultstate) {
	m_CurMenu = &g_SetupMenu;
      }
      else {
	g_EvseController.Sleep();
	m_CurMenu = &g_SettingsMenu;
      }
      m_CurMenu->Init();
    }
  }
}
#endif // BTN_MENU

// Start Delay Timer Functions - GoldServe
#ifdef DELAYTIMER
void DelayTimer::Init() {
  //Read EEPROM settings
  uint8_t rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_FLAGS);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_DelayTimerEnabled = 0x00;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  }
  else {
    m_DelayTimerEnabled = rtmp;
  }

  if (m_DelayTimerEnabled) g_EvseController.SetDelayTimerOnFlag();
  else g_EvseController.ClrDelayTimerOnFlag();

  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerHour = DEFAULT_START_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_HOUR, m_StartTimerHour);
  }
  else {
    m_StartTimerHour = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerMin = DEFAULT_START_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_MIN, m_StartTimerMin);
  }
  else {
    m_StartTimerMin = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerHour = DEFAULT_STOP_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
  }
  else {
    m_StopTimerHour = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerMin = DEFAULT_STOP_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_MIN, m_StopTimerMin);
  }
  else {
    m_StopTimerMin = rtmp;
  }

  ClrManualOverride();
}

uint8_t DelayTimer::IsInAwakeTimeInterval()
{
  uint8_t inTimeInterval = false;

  if (IsTimerEnabled() && IsTimerValid()) {
    DateTime t = g_RTC.now();
    uint8_t currHour = t.hour();
    uint8_t currMin = t.minute();
    
    uint16_t startTimerMinutes = m_StartTimerHour * 60 + m_StartTimerMin; 
    uint16_t stopTimerMinutes = m_StopTimerHour * 60 + m_StopTimerMin;
    uint16_t currTimeMinutes = currHour * 60 + currMin;

    if (stopTimerMinutes < startTimerMinutes) { //End time is for next day 
      
      if ( ( (currTimeMinutes >= startTimerMinutes) && (currTimeMinutes > stopTimerMinutes) ) || 
	   ( (currTimeMinutes <= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes) ) ){
	inTimeInterval = true;
      }
    }
    else { // not crossing midnite
      if ((currTimeMinutes >= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes)) { 
	inTimeInterval = true;
      }
    }
  }

  return inTimeInterval;
}

void DelayTimer::CheckTime()
{
  if (!g_EvseController.InFaultState() &&
      !(g_EvseController.GetState() == EVSE_STATE_DISABLED) &&
      IsTimerEnabled() &&
      IsTimerValid()) {
    unsigned long curms = millis();
    if ((curms - m_LastCheck) > 1000ul) {
      uint8_t inTimeInterval = IsInAwakeTimeInterval();
      uint8_t evseState = g_EvseController.GetState();

      if (inTimeInterval) { // charge now
	if (!ManualOverrideIsSet()) {
	  if (evseState == EVSE_STATE_SLEEPING) {
	    g_EvseController.Enable();
	  }
	}
	else {
	  if (evseState != EVSE_STATE_SLEEPING) {
	    // we got here because manual override was set by
	    // waking EVSE shortpress while it was sleeping
	    ClrManualOverride();
	  }
	}
      }
      else { // sleep now
	if (!ManualOverrideIsSet()) {
	  if ((evseState != EVSE_STATE_SLEEPING)
	      //	       && (evseState != EVSE_STATE_C) // don't interrupt active charging
	      ) {
	    g_EvseController.Sleep();
	  }
	}
	else { // manual override is set
	  if (evseState == EVSE_STATE_SLEEPING) {
	    // we got here because manual override was set by
	    // putting EVSE to sleep via shortpress while it was charging
	    ClrManualOverride();
	  }
	}
      }

      m_LastCheck = curms;
    }
  }
}
void DelayTimer::Enable(){
  m_DelayTimerEnabled = 0x01;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  ClrManualOverride();
  //  g_EvseController.SaveSettings();
  //  CheckTime();
  g_EvseController.SetDelayTimerOnFlag();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::Disable(){
  m_DelayTimerEnabled = 0x00;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  ClrManualOverride();
  //  g_EvseController.SaveSettings();
  g_EvseController.ClrDelayTimerOnFlag();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::PrintTimerIcon(){
#ifdef LCD16X2
  if (!ManualOverrideIsSet() && IsTimerEnabled() && IsTimerValid()){
    g_OBD.LcdWrite(6);
  }
#endif // LCD16X2
}
// End Delay Timer Functions - GoldServe
#endif //#ifdef DELAYTIMER


void ProcessInputs()
{
#ifdef RAPI
  RapiDoCmd();
#endif
#ifdef BTN_MENU
  g_BtnHandler.ChkBtn();
#endif
#ifdef TEMPERATURE_MONITORING
  g_TempMonitor.Read();  //   update temperatures once per second
#endif
}


void EvseReset()
{
  Wire.begin();
  g_OBD.Init();

#ifdef RAPI
  RapiInit();
#endif

  g_EvseController.Init();

#ifdef DELAYTIMER
  g_DelayTimer.Init(); // this *must* run after g_EvseController.Init() because it sets one of the vFlags
#endif  // DELAYTIMER

#ifdef PP_AUTO_AMPACITY
  g_ACCController.AutoSetCurrentCapacity();
#endif
}

#ifdef PP_AUTO_AMPACITY
uint8_t StateTransitionReqFunc(uint8_t curPilotState,uint8_t newPilotState,uint8_t curEvseState,uint8_t newEvseState)
{
  uint8_t retEvseState = newEvseState;

  if ((newEvseState >= EVSE_STATE_B) && (newEvseState <= EVSE_STATE_C)) {
    //n.b. no debounce delay needed because J1772EvseController::Update()
    // already debounces before requesting the state transition, so we can
    // be absolutely sure that the PP pin has firm contact by the time we
    // get here
    if (g_ACCController.AutoSetCurrentCapacity()) {
      // invalid PP so 0 amps - force to stay in State A
      retEvseState = EVSE_STATE_A;
    }
  }
  /*  else { // EVSE_STATE_A
    // reset back to default max current
    uint8_t amps = g_EvseController.GetMaxCurrentCapacity();
    g_EvseController.SetCurrentCapacity(amps,0,1);
  }
  */
  //  Serial.print(" r: ");Serial.print(retEvseState);Serial.print(" a: ");Serial.print(g_ACCController.GetCurAmps());
  //  Serial.print(" c: ");Serial.print(curEvseState);Serial.print(" n: ");Serial.print(newEvseState);Serial.print(" r: ");Serial.print(retEvseState);

  return retEvseState;
}
#endif //PP_AUTO_AMPACITY


void setup()
{
  wdt_disable();

  delay(400);  // give I2C devices time to be ready before running code that wants to initialize I2C devices.  Otherwise a hang can occur upon powerup.
  
  Serial.begin(SERIAL_BAUD);

#ifdef BTN_MENU
  g_BtnHandler.init();
#endif // BTN_MENU

#ifdef PP_AUTO_AMPACITY
  g_EvseController.SetStateTransitionReqFunc(&StateTransitionReqFunc);
#endif //PP_AUTO_AMPACITY
  EvseReset();
  
#ifdef TEMPERATURE_MONITORING
  g_TempMonitor.Init();
#endif

  

#ifdef BOOTLOCK
#ifdef LCD16X2
  g_OBD.LcdMsg_P(PSTR("Waiting for"),PSTR("Initialization.."));
#endif // LCD16X2
  while (g_EvseController.IsBootLocked()) {
    ProcessInputs();
  }
#endif // BOOTLOCK


  WDT_ENABLE();
}  // setup()


void loop()
{
  WDT_RESET();

  g_EvseController.Update();

#ifdef KWH_RECORDING
  g_EnergyMeter.Update();
#endif // KWH_RECORDING


#ifdef PERIODIC_LCD_REFRESH_MS
  // Force LCD update (required for CE certification testing) to restore LCD if corrupted.
  {
    static unsigned long lastlcdreset = 0;
    if ((millis()-lastlcdreset)>PERIODIC_LCD_REFRESH_MS) {
      g_OBD.Update(OBD_UPD_FORCE);
      lastlcdreset = millis();
    }
    else g_OBD.Update();
  }
#else // !PERIODIC_LCD_REFRESH_MS
  g_OBD.Update();
#endif // PERIODIC_LCD_REFRESH_MS

  ProcessInputs();

  //BackLightTimeout();

  // Delay Timer Handler - GoldServe
#ifdef DELAYTIMER
  g_DelayTimer.CheckTime();
#endif //#ifdef DELAYTIMER
}
