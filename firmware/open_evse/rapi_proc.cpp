// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2013-2023 Sam C. Lin <lincomatic@gmail.com>
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

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO
#include "open_evse.h"

#ifdef RAPI
const char RAPI_VER[] PROGMEM = RAPIVER;


#ifdef MCU_ID_LEN
// mcuid *must* be of size MCU_ID_LEN
#include <avr/boot.h>
void getMcuId(uint8_t *mcuid)
{
  for (int i=0;i < MCU_ID_LEN;i++) {
    mcuid[i] = boot_signature_byte_get(0x0E + i);
  }
}
#endif // MCU_ID_LEN


// convert 2-digit hex string to uint8_t
uint8_t htou8(const char *s)
{
  uint8_t u = 0;
  for (int i=0;i < 2;i++) {
    char c = s[i];
    if (c != '\0') {
      if (i == 1) u <<= 4;
    if ((c >= '0') && (c <= '9')) {
      u += c - '0';
    }
    else if ((c >= 'A') && (c <= 'F')) {
      u += c - 'A' + 10;
    }
      else if ((c >= 'a') && (c <= 'f')) {
	u += c - 'a' + 10;
      }
      else {
	// invalid character received
	return 0;
      }
    }
  }
  return u;
}

// convert decimal string to uint32_t
uint32_t dtou32(const char *s)
{
  uint32_t u = 0;
  while (*s) {
    u *= 10;
    u += *(s++) - '0';
  }
  return u;
}

#ifdef RAPI_I2C
//get data from master - HINT: this is a ISR call!
//HINT2: do not handle stuff here!! this will NOT work
//collect only data here and process it in the main loop!
void receiveEvent(int numBytes)
{
  //do nothing here
}
#endif // RAPI_I2C

EvseRapiProcessor::EvseRapiProcessor()
{
  curReceivedSeqId = INVALID_SEQUENCE_ID;
#ifdef RAPI_SENDER
  curSentSeqId = INVALID_SEQUENCE_ID;
#endif
}

void EvseRapiProcessor::init()
{
  echo = 0;
  reset();
}

int EvseRapiProcessor::doCmd()
{
  int rc = 1;

  int bcnt = available();
  if (bcnt) {
    for (int i=0;i < bcnt;i++) {
      char c = read();
      if (echo) write(c);

      if (c == ESRAPI_SOC) {
	buffer[0] = ESRAPI_SOC;
	bufCnt = 1;
      }
      else if (buffer[0] == ESRAPI_SOC) {
	if (bufCnt < ESRAPI_BUFLEN) {
	  if (c == ESRAPI_EOC) {
	    buffer[bufCnt++] = 0;
	    if (!tokenize(buffer)) {
	      rc = processCmd();
	    }
	    else {
	      reset();
	      curReceivedSeqId = INVALID_SEQUENCE_ID;
	      response(0);
	    }
	  }
	  else {
	    buffer[bufCnt++] = c;
	  }
	}
	else { // too many chars
	  reset();
	}
      }
    }
  }

  return rc;
}


void EvseRapiProcessor::sendBootNotification()
{
  sprintf(g_sTmp,"%cAB %02x ",ESRAPI_SOC,g_EvseController.GetState());
  char *s = g_sTmp+strlen(g_sTmp);
  GetVerStr(s);
  appendChk(g_sTmp);
  writeStart();
  write(g_sTmp);
  writeEnd();
}


void EvseRapiProcessor::sendEvseState()
{
    sprintf(g_sTmp,"%cAT %02x %02x %d %04x",ESRAPI_SOC,g_EvseController.GetState(),g_EvseController.GetPilotState(),g_EvseController.GetCurrentCapacity(),g_EvseController.GetVFlags());
  appendChk(g_sTmp);
  writeStart();
  write(g_sTmp);
  writeEnd();
}

#ifdef RAPI_WF
void EvseRapiProcessor::setWifiMode(uint8_t mode)
{
  sprintf(g_sTmp,"%cWF %02x",ESRAPI_SOC,(int)mode);
  appendChk(g_sTmp);
  writeStart();
  write(g_sTmp);
  writeEnd();
}
#endif // RAPI_WF

#ifdef RAPI_BTN
void EvseRapiProcessor::sendButtonPress(uint8_t long_press)
{
  sprintf(g_sTmp,"%cAN %d", ESRAPI_SOC, long_press);
  appendChk(g_sTmp);
  writeStart();
  write(g_sTmp);
  writeEnd();
}
#endif // RAPI_BTN

int EvseRapiProcessor::tokenize(char *buf)
{
  tokens[0] = &buf[1];
  char *s = &buf[2];
  tokenCnt = 1;
  uint8_t achkSum = ESRAPI_SOC + buf[1];
  uint8_t xchkSum = ESRAPI_SOC ^ buf[1];
  uint8_t hchkSum;
  uint8_t chktype=0; // 0=none,1=additive,2=xor
  while (*s) {
    if (*s == ' ') {
      if (tokenCnt >= ESRAPI_MAX_ARGS) {
	chktype = 255;
	break;
      }
      else {
	achkSum += *s;
	xchkSum ^= *s;
	*s = '\0';
	tokens[tokenCnt++] = ++s;
      }
    }
    else if ((*s == '*') ||// additive checksum
	     (*s == '^')) { // XOR checksum
      if (*s == '*') chktype = 1;
      else if (*s == '^') chktype = 2;
      *(s++) = '\0';
      hchkSum = htou8(s);
      break;
    }
    else {
      achkSum += *s;
      xchkSum ^= *(s++);
    }
  }

  int rc = ((chktype == 0) ||
	   ((chktype == 1) && (hchkSum == achkSum)) ||
 	   ((chktype == 2) && (hchkSum == xchkSum))) ? 0 : 1;
  if (rc) tokenCnt = 0;
  //  sprintf(g_sTmp,"trc: %d",rc);
  //  g_EIRP.writeStr(g_sTmp);

  return rc;
}

uint8_t g_inRapiCommand = 0;
int EvseRapiProcessor::processCmd()
{
  g_inRapiCommand = 1;

  UNION4B u1,u2,u3,u4;
  int rc = -1;

#ifdef RAPI_SENDER
  // throw away extraneous responses that we weren't expecting
  // these could be from commands that we already timed out
  if (isRespToken()) {
    g_inRapiCommand = 0;
    return rc;
  }
#endif // RAPI_SENDER

  curReceivedSeqId = INVALID_SEQUENCE_ID;
  const char *seqtoken = tokens[tokenCnt-1];
  if ((tokenCnt > 1) && (*seqtoken == ESRAPI_SOS)) {
    curReceivedSeqId = htou8(++seqtoken);
    tokenCnt--;
  }

  // we use bufCnt as a flag in response() to signify data to write
  bufCnt = 0;

  char *s = tokens[0];
  switch(*(s++)) {
  case 'F': // function
    switch(*s) {
    case '0': // enable/disable LCD update
      if (tokenCnt == 2) {
	g_OBD.DisableUpdate((*tokens[1] == '0') ? 1 : 0);
	if (*tokens[1] != '0') g_OBD.Update(OBD_UPD_FORCE);
	rc = 0;
      }
      break;
 #ifdef BTN_MENU
    case '1': // simulate front panel short press
      g_BtnHandler.DoShortPress(g_EvseController.InFaultState());
      g_OBD.Update(OBD_UPD_FORCE);
      rc = 0;
      break;
#endif // BTN_MENU
#ifdef LCD16X2
    case 'B': // LCD backlight
      if (tokenCnt == 2) {
	g_OBD.LcdSetBacklightColor(dtou32(tokens[1]));
	rc = 0;
      }
      break;
#endif // LCD16X2
    case 'D': // disable EVSE
      g_EvseController.Disable();
      rc = 0;
      break;
    case 'E': // enable EVSE
      g_EvseController.Enable();
      rc = 0;
      break;
    case 'F': // enable/disable feature
      if (tokenCnt == 3) {
	u1.u8 = (uint8_t)(*tokens[2] - '0');
	if (u1.u8 <= 1) {
	  rc = 0;
	  switch(*tokens[1]) {
#ifdef BTN_MENU
	  case 'B': // front button enable
	    g_EvseController.ButtonEnable(u1.u8);
	    break;
#endif // BTN_MENU
	  case 'D': // diode check
	    g_EvseController.EnableDiodeCheck(u1.u8);
	    break;
	  case 'E': // command echo
	    echo = ((u1.u8 == '0') ? 0 : 1);
	    break;
#ifdef ADVPWR
	  case 'F': // GFI self test
	    g_EvseController.EnableGfiSelfTest(u1.u8);
	    break;
	  case 'G': // ground check
	    g_EvseController.EnableGndChk(u1.u8);
	    break;
	  case 'R': // stuck relay check
	    g_EvseController.EnableStuckRelayChk(u1.u8);
	    break;
#endif // ADVPWR
#ifdef TEMPERATURE_MONITORING
	  case 'T': // temperature monitoring
	    g_EvseController.EnableTempChk(u1.u8);
	    break;
#endif // TEMPERATURE_MONITORING
	  case 'V': // vent required check
	    g_EvseController.EnableVentReq(u1.u8);
	    break;
	  default: // unknown
	    rc = -1;
	  }
	}
      }
      break;
#ifdef LCD16X2
    case 'P': // print to LCD
      if ((tokenCnt >= 4) && !g_EvseController.InHardFault()) {
	u1.u = dtou32(tokens[1]); // x
	u2.u = dtou32(tokens[2]); // y
	// now restore the spaces that were replaced w/ nulls by tokenizing
	for (u3.i=4;u3.i < tokenCnt;u3.i++) {
	  *(tokens[u3.i]-1) = ' ';
	}
	g_OBD.LcdPrint(u1.u,u2.u,tokens[3]);
	rc = 0;
      }
      break;
#endif // LCD16X2
    case 'R': // reset EVSE
      g_EvseController.Reboot();
      rc = 0;
      break;
    case 'S': // sleep
      g_EvseController.Sleep();
      rc = 0;
      break;
    }
    break;

  case 'S': // set parameter
    switch(*s) {
#ifdef LCD16X2
    case '0': // set LCD type
      if (tokenCnt == 2) {
#ifdef RGBLCD
	rc = g_EvseController.SetBacklightType((*tokens[1] == '0') ? BKL_TYPE_MONO : BKL_TYPE_RGB);
#endif // RGBLCD
      }
      break;
#endif // LCD16X2
#ifdef RTC
    case '1': // set RTC
      if (tokenCnt == 7) {
	extern void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s);
	SetRTC(dtou32(tokens[1]),dtou32(tokens[2]),dtou32(tokens[3]),
	       dtou32(tokens[4]),dtou32(tokens[5]),dtou32(tokens[6]));
	rc = 0;
      }
      break;
#endif // RTC
#if defined(AMMETER) && defined(ECVF_AMMETER_CAL)
    case '2': // ammeter calibration mode
      if (tokenCnt == 2) {
	g_EvseController.EnableAmmeterCal((*tokens[1] == '1') ? 1 : 0);
	rc = 0;
      }
      break;
#endif // AMMETER && ECVF_AMMETER_CAL
#ifdef TIME_LIMIT
    case '3': // set time limit
      if (tokenCnt == 2) {
	if (g_EvseController.LimitsAllowed()) {
	  g_EvseController.SetTimeLimit15(dtou32(tokens[1]));
	  if (!g_OBD.UpdatesDisabled()) g_OBD.Update(OBD_UPD_FORCE);
	  rc = 0;
	}
      }
      break;
#endif // TIME_LIMIT
#if defined(AUTH_LOCK) && !defined(AUTH_LOCK_REG)
    case '4': // auth lock
      if (tokenCnt == 2) {
	g_EvseController.AuthLock((int8_t)dtou32(tokens[1]),1);
	rc = 0;
      }
      break;
#endif // AUTH_LOCK && !AUTH_LOCK_REG
#ifdef MENNEKES_LOCK
    case '5': // mennekes setting
      if (tokenCnt == 2) {
	rc = 0;
        switch(*tokens[1]) {
        case '0':
          g_EvseController.UnlockMennekes();
          break;
        case '1':
          g_EvseController.LockMennekes();
          break;
        case 'A':
          g_EvseController.ClrMennekesManual();
          break;
        case 'M':
          g_EvseController.SetMennekesManual();
          break;
        default:
          rc = 1;
        }
      }
      break;
#endif // MENNEKES_LOCK
#ifdef AMMETER
    case 'A':
      if (tokenCnt == 3) {
	g_EvseController.SetCurrentScaleFactor(dtou32(tokens[1]));
	g_EvseController.SetAmmeterCurrentOffset(dtou32(tokens[2]));
	rc = 0;
      }
      break;
#endif // AMMETER
#ifdef BOOTLOCK
    case 'B':
      if (g_EvseController.InFaultState()) {
        strcpy(buffer,"1");
      }
      else {
        g_EvseController.ClearBootLock();
        strcpy(buffer,"0");
      }
      bufCnt=1;
      rc = 0;
      break;
#endif // BOOTLOCK
    case 'C': // current capacity
      if ((tokenCnt == 2) || (tokenCnt == 3)) {
	u2.u8 = dtou32(tokens[1]);
	if ((tokenCnt == 3) && (*tokens[2] == 'M')) {
	  rc = g_EvseController.SetMaxHwCurrentCapacity(u2.u8);
	  sprintf(buffer,"%d",(int)g_EvseController.GetMaxHwCurrentCapacity());
	}
	else {
	  if (tokenCnt == 3) {
	    // just make volatile no matter what character specified
	    u1.u8 = 1; // nosave = 1
	  }
	  else {
	    u1.u8 = 0; // nosave = 0
	  }
#ifdef TEMPERATURE_MONITORING
	  if (g_TempMonitor.OverTemperature() &&
	      (u2.u8 > g_EvseController.GetCurrentCapacity())) {
	    // don't allow raising current capacity during
	    // overtemperature event
	    rc = 1;
	  }
	  else {
	    rc = g_EvseController.SetCurrentCapacity(u2.u8,1,u1.u8);
	  }
#else // !TEMPERATURE_MONITORING
	  rc = g_EvseController.SetCurrentCapacity(u2.u8,1,u1.u8);
#endif // TEMPERATURE_MONITORING
  
	  sprintf(buffer,"%d",(int)g_EvseController.GetCurrentCapacity());
	}
	bufCnt = 1; // flag response text output
      }
      break;
#ifdef CHARGE_LIMIT
    case 'H': // cHarge limit
      if (tokenCnt == 2) {
	if (g_EvseController.LimitsAllowed()) {
	  g_EvseController.SetChargeLimitkWh(dtou32(tokens[1]));
	  if (!g_OBD.UpdatesDisabled()) g_OBD.Update(OBD_UPD_FORCE);
	  rc = 0;
	}
      }
      break;
#endif // CHARGE_LIMIT
#ifdef KWH_RECORDING
    case 'K': // set accumulated kwh
      g_EnergyMeter.SetTotkWh(dtou32(tokens[1]));
      g_EnergyMeter.SaveTotkWh();
      rc = 0;
      break;
#endif //KWH_RECORDING
    case 'L': // service level
      if (tokenCnt == 2) {
      switch(*tokens[1]) {
	case '1':
	case '2':
	  g_EvseController.SetSvcLevel(*tokens[1] - '0',1);
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
	  g_EvseController.EnableAutoSvcLevel(0);
#endif
	  rc = 0;
	  break;
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
	case 'A':
	  g_EvseController.EnableAutoSvcLevel(1);
	  rc = 0;
	  break;
#endif // ADVPWR && AUTOSVCLEVEL
	}
      }
      break;
#ifdef VOLTMETER
    case 'M':
      if (tokenCnt == 3) {
        g_EvseController.SetVoltmeter(dtou32(tokens[1]),dtou32(tokens[2]));
	rc = 0;
      }
      break;
#endif // VOLTMETER

#ifdef OEV6
  case 'R':   //Relay2 enable
    if (tokenCnt == 2) {
      if ( (uint8_t)dtou32(tokens[1]) == 1 ) {
        g_EvseController.Relay2Enable(1);
      } else {
        g_EvseController.Relay2Enable(0);
      }
    }
    rc = 0;
    break;
#endif //OEV6

#ifdef DELAYTIMER     
    case 'T': // timer
      if (tokenCnt == 5) {
	extern DelayTimer g_DelayTimer;
	u1.u8 = (uint8_t)dtou32(tokens[1]);
	u2.u8 = (uint8_t)dtou32(tokens[2]);
	u3.u8 = (uint8_t)dtou32(tokens[3]);
	u4.u8 = (uint8_t)dtou32(tokens[4]);
	if ((u1.u8 == 0) && (u2.u8 == 0) && (u3.u8 == 0) && (u4.u8 == 0)) {
	  g_DelayTimer.Disable();
	}
	else {
	  g_DelayTimer.SetStartTimer(u1.u8,u2.u8);
	  g_DelayTimer.SetStopTimer(u3.u8,u4.u8);
	  g_DelayTimer.Enable();
	}
	rc = 0;
      }
      break;
#endif // DELAYTIMER      

#if defined(KWH_RECORDING) && !defined(VOLTMETER)
    case 'V': // set voltage
      if (tokenCnt == 2) {
        g_EvseController.SetMV(dtou32(tokens[1]));
	rc = 0;
      }
      break;
#endif //defined(KWH_RECORDING) && !defined(VOLTMETER)

#ifdef HEARTBEAT_SUPERVISION
    case 'Y': // HEARTBEAT SUPERVISION
      if (tokenCnt == 1)  { //This is a heartbeat
        rc = g_EvseController.HsPulse(); //pet the dog
      }
      else if (tokenCnt == 3) { //This is a full HEARTBEAT_SUPERVISION setpoint command with both parameters
	    rc = 0;
        u1.u16 = (uint16_t)dtou32(tokens[1]);	// HS Interval in seconds.  0 = disabled
        u2.u8 = (uint8_t)dtou32(tokens[2]);	// HS fallback current, in amperes 
		if (u1.u16 == 0) { //Test for deactivation {
          rc = g_EvseController.HsRestoreAmpacity();
		}
		rc |= g_EvseController.HeartbeatSupervision(u1.u16, u2.u8);
      }
      else if (tokenCnt == 2) { //This is a command to ack a heartbeat supervision miss
        u1.u8 = (uint8_t)dtou32(tokens[1]); //Magic cookie
        rc = g_EvseController.HsAckMissedPulse(u1.u8);
      }
      else { //Invalid number of tokens, return 1
        rc = 1; //Invalid number of tokens
      }
      sprintf(buffer,"%d %d %d", g_EvseController.GetHearbeatInterval(), g_EvseController.GetHearbeatCurrent(), g_EvseController.GetHearbeatTrigger());
      bufCnt = 1; 
      break;
#endif //HEARTBEAT_SUPERVISION

    }
    break;

  case 'G': // get parameter
    switch(*s) {
    case '0': // get EV connect state
      {
	uint8_t connstate;
	if (g_EvseController.GetPilot()->GetState() == PILOT_STATE_N12) {
	  connstate = 2; // unknown
	}
	else {
	  if (g_EvseController.EvConnected()) connstate = 1;
	  else connstate = 0;
	}
	sprintf(buffer,"%d",(int)connstate);
      }
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#ifdef TIME_LIMIT
    case '3': // get time limit
      sprintf(buffer,"%d",(int)g_EvseController.GetTimeLimit15());
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // TIME_LIMIT
#if defined(AUTH_LOCK) && !defined(AUTH_LOCK_REG)
    case '4': // get auth lock
      sprintf(buffer,"%d",(int)g_EvseController.AuthLockIsOn() ? 1 : 0);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // AUTH_LOCK && !AUTH_LOCK_REG
#ifdef MENNEKES_LOCK
    case '5': // get mennekes setting
      sprintf(buffer,"%d %c",g_EvseController.MennekesIsLocked(),
              g_EvseController.MennekesIsManual() ? 'M' : 'A');
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // MENNEKES_LOCK
#ifdef AMMETER
    case 'A':
      u1.i = g_EvseController.GetCurrentScaleFactor();
      u2.i = g_EvseController.GetAmmeterCurrentOffset();
      sprintf(buffer,"%d %d",u1.i,u2.i);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // AMMETER
    case 'C': // get current capacity range
      u1.i = MIN_CURRENT_CAPACITY_J1772;
      if (g_EvseController.GetCurSvcLevel() == 2) {
	u2.i = g_EvseController.GetMaxHwCurrentCapacity();
      }
      else {
	u2.i = MAX_CURRENT_CAPACITY_L1;
      }
      u3.i = g_EvseController.GetCurrentCapacity();
      u4.i = g_EvseController.GetMaxCurrentCapacity();
      sprintf(buffer,"%d %d %d %d",u1.i,u2.i,u3.i,u4.i);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#ifdef DELAYTIMER
    case 'D': // get delay timer
      extern DelayTimer g_DelayTimer;
      if (g_DelayTimer.IsTimerEnabled()) {
	u1.i = g_DelayTimer.GetStartTimerHour();
	u2.i = g_DelayTimer.GetStartTimerMin();
	u3.i = g_DelayTimer.GetStopTimerHour();
	u4.i = g_DelayTimer.GetStopTimerMin();
      }
      else {
	u1.i = 0;
	u2.i = 0;
	u3.i = 0;
	u4.i = 0;
      }
      sprintf(buffer,"%d %d %d %d",u1.i,u2.i,u3.i,u4.i);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // DELAYTIMER
    case 'E': // get settings
      u1.u = g_EvseController.GetCurrentCapacity();
      u2.u = g_EvseController.GetFlags();
      sprintf(buffer,"%d %04x",u1.u,u2.u);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
    case 'F': // get fault counters
#ifdef GFI
      u1.u = g_EvseController.GetGfiTripCnt();
#else
      u1.u = 0;
#endif // GFI
#ifdef ADVPWR
      u2.u = g_EvseController.GetNoGndTripCnt();
      u3.u = g_EvseController.GetStuckRelayTripCnt();
#else
	  u2.u = 0;
	  u3.u = 0;
#endif // ADVPWR
      sprintf(buffer,"%x %x %x",u1.u,u2.u,u3.u);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#if defined(AMMETER)||defined(VOLTMETER)
    case 'G':
      u1.i32 = g_EvseController.GetChargingCurrent();
      u2.i32 = (int32_t)g_EvseController.GetVoltage();
      sprintf(buffer,"%ld %ld",u1.i32,u2.i32);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // AMMETER || VOLTMETER
#ifdef CHARGE_LIMIT
    case 'H': // get cHarge limit
      sprintf(buffer,"%d",(int)g_EvseController.GetChargeLimitkWh());
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // CHARGE_LIMIT
#ifdef MCU_ID_LEN
    case 'I': // get MCU ID
      {
        uint8_t mcuid[MCU_ID_LEN];
        getMcuId(mcuid);
        char *s = buffer;
        *(s++) = ' ';
        for (int i=0;i < 6;i++) {
          *(s++) = mcuid[i];
        }
        for (int i=6;i < MCU_ID_LEN;i++) {
          sprintf(s,"%02X",mcuid[i]);
          s += 2;
        }
        bufCnt = 1; // flag response text output
        rc = 0;
      }
      break;
#endif // MCU_ID_LEN
#ifdef VOLTMETER
    case 'M':
      u1.i = g_EvseController.GetVoltScaleFactor();
      u2.i32 = g_EvseController.GetVoltOffset();
      sprintf(buffer,"%d %ld",u1.i,u2.i32);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // VOLTMETER
#ifdef TEMPERATURE_MONITORING
#ifdef TEMPERATURE_MONITORING_NY
    case 'O':
      u1.i = g_TempMonitor.m_ambient_thresh;
      u2.i = g_TempMonitor.m_ir_thresh;
      sprintf(buffer,"%d %d",u1.i,u2.i);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // TEMPERATURE_MONITORING_NY
    case 'P':
      sprintf(buffer,"%d %d %d",(int)g_TempMonitor.m_DS3231_temperature,
	      (int)g_TempMonitor.m_MCP9808_temperature,
	      (int)g_TempMonitor.m_TMP007_temperature);
      /* this is bigger than using sprintf
      strcpy(buffer,u2a(g_TempMonitor.m_DS3231_temperature));
      strcat(buffer,g_sSpace);
      strcat(buffer,u2a(g_TempMonitor.m_MCP9808_temperature));
      strcat(buffer,g_sSpace);
      strcat(buffer,u2a(g_TempMonitor.m_TMP007_temperature));
      */
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // TEMPERATURE_MONITORING
    case 'S': // get state
      u1.u8 = g_EvseController.GetState();
      u2.u8 = g_EvseController.GetPilotState();
      u3.u16 = g_EvseController.GetVFlags();
      sprintf(buffer,"%02x %ld %02x %04x",u1.u8,g_EvseController.GetElapsedChargeTime(),u2.u8,u3.u16);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#ifdef RTC
    case 'T': // get time
      extern void GetRTC(char *buf);
      GetRTC(buffer);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // RTC
#ifdef KWH_RECORDING
    case 'U':
      sprintf(buffer,"%lu %lu",g_EnergyMeter.GetSessionWs(),g_EnergyMeter.GetTotkWh());
      bufCnt = 1;
      rc = 0;
      break;
#endif // KWH_RECORDING
    case 'V': // get version
      GetVerStr(buffer);
      strcat(buffer," ");
      strcat_P(buffer,RAPI_VER);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
	  
#ifdef HEARTBEAT_SUPERVISION
    case 'Y': // HEARTBEAT SUPERVISION
	  sprintf(buffer,"%d %d %d", g_EvseController.GetHearbeatInterval(), g_EvseController.GetHearbeatCurrent(), g_EvseController.GetHearbeatTrigger());
      bufCnt = 1; 
	  rc = 0;
      break;
#endif //HEARTBEAT_SUPERVISION
	   
    }
    break;

#ifdef RAPI_T_COMMANDS
  case 'T': // testing op
    switch(*s) {
#ifdef FAKE_CHARGING_CURRENT
    case '0': // set fake charging current
      if (tokenCnt == 2) {
	g_EvseController.SetChargingCurrent(dtou32(tokens[1])*1000);
	g_OBD.SetAmmeterDirty(1);
	g_OBD.Update(OBD_UPD_FORCE);
	rc = 0;
      }
      break;
#endif // FAKE_CHARGING_CURRENT
    }
    break;
#endif //RAPI_T_COMMANDS
#if defined(RELAY_HOLD_DELAY_TUNING)
  case 'Z': // reserved op
    switch(*s) {
    case '0': // set relayCloseMs
      if (tokenCnt == 3) {
	u1.u8 = dtou32(tokens[1]);
	u2.u8 = dtou32(tokens[2]);
	g_EvseController.setPwmPinParms(u1.u8,u2.u8);
	sprintf(g_sTmp,"\nZ0 %u %u",(unsigned)u1.u8,(unsigned)u2.u8);
	Serial.println(g_sTmp);
	eeprom_write_byte((uint8_t*)EOFS_RELAY_CLOSE_MS,u1.u8);
	eeprom_write_byte((uint8_t*)EOFS_RELAY_HOLD_PWM,u2.u8);
      }
      rc = 0;
      break;

    }
    break;
#endif // RELAY_HOLD_DELAY_TUNING

  default:
    ; // do nothing
  }

  if (bufCnt != -1){
    response((rc == 0) ? 1 : 0);
  }

  reset();

  g_inRapiCommand = 0;

  // command might have changed EVSE state
  RapiSendEvseState();

  return rc;
}

// append
void EvseRapiProcessor::appendChk(char *buf)
{
  char *s = buf;
  uint8_t chk = 0;
  while (*s) {
    chk ^= *(s++);
  }
  sprintf(s,"^%02X",(unsigned)chk);
  s[3] = ESRAPI_EOC;
  s[4] = '\0';
}


void EvseRapiProcessor::response(uint8_t ok)
{
  writeStart();

  sprintf(g_sTmp,"%c%s",ESRAPI_SOC,ok ? "OK" : "NK");
  if (bufCnt) {
    strcat(g_sTmp," ");
    strcat(g_sTmp,buffer);
  }
  if (curReceivedSeqId != INVALID_SEQUENCE_ID) {
    appendSequenceId(g_sTmp,curReceivedSeqId);
  }
  appendChk(g_sTmp);
  write(g_sTmp);
  if (echo) write('\n');

  writeEnd();
}

void EvseRapiProcessor::appendSequenceId(char *s,uint8_t seqId)
{
  sprintf(s+strlen(s)," %c%02X",ESRAPI_SOS,seqId);
}

#ifdef RAPI_SENDER
uint8_t EvseRapiProcessor::getSendSequenceId()
{
  if (++curSentSeqId == INVALID_SEQUENCE_ID) ++curSentSeqId;
  return curSentSeqId;
}

int8_t EvseRapiProcessor::isAsyncToken()
{
  if ((*tokens[0] == 'A') ||
      !strcmp(tokens[0],"WF") ||
      (!strcmp(tokens[0],"ST") && (tokenCnt == 2))) {
    return 1;
  }
  else {
    return 0;
  }
}

// OK or NK
int8_t EvseRapiProcessor::isRespToken()
{
  const char *token = tokens[0];
  if ((strlen(token) == 2) && (token[1] == 'K') &&
      ((*token == 'O') || (*token == 'N'))) {
    return 1;
  }
  else {
    return 0;
  }
}


void EvseRapiProcessor::_sendCmd(const char *cmdstr)
{
  *sendbuf = ESRAPI_SOC;
  strcpy(sendbuf+1,cmdstr);
  appendSequenceId(sendbuf,getSendSequenceId());
  appendChk(sendbuf);
  writeStart();
  write(sendbuf);
  writeEnd();
}

int8_t EvseRapiProcessor::receiveResp(unsigned long msstart)
{
  tokenCnt = 0;
  *sendbuf = 0;
  int bufpos = 0;

  // wait for response
  do {
    WDT_RESET();
    int bytesavail = available();
    if (bytesavail) {
      for (int i=0;i < bytesavail;i++) {
	char c = read();

	if (!bufpos && c != ESRAPI_SOC) {
	  // wait for start character
	  continue;
	}
	else if (c == ESRAPI_EOC) {
	  sendbuf[bufpos] = '\0';
	  if (!tokenize(sendbuf)) return 0;
	  else return 1;
	  
	}
	else {
	  sendbuf[bufpos++] = c;
	  if (bufpos >= (RAPIS_BUFLEN-1)) return 2;
	}
      }
    }
  } while (!tokenCnt && ((millis() - msstart) < RAPIS_TIMEOUT_MS));

  return -1;
}

int8_t EvseRapiProcessor::sendCmd(const char *cmdstr)
{
  _sendCmd(cmdstr);

  unsigned long msstart = millis();
 start:
  while (receiveResp(msstart) > 0) WDT_RESET();
  if (tokenCnt) {
    uint8_t seqId = INVALID_SEQUENCE_ID;
    const char *seqtoken = tokens[tokenCnt-1];
    if ((tokenCnt > 1) && isRespToken() && (*seqtoken == ESRAPI_SOS)) {
      seqId = htou8(++seqtoken);
      tokenCnt--;
    }
    if (!strcmp(tokens[0],"OK")
	&& (seqId == curSentSeqId)
	) {
      return 0;
    }
    else if (!strcmp(tokens[0],"NK")
	     && (seqId == curSentSeqId)
	     ) {
      return 1;
    }
    else { // command or async notification received - process it
      processCmd();
      msstart = millis();
      goto start;
    }
  }
  else {
    return -1;
  }
}

#endif // RAPI_SENDER


#ifdef RAPI_SERIAL
EvseSerialRapiProcessor::EvseSerialRapiProcessor()
{
}

void EvseSerialRapiProcessor::init()
{
  EvseRapiProcessor::init();
}
#endif // RAPI_SERIAL


#ifdef RAPI_I2C

EvseI2cRapiProcessor::EvseI2cRapiProcessor()
{
}

void EvseI2cRapiProcessor::init()
{
  Wire.begin(RAPI_I2C_LOCAL_ADDR);
  Wire.onReceive(receiveEvent);   // define the receive function for receiving data from master

  EvseRapiProcessor::init();
}


#endif // RAPI_I2C

#ifdef RAPI_SERIAL
EvseSerialRapiProcessor g_ESRP;
#endif
#ifdef RAPI_I2C
EvseI2cRapiProcessor g_EIRP;
#endif

void RapiInit()
{
#ifdef RAPI_SERIAL
  g_ESRP.init();
#ifdef GPPBUGKLUDGE
  static char g_rapiSerialBuffer[ESRAPI_BUFLEN];
  g_ESRP.setBuffer(g_rapiSerialBuffer);
#endif // GPPBUGKLUDGE
#endif // RAPI_SERIAL
#ifdef RAPI_I2C
  g_EIRP.init();
#ifdef GPPBUGKLUDGE
  static char g_rapiI2ClBuffer[ESRAPI_BUFLEN];
  g_ESRP.setBuffer(g_rapiI2CBuffer);
#endif // GPPBUGKLUDGE
#endif // RAPI_I2C
}

void RapiDoCmd()
{
#ifdef RAPI_SERIAL
  g_ESRP.doCmd();
#endif
#ifdef RAPI_I2C
  // kludge - delay below is needed when RapiDoCmd() is running in a tight loop
  // such as during a hard fault. without the delay, I2C can't receive characters
#define RDCDELAY 6
#ifdef RDCDELAY
  static unsigned long lastdocmd;
  unsigned long msnow = millis();
  if ((msnow-lastdocmd) < RDCDELAY) {
    delay(RDCDELAY - (msnow-lastdocmd));
  }
  lastdocmd = msnow;
#endif // RDCDELAY

  g_EIRP.doCmd();
#endif // RAPI_I2C
}

// return: 0=sent 
//         1=nothing changed, didn't send
//         2=in processCmd(), didn't send
uint8_t RapiSendEvseState(uint8_t force)
{
  static uint8_t evseStateSent = EVSE_STATE_UNKNOWN;
  static uint8_t pilotStateSent = EVSE_STATE_UNKNOWN;
  static uint8_t currentCapacitySent = 0;
  static uint16_t vFlagsSent = 0;

  if (!g_inRapiCommand) {
    uint8_t evseState = g_EvseController.GetState();
    uint8_t pilotState = g_EvseController.GetPilotState();
    uint8_t currentCapacity = g_EvseController.GetCurrentCapacity();
    uint16_t vFlags = g_EvseController.GetVFlags() & ECVF_CHANGED_TEST;
    if (force ||
	((evseState != EVSE_STATE_UNKNOWN) && 
	 !((evseState == EVSE_STATE_A) && (vFlags & ECVF_EV_CONNECTED)) &&
	 !((evseState == EVSE_STATE_B) && !(vFlags & ECVF_EV_CONNECTED)) &&
	 !((evseState == EVSE_STATE_C) && !(vFlags & ECVF_EV_CONNECTED)) &&
	 ((evseStateSent != evseState) ||
	  (pilotStateSent != pilotState) ||
	  (currentCapacitySent != currentCapacity) ||
	  (vFlagsSent != vFlags)))) {
#ifdef RAPI_SERIAL
      g_ESRP.sendEvseState();
#endif
#ifdef RAPI_I2C
      g_EIRP.sendEvseState();
#endif
      evseStateSent = evseState;
      pilotStateSent = pilotState;
      currentCapacitySent = currentCapacity;
      vFlagsSent = vFlags;
      return 0;
    }

    return 1;
  }
  else return 2;
}

void RapiSendBootNotification()
{
#ifdef RAPI_SERIAL
  g_ESRP.sendBootNotification();
#endif
#ifdef RAPI_I2C
  g_EIRP.sendBootNotification();
#endif
}

#ifdef RAPI_WF
void RapiSetWifiMode(uint8_t mode)
{
#ifdef RAPI_SERIAL
  g_ESRP.setWifiMode(mode);
#endif
#ifdef RAPI_I2C
  g_EIRP.setWifiMode(mode);
#endif
}
#endif // RAPI_WF

#ifdef RAPI_BTN
void RapiSendButtonPress(uint8_t long_press)
{
#ifdef RAPI_SERIAL
  g_ESRP.sendButtonPress(long_press);
#endif
#ifdef RAPI_I2C
  g_EIRP.sendButtonPress(long_press);
#endif
}
#endif // RAPI_WF
#endif // RAPI
