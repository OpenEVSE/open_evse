#pragma once
/*
 * This file is part of Open EVSE.
 *
 * Copyright (c) 2011-2023 Sam C. Lin
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


// EVSE states for m_EvseState
#define EVSE_STATE_UNKNOWN 0x00
#define EVSE_STATE_A       0x01 // vehicle state A 12V - not connected
#define EVSE_STATE_B       0x02 // vehicle state B 9V - connected, ready
#define EVSE_STATE_C       0x03 // vehicle state C 6V - charging
#define EVSE_STATE_D       0x04 // vehicle state D 3V - vent required
#define EVSE_FAULT_STATE_BEGIN EVSE_STATE_D
#define EVSE_STATE_DIODE_CHK_FAILED 0x05 // diode check failed
#define EVSE_STATE_GFCI_FAULT 0x06       // GFCI fault
#define EVSE_STATE_NO_GROUND 0x07 //bad ground
#define EVSE_STATE_STUCK_RELAY 0x08 //stuck relay
#define EVSE_STATE_GFI_TEST_FAILED 0x09 // GFI self-test failure
#define EVSE_STATE_OVER_TEMPERATURE 0x0A // over temperature error shutdown
#define EVSE_STATE_OVER_CURRENT 0x0B // over current error shutdown
//reserved #define EVSE_STATE_PILOT_ERROR 0x0C // pilot self test error
//reserved #define EVSE_STATE_TEMP_SENSOR_FAULT 0x0D // temp sensor dead
#define EVSE_STATE_RELAY_CLOSURE_FAULT 0x0E
#define EVSE_FAULT_STATE_END EVSE_STATE_RELAY_CLOSURE_FAULT
           
#define EVSE_STATE_SLEEPING 0xfe // waiting for timer
#define EVSE_STATE_DISABLED 0xff // disabled

inline int8_t IsEvseFaultState(uint8_t state) {
  if ((state >= EVSE_FAULT_STATE_BEGIN) && (state <= EVSE_FAULT_STATE_END)) return 1;
  else return 0;
}

typedef struct threshdata {
  uint16_t m_ThreshAB; // state A -> B
  uint16_t m_ThreshBC; // state B -> C
  uint16_t m_ThreshCD; // state C -> D
  uint16_t m_ThreshD;  // state D
  uint16_t m_ThreshDS; // diode short
} THRESH_DATA,*PTHRESH_DATA;

typedef struct calibdata {
  uint16_t m_pMax;
  uint16_t m_pAvg;
  uint16_t m_pMin;
  uint16_t m_nMax;
  uint16_t m_nAvg;
  uint16_t m_nMin;
} CALIB_DATA,*PCALIB_DATA;

// called whenever EVSE wants to transition state from curEvseState to newEvseState
// return 0 to allow transition to newEvseState
// else return desired state
typedef uint8_t (*EvseStateTransitionReqFunc)(uint8_t prevPilotState,uint8_t curPilotState,uint8_t curEvseState,uint8_t newEvseState);

// J1772EVSEController m_wFlags bits - saved to EEPROM
#define ECF_L2                 0x0001 // service level 2
#define ECF_DIODE_CHK_DISABLED 0x0002 // no diode check
#define ECF_VENT_REQ_DISABLED  0x0004 // no vent required state
#define ECF_GND_CHK_DISABLED   0x0008 // no chk for ground fault
#define ECF_STUCK_RELAY_CHK_DISABLED 0x0010 // no chk for stuck relay
#define ECF_AUTO_SVC_LEVEL_DISABLED  0x0020 // auto detect svc level - requires ADVPWR
// Ability set the EVSE for manual button press to start charging - GoldServe
#define ECF_AUTO_START_DISABLED 0x0040  // no auto start charging
#define ECF_SERIAL_DBG         0x0080 // enable debugging messages via serial
#define ECF_MONO_LCD           0x0100 // monochrome LCD backlight
#define ECF_GFI_TEST_DISABLED  0x0200 // no GFI self test
#define ECF_TEMP_CHK_DISABLED  0x0400 // no Temperature Monitoring
#define ECF_CGMI               0x1000 // continuous GMI
#define ECF_BUTTON_DISABLED    0x8000 // front panel button disabled
#define ECF_DEFAULT            0x0000

// J1772EVSEController volatile m_wVFlags bits - not saved to EEPROM
#define ECVF_AUTOSVCLVL_SKIPPED 0x0001 // auto svc level test skipped during post
#define ECVF_HARD_FAULT         0x0002 // in non-autoresettable fault
#define ECVF_LIMIT_SLEEP        0x0004 // currently sleeping after reaching time/charge limit
#define ECVF_AUTH_LOCKED        0x0008 // locked pending authentication
#define ECVF_TIME_LIMIT         0x0010 // time limit set
#define ECVF_NOGND_TRIPPED      0x0020 // no ground has tripped at least once
#define ECVF_CHARGING_ON        0x0040 // charging relay is closed
#define ECVF_GFI_TRIPPED        0x0080 // gfi has tripped at least once since boot
#define ECVF_EV_CONNECTED       0x0100 // EV connected - valid only when pilot not N12
#define ECVF_SESSION_ENDED      0x0200 // used for charging session time calc
#define ECVF_EV_CONNECTED_PREV  0x0400 // prev EV connected flag
#define ECVF_UI_IN_MENU         0x0800 // onboard UI currently in a menu
#define ECVF_TIMER_ON           0x1000 // delay timer enabled
#define ECVF_CHARGE_LIMIT       0x2000
#define ECVF_BOOT_LOCK          0x4000 // locked at boot
#define ECVF_MENNEKES_MANUAL    0x8000 // Mennekes lock manual mode
#if defined(AUTH_LOCK) && (AUTH_LOCK != 0)
#define ECVF_DEFAULT            ECVF_AUTH_LOCKED|ECVF_SESSION_ENDED
#else
#define ECVF_DEFAULT            ECVF_SESSION_ENDED
#endif

// used for RapiSendEvseState
#define ECVF_CHANGED_TEST (ECVF_AUTH_LOCKED|ECVF_EV_CONNECTED|ECVF_TIME_LIMIT|ECVF_CHARGE_LIMIT|ECVF_HARD_FAULT)


#define HS_INTERVAL_DEFAULT     0x0000  //By default, on an unformatted EEPROM, Heartbeat Supervision is not activated
#define HS_IFALLBACK_DEFAULT    0x00    //By default, on an unformatted EEPROM, HS fallback current is 0 Amperes 
#define HS_ACK_COOKIE           0XA5    //ACK will not work unless it contains this cookie
#define HS_MISSEDPULSE_NOACK    0x02    //HEARTBEAT_SUPERVISION missed a pulse and this has not been acknowleged
#define HS_MISSEDPULSE          0x01    //HEARTBEAT_SUPERVISION missed a pulse and this is the semi-permanent record flag
//#define DEBUG_HS //Uncomment this for debugging statemnts sent to serial port


// acpinstate bits - when set, means pin has voltage detected
#define ACPIN1_OPEN 2
#define ACPIN2_OPEN 1
#define ACPINS_OPEN (ACPIN1_OPEN|ACPIN2_OPEN)
// for ECF_CGMI
#define GND_TEST_PIN_OPEN ACPIN2_OPEN
#define RLY_TEST_PIN_OPEN ACPIN1_OPEN

class J1772EVSEController {
  J1772Pilot m_Pilot;
#ifdef GFI
  Gfi m_Gfi;
  unsigned long m_GfiFaultStartMs;
  uint8_t m_GfiRetryCnt;
  uint8_t m_GfiTripCnt; // contains tripcnt-1
#endif // GFI
  AdcPin adcPilot;
#ifdef CURRENT_PIN
  AdcPin adcCurrent;
#endif
#ifdef VOLTMETER_PIN
  AdcPin adcVoltMeter;
#endif

#ifdef CHARGING_REG
  DigitalPin pinCharging;
#endif
#ifdef CHARGING2_REG
  DigitalPin pinCharging2;
#endif
#ifdef CHARGINGAC_REG
  DigitalPin pinChargingAC;
#endif
#ifdef ACLINE1_REG
  DigitalPin pinAC1;
#endif
#ifdef ACLINE2_REG
  DigitalPin pinAC2;
#endif
#ifdef SLEEP_STATUS_REG
  DigitalPin pinSleepStatus;
#endif
#ifdef AUTH_LOCK_REG
  DigitalPin pinAuthLock;
#endif
#ifdef ADVPWR
  unsigned long m_NoGndStart;
  uint8_t m_NoGndRetryCnt;
  uint8_t m_NoGndTripCnt; // contains tripcnt-1
  unsigned long m_StuckRelayStartTimeMS;
  uint8_t m_StuckRelayTripCnt; // contains tripcnt-1
#endif // ADVPWR
#ifdef RELAY_PWM
  uint8_t m_relayCloseMs; // #ms for DC pulse to close relay
  uint8_t m_relayHoldPwm; // PWM duty cycle to hold relay closed
#endif // RELAY_PWM
  uint16_t m_wFlags; // ECF_xxx
  uint16_t m_wVFlags; // ECVF_xxx
  static THRESH_DATA m_ThreshData;
  uint8_t m_EvseState;
  uint8_t m_PrevEvseState;
  uint8_t m_TmpEvseState;
  uint8_t m_TmpPilotState;
  uint8_t m_PilotState;
  unsigned long m_TmpEvseStateStart;
  unsigned long m_TmpPilotStateStart;
  uint8_t m_MaxHwCurrentCapacity; // max L2 amps that can be set
  uint8_t m_CurrentCapacity; // max amps we can output
  unsigned long m_ChargeOnTimeMS; // millis() when relay last closed
  unsigned long m_ChargeOffTimeMS; // millis() when relay last opened
  time_t m_ElapsedChargeTime;
  time_t m_ElapsedChargeTimePrev;
  time_t m_AccumulatedChargeTime;
#ifdef STATE_TRANSITION_REQ_FUNC
  EvseStateTransitionReqFunc m_StateTransitionReqFunc;
#endif // STATE_TRANSITION_REQ_FUNC
#ifdef MENNEKES_LOCK
  MennekesLock m_MennekesLock;
#endif // MENNEKES_LOCK
#ifdef OVERCURRENT_THRESHOLD
  unsigned long m_OverCurrentStartMs;
#endif // OVERCURRENT_THRESHOLD
#ifdef OEV6
  uint8_t m_isV6;
#endif


  void setFlags(uint16_t flags) { 
    m_wFlags |= flags; 
  }
  void clrFlags(uint16_t flags) { 
    m_wFlags &= ~flags; 
  }
  uint8_t flagIsSet(uint16_t flag) {
    return (m_wFlags & flag) ? 1 : 0;
  }
  void setVFlags(uint16_t flags) { 
    m_wVFlags |= flags; 
  }
  void clrVFlags(uint16_t flags) { 
    m_wVFlags &= ~flags; 
  }
  uint16_t getVFlags(uint16_t flags) {
    return m_wVFlags & flags;
  }
  uint8_t vFlagIsSet(uint16_t flag) {
    return (m_wVFlags & flag) ? 1 : 0;
  }

#ifdef OEV6
  uint8_t isV6() { return m_isV6; }
  uint8_t relay2Enable = 1;   //default to both relays enabled
#endif

#ifdef ADVPWR
// power states for doPost() (active low)
#define both 0
#define L1on 1
#define L2on 2
#define none 3
// service states for doPost()
#define UD 0 // undefined
#define L1 1 // L1
#define L2 2 // L2
#define OG 3 // open ground
#define SR 4 // stuck relay
#define FG 5 // GFI fault

  uint8_t doPost();
#endif // ADVPWR
  void chargingOn();
  void chargingOff();
  uint8_t chargingIsOn() { return vFlagIsSet(ECVF_CHARGING_ON); }

#ifdef TIME_LIMIT
  uint8_t m_timeLimit15; // increments of 15min to extend charge time
  time_t m_timeLimitEnd; // end time
#endif

#ifdef AMMETER
  unsigned long m_AmmeterReading;
  int32_t m_ChargingCurrent;
  int16_t m_AmmeterCurrentOffset;
  int16_t m_CurrentScaleFactor;
#ifdef CHARGE_LIMIT
  uint8_t m_chargeLimitkWh; // kWh to extend session
  uint32_t m_chargeLimitTotWs; // total Ws limit
#endif

  void readAmmeter();
#endif // AMMETER
#ifdef VOLTMETER
  uint16_t m_VoltScaleFactor;
  uint32_t m_VoltOffset;
#endif // VOLTMETER
  uint32_t m_Voltage; // mV

#ifdef HEARTBEAT_SUPERVISION
  uint16_t      m_HsInterval;   // Number of seconds HS will wait for a heartbeat before reducing ampacity to m_IFallback.  If 0 disable.
  uint8_t       m_IFallback;    // HEARTBEAT_SUPERVISION fallback current in Amperes.  
  uint8_t       m_HsTriggered;  // Will be 0 if HEARTBEAT_SUPERVISION has never had a missed pulse
  unsigned long m_HsLastPulse;  // The last time we saw a HS pulse or the last time m_HsInterval elpased without seeing one.  Set to 0 if HEARTBEAT_SUPERVISION triggered                                   
#endif //HEARTBEAT_SUPERVISION

public:
  J1772EVSEController();
  void Init();
  void Update(uint8_t forcetransition=0); // read sensors
  void Enable();
  void Disable(); // panic stop - open relays abruptly
  void Sleep(); // graceful stop - e.g. waiting for timer to fire- give the EV time to stop charging first
  void Relay2Enable(uint8_t relay2on) { relay2Enable = relay2on; }

  uint16_t GetFlags() { return m_wFlags; }
  uint16_t GetVFlags() { return m_wVFlags; }

  uint8_t GetState() { 
    return m_EvseState; 
  }
  uint8_t GetPrevState() { 
    return m_PrevEvseState; 
  }
  int StateTransition() { 
    return (m_EvseState != m_PrevEvseState) ? 1 : 0; 
  }

  void SaveEvseFlags() {
    eeprom_write_word((uint16_t *)EOFS_FLAGS,m_wFlags);
  }

  int8_t InFaultState() {
    return ((m_EvseState >= EVSE_FAULT_STATE_BEGIN) && (m_EvseState <= EVSE_FAULT_STATE_END));
  }

#ifdef RELAY_PWM
  void setPwmPinParms(uint8_t delayms,uint8_t pwm) {
    m_relayCloseMs = delayms;
    m_relayHoldPwm = pwm;
  }
#endif // RELAY_PWM

  void SetHardFault() { setVFlags(ECVF_HARD_FAULT); }
  void ClrHardFault() { clrVFlags(ECVF_HARD_FAULT); }
  int8_t InHardFault() { return vFlagIsSet(ECVF_HARD_FAULT); }
  int8_t CGMIisEnabled() { return flagIsSet(ECF_CGMI); }
  unsigned long GetResetMs();

  uint8_t SetMaxHwCurrentCapacity(uint8_t amps);
  uint8_t GetMaxHwCurrentCapacity() { return m_MaxHwCurrentCapacity; }
  uint8_t GetCurrentCapacity() { 
    return m_CurrentCapacity; 
  }
  uint8_t GetMaxCurrentCapacity();
  int SetCurrentCapacity(uint8_t amps,uint8_t updatelcd=0,uint8_t nosave=0);

  time_t GetElapsedChargeTime() { 
    return m_ElapsedChargeTime+m_AccumulatedChargeTime; 
  }
  time_t GetElapsedChargeTimePrev() { 
    return m_ElapsedChargeTimePrev; 
  }
  void Calibrate(PCALIB_DATA pcd);
  uint8_t GetCurSvcLevel() { 
    return (m_wFlags & ECF_L2) ? 2 : 1; 
  }
  void SetSvcLevel(uint8_t svclvl,uint8_t updatelcd=0);
  PTHRESH_DATA GetThreshData() { 
    return &m_ThreshData; 
  }
  uint8_t DiodeCheckEnabled() { 
    return (m_wFlags & ECF_DIODE_CHK_DISABLED) ? 0 : 1;
  }
  void EnableDiodeCheck(uint8_t tf);
  uint8_t VentReqEnabled() { 
    return (m_wFlags & ECF_VENT_REQ_DISABLED) ? 0 : 1;
  }
  void EnableVentReq(uint8_t tf);
  void SaveSettings();
#ifdef ADVPWR
  uint8_t GndChkEnabled() { 
    return (m_wFlags & ECF_GND_CHK_DISABLED) ? 0 : 1;
  }
  void EnableGndChk(uint8_t tf);
  void EnableStuckRelayChk(uint8_t tf);
  uint8_t StuckRelayChkEnabled() { 
    return (m_wFlags & ECF_STUCK_RELAY_CHK_DISABLED) ? 0 : 1;
  }
#ifdef AUTOSVCLEVEL
  uint8_t AutoSvcLevelEnabled() { return (m_wFlags & ECF_AUTO_SVC_LEVEL_DISABLED) ? 0 : 1; }
  uint8_t EnableAutoSvcLevel(uint8_t tf);
  void SetAutoSvcLvlSkipped(uint8_t tf) {
    if (tf) setVFlags(ECVF_AUTOSVCLVL_SKIPPED);
    else clrVFlags(ECVF_AUTOSVCLVL_SKIPPED);
  }
  uint8_t AutoSvcLvlSkipped() { return vFlagIsSet(ECVF_AUTOSVCLVL_SKIPPED); }
#else
  uint8_t AutoSvcLevelEnabled() { return 0; }
  void EnableAutoSvcLevel(uint8_t tf) {}
  void SetAutoSvcLvlSkipped(uint8_t tf) {}
  uint8_t AutoSvcLvlSkipped() { return 1; }
#endif // AUTOSVCLEVEL
  void SetNoGndTripped();
  uint8_t NoGndTripped() { return vFlagIsSet(ECVF_NOGND_TRIPPED); }



  uint8_t ReadACPins();
#endif // ADVPWR

  void ClearBootLock() {
    clrVFlags(ECVF_BOOT_LOCK);
  }
  uint8_t IsBootLocked() {
#ifdef BOOTLOCK
    return vFlagIsSet(ECVF_BOOT_LOCK) ? 1 : 0;
#else
    return 0;
#endif
  }

  void HardFault(int8_t recoverable);

  void SetLimitSleep(int8_t tf) {
    if (tf) setVFlags(ECVF_LIMIT_SLEEP);
    else clrVFlags(ECVF_LIMIT_SLEEP);
  }
  uint8_t LimitSleepIsSet() { return vFlagIsSet(ECVF_LIMIT_SLEEP); }

#ifdef GFI
  void SetGfiTripped();
  uint8_t GfiTripped() { return vFlagIsSet(ECVF_GFI_TRIPPED); }
  uint8_t GetGfiTripCnt() { return m_GfiTripCnt+1; }
#ifdef GFI_SELFTEST
  uint8_t GfiSelfTestEnabled() {
    return (m_wFlags & ECF_GFI_TEST_DISABLED) ? 0 : 1;
  }
  void EnableGfiSelfTest(uint8_t tf);
#else // !GFI_SELFTEST
  uint8_t GfiSelfTestEnabled() { return 0; }
  void EnableGfiSelfTest(uint8_t tf) {}
#endif // GFI_SELFTEST
#endif // GFI

#ifdef TEMPERATURE_MONITORING  
    uint8_t TempChkEnabled() {
    return (m_wFlags & ECF_TEMP_CHK_DISABLED) ? 0 : 1;
  }
  void EnableTempChk(uint8_t tf);
#endif //TEMPERATURE_MONITORING

#ifdef HEARTBEAT_SUPERVISION
int HeartbeatSupervision(uint16_t interval, uint8_t amps);
int HsPulse();
int HsRestoreAmpacity();
int HsExpirationCheck();
int HsAckMissedPulse(uint8_t ack);
int GetHearbeatInterval();
int GetHearbeatCurrent();
int GetHearbeatTrigger();
#endif //HEARTBEAT_SUPERVISION


  uint8_t SerDbgEnabled() { 
    return (m_wFlags & ECF_SERIAL_DBG) ? 1 : 0;
  }
  void EnableSerDbg(uint8_t tf);
#ifdef RGBLCD
  int SetBacklightType(uint8_t t,uint8_t update=1); // BKL_TYPE_XXX
#endif // RGBLCD

#ifdef VOLTMETER
  uint16_t GetVoltScaleFactor() { return m_VoltScaleFactor; }
  uint32_t GetVoltOffset() { return m_VoltOffset; }
  void SetVoltmeter(uint16_t scale,uint32_t offset);
  uint32_t ReadVoltmeter();
#endif // VOLTMETER
#ifdef AMMETER
  int32_t GetChargingCurrent() {
#ifdef OCPPDBG
    return 78*1000;
#else
    return m_ChargingCurrent;
#endif // OCPPDBG
  }
#ifdef FAKE_CHARGING_CURRENT
  void SetChargingCurrent(int32_t current) {
    m_ChargingCurrent = current;
    m_AmmeterReading = current;
  }
#endif

  int16_t GetAmmeterCurrentOffset() { return m_AmmeterCurrentOffset; }
  int16_t GetCurrentScaleFactor() { return m_CurrentScaleFactor; }
  void SetAmmeterCurrentOffset(int16_t offset) {
    m_AmmeterCurrentOffset = offset;
    eeprom_write_word((uint16_t*)EOFS_AMMETER_CURR_OFFSET,offset);
  }
  void SetCurrentScaleFactor(int16_t scale) {
    m_CurrentScaleFactor = scale;
    eeprom_write_word((uint16_t*)EOFS_CURRENT_SCALE_FACTOR,scale);
  }
#ifdef ECVF_AMMETER_CAL
  uint8_t AmmeterCalEnabled() { 
    return vFlagIsSet(ECVF_AMMETER_CAL);
  }
  void EnableAmmeterCal(uint8_t tf) {
    if (tf) setVFlags(ECVF_AMMETER_CAL);
    else clrVFlags(ECVF_AMMETER_CAL);
  }
#endif // ECVF_AMMETER_CAL
  void ZeroChargingCurrent() { m_ChargingCurrent = 0; }
  uint8_t GetInstantaneousChargingAmps() {
    readAmmeter();
    return m_AmmeterReading / 1000;
  }
#ifdef CHARGE_LIMIT
  void ClrChargeLimit() {
    m_chargeLimitTotWs = 0;
    m_chargeLimitkWh = 0;
    clrVFlags(ECVF_CHARGE_LIMIT);
  }
  void SetChargeLimitkWh(uint8_t kwh);
  uint32_t GetChargeLimitTotWs() { return m_chargeLimitTotWs; }
  uint8_t GetChargeLimitkWh() { return m_chargeLimitkWh; }
#endif // CHARGE_LIMIT
#else // !AMMETER
  int32_t GetChargingCurrent() { return -1; }
#endif // AMMETER
  uint8_t LimitsAllowed() {
    return ((GetState() == EVSE_STATE_B) || (GetState() == EVSE_STATE_C)) ? 1 : 0;
  }
#ifdef TIME_LIMIT
  void ClrTimeLimit() {
    m_timeLimitEnd = 0;
    m_timeLimit15 = 0;
    clrVFlags(ECVF_TIME_LIMIT);
  }
  void SetTimeLimitEnd(time_t limit) { m_timeLimitEnd = limit; }
  void SetTimeLimit15(uint8_t mind15);
  uint8_t GetTimeLimit15() { return m_timeLimit15; }
  time_t GetTimeLimitEnd() { return m_timeLimitEnd; }
#endif // TIME_LIMIT
  void ReadPilot(uint16_t *plow=NULL,uint16_t *phigh=NULL);
  void Reboot();
#ifdef SHOW_DISABLED_TESTS
  void DisabledTest_P(PGM_P message);
  void ShowDisabledTests();
#endif
#ifdef ADVPWR
  uint8_t GetNoGndTripCnt() { return m_NoGndTripCnt+1; }
  uint8_t GetStuckRelayTripCnt() { return m_StuckRelayTripCnt+1; }
#endif // ADVPWR

#ifdef STATE_TRANSITION_REQ_FUNC
  void SetStateTransitionReqFunc(EvseStateTransitionReqFunc statetransitionreqfunc) {
    m_StateTransitionReqFunc = statetransitionreqfunc;
  }
#endif
  J1772Pilot *GetPilot() { return &m_Pilot; }
  uint8_t GetPilotState() { return m_PilotState; }
  void CloseRelay() {
    chargingOn();
  }
  void OpenRelay() {
    chargingOff();
  }
  uint8_t RelayIsClosed() { return vFlagIsSet(ECVF_CHARGING_ON); }
#ifdef AUTH_LOCK
  void AuthLock(uint8_t tf,uint8_t update);
  uint8_t AuthLockIsOn() { return vFlagIsSet(ECVF_AUTH_LOCKED); }
#endif // AUTH_LOCK

  void SetEvConnected() { setVFlags(ECVF_EV_CONNECTED); }
  void ClrEvConnected() {
    clrVFlags(ECVF_EV_CONNECTED);
    setVFlags(ECVF_SESSION_ENDED);
 }
  void SetEvConnectedPrev() { setVFlags(ECVF_EV_CONNECTED_PREV); }
  void ClrEvConnectedPrev() { clrVFlags(ECVF_EV_CONNECTED_PREV); }
  // EvConnected value valid when pilot state not N12
  uint8_t EvConnected() { return vFlagIsSet(ECVF_EV_CONNECTED); }
  uint8_t EvConnectedTransition() {
    if (((m_wVFlags & (ECVF_EV_CONNECTED|ECVF_EV_CONNECTED_PREV)) == 0) ||
	((m_wVFlags & (ECVF_EV_CONNECTED|ECVF_EV_CONNECTED_PREV)) == (ECVF_EV_CONNECTED|ECVF_EV_CONNECTED_PREV))) return 0;
    else return 1;
  }
  void SetInMenu() { setVFlags(ECVF_UI_IN_MENU); }
  void ClrInMenu() { clrVFlags(ECVF_UI_IN_MENU); }
  void SetDelayTimerOnFlag() {setVFlags(ECVF_TIMER_ON); }
  void ClrDelayTimerOnFlag() {clrVFlags(ECVF_TIMER_ON); }
#ifdef BTN_MENU
  void ButtonEnable(uint8_t tf) {
    if (!tf) setFlags(ECF_BUTTON_DISABLED); 
    else clrFlags(ECF_BUTTON_DISABLED);
    SaveEvseFlags();
  }
  uint8_t ButtonIsEnabled() { return flagIsSet(ECF_BUTTON_DISABLED) ? 0 : 1; }
#endif // BTN_MENU

#if defined(KWH_RECORDING) && !defined(VOLTMETER)
  void SetMV(uint32_t mv) { m_Voltage = mv; }
#endif
  int32_t GetVoltage() { return m_Voltage; }
#ifdef MENNEKES_LOCK
  void SetMennekesManual() { m_wVFlags |= ECVF_MENNEKES_MANUAL; }
  void ClrMennekesManual() { m_wVFlags &= ~ECVF_MENNEKES_MANUAL; }
  int8_t MennekesIsManual() { return (m_wVFlags & ECVF_MENNEKES_MANUAL) ? 1 : 0; }
  int8_t MennekesIsLocked() { return m_MennekesLock.IsLocked(); }
  void LockMennekes() { m_MennekesLock.Lock(1); }
  void UnlockMennekes() { m_MennekesLock.Unlock(1); }
#endif // MENNEKES_LOCK

};

#ifdef FT_ENDURANCE
extern int g_CycleCnt;
extern long g_CycleHalfStart;
extern uint8_t g_CycleState;
#endif 

extern J1772EVSEController g_EvseController;
