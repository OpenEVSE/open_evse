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


 **** RAPI protocol ****

Fx - function
Sx - set parameter
Gx - get parameter

command formats
1. with XOR checksum (recommended)
$cc pp pp ...^xk\r
2. with additive checksum (legacy)
$cc pp pp ...*ck\r
3. no checksum (FOR TESTING ONLY! DON'T USE FOR APPS)
$cc pp pp ...\r
4. checksum + sequence id (v3.0.0+)
$cc pp pp .. :ss^xk\r

\r = carriage return = 13d = 0x0D
cc = 2-letter command
pp = parameters
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'
ck = 2-hex-digit checksum - 8-bit sum of all characters before '*'
ss = optional 2-hex-digit sequence id - response will echo the sequence id
     so that receiver can verify that the response matches the command
     ss CANNOT be 00, which is reserved as an invalid value


response format (v1.0.3-)
$OK [optional parameters]\r - success
$NK [optional parameters]\r - failure

response format (v2.0.0+)
$OK [optional parameters]^xk\r - success
$NK [optional parameters]^xk\r - failure
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'

response format (v3.0.0+)
$OK [optional parameters] [:ss]^xk\r - success
$NK [optional parameters] [:ss]^xk\r - failure
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'
ss = optional 2-hex-digit sequence ID which was sent with the command
     only present if a sequence ID was send with the command

A-prefix: asynchronous notification messages

Boot Notification
$AB postcode fwrev
 postcode(hex):
   if boot OK then postcode = 00
   if error then postcode
       07 = bad ground
       08 = stuck relay
       09 = GFI test failed
 fwrev(string): firmware revision

EVSE state transition: sent whenever EVSE state changes
$AT evsestate pilotstate currentcapacity vflags
 evsestate(hex): EVSE_STATE_xxx
 pilotstate(hex): EVSE_STATE_xxx
 currentcapacity(decimal): amps
 vflags(hex): m_wVFlags bits

External button press notification - only if RAPI_BTN defined
When the button is disabled ($FF B 0) send the event via RAPI
$AN type
 type: 0 - short press, 1 - long press

Request client WiFi mode - only if RAPI_WF defined
$WF mode\r
 mode: WIFI_MODE_XXX
 (currently very long press (10 sec) of menu btn on OpenEVSE will send WIFI_MODE_AP_DEFAULT
v2.0.1+: 2-hex-digit XOR checksum appended to asynchronous messages

commands


F0 {1|0}- enable/disable display updates
     enables/disables g_OBD.Update()
 $F0 1^43 - enable display updates and call g_OBD.Update()
 $F0 0^42 - disable display updates
F1 - simulate front panel button short press
 N.B.: it is possible that an asynchronous state change will be sent by the
  EVSE prior to sending the response to $F1
FB color - set LCD backlight color
colors:
 OFF 0
 RED 1
 YELLOW 3
 GREEN 2
 TEAL 6
 BLUE 4
 VIOLET 5
 WHITE 7

 $FB 7*03 - set backlight to white
FC - reset fault counters and total energy
FD - disable EVSE
 $FD*AE
FE - enable EVSE
 $FE*AF
FH - reset relay Health/life estimate (requires RELAY_HEALTH)
 clears the relay-life cumulative-damage accumulator, the transit-time/
 thermal-index self-learned baselines, and the stuck-relay recovery
 counter - use after replacing the contactor, so the estimate doesn't
 carry over wear from the old relay.
 $FH*xx
FK - run stucK relay recovery cycle manually (requires ADVPWR)
 Cycles the relay (STUCK_RELAY_RECOVERY_ROUNDS rounds of
 STUCK_RELAY_RECOVERY_CYCLES on/off toggles each, shortening each toggle
 within a round) to try to free a welded/stuck contact, same routine the
 controller runs automatically on EVSE_STATE_STUCK_RELAY entry. NAK'd if
 an EV is connected (unsafe to cycle the relay under load) - check $G0
 first. Does not itself report whether the relay came free - check $GS
 (state) or $GL (recovery counter) afterward. Blocking: up to ~30s worst
 case (3 rounds x 5 toggles x up to 2s on+off, plus zero-cross/transit
 overhead) before the response comes back.
 $FK*xx
FO set Overtemperature threshold
 $FO panicthresh
 panicthresh in 10ths of a degree Celsius
 $FO 705 - set panic threshold to 70.5C
 if temperature exceeds panicthresh, EVSE goes into OVER_TEMPERATURE fault state

FP x y text - print text on lcd display
  OPTIONAL: can substitute character 0x11 for spaces within a string, because they print as <SPC> on HD44780. More reliable.
FR - restart EVSE
 $FR*BC
FS - sleep EVSE
 $FS*BD
FF - enable/disable feature
 $FF feature_id 0|1
 0|1 0=disable 1=enable
 feature_id:
  B = disable/enable front panel button
  D = Diode check
  E = command Echo
   use this for interactive terminal sessions with RAPI.
   RAPI will echo back characters as they are typed, and add a <LF> character
   after its replies. Valid only over a serial connection, DO NOT USE on I2C
  F = GFI self test
  G = Ground check
  L = boot Lock
  O = overcurrent check
  P = PP auto ampacity
  R = stuck Relay check
  T = temperature monitoring
  V = Vent required check
  Z = zero-cross detection for relay switching
 $FF D 0 - disable diode check
 $FF G 1 - enable ground check

S0 0|1 - set LCD type
 $S0 0*F7 = monochrome backlight
 $S0 1*F8 = RGB backlight
S1 yr mo day hr min sec - set clock (RTC) yr=2-digit year
S2 0|1 - disable/enable ammeter calibration mode - ammeter is read even when not charging
 $S2 0*F9
 $S2 1*FA
S3 cnt - set charge time limit to cnt*15 minutes (0=disable, max=255)
 NOTES:
  - allowed only when EV connected in State B or C
  - current session will stop when time reached. the limit automatically
    gets cancelled when EV disconnected
  - temporarily disables delay timer until EV disconnected or limit reached
 response:
  $OK - accepted
  $NK - invalid EVSE state
S4 0|1 - set auth lock (needs AUTH_LOCK defined and AUTH_LOCK_REG undefined)
   0 = unlocked
   1 = locked - EVSE won't charge until unlocked
   when auth lock is on, will not transition to State C and a lock icon is
   displayed in States A & B.
S5 A|M|0|1 - Mennekes lock setting
   A = enable automatic mode - locked when connected, unlocked otherwise
   M = enable manual control mode
   0 = unlock (valid only in manual mode)
   1 = lock (valid only in manual mode)
   n.b. requires MENNEKES_LOCK. manual mode is volatile - always boots in automatic mode
SA currentscalefactor currentoffset - set ammeter settings
SB - clear boot lock
  when BOOTLOCK is defined, EVSE won't allow charging after boot up until SB is received
 response: $OK 0 = unlock success
           $OK 1 = unlock fail - EVSE currently in fault state
SC amps [V|M]- set current capacity
 response:
   if amps < minimum current capacity, will set to minimum and return $NK ampsset
   if amps > maximum current capacity, will set to maximum and return $NK ampsset
   if in over temperature status, raising current capacity will fail and return $NK ampsset
   otherwise return $OK ampsset
   ampsset: the resultant current capacity
   default action is to save new current capacity to EEPROM for the currently active service level.
   if V is specified, then new current capacity is volatile, and will be
     reset to previous value at next reboot
   if M is specified, sets maximum L2 current capacity for the unit and writes
     to EEPROM. subsequent calls the $SC cannot exceed value set bye $SC M
     the value cannot be changed/erased via RAPI commands. Subsequent calls
     to $SC M will return $NK
SH kWh - set cHarge limit to kWh
 NOTES:
  - allowed only when EV connected in State B or C
  - current session will stop when total kWh reached. the limit automatically
    gets cancelled when EV disconnected
  - temporarily disables delay timer until EV disconnected or limit reached
 response:
  $OK - accepted
  $NK - invalid EVSE state
SK - set accumulated Wh (v1.0.3+)
 $SK 0^2C - set accumulated Wh to 0
SL 1|2|A  - set service level L1/L2/Auto
 $SL 1*14
 $SL 2*15
 $SL A*24
SM voltscalefactor voltoffset - set voltMeter settings
ST starthr startmin endhr endmin - set timer
 $ST 0 0 0 0^23 - cancel timer
SV mv - Set Voltage for power calculations to mv millivolts
 $SV 223576 - set voltage to 223.576
 NOTES:
  - only available if VOLTMETER not defined and KWH_RECORDING defined
  - volatile - value is lost, and replaced with VOLTS_FOR_Lx at boot
SY heartbeatinterval heartbeatcurrentlimit
 heartbeatinterval - seconds. 0 to disable
 heartbeatcurrentlimit - max current when no heartbeat within heartbeatinterval
 Response includes heartbeatinterval hearbeatcurrentlimit hearbeattrigger
 hearbeattrigger: 0 - There has never been a missed pulse,
 2 - there is a missed pulse, and HS is still in current limit
 1 - There was a missed pulse once, but it has since been acknowledged. Ampacity has been successfully restored to max permitted
 $SY 100 6  //If no pulse for 100 seconds, set EVE ampacity limit to 6A until missed pulse is acknowledged
 $SY        //This is a heartbeat supervision pulse.  Need one every heartbeatinterval seconds.
 $SY 165    //This is an acknowledgement of a missed pulse.  Magic Cookie = 165 (=0XA5)
 When you send a pulse, an NK response indicates that a previous pulse was missed and has not yet been acked

SR n 0|1 - enable/disable relay output (saved to EEPROM, applied at boot)
 n: 1=DC relay 1, 2=DC relay 2, 3=AC relay
 0=disable 1=enable (all relays enabled by default)
 $SR 1 0 - disable DC relay 1
 $SR 2 0 - disable DC relay 2
 $SR 3 0 - disable AC relay
 $SR 1 1 - re-enable DC relay 1

SZ ma - set relay-open current-zero threshold (requires RELAY_ZC_SWITCH)
 ma(decimal): current, in mA, below which the relay is considered safe to
   open at a current zero. Some units never read below the compiled-in
   default (200 mA) due to ammeter offset/noise floor, which stalls every
   non-emergency relay open for the full internal timeout (1s) - use this to
   retune without a firmware rebuild.
 range: 0-5000 (clamped). saved to EEPROM, applied immediately and at boot.
 $SZ 300 - relay is considered safe to open below 300 mA

G0 - get EV connect state
 response: $OK connectstate
 connectstate: 0=not connected, 1=connected, 2=unknown
 -> connectstate is unknown when EVSE pilot is -12VDC
 $G0^53

G3 - get charging time limit
 response: $OK cnt
 cnt*15 = minutes
        = 0 = no time limit
 $G3^50

G4 - get auth lock (needs AUTH_LOCK defined and AUTH_LOCK_REG undefined)
 response: $OK lockstate
  lockstate = 0=unlocked, =1=locked
 $G4^57

G5 - get Mennekes settings
 response: $OK state mode
   state: 0 = unlocked
          1 = locked
   mode: A = automatic mode - locked when connected, unlocked otherwise
         M = manual control mode
   Note: lock mode is also indicated by ECVF_MENNEKES_MANUAL
   n.b. requires MENNEKES_LOCK

GA - get ammeter settings
 response: $OK currentscalefactor currentoffset
 $GA^22

GC - get current capacity info
 response: $OK minamps hmaxamps pilotamps cmaxamps
 all values decimal
 minamps - min allowed current capacity
 hmaxamps - max hardware allowed current capacity MAX_CURRENT_CAPACITY_Ln
 pilotamps - current capacity advertised by pilot
 cmaxamps - max configured allowed current capacity (saved to EEPROM)
     if PP_AUTO_AMPACITY is enabled, then  in STATE B/C returns max capacity
     read from PP if lower than max configured capacity
 n.b. maxamps,emaxamps values are dependent on the active service level (L1/L2)
 $GC^20

GD - get Delay timer
 response: $OK starthr startmin endhr endmin
   all values decimal
   if timer disabled, starthr=startmin=endhr=endmin=0
 $GD^27

GE - get settings
 response: $OK amps(decimal) flags(hex)
 $GE^26

GF - get fault counters
 response: $OK gfitripcnt nogndtripcnt stuckrelaytripcnt (all values hex)
 maximum trip count = 0xFF for any counter
 $GF^25

GG - get charging current and voltage
 response: $OK milliamps millivolts
 AMMETER must be defined in order to get amps, otherwise returns -1 amps
 $GG^24

GH - get cHarge limit
 response: $OK kWh
 kWh = 0 = no charge limit
 $GH^2B

GR - get relay enable status
 response: $OK dc1 dc2 ac
 dc1/dc2/ac: 1=enabled 0=disabled
 $GR

GI - get MCU ID - requires MCU_ID_LEN to be defined
 response: $OK mcuid
 mcuid: MCU serial number
  AVR:
    mcuid is 6 ASCII characters followed by 8 hex digits
    first hex digit = FF for 328P
    WARNING: mcuid is guaranteed to be unique only for the 328PB. Uniqueness is
    unknown in 328P. The first 6 characters are ASCII, and the rest are
    hexadecimal.
  SAMD:
   mcuid is 128-bit number
   returned as a 32-character hex string

GM - get voltMeter settings
 response: $OK voltcalefactor voltoffset
 $GM^2E

GO get Overtemperature threshold
 response: $OK panicthresh
 panicthresh in 10ths of a degree Celsius
 if temperature exceeds panicthresh, EVSE goes into OVER_TEMPERATURE fault state
 $GO^2C
GP - get temPerature (v1.0.3+)
 response: $OK ds3231temp mcp9808temp tmp007temp
 ds3231temp - temperature from DS3231 RTC
 mcp9808temp - temperature from MCP9808
 tmp007temp - temperature from TMP007
 all temperatures are in 10th's of a degree Celcius
 if any temperature sensor is not installed, its return value is -2560
 $GP^33

GS - get state
 response: $OK evsestate elapsed pilotstate vflags
 evsestate(hex): EVSE_STATE_xxx
 elapsed(dec): elapsed charge time in seconds of current or last charging session
 pilotstate(hex): EVSE_STATE_xxx
 vflags(hex): EVCF_xxx
 $GS^30

GT - get time (RTC)
 response: $OK yr mo day hr min sec       yr=2-digit year
 $GT^37

GU - get energy usage (v1.0.3+)
 response: $OK Wattseconds Whacc
 Wattseconds - Watt-seconds used this charging session, note you'll divide Wattseconds by 3600 to get Wh
 Whacc - total Wh accumulated over all charging sessions, note you'll divide Wh by 1000 to get kWh
 $GU^36

GV - get version
 response: $OK firmware_version protocol_version
 NOTE: protocol_version is deprecated. too hard to maintain variants.
 ignore it, and test commands for compatibility, instead.
 $GV^35

T commands for debugging only #define RAPI_T_COMMMANDS
T0 amps - set fake charging current
 response: $OK
 $T0 75
 
GY - Get Hearbeat Supervision Status
 Response includes heartbeatinterval hearbeatcurrentlimit hearbeattrigger
 hearbeattrigger: 0 - There has never been a missed pulse,
 2 - there is a missed pulse, and HS is still in current limit
 1 - There was a missed pulse once, but it has since been acknkoledged. Ampacity has been successfully restored to max permitted
 See SY above for worked expamples.

GZ - get AC line frequency + relay-open current-zero threshold (requires RELAY_ZC_SWITCH)
 response: $OK freqx100 zerothreshma
 freqx100(decimal): measured AC frequency * 100, e.g. 6012 = 60.12 Hz
 0 = frequency not yet measured (no relay operation has occurred since boot)
 CGMI hardware: updated on relay close (zcWaitRelayClose) and open (zcWaitRelayOpen)
 non-CGMI hardware: updated on relay open only (zcWaitRelayOpen); AC pins are
   load-side on older non-V6 boards so no valid signal is available at relay close
 zerothreshma(decimal): current RAPI $SZ-configured current-zero threshold, in mA
 $GZ^38

GW - get relay Wear/life diagnostics (requires RELAY_ZC_SWITCH)
 response: $OK hotswitchcnt lastopenma closetransitms opentransitms
 hotswitchcnt(decimal): cumulative count of relay opens where current never
   reached the zero threshold before opening (i.e. an arced/"hot" break -
   includes emergency opens and opens while zero-cross switching is
   disabled via $FF Z 0). Saturates at 65534; persisted across reboots.
 lastopenma(decimal): ammeter reading, in mA, immediately before the most
   recent relay open. For relay-life estimation/trending.
 closetransitms(decimal): elapsed ms from the close command to the load-side
   AC-sense pin confirming the relay physically closed, from the most recent
   close. opentransitms: same, for the most recent open.
   300 (RELAY_TRANSIT_TIMEOUT_MS) = timed out/not confirmed, or not
   measurable on this hardware (requires CGMI)
 $GW^39

GL - get relay Life/health estimate (requires RELAY_HEALTH; needs RELAY_ZC_SWITCH + AMMETER)
 response: $OK pctremain coldopencnt elecdamagex1e6 transitbaselinems transitdrift thermalx100 thermalbaselinex100 thermalwarn stuckrelayrecoverycnt
 Cumulative-damage (Miner's rule) contact-life estimate built on the $GW
 diagnostics, plus (if TEMPERATURE_MONITORING) a self-baselined thermal
 index. Diagnostic only - never gates charging logic. See RelayHealth.h for
 the full derivation. Reset via $FH (e.g. after relay replacement).
 pctremain(decimal, 0-100): estimated relay life remaining. Combines an
   electrical-damage accumulator (hot opens, weighted by
   (I/I_rated)^2 * load-character * temperature multipliers) with a
   mechanical-damage term (cold opens / rated mechanical life) - in a
   well-behaved installation the electrical term dominates and is driven
   almost entirely by fault-interrupt/e-stop events, not normal sessions.
 coldopencnt(decimal): cumulative cold (non-arced) relay-open count.
 elecdamagex1e6(decimal): raw electrical-damage accumulator, in millionths
   of rated electrical life consumed (1000000 = 100%). Debug/trend field.
 transitbaselinems(decimal): self-learned baseline open (drop-out) transit
   time, in ms. 65535 = baseline not yet established (needs 8 post-reset
   measurements; requires CGMI hardware, see $GW).
 transitdrift(0|1): 1 if the most recent open-transit time is >=1.5x the
   baseline - an early symptom of contact welding, ahead of the hard
   EVSE_STATE_STUCK_RELAY check.
 thermalx100(decimal): most recent deltaT/I^2 sample (x100), proportional to
   contact resistance. 65535 = not available (no TEMPERATURE_MONITORING, no
   session yet, or current below the 6A sampling floor).
 thermalbaselinex100(decimal): self-learned baseline H0 (x100). 65535 = not
   yet established (needs 4 in-session samples) or not available.
 thermalwarn(decimal): 0=ok/not available 1=watch (>=1.5x baseline)
   2=warn (>=2x baseline).
 stuckrelayrecoverycnt(decimal): cumulative count of stuck-relay recovery
   attempts run (automatic, on EVSE_STATE_STUCK_RELAY entry with no EV
   connected, or manual via $FK). 0 if ADVPWR isn't built in. Saturates at
   65534; persisted across reboots; reset via $FH.
 $GL^3a

Z0 FOR TESTING RELAY_AUTO_PWM_PIN ONLY
Z0 closems holdpwm
   closems(dec) = # ms to apply DC to relay pin
   holdpwm(dec) = pwm duty cycle for relay hold 0-255


 *
 */

#ifdef RAPI

#define RAPIVER "6.0.0"

#define WIFI_MODE_AP 0
#define WIFI_MODE_CLIENT 1
#define WIFI_MODE_AP_DEFAULT 2

// buffer[] holds an inbound RAPI command and is reused to build the
// outbound response text. The longest response text is $GI (get MCU id),
// whose #else branch writes 2*MCU_ID_LEN hex chars plus a NUL. On SAMD
// MCU_ID_LEN is 16, so that is 2*16+1 = 33 bytes and the historic 32-byte
// buffer overflowed by one, corrupting the adjacent bufCnt member. Size
// per target so AVR RAM cost stays zero.
#ifdef TARGET_SAMD
#define ESRAPI_BUFLEN 40
#else
#define ESRAPI_BUFLEN 32
#endif
#if defined(MCU_ID_LEN) && (ESRAPI_BUFLEN < (2*MCU_ID_LEN + 1))
#error "ESRAPI_BUFLEN too small for the $GI response on this target"
#endif
#define ESRAPI_SOC '$' // start of command
#define ESRAPI_EOC 0xd // CR end of command
#define ESRAPI_SOS ':' // start of sequence id
#define ESRAPI_MAX_ARGS 10
// for RAPI_SENDER
#define RAPIS_TIMEOUT_MS 500
#define RAPIS_BUFLEN 20

#define INVALID_SEQUENCE_ID 0

class EvseRapiProcessor {
#ifdef GPPBUGKLUDGE
  char *buffer;
public:
  void setBuffer(char *buf) { buffer = buf; }
private:
#else
  char buffer[ESRAPI_BUFLEN]; // input buffer
#endif // GPPBUGKLUDGE
  int8_t bufCnt; // # valid bytes in buffer
  char *tokens[ESRAPI_MAX_ARGS];
  int8_t tokenCnt;
  char echo;
  uint8_t curReceivedSeqId;
  void appendSequenceId(char *s,uint8_t seqId);
#ifdef RAPI_SENDER
  uint8_t curSentSeqId;
  uint8_t getSendSequenceId();
  int8_t isAsyncToken();
  int8_t isRespToken();
#endif // RAPI_SENDER

  virtual int available() = 0;
  virtual int read() = 0;
  virtual void writeStart() {}
  virtual void writeEnd() {}
  virtual int write(uint8_t u8) = 0;
  virtual int write(const char *str) = 0;

  void reset() {
    buffer[0] = 0;
    bufCnt = 0;
  }

  int tokenize(char *buf);
  int processCmd();

  void response(uint8_t ok);
  void appendChk(char *buf);
  
#ifdef RAPI_SENDER
  char sendbuf[RAPIS_BUFLEN]; // input buffer
  void _sendCmd(const char *cmdstr);
#endif // RAPI_SENDER
  
public:
  EvseRapiProcessor();

  int doCmd();
  void sendEvseState();
  void sendBootNotification();
  void setWifiMode(uint8_t mode); // WIFI_MODE_xxx
  void sendButtonPress(uint8_t long_press);
  void writeStr(const char *msg) { writeStart();write(msg);writeEnd(); }

  virtual void init();

#ifdef RAPI_SENDER
  int8_t sendCmd(const char *cmdstr);
  int8_t receiveResp(unsigned long msstart);
#endif // RAPI_SENDER
};

#ifdef RAPI_SERIAL
class EvseSerialRapiProcessor : public EvseRapiProcessor {
  int available() { return RAPI_SERIAL_PORT.available(); }
  int read() { return RAPI_SERIAL_PORT.read(); }
  int write(uint8_t u8) { return RAPI_SERIAL_PORT.write(u8); }
  int write(const char *str) { return RAPI_SERIAL_PORT.write(str); }

public:
  EvseSerialRapiProcessor();
  void init();
};

extern EvseSerialRapiProcessor g_ESRP;
#endif // RAPI_SERIAL


#ifdef RAPI_I2C
class EvseI2cRapiProcessor : public EvseRapiProcessor {
  int available() { return Wire.available(); }
  int read() { return Wire.read(); }
  void writeStart() { Wire.beginTransmission(RAPI_I2C_REMOTE_ADDR); }
  void writeEnd() { Wire.endTransmission(); }
  int write(uint8_t u8) { return Wire.write(u8); }
  int write(const char *str) { return Wire.write(str); }

public:
  EvseI2cRapiProcessor();
  void init();
};

extern EvseI2cRapiProcessor g_EIRP;
#endif // RAPI_I2C

void RapiInit();
void RapiDoCmd();
uint8_t RapiSendEvseState(uint8_t force=0);
void RapiSetWifiMode(uint8_t mode);
void RapiSendButtonPress(uint8_t long_press);
void RapiSendBootNotification();

#endif // RAPI
