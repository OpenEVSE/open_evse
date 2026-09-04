// -*- C++ -*-
//
// Hardware control channel for the native target.
//
// A line-oriented protocol on a Unix socket carrying pin levels, ADC counts
// and pilot state. It deliberately carries no semantics: raw values and edges
// only. What a pilot voltage *means* is the driver's business -- the emulator
// or a test harness -- because the moment that judgement moves in here, the
// firmware stops being the only thing under test.
//
// Enabled by $OPENEVSE_HW_SOCKET. Unset, the channel never opens and the
// firmware runs standalone exactly as it does without this file.
//
// Protocol, newline terminated, uppercase from the firmware:
//
//   firmware -> driver
//     HELLO board=<oev6|nxt> adc_bits=N adc_max=N mcu_id_len=N version=<v>
//     OUT <NAME> <0|1>          digital output changed
//     PILOT <P12|PWM|N12> <duty_tenths_pct> <amps>
//     PONG
//
//   driver -> firmware
//     IN <NAME> <0|1>           drive a digital input
//     ADC <NAME> <high> [low]   set an ADC channel; two values describe a
//                               signal swinging between them, one a level
//     SNAP                      re-send HELLO and every current output
//     PING
//
// Signal names are symbolic so a driver never needs to know this target's
// arbitrary pin numbering, and so a session is readable under socat.

#pragma once

#include <stdint.h>

// Open the socket if $OPENEVSE_HW_SOCKET is set. Safe to call more than once.
void channelBegin();

// Accept a waiting driver, apply anything it has sent, and publish any output
// that changed since the last call. Called from nativeServiceIo().
void channelService();

// Record pilot state for publication. Called by J1772Pilot.
void channelPublishPilot(uint8_t state, uint32_t dutyTenthsPct, int amps);
