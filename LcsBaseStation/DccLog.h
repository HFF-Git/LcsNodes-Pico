//------------------------------------------------------------------------------------------------------------
//
// DCC Packet / RailCom Log
//
//------------------------------------------------------------------------------------------------------------
// DCC_LOG is a set of helper functions to understand and debug the DCC packet flow and RailCom interaction.
// When programming a decoder, quite a few packets need to be sent, power consumption to be monitored. In
// addition when using RailCom it would be great to see the flow of DCC packets and the resulting RailCom
// datagrams sent by the decoder. Unfortunately, all this cannot be done with simple Debug Pringt Messages
// for timing reasons. The DCC_LOG routines provide a simple log data array and write to this buffer. The
// data flow to look into is bracketd by a begin and end routine. Lateron this buffer can be printed and
// analyzed.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station DCC Track implementation file
// Copyright (C) 2019 - 2023  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
// Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along with this program. If not, see
// http://www.gnu.org/licenses
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//------------------------------------------------------------------------------------------------------------
#ifndef DccLog_h
#define DccLog_h

#include "arduino.h"

//------------------------------------------------------------------------------------------------------------
// DCC_LOG namespace part one. We need to declare the log engry types first, so tha they can also be used
// by the file local routines.
//
//------------------------------------------------------------------------------------------------------------
namespace DCC_LOG {

  //----------------------------------------------------------------------------------------------------------
  // Type of log entries. A log entry consist of the header byte, which contains in the first byte the
  // 4-bit log id and the 4-bit length of the log data. A log entry can therefore record up to 16 bytes of
  // payload.
  //
  //----------------------------------------------------------------------------------------------------------
  enum LogId : uint8_t {

    LOG_NIL       = 0,
    LOG_BEGIN     = 1,
    LOG_END       = 2,
    LOG_TSTAMP    = 3,
    LOG_DCC_IDL   = 4,
    LOG_DCC_RST   = 5,
    LOG_DCC_PKT   = 6,
    LOG_DCC_RCM   = 7,
    LOG_VAL       = 8,
    LOG_INV       = 15
  };

  //----------------------------------------------------------------------------------------------------------
  // The external interface. Every recoding session must be bracketed by a "begin" and and an "end" call, the
  // logging interface must be enabled in general. Listing the log entres is only possible when the log is
  // not active, i.e. a call to "end" was issued before.
  //
  //----------------------------------------------------------------------------------------------------------
  void enableLog( bool arg );
  void beginLog( );
  void endLog( );
  void printLog( );

  void        writeLogData( uint8_t id, uint8_t *buf, uint8_t len );
  void        writeLogId( uint8_t id );
  void        writeLogTs( );
  void        writeLogVal( uint8_t valId, uint16_t val );
};

#endif
