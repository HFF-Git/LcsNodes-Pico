//------------------------------------------------------------------------------------------------------------
//
// DCC Packet / RailCom Log - implementation file
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

#include "DccLog.h"

//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
#define LOG_OUT Serial

//------------------------------------------------------------------------------------------------------------
// File local declaration, not visible outside this file.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  //----------------------------------------------------------------------------------------------------------
  // The log buffer and the log index. When writing to the log buffer, the index will always point to the
  // next available position. Once the buffer is full, no further data can be added.
  //
  //----------------------------------------------------------------------------------------------------------
  const uint16_t  LOG_BUF_SIZE    = 4096;

  bool            logEnabled      = false;
  bool            logActive       = false;
  uint16_t        logBufIndex     = 0;

  uint8_t         logBuf[ LOG_BUF_SIZE ] = { 0 };

  //----------------------------------------------------------------------------------------------------------
  // Print a log entry routines. The fist byte of each log entry has encoded the log entry type and the
  // entry length. Depending on the log entry type, data is displayed as just the header, a numeric 16-bit
  // value, a numeric 32-bit vale or as an array of data bytes.
  //
  //----------------------------------------------------------------------------------------------------------
  void printPaddedHex( uint8_t val ) {

    LOG_OUT.print(( val < 0x10 ) ? "0x0" : "0x" );
    LOG_OUT.print( val, HEX );
  }

  void printLogEntryHead( uint16_t index ) {

    switch ( logBuf[ index ] >> 4 ) {

      case DCC_LOG::LOG_NIL:      LOG_OUT.print( F( "NIL        " )); break;
      case DCC_LOG::LOG_BEGIN:    LOG_OUT.print( F( "BEGIN      " )); break;
      case DCC_LOG::LOG_END:      LOG_OUT.print( F( "END        " )); break;
      case DCC_LOG::LOG_TSTAMP:   LOG_OUT.print( F( "TSTAMP     " )); break;
      case DCC_LOG::LOG_DCC_IDL:  LOG_OUT.print( F( "DCC_IDLE   " )); break;
      case DCC_LOG::LOG_DCC_RST:  LOG_OUT.print( F( "DCC_RESET  " )); break;
      case DCC_LOG::LOG_DCC_PKT:  LOG_OUT.print( F( "DCC_PKT    " )); break;
      case DCC_LOG::LOG_DCC_RCM:  LOG_OUT.print( F( "DCC_RCOM   " )); break;
      case DCC_LOG::LOG_VAL:      LOG_OUT.print( F( "VAL        " )); break;
      default: {

          LOG_OUT.print( F( "INVALID (" ));
          printPaddedHex( logBuf[ index ] >> 4 );
          LOG_OUT.print( F( ")" ));
        }
    }
  }

  void printLogTimeStamp( uint16_t index ) {

    uint32_t ts = logBuf[ index ];
    ts = ( ts << 8 ) | logBuf[ index + 1 ];
    ts = ( ts << 8 ) | logBuf[ index + 2 ];
    ts = ( ts << 8 ) | logBuf[ index + 3 ];

    LOG_OUT.print( F( "0x" ));
    LOG_OUT.print( ts, HEX );
  }

  void printLogVal( uint16_t index ) {

    printPaddedHex( logBuf[ index ] );
    LOG_OUT.print( F( ":" ));

    uint16_t val = logBuf[ index ] << 8 | logBuf[ index + 1 ];

    LOG_OUT.print( F( "0x" ));
    LOG_OUT.print( val, HEX );
  }

  void printLogData( uint16_t index, uint8_t len ) {

    for ( int i = 0; i < len; i++ ) {

      printPaddedHex( logBuf[ index + i ] );
      LOG_OUT.print( F( " " ));
    }
  }

  uint8_t printLogEntry( uint16_t index ) {

    if ( index < LOG_BUF_SIZE ) {

      uint8_t logEntryId  = logBuf[ index ] >> 4;
      uint8_t logEntryLen = logBuf[ index ] & 0x0F;

      printLogEntryHead( index );

      if      ( logEntryId == DCC_LOG::LOG_TSTAMP  )  printLogTimeStamp( index + 1 );
      else if ( logEntryId == DCC_LOG::LOG_VAL     )  printLogVal( index + 1 );
      else                                            printLogData( index + 1, logEntryLen );

      return ( logEntryLen + 1 );
    }
    else return ( 0 );
  }

};

//------------------------------------------------------------------------------------------------------------
// There are a couple of routines to write the log data. For convenience, some of the log entry types are
// available as a direct call. The order of data entry for numeric types is big endian, i.e. most significant
// byte first.
//
//------------------------------------------------------------------------------------------------------------
void DCC_LOG::writeLogData( uint8_t id, uint8_t *buf, uint8_t len ) {

  if ( logActive ) {

    len = len % 16;
    if ( logBufIndex + len + 1 < LOG_BUF_SIZE ) {

      logBuf[ logBufIndex ++ ] = ( id << 4 ) | len;
      for ( uint8_t i = 0; i < len; i++ ) logBuf[ logBufIndex ++ ] = buf[ i ];
    }
  }
}

void DCC_LOG::writeLogId( uint8_t id ) {

  if ( logActive ) logBuf[ logBufIndex ++ ] = ( id << 4 );
}

void DCC_LOG::writeLogTs( ) {

  if ( logActive ) {

    uint32_t ts = micros( );
    logBuf[ logBufIndex ++ ] = ( LOG_TSTAMP << 4 ) | 4;
    logBuf[ logBufIndex ++ ] = ( ts >> 24 ) & 0xFF;
    logBuf[ logBufIndex ++ ] = ( ts >> 16 ) & 0xFF;
    logBuf[ logBufIndex ++ ] = ( ts >> 8  ) & 0xFF;
    logBuf[ logBufIndex ++ ] = ( ts >> 0  ) & 0xFF;
  }
}

void DCC_LOG::writeLogVal( uint8_t valId, uint16_t val ) {

  if ( logActive ) {

    logBuf[ logBufIndex ++ ] = ( LOG_VAL << 4 ) | 3;
    logBuf[ logBufIndex ++ ] = valId;
    logBuf[ logBufIndex ++ ] = highByte( val );
    logBuf[ logBufIndex ++ ] = lowByte( val );
  }
}

//------------------------------------------------------------------------------------------------------------
// The log management routines. A typical transaction to log would start the logging process and then end
// it after the operation to analyze/debug. The "enableLog" call should be used to enable the logging
// process alltogether, the other calls will only do work when the log is enabled. With this call the
// recording process could be controlled from a command line setting or so.
//
//------------------------------------------------------------------------------------------------------------
void DCC_LOG::enableLog( bool arg ) {

  logEnabled = arg;
  logActive  = false;
}

void DCC_LOG::beginLog( ) {

  if ( logEnabled ) {

    logActive   = true;
    logBufIndex = 0;
    writeLogId( LOG_BEGIN );
    writeLogTs( );
  }
}

void DCC_LOG::endLog( ) {

  if ( logActive ) {

    writeLogTs( );
    writeLogId( LOG_END );
    logActive = false;
  }
}

//------------------------------------------------------------------------------------------------------------
// A simple routine to print out the log data. We print an entry on one line.
//
//------------------------------------------------------------------------------------------------------------
void DCC_LOG::printLog( ) {

  if ( logEnabled ) {

    if ( ! logActive ) {

      if ( logBufIndex > 0 ) {

        uint16_t entryIndex  = 0;
        uint8_t  entryLen    = 0;

        while ( entryIndex < logBufIndex ) {

          entryLen = printLogEntry( entryIndex );
          LOG_OUT.println( );

          if ( entryLen > 0 ) entryIndex += entryLen;
          else                break;
        }
      }
      else LOG_OUT.println( F( "DCC Log Buf: Nothing recorded" ));
    }
    else LOG_OUT.println( F( "DCC Log Active" ));
  }
  else LOG_OUT.println( F( "DCC Log disabled" ));
}
