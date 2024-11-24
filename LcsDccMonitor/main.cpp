//------------------------------------------------------------------------------------------------------------
//
// LCS - DCC Monitor, Raspberry PI Pico Implementation
//
//------------------------------------------------------------------------------------------------------------
// The DCC monitor is a utility that analyzes a DCC signal received and displays information about signal
// polarity, cutout detection, and so on. Most importantly, it displays the DCC packet in human readable
// format. The design is build upon the former DCC Sniffer program, however this monitor is a complete re-
// design and implementation.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - DCC Monitor, Raspberry PI Pico Implementation
// Copyright (C) 2019 - 2024  Helmut Fieres
//
//  Original versions: COPYRIGHT (c) 2013-2020
//
//  - DCC packet capture: Robin McKay, March 2014
//  - DCC packet analyze: Ruud Boer, October 2015
//  - Refactored to Version 2.0: Jürgen Winkler, March 2016
//  - Entirely rewritten to Version 3.0: Helmut Fieres, Oct 2020
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation, either version 3 of the License,
// or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details. You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - DCC Monitor, Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2024 Helmut Fieres
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
#include "LcsCdcLib.h"
#include "LcsDccPktFmtLib.h"
#include <cstring>

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
#define           DEBUG_PACKET_FORMATTER          1

//------------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//------------------------------------------------------------------------------------------------------------
CDC::CdcConfigDesc cfg;

//------------------------------------------------------------------------------------------------------------
// Configuration settings.
//
//------------------------------------------------------------------------------------------------------------
const uint16_t    ELAPSED_TIME_40               = 40;
const uint16_t    ELAPSED_TIME_80               = 80;
const uint16_t    ELAPSED_TIME_200              = 200;
const uint16_t    ELAPSED_IIME_400              = 400;

const uint16_t    DCC_MIN_PRAMBLE_BITS          = 12;
const uint16_t    DCC_MAX_PACKET_SIZE           = 12;
const uint16_t    BIT_BUFFER_SIZE               = 128;
const uint16_t    MAX_DCC_PACKETS               = 128;
const uint16_t    LINE_BUFFER_SIZE              = 256;
const uint16_t    DEFAULT_DCC_PACKETS           = 64;
const uint16_t    DEFAULT_REFRESH_TIME_MILLIS   = 4000;
const uint16_t    MAX_PREAMBLE_READ_ATTEMPTS    = 1024;

//------------------------------------------------------------------------------------------------------------
// We want to record the ranges of signal wave lengths we see. A simple TsRange object is used to remember
// the highest and lowest value seen in between resets.
//
//------------------------------------------------------------------------------------------------------------
struct TsRange {

  uint32_t low   = UINT32_MAX;
  uint32_t high  = 0;

  void reset( ) {

    low   = UINT32_MAX;
    high  = 0;
  }

  void add( uint32_t arg ) {

    if ( arg < low )  low   = arg;
    if ( arg > high ) high  = arg;
  }

  uint32_t getHigh( ) { return ( high );  }
  uint32_t getLow( )  { return ( low );   }
};

//------------------------------------------------------------------------------------------------------------
// DCC bit stream and packet declarations. The bit stream buffer is a circular buffer. The bit detecting
// routine add at the head, the packet assembler reads from the tail.
//
//------------------------------------------------------------------------------------------------------------
volatile uint8_t  bitBufHead = 0;
volatile uint8_t  bitBufTail = 0;
volatile uint8_t  bitBuffer[ BIT_BUFFER_SIZE ];
volatile bool     cutoutDetected = false;

//------------------------------------------------------------------------------------------------------------
// DCC Packet buffer declarations. During a time period, identical packets are remembered. This way, we only
// see new packets added. This has the not so nice effect that identical packets are not shown again. For
// example, turning F1 ON, then OFF, and then ON again in a period will not show the second turning ON. The
// same is true for for the loco speed up or down. Although correct, it looks a bit erratic on the screen,
// giving you the impression that the command was not sent. Once the time period expired, the buffer is reset.
//
//------------------------------------------------------------------------------------------------------------
uint8_t           dccPacketBufferSize       = DEFAULT_DCC_PACKETS;
uint32_t          refreshTimeInMillis       = DEFAULT_REFRESH_TIME_MILLIS;
uint32_t          timeToRefreshInMillis     = CDC::getMillis( ) + refreshTimeInMillis;
uint32_t          intervalCount             = 0;
uint32_t          packetsDetected           = 0;
uint32_t          errPacketsDetected        = 0;
uint32_t          resetPacketsDetected      = 0;
uint32_t          idlePacketsDetected       = 0;
uint32_t          preambleReadAttempts      = 0;

TsRange           belowSignal;
TsRange           oneBitSignal;
TsRange           zeroBitSignal;
TsRange           aboveSignal;

uint8_t           dccPacket[ DCC_MAX_PACKET_SIZE ];
uint32_t          packetBuffer[ MAX_DCC_PACKETS ];

//------------------------------------------------------------------------------------------------------------
// DCC Packet string declarations.
//
//------------------------------------------------------------------------------------------------------------
enum dccPacketShowOptions : uint8_t {

  SHOW_VERBOSE            = 0x01,
  SHOW_SVC_MODE           = 0x02,
  SHOW_LOC                = 0x04,
  SHOW_ACC                = 0x08,
  SHOW_HEX                = 0x10,
  SHOW_BIN                = 0x20,
  SHOW_IDLE_RESET         = 0x40
};

uint8_t  showFlags = SHOW_LOC | SHOW_ACC | SHOW_BIN | SHOW_IDLE_RESET | SHOW_VERBOSE;
char     lineBuf[ LINE_BUFFER_SIZE ];

//------------------------------------------------------------------------------------------------------------
// Forwards.
//
//------------------------------------------------------------------------------------------------------------
void fillPacket( );


//----------------------------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

  cfg = CDC::getConfigDefault( );

  cfg.EXT_INT_PIN           = 22;
  cfg.READY_LED_PIN         = 14;
  cfg.ACTIVE_LED_PIN        = 15;

  CDC::printConfigInfo( &cfg );
}

// ??? factor out as a separate object to detect and receive packets ?
//------------------------------------------------------------------------------------------------------------
// Getting the bits essentially means detecting the signal edge and then examine the signal some period later
// to see what the level is. A simple interrupt on the rising edge starting a timer interrupt period of 80
// microseconds and then reading the signal value in the timer interrupt routine should do. If low we are in
// DCC "one" bit situation, if not, it is as DCC "zero" bit situation. On detecting an edge we record the
// system time ( micros ( )) on a high signal value. On the next edge we we compare the delta system time
// against our threshold of 80 microseconds.
//
// The bits are stored in the bit buffer. From there the bits are consumed to assemble a DCC packet. DCC
// Packets are zeroes and ones, with a one having a 58 micro seconds and a zero having a 100 second half cycle.
// Note that the second approach does not even need a timer anymore. For now,
//
//------------------------------------------------------------------------------------------------------------
volatile uint32_t lastRisingTs   = 0;
volatile uint32_t lastFallingTs  = 0;

//------------------------------------------------------------------------------------------------------------
// This routine is invoked by a changing signal on the signal pin. We read in the value from the signal pin.
// A value of high means that we we detected a rising edge, for which we will just record the timestamp. On
// a signal value of low we will compute the elapsed time and compare against our threshold. Smaller than the
// threshold of 80 micro seconds is a DCC "one", else a DCC "zero".
//
// ??? I tried to measure the signal windows on AVR... but the numbers are really not good. The Scope shows
// them ranges precisely what they should be, the measured timing window with "micros" is really not good.
// ??? check again for the PICO.
//
// ??? if this is still not really good, we need a new approach .... to be designed then ...
//
//------------------------------------------------------------------------------------------------------------
void dccEdgeChange( uint8_t pin, uint8_t event ) {

  uint32_t  edgeChangeTs  = CDC::getMicros( );

  if ( event == CDC::EVT_RISE ) {

    lastRisingTs = edgeChangeTs;
  }
  else {

    lastFallingTs = edgeChangeTs;
    bitBufHead    = ( bitBufHead + 1 ) % BIT_BUFFER_SIZE;

    if ( bitBufHead != bitBufTail ) {

      uint32_t delta = lastFallingTs - lastRisingTs;

      if ( delta < ELAPSED_TIME_40 ) {

        belowSignal.add( delta );
      }
      else if ( delta < ELAPSED_TIME_80 ) {

        bitBuffer[ bitBufHead ] = 1;
        oneBitSignal.add( delta );
      }
      else if ( delta < ELAPSED_TIME_200 ) {

        bitBuffer[ bitBufHead ] = 0;
        zeroBitSignal.add( delta );
      }
      else {

        // ??? we cannot easily distinguish cutout from zero. Only chance is the long zero period....

        cutoutDetected = true;
        aboveSignal.add( delta );
      }
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// Attach an interrupt handler to the HW pin to detect a changing edge in the signal. We read in the signal
// from EXT_INT_PIN from the CDC configuration.
//
//------------------------------------------------------------------------------------------------------------
void startBitDetection( ) {

  bitBufHead = 1;
  bitBufTail = 0;

  CDC::configureDio( cfg.EXT_INT_PIN, CDC::IN );
  CDC::registerDioCallback( cfg.EXT_INT_PIN, CDC::EVT_CHANGE, dccEdgeChange );

  belowSignal.reset( );
  oneBitSignal.reset( );
  zeroBitSignal.reset( );
  aboveSignal.reset( );
}

void stopBitDetection( ) {

  // ??? to do ...
  // CDC::configureExtInt( cfg.EXT_INT_PIN, nullptr );
}

//------------------------------------------------------------------------------------------------------------
// "getBit" is the routine that works with the interrupt routine to fill the data buffer. If there are bits
// in the bit buffer, the routine will remove from the tail of the bit buffer. If there are no bits in the
// buffer, this routine will wait indefinitely for bits to arrive.
//
// ??? this is a bit unfortunate to wait forever when there are a no signal issues....
// ??? perhaps need a separate timer that just checks on the timer interrupt whether we made progress...
//------------------------------------------------------------------------------------------------------------
uint8_t getBit( ) {

  while ( bitBufTail == bitBufHead );

  uint8_t val = bitBuffer[ bitBufTail ];
  bitBufTail = ( bitBufTail + 1 ) % BIT_BUFFER_SIZE;

  return (val );
}

//------------------------------------------------------------------------------------------------------------
// "checkForPreamble" is the routine that makes sure we have seen a valid preamble. This means at least
//  DCC_MIN_PRAMBLE_BITS bits ONE followed by a ZERO bit.
//
//------------------------------------------------------------------------------------------------------------
void checkForPreamble( ) {

  bool      preambleFound         = false;
  uint8_t   preambleOneCount      = 0;
  uint32_t  preambleReadAttempts  = 0;

  while ( ! preambleFound ) {

    uint8_t nextBit = getBit( );
    if ( nextBit == 1 ) preambleOneCount++;

    if (( preambleOneCount < DCC_MIN_PRAMBLE_BITS )  && ( nextBit == 0 )) {

      preambleOneCount  = 0;
      preambleReadAttempts ++;

      if ( preambleReadAttempts > MAX_PREAMBLE_READ_ATTEMPTS ) {

        printf( "Failed to get a valid preamble..." );
        preambleReadAttempts  = 0;
      }
    }

    if (( preambleOneCount >= DCC_MIN_PRAMBLE_BITS ) && ( nextBit == 0 )) {

      preambleFound = true;
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// "getPacket" is the routine finally assembles a DCC packet from the bit stream and verifies that the packet
// has a valid checksum. Exclusive ORing all bytes of the DCC packet should result in a zero value. The first
// byte of the packet contains the packet length. We also do some of the statistics for valid, error, reset
// and idle packets. For testing the packet formatter routines, the DEBUG option fills the Dcc Packet buffer
// with valid packets from a list of test packets.
//
// ?? what if there is garbage ? how do we stop after a preamble the reading of bits that don't make sense ?
//------------------------------------------------------------------------------------------------------------
bool getPacket( ) {

  uint8_t checkSum = 0;

  #if DEBUG_PACKET_FORMATTER == 0

  bool packetEnd = false;

  memset( dccPacket, 0, sizeof( dccPacket ));
  checkForPreamble( );

  while ( ! packetEnd ) {

    uint8_t newByte = 0;
    for ( uint8_t n = 0; n < 8; n++ ) newByte = ( newByte << 1 ) + getBit( );

    checkSum ^= newByte;

    dccPacket[ 0 ] += 1;
    dccPacket[ dccPacket[ 0 ]] = newByte;

    if ( getBit( ) == 1 ) packetEnd = true;
  }

  #elif DEBUG_PACKET_FORMATTER == 1

  fillPacket( );

  #else
  #error "Need to set formatter option( 0 or 1 )"
  #endif

  if (( dccPacket[ 0 ] > 0 ) && ( checkSum == 0 )) {

    packetsDetected ++;

    if      ( LcsDccPacketFormatter::isResetPacket( dccPacket ))  resetPacketsDetected ++;
    else if ( LcsDccPacketFormatter::isIdlePacket( dccPacket ))   idlePacketsDetected ++;

    return ( true );
  }
  else {

    errPacketsDetected ++;
    return ( false );
  }
}

//------------------------------------------------------------------------------------------------------------
// "isNewPacket" searches the DCC packet buffer. It uses the packet length, the first two bytes and the
// checksum to build a packet key. If the packet is not found and there is still room in the buffer we add it
// to the buffer and indicate a new packet.
//
//------------------------------------------------------------------------------------------------------------
bool isNewPacket( uint8_t *dccPkt ) {

  uint32_t pp = dccPkt[ 0 ];

  pp = ( pp << 8 ) | dccPkt[ 1 ];
  pp = ( pp << 8 ) | dccPkt[ 2 ];
  pp = ( pp << 8 ) | dccPkt[ dccPkt[ 0 ]];

  for ( uint8_t n = 0; n < dccPacketBufferSize; n++ ) {

    if ( packetBuffer[ n ] == pp ) return ( false );
  }

  for ( uint8_t m = 0; m < dccPacketBufferSize; m++ ) {

    if ( packetBuffer[ m ] == 0 ) {

      packetBuffer[ m ] = pp;
      return ( true );
    }
  }

  return ( false );
}

//------------------------------------------------------------------------------------------------------------
// "showStatistics" will generate the statistical output to show on every elapsed recording cycle when the
// verbose option is enabled.
//
//------------------------------------------------------------------------------------------------------------
void showStatistics( ) {

  if ( showFlags & SHOW_VERBOSE ) {

    printf( "\n" );

    printf( "---> (%d), Packets: %d:%d, reset: $d, idle: %d, cut: %d", 
            intervalCount, packetsDetected, errPacketsDetected, 
            resetPacketsDetected, idlePacketsDetected, cutoutDetected );

    #if 0
    // ??? take them out for now, the numbers are just horrible... need a better method to capture time ...
    printf( ", Signal lengths: BELOW (%d:%d, ONE (%d:%d), ZERO (%d:%d)), ABOVE (%d:%d)",  
            belowSignal.getLow( ), 
            belowSignal.getHigh( ),
            oneBitSignal.getLow( ),
            oneBitSignal.getHigh( ),
            zeroBitSignal.getLow( ),
            zeroBitSignal.getHigh( ),
            aboveSignal.getLow( ),
            aboveSignal.getHigh( ));
    #endif

    printf( "\n" );
  }
}

//------------------------------------------------------------------------------------------------------------
// "showPackets" handles the DCC packets received. First we remember whether the last received packet was a
// RESET packet. This is needed for detecting that we should switch to service mode packet interpretation.
// If the new packet is not previously shown in the current display time interval, it will be displayed.
// Once in service mode, the formatter will stay in service mode until the first operations packet is received.
// For both modes, there is an option to list the packet in HEX and BINARY. The function returns the number
// of characters to the passed buffer appended.
//
//------------------------------------------------------------------------------------------------------------
void showPackets( ) {

  bool lastPacketWasReset = LcsDccPacketFormatter::isResetPacket( dccPacket );

  if ( getPacket( )) {

    if (( lastPacketWasReset ) && ( LcsDccPacketFormatter::isSvcModePacket( dccPacket )))
      showFlags |= SHOW_SVC_MODE;

    if (( showFlags & SHOW_SVC_MODE ) &&
        ( ! (( LcsDccPacketFormatter::isResetPacket( dccPacket )) ||
             ( LcsDccPacketFormatter::isSvcModePacket( dccPacket ))))) showFlags &= ~SHOW_SVC_MODE;

    if ( isNewPacket( dccPacket )) {

      int numChars = 0;

      if ((( showFlags & SHOW_IDLE_RESET ) && LcsDccPacketFormatter::isResetPacket( dccPacket )) ||
          (( showFlags & SHOW_IDLE_RESET ) && LcsDccPacketFormatter::isIdlePacket( dccPacket ))) {

        numChars = LcsDccPacketFormatter::formatDccPacketOpsMode( lineBuf, sizeof( lineBuf ), dccPacket );
      }
      else if ( showFlags & SHOW_SVC_MODE ) {

        numChars = LcsDccPacketFormatter::formatDccPacketSvcMode( lineBuf, sizeof( lineBuf ), dccPacket );
      }
      else {

        numChars = LcsDccPacketFormatter::formatDccPacketOpsMode( lineBuf, sizeof( lineBuf ), dccPacket );
      }

      if ( showFlags & SHOW_HEX ) {

        numChars +=
          LcsDccPacketFormatter::formatDccPacketHex( lineBuf + numChars, sizeof( lineBuf ) - numChars, dccPacket );
      }

      if ( showFlags & SHOW_BIN ) {

        numChars +=
          LcsDccPacketFormatter::formatDccPacketBin( lineBuf + numChars, sizeof( lineBuf ) - numChars, dccPacket );
      }

      if ( numChars > 0 ) printf( lineBuf );
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// DCC Packet buffer section. There is the idea of an interval in which packets are observed. Identical
// packets need not be printed out again. At refresh time the array is cleared and the collection starts
// again.
//
//------------------------------------------------------------------------------------------------------------
void refreshBuffer( ) {

  if ( CDC::getMillis( ) > timeToRefreshInMillis ) {

    timeToRefreshInMillis = CDC::getMillis( ) + refreshTimeInMillis;

    for ( uint8_t n = 0; n < MAX_DCC_PACKETS; n++ ) packetBuffer[ n ] = 0L;

    intervalCount ++;

    showStatistics( );
    belowSignal.reset( );
    oneBitSignal.reset( );
    zeroBitSignal.reset( );
    aboveSignal.reset( );

    packetsDetected           = 0;
    errPacketsDetected        = 0;
    resetPacketsDetected      = 0;
    idlePacketsDetected       = 0;
    cutoutDetected            = false;
  }
}

//------------------------------------------------------------------------------------------------------------
// Console IO section, welcome message.
//
//------------------------------------------------------------------------------------------------------------
void printWelcome( ) {

  printf( "DCC Packet Analyzer\n" );
  printf( "Updates every %d seconds\n", refreshTimeInMillis / 1000 );
}

//------------------------------------------------------------------------------------------------------------
// Console IO section, command line interface.
//
//------------------------------------------------------------------------------------------------------------
void checkUserInput( ) {

  char ch = CDC::getConsoleChar( );

  switch (ch ) {

    case '0': {

      showFlags               = SHOW_LOC | SHOW_ACC | SHOW_BIN | SHOW_IDLE_RESET | SHOW_VERBOSE;
      refreshTimeInMillis     = DEFAULT_REFRESH_TIME_MILLIS;
      dccPacketBufferSize     = DEFAULT_DCC_PACKETS;
      intervalCount           = 0;
      packetsDetected         = 0;
      errPacketsDetected      = 0;
      resetPacketsDetected    = 0;
      idlePacketsDetected     = 0;
      
    } break;

    case '1': {

      printf( "Refresh Time = 1s\n" );
      refreshTimeInMillis = 1000;

    } break;

    case '2': {

      printf( "Refresh Time = 2s\n" );
      refreshTimeInMillis = 2000;

    } break;

    case '3': {

      printf( "Refresh Time = 4s\n" );
      refreshTimeInMillis = 4000;

    } break;

    case '4': {

      printf( "Refresh Time = 8s\n" );
      refreshTimeInMillis = 8000;

    } break;

    case '5': {

      printf( "Refresh Time = 16s\n" );
      refreshTimeInMillis = 16000;

    } break;

    case '6': {

      printf( "Buffer Size = 16\n" );
      dccPacketBufferSize = 16;
      
    } break;

    case '7': {

      printf( "Buffer Size = 32\n" );
      dccPacketBufferSize = 32;
      
    } break;

    case '8': {

      printf( "Buffer Size = 64\n" );
      dccPacketBufferSize = 64;
      
    } break;

    case '9': {

      printf( "Buffer Size = 128\n" );
      dccPacketBufferSize = 128;
      
    } break;

    case 's': {

      printf( "Status -> " );
      printf( "Verbose: %d, ", (( showFlags & SHOW_VERBOSE ) ? 1 : 0 ));
      printf( "Svc Mode: %d, ", (( showFlags & SHOW_SVC_MODE ) ? 1 : 0 ));
      printf( "Loc: %d, ", (( showFlags & SHOW_LOC ) ? 1 : 0 ));
      printf( "Acc: %d, ", (( showFlags & SHOW_ACC ) ? 1 : 0 ));
      printf( "Hex: %d, ", (( showFlags & SHOW_HEX ) ? 1 : 0 ));
      printf( "Bin: %d, ", (( showFlags & SHOW_BIN ) ? 1 : 0 ));
      printf( "Idle-Reset: %d ", (( showFlags & SHOW_IDLE_RESET ) ? 1 : 0 ));
      printf( "Refresh-Time: %d, ", refreshTimeInMillis );
      printf( "DccPktBuf Size: %d ", dccPacketBufferSize );
      printf( "\n" );

    } break;

    case 'v': {
        
      if ( showFlags & SHOW_VERBOSE ) showFlags &= ~SHOW_VERBOSE; else showFlags |= SHOW_VERBOSE;
      printf( "show verbose = %d\n", (( showFlags & SHOW_VERBOSE ) ? 1 : 0 ));
       
    } break;

    case 'i': {

      if ( showFlags & SHOW_IDLE_RESET ) showFlags &= ~SHOW_IDLE_RESET; else showFlags |= SHOW_IDLE_RESET;
      printf( "show idle/reset packets = %d\n", (( showFlags & SHOW_IDLE_RESET ) ? 1 : 0 ));
      
    } break;

    case 'a': {

      if ( showFlags & SHOW_ACC ) showFlags &= ~SHOW_ACC; else showFlags |= SHOW_ACC;
      printf( "show acc packets = %d\n", (( showFlags & SHOW_ACC ) ? 1 : 0 ));
        
    } break;

    case 'l': {

      if ( showFlags & SHOW_LOC ) showFlags &= ~SHOW_LOC; else showFlags |= SHOW_LOC;
      printf( "show loc packets = %d\n", (( showFlags & SHOW_LOC ) ? 1 : 0 ));
        
    } break;

    case 'h': {

      if ( showFlags & SHOW_HEX ) showFlags &= ~SHOW_HEX; else showFlags |= SHOW_HEX;
      printf( "show packet data in hexadecimal = %d\n", (( showFlags & SHOW_HEX ) ? 1 : 0 ));
      
    } break;

    case 'b': {

      if ( showFlags & SHOW_BIN ) showFlags &= ~SHOW_BIN; else showFlags |= SHOW_BIN;
      printf( "show packet data in binary = %d\n", (( showFlags & SHOW_BIN ) ? 1 : 0 ));
      
    } break;

    case 'p': {

      if ( showFlags & SHOW_SVC_MODE ) showFlags &= ~SHOW_SVC_MODE; else showFlags |= SHOW_SVC_MODE;
      printf( "show service mode = %d\n", (( showFlags & SHOW_SVC_MODE ) ? 1 : 0 ));
        
    } break;

    case '?': {

      printf( "Available keyboard commands:" );
      printf( "  0 = reset, defaults" );
      printf( "  1 = 1s refresh time" );
      printf( "  2 = 2s" );
      printf( "  3 = 4s (default)" );
      printf( "  4 = 8s" );
      printf( "  5 = 16s" );
      printf( "\n" );
      printf( "  6 = 16 DCC packet buffer size" );
      printf( "  7 = 32" );
      printf( "  8 = 64 (default)" );
      printf( "  9 = 128" );
      printf( "\n" );
      printf( "  a = accessory packets display on / off toggle" );
      printf( "  l = locomotive packets display on / off toggle" );
      printf( "  i = idle packet display on / off toggle" );
      printf( "\n" );
      printf( "  h = hexadecimal output of packet data on / off toggle" );
      printf( "  b = binary output of packet data on / off toggle" );
      printf( "\n" );
      printf( "  s = show configuration" );
      printf( "  v = show verbose packet format" );
      printf( "  p = show service mode packet format" );
      printf( "\n" );

    } break;

      case '\r':  break;
      case '\n':  break;
      default:    printf( "Invalid command, use '?' for help\n" );
  }
}

//------------------------------------------------------------------------------------------------------------
// DCC Packet test data section. It helps a lot to test the correct formatting without being connected to an
// actual layout. The test data DCC packets are returned one after the other to also test sequences such as
// a reset packet followed by programming mode packet.
//
//------------------------------------------------------------------------------------------------------------
#if DEBUG_PACKET_FORMATTER == 1

uint8_t       debugFormatterTestIndex = 0;
const uint8_t testData[  ] [ 7 ]      = {

  // Loc 1 Forw 21
  { 3, 0b00000001, 0b01101100, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Idle
  { 3, 0b11111111, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Reset
  { 3, 0b00000000, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 1902 Forw 21
  { 4, 0b11000111, 0b01101110, 0b01101100, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 1902 Rev 14
  { 4, 0b11000111, 0b01101110, 0b01011000, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 4 L F4-F1 10000
  { 3, 0b00000100, 0b10010000, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 4 F8-F5 1010
  { 3, 0b00000100, 0b10111010, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 4 F12-F9 1010
  { 3, 0b00000100, 0b10101010, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 4 F20-F13 10101010
  { 4, 0b00000100, 0b11011110, 0b10101010, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 1902 F28-F21 10101010
  { 5, 0b11000111, 0b01101110, 0b11011111, 0b10101010, 0b11111111, 0b11111111 },

  // Loc 1902 For128 2
  { 5, 0b11000111, 0b01101110, 0b00111111, 0b10000010, 0b11111111, 0b11111111 },

  // Loc 4 BinStateLong 15 On
  { 5, 0b00000100, 0b11000000, 0b10001111, 0b00000000, 0b11111111, 0b11111111 },

  // Loc 4 BinStateShort 15 On
  { 4, 0b00000100, 0b11011101, 0b10001111, 0b11111111, 0b11111111, 0b11111111 },

  // Acc 13:1 1 On
  { 3, 0b10000011, 0b11111011, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 1902 CV Verify 3 128
  { 6, 0b11000111, 0b01101110, 0b11100100, 0b00000010, 0b10000000, 0b11111111 },

  // Reset
  { 3, 0b00000000, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // Reset
  { 3, 0b00000000, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111 },

  // CV Verify 1 128
  { 4, 0b01110100, 0b00000000, 0b10000000, 0b11111111, 0b11111111, 0b11111111 },

  // Loc 1902 For128 3
  { 5, 0b11000111, 0b01101110, 0b00111111, 0b10000011, 0b11111111, 0b11111111 },

  // Loc 1782 CV Write 3 128
  { 6, 0b11000110, 0b11110110, 0b11101100, 0b00000011, 0b10000000, 0b11111111 }

  // ??? add some more .... ?

};

//------------------------------------------------------------------------------------------------------------
// For the DEBUG option, this routine fills the DCC packet buffer with packet selected from the list and
// builds the checksum. The packet data contains the length of the packet including the checksum, which we
// compute and store as the last byte.
//
//------------------------------------------------------------------------------------------------------------
void fillPacket( ) {

  debugFormatterTestIndex = ( debugFormatterTestIndex + 1 ) % ( sizeof( testData ) / 7 );

  memcpy( dccPacket, &testData[ debugFormatterTestIndex ], 7 );

  uint8_t pktCnt = dccPacket[0];
  uint8_t val    = dccPacket[1];
  for ( int i = 2; i < pktCnt; i++ ) val ^= dccPacket[ i ];
  dccPacket[ pktCnt ] = val;
}

#endif // DEBUG Formatter

//----------------------------------------------------------------------------------------------------------
// Main. We first initialize the CDC layer. Next, just start the bit detection process. If we are in packet
// formatter debug mode, there is a DCC packet randomly selected from the the table of packets to test the 
// formatter. 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

  setupConfigInfo( );

  printWelcome( );

  #if DEBUG_PACKET_FORMATTER == 0

    startBitDetection( );

  #elif DEBUG_PACKET_FORMATTER == 1

    debugFormatterTestIndex = 0;

  #else
    #error "Need to set formatter option( 0 or 1 )"
  #endif

  while( true ) {

    checkUserInput( );
    refreshBuffer( );
    showPackets( );
  }
  
  return( 0 );
}