//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller - Track Power
//
//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller - Track Power
// Copyright (C) 2019 - 2024  Helmut Fieres
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
#include "LcsBlockController.h"
#include <math.h>

// ??? contains the code that manage the track power

// ??? leverage the DCC track module in base station.

// ??? descriptor has to specify the four pins for each H-Bridge


#if 0


#include <math.h>

//------------------------------------------------------------------------------------------------------------
// External global variables.
//
//------------------------------------------------------------------------------------------------------------
extern uint16_t debugMask;

//------------------------------------------------------------------------------------------------------------
// The Block Track Object local definitions. The hardware lower layers can be found in controller dependent 
// code (CDC) layer.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// DCC and RailCom definitions.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t   RAILCOM_BUFFER_SIZE         = 8;

//--------------------------------------------------------------------------------------------------------------
// Block controller global limits. Perhaps to move to a configurable place...
//
//-------------------------------------------------------------------------------------------------------------
const uint16_t MILLI_VOLT_PER_DIGIT        = 5;
const uint16_t MILLI_VOLT_PER_AMP          = 1500;

//------------------------------------------------------------------------------------------------------------
// Block track power management is also a a state machine managing the state of the power track. Maximum 
// values for the track power start and stop sequence as well as limits for power overload events are defined. 
// We also define reasonable default values.
//
//------------------------------------------------------------------------------------------------------------
const uint16_t MAX_START_TIME_THRESHOLD_MILLIS     = 2000;
const uint16_t MAX_STOP_TIME_THRESHOLD_MILLIS      = 1000;
const uint16_t MAX_OVERLOAD_TIME_THRESHOLD_MILLIS  = 500;
const uint16_t MAX_OVERLOAD_EVENT_COUNT            = 10;
const uint16_t MAX_OVERLOAD_RESTART_COUNT          = 10;

const uint16_t DEF_START_TIME_THRESHOLD_MILLIS     = 1000;
const uint16_t DEF_STOP_TIME_THRESHOLD_MILLIS      = 500;
const uint16_t DEF_OVERLOAD_TIME_THRESHOLD_MILLIS  = 300;
const uint16_t DEF_OVERLOAD_EVENT_COUNT            = 10;
const uint16_t DEF_OVERLOAD_RESTART_COUNT          = 10;

//------------------------------------------------------------------------------------------------------------
// Track state machine state definitions. See the track state machine routine for an explanation of the 
// individual states.
//
//------------------------------------------------------------------------------------------------------------
enum DccTrackState : uint8_t {

    DCC_TRACK_POWER_OFF       = 0,
    DCC_TRACK_POWER_ON        = 1,
    DCC_TRACK_POWER_OVERLOAD  = 2,
    DCC_TRACK_POWER_START1    = 3,
    DCC_TRACK_POWER_START2    = 4,
    DCC_TRACK_POWER_STOP1     = 5,
    DCC_TRACK_POWER_STOP2     = 6
};

//------------------------------------------------------------------------------------------------------------
// The DCC track object maintains an internal log facility for test and debugging purposes. During operation
// a set of log entries can be recorded to a log buffer. A log entry consist of the header byte, which 
// contains in the first byte the 4-bit log id and the 4-bit length of the log data. A log entry can therefore
// record up to 16 bytes of payload.
//
//------------------------------------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------------------------------------
// The log buffer and the log index. When writing to the log buffer, the index will always point to the
// next available position. Once the buffer is full, no further data can be added.
//
//------------------------------------------------------------------------------------------------------------
const uint16_t  LOG_BUF_SIZE            = 4096;

bool            logEnabled              = false;
bool            logActive               = false;
uint16_t        logBufIndex             = 0;
uint8_t         logBuf[ LOG_BUF_SIZE ]  = { 0 };

//------------------------------------------------------------------------------------------------------------
// RailCom decoder table. The Railcom communication will send raw bytes where only four bits are "one" in
// a byte ( hamming weight 4 ). The first two bytes are labelled "channel1" and the remaining six bytes
// are labelled "channel2". The actual data is then encode using the table below. Each raw byte will be
// translated to a 6 bits of data for the datagram to assemble. In total there are therefore a maximum
// of 48bits that are transmitted in a railcom message.
//
//------------------------------------------------------------------------------------------------------------
enum RailComDataBytes : uint8_t {

    INV   = 0xff,
    BUSY  = 0xfe,
    ACK   = 0xfd,
    NACK  = 0xfc,
    RSV1  = 0xfa,
    RSV2  = 0xf9,
    RSV3  = 0xf8
};

const uint8_t railComDecode[256] = {

    INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV,    // 0
    INV,    INV,    INV,    INV,    INV,    INV,    INV,    ACK,

    INV,    INV,    INV,    INV,    INV,    INV,    INV,    0x33,   // 1
    INV,    INV,    INV,    0x34,   INV,    0x35,   0x36,   INV,

    INV,    INV,    INV,    INV,    INV,    INV,    INV,    0x3A,   // 2
    INV,    INV,    INV,    0x3B,   INV,    0x3C,   0x37,   INV,

    INV,    INV,    INV,    0x3F,   INV,    0x3D,   0x38,   INV,    // 3
    INV,    0x3E,   0x39,   INV,    NACK,   INV,    INV,    INV,

    INV,    INV,    INV,    INV,    INV,    INV,    INV,    0x24,   // 4
    INV,    INV,    INV,    0x23,   INV,    0x22,   0x21,   INV,

    INV,    INV,    INV,    0x1F,   INV,    0x1E,   0x20,   INV,    // 5
    INV,    0x1D,   0x1C,   INV,    0x1B,   INV,    INV,    INV,

    INV,    INV,    INV,    0x19,   INV,    0x18,   0x1A,   INV,    // 6
    INV,    0x17,   0x16,   INV,    0x15,   INV,    INV,    INV,

    INV,    0x25,   0x14,   INV,    0x13,   INV,    INV,    INV,    // 7
    0x32,   INV,    INV,    INV,    INV,    INV,    INV,    INV,

    INV,    INV,    INV,    INV,    INV,    INV,    INV,    RSV2,   // 8
    INV,    INV,    INV,    0x0E,   INV,    0x0D,   0x0C,   INV,

    INV,    INV,    INV,    0x0A,   INV,    0x09,   0x0B,   INV,    // 9
    INV,    0x08,   0x07,   INV,    0x06,   INV,    INV,    INV,

    INV,    INV,    INV,    0x04,   INV,    0x03,   0x05,   INV,    // a
    INV,    0x02,   0x01,   INV,    0x00,   INV,    INV,    INV,

    INV,    0x0F,   0x10,   INV,    0x11,   INV,    INV,    INV,    // b
    0x12,   INV,    INV,    INV,    INV,    INV,    INV,    INV,

    INV,    INV,    INV,    RSV1,   INV,    0x2B,   0x30,   INV,    // c
    INV,    0x2A,   0x2F,   INV,    0x31,   INV,    INV,    INV,

    INV,    0x29,   0x2E,   INV,    0x2D,   INV,    INV,    INV,    // d
    0x2C,   INV,    INV,    INV,    INV,    INV,    INV,    INV,

    INV,    RSV3,   0x28,   INV,    0x27,   INV,    INV,    INV,    // e
    0x26,   INV,    INV,    INV,    INV,    INV,    INV,    INV,

    ACK,    INV,    INV,    INV,    INV,    INV,    INV,    INV,    // f
    INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
};

//------------------------------------------------------------------------------------------------------------
// Railcom datagrams are sent from a mobile or a stationary decoder.
//
//------------------------------------------------------------------------------------------------------------
enum railComDatagramType : uint8_t {

    RX_DG_TYPE_UNDEFINED  = 0,
    RC_DG_TYPE_MOB        = 1,
    RC_DG_TYPE_STAT       = 2
};

//------------------------------------------------------------------------------------------------------------
// Each mobile decoder railcom datagram will start with an ID field of four bits. Channel one will use only
// the ADR_HIG and ADR_LOW Ids. All IDs can be used for channel 2. Since decoders answer on channel one
// for each DCC packet they receive, here is a good chance that channel 1 will contains nonsense data. This
// is different for channel two, where only the addressed decoder explicitly answers. To decide whether
// a railcom message is valid, you should perhaps ignore channel 1 data and just check channel 2 for this
// purpose. A RC datagram starts with the 4-bit ID and an 8 to 32bit payload.
//
//      RC_DG_MOB_ID_POM       ( 0  )  - 12bit
//      RC_DG_MOB_ID_ADR_HIGH  ( 1  )  - 12bit
//      RC_DG_MOB_ID_ADR_LOW   ( 2  )  - 12bit
//      RC_DG_MOB_ID_APP_EXT   ( 3  )  - 18bit
//      RC_DG_MOB_ID_APP_DYN   ( 7  )  - 18bit
//      RC_DG_MOB_ID_XPOM_1    ( 8  )  - 36bit
//      RC_DG_MOB_ID_XPOM_2    ( 9  )  - 36bit
//      RC_DG_MOB_ID_XPOM_3    ( 10 )  - 36bit
//      RC_DG_MOB_ID_XPOM_4    ( 11 )  - 36bit
//      RC_DG_MOB_ID_TEST      ( 12 )  - ignore
//      RC_DG_MOB_ID_SEARCH    ( 14 )  - 48bit
//
// A datagram with the ID 14 is a DDC-A datagram and all 8 datagram bytes are combined to an 48bit datagram.
// A datagram packet can also contain more than one datagram. For example there could be two 18-bit length
// datagram in one packet or 3 12-bit packets and so on. Finally, unused bytes in channel two could contain
// an ACK to fill them up.
//
//------------------------------------------------------------------------------------------------------------
enum railComDatagramMobId : uint8_t {

    RC_DG_MOB_ID_POM        = 0,
    RC_DG_MOB_ID_ADR_HIGH   = 1,
    RC_DG_MOB_ID_ADR_LOW    = 2,
    RC_DG_MOB_ID_APP_EXT    = 3,
    RC_DG_MOB__IDAPP_DYN    = 7,
    RC_DG_MOB_ID_XPOM_1     = 8,
    RC_DG_MOB_ID_XPOM_2     = 9,
    RC_DG_MOB_ID_XPOM_3     = 10,
    RC_DG_MOB_ID_XPOM_4     = 11,
    RC_DG_MOB_ID_TEST       = 12,
    RC_DG_MOB_ID_SEARCH     = 14
};

//------------------------------------------------------------------------------------------------------------
// Similar to the mobile decode, a stationary decoder datagram will start an ID field of four bits. Stationary
// decoders also define a datagram with "SRQ" and no ID field to request service from the base station.
//
// ??? to fill in ...
//
//      RC_DG_STAT_ID_SRQ       ( 0  )  - 12bit
//      RC_DG_STAT_ID_POM       ( 1  )  - 12bit
//      RC_DG_STAT_ID_STAT1     ( 4  )  - 12bit
//      RC_DG_STAT_ID_TIME      ( 5  )  - xxbit
//      RC_DG_STAT_ID_ERR       ( 6  )  - xxbit
//      RC_DG_STAT_ID_XPOM_1    ( 8  )  - 36bit
//      RC_DG_STAT_ID_XPOM_2    ( 9  )  - 36bit
//      RC_DG_STAT_ID_XPOM_3    ( 10 )  - 36bit
//      RC_DG_STAT_ID_XPOM_4    ( 11 )  - 36bit
//      RC_DG_STAT_ID_TEST      ( 12 )  - ignore
//
//------------------------------------------------------------------------------------------------------------
enum railComDatagramStatId : uint8_t {

    RC_DG_STAT_ID_SRQ       = 0,
    RC_DG_STAT_ID_POM       = 1,
    RC_DG_STAT_ID_STAT1     = 4,
    RC_DG_STAT_ID_TIME      = 5,
    RC_DG_STAT_ID_ERR       = 6,
    RC_DG_STAT_ID_DYN       = 7,
    RC_DG_STAT_ID_XPOM_1    = 8,
    RC_DG_STAT_ID_XPOM_2    = 9,
    RC_DG_STAT_ID_XPOM_3    = 10,
    RC_DG_STAT_ID_XPOM_4    = 11,
    RC_DG_STAT_ID_TEST      = 12
};

//------------------------------------------------------------------------------------------------------------
// Utility routine for number range checks.
//
//------------------------------------------------------------------------------------------------------------
bool isInRangeU( uint8_t val, uint8_t lower, uint8_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

//------------------------------------------------------------------------------------------------------------
// Utility function to map a DCC address to a railcom decoder type.
//
//------------------------------------------------------------------------------------------------------------
inline uint8_t mapDccAdrToRailComDatagramType( uint16_t adr ) {

    if      (( adr >= 1 )   && ( adr <= 127 ))  return ( RC_DG_TYPE_MOB );
    else if (( adr >= 128 ) && ( adr <= 191 ))  return ( RC_DG_TYPE_STAT );
    else if (( adr >= 192 ) && ( adr <= 231 ))  return ( RC_DG_TYPE_MOB );
    else                                        return ( RX_DG_TYPE_UNDEFINED );
}

//------------------------------------------------------------------------------------------------------------
// Conversion functions between milliAmps and digit values as report4de by the analog to digital converter
// hardware. For a better precision, the formula uses 32 bit computation and stores the result back in a 
// 16 bit quantity. 
//
//------------------------------------------------------------------------------------------------------------
uint16_t milliAmpToDigitValue( uint16_t milliAmp, uint16_t digitsPerAmp ) {

    #if 0
    uint32_t mA = milliAmp;
    uint32_t dPA = digitsPerAmp;
    return (( uint16_t ) ( mA * dPA / 1000 ));
    #endif

    return ((uint16_t) ((((uint32_t) milliAmp ) * ((uint32_t) digitsPerAmp )) / 1000 ));
}

uint16_t digitValueToMilliAmp( uint16_t digitValue, uint16_t digitsPerAmp ) {

    #if 0
    uint32_t dV = digitValue;
    uint32_t dPA = digitsPerAmp;
    return ((uint16_t)( dV * 1000 / dPA ));
    #endif

    return ((uint16_t) ((((uint32_t) digitValue ) * 1000 ) / ((uint32_t) digitsPerAmp )));
}

//----------------------------------------------------------------------------------------------------------
// DCC log functions for printing the DCC log buffer. The fist byte of each log entry has encoded the log
// entry type and the entry length. Depending on the log entry type, data is displayed as just the header, 
// a numeric 16-bit value, a numeric 32-bit vale or as an array of data bytes. We return the length of the 
// DCC log entry.
//
//----------------------------------------------------------------------------------------------------------
void printLogTimeStamp( uint16_t index ) {

    uint32_t ts = logBuf[ index ];
    ts = ( ts << 8 ) | logBuf[ index + 1 ];
    ts = ( ts << 8 ) | logBuf[ index + 2 ];
    ts = ( ts << 8 ) | logBuf[ index + 3 ];
    printf( "0x%x", ts );
}

void printLogVal( uint16_t index ) {

    uint16_t val = logBuf[ index ] << 8 | logBuf[ index + 1 ];
    printf( "0x%04x", val );
}

void printLogData( uint16_t index, uint8_t len ) {

    for ( int i = 0; i < len; i++ ) printf( "0x%02x ", logBuf[ index + i ] );
}

uint8_t printLogEntry( uint16_t index ) {

    if ( index < LOG_BUF_SIZE ) {

        uint8_t logEntryId  = logBuf[ index ] >> 4;
        uint8_t logEntryLen = logBuf[ index ] & 0x0F;

        switch ( logEntryId ) {

            case LOG_NIL:      printf( "NIL        " ); break;
            case LOG_BEGIN:    printf( "BEGIN      " ); break;
            case LOG_END:      printf( "END        " ); break;
            case LOG_TSTAMP:   printf( "TSTAMP     " ); break;
            case LOG_DCC_IDL:  printf( "DCC_IDLE   " ); break;
            case LOG_DCC_RST:  printf( "DCC_RESET  " ); break;
            case LOG_DCC_PKT:  printf( "DCC_PKT    " ); break;
            case LOG_DCC_RCM:  printf( "DCC_RCOM   " ); break;
            case LOG_VAL:      printf( "VAL        " ); break;
            default:           printf( "INVALID ( 0x%02 )", logBuf[ index ] >> 4 );
        }

        if      ( logEntryId == LOG_TSTAMP  )  printLogTimeStamp( index + 1 );
        else if ( logEntryId == LOG_VAL     )  printLogVal( index + 1 );
        else                                   printLogData( index + 1, logEntryLen );

        return ( logEntryLen + 1 );
    }
    else return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
// There are a couple of routines to write the log data. For convenience, some of the log entry types are
// available as a direct call. The order of data entry for numeric types is big endian, i.e. most significant
// byte first.
//
//------------------------------------------------------------------------------------------------------------
void writeLogData( uint8_t id, uint8_t *buf, uint8_t len ) {

    if ( logActive ) {

        len = len % 16;
        if ( logBufIndex + len + 1 < LOG_BUF_SIZE ) {

            logBuf[ logBufIndex ++ ] = ( id << 4 ) | len;
            for ( uint8_t i = 0; i < len; i++ ) logBuf[ logBufIndex ++ ] = buf[ i ];
        }
    }
}

void writeLogId( uint8_t id ) {

    if ( logActive ) logBuf[ logBufIndex ++ ] = ( id << 4 ) | 1;
}

void writeLogTs( ) {

    if ( logActive ) {

        uint32_t ts = CDC::getMicros( );
        logBuf[ logBufIndex ++ ] = ( LOG_TSTAMP << 4 ) | 4;
        logBuf[ logBufIndex ++ ] = ( ts >> 24 ) & 0xFF;
        logBuf[ logBufIndex ++ ] = ( ts >> 16 ) & 0xFF;
        logBuf[ logBufIndex ++ ] = ( ts >> 8  ) & 0xFF;
        logBuf[ logBufIndex ++ ] = ( ts >> 0  ) & 0xFF;
    }
}

void writeLogVal( uint8_t valId, uint16_t val ) {

    if ( logActive ) {

        logBuf[ logBufIndex ++ ] = ( LOG_VAL << 4 ) | 3;
        logBuf[ logBufIndex ++ ] = valId;
        logBuf[ logBufIndex ++ ] = val >> 8;
        logBuf[ logBufIndex ++ ] = val & 0xFF;
    }
}

//------------------------------------------------------------------------------------------------------------
// The log management routines. A typical transaction to log would start the logging process and then end
// it after the operation to analyze/debug. The "enableLog" call should be used to enable the logging
// process all together, the other calls will only do work when the log is enabled. With this call the
// recording process could be controlled from a command line setting or so.
//
//------------------------------------------------------------------------------------------------------------
void enableLog( bool arg ) {

    logEnabled = arg;
    logActive  = false;
}

void beginLog( ) {

    if ( logEnabled ) {

        logActive   = true;
        logBufIndex = 0;
        writeLogId( LOG_BEGIN );
        writeLogTs( );
    }
}

void endLog( ) {

    if ( logActive ) {

        writeLogTs( );
        writeLogId( LOG_END );
        logActive = false;
    }
}

//------------------------------------------------------------------------------------------------------------
// A simple routine to print out the log data, one entry on one line.
//
// ??? what is exactly the stop condition ? The END entry having a length of zero ?
//------------------------------------------------------------------------------------------------------------
void printLog( ) {

    if ( logEnabled ) {

        if ( ! logActive ) {

            if ( logBufIndex > 0 ) {

                printf( "\n" );

                uint16_t entryIndex  = 0;
                uint8_t  entryLen    = 0;

                while ( entryIndex < logBufIndex ) {

                    entryLen = printLogEntry( entryIndex );
                    printf( "\n" );

                    if ( entryLen > 0 ) entryIndex += entryLen;
                    else                break;
                }
            }
            else printf( "DCC Log Buf: Nothing recorded\n" );
        }
        else printf( "DCC Log Active\n" );
    }
    else printf( "DCC Log disabled\n" );
}

}; // namespace


//============================================================================================================
//============================================================================================================
//
// Object part.
//
//============================================================================================================
//============================================================================================================


//------------------------------------------------------------------------------------------------------------
// Object instance section. The DccTrack constructor. Nothing to do so far.
//
//------------------------------------------------------------------------------------------------------------
LcsBaseStationDccTrack::LcsBaseStationDccTrack( ) { }

//------------------------------------------------------------------------------------------------------------
// "setupDccTrack" performs the setup tasks for the DCC track.  We will configure the hardware, the DCC
// packet options such as preamble and postamble length, the initial state machine state current consumption
// limit and load the initial packet into the active buffer. There is quite a list of parameters and options
// that can be set. This routine does the following checking:
//
//    - the pins used in the CDC layer must be a pair ( for atmega controllers ).
//    - the sensePin must be an analog input pin.
//    - if the track is a service track, cutout and RailCom are not supported.
//    - if RailCom is set, Cutout must be set too.
//    - the initial current limit consumption setting must be less than the current limit setting.
//    - the current limit setting must be less than the maximum current limit setting.
//
// Once the DCC track object is initialized, the last thing to do is to remember the object instance in the
// file static variables. This is necessary for the interrupt handlers to work. If any of the checks fails,
// the flag field will have the error bit set.
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsBaseStationDccTrack::setupDccTrack( LcsBaseStationTrackDesc* trackDesc ) {

    if ((  trackDesc -> enablePin  == CDC::UNDEFINED_PIN ) ||
        (  trackDesc -> dccSigPin1 == CDC::UNDEFINED_PIN ) ||
        (  trackDesc -> dccSigPin2 == CDC::UNDEFINED_PIN ) ||
        (  trackDesc -> sensePin   == CDC::UNDEFINED_PIN )) {

        flags = DT_F_CONFIG_ERROR;
        return ( ERR_DCC_PIN_CONFIG );
    }

    if ((( trackDesc -> options & DT_OPT_SERVICE_MODE_TRACK ) && ( trackDesc -> options & DT_OPT_CUTOUT ))    ||
        (( trackDesc -> options & DT_OPT_SERVICE_MODE_TRACK ) && ( trackDesc -> options & DT_OPT_RAILCOM ))   ||
        (( trackDesc -> options & DT_OPT_RAILCOM ) && ( ! ( trackDesc -> options & DT_OPT_CUTOUT )))          ||
        ( trackDesc -> initCurrentMilliAmp  > trackDesc -> limitCurrentMilliAmp )                             ||
        ( trackDesc -> limitCurrentMilliAmp > trackDesc -> maxCurrentMilliAmp )                               ||
        ( trackDesc -> startTimeThresholdMillis > MAX_START_TIME_THRESHOLD_MILLIS )                           ||
        ( trackDesc -> stopTimeThresholdMillis > MAX_STOP_TIME_THRESHOLD_MILLIS )                             ||
        ( trackDesc -> overloadTimeThresholdMillis > MAX_OVERLOAD_TIME_THRESHOLD_MILLIS )                     ||
        ( trackDesc -> overloadEventThreshold > MAX_OVERLOAD_EVENT_COUNT )                                    ||
        ( trackDesc -> overloadRestartThreshold > MAX_OVERLOAD_RESTART_COUNT )
        ) {

        flags = DT_F_CONFIG_ERROR;
        return ( ERR_DCC_TRACK_CONFIG );
    }

    signalState               = DCC_SIG_START_BIT;
    trackState                = DCC_TRACK_POWER_OFF;
    flags                     = DT_F_DEFAULT_SETTING;
    options                   = trackDesc -> options;
    enablePin                 = trackDesc -> enablePin;
    dccSigPin1                = trackDesc -> dccSigPin1;
    dccSigPin2                = trackDesc -> dccSigPin2;
    sensePin                  = trackDesc -> sensePin;
    uartRxPin                 = trackDesc -> uartRxPin;
    initCurrentMilliAmp       = trackDesc -> initCurrentMilliAmp;
    limitCurrentMilliAmp      = trackDesc -> limitCurrentMilliAmp;
    maxCurrentMilliAmp        = trackDesc -> maxCurrentMilliAmp;
    startTimeThreshold        = trackDesc -> startTimeThresholdMillis;
    stopTimeThreshold         = trackDesc -> stopTimeThresholdMillis;
    overloadTimeThreshold     = trackDesc -> overloadTimeThresholdMillis;
    overloadEventThreshold    = trackDesc -> overloadEventThreshold;
    overloadRestartThreshold  = trackDesc -> overloadRestartThreshold;

    // ??? MILLI_VOLT_PER_DIGIT is actually 4,72V / 1024 = 4,6 mV. How to make this more precise ?

    milliVoltPerAmp           = trackDesc -> milliVoltPerAmp;
    digitsPerAmp              = milliVoltPerAmp / MILLI_VOLT_PER_DIGIT;

    limitCurrentDigitValue    = milliAmpToDigitValue( initCurrentMilliAmp, digitsPerAmp );
    ackThresholdDigitValue    = milliAmpToDigitValue( ACK_TRESHOLD_VAL, digitsPerAmp );
    actualCurrentDigitValue   = 0;
    dccPacketsSend            = 0;
    totalPwrSamplesTaken      = 0;
    lastPwrSamplePerSecTaken  = 0;
    pwrSamplesPerSec          = 0;

    CDC::configureDio( enablePin, CDC::OUT );
    CDC::configureDio( dccSigPin1, CDC::OUT );
    CDC::configureDio( dccSigPin2, CDC::OUT );
    CDC::configureAdc( sensePin );

    CDC::writeDio( enablePin, false );
    CDC::writeDioPair( dccSigPin1, false, dccSigPin2, false );

    CDC::onTimerEvent( timerCallback );

    if ( options & DT_OPT_SERVICE_MODE_TRACK ) {

        progTrack     =   this;
        preambleLen   =   PROG_PACKET_PREAMBLE_LEN;
        postambleLen  =   PROG_PACKET_POSTAMBLE_LEN;
        flags         |=  DT_F_SERVICE_MODE_ON;
        activeBufPtr  = &resetDccPacket;
        pendingBufPtr = &dccBuf1;
    }
    else {

        mainTrack     =   this;
        preambleLen   =   MAIN_PACKET_PREAMBLE_LEN;
        postambleLen  =   MAIN_PACKET_POSTAMBLE_LEN;
        activeBufPtr  = &idleDccPacket;
        pendingBufPtr = &dccBuf1;
    }

    if ( trackDesc -> options & DT_OPT_CUTOUT ) {

        preambleLen =  MAIN_PACKET_PREAMBLE_LEN - DCC_PACKET_CUTOUT_LEN;
        flags       |= DT_F_CUTOUT_MODE_ON;
        signalState =  DCC_SIG_CUTOUT_START;
    }

    if ( trackDesc -> options & DT_OPT_RAILCOM ) {

        flags |= DT_F_RAILCOM_MODE_ON;
        if ( CDC::configureUart( uartRxPin, CDC::UNDEFINED_PIN, 250000, CDC::UART_MODE_8N1 ) != ALL_OK ) {

            flags = DT_F_CONFIG_ERROR;
            return ( ERR_DCC_TRACK_CONFIG );
        }
    }

    return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
// Railcom. If the cutout period and the RailCom feature is enabled, the signal state machine will also start
// and stop the UART reader for RailCom data. The final message is then to handle that message. In the cutout
// period, a decoder sends 8 data bytes. They are divided into two channels, 2bytes and another 6 bytes. The
// bytes themselves are encoded such that each byte has four bits set, i.e. a hamming weight of 4. The first
// channel is used to just send the locomotive address when the decoder is addressed. The second channel is
// used only when the decoder is explicitly addressed via a CV operation command to provide the answer to the
// request.
//
// The received datagrams are also recorded in the DCC_LOG, if enabled.
//
// ??? under construction....
// ??? we could store the last loco address in some global variable.
// ??? we could store the channel 2 datagram in the corresponding session.
// ??? still, both pieces of data needs to go somewhere before the next message is received...
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::startRailComIO( ) {

    CDC::startUartRead( uartRxPin );
}

void LcsBaseStationDccTrack::stopRailComIO( ) {

    CDC::stopUartRead( uartRxPin );
}

uint8_t LcsBaseStationDccTrack::handleRailComMsg( ) {

    railComBufIndex = CDC::getUartBuffer( uartRxPin, railComMsgBuf, sizeof( railComMsgBuf ));

    writeLogData( LOG_DCC_RCM, railComMsgBuf, railComBufIndex );

    for ( uint8_t i = 0; i < railComBufIndex; i++ ) {

        uint8_t dataByte = railComDecode[ railComMsgBuf[ i ]];

        if      ( dataByte == ACK ) ;
        else if ( dataByte == NACK ) ;
        else if ( dataByte == BUSY ) ;
        else if ( dataByte < 64 ) {

            // ??? valid
            // ??? a railCom message can have multiple datagrams
            // we would need to handle each datagram, one at a time or fill them into a kind of structure
            // that has a slot for the up to maximum 4 datagrams per railCom cutout period.
        }
        else {

            // ??? invalid packet ... if this is channel2, discard the entire message.
        }

        railComMsgBuf[ i ] = dataByte;
    }

    flags &= ~ DT_F_RAILCOM_MSG_PENDING;
    return ( ALL_OK );
}

// ??? not very useful, but good for debugging and initial testing .... and it works like a champ :-)

uint8_t LcsBaseStationDccTrack::getRailComMsg( uint8_t *buf, uint8_t bufLen ) {

    if (( railComBufIndex > 0 ) && ( bufLen > 0 )) {

        uint8_t i = 0;

        do {

            buf[ i ] = railComMsgBuf[ i ];
            i++;

        } while (( i < railComBufIndex ) && ( i < bufLen ));

        return ( i );

    } else return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
// DCC track power is not just a matter of turning power on or off. To address all the requirements of the
// standard, the track is managed by a state machine that implements the start and stop sequences. It is also
// important that we do not really block the progress of the entire base station, so any timing calls are
// handled by timestamp comparison in state machine WAIT states. The track state machine routine is expected
// to be called very often.
//
//  DCC_TRACK_POWER_START1    - this is the first state of a start sequence. When the track should be powered
//                              on, the first activity is to set the status flags and enable the power module.
//                              We set the power module current consumption to the initial limit configured.
//                              The next state is TRACK_POWER_START2.
//
//  DCC_TRACK_POWER_START2    - we stay in this state until the threshold time has passed. Once the threshold
//                              is reached, the current consumption limit is set to the configured limit.
//                              Then we move on to DCC_TRACK_POWER_ON.
//
//  DCC_TRACK_POWER_ON        - this is the state when power is on and things are running normal. An overload
//                              situation is set by the current measurement routines through setting the
//                              overload status flag. We make sure that we have seen a couple of overloads
//                              in a row before taking action which is to turn power off and set the
//                              DCC_TRACK_POWER_OVERLOAD state. Otherwise we stay in this state.
//
//  DCC_TRACK_POWER_OVERLOAD  - with power turned off, we stay in this state until the threshold time has
//                              passed. If passed, the overload restart count is incremented and checked for
//                              its threshold. If reached, we have tried to restart several times and failed.
//                              The track state becomes DCC_TRACK_POWER_STOP1, something is wrong on the track.
//                              If not, we move on to DCC_TRACK_POWER_START1.
//
//  DCC_TRACK_POWER_STOP1     - this state initiates a shutdown sequence. We disable the power module, set
//                              status flags and advance to the DCC_TRACK_POWER_STOP2 state.
//
//  DCC_TRACK_POWER_STOP2     - we stay in this state until the configured threshold has passed. Then we move
//                              on to DCC_TRACK_POWER_OFF. The key reason for this time delay is to implement
//                              the requirement that track turned off and perhaps switched to another mode,
//                              should be powerless for one second. Switch track modes becomes simply a matter
//                              of stopping and then starting again.
//
//  DCC_TRACK_POWER_OFF       - the track is disabled. We just stay in this state until the state is set to
//                              a different state from outside.
//
// During the power on state, we also append the actual current measurement value to a circular buffer when
// the time interval for this kind of measurement has passed. The idea is to measure the samples at a more
// or less constant interval rate and compute the power consumption RMS value from the data in the buffer
// when requested. In the interest of minimizing the controller load, the calculation is done in digit values
// the result is presented in then in milliAmps.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::runDccTrackStateMachine( ) {

    switch ( trackState ) {

        case DCC_TRACK_POWER_START1: {

            // ??? do we need a way to check for overload during this initial phase, just like we do when ON ?

            trackTimeStamp          = CDC::getMillis( );
            flags                   |= DT_F_POWER_ON;
            flags                   &= ~DT_F_POWER_OVERLOAD;
            flags                   &= ~DT_F_MEASUREMENT_ON;
            limitCurrentDigitValue  = milliAmpToDigitValue( initCurrentMilliAmp, digitsPerAmp );

            CDC::writeDio( enablePin, true );
            trackState = DCC_TRACK_POWER_START2;

        }  break;

        case DCC_TRACK_POWER_START2: {

            if (( CDC::getMillis( ) - trackTimeStamp ) > startTimeThreshold ) {

                highWaterMarkDigitValue = 0;
                actualCurrentDigitValue = 0;
                overloadRestartCount    = 0;
                overloadEventCount      = 0;
                flags                   |= DT_F_POWER_ON | DT_F_MEASUREMENT_ON;
                limitCurrentDigitValue  = milliAmpToDigitValue( limitCurrentMilliAmp, digitsPerAmp );

                CDC::writeDio( enablePin, true );
                trackState = DCC_TRACK_POWER_ON;
            }

        } break;

        case DCC_TRACK_POWER_ON: {

            if (( CDC::getMillis( ) - lastPwrSampleTimeStamp ) > PWR_SAMPLE_TIME_INTERVAL_MILLIS ) {

                pwrSampleBuf[ pwrSampleBufIndex % DCC_TRACK_POWER_ON ] = actualCurrentDigitValue;
                pwrSampleBufIndex ++;
                lastPwrSampleTimeStamp = CDC::getMillis( );
            }

            if (( CDC::getMillis( ) - lastPwrSamplePerSecTimeStamp ) > 1000 ) {

                pwrSamplesPerSec          = totalPwrSamplesTaken - lastPwrSamplePerSecTaken;
                lastPwrSamplePerSecTaken  = totalPwrSamplesTaken;
                lastPwrSamplePerSecTimeStamp = CDC::getMillis( );
            }

            if ( flags & DT_F_POWER_OVERLOAD ) {

                overloadEventCount  ++;

                if ( overloadEventCount > overloadEventThreshold ) {

                    if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_TRACK_POWER_MGMT )) {

                        printf( "Overload detected: " );

                        if ( options &  DT_OPT_SERVICE_MODE_TRACK ) printf( "Prog Track: " );
                        else                                        printf( "Main Track: " );

                        #if 0
                        printf( "(hwm(mA): %d : limit(mA): %d )\n", 
                                digitValueToMilliAmp( highWaterMarkDigitValue, digitsPerAmp ),
                                digitValueToMilliAmp( limitCurrentDigitValue, digitsPerAmp ));
                        
                        #else
                        printf( "(hwm(dVal): %d  : limit(dVal): %d )\n", highWaterMarkDigitValue, limitCurrentDigitValue );
                        #endif
                    }

                    trackTimeStamp  = CDC::getMillis( );
                    flags           |= DT_F_POWER_OVERLOAD;
                    flags           &= ~DT_F_POWER_ON;
                    flags           &= ~DT_F_MEASUREMENT_ON;

                    CDC::writeDio( enablePin, false );
                    trackState = DCC_TRACK_POWER_OVERLOAD;
                }
            }

        }  break;

        case DCC_TRACK_POWER_OVERLOAD: {

            if ( CDC::getMillis( ) - trackTimeStamp > overloadTimeThreshold ) {

                overloadRestartCount ++;

                if ( overloadRestartCount > overloadRestartThreshold ) {

                    if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_TRACK_POWER_MGMT )) {

                        printf( "Overload restart failed, Cnt:%d\n", overloadRestartCount );
                    }

                    trackState = DCC_TRACK_POWER_STOP1;
                }
                else trackState = DCC_TRACK_POWER_START1;
            }

        }  break;

        case DCC_TRACK_POWER_STOP1: {

            trackTimeStamp  = CDC::getMillis( );
            flags           &= ~DT_F_POWER_ON;
            flags           &= ~DT_F_POWER_OVERLOAD;
            flags           &= ~DT_F_MEASUREMENT_ON;

            CDC::writeDio( enablePin, false );
            trackState = DCC_TRACK_POWER_STOP2;

        }  break;

        case DCC_TRACK_POWER_STOP2: {

            if ( CDC::getMillis( ) - trackTimeStamp > stopTimeThreshold ) trackState = DCC_TRACK_POWER_OFF;

        } break;

        case DCC_TRACK_POWER_OFF: {

        } break;
    }
}

//------------------------------------------------------------------------------------------------------------
// Some getter functions. Straightforward.
//
//------------------------------------------------------------------------------------------------------------
uint16_t LcsBaseStationDccTrack::getFlags( ) {

    return ( flags );
}

uint16_t LcsBaseStationDccTrack::getOptions( ) {

    return ( options );
}

uint32_t LcsBaseStationDccTrack::getPwrSamplesTaken( ) {

    return ( totalPwrSamplesTaken );
}

uint16_t LcsBaseStationDccTrack::getPwrSamplesPerSec( ) {

    return ( pwrSamplesPerSec );
}

bool LcsBaseStationDccTrack::isPowerOn( ) {

    return ( flags & DT_F_POWER_ON );
}

bool LcsBaseStationDccTrack::isPowerOverload( ) {

    return ( flags & DT_F_POWER_OVERLOAD );
}

bool LcsBaseStationDccTrack::isServiceModeOn( ) {

    return ( flags & DT_F_SERVICE_MODE_ON );
}

bool LcsBaseStationDccTrack::isCutoutOn( ) {

    // ??? rather detected than set...

    return ( flags & DT_F_CUTOUT_MODE_ON );
}

bool LcsBaseStationDccTrack::isRailComOn( ) {

    return ( flags & DT_F_RAILCOM_MODE_ON );
}

//------------------------------------------------------------------------------------------------------------
// DCC track power management functions. The actual state of track power is kept in the track status field
// and can be queried or set by setting the respective flag. Starting and stopping track power is done by
// setting the respective START or STOP state.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::powerStart( ) {

    trackState = DCC_TRACK_POWER_START1;
}

void LcsBaseStationDccTrack::powerStop( ) {

    trackState = DCC_TRACK_POWER_STOP1;
}

void LcsBaseStationDccTrack::serviceModeOn( ) {

    if ( options & DT_OPT_SERVICE_MODE_TRACK ) flags |= DT_F_SERVICE_MODE_ON;
}

void LcsBaseStationDccTrack::serviceModeOff( ) {

    if ( options & DT_OPT_SERVICE_MODE_TRACK ) flags &= ~DT_F_SERVICE_MODE_ON;
}

void LcsBaseStationDccTrack::railComOn( ) {

    if ( ! ( options & DT_OPT_SERVICE_MODE_TRACK )) {

        flags |= DT_F_CUTOUT_MODE_ON | DT_F_RAILCOM_MODE_ON;
    }
}

void LcsBaseStationDccTrack::railComOff( ) {

    if ( ! ( options & DT_OPT_SERVICE_MODE_TRACK )) flags &= ~DT_F_RAILCOM_MODE_ON;
}

//------------------------------------------------------------------------------------------------------------
// Power Consumption Management. There are two key values. The first is the actual current consumption as
// measured by the ADC hardware on each ZERO DCC bit. This value is used to do the power overload checking.
// The second value is the high water mark built from these measurements. This values is used for the DCC
// decoder programming logic. The high water mark will be set to zero before collecting measurements. All
// measurement values are actually ADC digit values for performance reason. Only on limit setting and external
// data access are these values converted from and to milliAmps.
//
//------------------------------------------------------------------------------------------------------------
uint16_t LcsBaseStationDccTrack::getLimitCurrent( ) {

    return ( limitCurrentMilliAmp );
}

uint16_t LcsBaseStationDccTrack::getActualCurrent( ) {

    return ( digitValueToMilliAmp( actualCurrentDigitValue, digitsPerAmp ));
}

uint16_t LcsBaseStationDccTrack::getInitCurrent( ) {

    return ( initCurrentMilliAmp );
}

uint16_t LcsBaseStationDccTrack::getMaxCurrent( ) {

    return ( maxCurrentMilliAmp );
}

void LcsBaseStationDccTrack::setLimitCurrent( uint16_t val ) {

    if      ( val < initCurrentMilliAmp )  val = initCurrentMilliAmp;
    else if ( val > maxCurrentMilliAmp  )  val = maxCurrentMilliAmp;

    limitCurrentMilliAmp    = val;
    limitCurrentDigitValue  = milliAmpToDigitValue( val, digitsPerAmp );
}

//------------------------------------------------------------------------------------------------------------
// The "getRMSCurrent" function returns the power consumption based on the samples taken and stored in the
// sample buffer. The function computes the square root of the sum of the squares of the array elements. The
// result is returned in milliAmps. Note that our measurement is based on unsigned 16-bit quantities that come
// from the controller ADC converter. We compute the RMS based on 16-bit unsigned integers, which compared
// to floating point computation is not really precise. However, for our purpose to just show a rough power
// consumption, the error should be not a big issue. We will not use RMS values for power overload detection
// or decoder ACK detection.
//
//------------------------------------------------------------------------------------------------------------
uint16_t LcsBaseStationDccTrack::getRMSCurrent( ) {

    uint32_t res = 0;

    for ( uint8_t i = 0; i < PWR_SAMPLE_BUF_SIZE; i++ ) res += pwrSampleBuf[ i ] * pwrSampleBuf[ i ];

    return ( digitValueToMilliAmp( sqrt( res / PWR_SAMPLE_BUF_SIZE ), digitsPerAmp ));
}

//------------------------------------------------------------------------------------------------------------
// This function is called whenever a power measurement operation completes from the analog conversion
// interrupt handler. This typically takes place on the first half of the DCC "0" bit. If power measurement
// is enabled, we increment the number of samples taken, check the measured value for an overload situation
// and also set the high water mark accordingly. Since we are part of an interrupt handler, keep the amount
// work really short.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::powerMeasurement( ) {

    if ( flags & DT_F_MEASUREMENT_ON ) {

        actualCurrentDigitValue = CDC::readAdc( sensePin );

        totalPwrSamplesTaken ++;

        if ( actualCurrentDigitValue > highWaterMarkDigitValue ) highWaterMarkDigitValue = actualCurrentDigitValue;
        if ( actualCurrentDigitValue > limitCurrentDigitValue ) flags |= DT_F_POWER_OVERLOAD;
    }
}


//------------------------------------------------------------------------------------------------------------
// The log management routines. A typical transaction to log would start the logging process and then end
// it after the operation to analyze/debug. The "enableLog" call should be used to enable the logging
// process all together, the other calls will only do work when the log is enabled. With this call the
// recording process could be controlled from a command line setting or so. "beginLog" and "endLog" start 
// and end a recording sequence.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::enableLog( bool arg ) {

    logEnabled = arg;
    logActive  = false;
}

void LcsBaseStationDccTrack::beginLog( ) {

    if ( logEnabled ) {

        logActive   = true;
        logBufIndex = 0;
        writeLogId( LOG_BEGIN );
        writeLogTs( );
    }
}

void LcsBaseStationDccTrack::endLog( ) {

    if ( logActive ) {

        writeLogTs( );
        writeLogId( LOG_END );
        logActive = false;
    }
}

//------------------------------------------------------------------------------------------------------------
// There are a couple of routines to write the log data when the logging is active. For convenience, some of
// the log entry types are available as a direct call. The order of data entry for numeric types is big endian,
// i.e. most significant byte first.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::writeLogData( uint8_t id, uint8_t *buf, uint8_t len ) {

    if ( logActive ) {

        len = len % 16;
        if ( logBufIndex + len + 1 < LOG_BUF_SIZE ) {

            logBuf[ logBufIndex ++ ] = ( id << 4 ) | len;
            for ( uint8_t i = 0; i < len; i++ ) logBuf[ logBufIndex ++ ] = buf[ i ];
        }
    }
}

void LcsBaseStationDccTrack::writeLogId( uint8_t id ) {

    if ( logActive ) logBuf[ logBufIndex ++ ] = ( id << 4 );
}

void LcsBaseStationDccTrack::writeLogTs( ) {

    if ( logActive ) {

        uint32_t ts = CDC::getMicros( );
        logBuf[ logBufIndex ++ ] = ( LOG_TSTAMP << 4 ) | 4;
        logBuf[ logBufIndex ++ ] = ( ts >> 24 ) & 0xFF;
        logBuf[ logBufIndex ++ ] = ( ts >> 16 ) & 0xFF;
        logBuf[ logBufIndex ++ ] = ( ts >> 8  ) & 0xFF;
        logBuf[ logBufIndex ++ ] = ( ts >> 0  ) & 0xFF;
    }
}

void LcsBaseStationDccTrack::writeLogVal( uint8_t valId, uint16_t val ) {

    if ( logActive ) {

        logBuf[ logBufIndex ++ ] = ( LOG_VAL << 4 ) | 3;
        logBuf[ logBufIndex ++ ] = valId;
        logBuf[ logBufIndex ++ ] = val >> 8;
        logBuf[ logBufIndex ++ ] = val & 0xFF;
    }
}

//------------------------------------------------------------------------------------------------------------
// Print out the log data, one entry on one line. We only print the log buffer when there is no log sequence
// active.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::printLog( ) {

    if ( logEnabled ) {

        if ( ! logActive ) {

            if ( logBufIndex > 0 ) {

                printf( "\n" );

                uint16_t entryIndex  = 0;
                uint8_t  entryLen    = 0;

                while ( entryIndex < logBufIndex ) {

                    entryLen = printLogEntry( entryIndex );
                    printf( "\n" );

                    if ( entryLen > 0 ) entryIndex += entryLen;
                    else                break;
                }
            }
            else printf( "DCC Log Buf: Nothing recorded\n" );
        }
        else printf( "DCC Log Active\n" );
    }
    else printf( "DCC Log disabled\n" );
}

//------------------------------------------------------------------------------------------------------------
// Print out the DCC Track configuration data. For debugging purposes.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::printDccTrackConfig( ) {

    printf( "DccTrack Config: " );

    if ( options & DT_OPT_SERVICE_MODE_TRACK ) printf( "PROG \n" );
    else                                       printf( "MAIN \n" );

    printf( " Config options: ( 0x%x ) -> ", flags );
    
    if ( options &  DT_OPT_SERVICE_MODE_TRACK ) printf( "SvcMode Track " );
    if ( options & DT_OPT_CUTOUT ) printf( "Cutout " );
    if ( options & DT_OPT_RAILCOM ) printf( "Railcom " );
    printf( "\n" );

    printf( " Current Initial(mA): %d Current Limit(mA): %d Current Max(mA): %d\n",
            getInitCurrent( ), getLimitCurrent( ), getMaxCurrent( ));
    printf( " milliVoltPerAmp: %d\n", milliVoltPerAmp ); 
    printf( " digitsPerAmp: %d\n", digitsPerAmp );

    printf( " Limit Digit Value: %d\n", limitCurrentDigitValue );
    printf( " Ack Threshold Digit Value:%d\n", ackThresholdDigitValue );

    printf( " CDC enable Pin: %d, DCC signal Pins: (%d:%d), Sensor Pin: %d, RailCom Pin: %d\n",
            enablePin, dccSigPin1, dccSigPin2, sensePin, uartRxPin );

    printf( " PreambleLen: %d, PostambleLen: %d\n", preambleLen, postambleLen );
}

//------------------------------------------------------------------------------------------------------------
// Print out the DCC Track status.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::printDccTrackStatus( ) {

    printf( "DccTrack: " );

    if ( options & DT_OPT_SERVICE_MODE_TRACK )  printf( "PROG" );
    else                                        printf( "MAIN" );

    printf( ", Track Status: ( 0x%x ) -> ", flags );
    
    if ( flags & DT_F_POWER_ON         ) printf( "PowerOn " );
    if ( flags & DT_F_POWER_OVERLOAD   ) printf( "PowerOverload " );
    if ( flags & DT_F_MEASUREMENT_ON   ) printf( "PowerMeasOn " );
    if ( flags & DT_F_SERVICE_MODE_ON  ) printf( "SvcModeOn " );
    if ( flags & DT_F_CUTOUT_MODE_ON   ) printf( "CutoutOn " );
    if ( flags & DT_F_RAILCOM_MODE_ON  ) printf( "RailcomOn " );
    if ( flags & DT_F_CONFIG_ERROR     ) printf( "ConfigError " );
    printf( "\n" );

    printf( "Packets Send: %d\n", dccPacketsSend );
    printf( "Total Power Samples: %d\n", totalPwrSamplesTaken );
    printf( "Power Samples per Sec: %d\n", pwrSamplesPerSec );
    printf( "Power consumption (RMS): %d\n", getRMSCurrent( ));
    printf( "\n" );
}


#endif


