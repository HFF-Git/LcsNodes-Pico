//----------------------------------------------------------------------------------------
//
// LCS Base Station - DCC Track - implementation file
//
//----------------------------------------------------------------------------------------
// The DCC track object is one of the the key objects for the DCC subsystem. It is responsible for the DCC 
// track signal generation and the power management functions. There will be exactly two objects of this kind,
// one for the MAIN track and the other for the PROG track. The DCC track object has two major functional 
// parts. The first is to transmit a DCC packet to the track. This is the most important task, as with no
// packets no power is on the tracks and the locomotive will not work. The second task is to continuously 
// monitor the current consumption. Finally, for the RailCom option, the cutout generation and receiving 
// of the RailCOm packets is handled.
//
//----------------------------------------------------------------------------------------
//
// LCS - Base Station DCC Track implementation file
// Copyright (C) 2019 - 2025  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should
// have received a copy of the GNU General Public License along with this program. 
// If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsBaseStation.h"
#include <math.h>

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// DCC Signal debugging. A tick is defined to last 29 microseconds. There is a debugging option to set the
// clock much slower so that the waveform can be seen.
//
// ??? take out, we are past that ...... since a long time. -> one last check than out ...
//----------------------------------------------------------------------------------------

#define DEBUG_WAVE_FORM 0               

#if DEBUG_WAVE_FORM == 1
#define TICK_IN_MICROSECONDS  400000
#else
#define TICK_IN_MICROSECONDS  29
#endif

//----------------------------------------------------------------------------------------
// The DccTrack Object local definitions. The DCC track object is a bit special. There are exactly two object
// instances created, MAIN and PROG. Both however share the global mechanism for generating the DCC hardware
// signals. There are callback functions for the DCC timer and the serial I/O capability for the RailCom 
// feature. The hardware lower layers can be found in controller dependent code (CDC) layer.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// The DCC Track will allocate two DCC Track Objects. For the interrupt system to work, references to the
// objects must be static variables. The initialization sequence outside of this class will allocate the two
// objects and we keep a copy of the respective DCC track object created right here.
//
// ??? when we use the global variables in the "main" file, can this go away ?
 //----------------------------------------------------------------------------------------
LcsBaseStationDccTrack  *mainTrack  = nullptr;
LcsBaseStationDccTrack  *progTrack  = nullptr;

//----------------------------------------------------------------------------------------
// DCC packet definitions. A DCC packet payload is at most 15 bytes long, excluding the checksum byte. This
// is true for XPOM and DCC-A support, otherwise it is according to NMRA up to 6 bytes. The preamble is a
// series of "ONE" bits, which helps the decoders to sync to the bit stream. The standard specifies a
// minimum of 16 ONE bits for the MAIN track and 22 ONE bits for the PROG track. The postamble is exactly
// one "ONE" bit. If the cutout period option is enabled, the cutout overlays the first ONE bits the
// preamble.
//
//----------------------------------------------------------------------------------------
const uint8_t   MAIN_PACKET_PREAMBLE_LEN    = 17;
const uint8_t   MAIN_PACKET_POSTAMBLE_LEN   = 1;
const uint8_t   PROG_PACKET_PREAMBLE_LEN    = 22;
const uint8_t   PROG_PACKET_POSTAMBLE_LEN   = 1;
const uint8_t   DCC_PACKET_CUTOUT_LEN       = 4;
const uint8_t   MIN_DCC_PACKET_SIZE         = 2;
const uint8_t   MAX_DCC_PACKET_SIZE         = 16;
const uint8_t   MIN_DCC_PACKET_REPEATS      = 0;
const uint8_t   MAX_DCC_PACKET_REPEATS      = 8;
const uint8_t   RAILCOM_BUFFER_SIZE         = 8;

//----------------------------------------------------------------------------------------
// Constant values definition. We need the RESET and IDLE packet as well as a bit mask for a quick bit
// select in the data byte.
//
//----------------------------------------------------------------------------------------
DccPacket       idleDccPacket       = { 3, 0, { 0xFF, 0x00, 0xFF }};
DccPacket       resetDccPacket      = { 3, 0, { 0x00, 0x00, 0x00 }};
const uint8_t   bitMask9[ ]         = { 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01 };

//----------------------------------------------------------------------------------------
// Programming decoders require to detect a short rise in power consumption. The value is at least 60mA,
// but decoders can raise anything from 100mA to 250mA. This is a bit touchy and the value set to 100mA
// was done after testing several decoders. Still, a bit flaky ...
//
//----------------------------------------------------------------------------------------
const uint8_t ACK_TRESHOLD_VAL      = 100;

//----------------------------------------------------------------------------------------
// The DCC signal generator thinks in ticks. With a DCC ONE based on 58 microseconds and a DCC ZERO based
// on 116 microseconds half period, we define a tick as a 29 microsecond interval. Although, ONE and ZERO
// bit signals could be implemented using a multiple of 58 microseconds, the cutout function requires a
// signal length of 29 microseconds at the beginning of the period, right after the packet end bit of the
// previous packet. Luckily 2 * 29 is 58, 2 * 58 is 116. Perfect for DCC packets.
//
// ??? think directly in microseconds ?
//----------------------------------------------------------------------------------------
const uint16_t TIMER_RES_ID                 = 100; // arbitrarily chosen
const uint32_t TICKS_29_MICROS              =  1;
const uint32_t TICKS_58_MICROS              =  TICKS_29_MICROS * 2;
const uint32_t TICKS_116_MICROS             =  TICKS_29_MICROS * 4;
const uint32_t TICKS_CUTOUT_MICROS          =  TICKS_29_MICROS * 16;

//------------------------------------------------------------------------------------------
// Base Station global limits. Perhaps to move to a configurable place...
//
//-----------------------------------------------------------------------------------------
const uint16_t MILLI_VOLT_PER_DIGIT         = 5;
const uint16_t MILLI_VOLT_PER_AMP           = 1500;

//----------------------------------------------------------------------------------------
// DCC track power management is also a a state machine managing the state of the power track. Maximum values
// for the DCC track power start and stop sequence as well as limits for power overload events are defined. 
// We also define reasonable default values.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Track state machine state definitions. See the track state machine routine for an explanation of the 
// individual states.
//
//----------------------------------------------------------------------------------------
enum DccTrackState : uint8_t {

    DCC_TRACK_POWER_OFF       = 0,
    DCC_TRACK_POWER_ON        = 1,
    DCC_TRACK_POWER_OVERLOAD  = 2,
    DCC_TRACK_POWER_START1    = 3,
    DCC_TRACK_POWER_START2    = 4,
    DCC_TRACK_POWER_STOP1     = 5,
    DCC_TRACK_POWER_STOP2     = 6
};

//----------------------------------------------------------------------------------------
// DCC Track signal state machine states. See the DCC signal state machine routine for an explanation of
// the states.
//
//----------------------------------------------------------------------------------------
enum DccSignalState : uint8_t {

    DCC_SIG_CUTOUT_START      = 0,
    DCC_SIG_CUTOUT_1          = 1,
    DCC_SIG_CUTOUT_2          = 2,
    DCC_SIG_CUTOUT_3          = 3,
    DCC_SIG_CUTOUT_END        = 4,
    DCC_SIG_START_BIT         = 5,
    DCC_SIG_TEST_BIT          = 6,
    DCC_SIG_ZERO_SECOND_HALF  = 7
};

// ??? idea: each state has a number of ticks it will set. Have an array where to get this value and just
// set it from the table...
//
uint8_t ticksForState[ ] = {

    TICKS_29_MICROS,        // DCC_SIG_CUTOUT_START
    TICKS_CUTOUT_MICROS,    // DCC_SIG_CUTOUT_1
    TICKS_29_MICROS,        // DCC_SIG_CUTOUT_2
    TICKS_58_MICROS,        // DCC_SIG_CUTOUT_3
    TICKS_58_MICROS,        // DCC_SIG_CUTOUT_END
    TICKS_58_MICROS,        // DCC_SIG_START_BIT
    TICKS_58_MICROS,        // DCC_TEST_BIT,
    TICKS_116_MICROS        // DCC_SIG_ZERO_SECOND_HALF
};

//----------------------------------------------------------------------------------------
// DCC Track signal state machine follow up request items. The signal state machine first sets the hardware
// signal for both tracks and then determines whether a follow up action is required. See the track state
// machine routine for an explanation of the individual follow up actions.
//
//----------------------------------------------------------------------------------------
enum DccSignalStateFollowup : uint8_t {

    DCC_SIG_FOLLOW_UP_NONE                = 0,
    DCC_SIG_FOLLOW_UP_GET_BIT             = 1,
    DCC_SIG_FOLLOW_UP_GET_PACKET          = 2,
    DCC_SIG_FOLLOW_UP_MEASURE_CURRENT     = 3,
    DCC_SIG_FOLLOW_UP_START_RAILCOM_IO    = 4,
    DCC_SIG_FOLLOW_UP_STOP_RAILCOM_IO     = 5,
    DCC_SIG_FOLLOW_UP_RAILCOM_MSG         = 6,
};

//----------------------------------------------------------------------------------------
// The hardware timer needs to be set to the ticks we want to pass before interrupting again. There are
// three things to remember between interrupts. First, the current time interval, which tells us how many
// ticks will have passed when the timer interrupts again. Next, for each DCC track signal state we need to
// remember how many ticks are left before the state machine needs to run again. Each time the timer will
// interrupt, the passed ticks are subtracted from the ticks left counters. When the counter becomes zero,
// the state machine for the track will run.
//
//----------------------------------------------------------------------------------------
volatile uint8_t timeToInterrupt    = 0;
volatile uint8_t timeLeftMainTrack  = 0;
volatile uint8_t timeLeftProgTrack  = 0;

//----------------------------------------------------------------------------------------
// The DCC track object maintains an internal log facility for test and debugging purposes. During operation
// a set of log entries can be recorded to a log buffer. A log entry consist of the header byte, which 
// contains in the first byte the 4-bit log id and the 4-bit length of the log data. A log entry can therefore
// record up to 16 bytes of payload.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// The log buffer and the log index. When writing to the log buffer, the index will always point to the
// next available position. Once the buffer is full, no further data can be added.
//
//----------------------------------------------------------------------------------------
const uint16_t  LOG_BUF_SIZE            = 4096;

bool            logEnabled              = false;
bool            logActive               = false;
uint16_t        logBufIndex             = 0;
uint8_t         logBuf[ LOG_BUF_SIZE ]  = { 0 };

//----------------------------------------------------------------------------------------
// RailCom decoder table. The Railcom communication will send raw bytes where only four bits are "one" in
// a byte ( hamming weight 4 ). The first two bytes are labelled "channel1" and the remaining six bytes
// are labelled "channel2". The actual data is then encode using the table below. Each raw byte will be
// translated to a 6 bits of data for the datagram to assemble. In total there are therefore a maximum
// of 48bits that are transmitted in a railcom message.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Railcom datagrams are sent from a mobile or a stationary decoder.
//
//----------------------------------------------------------------------------------------
enum railComDatagramType : uint8_t {

    RX_DG_TYPE_UNDEFINED  = 0,
    RC_DG_TYPE_MOB        = 1,
    RC_DG_TYPE_STAT       = 2
};

//----------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Utility routine for number range checks.
//
//----------------------------------------------------------------------------------------
bool isInRangeU( uint8_t val, uint8_t lower, uint8_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

//----------------------------------------------------------------------------------------
// Utility function to map a DCC address to a railcom decoder type.
//
//----------------------------------------------------------------------------------------
inline uint8_t mapDccAdrToRailComDatagramType( uint16_t adr ) {

    if      (( adr >= 1 )   && ( adr <= 127 ))  return ( RC_DG_TYPE_MOB );
    else if (( adr >= 128 ) && ( adr <= 191 ))  return ( RC_DG_TYPE_STAT );
    else if (( adr >= 192 ) && ( adr <= 231 ))  return ( RC_DG_TYPE_MOB );
    else                                        return ( RX_DG_TYPE_UNDEFINED );
}

//----------------------------------------------------------------------------------------
// Conversion functions between milliAmps and digit values as report4de by the analog to digital converter
// hardware. For a better precision, the formula uses 32 bit computation and stores the result back in a 
// 16 bit quantity. 
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// The DccTrack timer interrupt handler routine implements the heartbeat of the DCC system. The two DCC
// track signal generators state machines MAIN and PROG use the same timer interrupt handler. Upon the timer
// interrupt, we first will update the time left counters. If a counter falls to zero, the signal state
// machine for that track will run and set the DCC signal levels. The state machine returns the next time
// interval it expects to be called again and a possible follow up action code. After handling both state
// machines, the timer is set to the smaller new remaining minimum time interval of both state machines.
// This is the time when the next state machine in one of the  signal generators needs to run. It is
// important to always have the timer running, so we keep decrementing the ticks to interrupt values.
//
// If a state machine determined that it needs to do some more elaborate action, the interrupt handler runs
// part two of its work. This split allows to run the time sensitive signal level settings first and any
// actions, such as getting the next packet, after both signal generator signal settings have been processed.
// Follow up actions are getting the next bit value to transmit, the next packet to send, a power consumption
// measurement and Railcom message processing. As we do not have all time in the world, these follow up
// actions still should be brief. The state machine carefully selects the spot for requesting such follow up
// actions in the DCC bit stream.
//
// The timer interrupt routine and all it calls runs with interrupts disabled. As said, better be quick.
// Top priority is to fetch the next bit and the next packet. Next is the Railcom processing if enabled. If
// there are power consumption measurement follow up actions, they are run last. Since the ADC converter
// hardware serializes the analog measurements, we will only do one measurement and drop the other. MAIN
// always has the higher priority.
//
// For the MAIN track with cutout enabled, the entry and exit of that cutout is a 29us timer call. That is
// awfully short and no follow-up action is scheduled there. All other intervals are either 58us or 116us
// or even longer for the cutout itself and give us some more room.
//
// ??? we could use timerVal, but this is in microseconds, not ticks. Convert one day...
//------------------------------------------------------------------------------------------
void timerCallback( uint32_t timerVal ) {

    uint8_t followUpMain = DCC_SIG_FOLLOW_UP_NONE;
    uint8_t followUpProg = DCC_SIG_FOLLOW_UP_NONE;

    timeLeftMainTrack -= timeToInterrupt;
    timeLeftProgTrack -= timeToInterrupt;

    if ( timeLeftMainTrack == 0 ) mainTrack -> runDccSignalStateMachine( &timeLeftMainTrack, &followUpMain );
    if ( timeLeftProgTrack == 0 ) progTrack -> runDccSignalStateMachine( &timeLeftProgTrack, &followUpProg );

    // take out after test ...
    // timeToInterrupt = min( timeLeftMainTrack, timeLeftProgTrack );

    timeToInterrupt = (( timeLeftMainTrack < timeLeftProgTrack ) ? timeLeftMainTrack : timeLeftProgTrack );

    CDC::setRepeatingTimerLimit( TIMER_RES_ID, timeToInterrupt * TICK_IN_MICROSECONDS );

    if (( followUpMain != DCC_SIG_FOLLOW_UP_NONE ) && ( followUpMain != DCC_SIG_FOLLOW_UP_MEASURE_CURRENT )) {

        if      ( followUpMain == DCC_SIG_FOLLOW_UP_GET_BIT )           mainTrack -> getNextBit( );
        else if ( followUpMain == DCC_SIG_FOLLOW_UP_GET_PACKET )        mainTrack -> getNextPacket( );
        else if ( followUpMain == DCC_SIG_FOLLOW_UP_START_RAILCOM_IO )  mainTrack -> startRailComIO( );
        else if ( followUpMain == DCC_SIG_FOLLOW_UP_STOP_RAILCOM_IO )   mainTrack -> stopRailComIO( );
        else if ( followUpMain == DCC_SIG_FOLLOW_UP_RAILCOM_MSG )       mainTrack -> handleRailComMsg( );
    }

    if (( followUpProg != DCC_SIG_FOLLOW_UP_NONE ) && ( followUpProg != DCC_SIG_FOLLOW_UP_MEASURE_CURRENT )) {

        if      ( followUpProg == DCC_SIG_FOLLOW_UP_GET_BIT )     progTrack -> getNextBit( );
        else if ( followUpProg == DCC_SIG_FOLLOW_UP_GET_PACKET )  progTrack -> getNextPacket( );
    }

    if      ( followUpMain == DCC_SIG_FOLLOW_UP_MEASURE_CURRENT ) mainTrack -> powerMeasurement( );
    else if ( followUpProg == DCC_SIG_FOLLOW_UP_MEASURE_CURRENT ) progTrack -> powerMeasurement( );

} // timerCallback

//----------------------------------------------------------------------------------------
// When all DCC track objects are initialized, the last thing to do before operation is to  start the timer
// heartbeat. We start b firing up the timer with a first short delay, so when it expires the timer routine
// will be called. The current time tick of zero and no ticks left, so the state machine for the signals 
// will run.
//
//----------------------------------------------------------------------------------------
void initDccTrackProcessing( ) {

    timeToInterrupt    = 0;
    timeLeftMainTrack  = 0;
    timeLeftProgTrack  = 0;
    
    uint8_t rStat = configureTimer( TIMER_RES_ID, CDC_INT_PRI_HIGH, timerCallback );
    startRepeatingTimer( TIMER_RES_ID, TICK_IN_MICROSECONDS );
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

//----------------------------------------------------------------------------------------
// There are a couple of routines to write the log data. For convenience, some of the log entry types are
// available as a direct call. The order of data entry for numeric types is big endian, i.e. most significant
// byte first.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// The log management routines. A typical transaction to log would start the logging process and then end
// it after the operation to analyze/debug. The "enableLog" call should be used to enable the logging
// process all together, the other calls will only do work when the log is enabled. With this call the
// recording process could be controlled from a command line setting or so.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// A simple routine to print out the log data, one entry on one line.
//
// ??? what is exactly the stop condition ? The END entry having a length of zero ?
//----------------------------------------------------------------------------------------
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
using namespace CDC;

//----------------------------------------------------------------------------------------
// "startDccProcessing" will kick off the DCC timer for the track signal processing. The idea is that the
// program first creates all the DCC track objects, does whatever else needs to be initialized and then starts
// the signal generation with this routine.
//
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::startDccProcessing( ) {

    initDccTrackProcessing( );
}

//----------------------------------------------------------------------------------------
// Object instance section. The DccTrack constructor. Nothing to do so far.
//
//----------------------------------------------------------------------------------------
LcsBaseStationDccTrack::LcsBaseStationDccTrack( ) { }

//----------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationDccTrack::setupDccTrack( LcsBaseStationTrackDesc* trackDesc ) {



    if ((  trackDesc -> rNumEnable  == 0 ) ||
        (  trackDesc -> rNumControl == 0 ) ||
        (  trackDesc -> rNumSense   == 0 )) {

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
    rNumEnable                = trackDesc -> rNumEnable;
    rNumControl               = trackDesc -> rNumControl;
    rNumSense                 = trackDesc -> rNumSense;
    rNumUartRx                = trackDesc -> rNumUartRx;
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

    configureDio( rNumEnable );
    configureDio( rNumControl );
    configureAdc( rNumSense );

    writeDio( rNumEnable, false );
    writeDio( rNumControl, false, false );

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

        uint8_t rStat = configureUart( rNumUartRx );
        if ( rStat != LCS_OK ) {

            flags = DT_F_CONFIG_ERROR;
            return ( ERR_DCC_TRACK_CONFIG );
        }
    }

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// DCC signal generation is done through a state machine that is invoked when the DCC timer interrupts. The
// interrupt timer thinks in multiples of 29us, which we will just call a "tick" in the description below. It
// runs as part of the timer interrupt handler, so we need to be short and quick. First, the HW signals are
// set. This keeps the track signals in their timing. Next, the new signal state, time to run again and any
// other follow up action of this invocation are set. The idea is to separate HW signal generation and follow
// up actions. The timer interrupt handler will first call both state machines, MAIN and PROG, and then work
// on the optional follow-up actions. The state machine has the following states:
//
// DCC_SIG_CUTOUT_START: if the cutout option is on, a new DCC packet starts with this signal state. The
// DCC signal goes HIGH for one tick and the signal state advances to signal state DCC_SIG_CUTOUT_1.
//
// DCC_SIG_CUTOUT_1: this stage sets the signal to CUTOUT for cutout period ticks. Also, if the RailCom
// is enabled, there is a follow up request to start the serial IO read function. The signal state advances
// to signal state DCC_SIG_CUTOUT_2.
//
// DCC_SIG_CUTOUT_2: this stage sets the signal to LOW for the cutout end tick. The signal state advances
// to signal state DCC_SIG_CUTOUT_3.
//
// DC_SIG_CUTOUT_3: the DC_SIG_CUTOUT_3 and DC_SIG_END_CUTOUT states represent the first DCC "One" after
// the cutout. The DCC signal is set to HIGH and the next period is two ticks. The follow-up request is to
// disable the UART receiver. The signal state advances to DC_SIG_CUTOUT_END.
//
// DC_SIG_CUTOUT_END: The DC_SIG_END_CUTOUT state is the second half of the DCC one. The signal is set
// to low and the next period to two ticks. If RailCom is enabled, this is the state where a follow up
// to handle the RailCom data takes place. The next state is then DCC_SIG_START_BIT to handle the next
// packet, starting with the preamble of DCC ones.
//
// DCC_SIG_START_BIT: this stage is the start of the DCC packet bits, which are preamble, the data bytes
// with separators and postamble. If the cutout option is off, this is also the start for the DCC packet.
// The signal is set HIGH, the tick count is two and we need a follow up to get the current bit, which
// determines the length of the signal for the bit we just started. The next stage is signal state
// DCC_SIG_TEST_BIT.
//
// DCC_SIG_TEST_BIT: coming from signal state DCC_SIG_START_BIT, we need to see if the current bit is a ONE
// or ZERO bit. If a ONE bit, the signal needs to become LOW, the next period is 2 ticks and the next state
// is signal state DCC_SIG_START_BIT. If it is the last ONE bit of the postamble, the next packet and
// signal state needs to be determined. For a CUTOUT enabled track this is state DCC_SIG_START_CUTOUT, else
// DCC_SIG_START_BIT. If a ZERO bit, the signal is kept HIGH for another two ticks and the state is
// DCC_SIG_ZERO_SECOND_HALF.
//
// The ZERO bit case is also a good place to do a current measurement. We are already two ticks into the
// signal polarity change and there should be no spike from the signal level transition. However, we do
// not want to measure all zero bits since this would mean several hundreds to few thousands per second.
// Each data byte starts with a DCC ZERO bit. We will just sample the current there and end up with a few
// hundred samples per second, which is less of a burden but still often enough for overload detection
// and so on.
//
// DCC_SIG_ZERO_SECOND_HALF: coming from signal state DCC_SIG_TEST_BIT, we need to transmit the second half
// of the ZERO bit. The signal is set to LOW for four ticks and set the next stage is signal state to
// DCC_SIG_START_BIT.
//
// Note: for a 16Mhz Atmega the implementation for the cutout support is a close call. If the timer value
// setting takes place after the internal timer counter HW has passed this value, you wrap around and the
// interrupt happens the next time the timer value matches, which is about 4 milliseconds later! If you see
// such a gap in the DCC signal, this is perhaps the issue. When using the railcom/cutout option it is
// recommended to set the processor frequency to 20Mhz, which you can do in your own design, but not on
// an Arduino board.
//
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::runDccSignalStateMachine(

    volatile uint8_t  *timeToInterrupt,
    uint8_t           *followUpAction

    ) {

    switch ( signalState ) {

        case DCC_SIG_CUTOUT_START: {

            writeDio( rNumControl, true, false );
            *timeToInterrupt  = TICKS_29_MICROS;
            *followUpAction   = DCC_SIG_FOLLOW_UP_NONE;
            signalState       = DCC_SIG_CUTOUT_1;

        } break;

        case DCC_SIG_CUTOUT_1: {

            writeDio( rNumControl, false, false );
            *timeToInterrupt  = TICKS_CUTOUT_MICROS;
            *followUpAction   = (( flags & DT_F_RAILCOM_MODE_ON ) ?
                                DCC_SIG_FOLLOW_UP_START_RAILCOM_IO : DCC_SIG_FOLLOW_UP_NONE );
            signalState       = DCC_SIG_CUTOUT_2;

        } break;

        case DCC_SIG_CUTOUT_2: {

            writeDio( rNumControl, false, true );
            *timeToInterrupt  = TICKS_29_MICROS;
            *followUpAction   = DCC_SIG_FOLLOW_UP_NONE;
            signalState       = DCC_SIG_CUTOUT_3;

        } break;

        case DCC_SIG_CUTOUT_3: {

            writeDio( rNumControl, true, false );
            *timeToInterrupt  = TICKS_58_MICROS;
            signalState       = DCC_SIG_CUTOUT_END;

            if ( flags & DT_F_RAILCOM_MODE_ON ) {

                flags           |= DT_F_RAILCOM_MSG_PENDING;
                *followUpAction = DCC_SIG_FOLLOW_UP_STOP_RAILCOM_IO;
            }
            else *followUpAction = DCC_SIG_FOLLOW_UP_NONE;

        } break;

        case DCC_SIG_CUTOUT_END: {

            writeDio( rNumControl, false, true );
            *timeToInterrupt  = TICKS_58_MICROS;
            *followUpAction   = (( flags & DT_F_RAILCOM_MODE_ON ) ?
                                DCC_SIG_FOLLOW_UP_RAILCOM_MSG : DCC_SIG_FOLLOW_UP_NONE );
            signalState       = DCC_SIG_START_BIT;

        } break;

        case DCC_SIG_START_BIT: {

            writeDio( rNumControl, true, false );
            *timeToInterrupt  = TICKS_58_MICROS;
            *followUpAction   = DCC_SIG_FOLLOW_UP_GET_BIT;
            signalState       = DCC_SIG_TEST_BIT;

        } break;

        case DCC_SIG_TEST_BIT: {

            if ( currentBit ) {

                writeDio( rNumControl, false, true );

                if ( postambleSent >= postambleLen ) {

                    *followUpAction = DCC_SIG_FOLLOW_UP_GET_PACKET;
                    signalState     = (( flags & DT_F_CUTOUT_MODE_ON ) ? DCC_SIG_CUTOUT_START : DCC_SIG_START_BIT );
                }
                else {

                    *followUpAction = DCC_SIG_FOLLOW_UP_NONE;
                    signalState     = DCC_SIG_START_BIT;
                }
            }
            else {

                *followUpAction   = (( bitsSent == 0 ) ? DCC_SIG_FOLLOW_UP_MEASURE_CURRENT :  DCC_SIG_FOLLOW_UP_NONE );
                signalState       = DCC_SIG_ZERO_SECOND_HALF;
            }

            *timeToInterrupt  = TICKS_58_MICROS;

        } break;

        case DCC_SIG_ZERO_SECOND_HALF: {

            writeDio( rNumControl, false, true );
            *timeToInterrupt  = TICKS_116_MICROS;
            *followUpAction   = DCC_SIG_FOLLOW_UP_NONE;
            signalState       = DCC_SIG_START_BIT;

        } break;

        default: {

            *followUpAction   = DCC_SIG_FOLLOW_UP_NONE;
            *timeToInterrupt  = TICKS_58_MICROS;
        }
    }
}

//----------------------------------------------------------------------------------------
// The "getNextBit" routine works through the active packet buffer bit for bit. A packet consists of the
// optional cutout sequence, the preamble bits, the data bytes separated by a ZERO bit and the postamble bits.
// The cutout option, the preamble and postamble are configured at DCC track object init time. The preamble
// length is different for MAIN and PROG tracks with the the cutout period overlaid at the beginning of the
// preamble. The postamble is currently always just one HIGH bit, according to standard.
//
// The routine works first through the preamble bit count, then through the data byte bits, and finally
// through the postamble bits. The bits to select from the data byte is done with a 9-bit mask. Remember that
// the first bit to send is the data byte separator, which is always a zero. We run from 0 to 8 through the
// bit mask, the first bit being the ZERO bit.
//
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::getNextBit( ) {

    if ( preambleSent < preambleLen ) {

        currentBit = true;
        preambleSent ++;
    }
    else if ( bytesSent < activeBufPtr -> len ) {

        currentBit = activeBufPtr -> buf[ bytesSent ] & bitMask9[ bitsSent ];
        bitsSent ++;

        if ( bitsSent == 9 ) {

            bytesSent ++;
            bitsSent = 0;
        }
    }
    else if ( postambleSent < postambleLen ) {

        currentBit = true;
        postambleSent ++;
    }
}

//----------------------------------------------------------------------------------------
// If all bits of a packet have been processed, the next packet will be determined during the last ONE bit
// transmission of the postamble. If there is a non-zero repeat count on the current packet, the same packet
// is sent again until the repeat count drops to zero. On a zero repeat count, we check if there is a pending
// packet. If so, it is copied to the active buffer and the pending flag is reset. This signals anyone waiting,
// that the next packet can be queued. If there is no pending packet, we still need to keep the track going and
// will load an IDLE or RESET packet.
//
// For non-service mode packets, there is a requirement that a decoder should not be receive two consecutive
// packets. The standards talks about 5 milliseconds between two packets to the same decoder. For now, we will
// not do anything special. A decoder will most likely, if there is more than one decoder active, not be
// addressed in two consecutive packets, simply because the session refresh mechanism will go round robin
// through the session list. However, if there is only one decoder active, two packets will be sent in a
// row, but the decoders are robust enough to ignore this fact. Better run more than one loco :-).
//
// This routine is the central place to submit a DCC packet to the track and therefore a good place to write
// a DCC_LOG record. We distinguish between a RESET, an IDLE and a data packet. Note that these records will
// only be written when DCC logging is enabled.
//
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::getNextPacket( ) {

    bytesSent     = 0;
    bitsSent      = 0;
    preambleSent  = 0;
    postambleSent = 0;

    if ( activeBufPtr -> repeat > 0 ) {

        activeBufPtr -> repeat --;

        writeLogData( LOG_DCC_PKT, activeBufPtr -> buf, activeBufPtr -> len );
    }
    else if ( flags & DT_F_DCC_PACKET_PENDING ) {

        activeBufPtr  = pendingBufPtr;
        pendingBufPtr = (( pendingBufPtr == &dccBuf1 ) ? &dccBuf2 : &dccBuf1 );
        flags &= ~ DT_F_DCC_PACKET_PENDING;

        writeLogData( LOG_DCC_PKT, activeBufPtr -> buf, activeBufPtr -> len );
    }
    else {

        if ( flags & DT_F_SERVICE_MODE_ON ) {

            activeBufPtr = &resetDccPacket;
            writeLogId( LOG_DCC_RST );
        }
        else {

            activeBufPtr = &idleDccPacket;
            writeLogId( LOG_DCC_IDL );
        }
    }

    dccPacketsSend ++;
}

//----------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::startRailComIO( ) {

    startUartRead( rNumUartRx );
}

void LcsBaseStationDccTrack::stopRailComIO( ) {

    stopUartRead( rNumUartRx );
}

uint8_t LcsBaseStationDccTrack::handleRailComMsg( ) {

    railComBufIndex = getUartBuffer( rNumUartRx, railComMsgBuf, sizeof( railComMsgBuf ));

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
    return ( LCS_OK );
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

//----------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::runDccTrackStateMachine( ) {

    switch ( trackState ) {

        case DCC_TRACK_POWER_START1: {

            // ??? do we need a way to check for overload during this initial phase, just like we do when ON ?

            trackTimeStamp          = CDC::getMillis( );
            flags                   |= DT_F_POWER_ON;
            flags                   &= ~DT_F_POWER_OVERLOAD;
            flags                   &= ~DT_F_MEASUREMENT_ON;
            limitCurrentDigitValue  = milliAmpToDigitValue( initCurrentMilliAmp, digitsPerAmp );

            writeDio( rNumEnable, true );
            trackState = DCC_TRACK_POWER_START2;

        }  break;

        case DCC_TRACK_POWER_START2: {

            if (( getMillis( ) - trackTimeStamp ) > startTimeThreshold ) {

                highWaterMarkDigitValue = 0;
                actualCurrentDigitValue = 0;
                overloadRestartCount    = 0;
                overloadEventCount      = 0;
                flags                   |= DT_F_POWER_ON | DT_F_MEASUREMENT_ON;
                limitCurrentDigitValue  = milliAmpToDigitValue( limitCurrentMilliAmp, digitsPerAmp );

                writeDio( rNumEnable, true );
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

                    writeDio( rNumEnable, false );
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

            writeDio( rNumEnable, false );
            trackState = DCC_TRACK_POWER_STOP2;

        }  break;

        case DCC_TRACK_POWER_STOP2: {

            if ( getMillis( ) - trackTimeStamp > stopTimeThreshold ) trackState = DCC_TRACK_POWER_OFF;

        } break;

        case DCC_TRACK_POWER_OFF: {

        } break;
    }
}

//----------------------------------------------------------------------------------------
// Some getter functions. Straightforward.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBaseStationDccTrack::getFlags( ) {

    return ( flags );
}

uint16_t LcsBaseStationDccTrack::getOptions( ) {

    return ( options );
}

uint32_t LcsBaseStationDccTrack::getDccPacketsSend( ) {

    return ( dccPacketsSend );
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

    return ( flags & DT_F_CUTOUT_MODE_ON );
}

bool LcsBaseStationDccTrack::isRailComOn( ) {

    return ( flags & DT_F_RAILCOM_MODE_ON );
}

//----------------------------------------------------------------------------------------
// DCC track power management functions. The actual state of track power is kept in the track status field
// and can be queried or set by setting the respective flag. Starting and stopping track power is done by
// setting the respective START or STOP state.
//
//----------------------------------------------------------------------------------------
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

void LcsBaseStationDccTrack::cutoutOn( ) {

    if ( ! ( options & DT_OPT_SERVICE_MODE_TRACK )) {

        preambleLen =  MAIN_PACKET_PREAMBLE_LEN - DCC_PACKET_CUTOUT_LEN;
        flags       |= DT_F_CUTOUT_MODE_ON;
    }
}

void LcsBaseStationDccTrack::cutoutOff( ) {

    if ( ! ( options & DT_OPT_SERVICE_MODE_TRACK )) {

        preambleLen =  MAIN_PACKET_PREAMBLE_LEN;
        flags       &= ~DT_F_CUTOUT_MODE_ON;
        flags       &= ~DT_F_RAILCOM_MODE_ON;
    }
}

void LcsBaseStationDccTrack::railComOn( ) {

    if ( ! ( options & DT_OPT_SERVICE_MODE_TRACK )) {

        flags |= DT_F_CUTOUT_MODE_ON | DT_F_RAILCOM_MODE_ON;
    }
}

void LcsBaseStationDccTrack::railComOff( ) {

    if ( ! ( options & DT_OPT_SERVICE_MODE_TRACK )) flags &= ~DT_F_RAILCOM_MODE_ON;
}

//----------------------------------------------------------------------------------------
// Power Consumption Management. There are two key values. The first is the actual current consumption as
// measured by the ADC hardware on each ZERO DCC bit. This value is used to do the power overload checking.
// The second value is the high water mark built from these measurements. This values is used for the DCC
// decoder programming logic. The high water mark will be set to zero before collecting measurements. All
// measurement values are actually ADC digit values for performance reason. Only on limit setting and external
// data access are these values converted from and to milliAmps.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// The "getRMSCurrent" function returns the power consumption based on the samples taken and stored in the
// sample buffer. The function computes the square root of the sum of the squares of the array elements. The
// result is returned in milliAmps. Note that our measurement is based on unsigned 16-bit quantities that come
// from the controller ADC converter. We compute the RMS based on 16-bit unsigned integers, which compared
// to floating point computation is not really precise. However, for our purpose to just show a rough power
// consumption, the error should be not a big issue. We will not use RMS values for power overload detection
// or decoder ACK detection.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBaseStationDccTrack::getRMSCurrent( ) {

    uint32_t res = 0;

    for ( uint8_t i = 0; i < PWR_SAMPLE_BUF_SIZE; i++ ) res += pwrSampleBuf[ i ] * pwrSampleBuf[ i ];

    return ( digitValueToMilliAmp( sqrt( res / PWR_SAMPLE_BUF_SIZE ), digitsPerAmp ));
}

//----------------------------------------------------------------------------------------
// This function is called whenever a power measurement operation completes from the analog conversion
// interrupt handler. This typically takes place on the first half of the DCC "0" bit. If power measurement
// is enabled, we increment the number of samples taken, check the measured value for an overload situation
// and also set the high water mark accordingly. Since we are part of an interrupt handler, keep the amount
// work really short.
//
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::powerMeasurement( ) {

    if ( flags & DT_F_MEASUREMENT_ON ) {

        uint16_t adcVal;

        uint8_t rStat = readAdc( rNumSense, &adcVal );

        actualCurrentDigitValue = adcVal;
        totalPwrSamplesTaken ++;

        if ( actualCurrentDigitValue > highWaterMarkDigitValue ) highWaterMarkDigitValue = actualCurrentDigitValue;
        if ( actualCurrentDigitValue > limitCurrentDigitValue ) flags |= DT_F_POWER_OVERLOAD;
    }
}

//----------------------------------------------------------------------------------------
// The DCC decoder programming requires the detection of a current consumption change. This is the way a DCC
// decoder signals an acknowledgement. To detect the consumption change we need first an idea what the actual
// average current baseline consumption of the decoder is. This method will send the required DCC reset packets
// according to the DCC standard and at the same time determine the current consumption as a baseline. We use
// the high water mark for this purpose.
//
// ??? although the routines for decoder ACK detection work, they will produce quite a number of packets.
// During this time, other LCS work is blocked. Perhaps we need a kind of state machine approach to cut the
// long sequence in smaller chunks to allow other work in between.
//----------------------------------------------------------------------------------------
uint16_t LcsBaseStationDccTrack::decoderAckBaseline( uint8_t resetPacketsToSend ) {

    if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {

        printf( "\nDecoder Ack setup: ( " );
    }

    uint16_t sum = 0;

    for ( uint8_t i = 0; i < resetPacketsToSend; i++ ) {

        highWaterMarkDigitValue = 0;

        loadPacket( resetDccPacketData, 2, 0 );

        if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {
       
            printf( "%d ", highWaterMarkDigitValue );
        }

        sum += highWaterMarkDigitValue;
    }

    if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {

        printf( ") -> %d\n", ( sum + resetPacketsToSend - 1 ) / resetPacketsToSend );
    }

    return (( sum + resetPacketsToSend - 1 ) / resetPacketsToSend );
}

//----------------------------------------------------------------------------------------
// "decoderAckDetect" is the counterpart to the decoder ack setup routine. The setup method established a base
// line for the power consumption and put the decoder in CV programming mode by sending the RESET packets. The
// decoder ACK detect routine now sends out resets packets to follow the programming packets required and
// monitors the current consumption. We use the high water mark for this purpose. The DCC standard specifies
// a time window in which the decoder should raise its power consumption level and signal an acknowledge this
// way. We will send out a series of reset packets and monitor after each packet the consumption level. The
// number of retries depends on whether it is a read ( 50ms window ) or a write ( 100ms window). If we detect
// a raised value the decoder did signal a positive outcome. If not, we time out after the last reset packet.
// The programming operation either failed or the decoder did on purpose not answer. We cannot tell.
//
// ??? although the routines for decoder ACK detection work, they will produce quite a number of packets.
// During this time, other LCS work is blocked. Perhaps we need a kind of state machine approach to cut the
// long sequence in smaller chunks to allow other work in between.
//----------------------------------------------------------------------------------------
bool LcsBaseStationDccTrack::decoderAckDetect( uint16_t baseDigitValue, uint8_t retries ) {

    if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {

        printf( "Decoder Ack detect: ( %d : %d : ( ", baseDigitValue, ackThresholdDigitValue );
    }

    for ( uint8_t i = 0; i < retries; i++ ) {

        highWaterMarkDigitValue = 0;

        loadPacket( resetDccPacketData, 2, 0 );

        if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {

            printf( "%d ", highWaterMarkDigitValue );
        }

        if (( highWaterMarkDigitValue >= baseDigitValue ) &&
            ( highWaterMarkDigitValue - baseDigitValue >= ackThresholdDigitValue )) {

            if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {

                printf( "[ %d ] ) -> OK\n", abs( highWaterMarkDigitValue - baseDigitValue ));
            }

        return ( true );
        }
    }

    if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_DCC_ACK_DETECT )) {

        printf( ") -> FAILED" );
    }

    return ( false );
}

//----------------------------------------------------------------------------------------
// LoadPacket is the central entry point to submit a DCC packet. The incoming packet is the the data to be
// sent without checksum, i.e. it is just the payload. The DCC track signal generator has two packet buffers.
// The first buffer holds the packet currently being transmitted. The second is the pending buffer. If it is
// used, we will simply busy wait for our turn to load the packet into the pending buffer. Upon completion of
// sending the active packet, the interrupt handler copies the currently pending buffer to the active buffer
// and then resets the pending flag. Either way, then it is our turn. We fill the pending buffer, compute the
// checksum and set the pending flag.
//
// ??? For a high number of session we may want to think about a queuing approach. Right now, this routine
// waits when there is a packet already queued, i.e. pending. This may cause issues in delaying other tasks
// such as receiving a CAN bus message.
//----------------------------------------------------------------------------------------
void LcsBaseStationDccTrack::loadPacket( const uint8_t *packet, uint8_t len, uint8_t repeat ) {

    if ( ! isInRangeU( len, MIN_DCC_PACKET_SIZE, MAX_DCC_PACKET_SIZE )) return;
    if ( ! isInRangeU( repeat, MIN_DCC_PACKET_REPEATS, MAX_DCC_PACKET_REPEATS )) return;

    while ( flags & DT_F_DCC_PACKET_PENDING );

    pendingBufPtr -> len    = len + 1;
    pendingBufPtr -> repeat = repeat;

    uint8_t checkSum = 0;
    uint8_t *bufPtr  = pendingBufPtr -> buf;

    for ( uint8_t i = 0; i < len; i++ ) {

        bufPtr[ i ] =   packet[ i ];
        checkSum    ^=  bufPtr[ i ];
    }

    bufPtr[ len ] = checkSum;
    flags         |= DT_F_DCC_PACKET_PENDING;
}

//----------------------------------------------------------------------------------------
// The log management routines. A typical transaction to log would start the logging process and then end
// it after the operation to analyze/debug. The "enableLog" call should be used to enable the logging
// process all together, the other calls will only do work when the log is enabled. With this call the
// recording process could be controlled from a command line setting or so. "beginLog" and "endLog" start 
// and end a recording sequence.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// There are a couple of routines to write the log data when the logging is active. For convenience, some of
// the log entry types are available as a direct call. The order of data entry for numeric types is big endian,
// i.e. most significant byte first.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Print out the log data, one entry on one line. We only print the log buffer when there is no log sequence
// active.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Print out the DCC Track configuration data. For debugging purposes.
//
//----------------------------------------------------------------------------------------
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

    printf( " CDC enable rNum: %d, DCC control rNum: %d, Sensor nRum: %d, RailCom rNum: %d\n",
            rNumEnable, rNumControl, rNumSense, rNumUartRx );

    printf( " PreambleLen: %d, PostambleLen: %d\n", preambleLen, postambleLen );
}

//----------------------------------------------------------------------------------------
// Print out the DCC Track status.
//
//----------------------------------------------------------------------------------------
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
