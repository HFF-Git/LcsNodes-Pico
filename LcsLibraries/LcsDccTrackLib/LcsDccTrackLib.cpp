///---------------------------------------------------------------------------------------
//
// LCS - DCC Track Manager
//
///---------------------------------------------------------------------------------------
// The DCC track object is one of the the key objects for the DCC subsystem. It 
// is responsible for the DCC track signal generation and the power management 
// functions.
//
// There will be exactly two objects of this kind, one for the MAIN track and the
// other for the PROG track. The DCC track object has two major functional parts. 
// The first is to transmit a DCC packet to the track. The second task is to 
// continuously monitor the current consumption and potential short circuits.
//
// Both channels are Decoder Ack Detect, Cutout and RailCom capable. Note that
// the two features are mutually exclusive. When a channel is in decoder ACK
// detect mode, RailCom and cutout is off. When RailCom is enabled, Cutout is
// also set and decoder ACk off.
//
///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect extension boards
// Copyright (C) 2020 - 2026  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with 
// this program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsDccTrackLib.h"
#include <math.h>

using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// 
// 
//----------------------------------------------------------------------------------------
inline bool dccTrackDebugEnabled(  ) {

    return ( true ); // ??? for now ... 
}

inline bool dccTrackDebugAccDetect( ) {

    return(( dccTrackDebugEnabled ) &&  false ); // ???? for now ...
}

inline bool dccTrackDebugRaiCom( ) {

    return(( dccTrackDebugEnabled ) &&  false ); // ???? for now ...
}

inline bool dccTrackDebugPowerMgt( ) {

    return(( dccTrackDebugEnabled ) &&  false ); // ???? for now ...
}

//----------------------------------------------------------------------------------------
// DCC Signal debugging. A tick is defined to last 29 microseconds. There is a 
// debugging option to set the clock much slower so that the waveform can be seen.
//
//----------------------------------------------------------------------------------------
#define DEBUG_WAVE_FORM 0               

#if DEBUG_WAVE_FORM == 1
#define TICK_IN_MICROSECONDS  400000
#else
#define TICK_IN_MICROSECONDS  29
#endif

//----------------------------------------------------------------------------------------
// The DccTrack Object local definitions. The DCC track object is a bit special. 
// There are exactly two object instances created, MAIN and PROG. Both however 
// share the global mechanism for generating the DCC hardware signals. There are
// callback functions for the DCC timer and the serial I/O capability for the 
// RailCom feature. The hardware lower layers can be found in the CDC library.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// The DCC Base Station will allocate two DCC Track Objects and also a timer for
// DCC signal generation. For the interrupt system to work, references to the 
// objects must be static variables. The initialization sequence outside of this
// class will allocate the two objects. We also keep a copy of these two object
// handles. The track resource descriptor options field has a bit for each type
// of track, so we know which global variable to set. Likewise, the descriptor
// contains the resource Id of the timer, last descriptor wins.
//
//----------------------------------------------------------------------------------------
uint8_t      rNumTimer  = 0;
LcsDccTrack  *trackA    = nullptr;
LcsDccTrack  *trackB    = nullptr;

//----------------------------------------------------------------------------------------
// DCC packet definitions. A DCC packet payload is at most 15 bytes long, 
// excluding the checksum byte. This is true for XPOM and DCC-A support, else it
// is according to NMRA up to 6 bytes. The preamble is a series of "ONE" bits, 
// which helps the decoders to sync to the bit stream. The standard specifies a 
// minimum of 16 ONE bits for the MAIN track and 22 ONE bits for the PROG track.
// The postamble is exactly one "ONE" bit. If the cutout period option is enabled, 
// the cutout is exactly one "ONE" bit. If the cutout period option is enabled, 
// the cutout overlays the first ONE bits the preamble. 
//
//----------------------------------------------------------------------------------------
const uint8_t   MAIN_PACKET_PREAMBLE_BIT_LEN    = 17;
const uint8_t   MAIN_PACKET_POSTAMBLE_BIT_LEN   = 1;
const uint8_t   PROG_PACKET_PREAMBLE_BIT_LEN    = 22;
const uint8_t   PROG_PACKET_POSTAMBLE_BIT_LEN   = 1;
const uint8_t   DCC_PACKET_CUTOUT_BIT_LEN       = 4;
const uint8_t   MIN_DCC_PACKET_SIZE             = 2;
const uint8_t   MAX_DCC_PACKET_SIZE             = 16;
const uint8_t   MIN_DCC_PACKET_REPEATS          = 0;
const uint8_t   MAX_DCC_PACKET_REPEATS          = 8;
const uint8_t   RAILCOM_BUFFER_SIZE             = 8;

//----------------------------------------------------------------------------------------
// Constant values definition. We need the RESET and IDLE packet as well as a bit
// mask for a quick bit select in the data byte.
//
//----------------------------------------------------------------------------------------
const uint8_t   idleDccPacketData[ ]    = { 0xFF, 0x00 };
const uint8_t   resetDccPacketData[ ]   = { 0x00, 0x00 };
const uint8_t   eStopDccPacketData[ ]   = { 0x00, 0x01 };

DccPacket       idleDccPacket           = { 3, 0, { 0xFF, 0x00, 0xFF }};
DccPacket       resetDccPacket          = { 3, 0, { 0x00, 0x00, 0x00 }};
const uint8_t   bitMask9[ ]             = { 0x00, 0x80, 0x40, 0x20, 0x10, 
                                            0x08, 0x04, 0x02, 0x01 };

//----------------------------------------------------------------------------------------
// Programming decoders require to detect a short rise in power consumption. The
// value is at least 60mA, but decoders can raise anything from 100mA to 250mA. 
// This setting is a bit touchy and the value set to 100mA was done after testing
// several decoders. If the value is too low, there will be false positives for
// some decoders. 
//
//----------------------------------------------------------------------------------------
const uint8_t ACK_TRESHOLD_VAL          = 100;

//----------------------------------------------------------------------------------------
// The DCC signal generator thinks in ticks. With a DCC ONE based on 58 micro
// seconds and a DCC ZERO based on 116 microseconds half period, we define a tick
// as a 29 microsecond interval. Although, ONE and ZERO bit signals could be 
// implemented using a multiple of 58 microseconds, the cutout function requires
// a signal length of 29 microseconds at the beginning of the period, right after
// the packet end bit of the previous packet. Luckily 2 * 29 is 58, 2 * 58 is 116. 
// Perfect for DCC packets.
//
//----------------------------------------------------------------------------------------
const uint32_t TICKS_29_MICROS         =  1;
const uint32_t TICKS_58_MICROS         =  TICKS_29_MICROS * 2;
const uint32_t TICKS_116_MICROS        =  TICKS_29_MICROS * 4;
const uint32_t TICKS_CUTOUT_MICROS     =  TICKS_29_MICROS * 16;

//----------------------------------------------------------------------------------------
// Current measurement is done via measuring the voltage over a shunt resistor.
// Externally we think in milliAmps, internally we think in ADC digits. The 
// global variables contain the offset and conversion factor used when converting.
// Current measurement conversion
//
//----------------------------------------------------------------------------------------
static uint32_t adcDigitsPerMilliAmpTimes1000;
static uint16_t currentZeroOffset;

//----------------------------------------------------------------------------------------
// DCC track power management is responsible for detecting among other things
// short circuits. However, we do not want to shout down because of one single
// short circuit situation. Only when the situation occurs repeatedly we shut
// down.
//
//----------------------------------------------------------------------------------------
const uint16_t MAX_OVERLOAD_EVENT_COUNT = 10;

//----------------------------------------------------------------------------------------
// DCC Track signal state machine states. See the DCC signal state machine 
// routine for an explanation of the states.
//
//----------------------------------------------------------------------------------------
enum DccSignalState : uint8_t {

    DCC_SIG_CUTOUT_START        = 1,
    DCC_SIG_CUTOUT_1            = 2,
    DCC_SIG_CUTOUT_2            = 3,
    DCC_SIG_CUTOUT_3            = 4,
    DCC_SIG_CUTOUT_END          = 5,
    DCC_SIG_START_BIT           = 6,
    DCC_SIG_TEST_BIT            = 7,
    DCC_SIG_ZERO_SECOND_HALF    = 8
};

//----------------------------------------------------------------------------------------
// DCC Track signal state machine follow up request items. The signal state 
// machine works in two phases. First the H-Bridge signals are set so we do not
// endanger accurate DCC timing. Next, any follow up action required by the 
// track, such as power measurement or bit data fetch, is performed. See the 
// track state machine routine for an explanation of the individual follow up
// actions.
//
//----------------------------------------------------------------------------------------
enum DccSignalStateFollowup : uint8_t {

    DCC_SIG_FOLLOW_UP_NONE                = 0,
    DCC_SIG_FOLLOW_UP_GET_BIT             = 1,
    DCC_SIG_FOLLOW_UP_GET_PACKET          = 2,
    DCC_SIG_FOLLOW_UP_MEASURE_CURRENT     = 3,
    DCC_SIG_FOLLOW_UP_START_RAILCOM_IO    = 4,
    DCC_SIG_FOLLOW_UP_STOP_RAILCOM_IO     = 5,
    DCC_SIG_FOLLOW_UP_RAILCOM_MSG         = 6
};

//----------------------------------------------------------------------------------------
// The hardware timer needs to be set to the ticks we want to pass before 
// interrupting again. There are three things to remember between interrupts. 
// First, the current time interval, which tells us how many ticks will have 
// passed when the timer interrupts again. Next, for each DCC track signal state
// we need to remember how many ticks are left before the state machine needs to 
// run again. Each time the timer will interrupt, the passed ticks are subtracted 
// from the ticks left counters. When the counter becomes zero, the state machine 
// for the track will run.
//
//----------------------------------------------------------------------------------------
volatile uint8_t timeToInterrupt    = 0;
volatile uint8_t timeLeftMainTrack  = 0;
volatile uint8_t timeLeftProgTrack  = 0;

//----------------------------------------------------------------------------------------
// The DCC track object maintains an internal log facility for test and debugging 
// purposes. During operation a set of log entries can be recorded to a log buffer. 
// A log entry consist of the header byte, which contains in the first byte the 
// 4-bit log id and the 4-bit length of the log data. A log entry can therefore 
// record up to 16 bytes of payload. When writing to the log buffer, the index 
// will always point to the next available position. Once the buffer is full, 
// no further data can be added.
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
// RailCom decoder table. The Railcom communication will send raw bytes where only
// four bits are "one" in a byte ( hamming weight 4 ). The first two bytes are 
// labelled "channel1" and the remaining six bytes are labelled "channel2". The
// actual data is then encode using the table below. Each raw byte will be translated
// to a 6 bits of data for the datagram to assemble. In total there are therefore
// a maximum of 48bits that are transmitted in a railcom message.
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
// Each mobile decoder railcom datagram will start with an ID field of four bits.
// Channel one will use only the ADR_HIG and ADR_LOW Ids. All IDs can be used for
// channel 2. Since decoders answer on channel one for each DCC packet they 
// receive, here is a good chance that channel 1 will contains nonsense data. 
// This is different for channel two, where only the addressed decoder explicitly 
// answers. To decide whether a railcom message is valid, you should perhaps ignore
// channel 1 data and just check channel 2 for this purpose. A RC datagram starts 
// with the 4-bit ID and an 8 to 32bit payload.     
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
// A datagram with the ID 14 is a DDC-A datagram and all 8 datagram bytes are 
// combined to an 48bit datagram. A datagram packet can also contain more than 
// one datagram. For example there could be two 18-bit length datagram in one 
// packet or 3 12-bit packets and so on. Finally, unused bytes in channel two 
// could contain an ACK to fill them up.
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
// Similar to the mobile decode, a stationary decoder datagram will start an ID 
// field of four bits. Stationary decoders also define a datagram with "SRQ" and 
// no ID field to request service from the base station.
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
// Setup the current measurement conversion. While externally we think in 
// milliAmps, internally we just thing in digit values. For convenience, the
// setup tales as input all the relevant hardware data and computes the 
// conversion factors. Our model is a ADC pin at the controller, an opAmp for
// amplification and a shunt resistor where the voltage is measured.
//
//  referenceMilliVolt      - ADC reference voltage in mV
//  shuntMilliOhm           - Shunt resistance in milliOhms
//  amplifierGain           - Amplifier gain
//  adcBits                 - ADC resolution
//  zeroOffset              - ADC value at zero current
//
// We keep for our conversion routines "currentCountsPerMilliAmpTimes1000", which
// contains ADC counts per mA, multiplied by 1000 for fixed-point precision.
//
// Formulas:
//
//  ADC counts per mA:
//
//     Vshunt = I[mA] * R[mOhm] / 1000
//     Vadc   = Vshunt * gain
//
//     ADC = Vadc * adcMax / Vref
//
//  Therefore:
//    
//               R[mOhm] * gain * adcMax
//      ADC/mA = -------------------------
//               1000 * Vref[mV]
//
//  Store ADC/mA multiplied by 1000 in "currentCountsPerMilliAmpTimes1000".
//
//----------------------------------------------------------------------------------------
void currentSenseSetup( uint16_t referenceMilliVolt,
                        uint16_t shuntMilliOhm,
                        uint16_t amplifierGain,
                        uint16_t adcBits,
                        uint16_t zeroOffset ) {

    uint32_t adcMax     = (1UL << adcBits) - 1;
    uint64_t numerator  = (uint64_t)shuntMilliOhm *
                          (uint64_t)amplifierGain *
                          (uint64_t)adcMax *
                          1000ULL;

    adcDigitsPerMilliAmpTimes1000 =
        (uint32_t) ((numerator + ((uint64_t)referenceMilliVolt * 500ULL )) /
                    ((uint64_t) referenceMilliVolt * 1000ULL ));

    currentZeroOffset = zeroOffset;
}

//----------------------------------------------------------------------------------------
// Current conversion routines.
//
//----------------------------------------------------------------------------------------
uint16_t milliAmpToAdcDigits( uint16_t milliAmp ) {
    
    uint64_t value = ((uint64_t) milliAmp *
                      (uint64_t) adcDigitsPerMilliAmpTimes1000 +
                      500ULL ) / 1000ULL;

    return ((uint16_t) value + currentZeroOffset );
}

uint16_t adcDigitsToMilliAmp( uint16_t adcValue ) {

    if ( adcValue <= currentZeroOffset ) return ( 0 );

    uint32_t value = adcValue - currentZeroOffset;

    uint64_t result = ((uint64_t) value * 1000ULL +
                       ((uint64_t) adcDigitsPerMilliAmpTimes1000 / 2ULL )) /
                       (uint64_t) adcDigitsPerMilliAmpTimes1000;

    return (uint16_t)result;
}

//----------------------------------------------------------------------------------------
// "followUpDccTrackWork" is the follow up handler when the signal state 
// machine signaled that there is more work for this track other than just 
// setting the control signals. 
// 
// This split allows to run the time sensitive signal level settings first and
// any actions, such as getting the next packet, after both signal generator 
// signal settings have been processed. 
//
// The timer interrupt routine and all it calls runs with interrupts disabled. 
// As said, better be quick. Top priority is to fetch the next bit and the next 
// packet. 
//
//----------------------------------------------------------------------------------------
inline void followUpDccTrackWork( uint8_t followUp,
                                  LcsDccTrack *track ) {

    if ( followUp != DCC_SIG_FOLLOW_UP_NONE ) {
        
    } 
    else  if ( followUp == DCC_SIG_FOLLOW_UP_GET_BIT ) {       

        track -> getNextBit( );
    }
    else if ( followUp == DCC_SIG_FOLLOW_UP_GET_PACKET ) {    

        track -> getNextPacket( );
    }
    else if ( followUp == DCC_SIG_FOLLOW_UP_START_RAILCOM_IO ) { 
            
        track -> startRailComIO( );
    }
    else if ( followUp == DCC_SIG_FOLLOW_UP_STOP_RAILCOM_IO ) {
              
        track -> stopRailComIO( );
    }
    else if ( followUp == DCC_SIG_FOLLOW_UP_RAILCOM_MSG ) {

        track -> handleRailComMsg( );    
    }
    else if ( followUp == DCC_SIG_FOLLOW_UP_MEASURE_CURRENT ) {

        track -> powerManagement( );
    }
}

//----------------------------------------------------------------------------------------
// The DccTrack timer interrupt handler routine implements the heartbeat of the 
// DCC system. The two DCC track signal generators state machines MAIN and PROG 
// use the same timer interrupt handler. Upon the timer interrupt, we first will 
// update the time left counters. If a counter falls to zero, the signal state 
// machine for that track will run and set the DCC signal levels. The state 
// machine returns the next time interval it expects to be called again and a 
// possible follow up action code. 
//
// After handling both state machines, the timer is set to the smaller new 
// remaining minimum time interval of both state machines. This is the time when 
// the next state machine in one of the  signal generators needs to run. It is 
// important to always have the timer running, so we keep decrementing the ticks
// to interrupt values.
//
// If a state machine determined that it needs to do some more elaborate action, 
// the interrupt handler runs part two of its work. See "followUpDccTrackWork"
// what needs to be done.
//
// 
// ??? we need a way to ensure that the signal interrupts for both tacks stay
// in sync. The cutout introduces a 29 us tick but gets us back to a 58us
// baseline after the cutout. But what if a track mode is set with exactly 
// 1 tick, i.e. 29us, off ? Then we would have a lot of interrupts at 29us
// time marks. Is just more load .....
//
// Idea: introduce a "SYNC" state. If one track is not at a 58us boundary while
// switching modes, a delay of 1 tick is inserted. 
// 
// ??? how to know we are at that 58us boundary ?
//
//----------------------------------------------------------------------------------------
void timerCallback( uint32_t timerVal ) {

    uint8_t followUpMain = DCC_SIG_FOLLOW_UP_NONE;
    uint8_t followUpProg = DCC_SIG_FOLLOW_UP_NONE;

    timeLeftMainTrack -= timeToInterrupt;
    timeLeftProgTrack -= timeToInterrupt;

    if ( timeLeftMainTrack == 0 ) {

        trackA -> runDccTrackStateMachine( &timeLeftMainTrack, &followUpMain );
    }

    if ( timeLeftProgTrack == 0 ) {

        trackB -> runDccTrackStateMachine( &timeLeftProgTrack, &followUpProg );
    }

    timeToInterrupt = (( timeLeftMainTrack < timeLeftProgTrack ) ? 
                        timeLeftMainTrack : timeLeftProgTrack );

    setRepeatingTimerLimit( rNumTimer, timeToInterrupt * TICK_IN_MICROSECONDS );

    if ( followUpMain != DCC_SIG_FOLLOW_UP_NONE ) {

        followUpDccTrackWork( followUpMain, trackA );
    } 

    if ( followUpProg != DCC_SIG_FOLLOW_UP_NONE ) {

        followUpDccTrackWork( followUpProg, trackB );
    } 
   
} // timerCallback

//----------------------------------------------------------------------------------------
// When all DCC track objects are initialized, the last thing to do before 
// operation is to  start the timer heartbeat. We start by firing up the timer 
// with a first short delay, so when it expires the timer routine will be called. 
// The current time tick of zero and no ticks left, so the state machine for the 
// signals will run.
//
//----------------------------------------------------------------------------------------
void initDccTrackProcessing( ) {

    rNumTimer           = 0;
    timeToInterrupt     = 0;
    timeLeftMainTrack   = 0;
    timeLeftProgTrack   = 0;
    
    uint8_t rStat = configureTimer( rNumTimer, timerCallback );
    startRepeatingTimer( rNumTimer, TICKS_58_MICROS );
}   


}; // namespace

using namespace CDC;
using namespace LCS;

//========================================================================================
//========================================================================================
//
// Class part.
//
//========================================================================================
//========================================================================================

//----------------------------------------------------------------------------------------
// The DCC tracks are initialized with a set of class routines. We first setup 
// the class, create the two DCC track objects based on the descriptor data and
// then kick off the DCC timer for the track signal processing.
//
// The idea is that the program first creates all the DCC track objects, does 
// whatever else needs to be initialized and then starts the signal generation 
// with this routine.
//
//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::setupDccTrackLib( ) {


}

LcsDccTrack *LcsDccTrack::createTrackA( LcsDccTrackDesc *desc ) {

    trackA = new LcsDccTrack( );
    trackA -> setupDccTrack( desc );
    return( trackA );
}

LcsDccTrack *LcsDccTrack::createTrackB( LcsDccTrackDesc *desc ) {

    trackB = new LcsDccTrack( );
    trackB -> setupDccTrack( desc );
    return( trackB );
}

LcsDccTrack *LcsDccTrack::getTrackA( ) {

    return ( trackA );
}
    
LcsDccTrack *LcsDccTrack::getTrackB( ) {

    return ( trackB );
}
    
void LcsDccTrack::startDccProcessing( ) {

    initDccTrackProcessing( );
}

//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
//========================================================================================

//----------------------------------------------------------------------------------------
// Object instance section. The DccTrack constructor. Nothing to do so far.
//
//----------------------------------------------------------------------------------------
LcsDccTrack::LcsDccTrack( ) { }

//----------------------------------------------------------------------------------------
// "setupDccTrack" performs the setup tasks for the DCC track.  We will configure 
// the hardware, the DCC packet options such as preamble and postamble length, the
// initial state machine state current consumption limit and load the initial packet
// into the active buffer. There is quite a list of parameters and options that 
// can be set. 
//
// This routine does the parameter checking, and ensures consistency, except
// when critical parameters are wrong. The error is returned via a flag in the
// flag variable. 
//
// Once the DCC track object is initialized, the last thing to do is to remember 
// the object instance in the file static variables. This is necessary for the 
// interrupt handlers to work. 
//
//----------------------------------------------------------------------------------------
bool LcsDccTrack::setupDccTrack( LcsDccTrackDesc* tDesc ) {

    flags                       = DT_F_NIL;
    errCode                     = LCS_OK;
    options                     = tDesc -> options;
    
    rNumEnable                  = tDesc -> rNumEnable;
    rNumControl                 = tDesc -> rNumControl;
    rNumSense                   = tDesc -> rNumSense;
    rNumUartRx                  = tDesc -> rNumUartRx;

    limitCurrentMilliAmp        = tDesc -> limitCurrentMilliAmp;
    maxCurrentMilliAmp          = tDesc -> maxCurrentMilliAmp;
    overloadEventThreshold      = tDesc -> overloadEventThreshold;

    referenceMilliVolt          = tDesc -> referenceMilliVolt;
    shuntMilliOhm               = tDesc -> shuntMilliOhm;
    amplifierGain               = tDesc -> amplifierGain;
    adcBitResolution            = tDesc -> adcBitResolution; 
    adcZeroOffset               = tDesc -> adcZeroOffset;
    trackMode                   = tDesc -> initTrackMode;

    if ((  rNumEnable   == CDC_RN_UNDEFINED )   ||
        (  rNumControl  == CDC_RN_UNDEFINED )   ||
        (  rNumSense    == CDC_RN_UNDEFINED )   ||
        (( trackMode    == DT_M_RAILCOM     )   && 
         ( rNumUartRx   == CDC_RN_UNDEFINED )))  {

        flags = DT_F_SETUP_ERROR;
        errCode = ERR_DCC_RNUM_CONFIG_ERR;
        return( false );
    }

    if ( configureDio( rNumEnable ) != LCS_OK ) {

        flags = DT_F_SETUP_ERROR;
        errCode = ERR_DCC_ENABLE_RNUM_ERR;
        return( false );
    }
    
    if ( configureDio( rNumControl ) != LCS_OK ) {

        flags = DT_F_SETUP_ERROR;
        errCode = ERR_DCC_CONTROL_RNUM_ERR;
        return( false );
    }

    if ( configureAdc( rNumSense ) != LCS_OK ) {

        flags = DT_F_SETUP_ERROR;
        errCode = ERR_DCC_SENSE_RNUM_ERR;
        return( false );
    }

    writeDio( rNumEnable, false );
    writeDio( rNumControl, false, false );

    if ( trackMode == DT_M_RAILCOM ) {

        uint8_t rStat = configureUart( rNumUartRx );
        if ( rStat != LCS_OK ) {

            flags = DT_F_SETUP_ERROR;
            errCode = ERR_DCC_UART_RNUM_ERR;
            return ( false );
        }
    }

    limitCurrentMilliAmp    = clampU16( limitCurrentMilliAmp, 0, maxCurrentMilliAmp );
    overloadEventThreshold  = clampU16( overloadEventThreshold, 0, MAX_OVERLOAD_EVENT_COUNT );
    limitCurrentDigitValue  = milliAmpToAdcDigits( limitCurrentMilliAmp );
    ackThresholdDigitValue  = milliAmpToAdcDigits( ACK_TRESHOLD_VAL );
    actualCurrentDigitValue = 0;
    dccPacketsSend          = 0;

    setTrackMode( trackMode );

    // ??? how to best say which track this is ?

    if ( options & DT_OPT_SERVICE_MODE_TRACK )  trackB = this;
    else                                        trackA = this;

    // ??? we could also fire up the timer here ... if rNumTimer != 0 then 
    // we already did it ...

    // the usage would be 
    //
    //    ???

    return ( true );
}

//----------------------------------------------------------------------------------------
// DCC signal generation is done through a state machine that is invoked when 
// the DCC timer interrupts. The interrupt timer thinks in multiples of 29us, 
// which we will just call a "tick" in the description below. It runs as part of
// the timer interrupt handler, so we need to be short and quick. First, the HW 
// signals are set. This keeps the track signals in their timing. Next, the new
// signal state, time to run again and any other follow up action of this 
// invocation are set. The idea is to separate HW signal generation and follow 
// up actions. The timer interrupt handler will first call both state machines, 
// MAIN and PROG, and then work on the optional follow-up actions. 
//
// The state machine has the following states:
//
//  DCC_SIG_TRACK_OFF: The tack is turned off. We stay in this state until 
//  the state machine is set to DCC_SIG_CUTOUT_START or DCC_SIG_START_BIT.
//
//  DCC_SIG_CUTOUT_START: if the cutout option is on, a new DCC packet starts with 
//  this signal state. The DCC signal goes HIGH for one tick and the signal state
//  advances to signal state DCC_SIG_CUTOUT_1.
//
//  DCC_SIG_CUTOUT_1: this stage sets the signal to CUTOUT for cutout period ticks. 
//  Also, if the RailCom is enabled, there is a follow up request to start the 
//  serial IO read function. The signal state advances to state DCC_SIG_CUTOUT_2.
//
//  DCC_SIG_CUTOUT_2: this stage sets the signal to LOW for the cutout end tick.
//  The signal state advances to signal state DCC_SIG_CUTOUT_3.
//
//  DC_SIG_CUTOUT_3: the DC_SIG_CUTOUT_3 and DC_SIG_END_CUTOUT states represent 
//  the first DCC "One" after the cutout. The DCC signal is set to HIGH and the 
//  next period is two ticks. The follow-up request is to disable the UART receiver. 
//  The signal state advances to DC_SIG_CUTOUT_END. 
//
//  DC_SIG_CUTOUT_END: The DC_SIG_END_CUTOUT state is the second half of the DCC 
//  one. The signal is set to low and the next period to two ticks. If RailCom is
//  enabled, this is the state where a follow up to handle the RailCom data takes 
//  place. The next state is then DCC_SIG_START_BIT to handle the next packet, 
//  starting with the preamble of DCC ones.
//
//  DCC_SIG_START_BIT: this stage is the start of the DCC packet bits, which are 
//  preamble, the data bytes with separators and postamble. If the cutout option 
//  is off, this is also the start for the DCC packet. The signal is set HIGH, 
//  the tick count is two and we need a follow up to get the current bit, which 
//  determines the length of the signal for the bit we just started. The next
//  stage is signal state DCC_SIG_TEST_BIT.  
//
//  DCC_SIG_TEST_BIT: coming from signal state DCC_SIG_START_BIT, we need to see 
//  if the current bit is a ONE or ZERO bit. If a ONE bit, the signal needs to 
//  become LOW, the next period is 2 ticks and the next state is state 
//  DCC_SIG_START_BIT. If it is the last ONE bit of the postamble, the next packet
//  and signal state needs to be determined. For a CUTOUT enabled track this is 
//  state DCC_SIG_START_CUTOUT, else DCC_SIG_START_BIT. If a ZERO bit, the signal
//  is kept HIGH for another two ticks and the state is DCC_SIG_ZERO_SECOND_HALF.
//
//  The first half ZERO bit case is also a good place to do a current measurement 
//  and short circuit detection. The H-Bridge offers a method to signal that a 
//  short circuit was detected and the bridge was shut down. However, after a 
//  short period the bridge turns on after a short time, in the order of 
//  microseconds. For the L6205 chip, we have a dedicated input GPIO pin that we
//  will check on every second half of the zero bit transmitted. When set, we set
//  a flag in the  DCC track flag word.
//
//  The current measurement is also a task done while we transmit a zero bit.
//  The measurement is used in detecting circuits and also recording the power
//  consumption. 
//
//  Once a H-Bridge is turned off, it needs to be enabled manually. Note that
//  this object does not send a notification. The user is expected to periodically
//  check the track state.
//
//  DCC_SIG_ZERO_SECOND_HALF: coming from signal state DCC_SIG_TEST_BIT, we need 
//  to transmit the second half of the ZERO bit. The signal is set to LOW for 
//  four ticks and set the next stage is signal state to DCC_SIG_START_BIT.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::runDccTrackStateMachine(

    volatile uint8_t  *timeToInterrupt,
    uint8_t           *followUpAction

    ) {

    switch ( trackState ) {

        case DCC_SIG_CUTOUT_START: {

            writeDio( rNumControl, true, false );
            *timeToInterrupt    = TICKS_29_MICROS;
            *followUpAction     = DCC_SIG_FOLLOW_UP_NONE;
            trackState          = DCC_SIG_CUTOUT_1;

        } break;

        case DCC_SIG_CUTOUT_1: {

            writeDio( rNumControl, false, false );
            *timeToInterrupt    = TICKS_CUTOUT_MICROS;
            *followUpAction     = (( flags & DT_F_RAILCOM_ON ) ?
                                    DCC_SIG_FOLLOW_UP_START_RAILCOM_IO : 
                                    DCC_SIG_FOLLOW_UP_NONE );
            trackState          = DCC_SIG_CUTOUT_2;

        } break;

        case DCC_SIG_CUTOUT_2: {

            writeDio( rNumControl, false, true );
            *timeToInterrupt    = TICKS_29_MICROS;
            *followUpAction     = DCC_SIG_FOLLOW_UP_NONE;
            trackState          = DCC_SIG_CUTOUT_3;

        } break;

        case DCC_SIG_CUTOUT_3: {

            writeDio( rNumControl, true, false );
            *timeToInterrupt    = TICKS_58_MICROS;
            trackState          = DCC_SIG_CUTOUT_END;

            if ( flags & DT_F_RAILCOM_ON ) {

                flags           |= DT_F_RAILCOM_MSG_PENDING;
                *followUpAction = DCC_SIG_FOLLOW_UP_STOP_RAILCOM_IO;
            }
            else *followUpAction = DCC_SIG_FOLLOW_UP_NONE;

        } break;

        case DCC_SIG_CUTOUT_END: {

            writeDio( rNumControl, false, true );
            *timeToInterrupt    = TICKS_58_MICROS;
            *followUpAction     = (( flags & DT_F_RAILCOM_ON ) ?
                                    DCC_SIG_FOLLOW_UP_RAILCOM_MSG : 
                                    DCC_SIG_FOLLOW_UP_NONE );
            trackState          = DCC_SIG_START_BIT;

        } break;

        case DCC_SIG_START_BIT: {

            writeDio( rNumControl, true, false );
            *timeToInterrupt    = TICKS_58_MICROS;
            *followUpAction     = DCC_SIG_FOLLOW_UP_GET_BIT;
            trackState          = DCC_SIG_TEST_BIT;

        } break;

        case DCC_SIG_TEST_BIT: {

            if ( currentBit ) {

                writeDio( rNumControl, false, true );

                if ( postambleSent >= postambleLen ) {

                    *followUpAction = DCC_SIG_FOLLOW_UP_GET_PACKET;
                    trackState      = (( flags & DT_F_CUTOUT_ON ) ? 
                                        DCC_SIG_CUTOUT_START : DCC_SIG_START_BIT );
                }
                else {

                    *followUpAction = DCC_SIG_FOLLOW_UP_NONE;
                    trackState      = DCC_SIG_START_BIT;
                }
            }
            else {

                *followUpAction     = DCC_SIG_FOLLOW_UP_MEASURE_CURRENT;
                trackState          = DCC_SIG_ZERO_SECOND_HALF;
            }

            *timeToInterrupt  = TICKS_58_MICROS;

        } break;

        case DCC_SIG_ZERO_SECOND_HALF: {

            writeDio( rNumControl, false, true );
            *timeToInterrupt    = TICKS_116_MICROS;
            *followUpAction     = DCC_SIG_FOLLOW_UP_NONE;
            trackState          = DCC_SIG_START_BIT;

        } break;

        default: {

            *followUpAction     = DCC_SIG_FOLLOW_UP_NONE;
            *timeToInterrupt    = TICKS_58_MICROS;
        }
    }
}

//----------------------------------------------------------------------------------------
// The "getNextBit" routine works through the active packet buffer bit for bit. 
// A packet consists of the optional cutout sequence, the preamble bits, the 
// data bytes separated by a ZERO bit and the postamble bits. The cutout option, 
// the preamble and postamble are configured at DCC track object init time. The
// preamble length is different for MAIN and PROG tracks with the the cutout 
// period overlaid at the beginning of the preamble. The postamble is currently 
// always just one HIGH bit, according to standard.
//
// The routine works first through the preamble bit count, then through the data 
// byte bits, and finally through the postamble bits. The bits to select from the 
// data byte is done with a 9-bit mask. Remember that the first bit to send is 
// the data byte separator, which is always a zero. We run from 0 to 8 through 
// the bit mask, the first bit being the ZERO bit.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::getNextBit( ) {

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
// If all bits of a packet have been processed, the next packet is determined
// during the last ONE bit transmission of the postamble. If there is a non-zero 
// repeat count on the current packet, the same packet is sent again until the 
// repeat count drops to zero. On a zero repeat count, we check if there is a 
// pending packet. If so, it is copied to the active buffer and the pending flag 
// is reset. 
//    
// This signals anyone waiting, that the next packet can be queued. If there is 
// no pending packet, we still need to keep the track going and will load an IDLE 
// or RESET packet.
//
// For non-service mode packets, there is a requirement that a decoder should not 
// be receive two consecutive packets. The standards talks about 5 milliseconds 
// between two packets to the same decoder. For now, we will not do anything 
// special.
// 
// A decoder will most likely, if there is more than one decoder active, not be
// addressed in two consecutive packets, simply because the session refresh
// mechanism will go round robin through the session list. However, if there is
// only one decoder active, two packets will be sent in a row, but the decoders 
// are robust enough to ignore this fact. Better run more than one loco :-).
//
// This routine is the central place to submit a DCC packet to the track and 
// therefore a good place to write a DCC_LOG record. We distinguish between a 
// RESET, an IDLE and a data packet. Note that these records will only be written
// when DCC logging is enabled.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::getNextPacket( ) {

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

        if ( flags & DT_F_ACC_DETECT_ON ) {

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
// Railcom. If the cutout period and the RailCom feature is enabled, the signal 
// state machine will also start and stop the UART reader for RailCom data. The 
// final message is then to handle that message. In the cutout period, a decoder 
// sends 8 data bytes. They are divided into two channels, 2 bytes and another 6 
// bytes. The bytes themselves are encoded such that each byte has four bits set, 
// i.e. a hamming weight of 4. The first channel is used to just send the engine
// address when the decoder is addressed. The second channel is used only when 
// the decoder is explicitly addressed via a CV operation command to provide the
// answer to the request.
//
// The received datagrams are also recorded in the DCC_LOG, if enabled.
//
// ??? under construction....
// ??? we could store the last loco address in some global variable.
// ??? we could store the channel 2 datagram in the corresponding session.
// ??? still, both pieces of data needs to go somewhere before the next message
//  is received...
//----------------------------------------------------------------------------------------
void LcsDccTrack::startRailComIO( ) {

    startUartRead( rNumUartRx );
}

void LcsDccTrack::stopRailComIO( ) {

    stopUartRead( rNumUartRx );
}

uint8_t LcsDccTrack::handleRailComMsg( ) {

    railComBufIndex = getUartBuffer( rNumUartRx, 
                                     railComMsgBuf, 
                                     sizeof( railComMsgBuf ));

    writeLogData( LOG_DCC_RCM, railComMsgBuf, railComBufIndex );

    for ( uint8_t i = 0; i < railComBufIndex; i++ ) {

        uint8_t dataByte = railComDecode[ railComMsgBuf[ i ]];

        if      ( dataByte == ACK ) ;
        else if ( dataByte == NACK ) ;
        else if ( dataByte == BUSY ) ;
        else if ( dataByte < 64 ) {

            // ??? valid
            // ??? a railCom message can have multiple datagrams
            // we would need to handle each datagram, one at a time or fill them 
            // into a kind of structure
            // that has a slot for the up to maximum 4 datagrams per railCom
            // cutout period.
        }
        else {

            // ??? invalid packet ... if this is channel2, discard the entire message.
        }

        railComMsgBuf[ i ] = dataByte;
    }

    flags &= ~ DT_F_RAILCOM_MSG_PENDING;
    return ( LCS_OK );
}

// ??? not very useful, but good for debugging and initial testing .... 
// and it works like a champ :-)

uint8_t LcsDccTrack::getRailComMsg( uint8_t *buf, uint8_t bufLen ) {

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
// Some getter functions. Straightforward.
//
//----------------------------------------------------------------------------------------
uint16_t LcsDccTrack::getFlags( ) {

    return ( flags );
}

uint16_t LcsDccTrack::getOptions( ) {

    return ( options );
}

uint32_t LcsDccTrack::getDccPacketsSend( ) {

    return ( dccPacketsSend );
}

bool LcsDccTrack::isPowerOn( ) {

    return ( flags & DT_F_POWER_ON );
}

bool LcsDccTrack::isPowerOverload( ) {

    return ( flags & DT_F_POWER_OVERLOAD );
}

uint8_t LcsDccTrack::getErrCode( ) {

    return ( errCode );
}

//----------------------------------------------------------------------------------------
// DCC track power management functions. 
//
// ??? check what calls to offer .... 
// 
//----------------------------------------------------------------------------------------
void LcsDccTrack::powerEnable( bool enable ) {


    // ??? clear all flags and threshold counters on ON....

    flags &= ~ DT_F_POWER_OVERLOAD;

    if ( enable ) {

    
        flags |= DT_F_POWER_ON;
    }
    else {

        flags &= ~ DT_F_POWER_ON;
    }

   writeDio( rNumEnable, enable );
}

//----------------------------------------------------------------------------------------
// "setTrackMode" is the entry point to manage the track. For the different 
// modes, we set the packet characteristics and point the track state machine
// at the respective start.
//
// ??? perhaps a bit tricky if we are interrupted in the middle....
//----------------------------------------------------------------------------------------
void  LcsDccTrack::setTrackMode( DccTrackMode mode ) {

    trackMode = mode;

    switch ( trackMode ) {

        case DT_M_PLAIN: {

            preambleLen     =   MAIN_PACKET_PREAMBLE_BIT_LEN;
            postambleLen    =   MAIN_PACKET_POSTAMBLE_BIT_LEN;
            activeBufPtr    =   &idleDccPacket;
            pendingBufPtr   =   &dccBuf1;
            trackState      =   DCC_SIG_START_BIT;

        } break;

        case DT_M_CUTOUT: {

            preambleLen     =   MAIN_PACKET_PREAMBLE_BIT_LEN;
            preambleLen     -=  DCC_PACKET_CUTOUT_BIT_LEN;
            postambleLen    =   MAIN_PACKET_POSTAMBLE_BIT_LEN;
            flags           |=  DT_F_CUTOUT_ON;
            activeBufPtr    =   &idleDccPacket;
            pendingBufPtr   =   &dccBuf1;
            trackState      =   DCC_SIG_CUTOUT_START;

        } break;

        case DT_M_RAILCOM: {

            preambleLen     =   MAIN_PACKET_PREAMBLE_BIT_LEN;
            preambleLen     -=  DCC_PACKET_CUTOUT_BIT_LEN;
            postambleLen    =   MAIN_PACKET_POSTAMBLE_BIT_LEN;
            flags           |=  DT_F_CUTOUT_ON | DT_F_RAILCOM_ON;
            activeBufPtr    =   &idleDccPacket;
            pendingBufPtr   =   &dccBuf1;
            trackState      =   DCC_SIG_CUTOUT_START;

        } break;

        case DT_M_ACC_DETECT: {

            preambleLen     =   PROG_PACKET_PREAMBLE_BIT_LEN;
            postambleLen    =   PROG_PACKET_POSTAMBLE_BIT_LEN;
            flags           |=  DT_F_ACC_DETECT_ON;
            activeBufPtr    =   &resetDccPacket;
            pendingBufPtr   =   &dccBuf1;
            trackState      =   DCC_SIG_START_BIT;

        } break;

        default: {

            preambleLen     =   MAIN_PACKET_PREAMBLE_BIT_LEN;
            postambleLen    =   MAIN_PACKET_POSTAMBLE_BIT_LEN;
            activeBufPtr    =   &idleDccPacket;
            pendingBufPtr   =   &dccBuf1;
            trackState      =   DCC_SIG_START_BIT;
        }
    }
}

DccTrackMode LcsDccTrack::getTrackMode( ) {

    return( trackMode );
}
 
//----------------------------------------------------------------------------------------
// This function is called by the DCC signal state machine to analyze the 
// H-Bridge power consumption. It is invoked on a zero bit, the second half
// of the signal. The sample is taken at the middle of this part of the 
// signal, to have a stable measurement.
// 
// If the sample is higher than the current limit, we count N of such samples
// in a row, and if so, stop the bridge.
//
// For supporting ACC detect, we keep a high water value mark, which is updated
// if the current sample is higher than the high water mark.
//
// Finally, we add the square of every Nth sample to a sum, keeping track how 
// many sample we took. When the caller wants to know the current consumption
// it is a matter of diving the sum by the number of samples and taking the 
// square root.
//
// This routine runs as part of the interrupt request handler!
//----------------------------------------------------------------------------------------
void LcsDccTrack::powerManagement( ) {

    uint16_t adcVal;
    uint8_t  rStat = readAdc( rNumSense, &adcVal );

    actualCurrentDigitValue = adcVal;

    if ( actualCurrentDigitValue > highWaterMarkDigitValue ) 
            highWaterMarkDigitValue = actualCurrentDigitValue;

    if ( actualCurrentDigitValue > limitCurrentDigitValue ) {

        overloadEventCount ++;

        if ( overloadEventCount > overloadEventThreshold ) {

            flags |= DT_F_POWER_OVERLOAD;
            flags &= ~DT_F_POWER_ON;

            writeDio( rNumEnable, false );
        }
    }
    else overloadEventCount = 0;

    if ( sampleCount < sampleSize ) {

        sampleIntervalCount ++;
        if ( sampleIntervalCount >= sampleIntervalSize ) {

            sampleIntervalCount = 0;
            sampleSum += ( uint32_t) actualCurrentDigitValue * 
                                ( uint32_t) actualCurrentDigitValue;

            sampleCount ++;
        }    
    }
    else flags |= DT_F_POWER_SAMPLE_PENDING;
}

//----------------------------------------------------------------------------------------
// The DCC decoder programming requires the detection of a current consumption 
// change. This is the way a DCC decoder signals an acknowledgement. To detect 
// the consumption change we need first an idea what the actual average current 
// baseline consumption of the decoder is. This method will send the required 
// DCC reset packets according to the DCC standard and at the same time determine 
// the current consumption as a baseline. We use the high water mark for this 
// purpose.
//
// ??? although the routines for decoder ACK detection work, they will produce 
// quite a number of packets. During this time, other LCS work is blocked. 
// Perhaps we need a kind of state machine approach to cut the long sequence 
// in smaller chunks to allow other work in between.
//----------------------------------------------------------------------------------------
uint16_t LcsDccTrack::decoderAckBaseline( uint8_t resetPacketsToSend ) {

    if ( dccTrackDebugAccDetect( )) {

        printf( "\nDecoder Ack setup: ( " );
    }

    uint16_t sum = 0;

    for ( int i = 0; i < resetPacketsToSend; i++ ) {

        highWaterMarkDigitValue = 0;

        loadPacket( resetDccPacketData, 2, 0 );

        if ( dccTrackDebugAccDetect( )) {
       
            printf( "%d ", highWaterMarkDigitValue );
        }

        sum += highWaterMarkDigitValue;
    }

    if ( dccTrackDebugAccDetect( )) {

        printf( ") -> %d\n", sum / resetPacketsToSend );
    }

    return ( sum  / resetPacketsToSend );
}

//----------------------------------------------------------------------------------------
// "decoderAckDetect" is the counterpart to the decoder ack setup routine. The 
// setup method established a base line for the power consumption and put the 
// decoder in CV programming mode by sending the RESET packets. The decoder ACK 
// detect routine now sends out resets packets to follow the programming packets
// required and monitors the current consumption. We use the high water mark for 
// this purpose. The DCC standard specifies a time window in which the decoder 
// should raise its power consumption level and signal an acknowledge this way. 
// We will send out a series of reset packets and monitor after each packet the
// consumption level. The number of retries depends on whether it is a read 
// ( 50ms window ) or a write ( 100ms window ). If we detect a raised value the 
// decoder did signal a positive outcome. If not, we time out after the last 
// reset packet. The programming operation either failed or the decoder did on 
// purpose not answer. We cannot tell.
//
// ??? although the routines for decoder ACK detection work, they will produce 
// quite a number of packets. During this time, other LCS work is blocked. 
// Perhaps we need a kind of state machine approach to cut the long sequence 
// in smaller chunks to allow other work in between.
//----------------------------------------------------------------------------------------
bool LcsDccTrack::decoderAckDetect( uint16_t baseDigitValue, uint8_t  retries ) {

    if ( dccTrackDebugAccDetect( )) {

        printf( "Decoder Ack detect: ( %d : %d : ( ", 
                baseDigitValue, ackThresholdDigitValue );
    }

    for ( uint8_t i = 0; i < retries; i++ ) {

        highWaterMarkDigitValue = 0;
        loadPacket( resetDccPacketData, 2, 0 );

        if ( dccTrackDebugAccDetect( )) {

            printf( "%d ", highWaterMarkDigitValue );
        }

        if (( highWaterMarkDigitValue >= baseDigitValue ) &&
            ( highWaterMarkDigitValue - baseDigitValue >= ackThresholdDigitValue )) {

            if ( dccTrackDebugAccDetect( )) {

                printf( "[ %d ] ) -> OK\n", 
                        abs( highWaterMarkDigitValue - baseDigitValue ));
            }

            return ( true );
        }
    }

    if ( dccTrackDebugAccDetect( )) printf( ") -> FAILED" );
    return ( false );
}

//----------------------------------------------------------------------------------------
// Power Consumption Management. The external world thinks in milliAmps.
//
//----------------------------------------------------------------------------------------
uint16_t LcsDccTrack::getActualCurrent( ) {

    return ( adcDigitsToMilliAmp( actualCurrentDigitValue ));
}

uint16_t LcsDccTrack::getLimitCurrent( ) {

    return ( limitCurrentMilliAmp );
}

uint16_t LcsDccTrack::getMaxCurrent( ) {

    return ( maxCurrentMilliAmp );
}

void LcsDccTrack::setLimitCurrent( uint16_t val ) {

    if ( val > maxCurrentMilliAmp  ) val = maxCurrentMilliAmp;

    limitCurrentMilliAmp    = val;
    limitCurrentDigitValue  = milliAmpToAdcDigits( val );
}

//----------------------------------------------------------------------------------------
// The "getRMSCurrent" function returns the power consumption based on the 
// samples taken. 
//
//----------------------------------------------------------------------------------------
uint16_t LcsDccTrack::getRMSCurrent( ) {

    uint16_t res = adcDigitsToMilliAmp( sqrt( res / sampleSize ));

    sampleSum           = 0;
    sampleCount         = 0;
    sampleIntervalCount = 0;

    flags &= ~DT_F_POWER_SAMPLE_PENDING;

    return ( res );
}

//----------------------------------------------------------------------------------------
// LoadPacket is the central entry point to submit a DCC packet. The incoming 
// packet is the the data to be sent without checksum, i.e. it is just the
// payload. The DCC track signal generator has two packet buffers. The first 
// buffer holds the packet currently being transmitted. The second is the pending
// buffer. If it is used, we will simply busy wait for our turn to load the 
// packet into the pending buffer. Upon completion of sending the active packet,
// the interrupt handler copies the currently pending buffer to the active buffer
// and then resets the pending flag. Either way, then it is our turn. We fill 
// the pending buffer, compute the checksum and set the pending flag.
//
// ??? For a high number of session we may want to think about a queuing approach. 
// Right now, this routine waits when there is a packet already queued, i.e. pending. 
// This may cause issues in delaying other tasks such as receiving a CAN bus message.
//----------------------------------------------------------------------------------------
void LcsDccTrack::loadPacket( const uint8_t *packet, 
                                         uint8_t len, 
                                         uint8_t repeat ) {

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
// The log management routines. A typical transaction to log would start the 
// logging process and then end it after the operation to analyze/debug. The 
// "enableLog" call should be used to enable the logging process all together, 
// the other calls will only do work when the log is enabled. With this call 
// the recording process could be controlled from a command line setting or so.
// "beginLog" and "endLog" start and end a recording sequence.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::enableLog( bool enable ) {

    if ( enable ) {

        flags |= DT_F_LOG_ENABLED;
        flags &= ~ DT_F_LOG_ACTIVE;
    }
    else {

        flags &= ~ DT_F_LOG_ENABLED;
        flags &= ~ DT_F_LOG_ACTIVE;
    } 
}

void LcsDccTrack::beginLog( ) {

    if ( flags & DT_F_LOG_ENABLED ) {

        flags |= DT_F_LOG_ACTIVE;
        logBufOfs = 0;
        writeLogId( LOG_BEGIN );
        writeLogTs( );
    }
}

void LcsDccTrack::endLog( ) {

    if ( flags & DT_F_LOG_ACTIVE ) {

        writeLogTs( );
        writeLogId( LOG_END );
        flags &= ~ DT_F_LOG_ACTIVE;
    }
}

//----------------------------------------------------------------------------------------
// There are a couple of routines to write the log data when the logging is active. 
// The order of data entry for numeric types is big endian, i.e. most significant
// byte first.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::writeLogData( uint8_t id, uint8_t *buf, uint8_t len ) {

    if ( flags & DT_F_LOG_ACTIVE ) {

        len = len % 16;
        if ( logBufOfs + len + 1 < LOG_BUF_SIZE ) {

            logBuf[ logBufOfs ++ ] = ( id << 4 ) | len;
            for ( uint8_t i = 0; i < len; i++ ) logBuf[ logBufOfs ++ ] = buf[ i ];
        }
    }
}

void LcsDccTrack::writeLogId( uint8_t id ) {

    if ( flags & DT_F_LOG_ACTIVE ) logBuf[ logBufOfs ++ ] = ( id << 4 );
}

void LcsDccTrack::writeLogTs( ) {

    if ( flags & DT_F_LOG_ACTIVE ) {

        uint32_t ts = getMicros( );
        logBuf[ logBufOfs ++ ] = ( LOG_TSTAMP << 4 ) | 4;
        logBuf[ logBufOfs ++ ] = ( ts >> 24 ) & 0xFF;
        logBuf[ logBufOfs ++ ] = ( ts >> 16 ) & 0xFF;
        logBuf[ logBufOfs ++ ] = ( ts >> 8  ) & 0xFF;
        logBuf[ logBufOfs ++ ] = ( ts >> 0  ) & 0xFF;
    }
}

void LcsDccTrack::writeLogVal( uint8_t valId, uint16_t val ) {

    if ( flags & DT_F_LOG_ACTIVE ) {

        logBuf[ logBufOfs ++ ] = ( LOG_VAL << 4 ) | 3;
        logBuf[ logBufOfs ++ ] = valId;
        logBuf[ logBufOfs ++ ] = val >> 8;
        logBuf[ logBufOfs ++ ] = val & 0xFF;
    }
}

//----------------------------------------------------------------------------------------
// There are a couple of routines to print out the log data.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::printLogTimeStamp( uint16_t ofs ) {

    uint32_t ts = logBuf[ ofs ];
    ts = ( ts << 8 ) | logBuf[ ofs + 1 ];
    ts = ( ts << 8 ) | logBuf[ ofs + 2 ];
    ts = ( ts << 8 ) | logBuf[ ofs + 3 ];
    printf( "0x%x", ts );
}

void LcsDccTrack::printLogVal( uint16_t ofs ) {

    uint16_t val = logBuf[ ofs ] << 8 | logBuf[ ofs + 1 ];
    printf( "0x%04x", val );
}

void LcsDccTrack::printLogData( uint16_t ofs, uint8_t len ) {

    for ( int i = 0; i < len; i++ ) printf( "0x%02x ", logBuf[ ofs + i ] );
}

uint8_t LcsDccTrack::printLogEntry( uint16_t ofs ) {

    if ( ofs < LOG_BUF_SIZE ) {

        uint8_t logEntryId  = logBuf[ ofs ] >> 4;
        uint8_t logEntryLen = logBuf[ ofs ] & 0x0F;

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
            default:           printf( "INVALID ( 0x%02 )", logBuf[ ofs ] >> 4 );
        }

        if      ( logEntryId == LOG_TSTAMP  )  printLogTimeStamp( ofs + 1 );
        else if ( logEntryId == LOG_VAL     )  printLogVal( ofs + 1 );
        else                                   printLogData( ofs + 1, logEntryLen );

        return ( logEntryLen + 1 );
    }
    else return ( 0 );
}

void LcsDccTrack::printLog( ) {

    if (( flags & DT_F_LOG_ENABLED ) && ( ! ( flags & DT_F_LOG_ACTIVE ))) {

        if ( logBufOfs > 0 ) {

            printf( "\n" );

            uint16_t entryIndex  = 0;
            uint8_t  entryLen    = 0;

            while ( entryIndex < logBufOfs ) {

                entryLen = printLogEntry( entryIndex );
                printf( "\n" );

                if ( entryLen > 0 ) entryIndex += entryLen;
                else                break;
            }
        }
        else printf( "DCC Log Buf: Nothing recorded\n" );
    }
    else printf( "DCC Log disabled or active\n" );
}

//----------------------------------------------------------------------------------------
// Print out the DCC Track configuration data. For debugging purposes.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::printDccTrackConfig( ) {

    printf( "DccTrack Config: " );

    if ( options & DT_OPT_SERVICE_MODE_TRACK ) printf( "PROG \n" );
    else                                       printf( "MAIN \n" );

    printf( " Config options: ( 0x%x ) -> ", flags );
    
    if ( options &  DT_OPT_SERVICE_MODE_TRACK ) printf( "SvcMode Track " );
    printf( "\n" );

    printf( " Current Limit(mA): %d Current Max(mA): %d\n",
            getLimitCurrent( ), getMaxCurrent( ));
    

    printf( " Limit Digit Value: %d\n", limitCurrentDigitValue );
    printf( " Ack Threshold Digit Value:%d\n", ackThresholdDigitValue );

    printf( " CDC enable rNum: %d, DCC control rNum: %d, "
            "Sensor nRum: %d, RailCom rNum: %d\n",
            rNumEnable, rNumControl, rNumSense, rNumUartRx );

    printf( " PreambleLen: %d, PostambleLen: %d\n", preambleLen, postambleLen );
}

//----------------------------------------------------------------------------------------
// Print out the DCC Track status.
//
//----------------------------------------------------------------------------------------
void LcsDccTrack::printDccTrackStatus( ) {

    printf( "DccTrack: " );

    if ( options & DT_OPT_SERVICE_MODE_TRACK )  printf( "PROG" );
    else                                        printf( "MAIN" );

    printf( ", Track Status: ( 0x%x ) -> ", flags );
    
    if ( flags & DT_F_POWER_ON         ) printf( "PowerOn " );
    if ( flags & DT_F_POWER_OVERLOAD   ) printf( "PowerOverload " );
    if ( flags & DT_F_ACC_DETECT_ON ) printf( "AccDetectOn " );
    if ( flags & DT_F_CUTOUT_ON   ) printf( "CutoutOn " );
    if ( flags & DT_F_RAILCOM_ON  ) printf( "RailcomOn " );
    if ( flags & DT_F_SETUP_ERROR     ) printf( "ConfigError " );
    printf( "\n" );

    printf( "Packets Send: %d\n", dccPacketsSend );
    printf( "Power consumption (RMS): %d\n", getRMSCurrent( ));
    printf( "\n" );
}
