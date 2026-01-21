//----------------------------------------------------------------------------------------
//
// LCS Block Controller - RailCom Support
//
//----------------------------------------------------------------------------------------
//
//
//
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - RailCom Support
// Copyright (C) 2020 - 2026  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsBlockController.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// External declaration to global structures and routines in other files.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// "debugEnabled" and "retStat" are the debug support routines. We can easily 
// check whether debug is enabled at all. The return status routine will print 
// out a return status message when debugging is enabled. The macro "RET_STAT" 
// is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool railComDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_RAILCOM )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( railComDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------

} // namespace

//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
//========================================================================================
// Object constructor.
//
//----------------------------------------------------------------------------------------
LcsRailComDetect::LcsRailComDetect( ) {

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsRailComDetect::setupRailComDetect( ) {

    if ( railComDebugEnabled( )) printf( "Setup RailCom detect\n" );


    return( RET_STAT( LCS_OK ));
}


// ??? to work on ...

#if 0 

// ??? perhaps it makes more sense to create a RailCom library, as we also need 
// some of all this for the base station....

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
// RailCom decoder table. The Railcom communication will send raw bytes where only
// four bits are "one" in a byte ( hamming weight 4 ). The first two bytes are 
// labelled "channel1" and the remaining six bytes are labelled "channel2". The 
// actual data is then encode using the table below. Each raw byte will be 
// translated to a 6 bits of data for the datagram to assemble. In total there 
// are therefore a maximum of 48bits that are transmitted in a railcom message.
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

    RC_DG_TYPE_UNDEFINED  = 0,
    RC_DG_TYPE_MOB        = 1,
    RC_DG_TYPE_STAT       = 2
};

//----------------------------------------------------------------------------------------
// Each mobile decoder railcom datagram will start with an ID field of four bits. 
// Channel one will use only the ADR_HIG and ADR_LOW Ids. All IDs can be used for
// channel two. Since decoders answer on channel one for each DCC packet they 
// receive, here is a good chance that channel one will contains nonsense data.
// This is different for channel two, where only the addressed decoder explicitly 
// answers. To decide whether a railcom message is valid, you should perhaps 
// ignore channel one data and just check channel two for this purpose. A Railcom
// datagram starts with the 4-bit ID and an 8 to 32bit payload.
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
// The RailCom buffer size. During the cutout period up to eight bytes of raw data
// are sent by the decoder if the Railcom option is enabled.
//
//----------------------------------------------------------------------------------------
const uint8_t   RAILCOM_BUF_SIZE = 8;

struct RailCom {

    void        startRailComIO( );
    void        stopRailComIO( );
    uint8_t     handleRailComMsg( );
    uint8_t     getRailComMsg( uint8_t *buf, uint8_t bufLen );
};




//----------------------------------------------------------------------------------------
// Railcom. If the cutout period and the RailCom feature is enabled, the signal 
// state machine will also start and stop the UART reader for RailCom data. The 
// final message is then to handle that message. In the cutout period, a decoder
// sends 8 data bytes. They are divided into two channels, 2bytes and another 6 
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
// is received...
//----------------------------------------------------------------------------------------
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
            // we would need to handle each datagram, one at a time or fill them
            // into a kind of structure
            // that has a slot for the up to maximum 4 datagrams per railCom cutout
            // period.
        }
        else {

            // ??? invalid packet ... if this is channel2, discard the entire message.
        }

        railComMsgBuf[ i ] = dataByte;
    }

    flags &= ~ DT_F_RAILCOM_MSG_PENDING;
    return ( ALL_OK );
}

// ??? not very useful, but good for debugging and initial testing .... and it works
// like a champ :-)

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

#endif