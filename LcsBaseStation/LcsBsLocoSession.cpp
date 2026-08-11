//----------------------------------------------------------------------------------------
//
// LCS Base Station - Loco Session Management - implementation file
//
//----------------------------------------------------------------------------------------
// The locomotive session object is the besides the two DCC tracks the other main
// component of a base station. Each engine to run needs a session on this session 
// object. Typically, the handheld will "open" a session. The session identifier is
// then the handle to the locomotive. 
//
//----------------------------------------------------------------------------------------
//
// LCS - Base Station
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsBaseStation.h"
#include <malloc.h>

using namespace LCS;

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// Loco Session implementation file - local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// DCC packet definitions. A DCC packet payload is at most 10 bytes long, excluding
// the checksum byte. This is true for XPOM support, otherwise it is according to
// NMRA up to 6 bytes.
//
//----------------------------------------------------------------------------------------
const uint8_t   MIN_DCC_PACKET_SIZE         = 2;
const uint8_t   MAX_DCC_PACKET_SIZE         = 16;
const uint8_t   MIN_DCC_PACKET_REPEATS      = 0;
const uint8_t   MAX_DCC_PACKET_REPEATS      = 8;

//----------------------------------------------------------------------------------------
// Utility routines.
//
//----------------------------------------------------------------------------------------
bool validCabId( uint16_t cabId ) {

    return ( isInRangeU( cabId, MIN_CAB_ID, MAX_CAB_ID ));
}

bool validCvId( uint16_t cvId ) {

    return ( isInRangeU( cvId, MIN_DCC_CV_ID, MAX_DCC_CV_ID ));
}

bool validFunctionId( uint8_t fId ) {

    return ( isInRangeU( fId, MIN_DCC_FUNC_ID, MAX_DCC_FUNC_ID ));
}

bool validFunctionGroupId( uint8_t fGroup ) {

    return ( isInRangeU( fGroup, MIN_DCC_FUNC_GROUP_ID , MAX_DCC_FUNC_GROUP_ID ));
}

bool validDccPacketlen( uint8_t len ) {

    return ( isInRangeU( len, MIN_DCC_PACKET_SIZE, MAX_DCC_PACKET_SIZE ));
}

bool validDccPacketRepeatCnt( uint8_t nRepeat ) {

    return ( isInRangeU( nRepeat, MIN_DCC_PACKET_REPEATS, MAX_DCC_PACKET_REPEATS ));
}

uint8_t bitRead( uint8_t arg, uint8_t pos ) {

    return ( arg >> ( pos % 8 )) & 1;
}

void bitWrite( uint8_t *arg, uint8_t pos, bool val ) {

    if ( val )  *arg |= ( 1 << pos );  
    else        *arg &= ~( 1 << pos ); 
}

//----------------------------------------------------------------------------------------
// DDC function flags. The DCC function flags F0 .. F68 are stored in ten groups. 
// Group 0 contains F0 .. F4 stored in DCC command byte format. Group 1 contains 
// F5 .. F8, Group 2 contains F9 .. F12 in DCC command byte format. The remainder 
// F13 .. F68 are stored in 8 bits groups also in DCC command byte format. The
// routines support the get/set of an individual bit as well as setting an entire 
// function group. A DCC function group is labelled starting with index 1.
//
//----------------------------------------------------------------------------------------
bool getDccFuncBit( uint8_t *funcFlags, uint8_t fNum ) {

    if      ( fNum == 0 )                 return ( bitRead( funcFlags[ 0 ], 4 ));
    else if ( isInRangeU( fNum, 1, 4 ))   return ( bitRead( funcFlags[ 0 ], fNum - 1 ));
    else if ( isInRangeU( fNum, 5, 8 ))   return ( bitRead( funcFlags[ 1 ], fNum - 5 ));
    else if ( isInRangeU( fNum, 9, 12 ))  return ( bitRead( funcFlags[ 2 ], fNum - 9 ));
    else if ( isInRangeU( fNum, 13, 68 )) {

        return ( bitRead( funcFlags[ ( fNum - 13 ) / 8 + 3 ], ( fNum - 13 ) % 8 ));
    }   
    else return false;
}

void setDccFuncBit( uint8_t *funcFlags, uint8_t fNum, bool val ) {

    if      ( fNum == 0 )                 bitWrite( &funcFlags[ 0 ], 4, val );
    else if ( isInRangeU( fNum, 1, 4 ))   bitWrite( &funcFlags[ 0 ], fNum - 1, val );
    else if ( isInRangeU( fNum, 5, 8 ))   bitWrite( &funcFlags[ 1 ], fNum - 5, val );
    else if ( isInRangeU( fNum, 9, 12 ))  bitWrite( &funcFlags[ 2 ], fNum - 9, val );
    else if ( isInRangeU( fNum, 13, 68 )) {

        bitWrite( &funcFlags[ ( fNum - 13 ) / 8 + 3 ], ( fNum - 13 ) % 8, val );
    }
}

void setDccFuncGroupByte( uint8_t *funcFlags, uint8_t fGroup, uint8_t dccByte ) {

    if       ( fGroup == 1 )                  funcFlags[ 0 ] = dccByte & 0x1F;
    else if  ( fGroup == 2 )                  funcFlags[ 1 ] = dccByte & 0x0F;
    else if  ( fGroup == 3 )                  funcFlags[ 2 ] = dccByte & 0x0F;
    else if  ( isInRangeU( fGroup, 4, 10 ))   funcFlags[ fGroup - 1 ] = dccByte;
}

uint8_t dccFunctionBitToGroup( uint8_t fNum ) {

    if      ( isInRangeU( fNum, 0, 4 ))     return ( 1 );
    else if ( isInRangeU( fNum, 5, 8 ))     return ( 2 );
    else if ( isInRangeU( fNum, 9, 12 ))    return ( 3 );
    else if ( isInRangeU( fNum, 13, 68 ))   return (( fNum - 13 ) / 8 + 4 );
    else                                    return ( 0 );
}

}; // namespace

//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
//========================================================================================

//----------------------------------------------------------------------------------------
// "LocoSession" constructor. Nothing to do here.
//
//----------------------------------------------------------------------------------------
LcsBaseStationLocoSession::LcsBaseStationLocoSession( ) { }

//----------------------------------------------------------------------------------------
// Loco Session Map configuration. The session map contains an array of loco 
// sessions entries. We are passed the sessionMap descriptor and object handles 
// to the core library and the two tracks. Loco sessions are numbered from 1 to 
// MAX_SESSION_ID. 
//
// ??? we need track and options, no session maps any more...
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::setupSessionMap(

    LcsBaseStationSessionMapDesc  *sessionMapDesc,
    LcsDccTrack        *mainTrack,
    LcsDccTrack        *progTrack

    ) {

    if (( mainTrack == nullptr                              ) ||
        ( progTrack == nullptr                              ) ||
        ( sessionMapDesc -> maxSessions > MAX_CAB_SESSIONS  )) 
        return ( ERR_SESSION_SETUP );

    this -> mainTrack     = mainTrack;
    this -> progTrack     = progTrack;

    options               = sessionMapDesc -> options;
    flags                 = SM_F_DEFAULT_SETTING;

    sessionMap            = (SessionMapEntry *) calloc( 
                                sessionMapDesc -> maxSessions, 
                                sizeof( SessionMapEntry ));

    lastAliveCheckTime    = getMillis( );

    sessionMapHwm         = sessionMap;
    sessionMapLimit       = &sessionMap[ sessionMapDesc -> maxSessions ];
    sessionMapNextRefresh = sessionMap;

    if ( options & SM_OPT_ENABLE_REFRESH )      flags |= SM_F_ENABLE_REFRESH;
    if ( options & SM_OPT_KEEP_ALIVE_CHECKING ) flags |= SM_F_KEEP_ALIVE_CHECKING;

    for ( SessionMapEntry *smePtr = sessionMap; smePtr < sessionMapLimit; smePtr++ ) 
        initSessionEntry( smePtr );

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "requestSession" is the entry point to establish a session. There are several 
// modes. The NORMAL mode is to allocate a new session. There should be no session
// already existing for this cabId. The STEAL mode grabs an existing session from 
// the current session holder. The use case is that a dispatched locomotive can be
// taken over by another handheld. The SHARED option allows several handheld 
// controller to share the session entry and issue commands to the same locomotive. 
// Right now, the STEAL and SHARED option are not implemented.
//
// ??? there is no session concept anymore... out
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::requestSession(  uint16_t cabId, 
                                                    uint8_t mode, 
                                                    uint8_t *sId ) {

    *sId = NIL_LOCO_SESSION_ID;
    if ( ! validCabId( cabId )) return ( ERR_INVALID_CAB_ID );

    switch ( mode ) {

        case LSM_NORMAL: {

            SessionMapEntry *smePtr = allocateSessionEntry( cabId );
            if ( smePtr == nullptr ) return ( ERR_LOCO_SESSION_ALLOCATE );

            smePtr -> flags |= SME_SPDIR_REFRESH;

            *sId = smePtr - sessionMap + 1;
            return ( LCS_OK );
        }

        case LSM_STEAL: {

            // ??? need to inform the current handheld and put the new handheld in 
            // its place.

            return ( ERR_NOT_IMPLEMENTED );

        } break;

        case LSM_SHARED: {

            // ??? essentially, add another handheld to the session. We perhaps 
            // need a counter on how many handhelds share the session ...

            return ( ERR_NOT_IMPLEMENTED );

        } break;

        default: return ( ERR_NOT_IMPLEMENTED ); // ??? rather "invalid mode" ?
    }
}

//----------------------------------------------------------------------------------------
// A cab session can be released, freeing up the slot in the cab session table.
//
// ??? no session concept - out
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::releaseSession( uint8_t sId ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    deallocateSessionEntry( smePtr );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "updateSession" informs the base station about changes in the loco session 
// setting. To be implemented once we know what the flags and the update concept 
// should be ...
//
// ??? no session concept - out
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::updateSession( uint8_t sId, uint8_t flags ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "markSessionAlive" sets the keep alive time stamp on a loco session. This 
// routine is typically called by the LCS message receiver to update the session 
// last "alive" timestamp. The base station will periodically check this value to 
// see if a session is still alive.
//
// ??? change to markCabAlive
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::markSessionAlive( uint8_t sId ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    smePtr -> lastKeepAliveTime = CDC::getMillis( );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "refreshActiveSessions" walks through the session map up to the high water mark
// and invokes the session refresh function for each used entry. As the refresh
// entry routine will show, we will do this refreshing in small pieces in order to
// stay responsive to external requests.
//
//
// ??? this may should perhaps all be reworked. There are many more duties to do
// periodically.
//
// ??? an active loco ( speed > 0 ) needs to be address at least every 2.5 seconds.
//
// ??? also a base station needs to broadcast its capabilities every
//
//
// ??? change to refreshActiveCabIds
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::refreshActiveSessions( ) {

    if (( flags & SM_F_ENABLE_REFRESH ) && ( sessionMapHwm > sessionMap )) {

        refreshSessionEntry( sessionMapNextRefresh );

        sessionMapNextRefresh ++;

        if ( sessionMapNextRefresh >= sessionMapHwm ) 
            sessionMapNextRefresh = sessionMap;
    }
}

//----------------------------------------------------------------------------------------
// "refreshSessionEntry" checks first that the session is still alive and then 
// issues the next DCC packet for refreshing the loco session. To avoid DCC 
// bandwidth issues, a loco session refresh is done in several small steps. There
// is one state for speed and direction and steps to refresh the function groups 
// 1 to 5. If the function refresh option is set, we use the DCC command that sets
// speed, direction and the function flags in one DCC command.
//
//    Step 0  -> refresh speed and direction 
//              ( if FUNC_REFRESH is set also functions F0 .. F28 )
//
//    Step 1  -> refresh function group 0 ( F0  .. F4  )
//    Step 2  -> refresh function group 1 ( F5  .. F8  )
//    Step 3  -> refresh function group 2 ( F9  .. F12 )
//    Step 4  -> refresh function group 3 ( F13 .. F20 )
//    Step 5  -> refresh function group 4 ( F21 .. F28 )
//
// ??? should we alternate when SPDIR and FUNC are sent separately ?
// ??? is it something like: SPDIR, FG1, SPDIR, FG2, ...
//
// ??? what to do for emergency stop, keep refreshing ? keep alive checking ?
// ??? how do we integrate the STEAL/SHARE/DISPATCHED concept ?
//
// ??? separate out the check alive functionality ? it is a separate task...
// ??? sessionMapNextAliveCheck var needed ...
//
// ??? change to refreshCabEntry
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::refreshSessionEntry( SessionMapEntry *smePtr ) {

    // ??? introduce a return status ?

    if ( smePtr -> cabId != NIL_CAB_ID ) {

        if ( flags & SM_F_KEEP_ALIVE_CHECKING ) {

            if (( getMillis( ) - smePtr -> lastKeepAliveTime ) > 
                                            refreshAliveTimeOutVal ) {

                if (( debugMask & DBG_BS_CONFIG ) && 
                    ( debugMask & DBG_BS_CHECK_ALIVE_SESSIONS )) {

                    printf( "Session: %d expired\n", smePtr - sessionMap );
                }

                deallocateSessionEntry( smePtr );
            }
        }

        // ??? separate keep alive checking and refresh options...

        else {

            // ??? if ( smePtr -> speed > 0 )  // only active locos are refreshed...

            if ( smePtr -> nextRefreshStep == 0 ) {

                setThrottle( smePtr, smePtr -> speed, smePtr -> direction );

                smePtr -> nextRefreshStep = 
                    ((( smePtr -> flags & SME_COMBINED_REFRESH ) ||
                      ( smePtr -> flags & SME_SPDIR_REFRESH )) ? 0 : 1 );
            }
            else if ( smePtr -> nextRefreshStep <= 5 ) {

            uint8_t fGroup = smePtr -> nextRefreshStep;

            setDccFunctionGroup( smePtr, fGroup, smePtr -> functions[ fGroup - 1 ] );
            smePtr -> nextRefreshStep = (( smePtr -> nextRefreshStep >= 5 ) ? 
                                            0 : smePtr -> nextRefreshStep + 1 );
         
            }
        }    
    }
}

//----------------------------------------------------------------------------------------
// "emergencyStopAll" is called when one of the clients issued an emergency stop 
// all request. There is a DCC broadcast packet that causes all decoders to stop
// the locos. In addition, the base station is expected to discontinue sending
// non-zero speed packets until the situation is cleared. The standard does not 
// really say what exactly to do. In our base station, we will first issue the 
// ESTOP DCC broadcast packet and then set the speed value in each session to one,
// which is the value for emergency stop. All else is unchanged.
//
// ??? run through the can entries...
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::emergencyStopAll( ) {

    mainTrack -> loadPacket( eStopDccPacketData, 2, 4 );

    for ( SessionMapEntry *smePtr = sessionMap; smePtr < sessionMapHwm; smePtr++ ) {

        if ( smePtr -> cabId != NIL_CAB_ID ) smePtr -> speed = 1;
    }
}

//----------------------------------------------------------------------------------------
// Getter methods for session related info. Straightforward.
//
// ??? new concept - rework some calls....
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::getSessionIdByCabId( uint16_t cabId ) {

    SessionMapEntry *smePtr = lookupSessionEntry( cabId );
    return (( smePtr == nullptr ) ? 
             NIL_LOCO_SESSION_ID : (( smePtr - sessionMap ) + 1 ));
}

uint16_t LcsBaseStationLocoSession::getOptions( ) {

    return ( options );
}

uint16_t LcsBaseStationLocoSession::getFlags( ) {

    return ( flags );
}

uint8_t LcsBaseStationLocoSession::getSessionMapHwm( ) {

    return ( sessionMapHwm - sessionMap );
}

uint32_t LcsBaseStationLocoSession::getSessionKeepAliveInterval( ) {

    return ( refreshAliveTimeOutVal );
}

uint8_t LcsBaseStationLocoSession::getActiveSessions( ) {

    uint8_t sessionCnt = 0;

    for ( SessionMapEntry *smePtr = sessionMap; smePtr < sessionMapHwm; smePtr++ ) {

        if ( smePtr -> cabId != NIL_CAB_ID ) sessionCnt++;
    }

    return ( sessionCnt );
}

//----------------------------------------------------------------------------------------
// "setThrottle" is perhaps the most used function. After all, we want to run engines
// on the track. This signature will just locate the session map entry and then 
// invoke the internal signature with accepts a pointer to the entry.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::setThrottle( uint8_t sId, 
                                                uint8_t speed, 
                                                uint8_t direction ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    return ( setThrottle( smePtr, speed, direction ));
}

//----------------------------------------------------------------------------------------
// "setThrottle" will send a DCC packet with speed and direction for a loco. If
// the combined speed and function refresh option is enabled, the DCC command will
// specify speed, direction and functions to refresh in one packet.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::setThrottle( SessionMapEntry *smePtr, 
                                                uint8_t speed, 
                                                uint8_t direction ) {

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t pLen = 0;

    smePtr -> speed      = speed & 0x7F;
    smePtr -> direction  = direction % 2;

    if ( smePtr -> cabId > 127 ) pBuf[pLen++] = highByte( smePtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( smePtr -> cabId );

    pBuf[pLen++] = (( smePtr -> flags & SME_COMBINED_REFRESH ) ? 0x3c : 0x3F );
    pBuf[pLen++] = (( smePtr -> speed & 0x7F ) | (( smePtr -> direction ) ? 0x80 : 0 ));

    if ( smePtr -> flags & SME_COMBINED_REFRESH ) {

        pBuf[pLen++]  = ((( smePtr -> functions[0] & 0x10 ) >> 4 ) |
                        (( smePtr -> functions[0] & 0x0F ) << 1 ) |
                        (( smePtr -> functions[1] & 0x07 ) << 5 ));

        pBuf[pLen++]  = ((( smePtr -> functions[1] & 0x0F ) >> 3 ) |
                        (( smePtr -> functions[2] & 0x0F ) << 1 ) |
                        (( smePtr -> functions[3] & 0x07 ) << 5 ));

        pBuf[pLen++]  = ((( smePtr -> functions[3] & 0xf80 ) >> 3 ) |
                        (( smePtr -> functions[4] & 0x07 ) << 5 ));

        pBuf[pLen++]  = (( smePtr -> functions[4] & 0xf80 ) >> 3 );
    }

    mainTrack -> loadPacket( pBuf, pLen );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "setDccFunctionBit" controls the functions in a decoder. The DCC function flags
// F0 .. F68 are stored in ten groups. The routines first updates the function bit
// in the loco session entry data structure, so we can keep track of the values. 
// This is important as the DCC commands send out entire groups only. The actual 
// work is then done by the "setDccFunctionGroup" method.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::setDccFunctionBit( uint8_t sId, 
                                                      uint8_t fNum, 
                                                      uint8_t val ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    if ( ! validFunctionId( fNum )) return ( ERR_INVALID_FUNC_ID );
    setDccFuncBit( smePtr -> functions, fNum, val );

    uint8_t fGroup = dccFunctionBitToGroup( fNum );

    return ( setDccFunctionGroup( smePtr, fGroup, smePtr -> functions[ fGroup - 1 ] ));
}

//----------------------------------------------------------------------------------------
// "setDccFunctionGroup" sets an entire group of function flags. This signature 
// will first find the session entry, do the argument checks and the invoke the 
// internal signature.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::setDccFunctionGroup( uint8_t sId, 
                                                        uint8_t fGroup, 
                                                        uint8_t dccByte ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    return ( setDccFunctionGroup( smePtr, fGroup, dccByte ));
}

//----------------------------------------------------------------------------------------
// "setDccFunctionGroup" sets an entire group of function flags.The DCC function
// flags F0 .. F68 are stored in ten groups.
//
//      Group 1:  F0, F4, F3, F2, F1      DCC Command Format: 100DDDDD
//      Group 2:  F8, F7, F6, F5          DCC Command Format: 1011DDDD
//      Group 3:  F12, F11, F10, F9       DCC Command Format: 1010DDDD
//      Group 4:  F20 .. F13              DCC Command Format: 0xDE DDDDDDDD
//      Group 5:  F28 .. F21              DCC Command Format: 0xDF DDDDDDDD
//      Group 6:  F36 .. F29              DCC Command Format: 0xD8 DDDDDDDD
//      Group 7:  F44 .. F37              DCC Command Format: 0xD9 DDDDDDDD
//      Group 8:  F52 .. F45              DCC Command Format: 0xDA DDDDDDDD
//      Group 9:  F60 .. F53              DCC Command Format: 0xDB DDDDDDDD
//      Group 10: F68 .. F61              DCC Command Format: 0xDC DDDDDDDD
//
// The routines updates the entire function group byte in the loco session entry, 
// so we can keep track of the values. The function command is repeated 4 times to
// the track.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::setDccFunctionGroup( SessionMapEntry *smePtr, 
                                                        uint8_t fGroup, 
                                                        uint8_t dccByte ) {

    if ( ! validFunctionGroupId( fGroup )) return ( ERR_INVALID_FGROUP_ID );
    setDccFuncGroupByte( smePtr -> functions, fGroup, dccByte );

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE];
    uint8_t pLen = 0;

    if ( smePtr -> cabId > 127 ) pBuf[pLen++] = highByte( smePtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( smePtr -> cabId );

    switch ( fGroup - 1 ) {

        case 0: pBuf[pLen++] = ( smePtr -> functions[ 0 ] & 0x1F ) | 0x80; break;
        case 1: pBuf[pLen++] = ( smePtr -> functions[ 1 ] & 0x0F ) | 0xB0; break;
        case 2: pBuf[pLen++] = ( smePtr -> functions[ 2 ] & 0x0F ) | 0xA0; break;

        case 3: pBuf[pLen++] = 0xDE; pBuf[pLen++] = smePtr -> functions[ 3 ]; break;
        case 4: pBuf[pLen++] = 0xDF; pBuf[pLen++] = smePtr -> functions[ 4 ]; break;
        case 5: pBuf[pLen++] = 0xD8; pBuf[pLen++] = smePtr -> functions[ 5 ]; break;
        case 6: pBuf[pLen++] = 0xD9; pBuf[pLen++] = smePtr -> functions[ 6 ]; break;
        case 7: pBuf[pLen++] = 0xDA; pBuf[pLen++] = smePtr -> functions[ 7 ]; break;
        case 8: pBuf[pLen++] = 0xDB; pBuf[pLen++] = smePtr -> functions[ 8 ]; break;
        case 9: pBuf[pLen++] = 0xDC; pBuf[pLen++] = smePtr -> functions[ 9 ]; break;
    }

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCVMain" writes a CV value to the decoder on the main track. CV numbers 
// range from 1 to 1024, but are encoded from 0 to 1023. The DCC standard defines 
// various modes for retrieving CV values. This function implements CV write mode 
// mode 0 and 1, by calling the respective method. The other modes are not 
// supported. For bit mode access, the bit position and bit value are encoded in 
// the "val" parameter with bit 3 containing the data and bit 0 ..2 the bit offset.
//
//    0 Direct Byte
//    1 Direct Bit
//    2 Page Mode
//    3 Register Mode
//    4 Address Only Mode
//
//
// Note on the MAIN track, there is no way for the decoder to answer via a raise 
// in power consumption. The command shown here is just sent. If however RailCom 
// is available, the decoder can answer with the CV value in a following cutout. 
// This is currently not implemented.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeCVMain( uint8_t sId, 
                                                uint16_t cvId, 
                                                uint8_t mode, 
                                                uint8_t val ) {

    if ( mode == 0 )  
        return ( writeCVByteMain( sId, cvId, val ));
    else if ( mode == 1 )  
        return ( writeCVBitMain( sId, cvId, ( val & 0x07 ), (( val & 0x08 ) >> 3 )));
    else         
        return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "writeCVByteMain" writes a byte to the CV while the loco is on the main track. 
// The CV numbers range from 1 to 1024, but are encoded from 0 to 1023. This function
// implements CV write mode mode 0, which is write a byte at a time. There is no 
// way to validate our operation, only writes are possible. The packet is sent four 
// times.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeCVByteMain( uint8_t sId, 
                                                    uint16_t cvId, 
                                                    uint8_t val ) {

    uint8_t   pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t   pLen = 0;

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    if ( smePtr -> cabId > 127 ) pBuf[pLen++] = highByte( smePtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( smePtr -> cabId );
    pBuf[pLen++] = 0xEC + ( highByte( cvId ) & 0x03 );
    pBuf[pLen++] = lowByte( cvId );
    pBuf[pLen++] = val;

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCVBitMain" writes a bit to the CV while the loco is on the main track. 
// The CV numbers range from 1 to 1024, but are encoded from 0 to 1023. This 
// function implements CV write mode mode 1, which is write a bit at a time. On 
// input the "val" parameter encodes the bit position in bits 0 - 2 and the bit 
// value in bit 3. There is no way to validate our operation, only CV writes are 
// possible. The packet is sent four times.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeCVBitMain( uint8_t sId, 
                                                   uint16_t cvId, 
                                                   uint8_t bitPos, 
                                                   uint8_t val ) {

    SessionMapEntry *smePtr = getSessionMapEntryPtr( sId );
    if ( smePtr == nullptr ) return ( ERR_INVALID_SESSION_ID );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t pLen = 0;

    if ( smePtr -> cabId > 127 ) pBuf[pLen++] = highByte( smePtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( smePtr -> cabId );
    pBuf[pLen++] = 0xE8 + (highByte( cvId ) & 0x03 );
    pBuf[pLen++] = lowByte( cvId );
    pBuf[pLen++] = 0xF0 + (( val % 2 ) << 3 ) + ( bitPos % 8 );

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "readCV" retrieves a CV value from the decoder in service mode. CV numbers range
// from 1 to 1024, but are encoded from 0 to 1023. This command is only available
// in service mode, i.e. on a programming track. The DCC standard defines various 
// modes for retrieving CV values. We only support mode 0 and 1. The other modes
// are not supported. For bit mode access, the bit position and bit value are 
// encoded in the "val" parameter with bit 3 containing the data and bit 0 ..2 
// the bit offset.
//
//    0 - Direct Byte
//    1 - Direct Bit
//    2 - Page Mode
//    3 - Register Mode
//    4 - Address Only Mode
//
// This function implements the CV read mode 0 and 1, which is reading a byte or 
// a bit at a time by calling the respective method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::readCV( uint16_t cvId, uint8_t mode, uint8_t *val ) {

    if        ( mode == 0 )  return ( readCVByte( cvId, val ));
    else if   ( mode == 1 )  return ( readCVBit( cvId, *val % 8, val ));
    else                     return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "readCVByte" will retrieve a complete byte from the decoder. CV numbers range 
// from 1 to 1024, but are encoded from 0 to 1023. This command is only available 
// in service mode, i.e. on a programming track. Reading a CV value where the 
// decoder can only respond with a "yes" or "no" is a tedious matter. We are 
// actually reading the CV value bit by bit and then ask if the assembled byte 
// read is the one just read. The general packet sequence is a according to DCC 
// standard standard 3 or more RESET packets, 5 or more identical READ packets 
// and then RESET packages until acknowledge or timeout. The RESET packet preamble
// and postamble series are sent during the decoder ack setup and detect call to 
// the DCC track object. During the preamble we figure out the base current 
// consumption of the decoder, during the postamble packets we measure to get the
// decoder acknowledge, which is a short raise in power consumption to indicate 
//an ACK.
//
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs to
// be redesigned to use a kind of state machine that sends a packet at a time so 
// other work can interleave.
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::readCVByte( uint16_t cvId, uint8_t *val ) {

    if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );
    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t   pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t   bValue  = 0;
    uint16_t  base    = progTrack -> decoderAckBaseline( 5 );

    pBuf[0]  = 0x78 + ( highByte( cvId ) & 0x03 );
    pBuf[1]  = lowByte( cvId );

    for ( int i = 0; i < 8; i++ ) {

        pBuf[2] = 0xE8 + i;
        progTrack -> loadPacket( pBuf, 3, 5 );
        bitWrite( &bValue, i, progTrack -> decoderAckDetect( base, 9 ));
    }

    *val    = bValue;
    pBuf[0] = 0x74 + ( highByte( cvId ) & 0x03 );
    pBuf[1] = lowByte( cvId );
    pBuf[2] = bValue;
    progTrack -> loadPacket( pBuf, 3, 5 );

    return (( progTrack -> decoderAckDetect( base, 9 )) ? 
            LCS_OK : (LcsErrorCodes) ERR_CV_OP_FAILED );
}

//----------------------------------------------------------------------------------------
// "readCVBit" will retrieve one bit from a CV variable from the decoder. CV numbers
// range from 1 to 1024, but are encoded from 0 to 1023. This command is only 
// available in service mode, i.e. on a programming track. The "val" parameter 
// encodes the bit position in bits 0 - 2. We are reading the CV value bit and then
// ask if the bit read is the one just read. We first try to validate a zero bit. 
// If that succeeds, fine. Otherwise we try to validate a one bit. If that succeeds,
// fine. Otherwise we have a CV read error. The general packet sequence is 
// according to DCC standard 3 or more RESET packets, 5 or more identical READ 
// packets and then RESET packages until acknowledge or timeout. The RESET packet 
// preamble and postamble are sent during the decoder ack setup and detect call to
// the DCC track object. During the preamble we figure out the base current 
// consumption of the decoder, during the postamble we measure to get the decoder
// acknowledge, which is a short raise in power consumption to indicate an ACK.
//
// ??? This command may take a long time, a lot of packets are sent. While this not 
// an issue with the signal generation, which is done via interrupt handlers, it may
// be an issue with any other work of the base station. This code needs to be 
// redesigned to use a kind of state machine that sends a packet at a time so other
// work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::readCVBit( uint16_t cvId, 
                                              uint8_t bitPos, 
                                              uint8_t *val ) {

    if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t  pBuf[ MAX_DCC_PACKET_SIZE ];
    int   base = progTrack -> decoderAckBaseline( 5 );

    pBuf[0]  = 0x78 + ( highByte( cvId ) & 0x03 );
    pBuf[1]  = lowByte( cvId );
    pBuf[2]  = 0xE8 + ( bitPos % 8 );

    progTrack -> loadPacket( pBuf, 3, 5 );

    if ( ! ( progTrack -> decoderAckDetect( base, 9 ))) {
        
        pBuf[2] = 0xE8 + 8 + ( bitPos % 8 );
        progTrack -> loadPacket( pBuf, 3, 5 );
    
        if ( progTrack -> decoderAckDetect( base, 9 )) {

            *val = 1;
            return ( LCS_OK );
        }
        else return ( ERR_CV_OP_FAILED );
    }
    else return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCV" writes a CV value to the decoder. CV numbers range from 1 to 1024, but
// are encoded from 0 to 1023. This command is only available in service mode, i.e. 
// on a programming track. The DCC standard defines various modes for accessing CV
// values. For bit mode access, the bit position and bit value are encoded in
// the "val" parameter with bit 3 containing the data and bit 0 .. 2 the bit offset.
//
//    0 Direct Byte
//    1 Direct Bit
//    2 Page Mode
//    3 Register Mode
//    4 Address Only Mode
//
// This function implements the CV write mode 0 and 1, which is writing a byte or
// a bit at a time by calling the respective method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeCV( uint16_t cvId, 
                                            uint8_t mode, 
                                            uint8_t val ) {

    if ( mode == 0 ) 
        return ( writeCVByte( cvId, val ));
    else if ( mode == 1 )  
        return ( writeCVBit( cvId, ( val & 0x07 ), (( val & 0x08 ) >> 3 )));
    else                     
        return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "writeCVByte" puts a data byte into the CV on the decoder. This function is only
// available in service mode. The CV numbers range from 1 to 1024, but are encoded
// from 0 to 1023. The data byte written will also be verified. The packet sequence
// follows the DCC standard. We will send the CV byte write packet four times, send
// out several RESET packets and the send the verify packets to get the acknowledge
// from the decoder that the operation was successful.
//
// ??? This command may take a long time, a lot of packets are sent. While this not 
// an issue with the signal generation, which is done via interrupt handlers, it may
// be an issue with any other work of the base station. This code needs to be 
// redesigned to use a kind of state machine that sends a packet at a time so other
// work can interleave.
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeCVByte( uint16_t cvId, uint8_t val ) {

    if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    int     base = progTrack -> decoderAckBaseline( 5 );

    pBuf[0] = 0x7C + ( highByte( cvId ) & 0x03 );
    pBuf[1] = lowByte( cvId );
    pBuf[2] = val;

    progTrack -> loadPacket( pBuf, 3, 4 );
    progTrack -> loadPacket( resetDccPacketData, 2, 11 );

    pBuf[0] = 0x74 + ( highByte( cvId ) & 0x03 );
    progTrack -> loadPacket( pBuf, 3, 5 );

    return (( progTrack -> decoderAckDetect( base, 9 )) ?
                         LCS_OK : (LcsErrorCodes) ERR_CV_OP_FAILED );
}

//----------------------------------------------------------------------------------------
// "writeCVBit" puts a data bit into the CV on the decoder. This function is only
// available in session mode. The CV numbers range from 1 to 1024, but are encoded
// from 0 to 1023. For the bit mode,  the "val" parameter encodes the bit position
// in bits 0 - 2 and the bit value in bit 3. The packet sequence follows the DCC
// standard, similar to the byte write operation.
//
// ??? This command may take a long time, a lot of packets are sent. While this not 
// an issue with the signal generation, which is done via interrupt handlers, it may
// be an issue with any other work of the base station. This code needs to be 
// redesigned to use a kind of state machine that sends a packet at a time so other
// work can interleave.
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeCVBit( uint16_t cvId, 
                                               uint8_t bitPos, 
                                               uint8_t val ) {

    if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );
    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    int     base = progTrack -> decoderAckBaseline( 5 );

    pBuf[0] = 0x78 + ( highByte( cvId ) & 0x03 );
    pBuf[1] = lowByte( cvId );
    pBuf[2] = 0xF0 + (( val % 2 ) * 8 ) + ( bitPos % 8 );

    progTrack -> loadPacket( pBuf, 3, 4 );
    progTrack -> loadPacket( resetDccPacketData, 2, 11 );

    bitWrite( &pBuf[2], 4, false );
    progTrack -> loadPacket( pBuf, 3, 5 );

    return (( progTrack -> decoderAckDetect( base, 9 )) ? 
                        LCS_OK : (LcsErrorCodes) ERR_CV_OP_FAILED );
}

//----------------------------------------------------------------------------------------
// "writeDccPacketMain" just load the DCC packet into the buffer and out it goes 
// to the main track without any further checks.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeDccPacketMain( uint8_t *pBuf, 
                                                       uint8_t pLen, 
                                                       uint8_t nRepeat ) {

    if ( ! validDccPacketlen( pLen )) return ( ERR_INVALID_PACKET_LEN );
    if ( ! validDccPacketRepeatCnt( nRepeat )) return ( ERR_INVALID_REPEATS );

    mainTrack -> loadPacket( pBuf, pLen, nRepeat );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeDccPacketProg" just load the DCC packet into the buffer and out it goes
// to the programming track without any further checks.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationLocoSession::writeDccPacketProg( uint8_t *pBuf, 
                                                       uint8_t pLen,
                                                       uint8_t nRepeat ) {

    if ( ! validDccPacketlen( pLen )) return ( ERR_INVALID_PACKET_LEN );
    if ( ! validDccPacketRepeatCnt( nRepeat )) return ( ERR_INVALID_REPEATS );

    progTrack -> loadPacket( pBuf, pLen, nRepeat );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "allocateSessionEntry" allocates a new loco session entry and returns a pointer
// to the entry. We first check if there is already a session for the cabId and if
// so, we return a null pointer. If not, we try to find a free entry and if that 
// fails try to raise the high water mark. If that fails, we are out of luck and
// return a null pointer.
//
// ??? new concept, we don't have sessions...
// ??? allocation is done on the fly...
//----------------------------------------------------------------------------------------
SessionMapEntry* LcsBaseStationLocoSession::allocateSessionEntry( uint16_t cabId ) {

    if ( lookupSessionEntry( cabId ) != nullptr ) return ( nullptr );

    SessionMapEntry *freePtr = lookupSessionEntry( NIL_CAB_ID );

    if (( freePtr == nullptr ) && ( sessionMapHwm < sessionMapLimit )) 
        freePtr = sessionMapHwm ++;

    if ( freePtr != nullptr ) {

        initSessionEntry( freePtr );
        freePtr -> cabId  = cabId;
        freePtr -> flags  |= SME_ALLOCATED;

        if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_SESSION )) {

            printf( "Allocate session entry: %d, HWM: %d\n", 
                ( freePtr - sessionMap + 1 ), ( sessionMapHwm - sessionMap ));
        }
    }

    return ( freePtr );
}

//----------------------------------------------------------------------------------------
// "deallocateSessionEntry" is the counterpart to the entry allocation. We just 
// free up the entry. If the entry is at the high water mark, we try to free up all 
// possibly free entries from the high water mark downward, decrementing the high
// water mark. This way the high water mark shrinks again and we do not need to 
// work through unused entries in the middle.
//
// ??? new concept, we deallocate on the fly.
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::deallocateSessionEntry( SessionMapEntry *smePtr ) {

    if (( smePtr != nullptr ) && 
        ( smePtr >= sessionMap ) && 
        ( smePtr < sessionMapHwm )) {

        if ( smePtr == ( sessionMapHwm - 1 )) {

            do {

                initSessionEntry( smePtr );
                smePtr --;
            }
            while (( smePtr -> cabId == NIL_CAB_ID ) && ( smePtr >= sessionMap ));

            sessionMapHwm = smePtr + 1;
        }
        else initSessionEntry( smePtr );

       if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_SESSION )) {
        
            printf( "Released Session, sId: %d, ,new  HWM: %d\n", 
                ( smePtr - sessionMap + 1 ), ( sessionMapHwm - sessionMap ));
       }
    }
}

//----------------------------------------------------------------------------------------
// "lookupSessionEntry" scans the session map for a session entry for the cabId. 
// If none is found, a nullptr is returned. Note that a NIL_CAB_ID as argument is
// also a valid input and will return the first free entry.
//
// ??? new concept, not needed anymore...
//----------------------------------------------------------------------------------------
SessionMapEntry *LcsBaseStationLocoSession::lookupSessionEntry( uint16_t cabId ) {

    SessionMapEntry *smePtr = sessionMap;

    while ( smePtr < sessionMapHwm ) {

        if ( smePtr -> cabId == cabId ) return ( smePtr );
        else smePtr ++;
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// "initSessionEntry" initializes a session map entry with default values.
//
// ??? we keep this ... 
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::initSessionEntry( SessionMapEntry *smePtr ) {

    smePtr -> flags              = SME_DEFAULT_SETTING;
    smePtr -> cabId              = NIL_CAB_ID;
    smePtr -> speedSteps         = DCC_SPEED_STEPS_128;
    smePtr -> speed              = 0;
    smePtr -> direction          = 0;
    smePtr -> engineState        = 0;
    smePtr -> lastKeepAliveTime  = 0;
    smePtr -> nextRefreshStep    = 0;

    for ( int i = 0; i < MAX_DCC_FUNC_GROUP_ID; i++ ) smePtr -> functions[ i ] = 0;
}

//----------------------------------------------------------------------------------------
// "getSessionMapEntryPtr" returns a pointer to a valid and used sessionMap entry. 
// The sessionId starts with index 1.
//
// ??? new concept, we use cabId to get the entry...
//----------------------------------------------------------------------------------------
SessionMapEntry *LcsBaseStationLocoSession::getSessionMapEntryPtr( uint8_t sId ) {

    if ( ! isInRangeU( sId, MIN_LOCO_SESSION_ID, ( sessionMapHwm - sessionMap ))) 
        return ( nullptr );

    return (( sessionMap[ sId - 1 ].cabId == NIL_CAB_ID ) ? 
            nullptr : &sessionMap[ sId - 1 ] );
}

//----------------------------------------------------------------------------------------
// "printSessionMapConfig" lists cab session map configuration data.
//
// ??? what to print...
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::printSessionMapConfig( ) {

    printf( "Session Map Config\n" );
    printf( " Options: 0x%x\n", options );
    printf( " Session Map Size: %d\n", ( sessionMapLimit - sessionMap ));
}

//----------------------------------------------------------------------------------------
// "printSessionMapInfo" lists the cab session map data.
//
// ??? key to print, should we add "options" here ???
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::printSessionMapInfo( ) {

    printf( "Session Map Info\n" );
    printf( " Flags: 0x%x\n", flags );
    printf( " Session Map Hwm: %d\n", ( sessionMapHwm - sessionMap ));

    for ( SessionMapEntry *smePtr = sessionMap; smePtr < sessionMapHwm; smePtr ++ ) {

        if ( smePtr -> cabId != NIL_CAB_ID ) printSessionEntry( smePtr );
    }

     printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "printSessionEntry" lists a cab session.
//
// ??? revise ... but keep
//----------------------------------------------------------------------------------------
void LcsBaseStationLocoSession::printSessionEntry( SessionMapEntry *smePtr ) {

  if ( smePtr != nullptr ) {

    printf( " sId: %d, cabId: %d, speed: %d ", 
            ( smePtr - sessionMap + 1 ), smePtr -> cabId, smePtr -> speed );
   
    printf( "%s", (( smePtr -> direction ) ? "Rev" : "Fwd" ));
    printf( ", functions: " );

    for ( uint8_t i = 0; i < MAX_DCC_FUNC_GROUP_ID; i++ ) {

      printf( " 0x%x ", smePtr -> functions[ i ] );
    }

    printf( " Flags: 0x%x", ( smePtr -> flags ));
  }

  printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "cabIdToSessionId" searches the cab/session mapping table. The table is sorted
// by cabId, so we can use a binary search algorithm. If found, the sessionId is
// returned, otherwise -1.
//
// ??? goes away...
//----------------------------------------------------------------------------------------
int cabIdToSessionId(const LcsCabSessionMapTable *map, uint16_t cabId ) {

    int low  = 0;
    int high = map -> hwm - 1;

    while ( low <= high ) {

        int mid = (low + high) / 2;
        uint16_t k = map-> map[ mid ].cabId;

        if ( k == cabId ) return ( map -> map[ mid ].sessionId ); 

        if ( k < cabId ) low  = mid + 1;
        else             high = mid - 1;
    }

    return ( -1 );
}

//----------------------------------------------------------------------------------------
// "insertInCabSessionMap" inserts a new cab/session mapping entry into the table.
// The table is kept sorted by cabId, so we need to find the right location 
// first. If the cabId is already present, we return -2 as error code. If the
// table is full, we return -1 as error code. On success we return 0.
//
// ??? goes away... new routine...
//----------------------------------------------------------------------------------------
int insertInCabSessionMap( LcsCabSessionMapTable *map,
                           uint16_t cabId, 
                           uint8_t sessionId ) {

    if ( map -> hwm >= MAX_CAB_SESSIONS ) return ( -1 );

    int low       = 0;
    int high      = map -> hwm - 1;
    int insertPos = map->hwm;

    while (low <= high) {

        int      mid = (low + high) / 2;
        uint16_t k   = map-> map[mid].cabId;

        if ( k == cabId ) return ( -2 ); // duplicate 

        if ( k < cabId ) {

            low       = mid + 1;
            insertPos = low;

        } else {

            high      = mid - 1;
            insertPos = mid;
        }
    }

    // Shift to make room
    for ( int i = map -> hwm; i > insertPos; i-- ) {

        map->map[ i ] = map->map[ i - 1 ];
    }

    // Insert new entry
    map -> map[ insertPos].cabId     = cabId;
    map -> map[ insertPos].sessionId = sessionId;
    map -> hwm++;

    return 0;
}

//----------------------------------------------------------------------------------------
// "removeFromCabSessionMap" removes a cab/session mapping entry from the table.
// The table is kept sorted by cabId, so we first need to find the entry. If not
// found, we return -1 as error code. On success we return 0.
//
// ??? goes away... new routine...
//----------------------------------------------------------------------------------------
int removeFromCabSessionMap( LcsCabSessionMapTable *map, uint16_t cabId ) {

    // First find it
    int low  = 0;
    int high = map -> hwm - 1;
    int pos  = -1;

    while ( low <= high ) {

        int mid    = (low + high) / 2;
        uint16_t k = map -> map[ mid ].cabId;

        if ( k == cabId ) {

            pos = mid;
            break;
        }

        if ( k < cabId ) low  = mid + 1;
        else             high = mid - 1;
    }

    if ( pos < 0 ) return ( -1 ); // not found

    // Shift down
    for ( int i = pos; i < map-> hwm - 1; i++ ) {

        map->map[ i ] = map->map[ i + 1 ];
    }

    map-> hwm--;
    return 0;
}









// ??? how to start ? Add new routines for hashTab here ?
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "LcsCdcLib.h" // ??? not found at link time ...



//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
struct CabTableEntry {

    uint16_t cabId;
    uint32_t lastSeen;
};

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
struct CabTableIndexEntry {

    uint16_t cabId;
    uint16_t index;
};

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
namespace {

const int           MAX_CAB_ENTRIES = 256;
const int           MAX_CAB_HASH_TAB_ENTRIES = 512;   // must be power of two

uint16_t            cabTableHwm;
CabTableEntry       locoTable [ MAX_CAB_ENTRIES ];
CabTableIndexEntry  locoIndexTable [ MAX_CAB_HASH_TAB_ENTRIES ];


//----------------------------------------------------------------------------------------
// Hash function.
//
//----------------------------------------------------------------------------------------
static inline uint32_t hash( uint16_t x ) {

    x ^= x >> 7;
    x *= 0x9E37;
    x ^= x >> 9;
    return x;
}

//----------------------------------------------------------------------------------------
// Insert into hash table.
//
//----------------------------------------------------------------------------------------
void insertCabTableIndex( uint16_t cabId, uint16_t index ) {

    uint32_t i = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {
        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        if ( i == start ) return; // table full ( should not happen )
    }

    locoIndexTable [ i ].cabId = cabId;
    locoIndexTable [ i ].index = index;
}

//----------------------------------------------------------------------------------------
// Remove from hash table.
//
//----------------------------------------------------------------------------------------
void removeCabTableIndex( uint16_t cabId ) {

    uint32_t i      = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start  = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        if ( locoIndexTable [ i ].cabId == cabId ) break;
        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return;
    }

    if ( locoIndexTable [ i ].cabId == NIL_CAB_ID ) return;

    // remove
    locoIndexTable [ i ].cabId = NIL_CAB_ID;

    // reinsert cluster
    uint32_t j = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

    while ( locoIndexTable [ j ].cabId != NIL_CAB_ID ) {

        CabTableIndexEntry tmp = locoIndexTable [ j ];
        locoIndexTable [ j ].cabId = NIL_CAB_ID;

        uint32_t k = hash( tmp.cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        uint32_t kstart = k;

        while ( locoIndexTable [ k ].cabId != NIL_CAB_ID ) {

            k = ( k + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
            if ( k == kstart ) break;
        }

        locoIndexTable [ k ] = tmp;
        j = ( j + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    }
}

} // namespace

//----------------------------------------------------------------------------------------
// Setup hash table.
//
//----------------------------------------------------------------------------------------
void cabTableInit( ) {

    for ( int i = 0; i < MAX_CAB_HASH_TAB_ENTRIES; i++ ) {
        
        locoIndexTable [ i ].cabId = NIL_CAB_ID;
    }
    cabTableHwm = 0;
}

//----------------------------------------------------------------------------------------
// Lookup in hash table.
//
//----------------------------------------------------------------------------------------
CabTableEntry *lookupCab( uint16_t cabId ) {

    uint32_t i = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        if ( locoIndexTable [ i ].cabId == cabId )
            return &locoTable [ locoIndexTable [ i ].index ];

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return NULL;
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// Lookup cabId and add if not found. 
//
//----------------------------------------------------------------------------------------
CabTableEntry *lookupCabAndAdd( uint16_t cabId ) {

    if ( cabId == NIL_CAB_ID ) return ( nullptr );

    CabTableEntry *l = lookupCab( cabId );
    if ( l ) return l;

    if ( cabTableHwm >= MAX_CAB_ENTRIES ) return ( nullptr );

    uint16_t index = cabTableHwm++;
    locoTable [ index ].cabId = cabId;
    locoTable [ index ].lastSeen = CDC::getMillis( );

    insertCabTableIndex( cabId, index );
    return ( &locoTable [ index ] );
}

//----------------------------------------------------------------------------------------
// Remove a cabId.
//
//----------------------------------------------------------------------------------------
void removeCab( uint16_t cabId ) {

    CabTableEntry *l = lookupCab( cabId );
    if ( !l ) return;

    uint16_t index = ( uint16_t )( l - locoTable );
    uint16_t last = cabTableHwm - 1;

    removeCabTableIndex( cabId );

    if ( index != last ) {
        
        locoTable [ index ] = locoTable [ last ];

        uint16_t movedId = locoTable [ index ].cabId;
        uint32_t i       = hash( movedId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

            if ( locoIndexTable [ i ].cabId == movedId ) {

                locoIndexTable [ i ].index = index;
                break;
            }

            i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        }
    }

    cabTableHwm--;
}

//----------------------------------------------------------------------------------------
// Scan CabTable for expired cabs.
//
//----------------------------------------------------------------------------------------
void cabTableAge( uint32_t timeout ) {

    uint32_t now = CDC::getMillis( );

    for ( uint16_t i = 0; i < cabTableHwm; ) {

        if ( ( now - locoTable [ i ].lastSeen ) > timeout ) {

            removeCab( locoTable [ i ].cabId );

        } 
        else i++;
    }
}

//----------------------------------------------------------------------------------------
// List cabTable table.
//
//----------------------------------------------------------------------------------------
void dumpCabTable( ) {

    printf( "---- LOCOS ( %u ) ----\n", cabTableHwm );

    for ( uint16_t i = 0; i < cabTableHwm; i++ ) {

        printf( " [ %3u ] id=%5u last=%u\n",
               i,
               locoTable [ i ].cabId,
               locoTable [ i ].lastSeen );
    }
}

#if 0
//----------------------------------------------------------------------------------------
// Remove from hash table.
//
//----------------------------------------------------------------------------------------
/* ---------------- MAIN TEST ---------------- */

int main( void ) {

    cabTableInit( );

    for ( int i = 0; i < 20; i++ ) {

        for ( int j = 0; j < 3; j++ ) {

            uint16_t id = rand(  ) % 100;

            CabTableEntry *l = lookupCabAndAdd( id );
            if ( l ) l -> lastSeen = CDC::getMillis( );
        }

        CDC::sleepMillis( 2000 );

        cabTableAge( 1000 );

        printf( "\nTICK\n" );
        dumpCabTable( );
    }

    return 0;
}
#endif
