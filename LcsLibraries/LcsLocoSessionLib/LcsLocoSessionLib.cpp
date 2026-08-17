///---------------------------------------------------------------------------------------
//
// LCS - Loco Session Lib
//
///---------------------------------------------------------------------------------------
// This source file contains ...
//
///---------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
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
#include "LcsLocoSessionLib.h"

// ??? abandon the idea of an explicitly allocated  session
// ??? need to think how we still do refresh work
// ??? refresh should also move the entry to the top ?
// ??? be lazy in deallocating
// ??? keep the current commands for speed, function, etc. 
// ??? keep the CV programming stuff


///---------------------------------------------------------------------------------------
// Local name space.
//
///---------------------------------------------------------------------------------------
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
// Constant values definition. We need the RESET and IDLE packet as well as a bit
// mask for a quick bit select in the data byte.
//
//----------------------------------------------------------------------------------------
const uint8_t   idleDccPacketData[ ]    = { 0xFF, 0x00 };
const uint8_t   resetDccPacketData[ ]   = { 0x00, 0x00 };
const uint8_t   eStopDccPacketData[ ]   = { 0x00, 0x01 };

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

//----------------------------------------------------------------------------------------
// "LocoSession" constructor. Nothing to do here.
//
//----------------------------------------------------------------------------------------
LcsLocoSessions::LcsLocoSessions( ) { }

//----------------------------------------------------------------------------------------
// "setupLocoSessions" gets the show on the road.
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setupLocoSessions( uint16_t options, 
                                            LcsDccTrack *mainTrack,
                                            LcsDccTrack *progTrack ) {

    if (( mainTrack == nullptr ) || ( progTrack == nullptr )) {

        return ( ERR_SESSION_SETUP );
    } 
        
    this -> options                 = options;
    this -> mainTrack               = mainTrack;
    this -> progTrack               = progTrack;
    this -> flags                   = SM_F_DEFAULT_SETTING;
    this -> lastAliveCheckTime      = getMillis( );
    this -> refreshAliveTimeOutVal  = DCC_SESSION_TIMEOUT_MILLIS;
    this -> cabCount                = 0;

    if ( options & SM_OPT_ENABLE_REFRESH )      flags |= SM_F_ENABLE_REFRESH;
    if ( options & SM_OPT_KEEP_ALIVE_CHECKING ) flags |= SM_F_KEEP_ALIVE_CHECKING;

    for ( int i = 0; i < MAX_CAB_SESSIONS; i++ ) {

        cabIndexMap[ i ] = 0;
        setupCabEntry( &cabMap[ i ] );
    }

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
//
//
// ??? linkage to a dictionary ?
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setupCabEntry( LcsCabEntry *cPtr ) {

    cPtr -> flags              = SME_DEFAULT_SETTING;
    cPtr -> cabId               = NIL_CAB_ID;
    cPtr -> speed               = 0;
    cPtr -> speedSteps          = 128;
    cPtr -> direction           = 0;
    cPtr -> engineState         = 0;
    cPtr -> nextRefreshStep     = 0;
    cPtr -> lastKeepAliveTime   = 0;
    cPtr -> functions[ MAX_DCC_FUNC_GROUP_ID ] = { 0 };

    return ( LCS_OK );
}


//----------------------------------------------------------------------------------------
// Getter methods for session related info. Straightforward.
//
//----------------------------------------------------------------------------------------
uint16_t LcsLocoSessions::getOptions( ) {

    return ( options );
}

uint16_t LcsLocoSessions::getFlags( ) {

    return ( flags );
}

uint16_t LcsLocoSessions::getCabCount( ) {

    return ( cabCount );
}

//----------------------------------------------------------------------------------------
// ??? intended for external code to lookup an entry ... can return not found!
// ??? needed, what is the harm to allocate on tHE FLY ?
//
//----------------------------------------------------------------------------------------
LcsCabEntry *LcsLocoSessions::lookupCabEntry( uint16_t cabId ) {

     for ( uint16_t i = 0; i < cabCount; i++ ) {

        uint16_t cabIndex = cabIndexMap[ i ];

        if ( cabMap[ cabIndex ].cabId == cabId ) {

            if ( i != 0 ) {

                for ( uint16_t j = i; j > 0; j-- ) {

                    cabIndexMap[ j ] = cabIndexMap[ j - 1 ];
                }

                cabIndexMap[ 0 ] = cabIndex;
            }

            return( &cabMap[ cabIndex ] );
        }
    }

    return( nullptr );
}

//----------------------------------------------------------------------------------------
// "allocateCabEntry" returns the cab entry for the cabId. If the cab already
// exists, it is moved to position zero in the index map, making it the most
// recently used entry.
//
// If the cab does not exist, a new physical entry is allocated when space is
// available. Otherwise the least recently used entry is overlaid. In both cases
// the returned entry is at position zero in the index map.
//
//----------------------------------------------------------------------------------------
LcsCabEntry *LcsLocoSessions::allocateCabEntry( uint16_t cabId ) {

    if ( !validCabId( cabId )) return( nullptr );

    //------------------------------------------------------------------------------------
    // Look for an existing entry. If found, move the entry to the top and
    // shift all others down.
    //
    //------------------------------------------------------------------------------------
    for ( uint16_t i = 0; i < cabCount; i++ ) {

        uint16_t cabIndex = cabIndexMap[ i ];

        if ( cabMap[ cabIndex ].cabId == cabId ) {

            if ( i != 0 ) {

                for ( uint16_t j = i; j > 0; j-- ) {

                    cabIndexMap[ j ] = cabIndexMap[ j - 1 ];
                }

                cabIndexMap[ 0 ] = cabIndex;
            }

            return( &cabMap[ cabIndex ] );
        }
    }

    //------------------------------------------------------------------------------------
    // Cab was not found. Allocate a physical entry and make this entry the most
    // recently used one. If there is still room in the table, find an unused 
    // physical entry. If the table is full, reuse the least recently used entry.
    //
    //------------------------------------------------------------------------------------
    uint16_t cabIndex;

    if ( cabCount < MAX_CAB_SESSIONS ) {

        // Find a free physical entry.
        for ( cabIndex = 0; cabIndex < MAX_CAB_SESSIONS; cabIndex++ ) {

            if ( cabMap[ cabIndex ].cabId == NIL_CAB_ID ) break;
        }

        cabMap[ cabIndex ].cabId = cabId;
        cabCount++;

    } else {

        // Table is full. Reuse the least recently used entry.
        cabIndex = cabIndexMap[ MAX_CAB_SESSIONS - 1 ];
    }

    //------------------------------------------------------------------------------------
    // Make this entry the most recently used one. For a new entry this shifts
    // all existing entries one position down. For an overlay this removes the
    // old LRU entry from the map.
    //
    //------------------------------------------------------------------------------------
    for ( uint16_t j = cabCount - 1; j > 0; j-- ) {

        cabIndexMap[ j ] = cabIndexMap[ j - 1 ];
    }

    cabIndexMap[ 0 ] = cabIndex;

    //------------------------------------------------------------------------------------
    // Initialize the physical entry.
    //
    //------------------------------------------------------------------------------------
    LcsCabEntry *ptr = &cabMap[ cabIndex ];

    setupCabEntry( ptr, cabId );
    return( ptr );
}


//----------------------------------------------------------------------------------------
// "deallocateCabEntry" removes a cab entry from the map. Any entry above the
// entry to remove is moved one place down and cabCount adjusted.
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::deallocateCabEntry( uint16_t cabId ) {

    for ( uint16_t i = 0; i < cabCount; i++ ) {

        uint16_t cabIndex = cabIndexMap[ i ];

        if ( cabMap[ cabIndex ].cabId == cabId ) {

            // Remove the entry from the MRU map.
            for ( uint16_t j = i; j + 1 < cabCount; j++ ) {

                cabIndexMap[ j ] = cabIndexMap[ j + 1 ];
            }

            // The physical entry is now free.
            cabMap[ cabIndex ].cabId = NIL_CAB_ID;
            cabCount--;
            return;
        }
    }
}

//----------------------------------------------------------------------------------------
// "emergencyStopAll" is called when one of the clients issued an emergency stop 
// all request. There is a DCC broadcast packet that causes all decoders to stop
// the locos. In addition, the base station is expected to discontinue sending
// non-zero speed packets until the situation is cleared. The standard does not 
// really say what exactly to do. In our base station, we will first issue the 
// ESTOP DCC broadcast packet and then set the speed value in each session to 
// one, which is the value for emergency stop. All else is unchanged.
//
// 
//----------------------------------------------------------------------------------------
void LcsLocoSessions::emergencyStopAll( ) {

    mainTrack -> loadPacket( eStopDccPacketData, 2, 4 );

    // for ( SessionMapEntry *smePtr = sessionMap; smePtr < sessionMapHwm; smePtr++ ) {
    //
    //    if ( smePtr -> cabId != NIL_CAB_ID ) smePtr -> speed = 1;
    // }
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::refreshActiveSessions( ) {

    return( 0 );
}

//----------------------------------------------------------------------------------------
// "markCabAlive" will set the cab active timestamp. It is used by the base 
// station code to record the last time it touched the session. A caBId not
// used in a certain time, is a candidate for removal from the active list.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::markCabAlive( uint16_t cabId ) {

     LcsCabEntry *cPtr = allocateCabEntry( cabId );                                    
    if ( cPtr == nullptr ) return( ERR_INVALID_CAB_ID );

    cPtr -> lastKeepAliveTime = getMillis( );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "setThrottle" is perhaps the most used function. After all, we want to run 
// engines on the track. This signature will just locate the session map entry 
// and then invoke the internal signature with accepts a pointer to the entry.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setThrottle( uint16_t cabId, 
                                      uint8_t speed, 
                                      uint8_t direction ) {

    LcsCabEntry *ptr = allocateCabEntry( cabId );                                    
    if ( ptr == nullptr ) return( ERR_INVALID_CAB_ID );

    return ( setThrottle( ptr, speed, direction ));
}

//----------------------------------------------------------------------------------------
// "setThrottle" will send a DCC packet with speed and direction for a loco. If
// the combined speed and function refresh option is enabled, the DCC command 
// will specify speed, direction and functions to refresh in one packet.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setThrottle( LcsCabEntry *cPtr, 
                                      uint8_t speed, 
                                      uint8_t direction ) {

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t pLen = 0;

    cPtr -> speed      = speed & 0x7F;
    cPtr -> direction  = direction % 2;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );

    pBuf[pLen++] = (( cPtr -> flags & SME_COMBINED_REFRESH ) ? 0x3c : 0x3F );
    pBuf[pLen++] = (( cPtr -> speed & 0x7F ) | (( cPtr -> direction ) ? 0x80 : 0 ));

    if ( cPtr -> flags & SME_COMBINED_REFRESH ) {

        pBuf[pLen++]  = ((( cPtr -> functions[0] & 0x10 ) >> 4 ) |
                        (( cPtr -> functions[0] & 0x0F ) << 1 ) |
                        (( cPtr -> functions[1] & 0x07 ) << 5 ));

        pBuf[pLen++]  = ((( cPtr -> functions[1] & 0x0F ) >> 3 ) |
                        (( cPtr -> functions[2] & 0x0F ) << 1 ) |
                        (( cPtr -> functions[3] & 0x07 ) << 5 ));

        pBuf[pLen++]  = ((( cPtr -> functions[3] & 0xf80 ) >> 3 ) |
                        (( cPtr -> functions[4] & 0x07 ) << 5 ));

        pBuf[pLen++]  = (( cPtr -> functions[4] & 0xf80 ) >> 3 );
    }

    mainTrack -> loadPacket( pBuf, pLen );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "setDccFunctionBit" controls the functions in a decoder. The DCC function 
// flags F0 .. F68 are stored in ten groups. The routines first updates the 
// function bit in the loco session entry data structure, so we can keep track 
// of the values. This is important as the DCC commands send out entire groups 
// only. The actual work is then done by the "setDccFunctionGroup" method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setDccFunctionBit( uint16_t cabId, 
                                            uint8_t fNum, 
                                            uint8_t val ) {

    LcsCabEntry *ptr = allocateCabEntry( cabId );                                    
    if ( ptr == nullptr ) return( ERR_INVALID_CAB_ID );

    if ( ! validFunctionId( fNum )) return ( ERR_INVALID_FUNC_ID );
    setDccFuncBit( ptr -> functions, fNum, val );

    uint8_t fGroup = dccFunctionBitToGroup( fNum );

    return ( setDccFunctionGroup( ptr, fGroup, ptr -> functions[ fGroup - 1 ] ));
}

//----------------------------------------------------------------------------------------
// "setDccFunctionGroup" sets an entire group of function flags. This signature 
// will first find the session entry, do the argument checks and the invoke the 
// internal signature.
//
// ??? change to lookup by cabId
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setDccFunctionGroup( uint16_t cabId, 
                                              uint8_t fGroup, 
                                              uint8_t dccByte ) {

   LcsCabEntry *ptr = allocateCabEntry( cabId );                                    
    if ( ptr == nullptr ) return( ERR_INVALID_CAB_ID );

    return ( setDccFunctionGroup( ptr, fGroup, dccByte ));
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
uint8_t LcsLocoSessions::setDccFunctionGroup( LcsCabEntry *cPtr, 
                                              uint8_t fGroup, 
                                              uint8_t dccByte ) {

    if ( ! validFunctionGroupId( fGroup )) return ( ERR_INVALID_FGROUP_ID );
    setDccFuncGroupByte( cPtr -> functions, fGroup, dccByte );

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE];
    uint8_t pLen = 0;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );

    switch ( fGroup - 1 ) {

        case 0: pBuf[pLen++] = ( cPtr -> functions[ 0 ] & 0x1F ) | 0x80; break;
        case 1: pBuf[pLen++] = ( cPtr -> functions[ 1 ] & 0x0F ) | 0xB0; break;
        case 2: pBuf[pLen++] = ( cPtr -> functions[ 2 ] & 0x0F ) | 0xA0; break;

        case 3: pBuf[pLen++] = 0xDE; pBuf[pLen++] = cPtr -> functions[ 3 ]; break;
        case 4: pBuf[pLen++] = 0xDF; pBuf[pLen++] = cPtr -> functions[ 4 ]; break;
        case 5: pBuf[pLen++] = 0xD8; pBuf[pLen++] = cPtr -> functions[ 5 ]; break;
        case 6: pBuf[pLen++] = 0xD9; pBuf[pLen++] = cPtr -> functions[ 6 ]; break;
        case 7: pBuf[pLen++] = 0xDA; pBuf[pLen++] = cPtr -> functions[ 7 ]; break;
        case 8: pBuf[pLen++] = 0xDB; pBuf[pLen++] = cPtr -> functions[ 8 ]; break;
        case 9: pBuf[pLen++] = 0xDC; pBuf[pLen++] = cPtr -> functions[ 9 ]; break;
    }

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCVMain" writes a CV value to the decoder on the main track. CV numbers 
// range from 1 to 1024, but are encoded from 0 to 1023. The DCC standard defines 
// various modes for retrieving CV values. This function implements CV write mode 
// mode 0 and 1, by calling the respective method. The other modes are not 
// supported. For bit mode access, the bit position and bit value are encoded 
// in the "val" parameter with bit 3 containing the data and bit 0 ..2 the bit 
// offset.
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
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVMain( uint16_t cabId, 
                                      uint16_t cvId, 
                                      uint8_t mode, 
                                      uint8_t val ) {

    if ( mode == 0 )  
        return ( writeCVByteMain( cabId, cvId, val ));
    else if ( mode == 1 )  
        return ( writeCVBitMain( cabId, cvId, ( val & 0x07 ), (( val & 0x08 ) >> 3 )));
    else         
        return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "writeCVByteMain" writes a byte to the CV while the loco is on the main track. 
// The CV numbers range from 1 to 1024, but are encoded from 0 to 1023. This 
// function implements CV write mode mode 0, which is write a byte at a time. 
// There is no way to validate our operation, only writes are possible. The 
// packet is sent four times.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVByteMain( uint16_t cabId, 
                                          uint16_t cvId, 
                                          uint8_t val ) {

    uint8_t   pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t   pLen = 0;

    LcsCabEntry *cPtr = allocateCabEntry( cabId );                                    
    if ( cPtr == nullptr ) return( ERR_INVALID_CAB_ID );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );
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
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVBitMain( uint16_t cabId, 
                                         uint16_t cvId, 
                                         uint8_t bitPos, 
                                         uint8_t val ) {

    LcsCabEntry *cPtr = allocateCabEntry( cabId );                                    
    if ( cPtr == nullptr ) return( ERR_INVALID_CAB_ID );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t pLen = 0;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );
    pBuf[pLen++] = 0xE8 + (highByte( cvId ) & 0x03 );
    pBuf[pLen++] = lowByte( cvId );
    pBuf[pLen++] = 0xF0 + (( val % 2 ) << 3 ) + ( bitPos % 8 );

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "readCV" retrieves a CV value from the decoder in service mode. CV numbers 
// range from 1 to 1024, but are encoded from 0 to 1023. This command is only 
// available in service mode, i.e. on a programming track. The DCC standard 
// defines various modes for retrieving CV values. We only support mode 0 and 1.
// The other modes are not supported. For bit mode access, the bit position and 
// bit value are encoded in the "val" parameter with bit 3 containing the data 
// and bit 0 ..2 the bit offset.
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
uint8_t LcsLocoSessions::readCV( uint16_t cvId, uint8_t mode, uint8_t *val ) {

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
// consumption of the decoder, during the postamble packets we measure to get 
// the decoder acknowledge, which is a short raise in power consumption to 
// indicate an ACK.
//
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs 
// to be redesigned to use a kind of state machine that sends a packet at a time
// so other work can interleave.
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::readCVByte( uint16_t cvId, uint8_t *val ) {

    // ???
    // if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );
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
// "readCVBit" will retrieve one bit from a CV variable from the decoder. CV 
// numbers range from 1 to 1024, but are encoded from 0 to 1023. This command 
// is only available in service mode, i.e. on a programming track. The "val" 
// parameter encodes the bit position in bits 0 - 2. We are reading the CV value
// bit and then ask if the bit read is the one just read. We first try to 
// validate a "zero" bit. If that succeeds, fine. Otherwise we try to validate a 
// "one" bit. If that succeeds, fine. Otherwise we have a CV read error. 

// The general packet sequence is according to DCC standard 3 or more RESET 
// packets, 5 or more identical READ packets and then RESET packages until 
// acknowledge or timeout. The RESET packet preamble and postamble are sent 
// during the decoder ack setup and detect call to the DCC track object. During 
// the preamble we figure out the base current consumption of the decoder, 
// during the postamble we measure to get the decoder acknowledge, which is a 
// short raise in power consumption to indicate an ACK.
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs 
// to be redesigned to use a kind of state machine that sends a packet at a 
// time so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::readCVBit( uint16_t cvId, 
                                    uint8_t bitPos, 
                                    uint8_t *val ) {

    // ???
    // if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );

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
// "writeCV" writes a CV value to the decoder. CV numbers range from 1 to 1024,
// but are encoded from 0 to 1023. This command is only available in service 
// mode, i.e. on a programming track. The DCC standard defines various modes for
// accessing CV values. For bit mode access, the bit position and bit value are
// encoded in the "val" parameter with bit 3 containing the data and bit 0 .. 2
// the bit offset.
//
//    0 Direct Byte
//    1 Direct Bit
//    2 Page Mode
//    3 Register Mode
//    4 Address Only Mode
//
// This function implements the CV write mode 0 and 1, which is writing a byte
// or a bit at a time by calling the respective method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCV( uint16_t cvId, uint8_t mode, uint8_t val ) {

    if ( mode == 0 ) 
        return ( writeCVByte( cvId, val ));
    else if ( mode == 1 )  
        return ( writeCVBit( cvId, ( val & 0x07 ), (( val & 0x08 ) >> 3 )));
    else                     
        return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "writeCVByte" puts a data byte into the CV on the decoder. This function is 
// only available in service mode. The CV numbers range from 1 to 1024, but are
// encoded from 0 to 1023. The data byte written will also be verified. The 
// packet sequence follows the DCC standard. We will send the CV byte write 
// packet four times, send out several RESET packets and the send the verify 
// packets to get the acknowledge from the decoder that the operation was 
// successful.
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs 
// to be redesigned to use a kind of state machine that sends a packet at a time
// so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVByte( uint16_t cvId, uint8_t val ) {

    // ???
    // if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );

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
// "writeCVBit" puts a data bit into the CV on the decoder. This function is 
// only available in session mode. The CV numbers range from 1 to 1024, but are
// encoded from 0 to 1023. For the bit mode,  the "val" parameter encodes the
// bit position in bits 0 - 2 and the bit value in bit 3. The packet sequence 
// follows the DCC standard, similar to the byte write operation.
//
// ??? This command may take a long time, a lot of packets are sent. While this
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs
// to be redesigned to use a kind of state machine that sends a packet at a time
// so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVBit( uint16_t cvId, 
                                     uint8_t bitPos, 
                                     uint8_t val ) {

    // ???
    // if ( ! ( progTrack -> isServiceModeOn( ))) return ( ERR_NO_SVC_MODE );
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
//
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::printConfig( ) {

    printf( "LocoSessions Config\n" );
    printf( " Options: 0x%x\n", options );
   
    // ??? to do ...


}

//----------------------------------------------------------------------------------------
// "printCabInfo" lists a cab entry. 
//
// ??? what is a good line format ?
//----------------------------------------------------------------------------------------
void  LcsLocoSessions::printCabInfo( LcsCabEntry *cPtr ) {

    if ( cPtr == nullptr ) return;

    printf( "CabId: %d,  speed: %d, ", cPtr -> cabId, cPtr -> speed );
    printf( "%s, ", (( cPtr -> direction ) ? "Rev" : "Fwd" ));
   
    printf( "Functions: " );

    for ( uint8_t i = 0; i < MAX_DCC_FUNC_GROUP_ID; i++ ) {

      printf( " 0x%x ", cPtr -> functions[ i ] );
    }

    printf( " Flags: 0x%x", ( cPtr -> flags ));
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void  LcsLocoSessions::printCabMap( ) {

     // ??? to do ...
    printf( "Active Cabs: \n" );


}





#if 0

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

#endif