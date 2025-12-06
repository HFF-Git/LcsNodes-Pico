//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld Cab Stack implementation file
//
//----------------------------------------------------------------------------------------
// The cab stack of cab entries and the current cab are the central structure to 
// manage the engines known to the cab handheld. Most of cab handheld functions 
// apply to the current cab. The current cab can be saved and restored form a stack
// of cab entries. Finally, engine descriptions can be loaded from and stored to a
// central place such as the base station.
//
//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld Cab Stack implementation file
// Copyright (C) 2022 - 2025  Helmut Fieres
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
#include "LcsBasicThrottle.h"

using namespace LCS;

//----------------------------------------------------------------------------------------
// Externals.
//
//----------------------------------------------------------------------------------------
extern CabStack  *cabStack;

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {


}; // namespace

//----------------------------------------------------------------------------------------
// "setupCabStack" will create the local cab table and read in the entries from
// the NVM area.
//
// ??? optional: we could read from the base station any changes to the loco
// attributes... ?
//----------------------------------------------------------------------------------------
uint8_t setupCabStack( ) {

    cabStack = new CabStack( );
    cabStack -> loadCabSlotsFromNVM( );

    cabStack -> printCabSlots( );

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
// Cab Stack Entry Methods.
//
//----------------------------------------------------------------------------------------
// The cab entry is the central structure to describe a cab. It contains the engine
// address, its speed and direction, all the function settings and some other flags
// and data. The structure is organized in a set of 16 bit aligned data fields to 
// allow for an easy transfer of items to and from a basestation.
//
//----------------------------------------------------------------------------------------
void CabEntry::reset( uint16_t cabId ) {

    flags             = 0;
    this -> cabId     = cabId;

    keepAliveTimer    = 0;

    for ( int i = 0; i < MAX_LOCO_NAME_SIZE; i++ )      engineName[ i ]       = 0;
    for ( int i = 0; i < MAX_FUNC_STATE_SIZE; i++ )     dccFuncState[ i ]     = 0;
    for ( int i = 0; i < MAX_CAB_FUNC_ENTRIES; i++ )    cabFuncIdMap[ i ]     = 0;
}

uint8_t CabEntry::dccSpeedAndDirectionByte( ) {

    return (( speed & 0x7F ) | (( direction & 0x1 ) << 7 ));
}

//----------------------------------------------------------------------------------------
// The DCC function bitmap getter/setter functions. The function ID starts with 
// zero to 68.
//
//----------------------------------------------------------------------------------------
bool CabEntry::getDccFuncState( uint8_t fNum ) {

    return (( isInRange( fNum, MIN_DCC_FUNC_ID, MAX_DCC_FUNC_ID )) ?
            (( dccFuncState[ fNum / 8 ] >> ( 7 - ( fNum & 0x7 )) & 0x1 )) : false );
}

void CabEntry::setDccFuncState( uint8_t fNum, bool val ) {

    if ( isInRange( fNum, MIN_DCC_FUNC_ID, MAX_DCC_FUNC_ID )) {

        if ( val )  dccFuncState[ fNum / 8 ] |= 1 << ( 7 - ( fNum & 0x7 ));
        else        dccFuncState[ fNum / 8 ] &= ( ~ ( 1 << ( 7 - ( fNum & 0x7 ))));
    }
}

void CabEntry::toggleDccFuncState( uint8_t fNum ) {

    setDccFuncState( fNum, ( ! getDccFuncState( fNum )));
}

//----------------------------------------------------------------------------------------
// Getter/Setter for the cab handheld function ID assigned DCC function Id. The
// cab handheld function ID starts with one.
//
//----------------------------------------------------------------------------------------
uint8_t CabEntry::getDccFuncIdFromMap( uint8_t cNum ) {

    return (( isInRange( cNum, MIN_DCC_F_M, MAX_DCC_F_M )) ? 
                                     cabFuncIdMap[ cNum - 1 ] : 0 );
}

void CabEntry::setDccFuncIdInMap( uint8_t cNum, uint8_t fNum, uint8_t typ ) {

    if ( isInRange( cNum, MIN_DCC_F_M, MAX_DCC_F_M )) {

        cabFuncIdMap[ cNum - 1 ] = ( typ << 6 ) | ( fNum & 0x3F );
    }
}

//----------------------------------------------------------------------------------------
// "mapRnumToCabFuncId" is a help function to select the Cab Function based on the 
// function button resource number and the current function set selected.
// 
//----------------------------------------------------------------------------------------
uint8_t CabEntry::mapRnumToCabFuncId( uint8_t rNum, uint8_t functionSet ) {

    switch( rNum ) {

        case RNUM_F1_BUTTON: {

             if ( functionSet == 2 ) return ( DCC_F_M_F5 ); 
             else                    return ( DCC_F_M_F1 );
        
        } break;

        case RNUM_F2_BUTTON: {

             if ( functionSet == 2 ) return ( DCC_F_M_F6 ); 
             else                    return ( DCC_F_M_F2 );
        
        } break;

        case RNUM_F3_BUTTON: {

             if ( functionSet == 2 ) return ( DCC_F_M_F7 ); 
             else                    return ( DCC_F_M_F3 );
        
        } break;

        case RNUM_F4_BUTTON: {

             if ( functionSet == 2 ) return ( DCC_F_M_F8 ); 
             else                    return ( DCC_F_M_F4 );
        
        } break;

        default: return ( DCC_F_M_F1 );
    }
}

//----------------------------------------------------------------------------------------
// More getter/Setter methods.
//
//----------------------------------------------------------------------------------------
uint8_t CabEntry::getCabId( ) {

    return ( cabId );
}

void CabEntry::setCabId( uint16_t arg ) {

    cabId = arg;
}

uint8_t CabEntry::getEngineType( ) {

    return ( engineType );
}

char CabEntry::getEngineTypeChar( ) {

    switch ( engineType ) {

        case LOC_T_NIL:        return ( '-' );
        case LOC_T_STEAM:      return ( 'S' );
        case LOC_T_DIESEL:     return ( 'D' );
        case LOC_T_ELECTRIC:   return ( 'E' );
        default:                    return ( '-' );
    }
}

void CabEntry::setEngineType( uint8_t type ) {

    this -> engineType = type;
}

uint8_t CabEntry::getSpeed( ) {

    return ( speed );
}

void CabEntry::setSpeed( int speed ) {

    this -> speed = speed;
}

uint8_t CabEntry::getDirection( ) {

    return ( direction );
}

void CabEntry::setDirection( uint8_t direction ) {

    this -> direction = direction;
}

uint8_t CabEntry::getSessionId( ) {

    return ( sessionId );
}

void  CabEntry::setSessionId( uint8_t id ) {

    sessionId = id;
}

uint8_t CabEntry::getSessionState( ) {

    return ( sessionState );
}

void  CabEntry::setSessionState( uint8_t state ) {

    sessionState = state;
}

//----------------------------------------------------------------------------------------
// A cab entry needs to be set and saved from a central structure kept at for example 
// the base station. The methods below allow to get the data from the cab entry in 
// the defined number of 16-bit words.
//
//----------------------------------------------------------------------------------------
uint8_t CabEntry::getDataByItem( uint8_t item, uint16_t *arg ) {

    // ??? a big case statement to return the data by item

    return ( 0 );
}

uint8_t CabEntry::setDataByItem( uint8_t item, uint16_t arg ) {

    // ??? a big case statement to return the data by item

    return ( 0 );
}

//----------------------------------------------------------------------------------------
// A little helper function for debugging.
//
//----------------------------------------------------------------------------------------
void CabEntry::printCabEntry( ) {

    printf( "Cab Id: %d, %c, sId: %d, sState: %d, speed: %d, dir: %d \n", 
    cabId, getEngineTypeChar( ), sessionId, sessionState, speed, direction );

    printf( "DccFunc: " );

    for ( int i = 0; i < MAX_FUNC_STATE_SIZE; i++ ) {

        printf( "0x%x ", dccFuncState[ i ] );
    }

    printf( "\n" );
    printf( "CabFunc Id: " );

    for ( int i = 0; i < MAX_CAB_FUNC_ENTRIES; i++ ) {

        printf( "%d: 0x%x ", cabFuncIdMap[ i ] );
    }

    printf( "\n\n" );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
// Cab Stack Methods.
//
//----------------------------------------------------------------------------------------
// A cab handheld can manage a set of cab entries. One entry is the active cab, 
// the others are in a list of cab entries. The slots are numbered from one to MAX.
// Each entry describe an engine and its specific settings. The cabEntry stack is
// restored from the NVM at handheld start. An entry is updated when we change the
// config data for a cab.
//
//----------------------------------------------------------------------------------------
CabStack::CabStack( ) {

    cabSlots = (CabEntry *) calloc( MAX_CAB_LIST_ENTRIES, sizeof( CabEntry ));
    for ( int i = 0; i < MAX_CAB_LIST_ENTRIES; i++ ) cabSlots[ i ].reset( );

    currentCab.reset( );
}

void CabStack::loadCurrentCabFromSlot( int index ) {

    if (( index > 0 ) && ( index <= MAX_CAB_LIST_ENTRIES )) {

        currentCab = cabSlots[ index - 1 ];
    }
}

void CabStack::storeCurrentCabToSlot( int index ) {

    if (( index > 0 ) && ( index <= MAX_CAB_LIST_ENTRIES )) {

        for ( int i = 0; i < MAX_CAB_LIST_ENTRIES; i++ ) {

            if ( currentCab.getCabId( ) == 
                        cabSlots[ i ].getCabId( ))  cabSlots[ i ].reset( );
            }

        cabSlots[ index - 1 ] = currentCab;
    }
}

uint8_t CabStack::loadCabSlotsFromNVM( ) {

    // ??? to do ...

    return( LCS_OK );
}

uint8_t CabStack::updateCabSlotInNVM( int index ) {

    if (( index > 0 ) && ( index <= MAX_CAB_LIST_ENTRIES )) {

        // ??? to do ...
    }

  return( LCS_OK );
}

uint8_t CabStack::getMaxEntries( ) {

    return( MAX_CAB_LIST_ENTRIES );
}

void CabStack::printCabSlots( ) {

    for ( int i = 0; i < MAX_CAB_LIST_ENTRIES; i++ ) {

        cabSlots[ i ].printCabEntry( );
    }
}
