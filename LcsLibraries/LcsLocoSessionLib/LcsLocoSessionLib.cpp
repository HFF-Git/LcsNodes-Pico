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


}; // namespace

//----------------------------------------------------------------------------------------
// 
// ??? clear an entry
// ??? we could also lookup a dictionary to see what loco this might be
// ??? required to see if analog loco.
//----------------------------------------------------------------------------------------
void LcsLocoSessions::setupCabEntry( LcsCabEntry *entry, uint16_t cabId ) {

    // ??? to do ....
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

    uint16_t cabIndex;

    //------------------------------------------------------------------------------------
    // Look for an existing entry. If found, we move the entry to the top and 
    // shift all others down.
    //
    //------------------------------------------------------------------------------------
    for ( uint16_t i = 0; i < cabCount; i++ ) {

        cabIndex = cabIndexMap[ i ];

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
    // recently used one. For a new entry this shifts all existing entries one 
    // position down. For an overlay this also removes the old LRU entry from 
    // the map. Finally, setup the entry and return a pointer to it.
    //
    //------------------------------------------------------------------------------------
    if ( cabCount < MAX_CAB_SESSIONS ) {

        cabIndex = cabCount;
        cabCount ++;

    } else cabIndex = cabIndexMap[ MAX_CAB_SESSIONS - 1 ];

    for ( uint16_t j = cabCount - 1; j > 0; j-- ) {

        cabIndexMap[ j ] = cabIndexMap[ j - 1 ];
    }

    cabIndexMap[ 0 ] = cabIndex;

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

            for ( uint16_t j = i; j + 1 < cabCount; j++ ) {

                cabIndexMap[ j ] = cabIndexMap[ j + 1 ];
            }

            cabCount --;

            return;
        }
    }
}
