//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Block Control
//
//----------------------------------------------------------------------------------------
//
//
//
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Block Control
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
#include "LcsBlockController.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// Debug support routines. We can easily check whether debug is enabled at all. 
// The return status routines will print out a return status message when 
// debugging is enabled. The macro "RET_STAT" is a nice helper that adds the
// function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool blockDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_BLOCK )); 
}

inline uint8_t retStatBlock( char *name, uint8_t errId ) {

    if ( blockDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStatBlock((char *) __func__, ( x ))


//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------





} // namespace



// ??? there is one block control object per block.

// ??? need to find out how we best handle the debug mask facility when there
// are different types that can be debugged....


// ??? we need to be aware that a block can be on the same node or on another
// node. This needs to be handled transparently. 
//
// ??? easy said, harder done. A remote request is typically  REQ message and 
// later a REP message received. 
//
// ??? in order to avoid code duplication, we need to mimic this scheme while
// at the same time avoid using the LCS bus.
//
// ??? exception are events, which also need to be broadcasted to the LCS bus.
//
// ??? furthermore, the extension boards are available to all blocks on this node.
// ??? we may not need a separate object and rather use the driver calls ( REQ )
// directly. 
//
// ??? external requests are available to control an element such a turnout, which 
// must be translated to a driver request. 
//
// ??? in short, we model turnouts and signals, but just use the driver calls 
// to talk to the hardware. 

// ??? need some code to debounce the OCC detect data. The driver returns raw
// data sampled.

// ??? node some code to talk to a turnout. The idea is that this object has the
// methods to make these calls and invoke the driver library code.

// ??? not clear yet, where the RailCom code fits in. It is driven by the CUTPUT
// interrupt event. 



//========================================================================================
//========================================================================================
//
// Block Section.
//
//========================================================================================
//========================================================================================
//
//
//----------------------------------------------------------------------------------------
LcsBlockControl::LcsBlockControl(  ) {

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControl::setupBlockControl( ) {

    // ??? we need to get the HW resources so we can pass them to the track
    // object ...

    // ??? setup the block control logic, e.g. get handles to turnout, signal and 
    // ??? detection objects.

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// Each block is managed by the block state machine.
//
// 
//----------------------------------------------------------------------------------------
void LcsBlockControl::runBlockStateMachine( ) { 


}

//----------------------------------------------------------------------------------------
//
//
// ??? temp items to start testing ...
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControl::handleReqCallback( uint16_t npId, 
                                            uint8_t item, 
                                            uint16_t *arg1,
                                            uint16_t *arg2, 
                                            void *uData ) {

    if ( blockDebugEnabled( )) {

        printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    
        if ( arg1 != nullptr ) printf( ", arg1: %d ", *arg1 ); 
        else printf( ", arg1: null" );
    
        if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); 
        else printf( ", arg2: null" );
        printf( "\n" );
    }

    switch( item ) {

        case 64: {

            track -> setTrackModeSpeed( *arg1 & 0xFF, *arg2 & 0xFF );

        } break;

        case 65: {

            track -> setPwmFrequency( *arg1 );
            
        } break;

        default: {

        }
    }

    return ( RET_STAT( NO_ERR ));
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControl::handleRepCallback( uint16_t npId, 
                                            uint8_t item, 
                                            uint16_t arg1,
                                            uint16_t arg2, 
                                            uint8_t ret,
                                            void *uData ) {

    if ( blockDebugEnabled( )) {

        printf( "REP callback: npId: 0x%x, item: %d, ", npId, item );
        printf( "arg1: %d, arg2: %d, ret: %d ", arg1, arg2, ret );
    }


    return ( RET_STAT( NO_ERR ));
}



//========================================================================================
//========================================================================================
//
// Track Section.
//
//========================================================================================
//========================================================================================



//========================================================================================
//========================================================================================
//
// Occupancy Section.
//
//========================================================================================
//========================================================================================



//========================================================================================
//========================================================================================
//
// Turnout Section.
//
//========================================================================================
//========================================================================================



//========================================================================================
//========================================================================================
//
// Signal Section.
//
//========================================================================================
//========================================================================================

