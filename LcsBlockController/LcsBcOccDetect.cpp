//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Occupancy Detect
//
//----------------------------------------------------------------------------------------
//
// ??? contains the routines that manage the track section occupancy detection
//
// we need a handler to be called periodically to access the extension board and 
// get the occ mask.
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Occupancy Detect
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
inline bool occDetectDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG) && ( debugMask & DBG_BC_OCCUPANCY )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( occDetectDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
const int DEBOUNCE_MS                   = 30;
const int OCC_MASK_BITS                 = sizeof( uint16_t );

uint16_t debouncedMask                  = 0;
uint16_t lastMask                       = 0;
uint32_t stableTs[ OCC_MASK_BITS ]      = { 0 };

//----------------------------------------------------------------------------------------
// The occupancy detector delivers the state of all sections the extension
// board handles. We need to make sure that glitches are not leading to an 
// overreaction. There is for each big in the bit mask a timestamp that records
// since when we have a stable signal. Only of the signal is stable for a certain 
// period of time, will we update the result bit mask that is used for block
// section occupancy status.
//
//----------------------------------------------------------------------------------------
void debounceUpdate( uint16_t raw_mask ) {

    uint32_t now = getMillis();

    for ( int bit = 0; bit < OCC_MASK_BITS; bit++ ) {

        uint16_t bit_mask = (1u << bit);

        bool raw_bit  = (( raw_mask & bit_mask ) != 0 );
        bool prev_bit = (( lastMask & bit_mask ) != 0 );
        bool deb_bit  = (( debouncedMask & bit_mask ) != 0 );

        if (raw_bit == prev_bit) {
            
            if (( now - stableTs[bit] ) >= DEBOUNCE_MS ) {

                if ( raw_bit != deb_bit ) {

                    if ( raw_bit )  debouncedMask |= bit_mask;
                    else            debouncedMask &= ~bit_mask;
                }
            }
        }
        else stableTs[bit] = now;
    }

    lastMask = raw_mask;
}

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
LcsOccDetect::LcsOccDetect( ) { 

}

//----------------------------------------------------------------------------------------
//
// ??? initialize the attributes
// ??? how can we figure out what is connected ?
// ??? need to remember where the boards are exactly connected.
//----------------------------------------------------------------------------------------
 uint8_t LcsOccDetect::setupOccDetect( uint16_t extBoardId ) {

    if ( occDetectDebugEnabled( )) printf( "Setup OCC detect\n" );


    return( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// The state machine will periodically read the actual section occupancy bits
// from the extension board, run the debounce unit, and update the node attributes. 
//
//----------------------------------------------------------------------------------------
void LcsOccDetect::runOccDetectStateMachine( ) {

    // ??? get the values 

    uint16_t rawMask = 0;

    debounceUpdate( rawMask );

    // ??? update the attribute values...

}

//----------------------------------------------------------------------------------------
//
// ??? not sure this is necessary. We might as well directly update the 
// attributes...
//----------------------------------------------------------------------------------------
uint8_t LcsOccDetect::getOccDetectMask( uint16_t *mask ) {

    // ?? what if we have two boards ?

    *mask = debouncedMask;
    return( RET_STAT( LCS_OK ));
}