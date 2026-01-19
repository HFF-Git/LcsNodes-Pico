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

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

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
//
//
//----------------------------------------------------------------------------------------

const int DEBOUNCE_MS = 30;
const int NUM_BITS    = sizeof( uint16_t );



uint16_t debouncedMask     = 0;
uint16_t lastMask      = 0;
uint32_t stableSince[NUM_BITS];


} // namespace

//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
//========================================================================================
using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
LcsOccDetect::LcsOccDetect( ) { 

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
 uint8_t LcsOccDetect::setupOccDetect( uint16_t extBoardId ) {


    return( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsOccDetect::getOccDetectMask( uint16_t *mask ) {

    // ?? what if we have two boards ?

    // ??? update the LCS attributes ?
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void LcsOccDetect::runOccDetectStateMachine( ) {


}

// ??? read the mask every n ticks ?



//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
void debounce_update(uint16_t raw_mask)
{
    uint32_t now = getMillis();

    for (int bit = 0; bit < NUM_BITS; bit++) {
        uint16_t bit_mask = (1u << bit);

        bool raw_bit  = (raw_mask & bit_mask) != 0;
        bool prev_bit = (lastMask & bit_mask) != 0;
        bool deb_bit  = (debouncedMask & bit_mask) != 0;

        /* If raw bit changed, reset timer */
        if (raw_bit != prev_bit) {
            stableSince[bit] = now;
        }
        else {
            /* Raw bit is stable – check debounce time */
            if ((now - stableSince[bit]) >= DEBOUNCE_MS) {
                if (raw_bit != deb_bit) {
                    if (raw_bit)
                        debouncedMask |= bit_mask;
                    else
                        debouncedMask &= ~bit_mask;
                }
            }
        }
    }

    lastMask = raw_mask;

    // ??? update attribute with debouncedMask...
}

