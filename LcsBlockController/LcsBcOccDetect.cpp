//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Occupancy Detect
//
//----------------------------------------------------------------------------------------
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
// ??? contains the routines that manage the track section occupancy detection

// we need a handler to be called periodically to access the extension board and 
// get the occ mask.

// ??? reading is simple LCS attribute get from the OCC port. 

// ??? we would however also have a mechanism that makes sure that debounce the
// data, to avoid false alarms... 

// ??? should that be part of the driver or here ?

// ??? of all we do is to get the OCC data, we might as well do it in the 
// BLOCK object...