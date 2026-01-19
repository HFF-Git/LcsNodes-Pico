//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Control Logic
//
//----------------------------------------------------------------------------------------
//
//
//
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Control Logic
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
inline bool blockControlDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_BLOCK )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( blockControlDebugEnabled( )) {

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
LcsBlockControl::LcsBlockControl(  ) {

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControl::setupBlockControl( ) {

    // ??? setup the block control logic, e.g. get handles to turnout, signal and 
    // ??? detection objects.

    return ( RET_STAT( LCS_OK ));
}


// ??? there is one block control object per block.

