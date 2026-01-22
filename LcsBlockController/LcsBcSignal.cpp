//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Signal Control
//
//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Signal Control
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
inline bool signalDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG) && ( debugMask & DBG_BC_SIGNALS )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( signalDebugEnabled( )) {

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
LcsSignalControl::LcsSignalControl( ) {

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsSignalControl::setupSignalControl( uint16_t extBoardId ) {

    if ( signalDebugEnabled( )) printf( "setupTurnoutControl\n" );

    
    return( RET_STAT( NO_ERR ));
}

// ??? not clear what extension type this actually is, perhaps a GPIO board.
// ??? we just set...