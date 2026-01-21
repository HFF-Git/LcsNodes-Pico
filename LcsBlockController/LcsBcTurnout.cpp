//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Turnout Control
//
//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Turnout Control
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
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

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
inline bool turnoutsDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG) && ( debugMask & DBG_BC_TURNOUTS )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( turnoutsDebugEnabled( )) {

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
LcsTurnoutControl::LcsTurnoutControl( ) {

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsTurnoutControl::setupTurnoutControl( uint16_t extBoardId ) {

    if ( turnoutsDebugEnabled( )) printf( "setupTurnoutControl\n" );

    
    return( RET_STAT( NO_ERR ));
}

// ??? contains the routines that manage the turnout settings

// ??? this is a call to the servo extension board. 
// ??? do we need a different board when we have the turnout sockets board ?

// ??? the functions would be to just set the turnout, and perhaps get a status back.