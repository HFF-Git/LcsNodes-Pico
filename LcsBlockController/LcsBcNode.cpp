//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Node Management
//
//----------------------------------------------------------------------------------------
//
//
//
// ??? is this the handler for node related stuff ?
//----------------------------------------------------------------------------------------
// 
// LCS Block Controller - Node Management
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
inline bool nodeDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG) && ( debugMask & DBG_BC_NODE )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( nodeDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// Some little helper functions.
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

    int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

    for ( int i = 0; i < msgLen; i++ ) printf( "0x%x ", msg[i] );
    printf( "\n" );
}

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
//
//
//----------------------------------------------------------------------------------------
LcsBlockNode::LcsBlockNode(  ) {

}

//----------------------------------------------------------------------------------------
// The initialization callback.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handleInitCallback( uint16_t npId, void *uData ) {

     if ( nodeDebugEnabled( )) {

        printf( "Init Callback, npId: 0x%4x\n", npId );
    }

    return( RET_STAT( NO_ERR ));
}

//----------------------------------------------------------------------------------------
// The node or port reset callback.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handleResetCallback( uint16_t npId, void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Reset Callback, npId: 0x%4x\n", npId );
    }

    return( RET_STAT( NO_ERR ));
}

//----------------------------------------------------------------------------------------
// The power fail callback.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handlePfailCallback( uint16_t npId, void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Power Fail Callback, npId: 0x%4x\n", npId );
    }

    return( RET_STAT( NO_ERR ));
}

//----------------------------------------------------------------------------------------
// LCS message callback.
// 
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handleMsgCallback( uint8_t *msg, void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "MsgCallback, msg:\n" );
        printLcsMsg( msg );
    }

    return( RET_STAT( NO_ERR ));
}

//----------------------------------------------------------------------------------------
// Node request callback.
// 
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handleReqCallback( uint16_t npId, 
                                         uint8_t item, 
                                         uint16_t *arg1, 
                                         uint16_t *arg2,
                                         void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "REQ callback: npId: 0x%x, item: %d", npId, item );

        if ( arg1 != nullptr ) printf( ", arg1: %d ", *arg1 ); 
        else printf( ", arg1: null" );

        if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); 
        else printf( ", arg2: null" );
        printf( "\n" );
    }


    return( RET_STAT( NO_ERR ));
}

//---------------------------------------------------------------------------------------
// Node reply callback.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handleRepCallback( uint16_t npId, 
                                         uint8_t item, 
                                         uint16_t arg1, 
                                         uint16_t arg2, 
                                         uint8_t ret,
                                         void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "REP callback: npId: 0x%x, item: %d, ", npId, item );
        printf( "arg1: %d, arg2: %d, ret: %d\n", arg1, arg2, ret );
    }


    return( RET_STAT( NO_ERR ));
}

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::handleEventCallback( uint16_t npId, 
                                           uint16_t eId, 
                                           uint8_t eAction, 
                                           uint16_t eData,
                                           void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", 
                npId, eId, eAction, eData );
    }

    
    return( RET_STAT( NO_ERR ));
}
