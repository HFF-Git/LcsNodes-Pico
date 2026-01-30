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
// External declaration to global structures and routines in other files.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

inline bool nodeDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_NODE )); 
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
// Class part.
//
//========================================================================================
// As you cannot easily register a callback to an object method, we need these
// static wrapper functions that get the object pointer from the uData pointer.
// The register functions will register these static functions as callbacks and
// pass the object pointer as uData. The following static functions will get the
// object pointer from the uData pointer and call the actual object method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::nodeInitCallback( uint16_t npId, void *uData ) {

    if ( uData  != nullptr ) {

        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> initCallbackHandler( npId, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

uint8_t LcsBlockNode::nodePfailCallback( uint16_t npId, void *uData )  {

    if ( uData  != nullptr ) {
        
        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> pfailCallbackHandler( npId, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

uint8_t LcsBlockNode::nodeLcsMsgCallback( uint8_t *msg, void *uData )  {

    if ( uData  != nullptr ) {
        
        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> lcsMsgCallbackHandler( msg, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

uint8_t LcsBlockNode::nodeDccMsgCallback( uint8_t *msg, void *uData )  {

    if ( uData  != nullptr ) {
        
        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> dccMsgCallbackHandler( msg, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

uint8_t LcsBlockNode:: nodeReqCallback( uint16_t npId, 
                                  uint8_t item, 
                                  uint16_t *arg1, 
                                  uint16_t *arg2, 
                                  void *uData ) {
                                    
    if ( uData  != nullptr ) {
        
        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> reqCallbackHandler( npId, item, arg1, arg2, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

uint8_t LcsBlockNode::nodeRepCallback( uint16_t npId, 
                                  uint8_t item, 
                                  uint16_t arg1, 
                                  uint16_t arg2, 
                                  uint8_t ret, 
                                  void *uData ) {
                                    
    if ( uData  != nullptr ) {
        
        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> repCallbackHandler( npId, item, arg1, arg2, ret, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

uint8_t LcsBlockNode:: nodeEventCallback( uint16_t npId, 
                                    uint16_t eId, 
                                    uint8_t eAction, 
                                    uint16_t eData, 
                                    void *uData ) {

    if ( uData  != nullptr ) {
        
        LcsBlockNode *node = reinterpret_cast< LcsBlockNode * >( uData );
        return( node -> eventCallbackHandler( npId, eId, eAction, eData, uData ));
    }
    else return( ERR_CALLBACK_NOT_REGISTERED );
}

//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
// Object constructor and destructor.
//
//----------------------------------------------------------------------------------------
LcsBlockNode::LcsBlockNode( ) {

}

LcsBlockNode:: ~ LcsBlockNode( ) {

    delete occDetect;
    delete turnouts;
    delete signals;

    for ( int i = 0; i < blockHwm; i++ ) {

        if ( blocks[ i ] != nullptr ) delete blocks[ i ];
    }
}

//----------------------------------------------------------------------------------------
// Setup the block node with the resource map. The node is the central object that
// manages the block controller functionality. It creates the necessary block
// objects according to the resource map and registers the necessary callbacks.
//
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::setupBlockNode( CdcResourceDescMap *dMap ) {

    if ( nodeDebugEnabled( )) {

        printf( "Setup Block Node\n" );
    }

    this -> dMap = dMap;

    uint8_t rStat = LCS_OK;

    occDetect   = new LcsOccDetect( );
    turnouts    = new LcsTurnoutControl( );
    signals     = new LcsSignalControl( );

    registerInitCallback( LcsBlockNode::nodeInitCallback, this );
    registerPfailCallback( LcsBlockNode::nodePfailCallback, this );
    registerLcsMsgCallback( LcsBlockNode::nodeLcsMsgCallback, this );
    registerDccMsgCallback( LcsBlockNode::nodeDccMsgCallback, this );

    // ??? what do we do if we don't have any of the extensions  ?
    
    // ??? let the node in general handle the requests/events and
    // pass them to the extensions if they exist ?

    occDetect  -> setupOccDetect( 0 );      // fix: boardId
    turnouts   -> setupTurnoutControl( 0 ); // fix: boardId
    signals    -> setupSignalControl( 0 );  // fix: boardId

    // registering the OCC, T and S need port number.

    // Setup block objects according to resource map.

    for ( int i = 0; i < blockHwm; i++ ) {
        
        blocks[ i ] ->setupBlockControl( ); // fix: what needs to be passed ?

        // ??? register callbacks or let the node decode and invoke ??
    }

   
    return( RET_STAT( NO_ERR ));
}


//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockNode::initCallbackHandler( uint16_t npId, void *uData ) {

     if ( nodeDebugEnabled( )) {

        printf( "Node Init CallBack, npId: 0x%4x\n", npId );
    }


    return( RET_STAT( NO_ERR ));
}

uint8_t  LcsBlockNode::pfailCallbackHandler( uint16_t npId, void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Node PFail CallBack, npId: 0x%4x\n", npId );
    }   

    return( RET_STAT( NO_ERR ));
}

    
uint8_t  LcsBlockNode::lcsMsgCallbackHandler( uint8_t *msg, void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Node Msg CallBack: " );
        printLcsMsg( msg );
    }

    return( RET_STAT( NO_ERR ));
}

uint8_t  LcsBlockNode::dccMsgCallbackHandler( uint8_t *msg, void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Node Msg CallBack: " );
        printLcsMsg( msg );
    }

    return( RET_STAT( NO_ERR ));
}

uint8_t LcsBlockNode::reqCallbackHandler( uint16_t npId, 
                                  uint8_t item, 
                                  uint16_t *arg1, 
                                  uint16_t *arg2, 
                                  void *uData ) {

    if ( nodeDebugEnabled( )) {

        printf( "Node Req CallBack, npId: 0x%4x, item: %d, arg1: %d, arg2: %d\n", 
                npId, item, *arg1, *arg2 );
    }

    return( RET_STAT( NO_ERR ));
}

uint8_t LcsBlockNode::repCallbackHandler( uint16_t npId, 
                                  uint8_t item, 
                                  uint16_t arg1, 
                                  uint16_t arg2, 
                                  uint8_t ret, 
                                  void *uData ) {

    if ( nodeDebugEnabled( )) {
        printf( "Node Rep CallBack, npId: 0x%4x, item: %d, arg1: %d, arg2: %d, ret: %d\n", 
                npId, item, arg1, arg2, ret );
    }

    return( RET_STAT( NO_ERR ));
}

uint8_t  LcsBlockNode::eventCallbackHandler( uint16_t npId, 
                                    uint16_t eId, 
                                    uint8_t eAction, 
                                    uint16_t eData, 
                                    void *uData ) {


    if ( nodeDebugEnabled( )) {
        printf( "Node Event CallBack, npId: 0x%4x, eId: %d, eAction: %d, eData: %d\n", 
                npId, eId, eAction, eData );
    }

    return( RET_STAT( NO_ERR ));
}

