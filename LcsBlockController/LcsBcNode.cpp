//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Control Logic
//
//----------------------------------------------------------------------------------------
//
//
//
// ??? contains the main code, the setup, the message handler, etc.
//----------------------------------------------------------------------------------------
// 
// LCS Block Controller
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

namespace {

//----------------------------------------------------------------------------------------
// Some little helper functions.
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

  int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

  for ( int i = 0; i < msgLen; i++ ) printf( "0x%x ", msg[i] );
  printf( "\n" );
}

}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
LcsBlockControllerNode::LcsBlockControllerNode(  ) {


}

//----------------------------------------------------------------------------------------------------------
// The node and port initialization callback.
//
// ??? when we know what ports we actually need / use, disable the rest of the ports.
// ??? the number of ports / blocks should be note in the block descriptor.
// ??? invoke the configured block reset method in the block controller logic object...
//----------------------------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handleInitCallback( uint16_t npId ) {

    switch ( npId & 0xF ) {

        case 0:     printf( "Node Init Callback: 0x%x\n", npId >> 4     ); break;
        default:    printf( "Port Init Callback: 0x%x\n", npId &  0xF   );
    } 

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------------------------
// The node or port reset callback.
//
// ??? invoke the configured block reset method in the block controller logic object...
//----------------------------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handleResetCallback( uint16_t npId ) {

    switch ( npId & 0xF ) {

        case 0:     printf( "Node Reset Callback: 0x%x\n", npId >> 4     ); break;
        default:    printf( "Port Reset Callback: 0x%x\n", npId &  0xF   );
    } 

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------------------------
// The node or port power fail callback.
//
// ??? invoke the configured block pfail method in the block controller logic object...
//----------------------------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handlePfailCallback( uint16_t npId ) {

    switch ( npId & 0xF ) {

        case 0:     printf( "Node Power Fail Callback: 0x%x\n", npId >> 4     ); break;
        default:    printf( "Port Power Fail Callback: 0x%x\n", npId &  0xF   );
    } 

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------------------------
// LCS message callbacks. All we do is to list their invocation. ( for now )
//
// 
//----------------------------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handleLcsMsgCallback( uint8_t *msg ) {

    printf( "MsgCallback: ", msg  );
    printLcsMsg( msg );
    return( LCS_OK );
}


// ??? quick hack for testing. A node should only deal with the block controller 
// object, which in turn has the track object locally.

extern LcsBlockTrack *block1;
extern LcsBlockTrack *block2;


//----------------------------------------------------------------------------------------
// When the base station node receives a request with an item defined in the user
// item range or the base station itself issues such a request, the defined callback
// is invoked.
//
// ??? should dispatch to the respective objects...
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handleLcsReqCallback( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    
    if ( arg1 != nullptr ) printf( ", arg1: %d ", *arg1 ); 
    else printf( ", arg1: null" );
    
    if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); 
    else printf( ", arg2: null" );
    printf( "\n" );

    switch( item ) {

        case 64: {

            uint16_t port = npId & 0xF;

            if      ( port == 1 ) block1 -> setTrackMode( *arg1 & 0xFF, *arg2 & 0xFF );
            else if ( port == 2 ) block2 -> setTrackMode( *arg1 & 0xFF, *arg2 & 0xFF );

        } break;

        case 65: {

            uint16_t port = npId & 0xF;

            if      ( port == 1 ) block1 -> setPwmFrequency( *arg1 );
            else if ( port == 2 ) block2 -> setPwmFrequency( *arg1 );
        
        } break;

        default: {

        }
    }

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// When the base station gets a reply message for a request previously sent, this
// callback is invoked.
//
// ??? pass to the block that requested...
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handleLcsRepCallback( uint16_t npId, 
                                                      uint8_t item, 
                                                      uint16_t arg1, 
                                                      uint16_t arg2, 
                                                      uint8_t ret ) {

    printf( "REP callback: npId: 0x%x, item: %d, arg1: %d, arg2: %d, ret: %d ", 
            npId, item , arg1, arg2, ret );
    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// For any event on the LCS system that the block controller is interested in, this
// callback is invoked.
//
// ??? what events to listen to ? where are they configured/set ?
//----------------------------------------------------------------------------------------
uint8_t LcsBlockControllerNode::handleLcsEventCallback( uint16_t npId, 
                                                        uint16_t eId, 
                                                        uint8_t eAction, 
                                                        uint16_t eData ) {

    printf( "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", 
            npId, eId, eAction, eData );
    return( LCS_OK );
}

