//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld LCS Bus interface implementation file
//
//----------------------------------------------------------------------------------------
// This class is the CAN bus message interface to the LCS bus.
//
//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld LCS Bus interface implementation file
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
#include "LcsBasicThrottle.h"

using namespace LCS;

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

    int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

    for ( int i = 0; i < msgLen; i++ ) printf( "0x%x ", msg[i] );
    printf( "\n" );
}

}; // nameSpace

//----------------------------------------------------------------------------------------
// Externals.
//
//----------------------------------------------------------------------------------------
extern CabMsgBus *msgBus;

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t setupMsgBus( ) {

    msgBus = new CabMsgBus( );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// LCS Core library callback functions, place holders for testing.
//
// ??? do we actually have anything we want to do here ? If not, take out ...
//----------------------------------------------------------------------------------------
uint8_t initCallback ( uint16_t npId, uint16_t flags ) {

    printf( "initCallback -> npId: 0x%x\n", npId );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
// ??? GET callback
//----------------------------------------------------------------------------------------
uint8_t nodeGetCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "nodeGetCallback -> npId: 0x%x, item: %d, ", npId, item );
    if ( arg1 != nullptr ) printf( "%d", *arg1 ); else printf( "null" );
    printf( ":" );
    if ( arg2 != nullptr ) printf( "%d", *arg2 ); else printf( "null" );
    printf( "\n" );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
// ??? GET callback
//----------------------------------------------------------------------------------------
uint8_t nodePutCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "nodePutCallback -> npId: 0x%x, item: %d, ", npId, item );
    if ( arg1 != nullptr ) printf( "%d", *arg1 ); else printf( "null" );
    printf( ":" );
    if ( arg2 != nullptr ) printf( "%d", *arg2 ); else printf( "null" );
    printf( "\n" );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
// ??? REQ callback
//----------------------------------------------------------------------------------------
uint8_t nodeReqCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "nodeReqCallback -> npId: 0x%x, item: %d, ", npId, item );
    if ( arg1 != nullptr ) printf( "%d", *arg1 ); else printf( "null" );
    printf( ":" );
    if ( arg2 != nullptr ) printf( "%d", *arg2 ); else printf( "null" );
    printf( "\n" );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
// ??? REP callback
//----------------------------------------------------------------------------------------
uint8_t nodeRepCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "nodeRepCallback -> npId: 0x%x, item: %d, ", npId, item );
    if ( arg1 != nullptr ) printf( "%d", *arg1 ); else printf( "null" );
    printf( ":" );
    if ( arg2 != nullptr ) printf( "%d", *arg2 ); else printf( "null" );
    printf( "\n" );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void eventCallback( uint16_t npId, uint8_t eAction, uint16_t eId, uint16_t eData ) {

  // ?? what events would a can be interested in ...

    printf( "EventCallback -> npId: 0x%x, eId: %d, action: %d, data: %d\n", 
            npId, eId, eAction, eData );
}

//----------------------------------------------------------------------------------------
// General LCS Bus messages we might want to handle....
//
//----------------------------------------------------------------------------------------
void busMgtCallback( uint8_t *msg ) {

    printf( "busMgtCallback -> " );
    printLcsMsg( msg );

    switch ( msg[ 0 ] ) {

        case LCS_OP_OPS:
        case LCS_OP_CFG:
        case LCS_OP_BON:
        case LCS_OP_BOF:
        case LCS_OP_NCOL:
        case LCS_OP_RESET: break;

        default: ;
    }
}

//----------------------------------------------------------------------------------------
// DCC messages callback. We will not handle all LCS DCC messages though. 
// 
// Do: track TON, TOFF, ESTP
// What about shared cab handhelds ?
//
//----------------------------------------------------------------------------------------
void dccMsgCallback( uint8_t *msg ) {

    printf( "DCC Msg Callback -> " );
    printLcsMsg( msg );

    switch ( msg[ 0 ] ) {

        case LCS_OP_REQ_LOC:
        case LCS_OP_REL_LOC:
        case LCS_OP_REP_LOC:
        case LCS_OP_SET_LCON:
        case LCS_OP_KEEP_LOC:
        case LCS_OP_SET_LSPD:
        case LCS_OP_SET_LMOD:
        case LCS_OP_LOC_FON:
        case LCS_OP_LOC_FOF:

        case LCS_OP_TON:
        case LCS_OP_TOF:
        case LCS_OP_ESTP:

        case LCS_OP_DCC_ACK:
        case LCS_OP_DCC_ERR:  break;

        default: ;
    }
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t CabMsgBus::sendSpeedAndDir( CabEntry *cab ) {

    // ??? build the LCS message from the cab entry ...
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t CabMsgBus::sendDccFuncVal( CabEntry *cab, uint8_t dccFundId ) {

    // ??? get val from bitmap...

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t CabMsgBus::sendEngineOnOff( CabEntry *cab ) {

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t CabMsgBus::requestLocoSession( CabEntry *cab ) {

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t CabMsgBus::closeLocoSession( CabEntry *cab ) {

    // ??? not clear yet, a dispatch ? a true close ? what else ...

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t CabMsgBus::loadCabData( CabEntry *cab ) {

    // ??? we will load all data from the Basestation Dictionary for the engine... 
    // can be done one word at a time ...

    return ( NO_ERR );
}
