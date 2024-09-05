//------------------------------------------------------------------------------------------------------------
//
// "LcsRtMsgBus" - implementation file.
//
//------------------------------------------------------------------------------------------------------------
// At the message level, the LCS runtime offers a message to which all nodes are connected. Currently, it is
// a CAN bus. Pretty straightforward and robust. This file contains the routines to set up the communication
// as well as a set of convenience functions for sending a LCS message taking care of filling the message
// frame. Some LCS message are of a "request/reply" nature. When a request is sent out a entry is made in
// the pending request map. Since the message layer sees all reply message, this pending map is used to
// filter for the request we are waiting for.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Core Library
// Copyright (C) 2021 - 2024  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation, either version 3 of the License,
// or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details. You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.
//
//------------------------------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//------------------------------------------------------------------------------------------------------------
// External declaration to global structures.
//
//------------------------------------------------------------------------------------------------------------
extern LCS::LcsNodeMap        nodeMap;
extern LCS::LcsCallbackMap    callbackMap;
extern LCS::LcsPendingReqMap  pendingReqMap;
extern LCS::LcsTaskMap        taskMap;
extern LCS::LcsMsgBusCAN      *msgBus;

//------------------------------------------------------------------------------------------------------------
// File local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------  
// Debug and Trace support. Instead of conditional compilation, we will print debug messages based on the
// setting of the debug level.
//------------------------------------------------------------------------------------------------------------ 
uint8_t debugLevel = 0;

//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
uint8_t lowByte( uint16_t arg ) { return( arg & 0xFF ); }
uint8_t highByte( uint16_t arg ) { return( arg >> 8 ); }

//------------------------------------------------------------------------------------------------------------
// There are some LCS messages that expect a reply message. The library maintains a small pending request
// buffer. When a request type message is sent we add the target node to the buffer. Easy and simple. Note
// that there can be more than one entry for the same node in the buffer. If the buffer is full, an error
// is returned. We have too many outstanding requests then.
//
// Idea: scheme could be refined to have a node and a count for pending requests. Perhaps later...
//------------------------------------------------------------------------------------------------------------
uint8_t addToPendingReplyMap( uint16_t nodeId ) {

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        if ( pendingReqMap.map[ i ].nodeId == NIL_NODE_ID ) {

            pendingReqMap.map[ i ].nodeId = nodeId;
            return ( ALL_OK );
        }
    }
    
    return ( ERR_PENDING_REQ_MAP_FULL );
}

//------------------------------------------------------------------------------------------------------------
// "removeFromPendingReplyMap" removes an entry from the pending reply buffer. If the entry is not found, we
// received a reply for a request that we do not know. Right now, we just ignore this error.
//
//------------------------------------------------------------------------------------------------------------
uint8_t removeFromPendingReplyMap( uint16_t nodeId ) {

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        if ( pendingReqMap.map[ i ].nodeId == nodeId ) pendingReqMap.map[ i ].nodeId = NIL_NODE_ID;
    }

    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "checkPendingReplyMap" searches the pending request buffer for a matching node.
//
//------------------------------------------------------------------------------------------------------------
bool checkPendingReplyMap( uint16_t nodeId ) {

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        if ( pendingReqMap.map[ i ].nodeId == nodeId ) return ( true );
     }

    return ( false );
}

}; // namespace


// ??? check for timeouts on pending replies ...


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "setupMsgBus" is called during node initialization to setup the LCS message bus interface. Right now,
// only the CAN bus is supported. We first create the CANBus object and then call its initialization routine.
// The canId and nodeId are identical. We ensure through LCS configuration that the nodeIds are unique.
//
// ??? rework CAN BUS to a generic LCS message communication library and make CAN bus one implementation ?
//------------------------------------------------------------------------------------------------------------
uint8_t setupMsgBus( ) {

    uint8_t     rStat           = ALL_OK;
    uint16_t    canBusCtrlMode  = CAN_BUS_LIB_PICO_PIO_500K_M_CORE;
    uint8_t     canBusTxPin     = 0;
    uint8_t     canBusRxPin     = 0;

    if ( debugLevel > 0 ) {
        printf( "setupMsgBus -> %d:%d:%d%d\n", nodeMap.nodeId, canBusRxPin, canBusTxPin, canBusCtrlMode );
    }

    switch ( canBusCtrlMode ) {

        case CAN_BUS_LIB_PICO_PIO_125K:
        case CAN_BUS_LIB_PICO_PIO_250K:
        case CAN_BUS_LIB_PICO_PIO_500K:
        case CAN_BUS_LIB_PICO_PIO_1000K:
        case CAN_BUS_LIB_PICO_PIO_125K_M_CORE:
        case CAN_BUS_LIB_PICO_PIO_250K_M_CORE:
        case CAN_BUS_LIB_PICO_PIO_500K_M_CORE:
        case CAN_BUS_LIB_PICO_PIO_1000K_M_CORE: {

            msgBus = new LcsMsgBusCAN( );
            rStat = (( LcsMsgBusCAN *) msgBus ) -> init( nodeMap.nodeId, canBusRxPin, canBusTxPin, canBusCtrlMode );

        } break;

        default: rStat = ERR_CAN_SETUP;
    }

    if ( rStat != ALL_OK ) {

        if ( debugLevel > 0 ) printf( "setup CAN Bus failed: %d\n", rStat );
        return ( ERR_CAN_SETUP );
    }
     else {

        if ( debugLevel > 0 ) printf( " -> OK\n" );
        return ( ALL_OK );
    }
}

//------------------------------------------------------------------------------------------------------------
// The primary task of the receive function is to receive an LCS messages and pass them to the respective
// handler method. In order to not always check whether a valid message was processed, this routine will
// always return a valid message opCode. The "LCS_NO_MSG" pseudo message is used to indicate that something
// else happened and no further message processing is required.
//
// We also maintain a request / reply map.
//
// Also: maintain the request/reply map...
//
// ??? map the CAN BUS errors to generic communication errors ?
// ??? how to make this a generic msgBus receive method ?
//
// ??? should we have a pre-filter for message ID and nodeId match ?
// ??? this would be perhaps useful, when we run CAN bus on two cores on the pico. Then the checking would
// run on the interrupt handler processor.
//------------------------------------------------------------------------------------------------------------
uint8_t receiveLcsMsg( uint8_t *msg ) {

    int rStat = msgBus -> receiveLcsMsg( msg );

    if ( rStat == ALL_OK )  {

        if ( debugLevel > 0 ) printf( "Can Msg Received (OpCode): 0x%x\n", msg[ 0 ] );

        if (( msg[ 0 ] == LCS_OP_REP_NODE ) ||
            ( msg[ 0 ] == LCS_OP_ACK )      ||
            ( msg[ 0 ] == LCS_OP_ERR )) {

             uint16_t nodeId = (( msg[1] << 8 ) + msg[2] ) >> 4;

            if ( checkPendingReplyMap( nodeId )) {

                removeFromPendingReplyMap( nodeId );
                return ( msg[ 0 ] );
            
            } else return (  LCS_OP_NO_MSG );

        } else return ( msg[ 0 ] );

    } else return ( LCS_OP_NO_MSG );
}

//------------------------------------------------------------------------------------------------------------
// LCB message send routines. They all follow the same pattern. There is a method for each message opcode,
// which maps the input parameters to the byte array and then send it. Straightforward. For messages that are
// a part of a request / reply pair, the requesting messages will also add the requesting nodeId to the
// pending request map. This way we know that there is an outstanding request. The receiving message handler
// will clear the entry upon the matching receipt.
//
//------------------------------------------------------------------------------------------------------------
uint8_t sendReset( uint16_t nodeId, uint8_t portId, uint8_t flags ) {

    uint16_t tmp = (( nodeId << 4 ) | ( portId & 0x0F ));

    uint8_t msgBuf[ 8 ] = { LCS_OP_RESET };
    msgBuf[ 1 ] = highByte( tmp );
    msgBuf[ 2 ] = lowByte( tmp );
    msgBuf[ 3 ] = flags;

    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendBusOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BON };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendBusOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BOF };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendErr( uint16_t nodeId, uint8_t errCode, uint8_t arg1, uint8_t arg2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ERR };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = errCode;
    msgBuf[ 4 ] = arg1;
    msgBuf[ 5 ] = arg2;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendPing( uint16_t nodeId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_PING };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendAck( uint16_t nodeId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ACK };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendReqNodeId( uint16_t nodeId, uint32_t nodeUID, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_NID };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    msgBuf[ 7 ] = flags;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendRepNodeId( uint16_t nodeId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_NID };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendSetNodeId( uint16_t nodeId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_NID };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendNodeIdCollision( uint16_t nodeId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NCOL };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendQryNode( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    if ( addToPendingReplyMap( nodeId ) == ALL_OK ) {

        uint16_t tmp = (( nodeId << 4 ) | ( portId & 0x0F ));

        uint8_t msgBuf[ 8 ] = { LCS_OP_QRY_NODE };
        msgBuf[ 1 ] = highByte( tmp );
        msgBuf[ 2 ] = lowByte( tmp );
        msgBuf[ 3 ] = item;
        msgBuf[ 4 ] = highByte( val1 );
        msgBuf[ 5 ] = lowByte( val1 );
        msgBuf[ 6 ] = highByte( val2 );
        msgBuf[ 7 ] = lowByte( val2 );
        return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
    }
    else return ( ERR_NODE_OUTSTANDING_REQ_LIMIT );
}

uint8_t sendRepNode( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    uint16_t tmp = (( nodeId << 4 ) | ( portId & 0x0F ));

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_NODE };
    msgBuf[ 1 ] = highByte( tmp );
    msgBuf[ 2 ] = lowByte( tmp );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendReqNode( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    uint16_t tmp = (( nodeId << 4 ) | ( portId & 0x0F ));

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_NODE };
    msgBuf[ 1 ] = highByte( tmp );
    msgBuf[ 2 ] = lowByte( tmp );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendEventOn( uint16_t nodeId, uint16_t eventId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT_ON };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendEventOff( uint16_t nodeId, uint16_t eventId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT_OFF };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendEvent( uint16_t nodeId, uint16_t eventId, uint16_t arg ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT };
    msgBuf[ 1 ] = highByte( nodeId );
    msgBuf[ 2 ] = lowByte( nodeId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );
    msgBuf[ 5 ] = highByte( arg );
    msgBuf[ 6 ] = lowByte( arg );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendReqTrackOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_TON };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendTrackOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_TON };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendReqTrackOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_TOF };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendTrackOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_TOF };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendReqEstop( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_ESTP };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendEstop( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ESTP };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendReqLoc( uint16_t locAdr, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_LOC };
    msgBuf[ 1 ] = highByte( locAdr );
    msgBuf[ 2 ] = lowByte( locAdr );
    msgBuf[ 3 ] = flags;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRelLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REL_LOC };
    msgBuf[ 1 ] = sId;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRepLoc( uint8_t sId, uint16_t locAdr, uint8_t spDir, uint8_t fn1, uint8_t fn2, uint8_t fn3  ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_LOC };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = highByte( locAdr );
    msgBuf[ 3 ] = lowByte( locAdr );
    msgBuf[ 4 ] = spDir;
    msgBuf[ 5 ] = fn1;
    msgBuf[ 6 ] = fn2;
    msgBuf[ 7 ] = fn3;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendLocConsist( uint8_t sId, uint8_t consId, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LCON };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = consId;
    msgBuf[ 3 ] = flags;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendQueryLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_QRY_LOC };
    msgBuf[ 1 ] = sId;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendKeepLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_KEEP_LOC };
    msgBuf[ 1 ] = sId;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocSpDir( uint8_t sId, uint8_t spDir ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LSPD };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = spDir;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocMode( uint8_t sId, uint8_t mode ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LMOD };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = mode;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFuncOn( uint8_t sId, uint8_t fNum ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FON };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fNum;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFuncOff( uint8_t sId, uint8_t fNum ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FOF };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fNum;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFgroup( uint8_t sId, uint8_t fGroup, uint8_t data ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FGRP };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fGroup;
    msgBuf[ 3 ] = data;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocCvMain( uint8_t sId, uint16_t cvId, uint8_t mode, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_CVM };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = highByte( cvId );
    msgBuf[ 3 ] = lowByte( cvId );
    msgBuf[ 4 ] = mode;
    msgBuf[ 5 ] = val;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocCvProg( uint16_t cvId, uint8_t mode, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = mode;
    msgBuf[ 4 ] = val;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendReqLocCvProg( uint16_t cvId, uint8_t mode ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = mode;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRepLocCvProg( uint16_t cvId, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = val;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetBacc( uint16_t accAdr, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BACC };
    msgBuf[ 1 ] = highByte( accAdr );
    msgBuf[ 2 ] = lowByte( accAdr );
    msgBuf[ 3 ] = flags;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetEacc( uint16_t accAdr, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EACC };
    msgBuf[ 1 ] = highByte( accAdr );
    msgBuf[ 2 ] = lowByte( accAdr );
    msgBuf[ 3 ] = val;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC3 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC4 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC5 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    msgBuf[ 5 ] = arg5;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC6 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    msgBuf[ 5 ] = arg5;
    msgBuf[ 6 ] = arg6;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccAck( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_DCC_ACK };
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendDccErr( uint8_t errCode, uint8_t arg1, uint8_t arg2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_DCC_ERR };
    msgBuf[ 1 ] = errCode;
    msgBuf[ 2 ] = arg1;
    msgBuf[ 3 ] = arg2;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

}; // namespace LCS
