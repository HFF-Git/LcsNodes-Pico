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
// LCS - Runtime Library
// Copyright (C) 2021 - 2025  Helmut Fieres
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
// External declaration to global structures defined in "LcsRtSetup".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t             debugMask;
    extern LcsNodeMap           nodeMap;
    extern LcsPortMap           portMap;
    extern LcsPendingReqMap     pendingReqMap;
    extern LcsTaskMap           taskMap;
    extern LcsMsgBusCAN         *msgBus;

    extern uint8_t              localMsgEvent( uint8_t *msg );
};

//------------------------------------------------------------------------------------------------------------
// File local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Little helper functions and constants.
//
//------------------------------------------------------------------------------------------------------------
const uint32_t DEF_REQ_TIMEOUT_VAL_MS = 50000;

bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

  return (( val >= lower ) && ( val <= upper ));
}

uint16_t buildNpId( uint16_t nodeId, uint16_t portId ) {

    return (( nodeId << 4 ) | ( portId & 0xF ));
}

uint16_t nodeId( uint16_t npId ) {

    return ( npId >> 4 );
}

uint16_t portId( uint16_t npId ) {

    return ( npId & 0xF );
}

uint8_t lowByte( uint16_t arg ) { 
    
    return ( arg & 0xFF ); 
}

uint8_t highByte( uint16_t arg ) { 
    
    return ( arg >> 8 ); 
}

//------------------------------------------------------------------------------------------------------------
// There are some LCS messages that expect a reply message. The library maintains a small pending request
// buffer. When a request type message is sent we add the target node and a timer value to the buffer. Easy 
// and simple. Note that there can be more than one entry for the same node / port combination in the buffer.
// If the buffer is full, an error is returned. We have too many outstanding requests then.
// 
// A request can also be registered with a timeout value. When the timeout expires, the caller is informed
// that the request timed out. A timeout value of zero means that we wait indefinitely.
//
//------------------------------------------------------------------------------------------------------------
uint8_t addToPendingReqMap( uint16_t npId, uint32_t timeoutVal = 0 ) {

    uint32_t ts = CDC::getMillis( );

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        if ( pendingReqMap.map[ i ].npId == 0 ) {

            pendingReqMap.map[ i ].npId = npId;
            pendingReqMap.map[ i ].reqTimeoutTs = (( timeoutVal != 0 ) ? ts + timeoutVal : timeoutVal );
            return ( ALL_OK );
        }
    }
    
    return ( ERR_PENDING_REQ_MAP_FULL );
}

//------------------------------------------------------------------------------------------------------------
// "removeFromPendingReqMap" removes an entry from the pending reply buffer. If the entry is not found, we
// received a reply for a request that we do not know. Right now, we just ignore this error.
//
//------------------------------------------------------------------------------------------------------------
uint8_t removeFromPendingReqMap( uint16_t npId ) {

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        if ( pendingReqMap.map[ i ].npId == npId ) pendingReqMap.map[ i ].npId = NIL_NODE_ID;
    }

    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "searchPendingReqMap" searches the pending request buffer for a matching node. We just return a boolean
// answer whether the entry is there or not.
//
//------------------------------------------------------------------------------------------------------------
bool searchPendingReqMap( uint16_t npId ) {

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        if ( pendingReqMap.map[ i ].npId == npId ) return ( true );
    }

    return ( false );
}

//------------------------------------------------------------------------------------------------------------
// "processPendingReqMapTimeouts" is part of the periodic processing of the node. It will check wether any
// requests waiting for a reply have timed out. In this case, we should invoke the reply callback with an 
// error code and clear the entry.
//
//------------------------------------------------------------------------------------------------------------
void processPendingReqMapTimeouts( ) {

    uint32_t ts = CDC::getMillis( );

    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        LcsPendingReqEntry *tPtr = &pendingReqMap.map[ i ];

        if (( tPtr -> reqTimeoutTs != 0 ) && ( tPtr -> reqTimeoutTs < ts )) {

            LcsPortMapEntry *pPtr = &portMap.map[ portId( tPtr -> npId ) ];

            if ( pPtr -> repCallback != nullptr ) {

                pPtr -> repCallback( pendingReqMap.map[ i ].npId, 0, 0, 0, ERR_REQ_TIMEOUT );                
            }

            tPtr -> reqTimeoutTs    = 0;
            tPtr -> npId            = 0;
        } 
    }
}

//------------------------------------------------------------------------------------------------------------
// "sendLcsMsg" will send a message when the node is either OPERATe or CONFIG mode.
// 
// ??? not all messages should be enabled when we are in CFG mode...
// ??? should we also pass our own nodeId as the canId ?
//------------------------------------------------------------------------------------------------------------
uint8_t sendLcsMsg( uint8_t *msg, uint8_t msgPri ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );
    return ( msgBus -> sendLcsMsg( msg, msgPri ));
}

//------------------------------------------------------------------------------------------------------------
// Some messages are requests that expect a reply. We maintain a pending request map which keeps track of 
// outstanding requests. In addition we can pass a timeout value to handle cases where no reply is received
// in a given time interval.
//
// ??? should we also pass our own nodeId as the canId ?
//------------------------------------------------------------------------------------------------------------
uint8_t sendTimedReq( uint16_t npId, uint8_t *msg, uint8_t msgPri, uint32_t timeout = 0 ) {

    if ( addToPendingReqMap( npId , timeout ) == ALL_OK )   return ( sendLcsMsg( msg, msgPri ));
    else                                                    return ( ERR_NODE_OUTSTANDING_REQ_LIMIT );
}


}; // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// A simple helper to print an LCS message.
//
//------------------------------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

    printf( "LCS MSG: op: %d, data: ", msg[ 0 ] & 0x1F );
    for ( int i = 0; i < ( msg[ 0 ] >> 5 ) + 1; i ++ ) printf( "0x%x ", msg[ i ] ); 
    printf( "\n" );
}

//------------------------------------------------------------------------------------------------------------
// The primary task of the receive function is to receive an LCS messages and pass them to the respective
// handler method. In order to not always check whether a valid message was processed, this routine will
// always return a valid message opCode. The "LCS_NO_MSG" pseudo message is used to indicate that something
// else happened and no further message processing is required. We also maintain a request / reply map to
// keep track of outstanding requests transparently to the caller.
//
//------------------------------------------------------------------------------------------------------------
uint8_t receiveLcsMsg( uint8_t *msg ) {

    int rStat = msgBus -> receiveLcsMsg( msg );

    if ( rStat == ALL_OK )  {

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_MSG_BUS )) {
            
             printf( "Can Msg Received (OpCode): 0x%x\n", msg[ 0 ] );
        }

        if (( msg[ 0 ] == LCS_OP_NODE_REP ) || ( msg[ 0 ] == LCS_OP_ACK ) || ( msg[ 0 ] == LCS_OP_ERR )) {

             uint16_t nodeId = (( msg[1] << 8 ) + msg[2] ) >> 4;

            if ( searchPendingReqMap( nodeId )) {

                removeFromPendingReqMap( nodeId );
                return ( msg[ 0 ] );
            
            } else return ( LCS_OP_NO_MSG );

        } else return ( msg[ 0 ] );

    } else return ( LCS_OP_NO_MSG );
}

//------------------------------------------------------------------------------------------------------------
// LCB message send routines. They all follow the same pattern. There is a method for each message opcode,
// which maps the input parameters to the byte array and then send it. Depending on the type of sending
// there are different local routines used. Straightforward.
//
//------------------------------------------------------------------------------------------------------------
uint8_t sendCfg( uint16_t npId ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );

    uint8_t msgBuf[ 8 ] = { LCS_OP_CFG };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );

    return ( sendTimedReq( npId, msgBuf,  MSG_PRI_HIGH , 0 ));
}

uint8_t sendOps( uint16_t npId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_OPS };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );

    return ( sendTimedReq( npId, msgBuf,  MSG_PRI_HIGH , 0 ));
}

uint8_t sendReset( uint16_t npId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_RESET };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
   
    return ( sendTimedReq( npId, msgBuf, MSG_PRI_HIGH, 0 ));
}

uint8_t sendBusOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BON };
    return ( sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendBusOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BOF };
    return ( sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendErr( uint16_t npId, uint8_t errCode, uint8_t arg1, uint8_t arg2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ERR };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = errCode;
    msgBuf[ 4 ] = arg1;
    msgBuf[ 5 ] = arg2;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendAck( uint16_t npId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ACK };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendSync( uint16_t npId, uint8_t item ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SYNC };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = item;
    return ( sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendReqNodeId( uint16_t npId, uint32_t nodeUID, uint8_t flags ) {
    
    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_NID };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    msgBuf[ 7 ] = flags;
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendRepNodeId( uint16_t npId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_NID };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendSetNodeId( uint16_t npId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_NID };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendNodeIdCollision( uint16_t npId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NCOL };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendGetNode( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_GET };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( sendTimedReq( npId, msgBuf,  MSG_PRI_NORMAL, 0 ));
}

uint8_t sendSetNode( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_PUT };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( sendTimedReq( npId, msgBuf,  MSG_PRI_NORMAL, 0 ));
}

uint8_t sendReqNode( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_REQ };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( sendTimedReq( npId, msgBuf,  MSG_PRI_LOW, 0 ));
}

uint8_t sendRepNode( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_REP };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( msgBus -> sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendEventOn( uint16_t npId, uint16_t eventId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT_ON };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );

    localMsgEvent( msgBuf );
    return ( sendLcsMsg( msgBuf, MSG_PRI_LOW ));                                  
}

uint8_t sendEventOff( uint16_t npId, uint16_t eventId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT_OFF };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );

    localMsgEvent( msgBuf );
    return ( sendLcsMsg( msgBuf, MSG_PRI_LOW ));  
}

uint8_t sendEvent( uint16_t npId, uint16_t eventId, uint16_t arg ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );
    msgBuf[ 5 ] = highByte( arg );
    msgBuf[ 6 ] = lowByte( arg );

    localMsgEvent( msgBuf );
    return ( sendLcsMsg( msgBuf, MSG_PRI_LOW ));  
}

uint8_t sendTrackOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_TON };
    return ( sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendTrackOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_TOF };
    return ( sendLcsMsg( msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendEstop( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ESTP };
    return ( sendLcsMsg( msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendReqLoc( uint16_t locAdr, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_LOC };
    msgBuf[ 1 ] = highByte( locAdr );
    msgBuf[ 2 ] = lowByte( locAdr );
    msgBuf[ 3 ] = flags;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRelLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REL_LOC };
    msgBuf[ 1 ] = sId;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
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
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendLocConsist( uint8_t sId, uint8_t consId, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LCON };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = consId;
    msgBuf[ 3 ] = flags;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendQueryLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_QRY_LOC };
    msgBuf[ 1 ] = sId;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendKeepLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_KEEP_LOC };
    msgBuf[ 1 ] = sId;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocSpDir( uint8_t sId, uint8_t spDir ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LSPD };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = spDir;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocMode( uint8_t sId, uint8_t mode ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LMOD };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = mode;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFuncOn( uint8_t sId, uint8_t fNum ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FON };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fNum;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFuncOff( uint8_t sId, uint8_t fNum ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FOF };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fNum;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFgroup( uint8_t sId, uint8_t fGroup, uint8_t data ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FGRP };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fGroup;
    msgBuf[ 3 ] = data;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocCvMain( uint8_t sId, uint16_t cvId, uint8_t mode, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_CVM };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = highByte( cvId );
    msgBuf[ 3 ] = lowByte( cvId );
    msgBuf[ 4 ] = mode;
    msgBuf[ 5 ] = val;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocCvProg( uint16_t cvId, uint8_t mode, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = mode;
    msgBuf[ 4 ] = val;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendReqLocCvProg( uint16_t cvId, uint8_t mode ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = mode;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRepLocCvProg( uint16_t cvId, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = val;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetBacc( uint16_t accAdr, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BACC };
    msgBuf[ 1 ] = highByte( accAdr );
    msgBuf[ 2 ] = lowByte( accAdr );
    msgBuf[ 3 ] = flags;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetEacc( uint16_t accAdr, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EACC };
    msgBuf[ 1 ] = highByte( accAdr );
    msgBuf[ 2 ] = lowByte( accAdr );
    msgBuf[ 3 ] = val;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC3 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC4 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC5 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    msgBuf[ 5 ] = arg5;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC6 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    msgBuf[ 5 ] = arg5;
    msgBuf[ 6 ] = arg6;
    return ( sendLcsMsg( msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccAck( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_DCC_ACK };
    return ( sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

uint8_t sendDccErr( uint8_t errCode, uint8_t arg1, uint8_t arg2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_DCC_ERR };
    msgBuf[ 1 ] = errCode;
    msgBuf[ 2 ] = arg1;
    msgBuf[ 3 ] = arg2;
    return ( sendLcsMsg( msgBuf, MSG_PRI_LOW ));
}

// ??? add firmware update messages...

}; // namespace LCS
