
//----------------------------------------------------------------------------------------
//
// Layout Control System - LCS Message routines. 
//
//----------------------------------------------------------------------------------------
// At the message level, the LCS runtime offers a message bus to which all nodes 
// are connected. Currently, it is a CAN bus. Pretty straightforward and robust. 
// This file contains the routines to set up the node communication as well as a
// set of convenience functions for sending a LCS message taking care of filling 
// the message frame. Some LCS message are of a "request/reply" nature. When a 
// request is sent out an entry is made in the pending request map. Since the 
// message layer sees all reply message, this pending map is used to filter for 
// the request we are waiting for.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - LCS Message routines. 
// Copyright (C) 2020 - 2026 Helmut Fieres
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
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//----------------------------------------------------------------------------------------
// External declaration to global structures defined in "LcsRtSetup".
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t             debugMask;
    extern LcsNodeMap           nodeMap;
    extern LcsPortMap           portMap;
    extern LcsTaskMap           taskMap;
    extern LcsMsgBusCAN         *msgBus;

    extern uint8_t              localMsgEvent( uint8_t *msg );      
};

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------
// Little helper functions and constants.
//
//----------------------------------------------------------------------------------------
const uint32_t DEF_REQ_TIMEOUT_VAL_MS = 50000;

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
uint8_t sendMsg( uint16_t senderNpId, 
                 uint16_t targetNpId, 
                 uint8_t *msg, 
                 uint8_t  msgPri ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && 
        ( nodeMap.nodeState != NS_CONFIG )) 
        return ( ERR_LIB_NOT_READY );
        
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msg, msgPri ));
}

uint8_t sendTimedReq( uint16_t targetNpId, 
                      uint8_t *msg, 
                      uint8_t  msgPri, 
                      uint32_t timeout = 0 ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && 
        ( nodeMap.nodeState != NS_CONFIG )) 
        return ( ERR_LIB_NOT_READY );

    // check that the port/channel  is not already active ...

    // ??? add timeout data to port...

    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msg, msgPri ));
}



}; // namespace


//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// The primary task of the receive function is to receive an LCS messages and pass
// them to the respective handler method. In order to not always check whether a 
// valid message was processed, this routine will always return a valid message 
// opCode. The "LCS_NO_MSG" pseudo message is used to indicate that something else
// happened and no further message processing is required. 
//
//----------------------------------------------------------------------------------------
uint8_t receiveLcsMsg( uint16_t *senderNpId, uint8_t *msg ) {

    int rStat = msgBus -> receiveLcsMsg( senderNpId, msg );

    if ( rStat == LCS_OK )  {

        if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_MSG_BUS )) {
            
             printf( "Can Msg Received (OpCode): 0x%x\n", msg[ 0 ] );
        }

        if (( msg[ 0 ] == LCS_OP_NODE_REP ) || 
            ( msg[ 0 ] == LCS_OP_ACK ) || 
            ( msg[ 0 ] == LCS_OP_ERR )) {

            uint16_t npId = (( msg[1] << 8 ) + msg[2] ) >> 4;

            // ??? check the port / channel for a pending request...
            // fix ....
            
            return ( LCS_OP_NO_MSG );

        } else return ( msg[ 0 ] );

    } else return ( LCS_OP_NO_MSG );
}

//----------------------------------------------------------------------------------------
// LCB message send routines. They all follow the same pattern. There is a method
// for each message opcode, which maps the input parameters to the byte array and 
// then send it. Depending on the type of sending there are different local routines
// used.
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendCfg( uint16_t targetNpId ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && 
        ( nodeMap.nodeState != NS_CONFIG )) 
        return ( ERR_LIB_NOT_READY );

    uint8_t msgBuf[ 8 ] = { LCS_OP_CFG };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );

    return ( sendTimedReq( targetNpId, msgBuf,  MSG_PRI_HIGH, 0 ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendOps( uint16_t targetNpId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_OPS };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );

    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_HIGH , 0 ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendReset( uint16_t targetNpId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_RESET };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
   
    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_HIGH, 0 ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendBusOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BON };
    return ( sendMsg( nodeMap.nodeId, NIL_NODE_ID, msgBuf, MSG_PRI_VERY_HIGH ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendBusOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BOF };
    return ( sendMsg( nodeMap.nodeId, NIL_NODE_ID, msgBuf, MSG_PRI_VERY_HIGH ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
// ??? combine with ACK ?
uint8_t sendErr( uint16_t sendingNpId, 
                 uint16_t targetNpId, 
                 uint8_t errCode, 
                 uint8_t arg1, 
                 uint8_t arg2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ERR };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = errCode;
    msgBuf[ 4 ] = arg1;
    msgBuf[ 5 ] = arg2;
    return ( msgBus -> sendLcsMsg( sendingNpId, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendAck( uint16_t sendingNpId, uint16_t targetNpId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ACK };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    return ( sendMsg( sendingNpId, NIL_NODE_ID, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendSync( uint16_t targetNpId, uint16_t item ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SYNC };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = item;
    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_LOW, 0 ));
}

//----------------------------------------------------------------------------------------
// Node Id management.
//
//----------------------------------------------------------------------------------------
uint8_t sendReqNodeId( uint16_t npId, uint32_t nodeUID, uint8_t flags ) {
    
    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_NID };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    msgBuf[ 7 ] = flags;
    return ( msgBus -> sendLcsMsg( npId, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendRepNodeId( uint16_t npId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_NID };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendSetNodeId( uint16_t npId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_NID };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendNodeCollision( uint16_t npId, uint32_t nodeUID ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NCOL };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = ( nodeUID & 0xFF000000 ) >> 24;
    msgBuf[ 4 ] = ( nodeUID & 0x00FF0000 ) >> 16;
    msgBuf[ 5 ] = ( nodeUID & 0x0000FF00 ) >> 8;
    msgBuf[ 6 ] = ( nodeUID & 0x000000FF );
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_HIGH ));
}

//----------------------------------------------------------------------------------------
// Attribute access Management.
//
//----------------------------------------------------------------------------------------
uint8_t sendGetAttr( uint16_t sendingNpId, 
                     uint16_t targetNpId,           
                     uint16_t item, 
                     LcsRepCallback rep,
                     void *uData ) {

    LcsPortMapEntry *portPtr = &portMap.map[ portId( targetNpId ) ];

    // ??? check if port is not busy
    // ??? store sending npId, callback, uData and the timeout.

    uint8_t msgBuf[ 8 ] = { LCS_OP_ATTR_GET }; // ??? fix...
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = item;
    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_NORMAL, 0 ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendRepAttr( uint16_t sendingNpId, 
                  uint16_t targetNpId,           
                  uint16_t item, 
                  uint16_t arg )  {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_GET };

    // it is stored in the port...
    // ??? rather check if there is an active request...

    return ( sendMsg( targetNpId, NIL_NODE_ID, msgBuf, MSG_PRI_NORMAL ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendSetAttr( uint16_t sendingNpId, 
                     uint16_t targetNpId,           
                     uint16_t item, 
                     uint16_t arg,
                     LcsRepCallback rep,
                     void *uData )  {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ATTR_SET }; // ??? fix...
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );

    // it is stored in the port...
    // ??? rather check if there is an active request...

    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_NORMAL, 0 ));
}

//----------------------------------------------------------------------------------------
// Extended attributes access management.
//
//----------------------------------------------------------------------------------------
uint8_t sendGetExtAttr( uint16_t sendingNpId, 
                        uint16_t targetNpId,           
                        uint16_t index, 
                        LcsRepCallback rep,
                        void *uData )  {
    
    // ??? check if port is not busy
    // ??? store sending npId, callback, uData and the timeout.

    uint8_t msgBuf[ 8 ] = { LCS_OP_ATTR_GET }; // ??? fix...
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = highByte( index );
    msgBuf[ 4 ] = lowByte( index );

    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_NORMAL, 0 ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendRepExtAttr( uint16_t sendingNpId, 
                        uint16_t targetNpId,           
                        uint16_t index, 
                        uint16_t arg )  {

    // ??? why do we have targetNpId ?
    // it is stored in the port...
    // ??? rather check if there is an active request...

    uint8_t msgBuf[ 8 ] = { LCS_OP_ATTR_REP }; // ??? fix...
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = highByte( index );
    msgBuf[ 4 ] = lowByte( index );
    msgBuf[ 5 ] = highByte( arg );
    msgBuf[ 6 ] = lowByte( arg );

    return ( sendMsg( sendingNpId, targetNpId, msgBuf, MSG_PRI_NORMAL ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendSetExtAttr( uint16_t sendingNpId, 
                        uint16_t targetNpId,           
                        uint16_t index, 
                        uint16_t arg,
                        LcsRepCallback rep,
                        void *uData )  {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ATTR_SET }; // ??? fix...
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = highByte( index );
    msgBuf[ 4 ] = lowByte( index );
    msgBuf[ 5 ] = highByte( arg );
    msgBuf[ 6 ] = lowByte( arg );

    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_NORMAL, 0 ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendReqFunc( uint16_t sendingNpId, 
                     uint16_t targetNpId,           
                     uint16_t item, 
                     uint16_t arg1,
                     uint16_t arg2,
                     LcsRepCallback rep,
                     void *uData ) {

    // ??? check if port is not busy
    // ??? store sending npId, callback, uData and the timeout.

    return ( 0 );
}

uint8_t sendRepFunc( uint16_t sendingNpId, 
                     uint16_t targetNpId,           
                     uint16_t item, 
                     uint16_t arg1,
                     uint16_t arg2 )  {

    // it is stored in the port...
    // ??? rather check if there is an active request...

    return ( 0 );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendGetNode( uint16_t sendingNpId, 
                     uint16_t targetNpId, 
                     uint16_t item, 
                     uint16_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_GET };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val );
    msgBuf[ 5 ] = lowByte( val );
    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_NORMAL, 0 ));
}

uint8_t sendSetNode( uint16_t sendingNpId,
                     uint16_t targetNpId, 
                     uint16_t item, uint16_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_SET };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val );
    msgBuf[ 5 ] = lowByte( val );
    return ( sendTimedReq( targetNpId, msgBuf, MSG_PRI_NORMAL, 0 ));
}

uint8_t sendReqNode( uint16_t sendingNpId,
                     uint16_t targetNpId, 
                     uint16_t item, 
                     uint16_t val1, 
                     uint16_t val2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_REQ };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( sendTimedReq( targetNpId, msgBuf,  MSG_PRI_LOW, 0 ));
}

uint8_t sendRepNode( uint16_t sendingNpId,
                     uint16_t targetNpId,
                     uint16_t item, 
                     uint16_t val1,
                     uint16_t val2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_NODE_REP };
    msgBuf[ 1 ] = highByte( targetNpId );
    msgBuf[ 2 ] = lowByte( targetNpId );
    msgBuf[ 3 ] = item;
    msgBuf[ 4 ] = highByte( val1 );
    msgBuf[ 5 ] = lowByte( val1 );
    msgBuf[ 6 ] = highByte( val2 );
    msgBuf[ 7 ] = lowByte( val2 );
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t sendEventOn( uint16_t npId, uint16_t eventId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT_ON };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );

    localMsgEvent( msgBuf );
    return ( msgBus -> sendLcsMsg( npId, msgBuf, MSG_PRI_LOW ));                                  
}

uint8_t sendEventOff( uint16_t npId, uint16_t eventId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EVT_OFF };
    msgBuf[ 1 ] = highByte( npId );
    msgBuf[ 2 ] = lowByte( npId );
    msgBuf[ 3 ] = highByte( eventId );
    msgBuf[ 4 ] = lowByte( eventId );

    localMsgEvent( msgBuf );
    return ( msgBus -> sendLcsMsg( npId, msgBuf, MSG_PRI_LOW ));  
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
    return ( msgBus -> sendLcsMsg( npId, msgBuf, MSG_PRI_LOW ));  
}


//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t sendTrackOn( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_TON };
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendTrackOff( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_TOF };
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_HIGH ));
}

uint8_t sendEstop( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_ESTP };
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_VERY_HIGH ));
}

uint8_t sendReqLoc( uint16_t locAdr, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_LOC };
    msgBuf[ 1 ] = highByte( locAdr );
    msgBuf[ 2 ] = lowByte( locAdr );
    msgBuf[ 3 ] = flags;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRelLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REL_LOC };
    msgBuf[ 1 ] = sId;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRepLoc( uint8_t sId, 
                    uint16_t locAdr, 
                    uint8_t spDir, 
                    uint8_t fn1, 
                    uint8_t fn2, 
                    uint8_t fn3  ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_LOC };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = highByte( locAdr );
    msgBuf[ 3 ] = lowByte( locAdr );
    msgBuf[ 4 ] = spDir;
    msgBuf[ 5 ] = fn1;
    msgBuf[ 6 ] = fn2;
    msgBuf[ 7 ] = fn3;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendLocConsist( uint8_t sId, uint8_t consId, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LCON };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = consId;
    msgBuf[ 3 ] = flags;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendQueryLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_QRY_LOC };
    msgBuf[ 1 ] = sId;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendKeepLoc( uint8_t sId ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_KEEP_LOC };
    msgBuf[ 1 ] = sId;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocSpDir( uint8_t sId, uint8_t spDir ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LSPD };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = spDir;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocMode( uint8_t sId, uint8_t mode ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_LMOD };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = mode;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFuncOn( uint8_t sId, uint8_t fNum ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FON };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fNum;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFuncOff( uint8_t sId, uint8_t fNum ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FOF };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fNum;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocFgroup( uint8_t sId, uint8_t fGroup, uint8_t data ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_LOC_FGRP };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = fGroup;
    msgBuf[ 3 ] = data;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocCvMain( uint8_t sId, uint16_t cvId, uint8_t mode, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_CVM };
    msgBuf[ 1 ] = sId;
    msgBuf[ 2 ] = highByte( cvId );
    msgBuf[ 3 ] = lowByte( cvId );
    msgBuf[ 4 ] = mode;
    msgBuf[ 5 ] = val;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetLocCvProg( uint16_t cvId, uint8_t mode, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SET_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = mode;
    msgBuf[ 4 ] = val;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendReqLocCvProg( uint16_t cvId, uint8_t mode ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REQ_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = mode;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendRepLocCvProg( uint16_t cvId, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_REP_CVS };
    msgBuf[ 1 ] = highByte( cvId );
    msgBuf[ 2 ] = lowByte( cvId );
    msgBuf[ 3 ] = val;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetBacc( uint16_t accAdr, uint8_t flags ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_BACC };
    msgBuf[ 1 ] = highByte( accAdr );
    msgBuf[ 2 ] = lowByte( accAdr );
    msgBuf[ 3 ] = flags;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendSetEacc( uint16_t accAdr, uint8_t val ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_EACC };
    msgBuf[ 1 ] = highByte( accAdr );
    msgBuf[ 2 ] = lowByte( accAdr );
    msgBuf[ 3 ] = val;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC3 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC4 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, 
                       uint8_t arg2, 
                       uint8_t arg3, 
                       uint8_t arg4, 
                       uint8_t arg5 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC5 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    msgBuf[ 5 ] = arg5;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccPacket( uint8_t arg1, 
                       uint8_t arg2, 
                       uint8_t arg3, 
                       uint8_t arg4, 
                       uint8_t arg5, 
                       uint8_t arg6 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_SEND_DCC6 };
    msgBuf[ 1 ] = arg1;
    msgBuf[ 2 ] = arg2;
    msgBuf[ 3 ] = arg3;
    msgBuf[ 4 ] = arg4;
    msgBuf[ 5 ] = arg5;
    msgBuf[ 6 ] = arg6;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_NORMAL ));
}

uint8_t sendDccAck( ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_DCC_ACK };
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_LOW ));
}

uint8_t sendDccErr( uint8_t errCode, uint8_t arg1, uint8_t arg2 ) {

    uint8_t msgBuf[ 8 ] = { LCS_OP_DCC_ERR };
    msgBuf[ 1 ] = errCode;
    msgBuf[ 2 ] = arg1;
    msgBuf[ 3 ] = arg2;
    return ( msgBus -> sendLcsMsg( nodeMap.nodeId, msgBuf, MSG_PRI_LOW ));
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
// ??? add firmware update messages...

}; // namespace LCS
