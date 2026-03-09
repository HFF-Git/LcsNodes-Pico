//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime library core.
//
//----------------------------------------------------------------------------------------
// The file contains the runtime core routines. The runtime library is essentially
// a big state machine, which periodically scans for messages and pother work to do. 
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Firmware Update. 
// Copyright (C) 2020 - 2026 Helmut Fieres
//
/// This program is free software: you can redistribute it and/or modify it under 
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
#include "LcsRtLibInt.h"

//----------------------------------------------------------------------------------------
// External declaration to global structures and functions in other modules.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    using namespace CDC;

    extern uint16_t             debugMask;
    extern uint16_t             runtimeOptions;
   
    extern LcsNodeMap           nodeMap;
    extern LcsPortMap           portMap;
    extern LcsEventMap          eventMap;
    extern LcsTaskMap           taskMap;
    extern LcsDrvFuncMap        drvFuncMap;

    extern uint8_t              handleSerialCommand( );
    extern uint8_t              setupDriverFunctions( );
    extern uint8_t              setupPortMap( );
    extern int                  searchEvent( uint16_t eventId );
    extern uint8_t              rtNvmPutWord( uint32_t ofs, uint16_t word );
    extern uint8_t              receiveLcsMsg( uint16_t *senderNpId, uint8_t *msg );
};

//----------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Local constants and helper functions.
//
//----------------------------------------------------------------------------------------
const uint32_t  NODE_SETUP_RETRY_TIMER_VAL_MS   = 1000L;
uint32_t        timerVal                        = 0L;

//----------------------------------------------------------------------------------------
// Reset.
//
//----------------------------------------------------------------------------------------
void reset( uint16_t npId ) {

    if ( portId( npId ) == 0 ) {

        sendAck( nodeMap.nodeId, npId );
        sleepMillis( 10000 );
    }
    else {

        // ??? have a reset callback ?
        // ??? for EXT devices invoke the RESET function on the handler

        sendAck( nodeMap.nodeId, npId );
    }
}

//----------------------------------------------------------------------------------------
// "handleNodePortEvents" will be called for processing inbound port events on each 
// runtime loop iteration. Note that it does not matter where the events came from, 
// i.e. whether another node sends an event or the event was created by a firmware 
// call on this node. The event callback can optionally be delayed with a timer value.
//
//----------------------------------------------------------------------------------------
void handleNodePortEvents( ) {

    uint32_t ts = getMillis( );

    for ( int i = 0; i < portMap.mapHwm; i ++ ) {

        LcsPortMapEntry *pPtr = & portMap.map[ i ];

        if (( pPtr -> flags & NPF_PORT_ENABLED                  ) &&
            ( pPtr -> flags & NPF_PORT_EVENT_HANDLING_ENABLED   ) &&
            ( pPtr -> flags & NPF_EVENT_PENDING                 ) && 
            ( pPtr -> eventCallback != nullptr                  )) {

            if ( ts > pPtr -> eventTimeStamp ) {

                pPtr -> eventCallback(  pPtr -> eventNpId,
                                        pPtr -> eventId,
                                        pPtr -> eventAction,
                                        pPtr -> eventValue,
                                        pPtr -> eventCallBackUdata );
            }

            pPtr -> flags &= ~ NPF_EVENT_PENDING;
        }
    }
}

//----------------------------------------------------------------------------------------
// "handlePendingReqTimeouts" is part of the periodic processing of the node. 
// It will check wether any requests waiting for a reply have timed out. In this 
// case, we should invoke the reply callback with an error code and clear the entry.
//
// ??? is this called ?
//----------------------------------------------------------------------------------------
void handlePendingReqTimeouts( ) {

    uint32_t ts = getMillis( );

    #if 0
    for ( uint8_t i = 0; i < MAX_PENDING_REQ_MAP_ENTRIES; i++ ) {

        LcsPendingReqEntry *tPtr = &pendingReqMap.map[ i ];

        if (( tPtr -> reqTimeoutTs != 0 ) && ( tPtr -> reqTimeoutTs > ts )) {

            LcsPortMapEntry *pPtr = &portMap.map[ portId( tPtr -> npId ) ];

            if ( pPtr -> repCallback != nullptr ) {

                pPtr -> repCallback( pendingReqMap.map[ i ].npId, 
                                     0, 
                                     0, 
                                     0, 
                                     ERR_REQ_TIMEOUT,
                                     pPtr -> repCallBackUdata );                
            }

            tPtr -> reqTimeoutTs    = 0;
            tPtr -> npId            = 0;
        } 
    }
    #endif
}


//----------------------------------------------------------------------------------------
// "handlePeriodicTasks" is called from the core library main processing loop. The 
// idea is that there is a lot of periodic processing that needs to be one by any 
// firmware implementation. Instead of the firmware developer writing its own handler,
// there is a simple method that just samples the timestamps and interval and triggers
// the callback then the interval is reached. Note that this is not very accurate from
// a timing perspective but will do for simple periodic processing.
//
//----------------------------------------------------------------------------------------
void handlePeriodicTasks( ) {

    uint32_t ts = getMillis( );

    for ( int i = 0; i < taskMap.mapHwm; i++ ) {

        LcsPTaskMapEntry *thisEntry = &taskMap.map[ i ];

        if ( ts > thisEntry -> timeStamp  ) {

            if ( thisEntry -> task != nullptr ) thisEntry -> task( nullptr );
            thisEntry -> timeStamp = ts + thisEntry -> interval;
        }
    }
}

//----------------------------------------------------------------------------------------
// "handleMsgRepNid" handles the message that the configuring node sends to our 
// node in response to a nodeId setup request. If the UID matches, the message is 
// for our node and we update our nodeId accordingly in MEM and NVM. The next node
// state is OPERATE.
//
//----------------------------------------------------------------------------------------
void handleMsgRepNid( uint8_t *msg ) {

    uint16_t nodeId   = ( msg[1] << 8 ) + msg[2];
    uint32_t nodeUID  = ((uint32_t) msg[3] << 24 ) +
                        ((uint32_t) msg[4] << 16 ) +
                        ((uint32_t) msg[5] <<  8 ) +
                        ((uint32_t) msg[6] );

    if ( nodeUID == nodeMap.nodeUID ) {

        nodeMap.nodeId = nodeId;
        uint8_t rStat  =
            rtNvmPutWord( NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId ), nodeId );
            
        nodeMap.nodeState = NS_OPERATE;
    }
}

//----------------------------------------------------------------------------------------
// LCS management deals with messages concerning the general LCS management. Most 
// messages just update the MEM nodeMap. If there is a callback, it will be invoked.
//
//----------------------------------------------------------------------------------------
void handleMsgLcsMgt( uint8_t *msg ) {

    switch ( msg[ 0 ] ) {

        case LCS_OP_OPS: {

            nodeMap.nodeState = NS_OPERATE;
            if ( nodeMap.lcsMsgCallback != nullptr ) 
                nodeMap.lcsMsgCallback( msg, nodeMap.lcsMsgCallBackUdata );

        } break;

        case LCS_OP_CFG: {

            nodeMap.nodeState = NS_CONFIG;
            if ( nodeMap.lcsMsgCallback != nullptr ) 
                nodeMap.lcsMsgCallback( msg, nodeMap.lcsMsgCallBackUdata );

        } break;

        case LCS_OP_BON: {

            writeDio( CDC_RN_ACTIVITY_LED, true );

            nodeMap.nodeState = NS_OPERATE;
            if ( nodeMap.lcsMsgCallback != nullptr ) 
                nodeMap.lcsMsgCallback( msg, nodeMap.lcsMsgCallBackUdata );

        } break;

        case LCS_OP_BOF: {

            writeDio( CDC_RN_ACTIVITY_LED, false );

            nodeMap.nodeState = NS_HALTED;
            if ( nodeMap.lcsMsgCallback != nullptr ) 
                nodeMap.lcsMsgCallback( msg, nodeMap.lcsMsgCallBackUdata );

        } break;

        case LCS_OP_NCOL: {

            writeDio( CDC_RN_ACTIVITY_LED, false );

            nodeMap.nodeState = NS_COLLISION;
            if ( nodeMap.lcsMsgCallback != nullptr ) 
                nodeMap.lcsMsgCallback( msg, nodeMap.lcsMsgCallBackUdata );

        } break;

        case LCS_OP_RESET: {

            uint16_t npId = (( msg[1] << 8 ) + msg[2] );
            reset( npId );

        } break;

        case LCS_OP_SET_NID: {

            uint16_t nodeId   = ( msg[1] << 8 ) + msg[2];
            uint32_t nodeUID  = ((uint32_t) msg[3] << 24 ) +
                        ((uint32_t) msg[4] << 16 ) +
                        ((uint32_t) msg[5] <<  8 ) +
                        ((uint32_t) msg[6] );

            if ( nodeUID == nodeMap.nodeUID ) {

                if ( nodeMap.nodeState == NS_CONFIG ) {

                    nodeMap.nodeId = nodeId;
                    
                    uint8_t rStat = 
                        rtNvmPutWord( NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId ), 
                                      nodeId );

                    sendAck( nodeMap.nodeId, nodeId );
                }
                else sendErr( nodeId, ERR_NODE_NOT_CONFIG_STATE, 0, 0 );
            }

        } break;
    }
}

//----------------------------------------------------------------------------------------
// "handleMsgGetNode" processes an incoming GET message for a node or port attribute. 
// We construct the reply message with the requested data.
//
//----------------------------------------------------------------------------------------
void handleMsgGetNode( uint8_t *msg ) {

    uint16_t npId =  (( msg[1] << 8 ) + msg[2] );

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t   item  = msg[3];
        uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
        uint8_t   ret   = nodeGet( npId, item, &arg1 );

        if ( ret == NO_ERR )  sendRepNode( nodeMap.nodeId, npId, item, arg1, 0 );
        else                  sendErr( nodeMap.nodeId, npId, ret, 0, 0 );
    }
}

//----------------------------------------------------------------------------------------
// "handleMsgPutNode" processes an incoming PUT message for a node or port attribute. 
// We update the data and send a confirmation.
//
//----------------------------------------------------------------------------------------
void handleMsgPutNode( uint8_t *msg ) {

    uint16_t npId =  (( msg[1] << 8 ) + msg[2] );

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t   item  = msg[3];
        uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
        uint8_t   ret   = nodeSet( npId, item, arg1 );

        if ( ret == NO_ERR )  sendAck( nodeMap.nodeId, npId );
        else                  sendErr( nodeMap.nodeId, npId, ret, arg1, 0 );
    }
}

//----------------------------------------------------------------------------------------
// "handleMsgRepNode" processes the answer to a previously sent node query. The 
// incoming message will only result in an action when we have an outstanding request 
// for that node. That is, this handler will only be called when the we passed the 
// outstanding reply map check done before. The outstanding request table was already 
// cleared. All we do is to route the reply messages to the callback for the port. It
// is up to the firmware programmer to analyze for what and whom the reply really is. 
//
//----------------------------------------------------------------------------------------
void handleMsgRepNode( uint8_t *msg ) {

    uint16_t  npId    = (( msg[1] << 8 ) + msg[2] );
    uint8_t   item    = msg[3];
    uint16_t  arg1    = ( msg[4] << 8 ) + msg[5];
    uint16_t  arg2    = ( msg[6] << 8 ) + msg[7];
    
    LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];

    if ( pPtr -> repCallback != nullptr ) 
        pPtr -> repCallback( npId, item, arg1, arg2, LCS_OK, pPtr ->repCallBackUdata );
}

//----------------------------------------------------------------------------------------
// "handleMsgReqNode" processes an incoming request for a node or port. The REQ 
// message request will result in invoking the register firmware callback. We send a
// a reply or error message.
//
// ??? we need to get the sender Id...
//----------------------------------------------------------------------------------------
void handleMsgReqNode( uint8_t *msg ) {

    uint16_t npId = (( msg[1] << 8 ) + msg[2] );

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t   item  = msg[3];
        uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
        uint16_t  arg2  = ( msg[6] << 8 ) + msg[7];
        uint8_t   ret   = nodeReq( npId, item, &arg1 );

        if ( ret == NO_ERR )  sendRepNode( nodeMap.nodeId, npId, item, arg1, arg2 );
        else                  sendErr( nodeMap.nodeId, npId, ret );
    }
}

//----------------------------------------------------------------------------------------
// "handleMsgEvent" deals with the event messages for inbound ports. If the event 
// is configured in the event map, all bits set in the eventMask will result in 
// recording the event data and the optional future time stamp when the event should
// result in a callback. The actual event processing is done in the event processing
// routine, which will manage the timely invocation of the event callbacks. The event
// mask has a bit for each port. 
//
//----------------------------------------------------------------------------------------
void handleMsgEvent( uint8_t *msg ) {

    uint16_t  eventId = ( msg[3] << 8 ) + msg[4];
    int       index   = searchEvent( eventId );

    if ( index >= 0 ) {

        uint8_t     opCode          = msg[0];
        uint16_t    npId            = ( msg[1] * 256 ) + msg[2];
        uint16_t    eventData       = ( msg[5] * 256 ) + msg[6];
        uint8_t     eventAction     = PEA_EVENT_IDLE;
        uint32_t    ts              = getMillis( );
        uint16_t    eventMask       = eventMap.map[ index ].eventMask;

        switch ( opCode ) {

            case LCS_OP_EVT_ON:   eventAction = PEA_EVENT_ON;   break;
            case LCS_OP_EVT_OFF:  eventAction = PEA_EVENT_OFF;  break;
            case LCS_OP_EVT:      eventAction = PEA_EVENT_EVT;  break;
        }

        for ( int i = 0; i <= MAX_PORT_MAP_ENTRIES; i++ ) {

            LcsPortMapEntry *pPtr = &portMap.map[ i ];

            if (( pPtr -> flags & NPF_PORT_ENABLED                  ) &&
                ( pPtr -> flags & NPF_PORT_EVENT_HANDLING_ENABLED   ) &&
                ( pPtr -> eventCallback != nullptr                  )) {

                if ( eventMask & ( 1 << i )) {

                    pPtr -> eventNpId       = npId;
                    pPtr -> eventId         = eventId;
                    pPtr -> eventAction     = eventAction;
                    pPtr -> eventValue      = eventData;
                    pPtr -> eventTimeStamp  = ts + 
                                ( pPtr -> eventDelayTime * EVENT_DELAY_TICK_MILLIS );
                    pPtr -> flags           |= NPF_EVENT_PENDING;
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------
// We received a DCC subsystem message. These messages are handled solely by firmware, 
// which is typically the base station, a handheld, or a decoder alike device. All we
// do is to pass the message to the call back routine. One day, we could decode the 
// message a bit more and invoke more specialized callback.
//
//----------------------------------------------------------------------------------------
void handleMsgDccMgt( uint8_t *msg ) {

    if ( nodeMap.dccMsgCallback != NULL ) 
        nodeMap.dccMsgCallback( msg, nodeMap.dccMsgCallBackUdata );
}

//----------------------------------------------------------------------------------------
// Node State FAIL. This is the state after the node startup failed. We simply stay
// in this state.
//
//----------------------------------------------------------------------------------------
void handleNodeStateFail( ) {

    nodeMap.nodeState = NS_FAIL;
}

//----------------------------------------------------------------------------------------
// Node State Power FAIL. This is the state after when the node starts up after a
// power fail. We have this state so that the firmware programmer can take some 
// recovery action before the power goes away. 
//
//----------------------------------------------------------------------------------------
void handleNodeStatePfail( ) {

    if ( nodeMap.pfailCallback != nullptr ) {

        nodeMap.pfailCallback( nodeMap.nodeId, nodeMap.pfailCallBackUdata );
    }

    nodeMap.nodeState = NS_PFAIL;
}

//----------------------------------------------------------------------------------------
// Node state INIT. This is the first state after the initial library setup. The 
// runtime init call created all memory areas and initialized the data structures.
// After a successful runtime init call, the state is INIT and the firmware can 
// register the necessary callback functions and do other firmware specific work. 
// Eventually, the runtime start function is called. First, any drivers mapped to P1
// to P4 are sent a RESET request, so that any hardware initialization can be done. 
// Any other port with a registered callback is handled next. Each port with a
// successful return code will finally be enabled and the high water mark adjusted
// accordingly.
// 
// ??? is there anything that we will need to remember in NVM ? 
// ??? should all ports get a reset call ?
//----------------------------------------------------------------------------------------
void handleNodeStateInit( ) {

    uint8_t rStat = NO_ERR;

    setupDriverFunctions( );

    for ( int i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        if ( nodeMap.initCallback != nullptr ) {
                
            // ??? issue we need to invoke with the correct uData for ports...

            rStat = nodeMap.initCallback( buildNpId( nodeMap.nodeId, i, 0 ),
                                          nodeMap.initCallBackUdata );
                
            if ( rStat == NO_ERR ) {
                
                portMap.map[ i ].flags |= NPF_PORT_PRESENT;
                portMap.map[ i ].flags |= NPF_PORT_ENABLED;
                portMap.map[ i ].flags |= NPF_PORT_EVENT_HANDLING_ENABLED;

                portMap.mapHwm = i + 1;
            }
        }
    }

    if ( ! ( runtimeOptions & NPO_SKIP_NODE_ID_CONFIG )) {

        sendReqNodeId( 0, nodeMap.nodeUID, 0 ); // ??? fix ...
        timerVal  = getMillis( );

        nodeMap.nodeState = NS_REGISTER;
        return;
    } 
    
    nodeMap.nodeState = NS_OPERATE;
}

//----------------------------------------------------------------------------------------
// Node State REGISTER. This is the state after the INIT state when a nodeId setup
// was requested. We are waiting for a nodeId reply message. If there is a timely 
// reply message, we will handle the message reply and the node state will advance.
// If there is no timely reply, we will resubmit the request.
//
//----------------------------------------------------------------------------------------
void handleNodeStateRegister( ) {

    uint16_t senderNpId;
    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( receiveLcsMsg( &senderNpId, msg )) {

        case LCS_OP_REP_NID: handleMsgRepNid( msg );  break;
        case LCS_OP_RESET:   handleMsgLcsMgt( msg );  break;

        default: {

            if (( getMillis( ) - timerVal ) > NODE_SETUP_RETRY_TIMER_VAL_MS ) {

                sendReqNodeId( 0, nodeMap.nodeUID, 0 );
                timerVal = getMillis( );
             }
        }
    }
}

//----------------------------------------------------------------------------------------
// Node State COLLISION. This is the state after the node receiver routine 
// detected a nodeId collision. We will stay in this state and only react to 
// RESET and SET_NID messages.
//
//----------------------------------------------------------------------------------------
void handleNodeStateCollision( ) {

    uint16_t senderNpId;
    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( receiveLcsMsg( &senderNpId, msg )) {

        case LCS_OP_RESET:
        case LCS_OP_SET_NID:  handleMsgLcsMgt( msg ); break;
    }
}

//----------------------------------------------------------------------------------------
// Node State HALTED. The LCS communication bus was halted for all nodes. Note that
// the bus is still there, just not active. We just listen to the BON or RESET message
// to get going again.
//
//----------------------------------------------------------------------------------------
void handleNodeStateHalted( ) {

    uint16_t senderNpId;              
    uint8_t  msg[ MAX_LCS_MSG_SIZE ];

    switch ( receiveLcsMsg( &senderNpId,  msg )) {

        case LCS_OP_BON:
        case LCS_OP_RESET: handleMsgLcsMgt( msg ); break;
    }
}

//----------------------------------------------------------------------------------------
// Node State CONFIG. A node can be placed into configuration state. We process 
// any LCS message, handle the periodic tasks registered and port events that 
// may have been received. Note that we just listen to messages valid for that 
// mode and invoke the respective handler. All other messages are ignored.
//
//----------------------------------------------------------------------------------------
void handleNodeStateConfig( ) {

    uint16_t senderNpId;
    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( receiveLcsMsg( &senderNpId, msg )) {

        case LCS_OP_OPS:
        case LCS_OP_RESET:
        case LCS_OP_BON:
        case LCS_OP_BOF:
        case LCS_OP_ACK:
        case LCS_OP_ERR:
        case LCS_OP_SET_NID:
        case LCS_OP_NCOL:           handleMsgLcsMgt( msg );     break;

        case LCS_OP_NODE_GET:       
        case LCS_OP_NODE_GET2:      handleMsgGetNode( msg );    break;

        case LCS_OP_NODE_SET:       
        case LCS_OP_NODE_SET2:      handleMsgPutNode( msg );    break;

        case LCS_OP_NODE_REQ:
        case LCS_OP_NODE_REQ2:      handleMsgReqNode( msg );    break;

        case LCS_OP_NODE_REP:       
        case LCS_OP_NODE_REP2:      handleMsgRepNode( msg );    break;
    }

    handlePeriodicTasks( );
    handleNodePortEvents( );
    handlePendingReqTimeouts( );
}

//----------------------------------------------------------------------------------------
// Node State OPERATIONS. Most of the time the node state is in operations mode. 
// We process any LCS message,  handle the periodic tasks registered and port events
// that may have been received. Note that we just listen to messages valid for that
// mode and invoke the respective handler. All other messages are ignored.
//
//----------------------------------------------------------------------------------------
void handleNodeStateOperations( ) {

    uint16_t senderNpId;
    uint8_t msg [ MAX_LCS_MSG_SIZE ];

    switch ( receiveLcsMsg( &senderNpId, msg )) {

        case LCS_OP_CFG:
        case LCS_OP_RESET:
        case LCS_OP_BON:
        case LCS_OP_BOF:
        case LCS_OP_ACK:
        case LCS_OP_ERR:
        case LCS_OP_REQ_NID:
        case LCS_OP_NCOL:           handleMsgLcsMgt( msg );     break;

        case LCS_OP_NODE_GET:       handleMsgGetNode( msg );    break;
        case LCS_OP_NODE_SET:       handleMsgPutNode( msg );    break;
        case LCS_OP_NODE_REQ:       handleMsgReqNode( msg );    break;

        case LCS_OP_NODE_REP:       
        case LCS_OP_NODE_REP2:      handleMsgRepNode( msg );    break;
        
        case LCS_OP_EVT_ON:
        case LCS_OP_EVT_OFF:
        case LCS_OP_EVT:            handleMsgEvent( msg );      break;

        case LCS_OP_REQ_LOC:
        case LCS_OP_REL_LOC:
        case LCS_OP_REP_LOC:
        case LCS_OP_SET_LCON:
        case LCS_OP_KEEP_LOC:
        case LCS_OP_SET_LSPD:
        case LCS_OP_SET_LMOD:
        case LCS_OP_LOC_FON:
        case LCS_OP_LOC_FOF:

        case LCS_OP_SET_CVM:
        case LCS_OP_REQ_CVS:
        case LCS_OP_REP_CVS:
        case LCS_OP_SET_CVS:

        case LCS_OP_TON:
        case LCS_OP_TOF:
        case LCS_OP_ESTP:

        case LCS_OP_SEND_DCC3:
        case LCS_OP_SEND_DCC4:
        case LCS_OP_SEND_DCC5:
        case LCS_OP_SEND_DCC6:

        case LCS_OP_DCC_ACK:
        case LCS_OP_DCC_ERR:        handleMsgDccMgt( msg );      break;
    }

    handlePeriodicTasks( );
    handleNodePortEvents( );
    handlePendingReqTimeouts( );
}

//----------------------------------------------------------------------------------------
// "handleNodeState" is the main routine of the node activity processing. It is the 
// method called after all setup is done. Running in a loop, the primary function is
// to handle the activities according to the node state. The run loop also processes 
// the serial commands. Note that this function will not return.
//
//----------------------------------------------------------------------------------------
void handleNodeState( ) {

    while ( true ) {

        watchDogUpdate( ); 
        handleSerialCommand( );

        switch ( nodeMap.nodeState ) {

            case NS_FAIL:       handleNodeStateFail( );         break;
            case NS_PFAIL:      handleNodeStatePfail( );        break;
            case NS_REGISTER:   handleNodeStateRegister( );     break;
            case NS_INIT:       handleNodeStateInit( );         break;
            case NS_COLLISION:  handleNodeStateCollision( );    break;
            case NS_HALTED:     handleNodeStateHalted( );       break;
            case NS_CONFIG:     handleNodeStateConfig( );       break;
            case NS_OPERATE:    handleNodeStateOperations( );   break;
            default: ;
        }
    }
}

} // namespace

//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file. These routines are callable 
// from the user firmware and thus need to check whether the library was already 
// initialized.
//
//----------------------------------------------------------------------------------------
namespace LCS { 

//----------------------------------------------------------------------------------------
// General callback registration functions.
//
//      INIT        -   Callback invoked for each port when the node starts, i.e. 
//                      when the firmware calls "startRuntime".
//
//      PFAIL       -   Called when we are about to loose power. Time for saving 
//                      important data to NVM.
//
//      LCS_MSG -       Callback for general LCS messages.
//
//      DCC_MSG -       Callback for LCS messages that are directed to the 
//                      DCC subsystem.
//
//      CMD     -       Callback for command input that is not recognized as a 
//                      LCS command.
//
//      EVENT   -       Callback when an event is received that the node/port
//                      is interested in.
//
//      REQ     -       Callback when REQ item message was received that the 
//                      node/port registered for.
//
//      REP     -       Callback for a reply message for a previous request
//                      that the node/port registered for.
//
//      TASK    -       Callback for a period task.
//
//----------------------------------------------------------------------------------------
uint8_t registerInitCallback( LcsInitCallback functionId, void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    nodeMap.initCallback = functionId;
    return ( LCS_OK );
}

uint8_t registerPfailCallback( LcsInitCallback functionId, void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    nodeMap.pfailCallback = functionId;
    return ( LCS_OK );
}

uint8_t registerLcsMsgCallback( LcsMsgCallback functionId, void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    nodeMap.lcsMsgCallback = functionId;
    return ( LCS_OK );
}

uint8_t registerDccMsgCallback( LcsMsgCallback functionId, void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    nodeMap.dccMsgCallback = functionId;
    return ( LCS_OK );
}

uint8_t registerCmdCallback( LcsCmdCallback functionId, void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    nodeMap.cmdLineCallback = functionId;
    return ( LCS_OK );
}

uint8_t registerEventCallback( LcsEventCallback functionId, 
                               uint16_t portMask, 
                               void *uData ) {

   if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    for ( int i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        if ( portMask & ( 1 << i )) portMap.map[ i ].eventCallback = functionId;
    }

    return ( LCS_OK );
}

uint8_t registerReqCallback( LcsReqCallback functionId, 
                             uint16_t portMask, 
                             void *uData ) {

   if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    for ( int i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        if ( portMask & ( 1 << i )) portMap.map[ i ].reqCallback = functionId;
    }

    return ( LCS_OK );
}

uint8_t registerRepCallback( LcsRepCallback functionId, 
                             uint16_t portMask, 
                             void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    for ( int i = 0; i <= MAX_PORT_MAP_ENTRIES; i++ ) {

        if ( portMask & ( 1 << i )) portMap.map[ i ].repCallback = functionId;
    }

    return ( LCS_OK );
}

uint8_t registerTaskCallback( LcsTaskCallback task, 
                              uint32_t interval, 
                              void *uData ) {

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );

    if ( taskMap.mapHwm < MAX_TASK_MAP_ENTRIES ) {

        LcsPTaskMapEntry *tPtr = & taskMap.map[ taskMap.mapHwm ];

        tPtr -> task       = task;
        tPtr -> uData      = nullptr;
        tPtr -> interval   = interval;
        tPtr -> timeStamp  = getMillis( );
        taskMap.mapHwm ++;
        return ( LCS_OK );

    } else return ( ERR_TASK_MAP_SIZE_EXCEEDED );
}

//----------------------------------------------------------------------------------------
// "localMsgEvent" is called by the event message send routines to cover the case 
// where we send an event and we detect it needs to also be broadcasted to other 
// ports on the same node. In other words, the nodeId of the sending node and our
// node are the same. Care needs to be taken in the port callback that we do not
// end up in an endless loop. A callback that sends an event to which itself is 
// registered for would lead to a loop. In other words, you cannot send an event
// to which you are registered for.
//
// ??? should this be covered in the general issue of a node sending to itself ?
//----------------------------------------------------------------------------------------
uint8_t localMsgEvent( uint8_t *msg ) {

    if ( nodeMap.nodeState != NS_OPERATE ) return ( ERR_LIB_NOT_INITIALIZED );
    handleMsgEvent( msg );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "startRuntime" is the main routine of the node activity processing. We check 
// that the library was initialized properly and mark the nodeMap flag "READY". 
// And then we are in business.
//
//----------------------------------------------------------------------------------------
uint8_t startRuntime( ) {

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_SETUP ))
        printf( "Start LCS runtime\n");

    if ( nodeMap.nodeState != NS_INIT ) return ( ERR_LIB_NOT_INITIALIZED );
    
    handleNodeState( );
    return ( LCS_OK );
}

}; // namespace LCS
