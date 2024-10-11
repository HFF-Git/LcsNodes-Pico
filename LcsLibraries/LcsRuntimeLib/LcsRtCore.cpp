//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - runtime core.
//
//------------------------------------------------------------------------------------------------------------
// The file contains the runtime core routines. They implement the node state machine that reacts to messages
// and advances according to state. The routines are called from the runtime loop in the setup file.
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
// External declaration to global structures defined in "LcsRtSetup".
//
//-----------------------------------------------------------------------------------------------------------
extern uint16_t                 debugMask;
extern LCS::LcsCdcDesc          cdcMap;
extern LCS::LcsNodeMap          nodeMap;
extern LCS::LcsPortMap          portMap;
extern LCS::LcsEventMap         eventMap;
extern LCS::LcsCallbackMap      callbackMap;
extern LCS::LcsTaskMap          taskMap;
extern LCS::LcsPendingReqMap    pendingReqMap;
extern LCS::LcsDrvMap           drvMap;
extern LCS::LcsMsgBusCAN        *msgBus;

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Local constants and helper functions.
//
//------------------------------------------------------------------------------------------------------------
const uint32_t  NODE_SETUP_RETRY_TIMER_VAL_MS   = 1000L;
uint32_t        timerVal                        = 0L;

bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

  return (( val >= lower ) && ( val <= upper ));
}

uint16_t buildNpId( uint16_t nodeId, uint16_t portId ) {

    return(( nodeId << 4 ) | ( portId & 0xF ));
}

uint16_t nodeId( uint16_t npId ) {

    return( npId >> 4 );
}

uint16_t portId( uint16_t npId ) {

    return( npId & 0xF );
}

} // namespace

//------------------------------------------------------------------------------------------------------------
//  The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS { 

//------------------------------------------------------------------------------------------------------------
// General callback registration functions. They just set the function Id field. Straightforward.
//
//------------------------------------------------------------------------------------------------------------
void registerLcsMsgCallback( LcsMsgCallback functionId ) {

  callbackMap.lcsMsgCallback = functionId;
}

void registerDccMsgCallback( LcsMsgCallback functionId ) {

  callbackMap.dccMsgCallback = functionId;
}

void registerCmdCallback( LcsCmdCallback functionId ) {

  callbackMap.cmdLineCallback = functionId;
}

void registerEventCallback( LcsEventCallback functionId ) {

  callbackMap.eventCallback = functionId;
}

void registerInitCallback( LcsInitCallback functionId ) {

    callbackMap.initCallback = functionId;
}

void registerResetCallback( LcsInitCallback functionId ) {

    callbackMap.resetCallback = functionId;
}

void registerPfailCallback( LcsInitCallback functionId ) {

    callbackMap.pfailCallback = functionId;
}

void registerReqCallback( LcsReqCallback functionId ) {

    callbackMap.reqCallback = functionId;
}

void registerRepCallback( LcsRepCallback functionId ) {
    
    callbackMap.repCallback = functionId;
}

//-----------------------------------------------------------------------------------------------------------
// The core library features a very simple periodic task system. This routine adds a task callback to the
// pTaskMap. We only add entries, never remove them. A high water mark is used to record the highest entry
// used, so that processing will not run through empty entries.
//
// ??? perhaps this needs to be reworked to use HW driven timers. 
//-----------------------------------------------------------------------------------------------------------
uint8_t registerTaskCallback( LcsTaskCallback task, uint32_t interval ) {

    if ( nodeMap.taskMapHwm < MAX_TASK_MAP_ENTRIES ) {

        taskMap.map[ nodeMap.taskMapHwm ].task       = task;
        taskMap.map[ nodeMap.taskMapHwm ].interval   = interval;
        taskMap.map[ nodeMap.taskMapHwm ].timeStamp  = CDC::getMillis( );
        nodeMap.taskMapHwm ++;
        return ( ALL_OK );

    } else return ( ERR_TASK_MAP_SIZE_EXCEEDED );
}

//------------------------------------------------------------------------------------------------------------
// "handleNodePortEvents" will be called for processing inbound port events on each loop iteration. Note that
// it does not matter where the events came from, i.e. whether another node sends an event or the event was
// created by a firmware call on this node. The event callback can be delayed with a timer value.
//
//------------------------------------------------------------------------------------------------------------
void handleNodePortEvents( ) {

    if ( callbackMap.eventCallback == nullptr ) return;

    uint32_t ts = CDC::getMillis( );

    for ( uint8_t i = 0; i < MAX_PORT_MAP_ENTRIES; i ++ ) {

        LcsPortMapEntry *pPtr = & portMap.map[ i ];

        if (( pPtr -> flags & PF_PORT_ENABLED                 ) &&
            ( pPtr -> flags & PF_PORT_EVENT_HANDLING_ENABLED  ) &&
            ( pPtr -> flags & PF_EVENT_PENDING                )) {

            if ( ts > pPtr -> eventTimeStamp ) {

                callbackMap.eventCallback(  buildNpId( pPtr -> eventNodeId, i + 1 ),
                                            pPtr -> eventId,
                                            pPtr -> eventAction,
                                            pPtr -> eventValue );
            }

            pPtr -> flags &= ~ PF_EVENT_PENDING;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// "handlePeriodicTasks" is called from the core library main processing loop. The idea is that there is a 
// lot of periodic processing that needs to be one by any firmware implementation. Instead of the firmware
// developer writing its own handler, there is a crude method that just samples the timestamps and interval 
// and triggers the callback then the interval is reached. Note that this is not very accurate from a timing
// perspective but will do for simple periodic processing.
//
//------------------------------------------------------------------------------------------------------------
void handlePeriodicTasks( ) {

    uint32_t ts = CDC::getMillis( );

    for ( int i = 0; i < nodeMap.taskMapHwm; i++ ) {

        LcsPTaskMapEntry *thisEntry = &taskMap.map[ i ];

        if (( ts - thisEntry -> timeStamp ) > thisEntry -> interval ) {

            if ( thisEntry -> task != nullptr ) thisEntry -> task( );
            thisEntry -> timeStamp = ts;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgRepNid" handles the message that the configuring node sends to our node in response to a nodeId
// setup request. If the UID matches, the message is for our node and we update our nodeId accordingly in 
// MEM and NVM. The next node state is OPERATE.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgRepNid( uint8_t *msg ) {

    uint16_t nodeId   = ( msg[1] << 8 ) + msg[2];
    uint32_t nodeUID  = ((uint32_t) msg[3] << 24 ) +
                         ((uint32_t) msg[4] << 16 ) +
                         ((uint32_t) msg[5] << 8 ) +
                         msg[6];

    if ( nodeUID == nodeMap.nodeUID ) {

        if ( nodeMap.nodeId != nodeId ) {
            
            nodeMap.nodeId = nodeId;
            uint8_t rStat = rtNvmPutWord( NVM_NODE_MAP_START + offsetof( LcsNodeMap, nodeId ), nodeId );
        }

        nodeMap.nodeState = NS_OPERATE;
    }
}

//------------------------------------------------------------------------------------------------------------
// LCS management deals with messages concerning the general LCS management. If there is a callback defined
// it will be invoked. Then the node state is changed accordingly. Most updates are just to the MEM nodeMap.
// In addition, the READY and ACTIVITY LEDs are set.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgLcsMgt( uint8_t *msg ) {

    switch ( msg[ 0 ] ) {

        case LCS_OP_OPS: {

            nodeMap.nodeState = NS_OPERATE;
            if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

        } break;

        case LCS_OP_CFG: {

            nodeMap.nodeState = NS_CONFIG;
            if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

        } break;

        case LCS_OP_BON: {

            CDC::writeDio( cdcMap.cfg.READY_LED_PIN, true );
            nodeMap.nodeState = NS_OPERATE;
            if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

        } break;

        case LCS_OP_BOF: {

            CDC::writeDio( cdcMap.cfg.READY_LED_PIN, false );
            nodeMap.nodeState = NS_HALTED;
            if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

        } break;

        case LCS_OP_NCOL: {

            CDC::writeDio( cdcMap.cfg.READY_LED_PIN, false );
            CDC::writeDio( cdcMap.cfg.ACTIVE_LED_PIN, true );
            nodeMap.nodeState = NS_COLLISION;
            if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

        } break;

        case LCS_OP_RESET: {

            uint16_t npId = (( msg[1] << 8 ) + msg[2] );
            uint8_t rStat = resetNode( npId );

            if ( rStat == ALL_OK ) LCS::sendAck( npId );
            else                   LCS::sendErr( npId, rStat, 0, 0 );

        } break;

        case LCS_OP_SET_NID: {

            uint16_t nodeId   = ( msg[1] << 8 ) + msg[2];
            uint32_t nodeUID  = ((uint32_t) msg[3] << 24 ) + ((uint32_t) msg[4] << 16 ) +
                                  ((uint32_t) msg[5] << 8 ) + msg[6];

            if ( nodeUID == nodeMap.nodeUID ) {

                if ( nodeMap.nodeState == NS_CONFIG ) {

                    if ( nodeId != nodeMap.nodeId ) nodeMap.nodeId = nodeId;
                    uint8_t rStat = rtNvmPutWord( NVM_NODE_MAP_START + offsetof( LcsNodeMap, nodeId ), nodeId );

                    sendAck( nodeId );
                }
                else sendErr( nodeId, ERR_NODE_NOT_CONFIG_STATE, 0, 0 );
            }

        } break;
    }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgGetNode" processes an incoming GET message for a node or port attribute. We construct the 
// reply message with the requested data.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgGetNode( uint8_t *msg ) {

    uint16_t npId =  (( msg[1] << 8 ) + msg[2] );

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t   item  = msg[3];
        uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
        uint16_t  arg2  = ( msg[6] << 8 ) + msg[7];
        uint8_t   ret   = nodeGet( npId, item, &arg1, &arg2 );

        if ( ret == ALL_OK )  sendRepNode( npId, item, arg1, arg2 );
        else                  sendErr( npId, ret, 0, 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgPutNode" processes an incoming PUT message for a node or port attribute. We update the data
// and send a confirmation.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgPutNode( uint8_t *msg ) {

    uint16_t npId =  (( msg[1] << 8 ) + msg[2] );

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t   item  = msg[3];
        uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
        uint16_t  arg2  = ( msg[6] << 8 ) + msg[7];
        uint8_t   ret   = nodePut( npId, item, arg1, arg2 );

        if ( ret == ALL_OK )  sendAck( npId );
        else                  sendErr( npId, ret, 0, 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgRepNode" processes the answer to a previously sent node query. The incoming message will only
// result in an action when we have an outstanding request for that node. That is, this handler will only
// be called when the we passed the outstanding reply map check done before. All reply messages are routed
// to this one callback. It is up to the firmware programmer to analyze for what and whom the reply really
// is.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgRepNode( uint8_t *msg ) {

    uint16_t  npId    = (( msg[1] << 8 ) + msg[2] );
    uint8_t   item    = msg[3];
    uint16_t  arg1    = ( msg[4] << 8 ) + msg[5];
    uint16_t  arg2    = ( msg[6] << 8 ) + msg[7];

    if ( callbackMap.repCallback != nullptr ) callbackMap.repCallback( npId, item, &arg1, &arg2 );
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgReqNode" processes an incoming request for a node or port. The REQ message request will result
// in invoking the register firmware callback. We send a confirmation message.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgReqNode( uint8_t *msg ) {

    uint16_t npId = (( msg[1] << 8 ) + msg[2] );

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t   item  = msg[3];
        uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
        uint16_t  arg2  = ( msg[6] << 8 ) + msg[7];
        uint8_t   ret   = nodeReq( npId, item, &arg1, &arg2 );

        if ( ret == ALL_OK )  sendAck( npId );
        else                  sendErr( npId, ret, 0, 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgEvent" deals with the event messages for inbound ports. For all matching events in the eventMap,
// the respective port map entry fields are set. The searchEvent function will return us the first matching
// entry in the sorted event map, if any. From there, we just hop along the eventMap until the eventId does
// not match any longer. All we do in this routine is to record the event data and the optional future time
// stamp when the event should result in a callback. The actual event processing is done in the port event 
// processing routine, which will manage the timely invocation of the event callbacks.
//
// Note that we also are called from the event sending routine because another port on our node could be 
// interested in this event. It is up to the firmware programmer to ensure that a port does send itself an
// event and may trigger an infinite loop.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgEvent( uint8_t *msg ) {

    uint16_t  eventId = ( msg[3] * 256 ) + msg[4];
    int       index   = searchEvent( eventId );

    if ( index >= 0 ) {

        uint8_t   opCode            = msg[0];
        uint16_t  nodeId            = ( msg[1] * 256 ) + msg[2];
        uint16_t  eventData         = ( msg[5] * 256 ) + msg[6];
        uint8_t   eventAction       = PEA_EVENT_IDLE;
        uint32_t  ts                = CDC::getMillis( );

        switch ( opCode ) {

            case LCS_OP_EVT_ON:   eventAction = PEA_EVENT_ON;   break;
            case LCS_OP_EVT_OFF:  eventAction = PEA_EVENT_OFF;  break;
            case LCS_OP_EVT:      eventAction = PEA_EVENT_EVT;  break;
        }

        while (( index < nodeMap.eventMapHwm ) && ( eventMap.map[ index ].eventId == eventId )) {

            LcsPortMapEntry *pPtr = &portMap.map[ index ];

            if (( pPtr -> flags & PF_PORT_ENABLED                  ) &&
                ( pPtr -> flags & PF_PORT_EVENT_HANDLING_ENABLED   )) {

                pPtr -> eventNodeId     = nodeId;
                pPtr -> eventId         = eventId;
                pPtr -> eventAction     = eventAction;
                pPtr -> eventValue      = eventData;
                pPtr -> eventTimeStamp  = ts + ( pPtr -> eventDelayTime * EVENT_DELAY_TICK_MILLIS );
                pPtr -> flags           |= PF_EVENT_PENDING;
            }

            index++;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// We received a DCC subsystem message. These messages are handler solely by firmware, which is typically
// the base station, a handheld, or a decoder alike device. All we do is to pass the message to the call 
// back routine. One day, we could decode the message a bit more and invoke more specialized callback.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgDccMgt( uint8_t *msg ) {

    if ( callbackMap.dccMsgCallback != NULL ) callbackMap.dccMsgCallback( msg );
}

//------------------------------------------------------------------------------------------------------------
// Node state INIT. This is the first state after the initial library setup. The runtime init call created
// all memory areas and initialized the data structures. After a successful init call, the state is INIT and 
// the firmware programmer can register the necessary callback functions and do other firmware specific work. 
// Eventually, the runtime loop method is called. If the "init" option is set, the node init and port init 
// callback routine will be invoked. If the nodeId validation option is set, the node will request a nodeId 
// and enter the state SETUP. Otherwise the next state is OPERATE.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateInit( ) {

    if ( nodeMap.nodeOptions & ( ~ NOPT_SKIP_NODE_INIT_STEP )) {

        if ( callbackMap.initCallback != nullptr ) callbackMap.initCallback( nodeMap.nodeId << 4 );

        for ( uint8_t i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

            if ( callbackMap.initCallback != nullptr ) 
                callbackMap.initCallback(( nodeMap.nodeId << 4 ) | i + 1 );

            portMap.map[ i ].flags |= PF_PORT_ENABLED;
            portMap.map[ i ].flags |= PF_PORT_EVENT_HANDLING_ENABLED;
        }
    }

     if ( ! ( nodeMap.nodeOptions & NOPT_SKIP_NODE_ID_CONFIG )) {

        sendReqNodeId( nodeMap.nodeId, nodeMap.nodeUID, 0 );
        timerVal  = CDC::getMillis( );
        nodeMap.nodeState = NS_REGISTER;

    } else nodeMap.nodeState = NS_OPERATE;
}

//------------------------------------------------------------------------------------------------------------
// Node State FAIL. This is the state after the node startup failed. 
//
// ??? would we ever come here ? if the INIT failed, the firmware programmer should do what ?
//------------------------------------------------------------------------------------------------------------
void handleNodeStateFail( ) {


}

//------------------------------------------------------------------------------------------------------------
// Node State Power FAIL. This is the state after when the node starts up after a power fail. We have this
// state so that the firmware programmer can take some recovery action if desired. 
//
// The "initRuntime" routine will return a PFAIL status instead of ALL_OK when we restarted after a power 
// fail. OR we could return ALL_OK but have a flag set that we restarted from a power fail.
//
// The power fail callback is used to actually do the work BEFORE we have the power failure.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStatePfail( ) {

  
}

//------------------------------------------------------------------------------------------------------------
// Node State REGISTER. This is the state after the INIT state when a nodeId setup was requested. We are
// waiting for a nodeId reply message. If there is a timely reply message, we will handle the message reply
// and the node state will advance. If there is no timely reply, we will resubmit the request.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateRegister( ) {

    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( msgBus -> receiveLcsMsg( msg )) {

        case LCS_OP_REP_NID: handleMsgRepNid( msg );  break;
        case LCS_OP_RESET:   handleMsgLcsMgt( msg );  break;

        default: {

            if (( CDC::getMillis( ) - timerVal ) > NODE_SETUP_RETRY_TIMER_VAL_MS ) {

                sendReqNodeId( nodeMap.nodeId, nodeMap.nodeUID, 0 );
                timerVal = CDC::getMillis( );
             }
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// Node State COLLISION. This is the state after the node receiver routine detected a nodeId collision. We
// will stay in this state and only react to RESET and SET_NID messages.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateCollision( ) {

    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( msgBus -> receiveLcsMsg( msg )) {

        case LCS_OP_RESET:
        case LCS_OP_SET_NID:  handleMsgLcsMgt( msg ); break;
    }
}

//------------------------------------------------------------------------------------------------------------
// Node State HALTED. The LCS communication bus was halted for all nodes. Note that the bus is still there,
// just not active. We just listen to the BON or RESET message to get going again.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateHalted( ) {

    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( msgBus -> receiveLcsMsg( msg )) {

        case LCS_OP_BON:
        case LCS_OP_RESET: handleMsgLcsMgt( msg ); break;
    }
}

//------------------------------------------------------------------------------------------------------------
// Node State CONFIG. A node can be placed into configuration state. We first handle the command and general
// loop callback and then just listen to messages valid for that mode and invoke the respective handler.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateConfig( ) {

    uint8_t msg[ MAX_LCS_MSG_SIZE ];

    switch ( msgBus -> receiveLcsMsg( msg )) {

        case LCS_OP_OPS:
        case LCS_OP_RESET:
        case LCS_OP_BON:
        case LCS_OP_BOF:
        case LCS_OP_ACK:
        case LCS_OP_ERR:
        case LCS_OP_SET_NID:
        case LCS_OP_NCOL:           handleMsgLcsMgt( msg );               break;

        case LCS_OP_GET_NODE:       handleMsgGetNode( msg );              break;
        case LCS_OP_REP_NODE:       handleMsgRepNode( msg );              break;
        case LCS_OP_REQ_NODE:       handleMsgReqNode( msg );              break;
    }
}

//------------------------------------------------------------------------------------------------------------
// Node State OPERATIONS. Most of the time the node state is in operations mode. We first handle the command
// and  general loop callback and then just listen to messages valid for that mode and invoke the respective
// handler.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateOperations( ) {

    uint8_t msg [ MAX_LCS_MSG_SIZE ];

    switch ( msgBus -> receiveLcsMsg( msg )) {

        case LCS_OP_CFG:
        case LCS_OP_RESET:
        case LCS_OP_BON:
        case LCS_OP_BOF:
        case LCS_OP_ACK:
        case LCS_OP_ERR:
        case LCS_OP_REQ_NID:
        case LCS_OP_NCOL:           handleMsgLcsMgt( msg );             break;

        case LCS_OP_GET_NODE:       handleMsgGetNode( msg );            break;
        case LCS_OP_PUT_NODE:       handleMsgPutNode( msg );            break;
        case LCS_OP_REP_NODE:       handleMsgRepNode( msg );            break;
        case LCS_OP_REQ_NODE:       handleMsgReqNode( msg );            break;

        case LCS_OP_EVT_ON:
        case LCS_OP_EVT_OFF:
        case LCS_OP_EVT:            handleMsgEvent( msg );              break;

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
        case LCS_OP_DCC_ERR:        handleMsgDccMgt( msg );               break;
    }
}

}; // namespace
