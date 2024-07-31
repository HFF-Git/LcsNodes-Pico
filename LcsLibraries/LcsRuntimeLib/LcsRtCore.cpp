//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - runtime core.
//
//------------------------------------------------------------------------------------------------------------
// The file contains the runtime core. It contains the library internal global variables and the routines
// to handle messages.
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
// The LcsCoreLib implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  using namespace LCS;

  //----------------------------------------------------------------------------------------------------------  
  // Debug and Trace support. Instead of conditional cimpilation, we will print debug messages based on the
  // settoin of the debiug level.
  //---------------------------------------------------------------------------------------------------------- 
  uint8_t debugLevel = 0;

  //----------------------------------------------------------------------------------------------------------
  // During node Id allocation, the node tries in periodic intervals to obtain a node ID.
  //
  //----------------------------------------------------------------------------------------------------------
  const uint32_t  NODE_SETUP_RETRY_TIMER_VAL_MS   = 1000L;
  uint32_t        timerVal                        = 0L;


  //----------------------------------------------------------------------------------------------------------
  // The node states. The node starts in the INIT state and once all is initialized and registered ends up in
  // the OPS or CFG mode.
  //
  //  NS_NIL            -
  //  NS_FAIL           -
  //  NS_INIT           -
  //  NS_REGISTER       -
  //  NS_COLLISION      -
  //  NS_HALTED         -
  //  NS_CONFIG         -
  //  NS_OPERATE        -
  //
  //----------------------------------------------------------------------------------------------------------
  enum NodeState : uint16_t {

    NS_NIL            = 0,
    NS_FAIL           = 1,
    NS_INIT           = 2,
    NS_REGISTER       = 3,
    NS_COLLISION      = 4,
    NS_HALTED         = 5,
    NS_CONFIG         = 6,
    NS_OPERATE        = 7
  };

  //----------------------------------------------------------------------------------------------------------
  // A little helper function to check a number range.
  //
  //----------------------------------------------------------------------------------------------------------
  inline bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
  }

}; // namespace


//------------------------------------------------------------------------------------------------------------
// The global structures declaration. There is first of all the "runtime maps" which holds all non-volatile
// node and port data from which the volatile copies are built. The "callback map" holds all registered
// callback function pointers. The "task map" holds all registered periodc task function pointers. The "driver
// map" holds the configured driver for an extension board, if there is any configured.
//
// The "nmv" and "msgBus" are the handles to the NVM hardware and the message bus interface. Finally there is
// the overall node state, which holds the current state of the node.
//
//------------------------------------------------------------------------------------------------------------
LCS::LcsCdcDesc                cdcMap;
LCS::LcsNodeMap                nodeMap;
LCS::LcsPortMap                portMap;
LCS::LcsEventMap               eventMap;

LCS::LcsCallbackMap            callbackMap;
LCS::LcsTaskMap                taskMap;
LCS::LcsPendingReqMap          pendingReqMap;
LCS::LcsDrvMap                 drvMap;

LCS::LcsMsgBusCAN              *msgBus       = nullptr;
uint16_t                       nodeState     = 0;


//------------------------------------------------------------------------------------------------------------
//
//
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

void registerCommandCallback( LcsCommandCallback functionId ) {

  callbackMap.cmdLineCallback = functionId;
}

void registerReqRepCallback( LcsItemReqRepCallback handler ) {

  callbackMap.itemReqRepCallback = handler;
}

void registerPortEventCallback( LcsPortEventCallback functionId ) {

  callbackMap.portEventCallback = functionId;
}

//------------------------------------------------------------------------------------------------------------
// Callback registration functions for node and port callbacks. They are stored in teh callback map, which
// allows separate callbacks for individual ports and so on. Note that these callbacks need tobe registered
// on each startup and node reset. We will not keep the callback map across these events.
//
//------------------------------------------------------------------------------------------------------------
uint8_t registerInitCallback( uint8_t portId, LcsInitCallback handler ) {

  if ( portId <= MAX_PORT_ID ) {

    callbackMap.map[ portId ].initCallback = handler;
    return ( ALL_OK );

  } else return ( ERR_INVALID_PORT_ID  );
}

uint8_t registerInfoCallback( uint8_t portId, LcsInfoItemCallback handler ) {

  if ( portId <= MAX_PORT_ID ) {

    callbackMap.map[ portId ].infoItemCallback = handler;
    return ( ALL_OK );

  } else return ( ERR_INVALID_PORT_ID  );
}

uint8_t registerCtrlCallback( uint8_t portId, LcsCtrlItemCallback handler ) {

  if ( portId <= MAX_PORT_ID ) {

    callbackMap.map[ portId ].ctrlItemCallback = handler;
    return ( ALL_OK );

  } else return ( ERR_INVALID_PORT_ID  );
}

//-----------------------------------------------------------------------------------------------------------
// The core library features a very simple periodic task system. This routine adds a task callback to the
// pTaskMap. We only add entries, never remove them. A high water mark is used to record the highest entry
// used, so that processing will not run through empty entries.
//
//-----------------------------------------------------------------------------------------------------------
uint8_t registerPeriodicTask( LcsTaskCallback task, uint32_t interval ) {

  LcsPTaskMapEntry *limit  = &taskMap.map[ MAX_TASK_MAP_ENTRIES ];

  if ( taskMap.hwm < limit ) {

    taskMap.hwm -> task       = task;
    taskMap.hwm -> interval   = interval;
    taskMap.hwm -> timeStamp  = CDC::getMillis( );
    taskMap.hwm ++;

    return ( ALL_OK );

  } else return ( ERR_TASK_MAP_SIZE_EXCEEDED );
}

//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t registerDriver( LcsDrv *drv ) {

  // ??? how about we just register them as needed?

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
// The high level driver functions. All they do is to locate the driver object and invoke the respective
// method. The driver routines implement a similar "item" scheme just like nodes and ports.
//
// idea: why not have the same item ranges as node/port ?
// this way we can hve a common set of items which also apply to drivers...
//------------------------------------------------------------------------------------------------------------
uint8_t drvInit( uint8_t boardId, uint16_t flags ) {

  if ( boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  return (( drv != nullptr ) ? drv -> init( flags ) : ERR_INVALID_BOARD_ID );
}

// ??? fix ....
uint8_t drvControl( uint8_t boardId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

  if ( boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  if ( drv == nullptr ) return ( ERR_INVALID_BOARD_ID );

  return ( drv -> control( item, arg1, arg2 ));
}

// ??? fix ....
uint8_t drvInfo( uint8_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

  if ( boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  if ( drv == nullptr ) return ( ERR_INVALID_BOARD_ID );

  return ( drv -> info( 0, item, arg1, arg2 ));
}

uint8_t drvRead( uint8_t boardId, uint8_t padId, uint16_t *arg ) {

  if (  boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  if ( drv == nullptr ) return ( ERR_INVALID_BOARD_ID );

  return ( drv -> read( padId, arg ));
}

uint8_t drvRead( uint8_t boardId, uint8_t padId, uint8_t *arg, uint8_t *len ) {

  if ( boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  if ( drv == nullptr ) return ( ERR_INVALID_BOARD_ID );

  return ( drv -> read( padId, arg, len ));
}

uint8_t drvWrite( uint8_t boardId, uint8_t padId, uint16_t arg ) {

  if ( boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  if ( drv == nullptr ) return ( ERR_INVALID_BOARD_ID );

  return ( drv -> write( padId, arg ));
}

uint8_t drvWrite( uint8_t boardId, uint8_t padId, uint8_t *arg, uint8_t len ) {

  if ( boardId >= MAX_BOARD_ID ) return ( ERR_INVALID_BOARD_ID );

  LcsDrv *drv = drvMap.map[ boardId - 1 ];
  if ( drv == nullptr ) return ( ERR_INVALID_BOARD_ID );

  return ( drv -> write( padId, arg, len ));
}


//------------------------------------------------------------------------------------------------------------
// "resetNode" restarts a node. We first rebuild the MEM areas from their NVM counterparts. Next, the optional
// init call back is invoked. Finally, all ports are resetted as well. If an error occurs, the node reports a
// fatal error and stops.
//
// ??? what exactly is resetted when called ? Are all data structures re-initialized ?
// ??? does the user need to do all the registration work ?
// ??? we either should have a "reset" callback or a flag on the init callback to distinguish them both.
//------------------------------------------------------------------------------------------------------------
uint8_t resetNode( ) {

  uint8_t rStat = ALL_OK;

  // setupNode( );

  if ( callbackMap.map[ 0 ].initCallback != nullptr ) {

    rStat = callbackMap.map[ 0 ].initCallback( nodeMap.id, 0, 0 );
  }

  if ( rStat == ALL_OK ) {

    for ( uint8_t i = 1; i <= MAX_PORT_MAP_ENTRIES; i++ ) {

      rStat = resetPort( i );
      if ( rStat != ALL_OK ) break;
    }
  }

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "resetPort" will restart an individual port. We first copy the initial state for the port from NVM and then
// invoke the optional init callback. This function also called when we reset the entire node.
//
// ??? what exactly is resetted when called ? Are all data structures re-initialized ?
// ??? does the user need to do all the registration work ?
// ??? we either should have a "reset" callback or a flag on the init callback to distinguish them both.
//------------------------------------------------------------------------------------------------------------
uint8_t resetPort( uint8_t portId ) {

  if ( ! isInRangeU( portId, MIN_PORT_ID, MAX_PORT_MAP_ENTRIES )) return ( ERR_INVALID_PORT_ID );

  /*
    nvm -> getNvmBytes( nodeMap.portMapStart + (( portId - 1 ) * sizeof( LcsPortMapEntry )),
                        (uint8_t *) & portMap[ portId - 1 ],
                        sizeof( LcsPortMapEntry ));
  */
  if ( callbackMap.map[ portId ].initCallback != nullptr )
    return ( callbackMap.map[ portId ].initCallback( nodeMap.id, portId, 0 ));
  else return ( ALL_OK );

}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void resetCallbackMap( ) {

  callbackMap.flags                 = 0;
  callbackMap.size                  = MAX_PORT_MAP_ENTRIES + 1; 

  callbackMap.lcsMsgCallback        = nullptr;
  callbackMap.dccMsgCallback        = nullptr;
  callbackMap.cmdLineCallback       = nullptr;
  callbackMap.portEventCallback     = nullptr;
  callbackMap.itemReqRepCallback    = nullptr;

  for ( int i = 0; i <= MAX_PORT_MAP_ENTRIES; i++ ) {
      
    callbackMap.map[ i ].initCallback     = nullptr;
    callbackMap.map[ i ].infoItemCallback = nullptr;
    callbackMap.map[ i ].ctrlItemCallback = nullptr;
  }
}



//------------------------------------------------------------------------------------------------------------
// There is a call back that if set will be called for processing inbound port events on each loop iteration.
//
//------------------------------------------------------------------------------------------------------------
void handleNodePortEvents( ) {

  uint32_t ts = CDC::getMillis( );

  for ( uint8_t i = 0; i < MAX_PORT_MAP_ENTRIES; i ++ ) {

    LcsPortMapEntry *pPtr = & portMap.map[ i ];

    if (( pPtr -> flags & PF_PORT_ENABLED                 ) &&
        ( pPtr -> flags & PF_PORT_EVENT_HANDLING_ENABLED  ) &&
        ( pPtr -> flags & ( ~ PF_PORT_EVENT_DIRECTION    )) &&
        ( pPtr -> flags & PF_EVENT_PENDING               )) {

      if ( ts > pPtr -> eventTimeStamp ) {

        if ( callbackMap.portEventCallback != nullptr ) {

          callbackMap.portEventCallback( pPtr -> nodeId,
                                         i + 1,
                                         pPtr -> eventId,
                                         pPtr -> eventAction,
                                         pPtr -> eventValue );
        }

        pPtr -> flags &= ~ PF_EVENT_PENDING;
      }
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// "handlePeriodicTasks" is called from the core library main processing loop. The idea is that there is a lot
// of periodic processing that needs to be one by any firmware implmentation. Instead of the firmware developer
// writing its own handler, there is a crude method that just samples the timestamps and interval and triggers
// the callback hen the interval is reached. Note that this is not very accurate from a timing perspective
// but will do for simple periodic processing.
//
//------------------------------------------------------------------------------------------------------------
void handlePeriodicTasks( ) {

  uint32_t ts = CDC::getMillis( );

  for ( LcsPTaskMapEntry *thisEntry = taskMap.map; thisEntry < taskMap.hwm; thisEntry ++ ) {

    if (( ts - thisEntry -> timeStamp ) > thisEntry -> interval ) {

      if ( thisEntry -> task != nullptr ) thisEntry -> task( );
      thisEntry -> timeStamp = ts;
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgRepNid" handles the message that the configuring node sends to our node in response to a nodeId
// setup request. If the UID matches, the message is for our node and we update our nodeId accordingly. The
// next node state is OPERATE.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgRepNid( uint8_t *msg ) {

  uint16_t nodeId   = ( msg[1] << 8 ) + msg[2];
  uint32_t nodeUID  = ((uint32_t) msg[3] << 24 ) +
                      ((uint32_t) msg[4] << 16 ) +
                      ((uint32_t) msg[5] << 8 ) +
                      msg[6];

  if ( nodeUID == nodeMap.uid ) {

    if ( nodeMap.id != nodeId ) nodeControl( nodeId, NPC_SET_NODE_ID, nodeId );
    nodeState = NS_OPERATE;
  }
}

//------------------------------------------------------------------------------------------------------------
// LCS management deals with messags concerning the general LCS management. If there is a callback defined
// it will be invoked. Then the node state is changed accordingly.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgLcsMgt( uint8_t *msg ) {

  switch ( msg[ 0 ] ) {

    case LCS_OP_OPS: {

        nodeState = NS_OPERATE;
        if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

      } break;

    case LCS_OP_CFG: {

        nodeState = NS_CONFIG;
        if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

      } break;

    case LCS_OP_BON: {

        // ??? readyLed on
        nodeState = NS_OPERATE;
        if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

      } break;

    case LCS_OP_BOF: {

        /// ??? readyLed off
        nodeState = NS_HALTED;
        if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

      } break;

    case LCS_OP_NCOL: {

        // ??? readyLed off
        nodeState = NS_COLLISION;
        if ( callbackMap.lcsMsgCallback != nullptr ) callbackMap.lcsMsgCallback( msg );

      } break;

    case LCS_OP_RESET: {

        uint16_t nodeId  = (( msg[1] << 8 ) + msg[2] ) >> 4;
        uint8_t  portId  = (( msg[1] << 8 ) + msg[2] ) & 0x0F;

        if ( nodeId == NIL_NODE_ID ) {

          // ??? FIX ...
          // setupNode( );

          nodeState = NS_INIT;
        }
        else if (( nodeId != NIL_NODE_ID ) && ( portId == NIL_PORT_ID )) {

          if ( nodeId == nodeMap.id ) {


            // ??? fix
            // setupNode( );

            nodeState = NS_INIT;
          }
        }
        else if (( nodeId != NIL_NODE_ID ) && ( portId != NIL_PORT_ID )) {

          if ( nodeId == nodeMap.id ) {

            uint8_t rStat = resetPort( msg[ 3 ] );

            if ( rStat == ALL_OK ) LCS::sendAck( nodeId );
            else                   LCS::sendErr( nodeId, rStat, 0, 0 );
          }
        }

      } break;

    case LCS_OP_SET_NID: {

        uint16_t nodeId   = ( msg[1] << 8 ) + msg[2];
        uint32_t nodeUID  = ((uint32_t) msg[3] << 24 ) + ((uint32_t) msg[4] << 16 ) +
                            ((uint32_t) msg[5] << 8 ) + msg[6];

        if ( nodeUID == nodeMap.uid ) {

          if ( nodeState == NS_CONFIG ) {

            if ( nodeId != nodeMap.id ) nodeMap.id = nodeId;
            sendAck( nodeId );
          }
          else sendErr( nodeId, ERR_NODE_NOT_CONFIG_STATE, 0, 0 );
        }

      } break;
  }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgQryNode" processes an incoming query message for a node item, i.e. attribute.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgQryNode( uint8_t *msg ) {

  uint16_t nodeId  = (( msg[1] << 8 ) + msg[2] ) >> 4;
  uint8_t  portId  = (( msg[1] << 8 ) + msg[2] ) & 0x0F;

  if ( nodeId == nodeMap.id ) {

    uint8_t   item  = msg[3];
    uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
    uint16_t  arg2  = ( msg[6] << 8 ) + msg[7];
    uint8_t   ret   = nodeInfo( item, portId, &arg1, &arg2 );

    if ( ret == ALL_OK )  sendRepNode( nodeId, portId, item, arg1, arg2 );
    else                  sendErr( nodeId, ret, 0, 0 );
  }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgRepNode" processes the answer to a previously sent node query. The incoming message will only
// result in an action when we have an outstanding request for that node.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgRepNode( uint8_t *msg ) {

  uint16_t nodeId  = (( msg[1] << 8 ) + msg[2] ) >> 4;

  // ??? check if this is our node...

  // ??? check if we have a pending request....

  uint8_t   portId  = (( msg[1] << 8 ) + msg[2] ) & 0x0F;
  uint8_t   item    = msg[3];
  uint16_t  arg1    = ( msg[4] << 8 ) + msg[5];
  uint16_t  arg2    = ( msg[6] << 8 ) + msg[7];

  if ( callbackMap.itemReqRepCallback != nullptr )
    callbackMap.itemReqRepCallback( nodeId, portId, item, arg1, arg2 );

}

//------------------------------------------------------------------------------------------------------------
// "handleMsgSetNode" processes an incoming set message for a node map item, i.e. attribute.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgReqNode( uint8_t *msg ) {

  uint16_t nodeId  = (( msg[1] << 8 ) + msg[2] ) >> 4;
  uint8_t  portId  = msg[2] & 0x0F;
  if ( nodeId == nodeMap.id ) {

    uint8_t   item  = msg[3];
    uint16_t  arg1  = ( msg[4] << 8 ) + msg[5];
    uint16_t  arg2  = ( msg[6] << 8 ) + msg[7];
    uint8_t   ret   = nodeInfo( portId, item, &arg1, &arg2 );

    if ( ret == ALL_OK )  sendRepNode( nodeId, portId, item, arg1, arg2 );
    else                  sendErr( nodeId, ret, 0, 0 );
  }
}

//------------------------------------------------------------------------------------------------------------
// "handleMsgEvent" deals with the event messages for inbound ports. For all matching events in the eventMap,
// the respective port map entry fields are set. The searchEvent function will return us the first matching
// entry in the sorted event map, if any. From there, we just hop along the eventMap until the eventId does
// not match any longer. All we do in this routine is to record the event data and the future timestamp when
// the event should result in a callback. The actual event processing is done in the port event processing
// routine, which will manage the timely invocation of the event callbacks.
//
//
// ??? what events created on this node and a port is interested ? they should also be processed ....
//------------------------------------------------------------------------------------------------------------
void handleMsgEvent( uint8_t *msg ) {

  uint16_t  eventId = ( msg[3] * 256 ) + msg[4];
  int       index   = searchEvent( eventId );

  if ( index >= 0 ) {

    uint8_t   opCode            = msg[0];
    uint16_t  nodeId            = ( msg[1] * 256 ) + msg[2];
    uint16_t  eventData         = ( msg[5] * 256 ) + msg[6];
    uint8_t   eventAction       = PEA_EVENT_IDLE;

    switch ( opCode ) {

      case LCS_OP_EVT_ON:   eventAction = PEA_EVENT_ON;   break;
      case LCS_OP_EVT_OFF:  eventAction = PEA_EVENT_OFF;  break;
      case LCS_OP_EVT:      eventAction = PEA_EVENT_EVT;  break;
    }

    while (( index < eventMap.hwm ) && ( eventMap.map[ index ].eventId == eventId )) {

      LcsPortMapEntry *pPtr = &portMap.map[ index ];   // ???? --------

      if (( pPtr -> flags & PF_PORT_ENABLED                  ) &&
          ( pPtr -> flags & PF_PORT_EVENT_HANDLING_ENABLED   ) &&
          ( pPtr -> flags & ( ~ PF_PORT_EVENT_DIRECTION     ))) {

        pPtr -> nodeId          = nodeId;
        pPtr -> eventId         = eventId;
        pPtr -> eventAction     = eventAction;
        pPtr -> eventValue      = eventData;
        pPtr -> eventTimeStamp  = CDC::getMillis( ) + ( pPtr -> eventDelayTime * EVENT_DELAY_TICK_MILLIS );
        pPtr -> flags           |= PF_EVENT_PENDING;
      }

      index++;
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// We received a DCC subsystem message. Just pass it to the call back routine. DCC messages are typically
// handled by the base station and the cab handhelds.
//
//------------------------------------------------------------------------------------------------------------
void handleMsgDccMgt( uint8_t *msg ) {

  if ( callbackMap.dccMsgCallback != NULL ) callbackMap.dccMsgCallback( msg );
}


//------------------------------------------------------------------------------------------------------------
// Node state INIT. This is the first state after the initial library setup. The init call created all memory
// areas and initalized the data structures. After the init call, the firmeware programmer can register the
// necessary callback functions and do other firmware specific work. Eventually, the loop method is called.
// We enter the loop method and the node state is INIT. If set, the node init and port init callback routines
// will be invoked. If the nodeId validation step is set, the node will request a nodeId and enter the state
// SETUP. Otherwise the next state is OPERATE.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateInit( ) {

  if ( nodeMap.options & ( ~ NOPT_SKIP_NODE_INIT_STEP )) {

    if ( callbackMap.map[ 0 ].initCallback != nullptr )
      callbackMap.map[ 0 ].initCallback( nodeMap.id, 0, 0 );
  }

  if ( nodeMap.options & ( ~ NOPT_SKIP_PORT_INIT_STEP )) {

    for ( uint8_t i = 1; i <= MAX_PORT_MAP_ENTRIES; i++ ) {

      if ( callbackMap.map[ i ].initCallback != nullptr )
        callbackMap.map[ i ].initCallback( nodeMap.id, i, 0 );

      portMap.map[ i - 1 ].flags |= PF_PORT_ENABLED;
      portMap.map[ i - 1 ].flags |= PF_PORT_EVENT_HANDLING_ENABLED;
    }
  }

  if ( ! ( nodeMap.options & NOPT_SKIP_NODE_ID_CONFIG )) {

    sendReqNodeId( nodeMap.id, nodeMap.uid, 0 );
    timerVal  = CDC::getMillis( );
    nodeState = NS_REGISTER;

  } else nodeState = NS_OPERATE;
}

//------------------------------------------------------------------------------------------------------------
// Node State FAIL. This is the state after the node startup failed. We will stay in this state with the
// LEDs showing a fatal error.
//
//------------------------------------------------------------------------------------------------------------
void handleNodeStateFail( ) {

  // ??? readyLed off. fatal error code

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

          sendReqNodeId( nodeMap.id, nodeMap.uid, 0 );
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
// Node State HALTED. The LCS communication bus was halted for all nodes. We just listen to the BON or RESET
// message to get going again.
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

    case LCS_OP_QRY_NODE:       handleMsgQryNode( msg );              break;
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
    case LCS_OP_NCOL:           handleMsgLcsMgt( msg );               break;

    case LCS_OP_QRY_NODE:       handleMsgQryNode( msg );              break;
    case LCS_OP_REP_NODE:       handleMsgRepNode( msg );              break;
    case LCS_OP_REQ_NODE:       handleMsgReqNode( msg );              break;

    case LCS_OP_EVT_ON:
    case LCS_OP_EVT_OFF:
    case LCS_OP_EVT:            handleMsgEvent( msg );                break;

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

    case LCS_OP_REQ_TON:
    case LCS_OP_REQ_TOF:
    case LCS_OP_TON:
    case LCS_OP_TOF:
    case LCS_OP_REQ_ESTP:
    case LCS_OP_ESTP:

    case LCS_OP_SEND_DCC3:
    case LCS_OP_SEND_DCC4:
    case LCS_OP_SEND_DCC5:
    case LCS_OP_SEND_DCC6:

    case LCS_OP_DCC_ACK:
    case LCS_OP_DCC_ERR:        handleMsgDccMgt( msg );               break;
  }
}

//-----------------------------------------------------------------------------------------------------------
// "startRuntime" is the main routine of the node activity processing. It is the method called after all 
// setup is done. Running in a loop, the primary function is to handle the activities according to the node 
// state. The run loop also processes the serial commands, periodic tasks and events. Note that this function
// will not return.
//
//------------------------------------------------------------------------------------------------------------
void startRuntime( ) {

  while ( true ) {

    switch ( nodeState ) {

      case NS_INIT:       handleNodeStateInit( );       break;
      case NS_FAIL:       handleNodeStateFail( );       break;
      case NS_REGISTER:   handleNodeStateRegister( );   break;
      case NS_COLLISION:  handleNodeStateCollision( );  break;
      case NS_HALTED:     handleNodeStateHalted( );     break;
      case NS_CONFIG:     handleNodeStateConfig( );     break;
      case NS_OPERATE:    handleNodeStateOperations( ); break;
    }

    if (( nodeState == NS_OPERATE ) || ( nodeState == NS_CONFIG )) {

      handlePeriodicTasks( );
      handleNodePortEvents( );
    }

    handleSerialCommand( );
  }
}

}; // namespace
