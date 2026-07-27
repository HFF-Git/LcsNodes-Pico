//----------------------------------------------------------------------------------------
//
//  Layout Control System - Can Bus Interface Library, based on "can2040" library
//
//----------------------------------------------------------------------------------------
// The "LcsMsgBusCAN" object implements the LCS message bus as a CAN bus. The CAN 
// bus is a widely established bus, which is quite robust. We use the standard CAN 
// bus with a maximum CAN Id of 29 bits. In our case the node and port data 
// along with a 2 bit priority field represents the CAN address used on the bus.
//
// On the PICO, there is a library, "can2040", available that implements the CAN 
// bus protocol in software, using the PICO PIO state machines. This saves us an 
// external controller chip. In addition, we allow for the option to run the CAN 
// bus state machine on a separate core. This is highly recommend as the LCS Runtime
// has a lot of other things to do. Using a queue from the PICO C++ SDK, the core
// running the CAN state machine will just queue the received message to be picked
// up by the other core when ready.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Can Bus Interface Library, based on "can2040" library
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
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/clocks.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"

#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//----------------------------------------------------------------------------------------
// The can2040 is a C library. Make it extern C, otherwise the linker gets confused.
// 
//----------------------------------------------------------------------------------------
extern "C" {

    #include "./Can2040Lib/can2040.h"
}

//----------------------------------------------------------------------------------------
// The debug mask. See the internal include file for details.
// 
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t debugMask;
};

//----------------------------------------------------------------------------------------
// The name space for file local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------
// The maximum message length of a CAN bus ( and LCS ) message. The LCS library 
// still uses the "classic" CAN bus message size. Finally, the CAN bus library for 
// the RP2040 needs a static opaque structure. We also need a receiver queue for 
// storing the received messages when they come in.
//
//----------------------------------------------------------------------------------------
const uint8_t   MAX_CAN_MSG_SIZE  = 8;
const uint8_t   RX_QUEUE_SIZE     = 8;
const uint8_t   TX_RETRY_TIMEOUT  = 5;

//----------------------------------------------------------------------------------------
// The setup and start of the CAN Bus can run on either core zero or core one, 
// depending whether a multi-core implementation is desired. The "Can2040ConfigDesc"
// structure holds all the necessary configuration data for the initialization 
// routine to use.
//
//----------------------------------------------------------------------------------------
struct Can2040ConfigDesc {

    uint32_t        mcPioNum;
    uint32_t        mcSysClock;
    uint32_t        mcBitRate;
    uint32_t        mcRxPin;
    uint32_t        mcTxPin;
    can2040_rx_cb   mcRxCallback;
    uint32_t        mcRxQueueSize;
    bool            mcRunOnCore1;
    bool            mcSetupOK;
};

//----------------------------------------------------------------------------------------
// File local variables and constants.
//
//----------------------------------------------------------------------------------------
Can2040ConfigDesc       cfg;
struct can2040          cBus;
queue_t                 rxQueue;

//----------------------------------------------------------------------------------------
// "canBusDebugEnabled" and "retStat" are the debug support routines. We can 
// easily check whether debug is enabled at all. The return status routine will 
// print out a return status message when debugging is enabled. The macro 
// "RET_STAT" is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool canBusDebugEnabled(  ) {

    return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_CAN_BUS )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( canBusDebugEnabled( )) {

       // if ( errId == LCS_OK )  printf( "%s: OK\n", name )
       //  else                    printf( "%s: %d\n", name, errId );

       if (( errId != LCS_OK ) && ( errId != ERR_CAN_MSG_NO_MSG )) 
            printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// The "buildCanBusMsgHeader" constructs the canId header for the message. It 
// encodes the canId itself and flags such as EXT or RTR. The canId consists of
// the npId and a priority field.
//
//----------------------------------------------------------------------------------------
inline uint32_t buildCanBusMsgHeader( uint8_t npId,
                                      uint8_t msgPri, 
                                      bool RTR = false ) {

    uint32_t header = ((uint32_t)( msgPri & 0x3 ) << 16 ) |
                      ((uint32_t)( npId ))                | 
                      ((uint32_t)( 0x80000000 ));

    if ( RTR ) header |=  0x40000000;

    return ( header );
}

//----------------------------------------------------------------------------------------
// The interrupt signature to register with the RP2040 for PIO interrupts. The
// interrupt handler itself is provided by the can2040 library.
//
//----------------------------------------------------------------------------------------
void CanBusPIOIrqHandler( ) {

    can2040_pio_irq_handler( &cBus );
}

//----------------------------------------------------------------------------------------
// For each messages transmitted or received this callback is invoked from within 
// the interrupt handler, so all we can do is a quick non-blocking action. The 
// callback allows to react to a message sent, a message received and an internal 
// buffer overflow error.
//
// The callback could be used to filter messages at this stage. Only messages that 
// concern this node should be processed. Easy said, but perhaps no so easy to do. 
// We can basically to filtering at the higher message bus level or at the lower 
// layers. The benefit for doing it here is that when we run at the other core, the 
// main core is relieved even further. To think about one day.
// 
// ??? idea: we could group the LCS messages in a way that we can filter out.
//
//  ANY - all messages are passed
//  SYS - system wide messages
//  DCC - DCC messages 
//  GET/PUT - foreign node data access are filtered.
//  REQ - foreign node request is filtered.
// ...
//----------------------------------------------------------------------------------------
void canBusEventCallback( struct can2040 *cd, 
                          uint32_t notify, 
                          struct can2040_msg *msg ) {

    if ( notify == CAN2040_NOTIFY_RX ) {

        // read completed successfully
        // filter ....

        if ( ! queue_try_add( &rxQueue, msg )) {

            // ??? we could not add ... what to do ?
        }
    }
    else if ( notify == CAN2040_NOTIFY_TX ) {

        // transmit completed successfully
    }
    else if ( notify == CAN2040_NOTIFY_ERROR ) {

        // ??? internal can2040 buffer overflow ... what to do ?
    }
}

//----------------------------------------------------------------------------------------
// "canBusCore" is the routine that encapsulates the can2040 setup and launch work. 
// For the multi-core version it needs to be a routine that can be called from the 
// current core or be launched on the other core. The routine communicates the 
// successful setup with a boolean flag in the configuration descriptor. Note that 
// the setup routine must be a void procedure with no parameters. This is expected 
// by the launch routine in the PICO C++ SDK.
//
//----------------------------------------------------------------------------------------
void canBusCore( ) {

    if ( canBusDebugEnabled( )) {

        printf( "canBusSetup -> pio: %d, clk: %d, bitRate: %d, rxPin: %d,"
                 "txPin: %d, cb: %u, rxQS: %d, MC: %d\n",
                cfg.mcPioNum, cfg.mcSysClock, cfg.mcBitRate,
                cfg.mcRxPin, cfg.mcTxPin, cfg.mcRxCallback,
                cfg.mcRxQueueSize, cfg.mcRunOnCore1 );
    }

    queue_init( &rxQueue, sizeof( can2040_msg ), cfg.mcRxQueueSize );

    can2040_setup( &cBus, cfg.mcPioNum );
    can2040_callback_config( &cBus, cfg.mcRxCallback );

    if ( cfg.mcPioNum == 0 ) {

        irq_set_exclusive_handler( PIO0_IRQ_0, CanBusPIOIrqHandler );
        irq_set_priority( PIO0_IRQ_0, 1 );
        irq_set_enabled( PIO0_IRQ_0, true );
    }
    else if ( cfg.mcPioNum == 1 ) {

        irq_set_exclusive_handler( PIO1_IRQ_0, CanBusPIOIrqHandler );
        irq_set_priority( PIO1_IRQ_0, 1 );
        irq_set_enabled( PIO1_IRQ_0, true );
    }

    can2040_start( &cBus, cfg.mcSysClock, cfg.mcBitRate, cfg.mcRxPin, cfg.mcTxPin );

    cfg.mcSetupOK = true;

    if ( canBusDebugEnabled( )) {

        printf( "CAN Bus Initialized, runs on Core: %d\n", get_core_num( ));
    }

    if ( cfg.mcRunOnCore1 ) {

        while ( true ) tight_loop_contents( );
    }
}

}; // namespace

//----------------------------------------------------------------------------------------
// The LCS name space CanBus Object methods declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS { 

//----------------------------------------------------------------------------------------
// "init" is called to setup the CAN bus interface. We will first check the 
// parameters and setup the CAN bus. Next set up the interrupt handler and start 
// the CAN bus processing. 
// 
//----------------------------------------------------------------------------------------
uint8_t LcsMsgBusCAN::init( uint8_t  rxPin, 
                            uint8_t  txPin, 
                            uint32_t baudRate, 
                            bool     twoCores ) {

    if ( canBusDebugEnabled( )) {

        printf( "Init Can Bus -> rxPin: %d, txPin: %d, BaudRate: %d, twoCores: %d\n", 
                rxPin, txPin, baudRate, twoCores );
    }

    if (( rxPin == UNDEFINED_PIN ) || ( txPin == UNDEFINED_PIN )) {
    
        return ( RET_STAT( ERR_CAN_BUS_INIT ));
    }

    cfg.mcSetupOK       = true;
    cfg.mcRxPin         = rxPin;
    cfg.mcTxPin         = txPin;
    cfg.mcRxCallback    = canBusEventCallback;
    cfg.mcSysClock      = clock_get_hz( clk_sys );
    cfg.mcPioNum        = 0;
    cfg.mcRxQueueSize   = RX_QUEUE_SIZE;
    cfg.mcRunOnCore1    = twoCores;
    cfg.mcBitRate       = baudRate;
    cfg.mcSetupOK       = true;

    if ( cfg.mcRunOnCore1 ) multicore_launch_core1( canBusCore );
    else canBusCore( );

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// The CAN bus used the nodeId as canBus Id. We use this data to check wether
// the message to send is a local message, which we can directly queue onto the
// receive queue. The nodeId should be set before sending any messages.
//
//----------------------------------------------------------------------------------------
void LcsMsgBusCAN::setNodeId( uint8_t nodeId ) {

    localNodeId = nodeId; 
}

//----------------------------------------------------------------------------------------
// "sendLcsMsg" will send a data packet. We are passed the message buffer and the 
// message priority. The message length is encoded in the first message byte, 
// which represents the LCS message opCode as well as the length of the message. 
//
// There are two main cases. When the message is targeted to a remote node, we
// just send it. When the target is a another port on the local node, we simply
// queue the message onto the receive queue.  
//
// Event message are always queued to the receiver queue, we cannot know if a
// another port on the local node is interested. The event message is therefore
// received by all local ports that registered an interest.
//
// In all other cases the message should be sent to the outside world. When the
// send fails, we will retry a few times by raising the message priority. 
// When the message cannot be sent at the highest priority, we report a send 
// error.
//
//----------------------------------------------------------------------------------------
uint8_t LcsMsgBusCAN::sendLcsMsg ( uint16_t sendingNpId, 
                                   uint8_t *msgBuf,
                                   uint8_t msgPri ) {

    can2040_msg msg;

    msg.id  = buildCanBusMsgHeader( sendingNpId, msgPri );
    msg.dlc = ( msgBuf[ 0 ] >> 5 ) + 1;

    for ( uint32_t i = 0; i < msg.dlc; i++ ) msg.data[ i ] = msgBuf[ i ];

    if ( canBusDebugEnabled( )) {

        printf( "CAN Send (TS: 0x%x)(S-Id: 0x%x, Pri: %d)(Data: ", 
                getMillis( ), sendingNpId, msgPri );
        for ( int i = 0; i < msg.dlc; i++ ) printf( " 0x%x", msgBuf[ i ] );
        printf( ")\n" );
    }

    uint8_t msgOp = msgBuf[ 0 ];

    if (( msgOp == LCS_OP_NODE_GET  ) ||
        ( msgOp == LCS_OP_NODE_SET  ) ||
        ( msgOp == LCS_OP_NODE_FREQ )) { 

        uint16_t targetNpId = msg.data[ 1 ] << 8 | msg.data[ 2 ]; 

        if ( equalNodeId( sendingNpId, targetNpId )) {

            // ??? remember the nodeId in "localNodeId" ?
            localNodeId = nodeId( sendingNpId );

            msg.id = buildCanBusMsgHeader( buildNpId( NIL_NODE_ID,
                                           portId( sendingNpId ), 
                                           chanId( sendingNpId )), msgPri );

            if ( ! queue_try_add( &rxQueue, &msg )) {

                return ( RET_STAT( ERR_CAN_MSG_SEND ));
            }
        }
    }
    else if (( msgOp == LCS_OP_EVT       ) ||
             ( msgOp == LCS_OP_EVT_ON    )||
             ( msgOp == LCS_OP_EVT_OFF   ) ) { 
     
        if ( ! queue_try_add( &rxQueue, &msg )) {

                return ( RET_STAT( ERR_CAN_MSG_SEND ));
        }
    }
    else {

        if ( can2040_transmit( &cBus, &msg ) != 0 ) {

            sleepMillis( TX_RETRY_TIMEOUT );

            if ( msgPri > MSG_PRI_VERY_HIGH ) {
                
                return ( sendLcsMsg( sendingNpId, 
                                     msgBuf, 
                                     msgPri - 1 ));
            }
            else return ( ERR_CAN_MSG_SEND );
        } 
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "receiveLcsMsg" will check for a message on the receiver queue. The CAN Bus 
// library will place a message received onto this queue. In addition, local 
// massages, i.e. messages send from our own node, will also be placed in the
// receiver queue. These messages have a nodeId of NIL_NODE_ID and need to 
// be patched on receive with the local nodeId before returning to the caller.
//
// Besides receiving a message, there is the handling of Node Id collisions on
// the LCS bus. When we detect a non-zero length message with a Node Id that is
// our own and was not queued locally, we have a collision and report an error. 
// This could happen for example when a node hardware is connected to another 
// layout. Both nodes will then stop and wait for manual resolution. 
//
// In addition to message processing, we also need to react to a CAN Bus RTR 
// message by sending a zero length message response. Replying to such a message
// from other nodes results in a status return of "ERR_CAN_MSG_NO_MSG" on this 
// call as no LCS message was actually received. This is also the case when the 
// message queue is empty.
//
// ??? it would be a good place to do filtering. Any GET/SET/REQ/REP that we
// not involved, should be ignored right here. If we can also the broadcast
// type messages nicely grouped, we could also filter. For example, DCC messages
// typically send from throttle to base station, etc. Still, there should be
// an option to get any kind of message for a tracing tool, etc.
//----------------------------------------------------------------------------------------
uint8_t LcsMsgBusCAN::receiveLcsMsg( uint16_t *senderNpId, uint8_t *msgBuf ) {

    can2040_msg msg;

    if ( queue_try_remove( &rxQueue, &msg )) {

        if ( canBusDebugEnabled( )) {

            printf( "CAN Recv (TS: 0x%x)( Id: 0x%x, len: %d)(Data: ", 
                    getMillis( ), msg.id, msg.dlc );

            for ( uint32_t i = 0; i < msg.dlc; i++ ) 
                printf( " 0x%x", msg.data[ i ] );

            printf( ")\n" );
        }

        bool      rtrFlag     = ( msg.id & 0x40000000 );
        bool      extFlag     = ( msg.id & 0x80000000 );
        uint16_t  remoteNpId  = (( extFlag ) ? 
                                 ( msg.id & 0xFFFF ) : ( msg.id & 0x7F ));

        if ( equalNodeId( nodeId( remoteNpId ), NIL_NODE_ID )) {

            *senderNpId = buildNpId( localNodeId, 
                                     portId( remoteNpId ), 
                                     chanId( remoteNpId ));

            memcpy( msgBuf, msg.data, msg.dlc );
            return ( RET_STAT( LCS_OK ));
        }
        else if ( equalNodeId( remoteNpId, localNodeId ) && ( msg.dlc > 0 )) {

            return ( RET_STAT( ERR_CAN_ID_COLLISION ));
        }
        else if ( rtrFlag ) {

            msg.id          = buildNpId( localNodeId, portId( 0 ), chanId( 0 ));
            msg.dlc         = 0;
            msg.data32[ 0 ] = 0;
            msg.data32[ 1 ] = 0;

            can2040_transmit( &cBus, &msg );
            return ( RET_STAT( ERR_CAN_MSG_NO_MSG ));
        }
        else {

            *senderNpId = remoteNpId;
            memcpy( msgBuf, msg.data, msg.dlc );
            return ( RET_STAT( LCS_OK ));
        }
    }
    else return ( RET_STAT( ERR_CAN_MSG_NO_MSG ));
}

}; // namespace LCS
