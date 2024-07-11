//------------------------------------------------------------------------------------------------------------
//
// "LcsMsgBusCAN" - CAN Bus Interface for Raspberry PI Pico
//
//------------------------------------------------------------------------------------------------------------
// The "LcsMsgBusCAN" object implements the LCS message bus as a CAN bus. The CAN bus is a widely established
// bus, which is quite robust. We use the standard CAN bus with a maximum CAN Id of 29 bits. In our case the
// node / port ID is used.
//
// On the PICO, there is a library, "can2040", available that iplements the CAN bus protocol in software,
// using the PICO PIO state machines. This saves us an external controller. In addition, we allow for the
// option to run the CAN bus state machine on a separate core. This is highly recommend as the LCS Runtime
// has a lot of other things to do. Using a queue from the PICO C++ SDK, the core running the CAN state
// machine will just queue the received message to be pciked up by the other core when ready.
//
// As a further optimization we could implement a filter that filters out nodeId specific messages not
// itended for this node. One day....
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Can Bus Interface Library
// Copyright (C) 2022 - 2024,  Helmut Fieres
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
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"

#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//------------------------------------------------------------------------------------------------------------
// The can2040 is a C library. Make it so, otherwise the linker gets confused...
// 
//------------------------------------------------------------------------------------------------------------
extern "C" {

  #include "./Can2040Lib/can2040.h"
}

//------------------------------------------------------------------------------------------------------------
// The name space for file local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  //----------------------------------------------------------------------------------------------------------
  // The maximum message length of a CAN bus ( and LCS ) message. The LCS library still uses the "classic"
  // CAN bus message size. For the ennumration process, there is the time interval to collect enumeration
  // responses. Finally, the CAN bus library for the RP2040 needs a static opaque structure. We also need
  // a receiver queue for storing the received messages when they come in.
  //
  //----------------------------------------------------------------------------------------------------------
  const uint8_t   MAX_CAN_MSG_SIZE  = 8;
  const uint8_t   RX_QUEUE_SIZE     = 4;

  //----------------------------------------------------------------------------------------------------------
  // The setup and start of the CAN Bus can run on ether core 0 or core 1, depending whether a multi-core
  // implementation is desired. The "Can2040ConfigDesc" structure holds all the necessary configuration data
  // for the initialisation routine to use.
  //
  //----------------------------------------------------------------------------------------------------------
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

  //----------------------------------------------------------------------------------------------------------
  // File local variables and constants.
  //
  //----------------------------------------------------------------------------------------------------------
  struct can2040          cBus;
  queue_t                 rxQueue;
  Can2040ConfigDesc       cfg;

  //----------------------------------------------------------------------------------------------------------
  // The "buildMcpCanBusHeader" constructs the canId parameter for the Arduino "mcp_can" library. The canId
  // 32-bit word encodes the canId itself, and flags such as EXT or RTR.
  //
  //----------------------------------------------------------------------------------------------------------
  inline uint32_t buildMcpCanBusHeader( uint16_t canId, uint8_t msgPri, bool RTR = false ) {

    uint32_t header = canId | ((uint32_t)( msgPri & 0x3 ) << 16 ) | 0x80000000;

    if ( RTR ) header |=  0x40000000;

    return ( header );
  }

  //----------------------------------------------------------------------------------------------------------
  // The interrupt signature to register with the RP2040 fpr PIO interrupts. The interrupt handlr itself is
  // provided by the can2040 library.
  //
  //----------------------------------------------------------------------------------------------------------
  void CanBusPIOIrqHandler( ) {

    can2040_pio_irq_handler( &cBus );
  }

  //----------------------------------------------------------------------------------------------------------
  // For each messages transmitted or received a callback is invoked from within the interrupt handler, so
  // all we can do is a quick non-blocking action. The callback allows to react to a message sent, a message
  // received and an internal buffer overflow error.
  //
  // The callback could be used to filter messages directly at this stage. Only messages that concern this
  // node should be processed. Easy said, but perhaps no so easy to do. We can basically to filtering at the
  // higher message bus level or at the lower layers. The benefit fopr doing it hee is that it would run on
  // a separate core and relief the other core even further. To think about one day.
  // 
  //----------------------------------------------------------------------------------------------------------
  void canBusEventCallback( struct can2040 *cd, uint32_t notify, struct can2040_msg *msg ) {

    if ( notify == CAN2040_NOTIFY_RX ) {

        // ??? possible filtering right here. 
        // ??? We could also do the pending REQ / REPLY message processing here.

      if ( queue_try_add( &rxQueue, msg )) {

         // ??? successfully queued, remove from REQ pending list ?
      }
      else {

        // ??? we could not add ... what to do ?
      }
    }
    else if ( notify == CAN2040_NOTIFY_TX ) {

      // ??? add to pending reqwuest list ?
      // ??? transmit completed successfully
    }
    else if ( notify == CAN2040_NOTIFY_ERROR ) {

      // ??? internal buffer overflow ... what to do ?
    }
  }

  //----------------------------------------------------------------------------------------------------------
  // "canBusCore" is the routine that encapsulates the can2040 setup and launch work. For the multi-core
  // version it needs to be a routine that can be called from the current core or be launched on the other
  // core. The routine communicates the successful setup with a boolean flag in the configuration descriptor.
  // Note that the setup routine must be a void procedure with no parmeters. This is expected by the launch
  // routine in the PICO C++ SDK.
  //
  //----------------------------------------------------------------------------------------------------------
  void canBusCore( ) {

    #if DEBUG_CAN_BUS == 1
    printf( "canBusSetup -> pio: %d, clk: %d, bitRate: %d, rxPin: %d, txPin: %d, cb: %u, rxQS: %d, MC: %d\n",
            cfg.mcPioNum, cfg.mcSysClock, cfg.mcBitRate,
            cfg.mcRxPin, cfg.mcTxPin, cfg.mcRxCallback,
            cfg.mcRxQueueSize, cfg.mcRunOnCore1 );
    #endif

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

    #if DEBUG_CAN_BUS == 1
    printf( "CAN Bus Initialized, runs on Core: %D", get_core_num( ));
    #endif

    if ( cfg.mcRunOnCore1 ) {

      while ( true ) tight_loop_contents( );
    }
  }

}; // namespace


//------------------------------------------------------------------------------------------------------------
// "init" is called to setup the CAN bus interface. We will first check the parameters and setup the CAN bus.
// Next set up the interrupt handler and start the CAN bus processing. This is also the time to set the
// initial canId.
//
//------------------------------------------------------------------------------------------------------------
uint8_t LCS::LcsMsgBusCAN::init( uint16_t canId, uint8_t rxPin, uint8_t txPin, uint8_t fMode ) {

  #if DEBUG_CAN_BUS == 1
  printf( "Init Can Bus -> Node: %d, Rx: %d, Tx: %d, Mode: %d\n", canId, rxPin, txPin, fMode );
  #endif

  if (( rxPin == CDC::UNDEFINED_PIN ) || ( txPin == CDC::UNDEFINED_PIN )) return ( ERR_CAN_BUS_INIT );

  this -> canId = canId;

  cfg.mcSetupOK       = true;
  cfg.mcRxPin         = rxPin;
  cfg.mcTxPin         = txPin;
  cfg.mcRxCallback    = canBusEventCallback;
  cfg.mcSysClock      = CDC::getCpuFrequency( );
  cfg.mcPioNum        = 0;
  cfg.mcRxQueueSize   = RX_QUEUE_SIZE;

  cfg.mcRunOnCore1    = (( fMode == CAN_BUS_LIB_PICO_PIO_125K_M_CORE ) ||
                         ( fMode == CAN_BUS_LIB_PICO_PIO_250K_M_CORE ) ||
                         ( fMode == CAN_BUS_LIB_PICO_PIO_500K_M_CORE ) ||
                         ( fMode == CAN_BUS_LIB_PICO_PIO_1000K_M_CORE ));

  switch ( fMode ) {

    case CAN_BUS_LIB_PICO_PIO_125K:
    case CAN_BUS_LIB_PICO_PIO_125K_M_CORE:   cfg.mcBitRate = 125000; break;

    case CAN_BUS_LIB_PICO_PIO_250K:
    case CAN_BUS_LIB_PICO_PIO_250K_M_CORE:   cfg.mcBitRate = 250000; break;

    case CAN_BUS_LIB_PICO_PIO_500K:
    case CAN_BUS_LIB_PICO_PIO_500K_M_CORE:   cfg.mcBitRate = 500000; break;

    case CAN_BUS_LIB_PICO_PIO_1000K:
    case CAN_BUS_LIB_PICO_PIO_1000K_M_CORE:  cfg.mcBitRate = 1000000; break;

    default: cfg.mcSetupOK = false;
  }

  if ( cfg.mcSetupOK ) {

    if ( cfg.mcRunOnCore1 ) multicore_launch_core1( canBusCore );
    else canBusCore( );
  }

  return (( cfg.mcSetupOK ) ? ALL_OK : ERR_CAN_BUS_INIT );
}

//------------------------------------------------------------------------------------------------------------
// "sendLcsMsg" will send a data packet. We are passed the message buffer and the message priority. The
// message length is encoded in the first message byte, which represents the LCS message opCode.
//
// ??? Simple version for now. We just attempt to send the message. If the queue is full, it would be nice
// to just send it again. Also, how do we know that there was a timeout so that we can send it again with a
// higher priority ?
//------------------------------------------------------------------------------------------------------------
uint8_t LCS::LcsMsgBusCAN::sendLcsMsg ( uint8_t *msgBuf, uint8_t msgPri ) {

  can2040_msg msg;

  msg.id  = buildMcpCanBusHeader( canId, msgPri );
  msg.dlc = ( msgBuf[ 0 ] >> 5 ) + 1;

  for ( uint32_t i = 0; i < msg.dlc; i++ ) msg.data[ i ] = msgBuf[ i ];

  #if DEBUG_CAN_BUS == 1
  printf( "CAN Send (TS: 0x%x)(Id: 0x%x, Pri: %d)(Data: ", CDC::getMillis( ), canId, msgPri );
  for ( int i = 0; i < msg.dlc; i++ ) printf( " 0x%x", msgBuf[ i ] );
  printf( ")\n" );
  #endif

  if ( can2040_transmit( &cBus, &msg ) != 0 ) {

    CDC::sleepMillis( 5 );
    if ( msgPri > MSG_PRI_VERY_HIGH ) return ( sendLcsMsg( msgBuf, msgPri - 1 ));
    else                              return ( ERR_CAN_MSG_SEND );

  } else return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "receiveLcsMsg" will check for a CAN Bus message and if one is available fill the passed message buffer.
// Besides receiving a message, there is the handling of CAN Id collisions. When we detect a non-zero length
// message with a Can Id that is our own, we have a collision and report an error. This could happen for
// example when a node hardware is connected to another layout. Both nodes will then stop and wait for
// manual resolution.
//
// In addition to message processing, we also need to react to RTR messages. We answer such a request with
// sending a zero length message respionse. Replying to such a message from other nodes results in a status
// return of "ERR_CAN_MSG_NO_MSG" on this call as no LCS message was actually received.
//
// With the "can2040" library CAN bus messages are received via a callback function. This library will store
// each received message in the local receiver queue. This happens within the interrupt handler who calls the
// receiver callback. So, all this routine will do is work from the receiver queue.
//
//------------------------------------------------------------------------------------------------------------
uint8_t LCS::LcsMsgBusCAN::receiveLcsMsg( uint8_t *msgBuf ) {

  can2040_msg msg;

  if ( queue_try_remove( &rxQueue, &msg )) {

    #if DEBUG_CAN_BUS == 1
    printf( "CAN Recv (TS: 0x%x)(Id: 0x%x, len: %d)(Data: ", CDC::getMillis( ), msg.id, msg.dlc );
    for ( uint32_t i = 0; i < msg.dlc; i++ ) printf( " 0x%x", msg.data[ i ] );
    printf( ")\n" );
    #endif

    bool      rtrFlag     = ( msg.id & 0x40000000 );
    bool      extFlag     = ( msg.id & 0x80000000 );
    uint16_t  remoteCanId = (( extFlag ) ? ( msg.id & 0x3FFF ) : ( msg.id & 0x7F ));

    if (( remoteCanId == canId ) && ( msg.dlc > 0 )) {

      return ( ERR_CAN_ID_COLLISION );
    }
    else if ( rtrFlag ) {

      msg.id          = canId;
      msg.dlc         = 0;
      msg.data32[ 0 ] = 0;
      msg.data32[ 1 ] = 0;

      can2040_transmit( &cBus, &msg );
      return ( ERR_CAN_MSG_NO_MSG );
    }
    else {

      memcpy( msgBuf, msg.data, msg.dlc );
      return ( ALL_OK );
    }
  }
  else return ( ERR_CAN_MSG_NO_MSG );
}
