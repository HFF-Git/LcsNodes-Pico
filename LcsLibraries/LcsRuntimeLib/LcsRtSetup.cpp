//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime setup file.
//
//------------------------------------------------------------------------------------------------------------
// The file implements a part of the LcsRuntimeLib that deals with the startup and storage aspects of a node.
//
//
//
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
// Externals.
//
//------------------------------------------------------------------------------------------------------------
extern LCS::LcsMsgBusCAN             *msgBus;
extern LCS::LcsNodeMap               nodeMap;
extern LCS::LcsPortMap               portMap;
extern LCS::LcsEventMap              eventMap;
extern LCS::LcsCallbackMap           callbackMap;

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLibConfig implementation file local declarations and routines.
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
  // Utility routines for number range check.
  //
  //----------------------------------------------------------------------------------------------------------
  static inline bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
  }

  //----------------------------------------------------------------------------------------------------------
  // "roundup" rounds up a value to the next highest multiple of the block size.
  //
  //----------------------------------------------------------------------------------------------------------
  static uint16_t roundup( uint16_t elements, uint16_t alignSize ) {

    return ((( elements + alignSize - 1 ) / alignSize ) * alignSize );
  }

}; // namespace


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// The very first thing to do is to setup the CDC library and setup the "active" and "ready" LED pins used by
// the board. The pins need to to be configured. We also make a call to to initialize the CDC. Note that this
// may have been done before, when for example the firmware programmer wants to use the HW before calling any
// library setup code.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCdcLayer( CDC::CdcPinConfig *ci ) {

  #if DEBUG_CONFIG == 1
  
  #endif

  CDC::init( ci );

  if ( ci -> READY_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> READY_LED_PIN, CDC::OUT );
  if ( ci -> ACTIVE_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> ACTIVE_LED_PIN, CDC::OUT );

  #if DEBUG_CONFIG == 1
  #endif

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// Before we can do anything, we need access to the NVM data. The "nvmInitSubSys" will do that for us. If
// the setup fails and we have no USB console connected, we are at the end and a fatal error will indicate
// that something is quite wrong. If there is a console, we just report an error. The idea is that we are
// with a concole in the position to trouble shoort and perhaps even fix the issue.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initNvm( CDC::CdcPinConfig *ci ) {

  #if DEBUG_CONFIG == 1
  printf( "initNvm\n" );
  #endif

  uint8_t rStat = nvmInitSubSys( ci -> NVM_I2C_SCL_PIN, ci -> NVM_I2C_SDA_PIN, ci -> EXT_I2C_ADR_ROOT );
  if ( rStat != ALL_OK ) {

    if ( CDC::isConsoleConnected( ))  rStat = ERR_NVM_SETUP;
    else                              CDC::fatalError( 2 );
  }

  #if DEBUG_CONFIG == 1
  printf( "initNvm, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// Next is CAN bus setup. The message bus is the central communication mechanism. If we can get it up early
// we could use it not only for configurations and operations, perhaps even remote troubleshooting. TBD.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCanBus( CDC::CdcPinConfig *ci ) {

  #if DEBUG_CONFIG == 1
  printf( "initCanBus\n" );
  #endif

  msgBus = new LcsMsgBusCAN( );

  uint8_t rStat = msgBus -> init( 0, ci -> CAN_BUS_RX_PIN, ci -> CAN_BUS_TX_PIN, ci -> CAN_BUS_CTRL_MODE );
  if ( rStat != ALL_OK ) {

    // ??? check what to actuall return...
  }

  #if DEBUG_CONFIG == 1
  printf( "initCanBus, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// Now that the nodeMap is available, let's do the reqeuired consistency checks.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupNodeMap\n" );
  #endif

  uint8_t rStat = nvmGetBytes( NVM_NODE_MAP_START, (uint8_t *) &nodeMap, sizeof( LcsPortMap ));
  if ( rStat != ALL_OK ) {

    
  }

  
  // ??? what to validate ?


  #if DEBUG_CONFIG == 1
  printf( "setupNodeMap, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupPortMap" will read the last version from the NVM port map data area.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupPortMap\n" );
  #endif

  uint8_t rStat = nvmGetBytes( NVM_PORT_MAP_START, (uint8_t *) &portMap, sizeof( LcsPortMap ));
  if ( rStat != ALL_OK ) {

    
  }

  // ??? anything to validate ?

  #if DEBUG_CONFIG == 1
  printf( "setupPortMap, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The event map stores all event/port pairs this node is interested to process. The map is a sorted map and
// there is a high water mark, so that we do not read the large map when loading it. During setup, all the
// entries up to the water mark are read.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupEventMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupEventMap\n" );
  #endif

  // ??? we may just want to read up to the HWM...!!!!
   // ??? the event map is expected to be sorted as we only add / remove sorted.
  // ??? only read up to the high water mark.

  uint16_t hwm;
  uint16_t size;

  uint8_t rStat = nvmGetWord( NVM_EVENT_MAP_START + offsetof( LcsEventMap, hwm ),&hwm );
  if ( rStat != ALL_OK ) {

    
  }

  rStat = nvmGetWord( NVM_EVENT_MAP_START + offsetof( LcsEventMap, size ),&size );
  if ( rStat != ALL_OK ) {

    
  }

  // ??? check for a valid hwm ... then get the map


  rStat = nvmGetBytes( NVM_EVENT_MAP_START, (uint8_t *) &eventMap, sizeof( LcsEventMap ));
  if ( rStat != ALL_OK ) {

    
  }

  // ??? set the HWM and SIZE from NVM in the MEM structure.

  // ??? anything to validate ?

  #if DEBUG_CONFIG == 1
  printf( "setupEventMap, status: %d\n", rStat );
  #endif

  return ( rStat );
}


//------------------------------------------------------------------------------------------------------------
// The user map is the additional NVM storage that the chip set offers beyond the area allocatred for the 
// system. Since we have no idea what the user is doing, we just do the basic validation checks.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupUserMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupUserMap\n" );
  #endif

  uint8_t rStat = ALL_OK;

  // ??? anything to validate ?

  #if DEBUG_CONFIG == 1
  printf( "setupUserMap, status: %d\n", rStat );
  #endif

  return ( rStat );
}


// ??? setup callback Map ?

// ??? setup task map ?


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t detectExtensionBoards( ) {

  #if DEBUG_CONFIG == 1
  printf( "detectExtensionBoards\n" );
  #endif

  uint8_t rStat = ALL_OK;

  // ??? try to find extension boards ( 1 to 4 )

 #if DEBUG_CONFIG == 1
  printf( "detectExtensionBoards, status: %d\n", rStat );
  #endif

  return ( rStat );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupDriverEnv( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupDriverEnv\n" );
  #endif

  uint8_t rStat = ALL_OK;

  // ??? init each driver

  #if DEBUG_CONFIG == 1
  printf( "setupDriverEnv, status: %d\n", rStat );
  #endif

  return ( rStat );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupExtensionBoards( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupExtensionBoards\n" );
  #endif

  uint8_t rStat = detectExtensionBoards( );

  if ( rStat == ALL_OK ) {

    rStat = setupDriverEnv( );
  }

  #if DEBUG_CONFIG == 1
  printf( "setupExtensionBoards, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t invokeInitCallbacks( ) {

  #if DEBUG_CONFIG == 1
  printf( "invokeInitCallbacks\n" );
  #endif

  uint8_t rStat = ALL_OK;

  // ??? invoke registered user callbacks

   #if DEBUG_CONFIG == 1
  printf( "invokeInitCallbacks, status: %d\n", rStat );
  #endif

  return ( rStat );
}


//------------------------------------------------------------------------------------------------------------
// "initRuntime" is the routine that takes a controller board and initializes the whoe show. There is a lot
// to do. First, the CDC layer is initualized. NVM and CanBus follow. If all is OK, we have a valid basic
// nodeMap, we can work from. If the nodeMap read is not valid, the node enters the error state and the
// node map can be configured / corrected via teh USB console IO. This will be for example always be the case
// when a new board is powered up.
//
// In the other cases, portMap, eventMap and userMap setup follows. Now, we have the controller basic data
// structures in a reasonable shape. Next, the extension boards are located, and if there are any, their
// initialization follows. Finally, all registered callbacks are invoked. 
//
// The overal logic of the startup provcess is that if there is a fault, the follow on steps are simply 
// skipped. If the basic setup worked bit the nodeMap is not valid, the map can be configured via the USB
//  console IO commands. During this time the node is in "fault" state. After configuration, simply reset
// again and the provcess is repeated, this time hopefully with a valid nodeMap.
//

// ??? what is the sequence ? always do the init as the very first thing, then do registration, etc. 
// and then call startruntime ?
//

//------------------------------------------------------------------------------------------------------------
uint8_t initRuntime( CDC::CdcPinConfig *ci ) {

  #if DEBUG_CONFIG == 1
  printf( "init LCS runtime\n") ;
  #endif

  uint8_t rStat = ALL_OK;

  if ( rStat == ALL_OK ) rStat = initCdcLayer( ci );
  if ( rStat == ALL_OK ) rStat = initNvm( ci );
  if ( rStat == ALL_OK ) rStat = initCanBus( ci );

  if ( rStat == ALL_OK ) rStat = setupNodeMap( );
  if ( rStat == ALL_OK ) rStat = setupPortMap( );
  if ( rStat == ALL_OK ) rStat = setupEventMap( );
  if ( rStat == ALL_OK ) rStat = setupUserMap( );

  if ( rStat == ALL_OK ) rStat = setupExtensionBoards( );
  if ( rStat == ALL_OK ) rStat = invokeInitCallbacks( );

  // ??? anything else ? Setting the final nodeState ?
  if ( rStat == ALL_OK ) ;

  #if DEBUG_CONFIG == 1
  printf( "init LCS runtime, status: %d \n", rStat ) ;
  #endif

  return ( rStat );
}

}; // namespace LCS