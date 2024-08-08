//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime setup file.
//
//------------------------------------------------------------------------------------------------------------
// The file implements a part of the LcsRuntimeLib that deals with the setup and start sequence of a node.
// There is a lot to do. First, we need to initialize the CDC laer, our lower layer foundation. Next the
// NVM of the nodeMap is located and checked for validity. The nodeMap contains all the information for 
// setting up the entire node. If this steps fails, we either need to configure the nodeMap, or we habe a
// fata error and the node is not usable.
// 
// With a correct node map in place the memory structures for the node, the ports, events, callbacks and 
// periodic tasks are created. The node is basically ready to do work.
//
// Next is the extension board setup. We try to locate all connected extension boards and install the 
// corresponding driver. A driver is jst a procedure that lnows how tp talk to the particular extension 
// board. A failure in this part of the seqeunce sequence does not necessaruly mean tha the node cannot be
// used.
//
// Assuming all went fine, the runtime library is ready to accept calls for registering callbacks and a few
// other library calls. Once all this work is done, the last call of teh node formware would be to start 
// the runtime, which would as the very first thing invoke all registered initialization callbacks and the 
// enter the processing loop. We will not return from that routine.
//
// An error in the setup sequeunce does not necessaruily mean that the node is unusable. For example, when
// the nodeMap is not valid, the setup routine will report an error, but we can still call the runtime
// loop. The runtime loop will handle LCS mesages and also provide the console IO, which in turn allows us
// manually correct the node data for a sucessful restart. In a similar way, extension board errors can be
// be addressed.
// 
// This file contains the library global data declartions if teh LCS lrutime library. All other files will
// refer to them as "extern".
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
// Runtime globals. This file contais the global data strcuture declarations. All other files in the runtime
// loibrary will declare them as "extern" if needed.
//
//------------------------------------------------------------------------------------------------------------
LCS::LcsCdcDesc               cdcMap;
LCS::LcsMsgBusCAN             *msgBus;
LCS::LcsNodeData              nodeData;
LCS::LcsNodeMap               nodeMap;
LCS::LcsPortMap               portMap;
LCS::LcsEventMap              eventMap;
LCS::LcsCallbackMap           callbackMap;
LCS::LcsTaskMap               taskMap;
LCS::LcsDrvMap                drvMap;

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLibConfig implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------  
// Debug and Trace support. Instead of conditional cimpilation, we will print debug messages based on the
// settoin of the debiug level.
//------------------------------------------------------------------------------------------------------------ 
uint8_t debugLevel = 0;

//------------------------------------------------------------------------------------------------------------
// Utility routines for number range check.
//
//------------------------------------------------------------------------------------------------------------
bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

  return (( val >= lower ) && ( val <= upper ));
}

//------------------------------------------------------------------------------------------------------------
// "roundup" rounds up a value to the next highest multiple of the block size.
//
//------------------------------------------------------------------------------------------------------------
uint16_t roundup( uint16_t elements, uint16_t alignSize ) {

  return ((( elements + alignSize - 1 ) / alignSize ) * alignSize );
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultNodeMap" build a nodeMap with default values. 
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultNodeMap( LcsNodeMap *nMap ) {

  nMap -> magicWord1                    = NODE_MWORD_1;
  nMap -> magicWord2                    = NODE_MWORD_2;

  nMap -> options                       = 0;
  nMap -> flags                         = 0;

  nMap -> nodeState                     = NS_NIL;

  nMap -> controllerFamily              = CF_FAM_NIL;
  nMap -> nvmChipFamily                 = CF_FAM_MICROCHIP;
  nMap -> boardType                     = BT_NIL;

  nMap -> nodeSwVersion                 = 0;
  nMap -> nodeSwPatchLevel              = 0;
  nMap -> restartCnt                    = 0;

  nMap -> nodeId                        = NIL_NODE_ID;
  nMap -> nodeUID                       = CDC::createUid( );
  nMap -> nodeType                      = NIL_NODE_TYPE;   

  nMap -> restartCnt                    = 0;

  nMap -> nodeMapNvmOfs                 = NVM_NODE_MAP_START;
  nMap -> portMapNvmOfs                 = NVM_PORT_MAP_START;
  nMap -> nodeDataOfs                   = NVM_NODE_DATA_START;
  nMap -> eventMapNvmOfs                = NVM_EVENT_MAP_START;
  nMap -> userMapNvmOfs                 = NVM_USER_MAP_START;

  nMap -> nvmMemSizeInBlocks            = 0;

  nMap -> portMapOptions                = 0;
  nMap -> portMapFlags                  = 0;
  nMap -> portMapSize                   = 0;

  nMap -> eventMapOptions               = 0;
  nMap -> eventMapFlags                 = 0;
  nMap -> eventMapSize                  = 0;
  nMap -> eventMapHwm                   = 0;

  nMap -> debugEnabled                  = 0;
  nMap -> debugNodeSetup                = 0;
  nMap -> debugNvmAccess                = 0;
  nMap -> debugCanBusMsg                = 0;
  nMap -> debugAttrAccess               = 0;
  nMap -> debugEventHandling            = 0;
  
  memset( &nMap -> name, 0, MAX_NODE_NAME_SIZE );
}

}; // namespace


//------------------------------------------------------------------------------------------------------------
// LCS name space routines.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// The very first thing to do is to setup the CDC library and setup the "active" and "ready" LED pins used by
// the board. The pins need to to be configured. We also make a call to to initialize the CDC. Note that this
// may have been done before, when for example the firmware programmer wants to use the HW before calling any
// library setup code. It is no problem to call the CDC inti routine several times.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCdcLayer( CDC::CdcPinConfig *ci ) {

  CDC::init( ci );

  if ( ci -> READY_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> READY_LED_PIN, CDC::OUT );
  if ( ci -> ACTIVE_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> ACTIVE_LED_PIN, CDC::OUT );

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// The NVM library functions will work after this routine. We first set up the I2C channels, which are the
// heart of any internal board communication. After the I2C channels are initualied, we will configure the
// NVM library. If all is OK, we can talk to all NVMs on the boards making up the node.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initNvmChannels( CDC::CdcPinConfig *ci ) {

  #if DEBUG_CONFIG == 1
  printf( "initNvmChannels: nvmSCL: %d, nvmSDA: %d, nvmSCL: %d, nvmSDA: %d\n", 
          ci -> NVM_I2C_SCL_PIN, ci -> NVM_I2C_SDA_PIN, ci -> EXT_I2C_SCL_PIN, ci -> EXT_I2C_SDA_PIN ); 
  #endif

  uint8_t rStat;

  if (( ci -> NVM_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) && ( ci -> NVM_I2C_SDA_PIN != CDC::UNDEFINED_PIN )) {

    rStat = CDC::configureI2C( ci -> NVM_I2C_SCL_PIN , ci -> NVM_I2C_SDA_PIN );
    if ( rStat != ALL_OK ) return( rStat );
  }

  if (( ci -> EXT_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) && ( ci -> EXT_I2C_SDA_PIN != CDC::UNDEFINED_PIN )) {

    rStat = CDC::configureI2C( ci -> EXT_I2C_SCL_PIN , ci -> EXT_I2C_SDA_PIN );
    if ( rStat != ALL_OK ) return( rStat );
  }

  rStat = configNvm( ci );

  #if DEBUG_CONFIG == 1
  printf( "initNvmChannels, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// Next is CAN bus setup. The message bus is the central communication mechanism. If we can also get it up 
// early we could use it not only for configurations and operations, perhaps even remote troubleshooting. 
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
//This routine sets up the nodeMap. It is the first routine after all the basoc hardware settings is in place.
// First we read in the nodeMap from the node NVM. 
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupNodeMap\n" );
  #endif

  uint8_t rStat = rtNvmGetBytes( NVM_NODE_MAP_START, (uint8_t *) &nodeMap, sizeof( LcsNodeMap ));
  if ( rStat != ALL_OK ) return( rStat );

  if (( nodeMap.magicWord1 != NODE_MWORD_1 ) || ( nodeMap.magicWord2 != NODE_MWORD_2 )) {

    return( ERR_NVM_NODE_MAP_CORRUPT );
  }


  if ( rStat != ALL_OK ) {

    
  }

  
  // ??? what to validate ?


  #if DEBUG_CONFIG == 1
  printf( "setupNodeMap, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupPortMap" will read the port data the NVM port map data area into the memory counterpart.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupPortMap\n" );
  #endif

  uint8_t rStat = rtNvmGetBytes( NVM_PORT_MAP_START, (uint8_t *) &portMap, sizeof( LcsPortMap ));

  #if DEBUG_CONFIG == 1
  printf( "setupPortMap, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The event map stores all event/port pairs this node is interested to process. The map is a sorted map and
// there is a high water mark, so that we only read up to the last used entry in the map. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupEventMap( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupEventMap\n" );
  #endif

  // ??? check for a valid hwm ... then get the map 
  // ??? we may just want to read up to the HWM...!!!!
   // ??? the event map is expected to be sorted as we only add / remove sorted.
  // ??? only read up to the high water mark.

  


  uint8_t rStat;
  
  rStat = rtNvmGetBytes( NVM_EVENT_MAP_START, (uint8_t *) &eventMap, sizeof( LcsEventMap ));
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

  #if DEBUG_CONFIG == 1
  printf( "setupUserMap, status: %d\n", rStat );
  #endif

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupCallbackMap( ) {

   #if DEBUG_CONFIG == 1
  printf( "setupCallbackMap\n" );
  #endif

  uint8_t rStat = ALL_OK;

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

  #if DEBUG_CONFIG == 1
  printf( "setupCallbackMap, status: %d\n", rStat );
  #endif

  return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupTaskMap( ) {

   #if DEBUG_CONFIG == 1
  printf( "setupTaskMap\n" );
  #endif

  uint8_t rStat = ALL_OK;

  taskMap.flags = 0;
  taskMap.size  = MAX_TASK_MAP_ENTRIES;
  taskMap.hwm   = taskMap.map;
  taskMap.next  = taskMap.map;

  for ( int i = 0; i < MAX_TASK_MAP_ENTRIES; i++ ) {

    taskMap.map[ i ].task       = nullptr;
    taskMap.map[ i ].interval   = 0;
    taskMap.map[ i ].timeStamp  = 0;
  }

  #if DEBUG_CONFIG == 1
  printf( "setupTaskMap, status: %d\n", rStat );
  #endif

  return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t detectExtensionBoards( ) {

  #if DEBUG_CONFIG == 1
  printf( "detectExtensionBoards\n" );
  #endif

  uint8_t rStat = ALL_OK;

  for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

       // ??? try to find extension boards ( 1 to 4 )
    }

 #if DEBUG_CONFIG == 1
  printf( "detectExtensionBoards, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// For all detected extension bards, we will onvoke the driver with the "SETUP" item code.
//
// ??? what to do ona failure ? We do not want to sop the entire setup sequence ?
//------------------------------------------------------------------------------------------------------------
uint8_t setupExtensionBoards( ) {

  #if DEBUG_CONFIG == 1
  printf( "setupExtensionBoards\n" );
  #endif

  uint8_t rStat = ALL_OK;

  for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

    if ( drvMap.map[ i ].flags != 0  ) {

      rStat = drvMap.map[ i ].drvFunc( i, 0, 0, nullptr );  // for now ...
      
      // ??? on an error, we just mark the extension as "error" but continue ?
    } 
  }

  #if DEBUG_CONFIG == 1
  printf( "setupExtensionBoards, status: %d\n", rStat );
  #endif

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "invokeInitCallbacks" onvoles the registered initialization callbacks for the node and the ports. The 
// routine is called as the very first thing of the "startRuntime" call.
//
//------------------------------------------------------------------------------------------------------------
uint8_t invokeInitCallbacks( ) {

  #if DEBUG_CONFIG == 1
  printf( "invokeInitCallbacks\n" );
  #endif

  uint8_t rStat = ALL_OK;

  for ( uint8_t i = 0; i < MAX_PORT_MAP_ENTRIES + 1; i++ ) {

    if ( callbackMap.map[ i ].initCallback != nullptr ) {

      rStat = callbackMap.map[ i ].initCallback( nodeMap.nodeId, i, 0 );
      if ( rStat != ALL_OK ) break;
    } 
  }

   #if DEBUG_CONFIG == 1
  printf( "invokeInitCallbacks, status: %d\n", rStat );
  #endif

  return ( rStat );
}



// ??? reset Node and resetPort should go here. What do they actually do ?
// ??? looks like we will also just invoke the init callback...
// ??? would a node reset clear any outstanding requests ?
// ??? would it just drop all periodic tasks ?




//------------------------------------------------------------------------------------------------------------
// "initRuntime" is the routine that takes a controller board and initializes the whole show. It is the very
// first thing to call in a node firmware program. There is a lot to do. First, the CDC layer is initialized.
// NVM and CanBus follow. An error in this stage will result in a fatal error, we are not able to set up a
// vallid runtime.
//
// If the HW setup worked, we are ready to read in the nodeMap. A nodeMap can be valid or not. It is defined 
// as a map with vaid "magic" words and reasonable values for the other fields. In case of an invalid 
// nodeMap, a new defautl map is created and written back to the NVM. An invalid nodeMap could result from 
// erreneous writes to NVM locations or simply a brand new HW board. If all is OK, we have a valid basic 
// nodeMap that we can work from. 
//
// The setup of the portMap follows. In general, each map will have two fields, "options" and "flags". The
// option fields contains bits that are configured and guide the startup and later operations process. The 
// flag field contains dynamic flags that are always resetted on node start or reset. Other fields in a map
// are read in from the NVM first and set to a default state this way.
// 
// The eventMap is a bit special, in that it is a rather large map and potentially only a portion is used. 
// There is an eventMao high water mark field in the nodeMap that will tell how many entries are actually 
// used in the event map. Adding increases, deleting decreases the high water mark. Note that the eventMap
// is a sorted map. Everytime we insert or remove the eventMap is rebuilt. Instead of immediately updating
// the NVM storage, a dedicatecd command will SYNC between the MEM and the NVM eventMap.
//
// Next, we will set up the callback function and task map. We are now ready to register LCS callbacks as 
// well as register periodic tasks. Up to here an error detected will result in a fatal error if there is 
// no console connected.

// If all is OK so far, the extension boards are located, and if there are
// any, their initialization follows.
// 
// If the setup was successful, the firmware prigrammer can register callback functions and also perform 
// LCS library calls. The overall logic of the startup routine is that if there is a fault, the follow on
// steps are simply skipped and the node is put into the FAIL state. Note that we still are able to access 
// the node via the USB console and one day also via certain LCS messages. The idea is to allow the correct
// configuration  of the nodeMap, so that we can restart with a correct nodeMap. 
//
// 
//------------------------------------------------------------------------------------------------------------
uint8_t initRuntime( CDC::CdcPinConfig *ci ) {

  #if DEBUG_CONFIG == 1
  printf( "init LCS runtime\n") ;
  #endif

  uint8_t rStat = ALL_OK;

  if ( rStat == ALL_OK )  rStat = initCdcLayer( ci );
  if ( rStat != ALL_OK )  CDC::fatalErrorMsg((char *) "CDC Setup failed", 1 );
    
  if ( rStat == ALL_OK )  rStat = initCanBus( ci );
  if ( rStat == ALL_OK )  rStat = initNvmChannels( ci );
  if ( rStat != ALL_OK )  CDC::fatalErrorMsg((char *) "CAN bus or NVM Setup failed", 2 );

  if ( rStat == ALL_OK )  rStat = setupNodeMap( );
  if ( rStat == ALL_OK )  rStat = setupPortMap( );
  if ( rStat == ALL_OK )  rStat = setupEventMap( );
  if ( rStat == ALL_OK )  rStat = setupUserMap( );
  if ( rStat == ALL_OK )  rStat = setupCallbackMap( );
  if ( rStat == ALL_OK )  rStat = setupTaskMap( );
  if (( rStat != ALL_OK ) && ( ! CDC::isConsoleConnected( ))) CDC::fatalError( 3 );

  if ( rStat == ALL_OK )  rStat = detectExtensionBoards( );
  if ( rStat == ALL_OK )  rStat = setupExtensionBoards( );

  // ??? what to do when extension board setup fails ?
 
  if ( rStat == ALL_OK )  nodeMap.nodeState = NS_INIT;
  else                    nodeMap.nodeState = NS_FAIL;

  #if DEBUG_CONFIG == 1
  printf( "init LCS runtime, status: %d \n", rStat ) ;
  #endif

  return ( rStat );
}

//-----------------------------------------------------------------------------------------------------------
// "startRuntime" is the main routine of the node activity processing. It is the method called after all 
// setup is done. Running in a loop, the primary function is to handle the activities according to the node 
// state. The run loop also processes the serial commands and periodic tasks. Note that this function will
// not return. When the routine is called, it will as the very first thing invoke all registered init
// callback functions before entering the processing loop.
//
//------------------------------------------------------------------------------------------------------------
void startRuntime( ) {

  uint8_t rStat = ALL_OK;

  // ??? reset calls to node and port ?

  if ( rStat == ALL_OK ) rStat = invokeInitCallbacks( );

  while ( true ) {

    switch ( nodeMap.nodeState ) {

      case NS_INIT:       handleNodeStateInit( );       break;
      case NS_FAIL:       handleNodeStateFail( );       break;
      case NS_REGISTER:   handleNodeStateRegister( );   break;
      case NS_COLLISION:  handleNodeStateCollision( );  break;
      case NS_HALTED:     handleNodeStateHalted( );     break;
      case NS_CONFIG:     handleNodeStateConfig( );     break;
      case NS_OPERATE:    handleNodeStateOperations( ); break;
    }

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_CONFIG )) {

      handlePeriodicTasks( );
      handleNodePortEvents( );
    }

    handleSerialCommand( );
  }
}

}; // namespace LCS