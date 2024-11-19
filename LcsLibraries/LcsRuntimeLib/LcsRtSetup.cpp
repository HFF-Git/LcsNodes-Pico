//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime setup file.
//
//------------------------------------------------------------------------------------------------------------
// The file implements a part of the LcsRuntimeLib that deals with the setup and start sequence of a node.
// There is a lot to do. First, we need to initialize the CDC layer, our lower layer foundation. Next the
// NVM of the nodeMap is located and checked for validity. The nodeMap contains all the information for 
// setting up the entire node. If this steps fails, we either need to configure the nodeMap, or we have a
// data error and the node is not usable.
// 
// With a correct node map in place, the memory structures for the node, the ports, events, callbacks and 
// periodic tasks are created. The node is basically ready to do work. For a node that has no extension
// boards connected, we are done.
//
// Next is the extension board setup. We try to locate all connected extension boards and install the 
// corresponding driver. A driver is just a procedure that knows how to talk to the particular extension 
// board. A failure in this part of the sequence sequence does not necessarily mean that the node cannot be
// used.
//
// Assuming all went fine, the runtime library is ready to accept calls for registering callbacks and a few
// other library calls. Once all this work is done, the last call of the node firmware would be to start 
// the runtime, which would as the very first thing invoke all registered initialization callbacks and the 
// enter the processing loop. We will not return from that routine.
//
// An error in the setup sequence does not necessarily mean that the node is unusable. For example, when
// the nodeMap is not valid, the setup routine will report an error, but we can still call the runtime
// loop. The runtime loop will handle LCS messages and also provide the console IO, which in turn allows us
// manually correct the node data for a successful restart. In a similar way, extension board errors can be
// be addressed.
// 
// This file contains the library global data declarations if the LCS runtime library. All other files will
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
#include "LcsDrvOccDetectLib.h"
#include "LcsDrvServoLib.h"

//------------------------------------------------------------------------------------------------------------
// Runtime globals. This file contains the global data structure declarations. They are declared in the LCS
// name space. All other files in the runtime library will declare them as "extern" if needed.

// There is also the debug mask. The idea is to have a debug mask where each major part of the library has a
// bit. There could also be bits reserved for the firmware. Then we have control items to set these bits. 
// Wherever debugging or tracing is needed, the bit mask will be used to determine whether to print debugging
// data or not. From a performance perspective, the test will take just a couple of ARM instructions. In other
// words we do not take out debugging code when going into production. Never liked this approach of conditional
// debug code via "ifdefs".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    uint16_t                    debugMask    = 0;
    uint16_t                    startOptions = 0;

    LCS::LcsCdcDesc             cdcMap;
    LCS::LcsMsgBusCAN           *msgBus;
    LCS::LcsNodeData            nodeData;
    LCS::LcsNodeMap             nodeMap;
    LCS::LcsPortMap             portMap;
    LCS::LcsEventMap            eventMap;
    LCS::LcsCallbackMap         callbackMap;
    LCS::LcsPendingReqMap       pendingReqMap;
    LCS::LcsTaskMap             taskMap;
    LCS::LcsDrvFuncMap          drvFuncMap;
    LCS::LcsDrvMap              drvMap;
}
    
//------------------------------------------------------------------------------------------------------------
// The LcsCoreLibConfig implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Utility routines and constants.
//
//------------------------------------------------------------------------------------------------------------
const char  *nodeDefName  = "Node Name";

uint16_t roundup( uint16_t elements, uint16_t alignSize ) {

    return ((( elements + alignSize - 1 ) / alignSize ) * alignSize );
}

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

//------------------------------------------------------------------------------------------------------------
// "buildDefaultNodeMap" build a nodeMap with the default values from the declaration structure. The default
// nodeMap is used for initializing the memory runtime node map for formatting a new or corrupted runtime NVM.
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultNodeMap( LcsNodeMap *nMap ) {

    LcsNodeMap tmp;

    memcpy( tmp.name, nodeDefName, strlen( nodeDefName ));
    tmp.nodeUID = CDC::createUid( );

    *nMap = tmp;
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultPortMap" will initialize the portMap data structure on MEM or NVM, It is just an array of 
// portMap entries. Each port will get a default name.
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultPortMap( LcsPortMap *pMap ) {

    for ( uint16_t i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        LcsPortMapEntry pEntry;

        snprintf( pEntry.name, MAX_PORT_NAME_SIZE, "Port: %d", i );
        pMap -> map[ i ] = pEntry;
    }
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultEventMap" initializes the event map on MEM or NVM. It is just an array of eventMap entries. 
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultEventMap( LcsEventMap *eMap ) {

    LcsEventMapEntry e;
    for ( uint16_t i = 0; i < MAX_EVENT_MAP_ENTRIES; i++ ) eMap -> map[ i ] = e;
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultNodeData" initializes the node data map on MEM or NVM. It is just an array of variables. We 
// clear them out.
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultNodeData( LcsNodeData *nData ) {

    memset( nData -> map, 0, MAX_NODE_DATA_BLOCKS * MAX_ATTR_MAP_ENTRIES * sizeof( uint16_t));
}

//------------------------------------------------------------------------------------------------------------
// "buildNvmRuntimeStructures" initializes a new or corrupt runtime NVM with default data. After successful
// completion, we will have a valid runtime map.
//
//------------------------------------------------------------------------------------------------------------
uint8_t buildNvmRuntimeStructures( ) {

    uint8_t rStat;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "buildNvmRuntimeStructures\n" );

    rtNvmClearArea( NVM_NODE_MAP_START, NVM_RUNTIME_AREA_SIZE );

    buildDefaultNodeMap( &nodeMap );
    buildDefaultPortMap( &portMap );
    buildDefaultEventMap( &eventMap );
    buildDefaultNodeData( &nodeData );

    rStat = rtNvmPutBytes( NVM_NODE_MAP_START, (uint8_t *) &nodeMap, sizeof( LcsNodeMap ));
    if ( rStat == ALL_OK ) rtNvmPutBytes( NVM_PORT_MAP_START, (uint8_t *) &portMap, sizeof( LcsPortMap ));
    if ( rStat == ALL_OK ) rtNvmPutBytes( NVM_EVENT_MAP_START, (uint8_t *) &eventMap, sizeof( LcsEventMap ));
    if ( rStat == ALL_OK ) rtNvmPutBytes( NVM_NODE_DATA_START, (uint8_t *) &nodeData, sizeof( LcsNodeData ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "buildNvmRuntimeStructures, stat: %d\n", rStat  );

    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultBoardDesc" initializes the extension board structure. An extension board has a header
// structure identical to the nodeMap structure. In addition, there is the driver data area. 
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultBoardDesc( LcsDrvBoardDesc *bDesc ) {

    LcsNvmHeader tmp;

    bDesc -> head = tmp;
    memset( bDesc -> driverData, 0, MAX_DRV_DATA_SIZE * sizeof( uint16_t ));
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultDrvMap" initializes the driver map memory structure.
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultDrvMap( LcsDrvMap *drv ) {

    LcsDrvBoardDesc initDesc;

    for ( uint16_t i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i ++ ) {

        drvMap.map[ i ].flags       = 0;
        drvMap.map[ i ].drvFunc     = nullptr;
        drvMap.map[ i ].extBoard    = initDesc;
    }
}

//------------------------------------------------------------------------------------------------------------
// "buildNvmRuntimeStructures" initializes a new or corrupt runtime NVM with default data. However, the write
// will only succeed when we have the board write enabled. Otherwise an error is returned.
//
//------------------------------------------------------------------------------------------------------------
uint8_t buildNvmExtBoardStructure( uint8_t boardId ) {

    uint8_t     rStat;
    LcsDrvEntry entry;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {
        
        printf( "buildNvmExtBoardStructure for board: %d\n", boardId );
    }

    rStat = extNvmClearArea( boardId, 0, sizeof( LcsDrvEntry ));
    if ( rStat == ALL_OK ) extNvmPutBytes( boardId, 0, (uint8_t *) &entry, sizeof( entry ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "buildNvmExtBoardStructure, stat: %d\n", rStat  );
    }

    return( rStat ); // ??? own error constant ?
}

}; // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// When the node is powered on, the very first thing to do is to setup the CDC library and setup the "active"
// and "ready" LED pins used by the board. The pins need to to be configured. We also make a call to to 
// initialize the CDC. Note that this may have been done before, when for example the firmware programmer 
// wants to use the HW before calling any library setup code. It is no problem to call the CDC init routine
// several times.
//
// There are two basic modes. The first is when we have a console connected. We will prompt and wait for a
// start command. There are several options for starting a node. The easiest is "R" which just starts the 
// node. The "D" command will start with debugging enabled. We will set the setup debug flags to check any
// issues during the startup phase. Finally, there is there "F" command, which will format the NVM runtime 
// area. However, all that is happening in this routine is to set these options to be executed at the right
// place in the setup sequence.
//
// The second mode is when there no console connected. In this case, Debug is disabled and we just setup
// the node. This mode should be the normal case for all the nodes in a layout.
// 
// Perhaps one day, this routine could be enhanced to allow commands to pile up the start options followed
// by the final start command to get the show going. especially the debug mask would be a candidate.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCdcLayer( CDC::CdcConfigDesc *ci ) {

    const uint32_t CONSOLE_TIMEOUT = 1024 * 1024 * 4;

    cdcMap.cfg = *ci;

    CDC::init( ci );

    if ( ci -> READY_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> READY_LED_PIN, CDC::OUT );
    if ( ci -> ACTIVE_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> ACTIVE_LED_PIN, CDC::OUT );

    if ( CDC::isConsoleConnected( )) {

        CDC::writeDio( ci -> READY_LED_PIN, true );

        while ( true ) {

            printf( "=>" );   

            char ch = CDC::getConsoleChar( CONSOLE_TIMEOUT );

            if (( ch == 'R' ) || ( ch == 'r' )) {

                printf( "Starting - normal mode\n" );

                debugMask       &= ~ DBG_CONFIG;
                startOptions    = NOPT_NIL;
                return( ALL_OK );
            }
            else if (( ch == 'D' ) || ( ch == 'd' )) {

                 printf( "Starting - debug mode\n" );

                debugMask       = DBG_CONFIG | DBG_SETUP;
                startOptions    = NOPT_NIL;
                return( ALL_OK );
            }
            else if (( ch == 'F' ) || ( ch == 'f' )) {

                 printf( "Starting - format mode\n" );

                debugMask       &= ~ DBG_CONFIG;

                debugMask       = DBG_CONFIG | DBG_SETUP;
                
                startOptions    = NOPT_FORMAT_RUNTIME;
                return( ALL_OK );
            }
            else if ( ch == '?' ) {

                printf( "Setup options:\n" );
                printf( "r, R -> start the node with debug initially disabled\n" );
                printf( "d, D -> start the node with \"setup\" debug options enabled\n" );
                printf( "f, F -> start the node with a newly formatted runtime map\n" );
            }
            else printf( "\n" );
        }
    }
    else {
        
        debugMask       &= ~ DBG_CONFIG;
        startOptions    = NOPT_NIL;
        return ( ALL_OK );
    }
}

//------------------------------------------------------------------------------------------------------------
// The NVM library functions will work after this routine. We first set up the I2C channels, which are the
// heart of any internal board communication. After the I2C channels are initialized, we will configure the
// NVM library. If all is OK, we can talk to all NVMs on the boards making up the node.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initNvmChannels( CDC::CdcConfigDesc *ci ) {

   if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) { 

        printf( "initNvmChannels: nvmSCL: %d, nvmSDA: %d, extSCL: %d, extSDA: %d\n", 
                ci -> NVM_I2C_SCL_PIN, ci -> NVM_I2C_SDA_PIN, ci -> EXT_I2C_SCL_PIN, ci -> EXT_I2C_SDA_PIN ); 
    }

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

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "initNvmChannels, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// Next is CAN bus setup. The message bus is the central communication mechanism. If we can also get it up 
// early we could use it not only for configurations and operations but perhaps for remote troubleshooting. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCanBus( CDC::CdcConfigDesc *ci ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "initCanBus\n" );
    
    msgBus = new LcsMsgBusCAN( );

    uint8_t rStat = msgBus -> init( 0, ci -> CAN_BUS_RX_PIN, ci -> CAN_BUS_TX_PIN, ci -> CAN_BUS_CTRL_MODE );
    if ( rStat != ALL_OK ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
            printf( "initCanBus, CAN status: %d\n", rStat );
        
        rStat = ERR_CAN_SETUP;
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "initCanBus, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupNodeMap" sets up the nodeMap. It is the first routine after all the basic hardware settings is in 
// place. Unless the start options tell us to just format a new runtime area, we read in the nodeMap from 
// the node NVM. A quick check of the magic words and the the nodeMap size field will tell us whether this
// nodeMap was initialized before. If this is not the case, we must assume a corrupt nodeMap or a new board. 
// The runtime area data structures are created with default values and written to the NVM.
//
// In any case, the follow-on setup routines can assume a valid data structure to work from and just read 
// the NVM as normal. If this routine has an error it should be considered as a fatal error.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeMap( LcsConfigDesc *cfg ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupNodeMap\n" );

    if ( startOptions & NOPT_FORMAT_RUNTIME ) {

        rStat = buildNvmRuntimeStructures( );
    }

    rStat = rtNvmGetBytes( NVM_NODE_MAP_START, (uint8_t *) &nodeMap, sizeof( LcsNodeMap ));
    if ( rStat != ALL_OK ) return( rStat );

     if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        uint16_t *ptr = (uint16_t *) &nodeMap;

        printf( "NodeMap Head: " );
        for ( int i = 0; i < 16; i++ ) printf( "0x%x ", ptr[ i ] );
        printf( "\n" );
     }

    if (( nodeMap.head.magicWord1 != NVM_MWORD_1 ) || 
        ( nodeMap.head.magicWord2 != NVM_MWORD_2 ) || 
        ( nodeMap.nodeMapSize != sizeof( LcsNodeMap ))) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
            printf( "setupNodeMap: invalid header, re-format\n" );

        rStat = buildNvmRuntimeStructures( );
    }

    nodeMap.nodeOptions = cfg -> options;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupNodeMap, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupPortMap" will read the port data the NVM port map data area into the memory counterpart.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupPortMap\n" );

    uint8_t rStat = rtNvmGetBytes( NVM_PORT_MAP_START, (uint8_t *) &portMap, sizeof( LcsPortMap ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupPortMap, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupNodeDataMap" will read the node data blocks.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeDataMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupNodeDataMap\n" );

    uint8_t rStat = rtNvmGetBytes( NVM_NODE_DATA_START, (uint8_t *) &nodeData.map, sizeof( nodeData.map ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupNodeDataMap, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The event map stores all event/port pairs this node is interested to process. The map is a sorted map and
// there is a high water mark, so that we only read up to the last used entry in the map. Just like other 
// data structures we could just read in all entries. However, this is a large map. It would be better to 
// just read up to the HWM, if the HWM is valid. If this is not the case, we have to assume that there are
// bigger issues with the event map. In this case we will read the entire map entry by entry, add used 
// entries, i.e. entries with a non-NIL event ID to the memory map. After reading all entries, the newly
// created event map is written back to the NVM place.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupEventMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {
        
        printf( "setupEventMap, entries: %d, HWM: %d\n", nodeMap.eventMapEntries, nodeMap.eventMapHwm );
    }
   
    uint8_t rStat = ALL_OK;

    if ( nodeMap.eventMapHwm <= MAX_EVENT_MAP_ENTRIES ) {

        for ( uint16_t i = 0; i < nodeMap.eventMapHwm; i++ ) {

            rStat = rtNvmGetBytes(  NVM_EVENT_MAP_START + i * sizeof( LcsEventMapEntry), 
                                    (uint8_t *) &eventMap.map[ i ], 
                                    sizeof( LcsEventMapEntry ));
        }
    }
    else {

        LcsEventMapEntry e;
        for ( uint16_t i = 0; i < nodeMap.eventMapEntries; i++ ) eventMap.map[ i ] = e;

        nodeMap.eventMapHwm = 0;
        for ( uint16_t i = 0; i < nodeMap.eventMapEntries; i++ ) {

            LcsEventMapEntry eventEntry;

            rStat = rtNvmGetBytes(  NVM_EVENT_MAP_START + i * sizeof(LcsEventMapEntry), 
                                    (uint8_t *) &eventEntry, 
                                    sizeof(LcsEventMapEntry));

            if (( rStat == ALL_OK ) && ( eventEntry.eventId != NIL_EVENT_ID )) {

                addEvent( eventEntry.eventId, eventEntry.portId );
            }
        }

        rStat = syncEventMap( );
    }
  
    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupEventMap, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The user map is the additional NVM storage that the chip set offers beyond the area allocated for the 
// system. Since we have no idea what the user is doing, we do nothing for now. It is just a placeholder.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupUserMap( ) {

   if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupUserMap\n" );

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupUserMap, status: %d\n", rStat );
    
    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "setupCallbackMap" initializes the callback map. We expect the user to register their callbacks between
// the runtime init and runtime start routine.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupCallbackMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupCallbackMap\n" );

    uint8_t rStat = ALL_OK;

    callbackMap.lcsMsgCallback      = nullptr;
    callbackMap.dccMsgCallback      = nullptr;
    callbackMap.cmdLineCallback     = nullptr;

    callbackMap.initCallback        = nullptr;
    callbackMap.resetCallback       = nullptr;
    callbackMap.pfailCallback       = nullptr;

    callbackMap.eventCallback       = nullptr;
    callbackMap.reqCallback         = nullptr;
    callbackMap.repCallback         = nullptr;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupCallbackMap, status: %d\n", rStat );
    
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupTaskMap" initializes the task map. A user can register routines that are executed on a periodic
// basis.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupTaskMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupTaskMap\n" );

    uint8_t rStat = ALL_OK;

    nodeMap.taskMapEntries  = MAX_TASK_MAP_ENTRIES;
    nodeMap.taskMapHwm      = 0;

    for ( int i = 0; i < MAX_TASK_MAP_ENTRIES; i++ ) {

        taskMap.map[ i ].task       = nullptr;
        taskMap.map[ i ].interval   = 0;
        taskMap.map[ i ].timeStamp  = 0;
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupTaskMap, status: %d\n", rStat );
    
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupPendingReqMap" initializes the pending request map. Currently, we do not use a HWM approach, but 
// just use all entries when searching the map.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupPendingReqMap( ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupPendingReqMap\n" );

    nodeMap.pendingMapEntries   = MAX_PENDING_REQ_MAP_ENTRIES;
    nodeMap.pendingMapHwm       = MAX_PENDING_REQ_MAP_ENTRIES;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupPendingReqMap, status: %d\n", rStat );
    
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupDrvFuncMap" initializes the driver function label map. This table is used when we need to find the 
// driver for an extension board type.
//
// ??? we could pre register all known drivers... a user could still register a new one and also overwrite
// a pre-registered driver with a new func label.
//------------------------------------------------------------------------------------------------------------
uint8_t setupDrvFuncMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupDrvLabelMap\n" );

    uint8_t rStat = ALL_OK;

    nodeMap.drvFuncMapEntries = MAX_DRV_TYPES;
    nodeMap.drvFuncMapHwm     = MAX_DRV_TYPES;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupDrvLabelMap, status: %d\n", rStat );
    
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupDrvMap" initializes the driver map. For each possible extension board, up to four, there is an
// entry in this map.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupDrvMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupDrvMap\n" );

    uint8_t rStat = ALL_OK;

    nodeMap.pendingMapEntries   = MAX_PENDING_REQ_MAP_ENTRIES;
    nodeMap.pendingMapHwm       = MAX_PENDING_REQ_MAP_ENTRIES;

    buildDefaultDrvMap( &drvMap );

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupDrvMap, status: %d\n", rStat  );
    
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The runtime library will one day perhaps a set of internal functions to execute periodically. Right now,
// this routine will do nothing.
//
//------------------------------------------------------------------------------------------------------------
uint8_t registerInternalTasks( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "registerInternalTasks\n" );

    uint8_t rStat = ALL_OK;


    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "registerInternalTasks, status: %d\n" );
    
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// With the node properly initialized, it is time to see whether we have connected extension boards. An 
// extension board has also a small NVM on the board that will tell us what the board type is and keep driver
// configuration data. There can be up to four boards, numbered from 0 to 3. The runtime has a driver map 
// with four extension descriptor entries. After initializing the driver map, we attempt to read from each 
// extension NVM on an extension board. If the read fails, there is no board at that location, which will 
// mark in the flags field. The drvMap HWM will tell us how many board we actually found. Note that the 
// boards are by hardware always connected in the order 0,1,2 and 3. If for example only two boards are 
// connected, the HWM would be 2.
//
//------------------------------------------------------------------------------------------------------------
uint8_t detectExtensionBoards( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "detectExtensionBoards\n" );

    uint8_t rStat = ALL_OK;

    for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP ))
            printf( "detectExtensionBoard, boardId: %d\n", i ); 

        LcsDrvEntry *drvEntry = &drvMap.map[ i ]; 

        rStat = extNvmGetBytes( i, 0, (uint8_t *) &drvEntry -> extBoard, sizeof( LcsDrvBoardDesc ));
        if ( rStat == ALL_OK ) {

            uint16_t *ptr = (uint16_t *) &drvEntry -> extBoard;

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                printf( "Extension Board Desc Head: " );
                for ( int j = 0; j < 16; j++ ) printf( "0x%x ", ptr[ j ] );
                printf( "\n" );
            }

            nodeMap.nodeFlags |= NFLAGS_EXT_PRESENT;
            nodeMap.drvMapHwm ++; 

            drvEntry -> flags |= BF_EXT_BOARD_PRESENT;

            if (( drvEntry -> extBoard.head.magicWord1 == NVM_MWORD_1 ) && 
                ( drvEntry -> extBoard.head.magicWord2 == NVM_MWORD_2 )) {

                drvEntry -> flags |= BF_EXT_BOARD_VALID;

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP ))
                    printf( "detectExtensionBoard, boardId: %d -> valid\n", i ); 
            }
            else  {

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP ))
                    printf( "detectExtensionBoard, boardId: %d -> inValid\n", i ); 
            }
        } 
        else {
            
            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP ))
                printf( "detectExtensionBoard, boardId: %d, rStat: %d\n", i, rStat ); 
        }          
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "detectExtensionBoards, status: %d\n", ALL_OK );

    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "lookupDrvFunc" searches the driver function label map. It is called when we setup the extension board.
// When the type is unknown, a nullptr is returned.
//
//------------------------------------------------------------------------------------------------------------
LcsDrvReqFunc lookupDrvFunc( uint16_t drvType ) {

    for ( int i = 0; i < MAX_DRV_TYPES; i++ ) {

        if ( drvFuncMap.map[ i ].drvType == drvType ) return( drvFuncMap.map[ i ].drvFunc );
    }

    return( nullptr );
}

//------------------------------------------------------------------------------------------------------------
// For all detected extension boards, we will first check that the board descriptor at slot "n" is there and
// that the board descriptor is reasonable. If so, the board type is used to load the respective driver.
// Note that during normal operations we cannot manipulate the NVM, as it is read protected. The jumper on
// the extension board needs to be removed for this. When removed, the extension board NVM can be written to
// with commands from the runtime. 
//
// When the driver function is not pre registered, we do not have a function to set in the driver map.
// In this case the driver is not usable yet. The problem is that we can only make driver registration 
// calls after LcsInitRuntime. But the all driver boards have been detected. There are two ways. For driver
// know already, we "pre-register" them. For any other driver, the driver function registration routine will
// check the driver table for the driver type and the patch the function label. All this should takes place 
// before the final call to "startRuntime".
// 
//------------------------------------------------------------------------------------------------------------
uint8_t setupExtensionBoards( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupExtensionBoards\n" );

    uint8_t rStat = ALL_OK;

    for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        LcsDrvEntry *drvEntry = &drvMap.map[ i ]; 

        if (( drvEntry -> flags & BF_EXT_BOARD_PRESENT ) && ( drvEntry -> flags & BF_EXT_BOARD_VALID )) {

            LcsDrvReqFunc drvFunc = lookupDrvFunc( drvEntry -> extBoard.head.boardType ); 

            if ( drvFunc != nullptr ) {

                drvEntry -> drvFunc = drvFunc; 
                drvEntry -> flags   |= BF_EXT_BOARD_READY;
                drvEntry -> lastErr = drvEntry -> drvFunc( i, ITEM_ID_RESET, nullptr, nullptr );

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {
                    
                    printf( "Driver setup, type: %d, stat: %d\n", 
                        drvEntry -> extBoard.head.boardType, drvEntry -> lastErr );
                }
            }
            else {

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                    printf( "Driver setup, type not found: %d\n", drvEntry -> extBoard.head.boardType );
                }
            }
        }
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupExtensionBoards, status: %d\n", rStat );

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// Driver function registration. There is a simple table which maintains extension boards types and the 
// driver function form them. If the type is already registered, we just overwrite the function signature.
// Otherwise we find a free entry and use it.
//
// The driver function registration can only be called after the initialization of the LCS runtime. By then 
// the extension boards are however already detected, and if there are no drivers pre-registered no driver 
// function was registered. The extension board is therefore not ready and was also not reseted. Consequently, 
// the  registration call will check or detected valid extension boards and patch the driver function label 
// and invoke the RESET request.
//
//------------------------------------------------------------------------------------------------------------
uint8_t registerDrvFunc(  uint16_t drvType, LcsDrvReqFunc drvReqFunction ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "registerDrvFunc, type: %d, func: %p\n", drvType, drvReqFunction );

    bool found = false;

    for ( int i = 0; i < MAX_DRV_TYPES; i++ ) {

        if ( drvFuncMap.map[ i ].drvType == drvType ) {

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, overwrite: %d\n", i );

            drvFuncMap.map[ i ].drvFunc = drvReqFunction;
            found = true;
            break;
        }
    }

    if ( ! found ) {

        for ( int i = 0; i < MAX_DRV_TYPES; i++ ) {

            if ( drvFuncMap.map[ i ].drvType == BT_NIL ) {

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                    printf( "registerDrvFunc, allocate: %d\n", i );

                drvFuncMap.map[ i ].drvType = drvType;
                drvFuncMap.map[ i ].drvFunc = drvReqFunction;
                found = true;
                break;
            }
        }
    }

    if ( ! found ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, table full\n" );
    
        return( ERR_DRV_FUNC_MAP_FULL );
    }

    for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        LcsDrvEntry *drvEntry = &drvMap.map[ i ]; 

        if (( drvEntry -> flags & BF_EXT_BOARD_PRESENT ) && ( drvEntry -> flags & BF_EXT_BOARD_VALID )) {

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, set func for board: %d\n", i );

            if ( drvEntry -> extBoard.head.boardType == drvType ) {

                drvEntry -> drvFunc     = drvReqFunction;
                drvEntry -> flags       |= BF_EXT_BOARD_READY;
                drvEntry -> lastErr     = drvEntry -> drvFunc( i, ITEM_ID_RESET, nullptr, nullptr );
            }

        }
     }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, ret: ALL_OK\n" );

    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "powerFailHandler" is the routine called when the hardware detects an imminent loss of power. We will save
// crucial data to NVM. Finally, the optionally registered firmware power fail callback is called. The node
// state becomes "PFAIL".
//
//------------------------------------------------------------------------------------------------------------
uint8_t powerFailHandler( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "powerFailHandler\n" );
    
    uint8_t rStat = ALL_OK;

    nodeMap.nodeState = NS_PFAIL;
    rStat = rtNvmPutWord( NVM_NODE_MAP_START + offsetof( LcsNodeMap, nodeState ), NS_PFAIL );
    
    if ( callbackMap.pfailCallback != nullptr ) callbackMap.pfailCallback( nodeMap.nodeId );

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "powerFailHandler, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "resetNode" restarts a node. We first rebuild the MEM areas from their NVM counterparts. Next, the optional
// reset call back is invoked. Finally, all ports are reseted as well. 
//
// ??? read NVM to MEM.
// ??? would a node reset clear any outstanding requests ? or do we need to inform potential waiters ?
// ??? would it just drop all periodic tasks ? who registers them again ?
// ??? how do we make sure we only cover ports that are used ?
// ??? or would we need to be a bit more sensible what reset mean ?
//------------------------------------------------------------------------------------------------------------
uint8_t resetNode( uint16_t npId ) {

    uint8_t rStat = ALL_OK;

    if ( callbackMap.resetCallback != nullptr ) {

         // ??? load MEM from NVM....

        if ( callbackMap.resetCallback != nullptr ) {

            rStat = callbackMap.resetCallback( buildNpId( nodeMap.nodeId, 0  ));
        }

        
    }

    if ( rStat == ALL_OK ) {

        for ( uint8_t i = 1; i <= MAX_PORT_MAP_ENTRIES; i++ ) {

            // ??? load MEM from NVM....

            if ( callbackMap.resetCallback != nullptr ) {

                rStat = callbackMap.resetCallback( buildNpId( nodeMap.nodeId, i ));
            }
        }
    }

    if ( rStat == ALL_OK ) {

        // ??? reset the drivers...
    }

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The LCS library has a set of configuration parameters. Currently this is only the "options" field. In the
// future there may be more fields. This routines returns a config structure with reasonable defaults set.
//
//------------------------------------------------------------------------------------------------------------
LcsConfigDesc getConfigDefault( ) {

    LcsConfigDesc cfg;

    cfg.options = 0;

    return( cfg );
}

//------------------------------------------------------------------------------------------------------------
// "initRuntime" is the routine that takes a controller board and initializes the whole show. It is the very
// first thing to call in a node firmware program. There is a lot to do. First, the CDC layer is initialized.
// NVM and CanBus follow. An error in this stage will result in a fatal error, we are not able to set up a
// valid runtime.
//
// If the HW setup worked, we are ready to read in the nodeMap. A nodeMap can be valid or not. It is defined 
// as a map with valid "magic" words and reasonable values for the other fields. In case of an invalid 
// nodeMap, a new default map is created and written back to the NVM. An invalid nodeMap could result from 
// erroneous writes to NVM locations or simply a brand new HW board. If all is OK, we have a valid basic 
// nodeMap that we can work from. 
//
// The setup of the portMap follows. The flag field contains dynamic flags that are always reseted on node 
// start or reset. Other fields in a map are read in from the NVM first and set to a default state this way.
// 
// The eventMap initialization is a bit special, in that it is a rather large map and potentially only a 
// portion is used. There is an eventMao high water mark field in the nodeMap that will tell how many entries
// are actually used in the event map. Adding increases, deleting decreases the high water mark. Note that 
// the eventMap is a sorted map. Every time we insert or remove the eventMap is rebuilt. Instead of
// immediately updating the NVM storage, a dedicated command will SYNC between the MEM and the NVM eventMap.
//
// Next, we will set up the pending request map, callback function and task map. They are just memory data 
// areas to be initialized. Up to here an error detected will result in a fatal error. If a console is 
// connected the error messages are listed for analysis.
//
// If all is OK so far, extension boards are located, and if there are any, their initialization follows. 
// First, we try to detect any. For all detected entries we validate the extension board NVM header and 
// set the driver for a valid header found. The driver data area is copied to its memory counter part. 
// All drivers are ready by then. 
// 
// The overall logic of the startup routine code below is that if there is a fault, the follow on steps are
// simply  skipped and the node is put into the FAIL state. Note that we still are able to access the node
// via the USB console and one day also via diagnostic LCS messages. The idea is to allow the correct 
// configuration  of the nodeMap, so that we can restart with a correct nodeMap. 
//
// ??? how do we deal wit PFAIL restarts ?
// ??? we could have also callbacks for the "restart" case ? or pass to init a flag...
//------------------------------------------------------------------------------------------------------------
uint8_t initRuntime( LcsConfigDesc *lcsConfig, CDC::CdcConfigDesc *cdcConfig ) {

    uint8_t rStat = ALL_OK;

    if ( rStat == ALL_OK )  rStat = initCdcLayer( cdcConfig );
    if ( rStat != ALL_OK )  CDC::fatalErrorMsg((char *) "Fatal: CDC Setup failed", 1, rStat );

    CDC::writeDio( cdcConfig -> READY_LED_PIN, false );
    CDC::writeDio( cdcConfig -> ACTIVE_LED_PIN, true );

    if ( rStat == ALL_OK )  rStat = initCanBus( cdcConfig );
    if ( rStat == ALL_OK )  rStat = initNvmChannels( cdcConfig );
    if ( rStat != ALL_OK )  CDC::fatalErrorMsg((char *) "Fatal: CAN bus or NVM Setup failed", 2, rStat );

    if ( rStat == ALL_OK )  rStat = setupNodeMap( lcsConfig );
    if ( rStat == ALL_OK )  rStat = setupPortMap( );
    if ( rStat == ALL_OK )  rStat = setupNodeDataMap( );
    if ( rStat == ALL_OK )  rStat = setupEventMap( );
    if ( rStat == ALL_OK )  rStat = setupUserMap( );
    if ( rStat == ALL_OK )  rStat = setupCallbackMap( );
    if ( rStat == ALL_OK )  rStat = setupTaskMap( );
    if ( rStat == ALL_OK )  rStat = setupPendingReqMap( );
    if ( rStat == ALL_OK )  rStat = setupDrvFuncMap( );
    if ( rStat == ALL_OK )  rStat = setupDrvMap( );
    if ( rStat == ALL_OK )  rStat = registerInternalTasks( );
    if ( rStat != ALL_OK )  CDC::fatalErrorMsg((char *) "Node setup Setup failed", 3, rStat );

    if ( rStat == ALL_OK )  rStat = detectExtensionBoards( );
    if ( rStat == ALL_OK )  rStat = setupExtensionBoards( );
    if ( rStat != ALL_OK )  printf( "Extension boards setup Setup failed, stat: %d\n", rStat );

    if ( rStat == ALL_OK )  nodeMap.nodeState = NS_INIT;
    else                    nodeMap.nodeState = NS_FAIL;

    if ( rStat == ALL_OK )  {
        
        CDC::writeDio( cdcConfig -> READY_LED_PIN, true );
        CDC::writeDio( cdcConfig -> ACTIVE_LED_PIN, false );
    }

    if ( debugMask & ( DBG_CONFIG && DBG_SETUP )) printf( "init LCS runtime, status: %d \n", rStat );
    return ( rStat );
}

//-----------------------------------------------------------------------------------------------------------
// "startRuntime" is the main routine of the node activity processing. All it does is to call the node
// state machine.
//
//------------------------------------------------------------------------------------------------------------
void startRuntime( ) {

     if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "Start LCS runtime\n");

     handleNodeState( );
}

}; // namespace LCS