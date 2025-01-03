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

    uint16_t                debugMask    = 0;
    uint16_t                startOptions = 0;

    LcsMsgBusCAN            *msgBus;
    LcsNvmHeaderMap         nvmHeaderMap;
    LcsCdcMap               cdcMap;
    LcsNodeData             nodeData;
    LcsNodeMap              nodeMap;
    LcsPortMap              portMap;
    LcsEventMap             eventMap;
    LcsPendingReqMap        pendingReqMap;
    LcsTaskMap              taskMap;
    LcsDrvFuncMap           drvFuncMap;
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
// "buildDefaultNodeMap" build a header map with the default values from the declaration structure. The 
// default header map is used for initializing the memory runtime node map for formatting a new or corrupted
// runtime NVM.
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultHeaderMap( LcsNvmHeaderMap *map ) {

    LcsNvmHeaderMap tmp;

    *map = tmp;
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultCdcMap" builds ...
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultCdcMap( LcsCdcMap *map ) {

    LcsCdcMap tmp;

    *map = tmp;
}

//------------------------------------------------------------------------------------------------------------
// "buildDefaultNodeMap" build a nodeMap with the default values from the declaration structure. The default
// nodeMap is used for initializing the memory runtime node map for formatting a new or corrupted runtime NVM.
//
//------------------------------------------------------------------------------------------------------------
void buildDefaultNodeMap( LcsNodeMap *nMap ) {

    LcsNodeMap tmp;

    snprintf( tmp.name, MAX_NODE_NAME_SIZE, "Node" );
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

        snprintf( pEntry.name, MAX_PORT_NAME_SIZE, "Port-%d", i + 1 );
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
// ??? CDC map business ???
//------------------------------------------------------------------------------------------------------------
uint8_t buildNvmRuntimeStructures( ) {

    uint8_t rStat;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "buildNvmRuntimeStructures\n" );

    buildDefaultHeaderMap( &nvmHeaderMap );
    buildDefaultCdcMap( &cdcMap );
    buildDefaultNodeMap( &nodeMap );
    buildDefaultPortMap( &portMap );
    buildDefaultEventMap( &eventMap );
    buildDefaultNodeData( &nodeData );

    rStat = rtNvmPutBytes( nodeMap.nvmHeaderMapOfs, (uint8_t *) &nvmHeaderMap.map[ 0 ], sizeof( LcsNvmHeader));      
    if ( rStat == ALL_OK ) rStat = rtNvmPutBytes( nodeMap.nvmNodeMapOfs, (uint8_t *) &nodeMap, sizeof( LcsNodeMap ));
    if ( rStat == ALL_OK ) rStat = rtNvmPutBytes( nodeMap.nvmPortMapOfs, (uint8_t *) &portMap, sizeof( LcsPortMap ));
    if ( rStat == ALL_OK ) rStat = rtNvmPutBytes( nodeMap.nvmEventMapOfs, (uint8_t *) &eventMap, sizeof( LcsEventMap ));
    if ( rStat == ALL_OK ) rStat = rtNvmPutBytes( nodeMap.nvmNodeDataOfs, (uint8_t *) &nodeData, sizeof( LcsNodeData ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "buildNvmRuntimeStructures, stat: %d\n", rStat  );

    return( rStat );
}

#if 0
//------------------------------------------------------------------------------------------------------------
// "buildNvmRuntimeStructures" initializes a new or corrupt runtime NVM with default data. 
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
#endif


//-----------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
void printNvmHeader( LcsNvmHeader *head ) {

    uint16_t *ptr = (uint16_t *) head;

    printf( "NVM Head: " );
    for ( int j = 0; j < sizeof( LcsNvmHeader ) / 2 ; j++ ) printf( "0x%x ", ptr[ j ] );
    printf( "\n" );     
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

                debugMask       = DBG_CONFIG | DBG_SETUP | DBG_EVENTS;
                startOptions    = NOPT_NIL;
                return( ALL_OK );
            }
            else if (( ch == 'F' ) || ( ch == 'f' )) {

                printf( "Starting - format mode\n" );

                debugMask       &= ~ DBG_CONFIG;

                debugMask       = DBG_CONFIG | DBG_SETUP | DBG_NVM_ACCESS;
                
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
// ??? should we assume a "architectural" setting of the IC2 channels and not rely on CDC map ?
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

        rStat = CDC::configureI2C( ci -> EXT_I2C_SCL_PIN , ci -> EXT_I2C_SDA_PIN, 50 * 1000 );
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
// ??? should we assume a "architectural" setting of the CAN channel and not rely on CDC map ?
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
// "setupNodeNvmHeader" sets up the main controller header map entry. It is the first routine after all the 
// basic hardware settings is in place. If the header is invalid or formatting was requested, a default 
// structure will be created. Either way we return with a valid NVM structure for the node. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeNvmHeader( LcsConfigDesc *cfg ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupNodeNvmHeader\n" );

    if ( startOptions & NOPT_FORMAT_RUNTIME ) {

        rStat = buildNvmRuntimeStructures( );
    }

    LcsNvmHeader *hPtr = &nvmHeaderMap.map[ 0 ];

    rStat = rtNvmGetBytes( nodeMap.nvmNodeMapOfs, (uint8_t *) hPtr, sizeof( LcsNvmHeader ));
    if ( rStat != ALL_OK ) return( rStat );

    if (( hPtr -> magicWord1 != NVM_MWORD_1 ) || 
        ( hPtr -> magicWord2 != NVM_MWORD_2 )) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

            printNvmHeader( hPtr);
            printf( "setupHeaderMap: invalid header, re-format\n" );
        }

        rStat = buildNvmRuntimeStructures( );
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printNvmHeader( hPtr);
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupNodeNvmHeader, status: %d\n", rStat );
    
    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// With the NVM channels in place and the main controller NVM header valid, it is time read in any potentially 
// connected extension board NVM header. These header will also be stored in the NVM header map. Entry zero is 
// always the main controller board NVM header, the optional extension board NVM headers are stored in 
// entry 1 to 4. If the read fails, there is no board at that location and we set the magic words to zero to 
// record this fact.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupExtNvmHeaders( LcsConfigDesc *cfg ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupExtNvmHeaders\n" );

    uint8_t rStat = ALL_OK;

    for ( int i = 1; i <= MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

            printf( "setupExtNvmHeaders, boardId: %d\n", i ); 
        }

        LcsNvmHeader *hPtr = &nvmHeaderMap.map[ i ]; 

        rStat = extNvmGetBytes( i, 0, (uint8_t *) hPtr, sizeof( LcsNvmHeader ));
        if ( rStat == ALL_OK ) {

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                printNvmHeader( hPtr );
            }

            if (( hPtr -> magicWord1 == NVM_MWORD_1 ) && 
                ( hPtr -> magicWord2 == NVM_MWORD_2 )) {

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                    printf( "setupExtNvmHeaders, boardId: %d -> valid\n", i ); 
                }
            }
            else  {

                hPtr -> magicWord1 = 0;
                hPtr -> magicWord2 = 0;

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                    printf( "setupExtNvmHeaders, boardId: %d -> inValid\n", i ); 
                }
            }
        } 
        else {
            
            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP ))
                printf( "setupExtNvmHeaders, boardId: %d, rStat: %d\n", i, rStat ); 
        }          
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "setupExtNvmHeaders, status: %d\n", ALL_OK );

    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "setupCdcMap" will read the CDC descriptor from the NVM.
//
// ??? we should read it from the NVM, which implies that we need a way to handle vanilla boards...
// ??? we should also have a file with CDC descriptors for all boards we currently have...
// ??? if we detect a board with the vanilla board type, we can set the correct board type and the 
// reboot...
//------------------------------------------------------------------------------------------------------------
uint8_t setupCdcMap( CDC::CdcConfigDesc *cdcConfig ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupCdcMap\n" );

    uint8_t rStat = ALL_OK;


    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "setupCdcMap, status: %d\n", rStat );
    }

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupNodeMap" sets up the nodeMap. It is the routine that is called after we read in the NVM headers. 
// If the main controller NVM header was invalid or formatting was requested, a default structure was created. 
// Either way we can rely on a default map layout. We fetch the node map and validate the size of each map of
// NVM. This will catch the case that the library data structures have changed and the NVM data is therefore 
// invalid.
//
// ??? load CDC map if we only have a board type ?
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeMap( LcsConfigDesc *cfg ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupNodeMap\n" );

    rStat = rtNvmGetBytes( nodeMap.nvmNodeMapOfs, (uint8_t *) &nodeMap, sizeof( LcsNodeMap ));
    if ( rStat != ALL_OK ) return( rStat );

    nodeMap.nodeOptions = cfg -> options;

    if (( nodeMap.nvmHeaderMapSize  != sizeof( LcsNvmHeader )) ||
        ( nodeMap.nvmNodeMapSize    != sizeof( LcsNodeMap   )) ||
        ( nodeMap.nvmCdcMapSize     != sizeof( LcsCdcMap    )) ||
        ( nodeMap.nvmPortMapSize    != sizeof( LcsPortMap   )) ||
        ( nodeMap.nvmNodeDataSize   != sizeof( LcsNodeData  )) ||
        ( nodeMap.nvmEventMapSize   != sizeof( LcsEventMap  ))) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

            printf( "setupNodeMap: invalid structure size(s), re-format\n" );
        }

        rStat = buildNvmRuntimeStructures( );
    }
 
    if (( nodeMap.portMapEntries    != MAX_PORT_MAP_ENTRIES     ) ||
        ( nodeMap.eventMapEntries   != MAX_EVENT_MAP_ENTRIES    )) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

            printf( "setupNodeMap: invalid map size(s), re-format\n" );
        }

        rStat = buildNvmRuntimeStructures( );
    }

    if (( nodeMap.portMapHwm    > MAX_PORT_MAP_ENTRIES     ) ||
        ( nodeMap.eventMapHwm   > MAX_EVENT_MAP_ENTRIES    )) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

            printf( "setupNodeMap: invalid map HWM size(s), re-format\n" );
        }

        // ??? a less drastic way to address the issue ?

        rStat = buildNvmRuntimeStructures( );
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "setupNodeMap, status: %d\n", rStat );
    }

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupPortMap" will read the port data the NVM port map data area into the memory counterpart. A node can
// have up to 15 ports. If there are extension boards connected, the first N ports refer to these boards.
// We consult the nvmHeaderMap for detected extension boards.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupPortMap\n" );

    uint8_t rStat = rtNvmGetBytes( nodeMap.nvmPortMapOfs, (uint8_t *) &portMap, sizeof( LcsPortMap ));

    for ( int i = 1; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

            printf( "Setup Port %d\n", i );
        }

        LcsNvmHeader *hPtr = &nvmHeaderMap.map[ i ];
        
        if (( hPtr -> magicWord1 == NVM_MWORD_1 ) && ( hPtr -> magicWord2 == NVM_MWORD_2 )) {

             if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                printf( "Valid Extension Board detected\n" );
             }

            nodeMap.nodeFlags |= NF_EXT_PRESENT;
            nodeMap.drvMapHwm ++;

            portMap.map[ i ].flags              |= PF_EXT_BOARD_PRESENT;
            portMap.map[ i ].flags              |= PF_EXT_BOARD_VALID;
            portMap.map[ i ].reqCallbackFunc     = nullptr;
        }
    }

    // ??? not sure where and when to set the driver label... yet

    #if 0
    for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        LcsDrvEntry *drvEntry = &drvMap.map[ i ]; 

        if (( drvEntry -> flags & BF_EXT_BOARD_PRESENT ) && ( drvEntry -> flags & BF_EXT_BOARD_VALID )) {

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, set func for board: %d\n", i );

            if ( drvEntry -> extBoard.head.boardType == drvType ) {

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, set func for driver type: %d\n", drvType );

                drvEntry -> drvFunc     = drvReqFunction;
                drvEntry -> flags       |= BF_EXT_BOARD_READY;
                drvEntry -> lastErr     = drvEntry -> drvFunc( i, ITEM_ID_RESET, nullptr, nullptr );

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
                printf( "registerDrvFunc, set func for driver type, status: %d\n", drvEntry -> lastErr );
            }
        }
     }
     #endif

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "setupPortMap, status: %d\n", rStat );
    }

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "setupNodeDataMap" will read the node data blocks.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeDataMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupNodeDataMap\n" );

    uint8_t rStat = rtNvmGetBytes( nodeMap.nvmNodeDataOfs, (uint8_t *) &nodeData.map, sizeof( nodeData.map ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "setupNodeDataMap, status: %d\n", rStat );
    }

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

            rStat = rtNvmGetBytes(  nodeMap.nvmEventMapOfs + i * sizeof( LcsEventMapEntry), 
                                    (uint8_t *) &eventMap.map[ i ], 
                                    sizeof( LcsEventMapEntry ));
            if ( rStat != ALL_OK ) break;
        }
    }
    else {

        LcsEventMapEntry e;
        for ( uint16_t i = 0; i < nodeMap.eventMapEntries; i++ ) eventMap.map[ i ] = e;

        nodeMap.eventMapHwm = 0;
        for ( uint16_t i = 0; i < nodeMap.eventMapEntries; i++ ) {

            LcsEventMapEntry eventEntry;

            rStat = rtNvmGetBytes(  nodeMap.nvmEventMapOfs + i * sizeof(LcsEventMapEntry), 
                                    (uint8_t *) &eventEntry, 
                                    sizeof(LcsEventMapEntry));

            if (( rStat == ALL_OK ) && ( eventEntry.eventId != NIL_EVENT_ID )) {

                addEvent( eventEntry.eventId, eventEntry.eventMask );
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

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "setupUserMap, status: %d\n", rStat );
    }

    return ( ALL_OK );
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
// ??? what if the pending requests are always linked to a port ?
// ??? in other words, should we allow that a node with out a portId sends a request or always on behalf
// of a port ?
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
// ??? we could pre-register all known drivers... a user could still register a new one and also overwrite
// a pre-registered driver with a new func label.
//------------------------------------------------------------------------------------------------------------
uint8_t setupDrvFuncMap( ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) printf( "setupDrvLabelMap\n" );

    uint8_t rStat = ALL_OK;

    nodeMap.drvFuncMapEntries = MAX_DRV_TYPES;
    nodeMap.drvFuncMapHwm     = MAX_DRV_TYPES;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {
            
        printf( "setupDrvLabelMap, status: %d\n", rStat );
    }

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

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "registerInternalTasks, status: %d\n" );
    }

    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// Driver function registration. There is a simple table which maintains extension boards types and the 
// driver function form them. If the type is already registered, we just overwrite the function signature.
// Otherwise we find a free entry and use it. The register driver call is only valid when the LCS runtime is 
// initialized. When there are ports that have an extension board associated and the driver type matches, the
// driver request function is set. Note that the driver reset request is issued during LCS start of the 
// runtime. The sequence is INIT -> register > START.
//
//------------------------------------------------------------------------------------------------------------
uint8_t registerDrvFunc(  uint16_t drvType, LcsReqCallback drvReqFunction ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "registerDrvFunc, type: %d, func: %p\n", drvType, drvReqFunction );
    }

    bool found = false;

    for ( int i = 0; i < MAX_DRV_TYPES; i++ ) {

        if ( drvFuncMap.map[ i ].drvType == drvType ) {

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                printf( "registerDrvFunc, overwrite: %d\n", i );
            }
            
            drvFuncMap.map[ i ].drvFunc = drvReqFunction;
            found = true;
            break;
        }
    }

    if ( ! found ) {

        for ( int i = 0; i < MAX_DRV_TYPES; i++ ) {

            if ( drvFuncMap.map[ i ].drvType == BT_NIL ) {

                if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                    printf( "registerDrvFunc, allocate: %d\n", i );
                }
                
                drvFuncMap.map[ i ].drvType = drvType;
                drvFuncMap.map[ i ].drvFunc = drvReqFunction;
                found = true;
                break;
            }
        }
    }

    if ( ! found ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                printf( "registerDrvFunc, table full\n" );
        }

        return( ERR_DRV_FUNC_MAP_FULL );
    }

    for ( int i = 0; i < MAX_EXT_BOARD_MAP_ENTRIES; i++ ) {

        LcsPortMapEntry *hPtr = &portMap.map[ i ]; 

        if (( hPtr -> flags & PF_EXT_BOARD_PRESENT ) && ( hPtr -> flags & PF_EXT_BOARD_VALID ) && 
            ( nvmHeaderMap.map[ i ].boardType == drvType )) {

            if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

                printf( "registerDrvFunc, set func for board: %d, drvType: %D\n", i, drvType );
            }

            hPtr -> reqCallbackFunc = drvReqFunction;
            hPtr -> flags       |= PF_EXT_BOARD_READY;
        }
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) {

        printf( "registerDrvFunc, ret: ALL_OK\n" );
    }

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
    rStat = rtNvmPutWord( nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeState ), NS_PFAIL );
    
    if ( nodeMap.pfailCallback != nullptr ) nodeMap.pfailCallback( nodeMap.nodeId );

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "powerFailHandler, status: %d\n", rStat );
    
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
// ??? ensure that this routine is idempotent.
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

    if ( rStat == ALL_OK )  rStat = setupNodeNvmHeader( lcsConfig );
    if ( rStat == ALL_OK )  rStat = setupExtNvmHeaders( lcsConfig );
    if ( rStat == ALL_OK )  rStat = setupNodeMap( lcsConfig );
    if ( rStat == ALL_OK )  rStat = setupCdcMap( cdcConfig );
    if ( rStat == ALL_OK )  rStat = setupPortMap( );
    if ( rStat == ALL_OK )  rStat = setupNodeDataMap( );
    if ( rStat == ALL_OK )  rStat = setupEventMap( );
    if ( rStat == ALL_OK )  rStat = setupUserMap( );
    if ( rStat == ALL_OK )  rStat = setupTaskMap( );
    if ( rStat == ALL_OK )  rStat = setupPendingReqMap( );
    if ( rStat == ALL_OK )  rStat = setupDrvFuncMap( );
    if ( rStat == ALL_OK )  rStat = registerInternalTasks( );
    if ( rStat != ALL_OK )  CDC::fatalErrorMsg((char *) "Node setup Setup failed", 3, rStat );

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