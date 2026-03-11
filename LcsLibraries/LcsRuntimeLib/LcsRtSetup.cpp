//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime library setup.
//
//----------------------------------------------------------------------------------------
// The file implements a part of the LcsRuntimeLib that deals with the setup and
// start sequence of a node. There is a lot to do. First, we need to initialize 
// the CDC layer, our lower layer foundation. Next the NVM header is located and
// checked for validity. If valid, the nodeMap is read. It contains all the data
// for setting up the entire node. If this steps fails, we either need to configure
// the nodeMap, or we have a data error and the node is not usable and manual 
// intervention is required.
//
// With a correct node map in place, the memory structures for the node, the ports,
// events, callbacks and periodic tasks are created. The node is basically ready 
// to do work. For a node that has no extension boards connected, we are done.
//
// Next is the extension board setup. We try to locate all connected extension 
// boards and install the corresponding driver. A driver is just a procedure that
// knows how to talk to the particular extension board. A failure in this part of
// the sequence does not necessarily mean that the node cannot be used.
//
// Assuming all went fine, the runtime library is ready to accept registration 
// calls and is able to execute a few other library calls. Once all this work is
// done, the last call of the node firmware would be to start the runtime, which 
// would as the very first thing invoke all registered initialization callbacks
// and the enter the processing loop. We will not return from that routine.
//
// An error in the setup sequence does not necessarily mean that the node is 
// unusable. For example, when the nodeMap is not valid, the setup routine will 
// report an error, but we can still call the runtime loop. The runtime loop will
// handle LCS messages and also provide the console IO, which in turn allows us
// manually correct the node data for a successful restart. In a similar way, 
// extension board errors can be be addressed.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime library setup.
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
//----------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"
#include "LcsDrvOccDetectLib.h"
#include "LcsDrvServoLib.h"




// ??? idea: we could add a printf-like function, so that a firmware does not have
// to deal with whether we have a console or not...





//----------------------------------------------------------------------------------------
// Runtime globals. This file contains all the global data structure declarations.
// They are declared in the LCS name space. All other files in the runtime library
// will declare them as "extern" if needed.
//
// There is also the debug mask. The idea is to have a debug mask where each major
// part of the library has a bit. There could also be bits reserved for the firmware.
// Then we have control items to set these bits. Wherever debugging or tracing is
// needed, the bit mask will be used to determine whether to print debugging data 
// or not. From a performance perspective, the test will take just a few of 
// instructions. In other words we do not take out debugging code when going into
// production. Never liked this approach of conditional debug code via "ifdefs".
//
//----------------------------------------------------------------------------------------
namespace LCS {

    uint16_t            debugMask       = 0;
    uint16_t            runtimeOptions  = NPO_NIL;
    uint16_t            firmwareOptions = NPO_NIL;
    
    CdcResourceDescMap  dMap;

    LcsMsgBusCAN        *msgBus;
    LcsBoardDesc        boardDesc;
    LcsNodeMap          nodeMap;
    LcsPortMap          portMap;
    LcsNodeData         nodeData;
    LcsEventMap         eventMap;
    LcsTaskMap          taskMap;
    LcsDrvFuncMap       drvFuncMap;
}

//----------------------------------------------------------------------------------------
// Runtime library routines declared in other files we need here.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint8_t configNvm(uint8_t rIdNvm, uint32_t nvmSize );

    extern uint8_t rtNvmPutWord(uint32_t ofs, uint16_t word);
    extern uint8_t rtNvmPutBytes(uint32_t ofs, uint8_t *buf, uint32_t len);
    extern uint8_t rtNvmGetBytes(uint32_t ofs, uint8_t *buf, uint32_t len);

    extern uint8_t extNvmGetBytes(uint8_t boardId,
                                  uint32_t ofs,
                                  uint8_t *buf,
                                  uint32_t len);

    extern uint8_t extNvmPutBytes(uint8_t boardId,
                                  uint32_t ofs,
                                  uint8_t *buf,
                                  uint32_t len);

    extern uint8_t setEventMask(uint16_t eventId, uint16_t eventMask);
    extern uint8_t syncEventMapToMem( );
}

//----------------------------------------------------------------------------------------
// The LcsCoreLibConfig implementation file local declarations and routines. They are
// not visible to the other files.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// "setupDebugEnabled" and "retStat" are the debug support routines. We can easily 
// check whether debug is enabled at all. The return status routine will print 
// out a return status message when debugging is enabled. The macro "RET_STAT" 
// is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool setupDebugEnabled( ) {

    return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_SETUP )); 
}

inline void enterFunc( char *name ) {

    if ( setupDebugEnabled( )) printf( "--> %s\n", name );
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( setupDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "<-- %s: OK\n", name );
        else                    printf( "<-- %s: %d\n", name, errId );
    }

    return ( errId );
}

#define ENTER_FUNC() enterFunc((char *) __func__)
#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// "setupDefaultNodeHeader" initializes the NVM header map. We fill in the data 
// for the main board from the board descriptor map. The extension entries are 
// just cleared. The new NVM Node Map header is stored to NVM.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultNodeHeader( ) {

    ENTER_FUNC();

    LcsBoardDesc tmp;

    tmp.boardMword     = NVM_MWORD_NODE_HEADER;
    tmp.boardInfo      = dMap.boardInfo;
    tmp.boardVersion   = dMap.boardVersion;
    tmp.boardCtrlInfo  = dMap.boardCtrlInfo;

    return ( RET_STAT( rtNvmPutBytes( NVM_MAP_STORAGE_START,
                                      (uint8_t *)&tmp,
                                      sizeof( LcsBoardDesc ))));
}

//----------------------------------------------------------------------------------------
// "setupDefaultNodeMap" builds the node map structure. We allocate the Node UID.
// The newly created default node map is stored to its place in the NVM. We also
// return the new map.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultNodeMap( ) {

    ENTER_FUNC( );

    nodeMap.magicWord               = NVM_MWORD_NODE_MAP;
    nodeMap.nvmOfs                  = NVM_NODE_MAP_OFS;
    nodeMap.nvmSize                 = sizeof(LcsNodeMap);
    nodeMap.rtLibSwVersion          = LCS_RT_LIB_VERSION;
    nodeMap.rtLibSwPatchLevel       = LCS_RT_LIB_PATCH_LEVEL;

    nodeMap.nodeState               = NS_NIL;
    nodeMap.nodeId                  = NIL_NODE_ID;
    nodeMap.nodeUID                 = createUid( );
    nodeMap.nodeRestartCnt          = 0;
    nodeMap.nodeSystemTime          = 0;

    nodeMap.initCallback            = nullptr;
    nodeMap.initCallBackUdata       = nullptr;

    nodeMap.pfailCallback           = nullptr;
    nodeMap.pfailCallBackUdata      = nullptr;

    nodeMap.lcsMsgCallback          = nullptr;
    nodeMap.lcsMsgCallBackUdata     = nullptr;

    nodeMap.dccMsgCallback          = nullptr;
    nodeMap.dccMsgCallBackUdata     = nullptr;

    nodeMap.cmdLineCallback         = nullptr;
    nodeMap.cmdLineCallBackUdata    = nullptr;

    return ( RET_STAT( rtNvmPutBytes( NVM_NODE_MAP_OFS,
                                      (uint8_t *)&nodeMap,
                                      NVM_NODE_MAP_SIZE )));
}

//----------------------------------------------------------------------------------------
// "buildMemDefaultNodeData" builds the node data blocks and initializes the NVM
// portion for it. We also return the newly created node data map.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultNodeData( ) {

    ENTER_FUNC( );

    nodeData.magicWord = NVM_MWORD_NODE_DATA_MAP;
    nodeData.nvmOfs    = NVM_NODE_DATA_OFS;
    nodeData.nvmSize   = NVM_NODE_DATA_SIZE;

    memset( nodeData.map,
            0,
            MAX_NODE_DATA_BLOCKS * MAX_ATTR_MAP_ENTRIES * sizeof(uint16_t));

    return ( RET_STAT( rtNvmPutBytes( NVM_NODE_DATA_OFS,
                                     (uint8_t *)&nodeData,
                                     NVM_NODE_DATA_SIZE )));
}

//----------------------------------------------------------------------------------------
// "setupDefaultEventMap" initializes the event map and write it to NVM. We also
// return the newly created event map.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultEventMap( ) {

    ENTER_FUNC();

    eventMap.magicWord = NVM_MWORD_EVENT_MAP;
    eventMap.nvmOfs    = NVM_EVENT_MAP_OFS;
    eventMap.nvmSize   = NVM_EVENT_MAP_SIZE;
    eventMap.mapHwm    = 0;

    for ( uint16_t i = 0; i < MAX_EVENT_MAP_ENTRIES; i++ ) {

        eventMap.map[i].eventId = 0;
        eventMap.map[i].eventMask = 0;
    }

    return ( RET_STAT( rtNvmPutBytes( NVM_EVENT_MAP_OFS,
                                      (uint8_t *)&eventMap,
                                      NVM_EVENT_MAP_SIZE)));
}

//----------------------------------------------------------------------------------------
// "buildNvmRuntimeStructure" initializes a runtime NVM with default data. It is 
// used for a new board or when we detect a corrupt NVM image. We initialize the 
// MEM structures and just write them to their spot in NVM. After successful 
// completion, we will have a valid runtime map on NVM and MEM.
//
//----------------------------------------------------------------------------------------
uint8_t buildNvmRuntimeStructure( ) {

    ENTER_FUNC( );

    uint8_t rStat = LCS_OK;
    if ( rStat == LCS_OK ) rStat = setupDefaultNodeHeader( );
    if ( rStat == LCS_OK ) rStat = setupDefaultNodeMap( );
    if ( rStat == LCS_OK ) rStat = setupDefaultNodeData( );
    if ( rStat == LCS_OK ) rStat = setupDefaultEventMap( );
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// A little helper to print a board descriptor structure in HEX for debugging 
// purposes.
//
//----------------------------------------------------------------------------------------
void printBoardDesc( LcsBoardDesc *head ) {

    uint16_t *ptr  = (uint16_t *) head;
    size_t   words = sizeof(LcsBoardDesc) / 2;

    printf( "NVM Header ( %zu words ):\n", words );

    for ( size_t j = 0; j < words; j++ ) {
    
        printf( "0x%04x ", ptr[j] );
        if ((( j + 1 ) % 8 ) == 0 ) printf( "\n" );
    }

    if (( words % 8 ) != 0 ) printf( "\n" );
}

}; // namespace

//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// When the node is powered on, the very first thing to do is to setup the CDC
// library and configure the hardware resources. Note that this may have been 
// done before, when for example the firmware programmer wants to use the CDC 
// resources before calling any library setup code.
//
// There are two basic modes. The first is when we have a console connected. We
// will prompt and wait for a start command. There are several options for starting
// a node. The easiest is "R" which just starts the node. The "D" command will 
// start with debugging enabled. We will set the setup debug flags to check any 
// issues during the startup phase. Finally, there is there "F" command, which 
// will format the NVM runtime area. However, all that is happening in this 
// routine is to set these options to be executed at the right place in the setup
// sequence.
//
// The second mode is when there no console connected. In this case, Debug is 
// disabled and we just setup the node. This mode should be the normal case for 
// all the nodes in a layout.
//
// Perhaps one day, this routine could be enhanced to allow commands to pile up 
// the start options followed by the final start command to get the show going. 
// Especially the debug mask would be a candidate.
//
//----------------------------------------------------------------------------------------
uint8_t initCdcLayer( ) {

    const uint32_t CONSOLE_TIMEOUT = 1024 * 1024 * 4;

    uint8_t rStat = cdcInit( &dMap, runtimeOptions, debugMask );

    rStat = configureDio( CDC_RN_ACTIVITY_LED );

    if ( usbIsConnected( )) {

        printf( "Type '?' for help\n" );

        while ( true ) {

            printf( "=>" );

            char ch = usbIoGetChar( 0, CONSOLE_TIMEOUT );

            if ((ch == 'R') || (ch == 'r')) {

                printf( "Starting - normal mode\n" );

                debugMask &= ~LCS_DBG_ENABLE;
                return ( RET_STAT( LCS_OK ));
            } 
            else if (( ch == 'D' ) || ( ch == 'd' )) {

                printf( "Starting - debug mode\n" );

                debugMask |= LCS_DBG_ENABLE | LCS_DBG_SETUP;
                return ( RET_STAT( LCS_OK ));
            } 
            else if (( ch == 'F' ) || ( ch == 'f' )) {

                printf( "Starting - format mode\n" );

                debugMask    |= LCS_DBG_ENABLE | LCS_DBG_SETUP;
                runtimeOptions |= NPO_FORMAT_RUNTIME;
                return ( RET_STAT( LCS_OK ));
            } 
            else if ( ch == '?' ) {

                printf( "Setup options:\n" );
                printf( "r, R -> start the node with debug initially disabled\n" );
                printf( "d, D -> start the node with \"setup\" debug options enabled\n" );
                printf( "f, F -> start the node with a newly formatted runtime map\n" );
           
            } else printf( "\n" );
        }
    }
    else {

        debugMask = 0;
        return ( RET_STAT( LCS_OK ));
    }
}

//----------------------------------------------------------------------------------------
// The first thing is to setup our I2C channels. The NVM channel allows us access
// to the runtime NVM, the other one to external boards.
//
//----------------------------------------------------------------------------------------
uint8_t initI2cChannels( ) {

    ENTER_FUNC( );

    uint8_t rStat = LCS_OK;
    if ( rStat == LCS_OK) rStat = configureI2C( CDC_RN_NVM );
    if ( rStat == LCS_OK) rStat = configureI2C( CDC_RN_EXT_NVM );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// When the I2C channel is in place, we can set up the runtime lib NVM.
//
// ??? just do the runtime NVM ?
//----------------------------------------------------------------------------------------
uint8_t configNodeNvm( ) {

    ENTER_FUNC( );

    uint8_t rStat = configNvm( CDC_RN_NVM, NVM_MAIN_BOARD_DEF_SIZE );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// Next is the CAN bus setup. The message bus is the central communication 
// mechanism. If we can also get it up early we could use it not only for 
// configurations and operations but perhaps for remote troubleshooting.
//
//----------------------------------------------------------------------------------------
uint8_t initCanBus( ) {

    ENTER_FUNC( );

    uint8_t rStat = configureCanBus( CDC_RN_CAN_BUS );

    if ( rStat == LCS_OK ) {

        msgBus = new LcsMsgBusCAN( );

        rStat = msgBus->init( canGetRxPin( CDC_RN_CAN_BUS ),
                              canGetTxPin( CDC_RN_CAN_BUS ),
                              canGetBaudrate( CDC_RN_CAN_BUS ),
                              canGetTwoCores( CDC_RN_CAN_BUS ));

        if ( rStat != LCS_OK ) {

            if ( setupDebugEnabled( )) {

                printf( "Init Can Bus, CAN status: %d\n", rStat ); 
            }

            rStat = ERR_CAN_SETUP;
        }
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// Setup the watchdog timer. Nothing to do right now.
//
//----------------------------------------------------------------------------------------
uint8_t setupWatchdog( CdcResourceDescMap *map ) {

    ENTER_FUNC( );
    return ( RET_STAT( watchDogEnable( ! ( runtimeOptions & NPO_DISABLE_WATCHDOG ))));
}

//----------------------------------------------------------------------------------------
// Setup the power fail facility. There is a pin to which the power fail detection
// circuitry is connected. When power goes away, the falling edge on the pin will
// cause an interrupt and the power fail handler executes.
//
//----------------------------------------------------------------------------------------
uint8_t setupPfail( CdcResourceDescMap *map ) {

    ENTER_FUNC( );
    return ( RET_STAT( configureDio( CDC_RN_PFAIL )));
}

//----------------------------------------------------------------------------------------
// "checkMagicWords" is the routine that checks of the individual areas in the 
// NVM memory area are valid areas. Each individual map starts with a magic word, 
// and we expect them at the fixed location. If there is a mismatch, the NVM is 
// corrupted or the software has changed. In both cases we attempt to reformat 
// the NVM area.
//
//----------------------------------------------------------------------------------------
uint8_t checkMagicWords( ) {

    ENTER_FUNC( );

    uint8_t  rStat = LCS_OK;
    uint32_t mWord = 0;

    if ( rStat == LCS_OK ) {

        rStat = rtNvmGetBytes( NVM_HEADER_MAP_OFS, (uint8_t *) &mWord, sizeof( mWord ));
        if ( rStat == LCS_OK) {

            if ( mWord != NVM_MWORD_NODE_HEADER ) rStat = ERR_MWORD_NODE_HEADER;
        }
    }

    if ( rStat == LCS_OK ) {

        rStat = rtNvmGetBytes( NVM_NODE_MAP_OFS, (uint8_t *) &mWord, sizeof( mWord ));
        if ( rStat == LCS_OK ) {

            if ( mWord != NVM_MWORD_NODE_MAP ) rStat = ERR_MWORD_NODE_MAP;
        }
    }

    if ( rStat == LCS_OK ) {

        rStat = rtNvmGetBytes( NVM_NODE_DATA_OFS, (uint8_t *) &mWord, sizeof( mWord ));
        if ( rStat == LCS_OK ) {

            if ( mWord != NVM_MWORD_NODE_DATA_MAP ) rStat = ERR_MWORD_NODE_DATA;
        }
    }

    if ( rStat == LCS_OK) {

        rStat = rtNvmGetBytes( NVM_EVENT_MAP_OFS, (uint8_t *) &mWord, sizeof( mWord ));
        if ( rStat == LCS_OK ) {

            if ( mWord != NVM_MWORD_EVENT_MAP ) rStat = ERR_MWORD_EVENT_MAP;
        }
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "setupNodeNvmHeader" sets up the main controller header map entry. It is the 
// first routine after all the basic hardware settings is in place. If we detect
// an invalid NVM header or NVM formatting was requested, a default structure will 
// be created. Either way we return with a valid NVM structure for the node.
//
//----------------------------------------------------------------------------------------
uint8_t setupNodeNvmHeader(CdcResourceDescMap *map) {

    ENTER_FUNC( );

    uint8_t rStat = LCS_OK;

    if ( runtimeOptions & NPO_FORMAT_RUNTIME ) {

        rStat = buildNvmRuntimeStructure( );
        if ( rStat != LCS_OK) return ( RET_STAT( rStat ));
    }

    rStat = checkMagicWords( );
    if ( rStat != LCS_OK ) {

        rStat = buildNvmRuntimeStructure( );
        if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));
    }

    rStat = rtNvmGetBytes( NVM_HEADER_MAP_OFS,
                           (uint8_t *) &boardDesc,
                           sizeof( LcsBoardDesc ));
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if (( boardDesc.boardInfo != dMap.boardInfo ) &&
        ( boardDesc.boardCtrlInfo != dMap.boardCtrlInfo )) {
    
        }

    if ( boardDesc.boardVersion != dMap.boardVersion ) {

    }

    if ( setupDebugEnabled( )) printBoardDesc( &boardDesc );
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "setupNodeMap" sets up the nodeMap. It is the routine that is called after we
// read in the NVM headers. If the main controller NVM header was invalid or
// formatting was requested, a default structure was created. Either way we can
// rely on a valid map layout. Note that some items are stored in Port Map entry 0,
// which by definition is the port for the node itself. We will store these items
// in the port setup routine. To be sure, we explicitly clear some nodeMap fields,
// such as the callback labels.
//
//----------------------------------------------------------------------------------------
uint8_t setupNodeMap( ) {

    ENTER_FUNC( );

    uint8_t rStat = rtNvmGetBytes( NVM_NODE_MAP_OFS,
                                   (uint8_t *)&nodeMap,
                                   NVM_NODE_MAP_SIZE);

    if ( rStat == LCS_OK ) {

        nodeMap.initCallback            = nullptr;
        nodeMap.cmdLineCallBackUdata    = nullptr;

        nodeMap.pfailCallback           = nullptr;
        nodeMap.pfailCallBackUdata      = nullptr;

        nodeMap.lcsMsgCallback          = nullptr;
        nodeMap.lcsMsgCallBackUdata     = nullptr;

        nodeMap.dccMsgCallback          = nullptr;
        nodeMap.dccMsgCallBackUdata     = nullptr;

        nodeMap.cmdLineCallback         = nullptr;
        nodeMap.cmdLineCallBackUdata    = nullptr;
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "setupPortMap" will initialize the portMap. A node can have up to 16 ports.
//
//----------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

    ENTER_FUNC( );

    portMap.mapHwm = 0;
    for ( uint16_t i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        LcsPortMapEntry pEntry;
        portMap.map[i] = pEntry;
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "setupNodeDataMap" will read the node data blocks.
//
//----------------------------------------------------------------------------------------
uint8_t setupNodeDataMap( ) {

    ENTER_FUNC( );
    return ( RET_STAT( rtNvmGetBytes( NVM_NODE_DATA_OFS,
                                   (uint8_t *)&nodeData,
                                   sizeof( nodeData ))));
}

//----------------------------------------------------------------------------------------
// The event map stores all events this node is interested to process. The map is 
// a sorted map of event Id and port mask pairs. There is a high water mark, so 
// that we only read up to the last used entry in the map. Just like other data 
// structures we could just read in all entries. However, this is a large map. It 
// is better to just read up to the HWM, if the HWM is valid. If this is not the 
// case, we have to assume that there are issues with the event map. In this case
// we will read the entire  map entry by entry, add used entries, i.e. entries 
// with a non-NIL event ID to the memory map. After reading all entries, the newly
// created event map is written back to the NVM. We now have a valid map again.
//
//----------------------------------------------------------------------------------------
uint8_t setupEventMap( ) {

    ENTER_FUNC( );
    return ( RET_STAT( syncEventMapToMem( )));
}

//----------------------------------------------------------------------------------------
// The user map is the additional NVM storage that the chip set offers beyond the
// area allocated for the runtime data. The size is depending on the actual NVM
// chip used and the configured user map size.
//
//----------------------------------------------------------------------------------------
uint8_t setupUserMap( ) {

    ENTER_FUNC( );

    // ??? figure out how much memory there is, load it.
    // ??? we also need to allocate memory for this map first.

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "setupTaskMap" initializes the task map. A user can register routines that are
// executed on a periodic basis.
//
//----------------------------------------------------------------------------------------
uint8_t setupTaskMap( ) {

    ENTER_FUNC( );

    taskMap.mapHwm = 0;
    for ( int i = 0; i < MAX_TASK_MAP_ENTRIES; i++ ) {

        LcsPTaskMapEntry tmp;
        taskMap.map[i] = tmp;
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// A port offers a set of up to 8 channels. For each channel we form the I2C 
// addresses from portId and channelId and try to read from that address. If
// there is a response, try to read a header to find out what is connected at 
// that address. The I2C address is computed from ( portId * 16 + chanId ) + 8.
// The funny "plus 8" is due to the fact that the I2C bus reserves the first 
// 8 addresses for itself.  
//
// If we got a valid header, the port entry records the I/O element type, which 
// in turn defines the driver function to use. All channels on a given port must
// have the same type.
//
//----------------------------------------------------------------------------------------
uint8_t discoverChannels( ) {

    ENTER_FUNC( );
   
    for ( int i = 1; i < MAX_PORT_ID; i++ ) {

        for ( int j = 0; j < MAX_CHAN_ID; j++ ) {

            uint8_t i2cAdr = i * MAX_PORT_MAP_ENTRIES + j;

            if ( setupDebugEnabled( )) {

                printf( "Testing I2C adr: 0x%2x\n", i2cAdr );
            }

            // ??? try to read ...

            uint8_t tmp;
            uint8_t rStat = i2cRead( CDC_RN_EXT_NVM, i2cAdr, (uint8_t *) &tmp, 1 );

            if ( rStat != LCS_OK ) continue;

            if ( setupDebugEnabled( )) {

                printf( "Found a device\n", rStat );
            }

            
        
            // ??? if OK, read the entire header.

            // ??? check if the type matches the port type of there is already 
            // a type set for the port. If no match it is an error, and the port
            // is disabled in error.

            // ??? if match, remember the channelId in a bit mask. Not all 
            // channel Ids need to be in use...

            // ??? to do ...

            // ??? update the portMap entry flags.

            // portMap.map[0].flags        |= NPF_EXT_BOARD_PRESENT;
            // portMap.map[i].flags        |= NPF_EXT_BOARD_PRESENT;
            // portMap.map[i].flags        |= NPF_EXT_BOARD_VALID;
            // portMap.map[i].reqCallback  = nullptr;
        }
    }

    return( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "setupDrvFuncMap" initializes the driver function label map. This table is used
// when we need to find the driver label for an channel type.
//
//----------------------------------------------------------------------------------------
uint8_t setupDrvFuncMap( ) {

    ENTER_FUNC( );
    
    drvFuncMap.mapHwm = 0;
    for ( int i = 0; i < MAX_DRV_TYPE_MAP_ENTRIES; i++ ) {

        LcsDrvFuncEntry e;
        drvFuncMap.map[i] = e;
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// The runtime library will one day perhaps a set of internal functions to execute
// periodically. They should be added here. Right now, this routine will do nothing.
//
//----------------------------------------------------------------------------------------
uint8_t registerInternalTasks( ) {

    ENTER_FUNC( );
    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// Driver function registration. There is a simple table which maintains board
// types and the driver REQ function for them. For already registered types, we 
// just overwrite the function signature. Drivers are never deallocated from the
// table, the next free entry is the HWM.
//
//----------------------------------------------------------------------------------------
uint8_t registerDrvFunc( LcsReqCallback drvReqFunction, uint16_t drvType, void *uData ) {

    if ( setupDebugEnabled( )) {

        printf( "--> registerDrvFunc, type: %d\n", drvType );
    }

    for ( int i = 0; i < MAX_DRV_TYPE_MAP_ENTRIES; i++ ) {

        if ( drvFuncMap.map[i].drvType == drvType ) {

            if ( setupDebugEnabled( )) {

                printf( "registerDrvFunc, overwrite: %d\n", i );
            }

            drvFuncMap.map[ i ].drvFunc = drvReqFunction;
            drvFuncMap.map[ i ].uData   = uData;
            return ( RET_STAT( LCS_OK ));
        }
    }

    if ( drvFuncMap.mapHwm < MAX_DRV_TYPE_MAP_ENTRIES ) {

        if ( setupDebugEnabled( )) {

            printf( "registerDrvFunc, allocate: %d\n", drvFuncMap.mapHwm );
        }

        drvFuncMap.map[ drvFuncMap.mapHwm ].drvType = drvType;
        drvFuncMap.map[ drvFuncMap.mapHwm ].drvFunc = drvReqFunction;
        drvFuncMap.mapHwm++;
        return ( RET_STAT( LCS_OK ));
    }

    return ( RET_STAT( ERR_DRV_FUNC_MAP_FULL ));
}

//----------------------------------------------------------------------------------------
// During the initialization sequence INIT -> register -> START, the driver function
// labels for a driver type have been registered. The INIT portion detected any
// I2C address used and recorded the required driver type in the portMap entry. 
// Before START, required driver types have been registered. On START all we do 
// is to store the driver signature on the portMap entry and mark the port configured.
//
// ??? to do ...
//----------------------------------------------------------------------------------------
uint8_t setupDriverFunctions( ) {

    ENTER_FUNC( );

    for ( int i = 1; i < MAX_PORT_MAP_ENTRIES; i++ )  {

        LcsPortMapEntry *pPtr = &portMap.map[i];

        if (( pPtr->flags & NPF_EXT_BOARD_PRESENT ) &&
            ( pPtr->flags & NPF_EXT_BOARD_VALID   )) {

            for ( int j = 0; j < MAX_DRV_TYPE_MAP_ENTRIES; j++ ) {

                if ( boardDesc.boardInfo == drvFuncMap.map[j].drvType ) {

                    if ( setupDebugEnabled( )) {

                        printf( "setupDriverFunctions, board: %d,"
                                " drvType entry: %d\n",
                                i, j);
                    }

                    pPtr->reqCallback = drvFuncMap.map[j].drvFunc;
                    pPtr->flags       |= NPF_EXT_BOARD_READY;
                }
            }
        }
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "powerFailHandler" is the routine called when the hardware detects an imminent
// loss of power. Our chance to save crucial data to NVM. Finally, the optionally
// registered firmware power fail callback is called. The node state becomes 
// "PFAIL". Upon restart, we check this state and know that we came back after a
// power fail.
//
//----------------------------------------------------------------------------------------
uint8_t powerFailHandler( ) {

    ENTER_FUNC( );

    uint8_t rStat = LCS_OK;

    nodeMap.nodeState = NS_PFAIL;
    rStat = rtNvmPutWord( NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeState ),
                          NS_PFAIL);

    if ( nodeMap.pfailCallback != nullptr ) 
        nodeMap.pfailCallback( nodeMap.nodeId, nodeMap.pfailCallBackUdata );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "initRuntime" is the routine that takes a controller board and initializes the
// whole show. It is the very first thing to call in a node firmware program. There
// is a lot to do. This routine will invoke the various initializers, one at a time.
//
// The first three calls are the basic setup of the CDC layer, the I2C, NVM and 
// CanBus channel. The CDC layer setup also checks for a console presence and if 
// so, allows for different start modes. However, if any of them fails, we have a
// fatal error and stop. If we have a basic hardware setup, let's check whether we
// are starting from a watchdog timer or power fail event.
//
// The remainder of the calls will setup the individual portions of the LCS runtime.
// The overall logic of the startup code below is that if there is a fault, the 
// follow on steps are simply skipped and the node is put into the FAIL state. 
//
// Note that we still are able to access the node via the USB console and one day
// also via diagnostic LCS messages. The idea is to allow problem resolution and
// correct configuration of the nodeMap, so that we can hopefully restart with a
// correct nodeMap.
//
//----------------------------------------------------------------------------------------
uint8_t initRuntime( CdcResourceDescMap  *descMap,
                     uint16_t            options,
                     uint16_t            dbgMask) {

    uint8_t rStat = LCS_OK;

    dMap            = *descMap;
    runtimeOptions  = options;
    debugMask       = dbgMask;

    rStat = initCdcLayer( );
    if ( rStat != LCS_OK ) {

        fatalError( 1, (char *) "Fatal: CDC Layer Setup failed", rStat );
    }
  
    rStat = initI2cChannels( );
    if ( rStat != LCS_OK ) {

        fatalError( 2, (char *) "Fatal: NVM channel configuration failed", rStat );
    }

    rStat = initCanBus( );
    if ( rStat != LCS_OK ) {

        fatalError( 3, (char *) "Fatal: CAN bus Configuration failed", rStat );
    }

    if ( rStat == LCS_OK )  rStat = configNodeNvm( );
    if ( rStat == LCS_OK )  rStat = setupWatchdog( &dMap );
    if ( rStat == LCS_OK )  rStat = setupPfail( &dMap );
    if ( rStat == LCS_OK )  rStat = setupNodeNvmHeader( &dMap );
    if ( rStat == LCS_OK )  rStat = setupNodeMap( );
    if ( rStat == LCS_OK )  rStat = setupPortMap( );
    if ( rStat == LCS_OK )  rStat = discoverChannels( );
    if ( rStat == LCS_OK )  rStat = setupNodeDataMap( );
    if ( rStat == LCS_OK )  rStat = setupEventMap( );
    if ( rStat == LCS_OK )  rStat = setupUserMap( );
    if ( rStat == LCS_OK )  rStat = setupTaskMap( );
    if ( rStat == LCS_OK )  rStat = setupDrvFuncMap( );
    if ( rStat == LCS_OK )  rStat = registerInternalTasks( );

    if ( rStat == LCS_OK ) {

        if ( nodeMap.nodeState == NS_PFAIL ) {

            // ??? we came back from a PFAIL ?
        }

        nodeMap.nodeState = NS_INIT;
    }
    else nodeMap.nodeState = NS_FAIL;

    writeDio( CDC_RN_ACTIVITY_LED, ( rStat == LCS_OK ));
    return ( RET_STAT( rStat ));
}

}; // namespace LCS