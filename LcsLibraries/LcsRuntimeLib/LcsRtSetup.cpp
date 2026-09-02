//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime library setup.
//
//----------------------------------------------------------------------------------------
// The file implements a part of the LcsRuntimeLib that deals with the setup and
// start sequence of a node. There is a lot to do. First, we need to initialize 
// the CDC layer, our lower layer foundation. Next, the I2C and CAN bus resources
// are initialized. What follows is the setup of the runtime major components.
// See the "initRuntime" routine comments for details.
//
// Assuming all went fine, the runtime library is ready to accept registration 
// calls and is able to execute a few other library calls. Once all this work is
// done, the last call of the node firmware would be to start the runtime, which 
// would as the very first thing invoke all registered initialization callbacks
// and the enter the processing loop. We will not return from that routine.
//
// An error in the setup sequence does not necessarily mean that the node is 
// unusable. For example, when some data the nodeMap is not valid, the setup 
// routine will report an error, but we can still call the runtime loop. The 
// runtime loop will handle LCS messages and also provide the console IO, which
// in turn allows us manually correct the node data for a successful restart.
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
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with 
// this program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

// ??? idea: we could add a printf-like function, so that a firmware does not have
// to deal with whether we have a console or not...


//----------------------------------------------------------------------------------------
// Runtime globals. This file contains all runtime data structure declarations.
// They are declared in the LCS name space. All other runtime files will declare
// them as "extern".
//
// There is also the debug mask. The idea is to have a debug flag where each 
// major part of the library runtime. We have control items to set these bits. 
//
//----------------------------------------------------------------------------------------
namespace LCS {

    uint16_t            debugMask       = 0;
    uint16_t            runtimeOptions  = NPO_NIL;
    uint16_t            firmwareOptions = NPO_NIL;
    
    CdcResourceDescMap  dMap;
    LcsMsgBusCAN        *msgBus;
    LcsNvmHeader        nvmHeader;
    LcsNodeMap          nodeMap;
    LcsPortMap          portMap;
    LcsEventMap         eventMap;
    LcsPortDataMap      portDataMap;
    LcsGlobalDataMap    globalDataMap;
    LcsTaskMap          taskMap;
}

//----------------------------------------------------------------------------------------
// Runtime library routines declared in other files we need here.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint8_t  rtNvmConfig( uint8_t rIdNvm, uint32_t nvmSize );
    extern uint8_t  rtNvmPutWord( uint32_t ofs, uint16_t word);
    extern uint8_t  rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
    extern uint8_t  rtNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
    extern uint32_t rtNvmGetSize( );

    extern uint8_t  loadEventMap( );
}

//----------------------------------------------------------------------------------------
// The file local declarations and routines. They are not visible to the other 
//files.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// Debug support routines. We can easily check whether debug is enabled at all.
// The return status routine will print out a return status message when 
// debugging is enabled. The macro "RET_STAT" is a nice helper that adds the 
// function name. The ENTER_FUNC macro is a helper to print out the function 
// name when entering a routine.
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
// "setupDefaultNodeHeader" initializes the NVM header map for a new NVM.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultNodeHeader( ) {

    ENTER_FUNC();

    LcsNvmHeader tmp;

    tmp.magicWord      = NVM_MWORD_NODE_HEADER;
    tmp.reserved1      = 0;
    tmp.reserved2      = 0;
    tmp.reserved3      = 0;

    return ( RET_STAT( rtNvmPutBytes( NVM_MAP_STORAGE_START,
                                      (uint8_t *)&tmp,
                                      sizeof( LcsNvmHeader ))));
}

//----------------------------------------------------------------------------------------
// "setupDefaultNodeMap" builds the node map structure. The newly created 
// default node map is stored to its place in the NVM. 
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultNodeMap( ) {

    ENTER_FUNC( );

    nodeMap.magicWord               = NVM_MWORD_NODE_MAP;
    nodeMap.nvmOfs                  = NVM_NODE_MAP_OFS;
    nodeMap.nvmSize                 = sizeof(LcsNodeMap);

    nodeMap.boardType               = dMap.boardType;
    nodeMap.boardVersion            = dMap.boardVersion;
    nodeMap.serialNum               = getSerialNum( );
   
    nodeMap.nodeType                = 0; 
    nodeMap.nodeFlags               = 0;
    nodeMap.nodeOptions             = 0;
    nodeMap.nodeLastErr             = 0;

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
// "setupDefaultPortData" builds the port data blocks and initializes the NVM
// portion for it. We also return the newly created node data map.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultPortData( ) {

    ENTER_FUNC( );

    portDataMap.magicWord = NVM_MWORD_PORT_DATA_MAP;
    portDataMap.nvmOfs    = NVM_PORT_DATA_OFS;
    portDataMap.nvmSize   = NVM_NODE_DATA_SIZE;

    memset( portDataMap.map, 0,
            MAX_PORT_MAP_ENTRIES * MAX_PORT_ATTR_MAP_ENTRIES * sizeof(uint16_t));

    return ( RET_STAT( rtNvmPutBytes( NVM_PORT_DATA_OFS,
                                     (uint8_t *)&portDataMap,
                                     NVM_NODE_DATA_SIZE )));
}

//----------------------------------------------------------------------------------------
// "setupDefaultEventMap" initializes an event map and writes it to NVM. 
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultEventMap( ) {

    ENTER_FUNC( );

    eventMap.magicWord = NVM_MWORD_NODE_EVENT_MAP;
    eventMap.nvmOfs    = NVM_EVENT_MAP_OFS;
    eventMap.nvmSize   = NVM_EVENT_MAP_SIZE;
    eventMap.mapHwm    = 0;

    for ( uint16_t i = 0; i < MAX_EVENT_MAP_ENTRIES; i++ ) {

        eventMap.map[i].eventId = 0;
        eventMap.map[i].eventMask = 0;
    }

    return ( RET_STAT( rtNvmPutBytes( NVM_EVENT_MAP_OFS,
                                      (uint8_t *)&eventMap,
                                      NVM_EVENT_MAP_SIZE )));
}

//----------------------------------------------------------------------------------------
// "setupDefaultGlobalData" initializes the extended attribute map and writes
// the header to NVM. The extended attribute map is just a block of attributes
// words that fills the remaining NVM space after the runtime data. We first
// check whether the globalDataMap has a MEM area allocated from a previous use.
// If so, we delete it and allocate a new one.
//
//----------------------------------------------------------------------------------------
uint8_t setupDefaultGlobalData( ) {

    ENTER_FUNC();

    globalDataMap.magicWord = NVM_MWORD_GLOBAL_DATA_MAP;   
    globalDataMap.nvmOfs    = NVM_GLOBAL_DATA_OFS;
    globalDataMap.nvmSize   = rtNvmGetSize( ) - NVM_RUNTIME_MAPS_SIZE;

    if ( globalDataMap.map != nullptr ) delete [ ] globalDataMap.map; 

    globalDataMap.map = (uint16_t *)
        new uint8_t ( globalDataMap.nvmSize - sizeof( LcsGlobalDataMap ));

    return( RET_STAT( rtNvmPutBytes( NVM_GLOBAL_DATA_OFS,
                                     (uint8_t *) &globalDataMap,
                                     globalDataMap.nvmSize )));
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
    if ( rStat == LCS_OK ) rStat = setupDefaultPortData( );
    if ( rStat == LCS_OK ) rStat = setupDefaultEventMap( );
    if ( rStat == LCS_OK ) rStat = setupDefaultGlobalData( );
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// A little helper to print a board descriptor structure in HEX for debugging 
// purposes.
//
//----------------------------------------------------------------------------------------
void printNvmHeader( LcsNvmHeader *head ) {

    uint16_t *ptr  = (uint16_t *) head;
    size_t   words = sizeof(LcsNvmHeader) / 2;

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
// There are two basic modes. The first is when we have a console connected. 
// We will prompt and wait for a start command. There are several options for 
// starting a node. The easiest is "R" which just starts the node. The "D" 
// command will start with debugging enabled. We will set the setup debug flags
// to check any issues during the startup phase. Finally, there is there "F" 
// command, which will format the NVM runtime area. 
//
// The second mode is when there no console connected. In this case, Debug is 
// disabled and we just setup the node. This mode should be the normal case for 
// all the nodes in a layout.
//
// Perhaps one day, this routine could be enhanced to allow commands to pile up 
// the start options followed by the final start command to get the show going. 
// Especially the debug mask would be a candidate.
//
// ??? what is the initial debug mask ?
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

        debugMask = LCS_DBG_NIL;
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
//----------------------------------------------------------------------------------------
uint8_t configNodeNvm( ) {

    ENTER_FUNC( );
    uint8_t rStat = rtNvmConfig( CDC_RN_NVM, NVM_MAIN_BOARD_DEF_SIZE );
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
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat )); 
    
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
    
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// Setup the watchdog timer. If enabled, the watchdog timer needs to be fed 
// periodically, otherwise we will restart. Very useful, if there is a hang
// situation.
//
//----------------------------------------------------------------------------------------
uint8_t setupWatchdog( CdcResourceDescMap *map ) {

    ENTER_FUNC( );

    uint8_t rStat = watchDogEnable( ! ( runtimeOptions & NPO_DISABLE_WATCHDOG ));
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// Setup the power fail facility. We configure the power fail detection pin.
// When power goes away, the falling edge on the pin will cause an interrupt 
// and the power fail handler executes.
//
// ??? anything to remember or set from previous PFAIL state ?
//----------------------------------------------------------------------------------------
uint8_t setupPfail( CdcResourceDescMap *map ) {

    ENTER_FUNC( );

    uint8_t rStat = configureDio( CDC_RN_PFAIL );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "checkMagicWords" is the routine which checks the individual area headers.
// Each map starts with a magic word, and we expect them at the fixed location.
// If there is a mismatch, the NVM is corrupted or the software has changed. In
// both cases we attempt to reformat the NVM area.
//
//----------------------------------------------------------------------------------------
uint8_t checkMagicWords( ) {

    ENTER_FUNC( );

    uint8_t  rStat = LCS_OK;
    uint32_t mWord = 0;

    rStat = rtNvmGetBytes( NVM_HEADER_MAP_OFS, (uint8_t *) &mWord, sizeof( mWord ));
    if ( rStat != LCS_OK) return( RET_STAT( rStat ));
    if ( mWord != NVM_MWORD_NODE_HEADER ) 
        return ( RET_STAT ( ERR_NVM_HEADER ));

    rStat = rtNvmGetBytes( NVM_NODE_MAP_OFS, (uint8_t *) &mWord, sizeof( mWord ));
    if ( rStat != LCS_OK) return( RET_STAT( rStat ));
    if ( mWord != NVM_MWORD_NODE_MAP ) 
        return ( RET_STAT ( ERR_NODE_MAP_HEADER ));

    rStat = rtNvmGetBytes( NVM_PORT_DATA_OFS, (uint8_t *) &mWord, sizeof( mWord ));
    if ( rStat != LCS_OK) return( RET_STAT( rStat ));
    if ( mWord != NVM_MWORD_PORT_DATA_MAP ) 
        return ( RET_STAT ( ERR_PORT_DATA_HEADER ));

    rStat = rtNvmGetBytes( NVM_EVENT_MAP_OFS, (uint8_t *) &mWord, sizeof( mWord ));
    if ( rStat != LCS_OK) return( RET_STAT( rStat ));
    if ( mWord != NVM_MWORD_NODE_EVENT_MAP ) 
        return ( RET_STAT( ERR_EVENT_MAP_HEADER ));

    rStat = rtNvmGetBytes( NVM_GLOBAL_DATA_OFS, (uint8_t *) &mWord, sizeof( mWord ));
    if ( rStat != LCS_OK) return( RET_STAT( rStat ));
    if ( mWord != NVM_MWORD_GLOBAL_DATA_MAP ) 
        return ( RET_STAT( ERR_GLOBAL_DATA_HEADER ));
    
    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "setupNodeNvmHeader" sets up the main controller header map entry. It is the 
// first routine after all the basic hardware settings is in place. If we detect
// an invalid NVM header or NVM formatting was requested, a default NVM runtime
// structure will be created. Either way we return with a valid NVM structure
// for the node.
//
//----------------------------------------------------------------------------------------
uint8_t setupNodeNvmHeader( CdcResourceDescMap *map ) {

    ENTER_FUNC( );

    uint8_t rStat = LCS_OK;

    if ( runtimeOptions & NPO_FORMAT_RUNTIME ) {

        if ( setupDebugEnabled( )) printf ( "Runtime Option: FORMAT\n" );

        rStat = buildNvmRuntimeStructure( );
        if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));
    }

    rStat = checkMagicWords( );
    if ( rStat != LCS_OK ) {

        rStat = buildNvmRuntimeStructure( );
        if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));
    }

    rStat = rtNvmGetBytes( NVM_HEADER_MAP_OFS,
                           (uint8_t *) &nvmHeader,
                           sizeof( LcsNvmHeader ));
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if ( setupDebugEnabled( )) printNvmHeader( &nvmHeader );
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "setupNodeMap" sets up the nodeMap. It is the routine that is called after 
// we read in the NVM headers and have a valid node runtime NVM structure. 
//
//----------------------------------------------------------------------------------------
uint8_t setupNodeMap( ) {

    ENTER_FUNC( );

    uint8_t rStat = rtNvmGetBytes( NVM_NODE_MAP_OFS,
                                   (uint8_t *) &nodeMap,
                                   NVM_NODE_MAP_SIZE);

    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if (( nodeMap.nvmOfs != NVM_NODE_MAP_OFS ) ||
        ( nodeMap.nvmSize != sizeof ( LcsNodeMap ))) {

        return( RET_STAT( ERR_NODE_MAP_HEADER ));
    }

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
// "setupPortMap" will initialize the portMap. A node can have up to 8 ports.
// PortMap is a memory only data structure.
//
//----------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

    ENTER_FUNC( );

    portMap.mapHwm = 0;

    for ( uint16_t i = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        LcsPortMapEntry *pEntry = &portMap.map[ i ];
       
        pEntry -> portOptions               = 0;
        pEntry -> portFlags                 = 0;
        pEntry -> portType                  = 0;
        pEntry -> portLastErr               = LCS_OK;
   
        pEntry -> reqCallback               = nullptr;
        pEntry -> reqCallBackUdata          = nullptr;

        pEntry -> eventCallback             = nullptr;
        pEntry -> eventCallBackUdata        = nullptr;

        pEntry -> eventNpId                 = 0;
        pEntry -> eventId                   = 0;
        pEntry -> eventValue                = 0;
        pEntry -> eventAction               = 0;
        pEntry -> eventDelayTime            = 0;
        pEntry -> eventTimeStamp            = 0;

        pEntry -> targetNpId                = 0;
        pEntry -> targetReqTs               = 0; 
        pEntry -> targetRepCallback         = nullptr;
        pEntry -> targetRepCallBackUdata    = nullptr;

        pEntry -> channelMap                = 0; 
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "setupPortDataMap" will read the port data blocks from NVM. 
//
//----------------------------------------------------------------------------------------
uint8_t setupPortDataMap( ) {

    ENTER_FUNC( );

    uint8_t rStat = rtNvmGetBytes( NVM_PORT_DATA_OFS,
                                   (uint8_t *)&portDataMap,
                                   sizeof( LcsPortDataMap ));

    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if (( portDataMap.nvmOfs != NVM_PORT_DATA_OFS ) ||
        ( portDataMap.nvmSize != sizeof ( LcsPortDataMap ))) {
         
        return( RET_STAT( ERR_PORT_DATA_HEADER ));
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// The event map stores all events this node is interested to process. The work
// is done in the event module.
//
//----------------------------------------------------------------------------------------
uint8_t setupEventMap( ) {

    ENTER_FUNC( );

    uint8_t rStat = loadEventMap( );
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// The global data map is the additional NVM storage that the chip set offers
// beyond the area allocated for the runtime data. The size depends on the 
// actual NVM chip used and the configured user map size.
//
//----------------------------------------------------------------------------------------
uint8_t setupGlobalDataMap( ) {

    ENTER_FUNC( );

    uint8_t rStat = rtNvmGetBytes( NVM_GLOBAL_DATA_OFS,
                                   (uint8_t *) &globalDataMap,
                                   sizeof( LcsGlobalDataMap ));
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    uint32_t nvmSize = rtNvmGetSize( ) - NVM_RUNTIME_MAPS_SIZE;

    if (( globalDataMap.nvmOfs != NVM_GLOBAL_DATA_OFS ) ||
        ( globalDataMap.nvmSize != nvmSize )) {

        return( RET_STAT( ERR_GLOBAL_DATA_HEADER ));
    }
    
    if ( globalDataMap.map != nullptr ) delete [ ] globalDataMap.map; 

    globalDataMap.map = (uint16_t *)
        new uint8_t ( globalDataMap.nvmSize - sizeof( LcsGlobalDataMap ));

    rStat = rtNvmGetBytes( globalDataMap.nvmOfs,
                           (uint8_t *) globalDataMap.map, 
                           globalDataMap.nvmSize );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "setupTaskMap" initializes the task map. A user can register routines that 
// are executed on a periodic basis.
//
//----------------------------------------------------------------------------------------
uint8_t setupTaskMap( ) {

    ENTER_FUNC( );

    taskMap.mapHwm = 0;
    for ( int i = 0; i < MAX_TASK_MAP_ENTRIES; i++ ) {

        LcsPTaskMapEntry *tmp = &taskMap.map[ i ];

        tmp -> task         = nullptr;
        tmp -> uData        = nullptr; 
        tmp -> timeStamp    = 0;
        tmp -> interval     = 0;
    }

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// The runtime library will one day perhaps a set of internal functions to 
// execute periodically. They should be added here. Right now, this routine
// will do nothing.
//
//----------------------------------------------------------------------------------------
uint8_t registerInternalTasks( ) {

    ENTER_FUNC( );
    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "powerFailHandler" is called when the hardware detects an imminent loss of 
//power. Our chance to save crucial data to NVM. Finally, the optionally
// registered firmware power fail callback is called. The node state becomes 
// "PFAIL". Upon restart, we check this state and know that we came back after a
// power fail.
//
// ??? we should find a way to do the PFAIL work but before entering the PFAIL 
// state check that power is still gone. Suppose we have a short power glitch
// that the capacitor covered. When power is back in the time the capacitor
// covered, we are in good shape and just continue rather than shut down.
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
// "initRuntime" is the very first thing to call in a node firmware program.
// There is a lot to do. This routine will invoke the various initializers, one
// at a time.
//
// The first three calls are the basic setup of the CDC layer, the I2C, NVM and 
// CanBus channel. The CDC layer setup also checks for a console presence and if 
// so, allows for different start modes. However, if any of them fails, we have 
// a fatal error and stop. If we have a basic hardware setup, let's check whether
// we are starting from a watchdog timer or power fail event.
//
// The remainder of the calls will setup the individual portions of the runtime.
// The overall logic of the startup code below is that if there is a fault, the 
// follow on steps are simply skipped and the node is put into the FAIL state. 
//
// Note that we still are able to access the node via the USB console and one day
// also via diagnostic LCS messages. The idea is to allow problem resolution and
// correct configuration of the nodeMap, so that we can hopefully restart with a
// correct nodeMap.
//
// ?? what would we do different when we came back from a watchdog or 
// power fail ?
//
//----------------------------------------------------------------------------------------
uint8_t initRuntime( CdcResourceDescMap  *descMap,
                     uint16_t            options,
                     uint16_t            dbgMask ) {

    uint8_t rStat = LCS_OK;

    dMap            = *descMap;
    runtimeOptions  = options;
    debugMask       = dbgMask;

    rStat = initCdcLayer( );
    if ( rStat != LCS_OK ) {

        fatalError( 1, (char *) "Fatal: CDC Layer Setup failed", rStat );
    }

    printf( "DebugMask: 0x%04x\n", debugMask );
  
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
    if ( rStat == LCS_OK )  rStat = setupPortDataMap( );
    if ( rStat == LCS_OK )  rStat = setupEventMap( );
    if ( rStat == LCS_OK )  rStat = setupGlobalDataMap( );
    if ( rStat == LCS_OK )  rStat = setupTaskMap( );
    if ( rStat == LCS_OK )  rStat = registerInternalTasks( );

    if ( rStat == LCS_OK ) {

       // ??? what would we do when we came from a PFAIL or WATCHDOG ?

        nodeMap.nodeState = NS_INIT;
    }
    else nodeMap.nodeState = NS_FAIL;

    writeDio( CDC_RN_ACTIVITY_LED, ( rStat == LCS_OK ));
    return ( RET_STAT( rStat ));
}

}; // namespace LCS