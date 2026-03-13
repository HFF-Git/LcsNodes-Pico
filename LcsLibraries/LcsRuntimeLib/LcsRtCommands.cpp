//----------------------------------------------------------------------------------------
//
// Layout Control System - Command Interpreter
//
//----------------------------------------------------------------------------------------
// Based on the Raspberry Pi PICO controller USB interface, the LCS node has an 
// option to accept commands and display data via a serial interface. This interface 
// is used for manual node and extension board configuration as well as debug and 
// troubleshooting. Most commands are sensitive to the node/port ID. If there is 
// another node than our own node, specified with a zero node ID value, the commands
// is sent via the  bus to that node.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Command Interpreter
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
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"
#include "LcsUtilLib.h"

//----------------------------------------------------------------------------------------
// External declaration to global structures and functions.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t             debugMask;
    extern uint16_t             runtimeOptions;
    extern LcsNodeMap           nodeMap;
    extern LcsNodeData          nodeData;
    extern LcsPortMap           portMap;
    extern LcsEventMap          eventMap;
    extern LcsTaskMap           taskMap;
    extern LcsDrvFuncMap        drvFuncMap;
    extern LcsMsgBusCAN         *msgBus;

    extern CdcResourceDescMap   dMap;

    extern int                  searchEvent( uint16_t eventId );
    extern uint8_t              rtNvmGetWord( uint32_t ofs, uint16_t *word );
    extern uint8_t              extNvmGetWord(  uint8_t boardId, 
                                                uint32_t ofs, 
                                                uint16_t *word );
};

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------  
// The command line buffer.
//
//----------------------------------------------------------------------------------------  
char commandBuf [ MAX_COMMAND_LINE_SIZE ];

//----------------------------------------------------------------------------------------
// Helper routines for error status handling.
//
//----------------------------------------------------------------------------------------
void errArgList( ) {

    printf( "Argument list error, use \"?\" for help\n" );
}

void errStat( char *msg, uint8_t ret ) {

    printf( "Error: %s ( %d )\n", msg, ret );
}

//----------------------------------------------------------------------------------------
// "dumpMemData" lists the memory data content of the storage area passed. The data 
// is displayed in 16-bit  quantities. Because the PICO uses little-endian format, 
// ASCII characters may appear reversed when interpreted directly.
//
//----------------------------------------------------------------------------------------
void dumpMemData( uint16_t *area, 
                  uint16_t len,
                  uint8_t  itemsPerLine = 8,
                  bool     printAscii   = false ) {

    uint16_t  index   = 0;
    uint16_t  limit   = ( len + 1 ) / 2; 
    uint16_t  *ptr    = area;

    while ( index < limit ) {

        printf( "0x%08x: ", index * sizeof( uint16_t ));

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            if ( index + i < limit ) printf( "0x%04x ", ptr[ index + i ] );
        }

        if ( printAscii ) {

            if ( index + itemsPerLine >= limit ) {

                int tmp = index + itemsPerLine - limit;
                for ( int i = 0; i < tmp; i++ ) printf( "       " );
            };

            printf( "  " );

            for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

                if ( index + i < limit ) {

                    if ( isprint( ptr[ index + i ] >> 8  )) 
                        printf( "%c", ptr[ index + i ] >> 8 );
                    else                                   
                        printf( "." );

                    if ( isprint( ptr[ index + i ] & 0xff )) 
                        printf( "%c ", ptr[ index + i ] & 0xff );
                    else                                     
                        printf( ". " );
                }
            }
        }
       
        index += itemsPerLine;
        printf( "\n" );
    }
}

//----------------------------------------------------------------------------------------
// List the NVM storage data. The function receives the absolute byte offset within 
// the NVM area and the length in bytes. The data is displayed in 16-bit quantities. 
// Because the PICO uses little-endian format, ASCII characters may appear reversed 
// when interpreted directly.
//
//----------------------------------------------------------------------------------------
void dumpNvmData( uint32_t  start, 
                  uint32_t  len, 
                  uint32_t  itemsPerLine = 8, 
                  bool      printAscii = false ) {

    uint8_t     rStat = NO_ERR;
    uint32_t    limit = start + len;
    uint16_t    val   = 0;

    while ( start < limit ) {

        printf( "0x%08x: ", start );

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = ( start + ( i * sizeof(uint16_t)));

            if ( ofs < limit ) {

                rStat = rtNvmGetWord( ofs, &val );
                if ( rStat == NO_ERR ) printf( "0x%04x ", val );
                else printf( "**** " );
            }
        }

        if ( printAscii ) {

            if ( start + ( itemsPerLine * sizeof(uint16_t)) >= limit ) {

                int tmp = ( start + 
                    ( itemsPerLine * sizeof(uint16_t)) - limit ) / sizeof( uint16_t);

                for ( int i = 0; i < tmp; i++ ) printf( "       " );
            };

            printf( "  " );

            for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

                uint32_t ofs = start + ( i * sizeof(uint16_t));

                if ( ofs < limit ) {

                    rStat = rtNvmGetWord( ofs, &val );
                    if ( rStat == NO_ERR ) {

                        if ( isprint( val >> 8  )) printf( "%c", val >> 8 );
                        else                       printf( "." );

                        if ( isprint( val & 0xff )) printf( "%c ", val & 0xFF );
                        else                        printf( ". " );
                    }
                }
            }
        }
   
        start = start + ( itemsPerLine * sizeof(uint16_t));
        printf( "\n" );
    }
}

//----------------------------------------------------------------------------------------
// Routines to list contents of the various memory areas. Right now, we just dump out 
// hex data. It would be nice to show formatted data. Perhaps one day...
//
//----------------------------------------------------------------------------------------
void dumpMemNodeMap( ) {

    printf( "MEM Node Map: \n\n" );
    dumpMemData((uint16_t *) &nodeMap, sizeof( LcsNodeMap ), 8, true);
    printf( "\n" );
}

void dumpMemPortMap( ) {

    printf( "MEM Port Map: \n\n" );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        printf( "Port %d:\n", i );
        dumpMemData((uint16_t *) &portMap.map[ i ], sizeof( LcsPortMapEntry ));
        printf( "\n" );
    }
    
    printf( "\n" );
}
 
void dumpMemNodeData( ) {

    printf( "MEM Node Data: \n\n" );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        printf( "Port %d:\n", i );
        dumpMemData((uint16_t *) &nodeData.map[ i ], 
                    MAX_ATTR_MAP_ENTRIES * sizeof( uint16_t ), 8, true );
        printf( "\n" );
    }
}

void dumpMemEventMap( ) {

    printf( "MEM Event Map (Size: %d, Hwm: %d): \n\n", 
            MAX_EVENT_MAP_ENTRIES, eventMap.mapHwm );

    dumpMemData((uint16_t *) &eventMap, sizeof( LcsEventMap ));
    printf( "\n" );
}

void dumpMemTaskMap( ) {

    printf( "MEM Task Map: (Size: %d, Hwm: %d) \n\n", 
            MAX_TASK_MAP_ENTRIES, taskMap.mapHwm );

    dumpMemData((uint16_t *) &taskMap, sizeof( LcsTaskMap ));
    printf( "\n" );
}

void dumpMemDrvFuncMap( ) {

    printf( "MEM Driver Function Map: (Size: %d, Hwm: %d) \n\n", 
            MAX_DRV_TYPE_MAP_ENTRIES, drvFuncMap.mapHwm );

    for ( int i  = 0; i < MAX_DRV_TYPE_MAP_ENTRIES; i++ ) {

        LcsDrvFuncEntry *entry = &drvFuncMap.map[ i ];
        printf( "%2d: Type: %2d, Func: %p\n", i, entry -> drvType, entry -> drvFunc );
    }

    printf( "\n" );
}

void dumpMemRuntimeArea( ) {

    printf( "MEM Area Dump: \n\n" );
    dumpMemNodeMap( );
    dumpMemPortMap( );
    dumpMemNodeData( );
    dumpMemEventMap( );
    dumpMemTaskMap( );
    dumpMemDrvFuncMap( );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// Routines to list contents of the various NVM areas. Right now, we just dump out hex 
// data. It would be nice to also show formatted data. Perhaps one day...
//
//----------------------------------------------------------------------------------------
void dumpNvmHeader( ) {

    printf( "NVM Header (Node): \n" );
    dumpNvmData( NVM_HEADER_MAP_OFS, sizeof(LcsBoardDesc), 8, true );
    printf( "\n" );
}

void dumpNvmNodeMap( ) {

    printf( "NVM Node Map Dump: \n\n" );
    dumpNvmData( NVM_NODE_MAP_OFS, NVM_NODE_MAP_SIZE, 8, true );
    printf( "\n" );
}

void dumpNvmNodeData( ) {

    printf( "NVM Node Data Dump: \n\n" );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        uint32_t start = NVM_NODE_DATA_OFS + ( i * MAX_ATTR_MAP_ENTRIES * 2 );

        printf( "Port %d:, Adr: 0x%08x\n", i, 
        NVM_NODE_DATA_OFS + ( i * MAX_ATTR_MAP_ENTRIES * 2 ));

        dumpNvmData( start, 
                     MAX_ATTR_MAP_ENTRIES * 2,
                     8, 
                     true );
        printf( "\n" );
    }

    printf( "\n" );
}

void dumpNvmEventMap( ) {

    printf( "NVM Node Event Dump: \n\n" );
    dumpNvmData( NVM_EVENT_MAP_OFS, NVM_EVENT_MAP_SIZE );
    printf( "\n" );
}

void dumpNvmRuntimeArea( ) {

    printf( "NVM Runtime Area Dump: \n\n" );
    dumpNvmData( NVM_MAP_STORAGE_START, NVM_RUNTIME_MAPS_SIZE, 8, true );
    printf( "\n" );
}

void dumpNvmUserArea( ) {

    // ??? given that this could be thousands... long list.

    printf( "NVM Area Dump: \n\n" );
    dumpNvmData( NVM_USER_MAP_OFS , usrNvmGetSize( ), 8, true );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// Print memory structures in a formatted way. Note that not all memory structures 
// are printed. Some of the maps contain dynamic data, which changes rapidly. There
// is no point in showing that kind of data.
//
//----------------------------------------------------------------------------------------
void printSummary( ) {

    printf( "LCS Node: \n" );
     
    printf( "Library Version: %d.%d, Patch Level: %d\n", 
            highByte( LCS_RT_LIB_VERSION ),
            lowByte( LCS_RT_LIB_VERSION ), 
            LCS_RT_LIB_PATCH_LEVEL );  

    printf( "Git Branch: %s\n", LCS_RT_LIB_GIT_BRANCH );
}

void printMemNodeMap( ) {

    printf( "MEM Node Map: \n\n" );

    printf( "NodeId: %d, nodeUID: %d\n",
            nodeMap.nodeId,
            nodeMap.nodeUID );

    printf( "RtLib SW Version: %d.%d.%d\n", 
            highByte( LCS_RT_LIB_VERSION ), 
            lowByte( LCS_RT_LIB_VERSION ),
            LCS_RT_LIB_PATCH_LEVEL );

    printf( "Options: 0x%04x\n", runtimeOptions );
    printf( "Restart Count: %d\n", nodeMap.nodeRestartCnt );
    printf( "Node State: %d\n", nodeMap.nodeState );
    printf( "\n" );
}

void printMemPortMap( ) {

    printf( "MEM Port Map (Size: %d, Hwm: %d): \n\n", 
            MAX_PORT_MAP_ENTRIES, portMap.mapHwm );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        LcsPortMapEntry *ptr = &portMap.map[ i ];

        printf( "Port %02d: Type: %02d, Flags: 0x%04x\n", 
                i + 1,  
                ptr -> portType,
                ptr -> portFlags );

        dumpMemData((uint16_t *) &portMap.map[ i ], 
                    sizeof( LcsPortMapEntry ), 8, true );

        printf( "\n" );
    }
}
 
void printMemEventMap( ) {

    const int itemsPerLine = 4;

    printf( "MEM Event Map (Size: %d, Hwm: %d): \n\n", 
            MAX_EVENT_MAP_ENTRIES, eventMap.mapHwm );

    if ( eventMap.mapHwm > 0 ) {

        for ( int i = 0; i < eventMap.mapHwm; i++ ) {

            for ( int j = 0; j < itemsPerLine; j++ ) {

                if (( i * itemsPerLine ) + j < eventMap.mapHwm ) {

                    printf( "(E: %d, M: 0x%04x) ", 
                    eventMap.map[ ( i * itemsPerLine ) + j ].eventId, 
                    eventMap.map[ ( i * itemsPerLine ) + j ].eventMask );
                }
            }

            printf( "\n" );
            i += itemsPerLine;
        }
    }
    else printf( "No entries in map\n" );
}

void printMemTaskMap( ) {

    printf( "Task Map (Size: %d, Hwm: %d): \n\n", 
            MAX_TASK_MAP_ENTRIES, taskMap.mapHwm );

    // ??? to do ...
}

void printMemDriverMap( ) {

    printf( "Driver Map (Size: %d, Hwm: %d): \n\n", 
            MAX_DRV_TYPE_MAP_ENTRIES, drvFuncMap.mapHwm );

    // ??? to do ...
}

//----------------------------------------------------------------------------------------
// List the I2C devices found on the internal and external I2C bus.
// 
//----------------------------------------------------------------------------------------
void listDevicesI2C( ) {

    scanI2CBus( CDC_RN_NVM );
    printf( "\n" );

    scanI2CBus( CDC_RN_EXT_NVM );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
char *skipSpaces( char *s ) {

    while (( *s == ' ' ) || ( *s == '\t' )) s++;
    return s;
}

}; // namespace


//----------------------------------------------------------------------------------------
// Routines in LCS name space.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "c" switches a node to CFG mode. For a local node command, we construct the 
// LCS_OP_CFG message payload data and invoke the msg handler for switching the 
// node mode. For any other node, we will just send a LCS message.
//
//    C [ npId ]
//
//    returns: none
//
//----------------------------------------------------------------------------------------
void switchToConfigCommand( char *s ) {

    int npId = NIL_NODE_ID;

    if ( sscanf( s, "%i", &npId ) > 1 ) return ( errArgList( ));

    uint16_t tmpNpId = (uint16_t) npId;

    if (( npId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {

       nodeMap.nodeState = NS_CONFIG;
    }
    else {

        uint8_t ret = sendCfg( tmpNpId );
        if ( ret != LCS_OK ) errStat((char *) "Remote Node send error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "o" switches the nodes to OPS mode. For a local node command, we construct the 
// LCS_OP_OPS message payload data and invoke the msg handler for switching the node 
// mode. For any other node, we will just send a LCS message.
//
//    O [ npId ]
//
//----------------------------------------------------------------------------------------
void switchToOperationsCommand( char *s ) {

    int npId = NIL_NODE_ID;

    if ( sscanf( s, "%i", &npId ) > 1 ) return ( errArgList( ));

    uint16_t tmpNpId = (uint16_t) npId;

    if (( npId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {

        nodeMap.nodeState = NS_OPERATE;
    }
    else {

        uint8_t ret = sendOps( tmpNpId );
        if ( ret != LCS_OK ) errStat((char *) "Remote Node send error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "g" handles the node/port attribute query command. If the node is our node, we 
// call the local access routines. Otherwise we send a message.
//
//    <!g npId item [ val ]>
//
//    npId      - the node/port Id.
//    item      - the node item to query, the result will be listed in HEX format.
//    val       - the argument 1 on input.
//
//----------------------------------------------------------------------------------------
void getNodeCommand( char *s ) {

    int     npId    = 0;
    int     item    = 0;
    int     arg     = 0;
    uint8_t ret     = LCS_OK;

    if ( sscanf(  s, "%i %i %i %i", &npId, &item, &arg ) < 2 ) 
        return ( errArgList( ));

    uint16_t tmpNpId    = (uint16_t) npId;
    uint8_t  tmpItem    = (uint8_t)  item;
    uint16_t tmpArg     = (uint16_t) arg;

    if (( tmpNpId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {

        ret = nodeGet ( tmpNpId, tmpItem, &tmpArg );
        if ( ret != LCS_OK ) errStat((char *) "Node GET error", ret );
        else printf( "Node: 0x%x, item: %d, arg1: 0x%x\n", 
                     tmpNpId, tmpItem, tmpArg );
    }
    else {

        ret = sendGetNode( 0, tmpNpId, tmpItem, tmpArg ); // ??? fix...
         if ( ret != LCS_OK ) errStat((char *) "Remote Node GET error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "p" handles the node or port attribute value set command. If the node is out node, 
// we call the local access routines. Otherwise we send a message.
//
//    <!p npId item [ val ]>
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val      - the item value 1
//
//----------------------------------------------------------------------------------------
void putNodeCommand( char *s ) {

    int     npId    = 0;
    int     item    = 0;
    int     val     = 0;
    uint8_t ret     = LCS_OK;

    if ( sscanf(  s, "%i %i %i %i", &npId, &item, &val ) < 2 ) 
        return ( errArgList( ));

    uint16_t tmpNpId    = (uint16_t) npId;
    uint8_t  tmpItem    = (uint8_t)  item;
    uint16_t tmpVal     = (uint16_t) val;

    printf ( "val: %d\n", val );

    if (( tmpNpId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {
     
        ret = nodeSet( tmpNpId, tmpItem, tmpVal );
        if ( ret != LCS_OK ) errStat((char *) "Node SET error", ret );
        else printf( "Node: 0x%x, item: %d, val: 0x%x\n", 
                    tmpNpId, tmpItem, tmpVal );
    }
    else {

        ret = sendSetNode( 0, tmpNpId, tmpItem, tmpVal ); // ??? fix ...
        if ( ret != LCS_OK ) errStat((char *) "Remote Node SET error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "r" handles the node / port request command. If the node is out node, we call
// the local access routines. Otherwise we send a message.
//
//    r npId item [ val1 [ val2 ]]
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//
// ??? need to rethink. if a REQ local to the node, we still want it to behave 
// like a REQ/REP pair... ?
//----------------------------------------------------------------------------------------
void reqNodeCommand( char *s ) {

    int     npId    = 0;
    int     item    = 0;
    int     val1    = 0;
    int     val2    = 0;
    uint8_t ret     = LCS_OK;

    if ( sscanf(  s, "%i %i %i %i", &npId, &item, &val1, &val2 ) < 2 ) 
        return ( errArgList( ));

    uint16_t tmpNpId    = (uint16_t) npId;
    uint8_t  tmpItem    = (uint8_t)  item;
    uint16_t tmpVal1    = (uint16_t) val1;
    uint16_t tmpVal2    = (uint16_t) val2;

    if (( tmpNpId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {
     
        ret = nodeReq( tmpNpId, tmpItem, &tmpVal1, &tmpVal2 );
        if ( ret != LCS_OK ) errStat((char *) "Node REQ error", ret );
        else printf( "Node: 0x%x, item: %d, val1: 0x%x, val2: 0x%x\n", 
                    tmpNpId, tmpItem, tmpVal1, tmpVal2 );
    }
    else {

        ret = sendReqNode( nodeMap.nodeId, tmpNpId, tmpItem, tmpVal1, tmpVal2 );
        if ( ret != LCS_OK ) errStat((char *) "Remote Node REQ error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "e" will send an event. We will broadcast a message and also simulate receiving 
// an event on the local node. Sending to ourselves is also quite useful for debug
// event callback handlers.
//
//    e mode npId eventId [ arg ]
//
//    mode      - 0 - ON, 1 - OFF, 2 - DATA
//    npId      - the sending node / port Id
//    eventId   - the event Id
//    arg       - optional data argument for the data event.
//
//----------------------------------------------------------------------------------------
void sendEventCommand( char *s ) {

    int     npId        = NIL_NODE_ID;
    int     eventId     = NIL_EVENT_ID;
    int     mode        = 0;
    int     arg         = 0;
    int     len         = 0;
    uint8_t ret         = LCS_OK;

    len = sscanf( s, "%i %i %i %i", &mode, &npId, &eventId, &arg );

    uint16_t tmpNpId    = (uint16_t) npId;
    uint16_t tmpEvent   = (uint16_t) eventId;
    uint16_t tmpArg     = (uint16_t) arg;

    if ( len < 3 ) return ( errArgList( ));

    if      ( mode == 0 ) ret = sendEventOn( tmpNpId, tmpEvent ); 
    else if ( mode == 1 ) ret = sendEventOff( tmpNpId, tmpEvent ); 
    else if ( mode == 2 ) ret = sendEvent( tmpNpId, tmpEvent, tmpArg ); 

    if ( ret != LCS_OK ) errStat((char *) "Send event error", ret );
}

//----------------------------------------------------------------------------------------
// "B" broadcasts a LCS message. Mainly used for low level debugging purposes. 
// Although most commands in the LCS console interface can also send messages to 
// other nodes, not all messages are covered. This command sends any kind of message,
// even undefined ones. 
//
//    B byte1 [ byte2 ... byte8 ]
//
//    byte1 .. byte8   - the packet data in hexadecimal
//
//----------------------------------------------------------------------------------------
void broadcastLcsMsgCommand( char *s ) {

    int     inBuf[ 8 ]  = { 0 };
    uint8_t b[ 8 ]      = { 0 };
    uint8_t nBytes  = sscanf( s, "%i %i %i %i %i %i %i %i",
                            inBuf, inBuf + 1, inBuf + 2, inBuf + 3, 
                            inBuf + 4, inBuf + 5, inBuf + 6, inBuf + 7 );

    if ( nBytes >= 1 && nBytes <= 8 ) {

        for ( int i = 0; i < 8; i++ ) b[ i ] == (uint8_t) inBuf[ i ];

        uint8_t ret = msgBus -> sendLcsMsg( nodeMap.nodeId, b ); 
        if ( ret != LCS_OK ) errStat((char *) "Can Bus send error", ret );
    }
    else errArgList( );
}

//----------------------------------------------------------------------------------------
// "s" lists status information. The level argument specifies the what and the detail 
// level.
//
//    s [ level ]
//
//    returns:  NONE.
//
//----------------------------------------------------------------------------------------
void listStatusCommand( char *s ) {

    int level = 0;

    if ( sscanf( s, " %i", &level ) > 0 ) {

        switch ( level ) {

            case 0:     printSummary( );                break;
            case 2:     dumpMemNodeMap( );              break;
            case 3:     dumpMemNodeData( );             break;
            case 4:     dumpMemEventMap( );             break;
            case 5:     dumpMemPortMap( );              break;
            case 6:     dumpMemTaskMap( );              break;
            case 7:     dumpMemDrvFuncMap( );           break;
            case 8:     dumpMemRuntimeArea( );          break;

            case 21:    dumpNvmHeader( );               break;
            case 22:    dumpNvmNodeMap( );              break;
            case 23:    dumpNvmNodeData( );             break;
            case 24:    dumpNvmEventMap( );             break;
            case 28:    dumpNvmRuntimeArea( );          break;

            case 42:    printMemNodeMap( );             break;
            case 44:    printMemEventMap( );            break;
            case 45:    printMemPortMap( );             break;
            case 46:    printMemTaskMap( );             break;
            case 47:    printMemDriverMap( );           break;
            
            case 50:    listDevicesI2C( );              break;   
            case 51:    printResourceDescMap( &dMap );  break;
            case 52:    printResourceMap( );            break;

            default: printf( "Unknown help option, use '?' for help\n" );
        }
    } 
    else printSummary( );
}

//----------------------------------------------------------------------------------------
// "?" lists core library help information. We just list the available commands and a 
// short description.
//
//    ?
//
//----------------------------------------------------------------------------------------
void listCoreLibHelpCommand( ) {

    printf( "Commands: \n\n" );
    printf( "c [ npId ] - enter config mode\n" );
    printf( "o [ npId ] - enter operations mode\n" );

    printf( "g npId item [ val1 ]         - gets a node attribute\n" );
    printf( "p npId item val1             - puts a node attribute\n" );
    printf( "r npId item [ val1 [ val2 ]] - request a node function\n" );
    printf( "e npId eventId mode [ arg ]  - simulate sending an event"
            " ( mode: 0 - ON, 1 - OFF, 2 - EVT )\n" );

    printf( "B byte1 [ byte2 ... byte8 ] - broadcast a raw LCS message\n" );

    printf( "s [ level ] - list status, default is summary\n" );
    printf( "   " " -   MEM  NVM  FMT   - what\n" );
    printf( "   " " -   0               - Board summary\n" );
    printf( "   " " -        21   41    - Node Header\n" );
    printf( "   " " -   2    22   42    - Node Map\n" );
    printf( "   " " -   3    23         - Node Data\n" );
    printf( "   " " -   4    24   44    - Event Map\n" );
    printf( "   " " -   5         45    - Port Map\n" );
    printf( "   " " -   6         46    - Task Map\n" );
    printf( "   " " -   7         47    - Driver Map\n" );
    printf( "   " " -   8    28         - Runtime Area\n" );

    printf( "   " " -  50  - Scan I2C Devices\n" );
    printf( "   " " -  51  - CDC Resource Desc Map\n");
    printf( "   " " -  52  - CDC Resource Map\n");
}

//----------------------------------------------------------------------------------------
// "setupSerialCommand" initializes the serial interface. We use the PICO USB as 
// console IO. The CDC lib contains functions for reading and writing to the console.
//
//----------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

    return ( configureUsbIO( ));
}

//----------------------------------------------------------------------------------------
// "executeCommand" is the single command handler. We decode the first character 
// pass the rest of the command string to the actual handler routine. An unknown
// command tries to pass the command to the optional callback routine.
//
//----------------------------------------------------------------------------------------
static void executeCommand( char *commandBuf ) {

    char *cmd = skipSpaces( commandBuf );

    if ( *cmd == '\0' ) return;

    switch ( cmd[0] ) {

        case 'C': switchToConfigCommand(cmd + 1);     break;
        case 'O': switchToOperationsCommand(cmd + 1); break;

        case 'g': getNodeCommand(cmd + 1);            break;
        case 'p': putNodeCommand(cmd + 1);            break;
        case 'r': reqNodeCommand(cmd + 1);            break;
        case 'e': sendEventCommand(cmd + 1);          break;

        case 'B': broadcastLcsMsgCommand(cmd + 1);    break;

        case 's': listStatusCommand(cmd + 1);         break;
        case '?': listCoreLibHelpCommand();           break;

        default: {
    
            if (nodeMap.cmdLineCallback != nullptr) {

                nodeMap.cmdLineCallback( cmd, nodeMap.cmdLineCallBackUdata );
            } 
            else printf("<Unknown command, use '?' for help>");
            
        } break;
    }
}

//----------------------------------------------------------------------------------------
// "handleSerialCommand" reads commands from the console. It is a simple command
// executor with simple syntax, originally used on the DCC++ world. Note that this
// routine is called as part of the runtime loop. Consequently, it cannot not block
// for IO. The interface is designed in a way that it assembles the character input
// when there are characters until a carriage return is received. A command line 
// can optionally consist of multiple commands separated by a "/" character.
//
// Since we are pretty basic on a character by character basis, we also add a bit
// of luxury and echo back what was typed and also process the backspace character.
//
//----------------------------------------------------------------------------------------
uint8_t handleSerialCommand( void ) {

    char c;
    bool escaped = false;
    bool append  = false;

    while (( c = usbIoGetChar( 0 )) > 0 ) {

        append = false;

        switch ( c ) {

            case '\\': {

                if ( escaped ) { 

                    append  = true;   // literal '\'
                    escaped = false;
                } 
                else escaped = true;

            } break;

            case '\b': {

                escaped = false;
                printf("\b \b");
                size_t len = strlen(commandBuf);
                if ( len > 0 ) commandBuf[len - 1] = '\0';
            
            } break;

            case '\r': {

                escaped = false;
                printf( "\n" );

                char *p = commandBuf;
                char *start = p;

                while ( *p ) {

                    if (( *p == '/' ) && (( p == commandBuf ) || ( p[-1] != '\\' ))) {

                        *p = '\0';
                        char *cmd = skipSpaces( start );
                        if ( *cmd ) executeCommand(cmd);
                        start = p + 1;
                    }

                    p++;
                }

                char *cmd = skipSpaces(start);
                if ( *cmd ) executeCommand(cmd);

                commandBuf[0] = '\0';

                if      (nodeMap.nodeState == NS_CONFIG)  printf( "(C)->" );
                else if (nodeMap.nodeState == NS_OPERATE) printf( "(O)->" );
                else                                      printf( "->" );

            } break;

            default: {
                
                append  = true;
                escaped = false;
            
            } break;
        }

        if ( append ) {

            printf( "%c", c );
            size_t len = strlen( commandBuf );

            if ( len < MAX_COMMAND_LINE_SIZE - 1 ) {
     
                commandBuf[len]     = c;
                commandBuf[len + 1] = '\0';
            }
        }
    }

    return ( LCS_OK );
}

}; // namespace LCS
