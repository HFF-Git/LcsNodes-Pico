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

//----------------------------------------------------------------------------------------
// External declaration to global structures and functions.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t             debugMask;
    extern uint16_t             runtimeOptions;
    extern LcsNodeMap           nodeMap;
    extern LcsPortMap           portMap;
    extern LcsEventMap          eventMap;
    extern LcsPortDataMap       portDataMap;
    extern LcsPortDataMap       globalDataMap;
    extern LcsTaskMap           taskMap;
    extern LcsMsgBusCAN         *msgBus;
    extern CdcResourceDescMap   dMap;

    extern uint8_t              rtNvmGetWord( uint32_t ofs, uint16_t *word );
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
// We can dump memory or NVM content. The routines to list the data are very
// similar. We print a line of up to n items and several lines of all zeroes
// are condensed. The routines accept a context, which tells from where the 
// data is coming from.
//
//----------------------------------------------------------------------------------------
typedef enum {

    DUMP_SRC_MEM,
    DUMP_SRC_NVM

} DumpSource;

typedef struct {

    DumpSource  type;
    uint16_t    *memPtr;
    uint32_t    nvmStart;

} DumpContext;

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
// A helper function to check if a line is all zeroes.
//
//----------------------------------------------------------------------------------------
bool isZeroLine( uint16_t *line, bool *valid, uint16_t itemsPerLine ) {

    for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

        if ( valid[ i ] && line[ i ] != 0 ) return false;
    }

    return true;
}

//----------------------------------------------------------------------------------------
// Print a line of data. We list the content in hex and optional append an 
//ASCII version.
//
//----------------------------------------------------------------------------------------
void printLineBuf( uint32_t   address,
                   uint16_t   *line,
                   bool       *valid,
                   uint16_t   itemsPerLine,
                   bool       printAscii ) {

    printf( "0x%08x: ", address );

    for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

        if ( valid[ i ]) printf( "0x%04x ", line[ i ]);
        else             printf( "     " );
    }

    if ( printAscii ) {

        printf( "  " );
 
        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {
            
            if ( valid[ i ] ) {

                uint16_t v = line[i];
                printf("%c", isprint( v >> 8 ) ? v >> 8 : '.' );
                printf("%c ", isprint( v & 0xff ) ? v & 0xff : '.' );
            }
        }
    }

    printf("\n");
}

//----------------------------------------------------------------------------------------
// "fetchLine" retrieves a line of data depending on the dump context. We are
// passed the starting byte address, the buffer to which the data is written,
// list options and the limit of the address range. We also detect if a line
// consists of all zeroes. The "valid" array indicates if the corresponding 
// item is valid or not.
//
//----------------------------------------------------------------------------------------
void fetchLine( const DumpContext *ctx,
                uint32_t index,
                uint16_t *line,
                bool *valid,
                uint16_t itemsPerLine,
                uint32_t limit ) {

    if ( ctx -> type == DUMP_SRC_MEM ) {

        uint32_t wordIndex = index / sizeof(uint16_t);

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t addr = ( wordIndex + i ) * sizeof(uint16_t);

            if ( addr < limit ) {

                line[ i ]  = ctx -> memPtr[ wordIndex + i ];
                valid[ i ] = true;
            } 
            else valid[ i ] = false;
        }
    }
    else {

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = index + i * sizeof(uint16_t);

            if ( ofs < limit ) {

                uint8_t rStat = rtNvmGetWord( ofs, &line[ i ]);
                valid[ i ] = ( rStat == NO_ERR );
            } 
            else valid[ i ] = false;
        }
    }
}

//----------------------------------------------------------------------------------------
// "dumpCore" is the work horse for listing data. We get the context, start and 
// end address and a couple of options that control the printout. "itemsPerLine"
// and "byteStep" are used to control how many items per line and the data size 
//in bytes of the itemLine shown.  
//
// For large data areas, we can compress lines of all zeroes. If the number of 
// consecutive zero lines exceeds a threshold, we print the first and last line
// of the run and indicate that there are lines in between that are not shown. 
// The "printAscii" option will append an ASCII version of the data to the hex
// printout. 
//
//----------------------------------------------------------------------------------------
void dumpCore( const DumpContext *ctx,
               uint32_t start,
               uint32_t limit,
               uint16_t itemsPerLine,
               uint32_t byteStep,
               bool printAscii,
               bool compressZeroes ) {

    const uint16_t zeroLinesThreshold = 4;

    uint16_t line[ MAX_ITEMS_PER_LINE ];
    bool     valid[ MAX_ITEMS_PER_LINE ];

    uint32_t index          = start;
    uint32_t zeroRunStart   = 0;
    uint32_t zeroRunLength  = 0;

    while ( index < limit ) {

        fetchLine( ctx, index, line, valid, itemsPerLine, limit );

        bool isZero = isZeroLine( line, valid, itemsPerLine );
        if ( !compressZeroes ) isZero = false;

        if ( isZero ) {

            if ( zeroRunLength == 0 ) zeroRunStart = index;
            zeroRunLength++;
        }
        else {

            if ( zeroRunLength > 0 ) {

                if ( zeroRunLength <= zeroLinesThreshold ) {

                    for ( uint32_t i = 0; i < zeroRunLength; i++ ) {

                        uint32_t tmp = zeroRunStart + i * byteStep;

                        fetchLine( ctx, tmp, line, valid,
                                   itemsPerLine, limit );

                        printLineBuf( tmp, line, valid,
                                      itemsPerLine, printAscii );
                    }

                } 
                else {

                    fetchLine( ctx, zeroRunStart, line, valid,
                               itemsPerLine, limit );

                    printLineBuf( zeroRunStart, line, valid,
                                  itemsPerLine, printAscii );

                    printf("...\n");

                    uint32_t last =
                        zeroRunStart + ( zeroRunLength - 1 ) * byteStep;

                    fetchLine( ctx, last, line, valid,
                                itemsPerLine, limit );

                    printLineBuf( last, line, valid,
                                  itemsPerLine, printAscii );
                }

                zeroRunLength = 0;
            }

            fetchLine( ctx, index, line, valid, itemsPerLine, limit );
            printLineBuf( index, line, valid, itemsPerLine, printAscii );
        }

        index += byteStep;
    }

    // flush trailing zeros
    if ( compressZeroes && zeroRunLength > 0 ) {

        if ( zeroRunLength <= zeroLinesThreshold ) {

            for ( uint32_t i = 0; i < zeroRunLength; i++ ) {

                uint32_t tmp = zeroRunStart + i * byteStep;

                fetchLine( ctx, tmp, line, valid, itemsPerLine, limit );
                printLineBuf( tmp, line, valid, itemsPerLine, printAscii );
            }

        } else {

            fetchLine( ctx, zeroRunStart, line, valid, itemsPerLine, limit );
            printLineBuf( zeroRunStart, line, valid, itemsPerLine, printAscii );

            printf( "...\n" );

            uint32_t last = zeroRunStart + ( zeroRunLength - 1 ) * byteStep;

            fetchLine( ctx, last, line, valid, itemsPerLine, limit );
            printLineBuf( last, line, valid, itemsPerLine, printAscii );
        }
    }
}

//----------------------------------------------------------------------------------------
// A simple API to list our memory content with convenient defaults. We dump the
// memory in 8 items per line, print an ASCII version and compress lines of all
// zeroes.
//
//----------------------------------------------------------------------------------------
void dumpMemData( uint16_t *area,
                  uint32_t len,
                  uint8_t itemsPerLine = 8,
                  bool printAscii = true,
                  bool compressZeroes = true ) {

    DumpContext ctx = { .type = DUMP_SRC_MEM, .memPtr = area };

    dumpCore( &ctx,
              0,
              len,
              itemsPerLine,
              itemsPerLine * sizeof(uint16_t),
              printAscii,
              compressZeroes );
}

//----------------------------------------------------------------------------------------
// A simple Api to list our NVM content. We dump the NVM starting at the start
// offset in 8 items per line, print an ASCII version and compress lines of all
//
//----------------------------------------------------------------------------------------
void dumpNvmData( uint32_t start,
                  uint32_t len,
                  uint8_t itemsPerLine = 8,
                  bool printAscii = true,
                  bool compressZeroes = true ) {

    DumpContext ctx = { .type = DUMP_SRC_NVM, .nvmStart = start };

    dumpCore( &ctx,
              start,
              start + len,
              itemsPerLine,
              itemsPerLine * sizeof(uint16_t),
              printAscii,
              compressZeroes );
}

//----------------------------------------------------------------------------------------
// "DumpMemNodeMap" dumps the MEM node map.
//
//----------------------------------------------------------------------------------------
void dumpMemNodeMap( ) {

    printf( "MEM Node Map: \n\n" );
    dumpMemData((uint16_t *) &nodeMap, sizeof( LcsNodeMap )) ;
    printf( "\n" ); 
}

//----------------------------------------------------------------------------------------
// "dumpMemPortMap" dumps the MEM port map structure in memory. The port map is
// an array of port map entries. We dump each entry separately.
//
//----------------------------------------------------------------------------------------
void dumpMemPortMap( ) {

    printf( "MEM Port Map: (Hwm: %d)\n\n", portMap.mapHwm );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        printf( "Port %d:\n", i );
        dumpMemData((uint16_t *) &portMap.map[ i ], sizeof( LcsPortMapEntry ));
        printf( "\n" );
    }
    
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpMemEventMap" dumps the MEM event map structure in memory. 
//
//----------------------------------------------------------------------------------------
void dumpMemEventMap( ) {

    printf( "MEM Event Map: (Hwm: %d) \n\n", eventMap.mapHwm );
    dumpMemData(( uint16_t *) eventMap.map, 
                sizeof( eventMap.mapHwm ), 
                8, 
                false,
                true );
}

//----------------------------------------------------------------------------------------
// "dumpMemPortData" dumps the MEM port data structure in memory. The port data
// map is an array of the items per port. We dump each port separately.
//
//----------------------------------------------------------------------------------------
void dumpMemPortData( ) {

    printf( "MEM Port Data: \n\n" );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        printf( "Port %d:\n", i );
        dumpMemData((uint16_t *) &portDataMap.map[ i ], 
                    MAX_PORT_ATTR_MAP_ENTRIES * sizeof( uint16_t ), 8, true );
        printf( "\n" );
    }
}

//----------------------------------------------------------------------------------------
// "dumpMemGlobalData" dumps the MEM global data structure in memory. 
//
//----------------------------------------------------------------------------------------
void dumpMemGlobalData( ) {

    printf( "MEM Global Data: ( Size: %d ) \n\n", globalDataMap.nvmSize );

    if ( globalDataMap.map == nullptr ) {

        printf( "Global data map pointer is null.\n" );
        return;
    }

    dumpMemData((uint16_t *) &globalDataMap.map, globalDataMap.nvmSize );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpMemTaskMap" dumps the MEM task map structure in memory. 
//
//----------------------------------------------------------------------------------------
void dumpMemTaskMap( ) {

    printf( "MEM Task Map: (Size: %d, Hwm: %d) \n\n", 
            MAX_TASK_MAP_ENTRIES, taskMap.mapHwm );

    dumpMemData((uint16_t *) &taskMap.map, sizeof( LcsTaskMap ) - 4 );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// And for the great finale, we dump the entire MEM runtime area. This is a 
// combination of all the MEM areas. 
//
//----------------------------------------------------------------------------------------
void dumpMemRuntimeArea( ) {

    printf( "MEM Area Dump: \n\n" );
    dumpMemNodeMap( );
    dumpMemPortMap( );
    dumpMemEventMap( );
    dumpMemPortData( );
    dumpMemGlobalData( );
    dumpMemTaskMap( );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpNvmHeader" dumps the NVM header structure. 
//
//----------------------------------------------------------------------------------------
void dumpNvmHeader( ) {

    printf( "NVM Header: \n" );
    dumpNvmData( NVM_HEADER_MAP_OFS, sizeof(LcsNvmHeader));
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
//  "dumpNvmNodeMap" dumps the NVM node map structure.
//
//----------------------------------------------------------------------------------------
void dumpNvmNodeMap( ) {

    printf( "NVM Node Map Dump: \n\n");
    printf( "Header: " );
    dumpNvmData( NVM_NODE_MAP_OFS, 12, 8, false );
    printf( "\n" );

    printf( "Data: \n\n" );
    uint32_t start = NVM_NODE_MAP_OFS + 12;
    dumpNvmData( start, NVM_NODE_MAP_SIZE );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpNvmEventMap" dumps the NVM event map structure. For convenience, we also
// lost the header separately, followed by the port entry data. The header size
// portion is 12 bytes.
//
//----------------------------------------------------------------------------------------
void dumpNvmEventMap( ) {

    printf( "NVM Event Map Dump: \n\n");
    printf( "Header: " );
    dumpNvmData( NVM_EVENT_MAP_OFS, 12, 8, false );
    printf( "\n" );

    printf( "Data: \n\n" );
    uint32_t start = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map );
    dumpNvmData( start, NVM_EVENT_MAP_SIZE );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpNvmPortMap" dumps the NVM port map structure. The port map is an array
// of port map entries. We dump each entry separately. For convenience, we also
// lost the header separately, followed by the port entry data. The header size
// portion is 12 bytes.
//
//----------------------------------------------------------------------------------------
void dumpNvmPortData( ) {

    printf( "NVM Node Data Dump: \n\n");
    printf( "Header: " );
    dumpNvmData( NVM_PORT_DATA_OFS, 12, 8, false );
    printf( "\n" );

    uint32_t start = NVM_PORT_DATA_OFS + offsetof( LcsPortDataMap, map );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        uint32_t adr = 
           start + ( i * MAX_PORT_ATTR_MAP_ENTRIES * sizeof( uint16_t ));

        printf( "Port %d:, Adr: 0x%08x\n", i, adr );
        dumpNvmData( adr, MAX_PORT_ATTR_MAP_ENTRIES * 2 );
        printf( "\n" );
    }

    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpNvmGlobalData" dumps the NVM global data structure. For convenience, we
// also lost the header separately, followed by the global data.The header size
// portion is 12 bytes.
//
//----------------------------------------------------------------------------------------
void dumpNvmGlobalData( ) {

    printf( "NVM Global Data Dump: \n\n");

    printf( "Header: " );
    dumpNvmData( NVM_GLOBAL_DATA_OFS, 12, 8, false );
    printf( "\n" );

    uint32_t start = NVM_GLOBAL_DATA_OFS + offsetof( LcsGlobalDataMap, map );

    dumpNvmData( start, globalDataMap.nvmSize );
    printf( "\n" );
}

//----------------------------------------------------------------------------------------
// "dumpNvmRuntimeArea" dumps the entire NVM runtime area. This is a combination
// of all the NVM areas except the global data area.
//
//----------------------------------------------------------------------------------------
void dumpNvmRuntimeArea( ) {

    printf( "NVM Runtime Area Dump: \n\n" );
    dumpNvmData( NVM_MAP_STORAGE_START, NVM_RUNTIME_MAPS_SIZE, 8, true );
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

//----------------------------------------------------------------------------------------
// "printMemNodeMap" prints the MEM node map structure in a formatted way.
//
// ??? to be completed...
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// "printMemPortMap" prints the MEM port map structure in a formatted way.
//
// ??? to be completed...
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// "printMemTaskMap" prints the MEM task map structure in a formatted way.
//
// ??? to be completed...
//----------------------------------------------------------------------------------------
void printMemTaskMap( ) {

    printf( "Task Map (Size: %d, Hwm: %d): \n\n", 
            MAX_TASK_MAP_ENTRIES, taskMap.mapHwm );

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
// A little helper function to skip spaces in a string. We return a pointer to 
// the first non-space character.
//
//----------------------------------------------------------------------------------------
char *skipSpaces( char *s ) {

    while (( *s == ' ' ) || ( *s == '\t' )) s++;
    return s;
}

//----------------------------------------------------------------------------------------
// When a command is sending message to another node where a reply is expected,
// we need a callback function for handling those reply messages.
//
//----------------------------------------------------------------------------------------
uint8_t repMsgCallback ( uint16_t npId, 
                         uint16_t item, 
                         uint16_t arg1, 
                         uint16_t arg2, 
                         uint8_t ret, 
                         void *uData ) {

    printf( "REP callback: npId: 0x%4x, item: %d, arg1: %d, arg2: %d, ret: %d\n ",
            npId, item, arg1, arg2, ret);
    return( LCS_OK );
}

uint8_t fRepCallback( uint16_t npId, 
                      uint16_t item, 
                      uint16_t arg1, 
                      uint16_t arg2, 
                      uint8_t ret, 
                      void *uData ) {

    printf( "FREP callback: npId: 0x%4x, item: %d, arg1: %d, arg2: %d, ret: %d\n ",
            npId, item, arg1, arg2, ret );
    return( LCS_OK );
}

}; // namespace


//----------------------------------------------------------------------------------------
// Routines in LCS name space.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "C" switches a node to CFG mode. For a local node command, we construct the 
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
// "O" switches the nodes to OPS mode. For a local node command, we construct the 
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
// "g" handles the item GET command. If the node is our node, we call the 
// local access routines. Otherwise we send a message. Note that for a local 
//access the result is returned right away. For a remote access, we just send
// the message, the result is returned in a callback when the reply message is
// received.
//
//    <!g npId item>
//
//    npId      - the node/port Id.
//    item      - the node item to query.
//    arg       - the argument.
//
//----------------------------------------------------------------------------------------
void getItemCommand( char *s ) {

    int     npId    = 0;
    int     item    = 0;
    int     arg     = 0;
    uint8_t ret     = LCS_OK;

    if ( sscanf(  s, "%i %i %i %i", &npId, &item, &arg ) < 2 ) 
        return ( errArgList( ));

    uint16_t tmpNpId    = (uint16_t) npId;
    uint16_t tmpItem    = (uint16_t) item;
    uint16_t tmpArg     = (uint16_t) arg;

     if (( npId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {

        ret = getItem( tmpNpId, tmpItem, &tmpArg );
        if ( ret != LCS_OK ) errStat((char *) "Node GET error", ret );
        else printf( "Node: 0x%x, item: %d, arg1: 0x%x\n", 
                     tmpNpId, tmpItem, tmpArg );
    }
    else {

        ret = sendGetNode( buildNpId( nodeMap.nodeId, 0, 0 ), 
                           tmpNpId, 
                           tmpItem, 
                           repMsgCallback, 
                           nullptr );
        if ( ret != LCS_OK ) errStat((char *) "Remote Node GET error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "p" handles the item SET command. If the node is our node, we call the local
// access routines. Otherwise we send a message. Note that for a local access
// the result is returned right away. For a remote access, we just send the 
// message, the acknowledge is returned in a callback when the reply message 
// is received.
//
//    <!p npId item val>
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val       - the item value 1
//
//----------------------------------------------------------------------------------------
void setItemCommand( char *s ) {

    int     npId    = 0;
    int     item    = 0;
    int     val     = 0;
    uint8_t ret     = LCS_OK;

    if ( sscanf(  s, "%i %i %i %i", &npId, &item, &val ) < 3 ) 
        return ( errArgList( ));

    uint16_t tmpNpId    = (uint16_t) npId;
    uint16_t tmpItem    = (uint16_t) item;
    uint16_t tmpVal     = (uint16_t) val;

    printf ( "val: %d\n", val );

     if (( npId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {
     
        ret = setItem( tmpNpId, tmpItem, &tmpVal );
        if ( ret != LCS_OK ) errStat((char *) "Node SET error", ret );
        else printf( "OK\n" );
    }
    else {

        ret = sendSetNode(  buildNpId( nodeMap.nodeId, 0, 0 ),
                            tmpNpId, 
                            tmpItem, 
                            tmpVal,
                            repMsgCallback, 
                            nullptr );
        if ( ret != LCS_OK ) errStat((char *) "Remote Node SET error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "r" handles the item request command. If the node is our node, we call
// the local access routine and return the result right away. Otherwise we send
// a FREQ message.
//
//    r npId item [ val1 [ val2 ]]
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//

//----------------------------------------------------------------------------------------
void reqItemCommand( char *s ) {

    int     npId    = 0;
    int     item    = 0;
    int     val1    = 0;
    int     val2    = 0;
    uint8_t ret     = LCS_OK;

    if ( sscanf(  s, "%i %i %i %i", &npId, &item, &val1, &val2 ) < 2 ) 
        return ( errArgList( ));

    uint16_t tmpNpId    = (uint16_t) npId;
    uint16_t tmpItem    = (uint8_t)  item;
    uint16_t tmpVal1    = (uint16_t) val1;
    uint16_t tmpVal2    = (uint16_t) val2;

    if (( tmpNpId == 0 ) || ( nodeId( tmpNpId ) == nodeMap.nodeId )) {
     
        ret = reqItem( tmpNpId, tmpItem, &tmpVal1, &tmpVal2 );
        if ( ret != LCS_OK ) errStat((char *) "Item REQ error", ret );
        else printf( "Node: 0x%x, item: %d, val1: 0x%x, val2: 0x%x\n", 
                    tmpNpId, tmpItem, tmpVal1, tmpVal2 );
    }
    else {

        ret = sendFuncReqNode( buildNpId( nodeMap.nodeId, 0, 0 ),
                               tmpNpId,
                               item,
                               val1,
                               val2,
                               fRepCallback,
                               nullptr  );
        if ( ret != LCS_OK ) errStat((char *) "Remote Node FREQ error", ret );
    }
}

//----------------------------------------------------------------------------------------
// "e" will send an event. 
//
//    e npId eventId mode [ arg ]
//
//    npId      - the sending node / port Id
//    eventId   - the event Id
//    mode      - 0 - ON, 1 - OFF, 2 - DATA
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

    len = sscanf( s, "%i %i %i %i", &npId, &eventId, &mode, &arg );

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
// Although most commands in the LCS console interface can also send messages 
// to other nodes, not all messages are covered. This command sends any kind 
// of message, even undefined ones. 
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
// "s" lists status information. There are many display options. The option 
// argument specifies what part to display. 
//
//    s [ opt ]
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
            case 3:     dumpMemPortMap( );              break;
            case 4:     dumpMemEventMap( );             break;
            case 5:     dumpMemPortData( );             break;
            case 6:     dumpMemGlobalData( );           break;
            case 7:     dumpMemTaskMap( );              break;
            case 8:     dumpMemRuntimeArea( );          break;

            case 21:    dumpNvmHeader( );               break;
            case 22:    dumpNvmNodeMap( );              break;
            case 24:    dumpNvmEventMap( );             break;
            case 25:    dumpNvmPortData( );             break;
            case 26:    dumpNvmGlobalData( );           break;
            case 28:    dumpNvmRuntimeArea( );          break;

            case 42:    printMemNodeMap( );             break;
            case 43:    printMemPortMap( );             break;
            case 46:    printMemTaskMap( );             break;
            
            case 50:    listDevicesI2C( );              break;   
            case 51:    printResourceDescMap( &dMap );  break;
            case 52:    printResourceMap( );            break;

            default: printf( "Unknown help option, use '?' for help\n" );
        }
    } 
    else printSummary( );
}

//----------------------------------------------------------------------------------------
// "?" lists core library help information. We just list the available commands
// and a short description. 
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
    printf( "   " " -        21         - Node Header\n" );
    printf( "   " " -   2    22   42    - Node Map\n" );
    printf( "   " " -   3    23   43    - Port Map\n" );
    printf( "   " " -   4    24         - Event Map\n" );
    printf( "   " " -   5    25         - Port Data\n" );
    printf( "   " " -   6    26         - Global Data\n" );
    printf( "   " " -   7         46    - Task Map\n" );
    printf( "   " " -   8    28         - Runtime Area\n" );

    printf( "   " " -  50  - Scan I2C Devices\n" );
    printf( "   " " -  51  - CDC Resource Desc Map\n");
    printf( "   " " -  52  - CDC Resource Map\n");
}

//----------------------------------------------------------------------------------------
// "setupSerialCommand" initializes the serial interface. We use the PICO USB 
// as console IO. The CDC lib contains functions for reading and writing to the
// console.
//
//----------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

    return ( configureUsbIO( ));
}

//----------------------------------------------------------------------------------------
// "executeCommand" is the command handler. We decode the first character and 
// pass the rest of the command string to the actual handler routine. An unknown
// command is passed to the user callback if it is defined. Otherwise we print
// an error.
//
//----------------------------------------------------------------------------------------
static void executeCommand( char *commandBuf ) {

    char *cmd = skipSpaces( commandBuf );

    if ( *cmd == '\0' ) return;

    switch ( cmd[0] ) {

        case 'C': switchToConfigCommand(cmd + 1);     break;
        case 'O': switchToOperationsCommand(cmd + 1); break;

        case 'g': getItemCommand(cmd + 1);            break;
        case 'p': setItemCommand(cmd + 1);            break;
        case 'r': reqItemCommand(cmd + 1);            break;
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
// executor with simple syntax, originally used on the DCC++ world. Note that 
// this routine is called as part of the runtime loop. Consequently, it cannot
// not block for IO. The interface is designed in a way that it assembles the 
// character input when there are characters until a carriage return is received. 
// A command line can optionally consist of multiple commands separated by a 
// "/" character.
//
// Since we are pretty basic on a character by character basis, we add a bit
// of luxury and echo back what was typed and also process the backspace 
// character.
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
