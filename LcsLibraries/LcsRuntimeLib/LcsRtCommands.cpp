//------------------------------------------------------------------------------------------------------------
//
// LCS Runtime - command line interface.
//
//------------------------------------------------------------------------------------------------------------
// Based on the Raspberry Pi PICO controller USB interface, the LCS node has an option to accept commands and
// display data. This interface is used for manual node and extension board configuration as well as debug
// and troubleshooting. Most commands are sensitive to the node/port ID. If there is another node than our 
// own node, specified with a zero node ID value, the commands is sent to the bus.
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

//-----------------------------------------------------------------------------------------------------------
// External declaration to global structures defined in "LcsRtSetup".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {
    
    extern uint16_t                 debugMask;
    extern LCS::LcsCdcDesc          cdcMap;
    extern LCS::LcsNodeMap          nodeMap;
    extern LCS::LcsNodeData         nodeData;
    extern LCS::LcsPortMap          portMap;
    extern LCS::LcsEventMap         eventMap;
    extern LCS::LcsCallbackMap      callbackMap;
    extern LCS::LcsTaskMap          taskMap;
    extern LCS::LcsPendingReqMap    pendingReqMap;
    extern LCS::LcsDrvFuncMap       drvFuncMap;
    extern LCS::LcsDrvMap           drvMap;
    extern LCS::LcsMsgBusCAN        *msgBus;
};

//------------------------------------------------------------------------------------------------------------
// Local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------  
// The command line buffer.
//
//------------------------------------------------------------------------------------------------------------  
char  commandBuf [ MAX_COMMAND_LINE_SIZE ];

//------------------------------------------------------------------------------------------------------------
// "dumpMemData" lists the memory data content of the storage area passed. The data is displayed in 16-bit 
// quantities.  Because the PICO uses little-endian format, ASCII characters may appear reversed when 
// interpreted directly.
//
//------------------------------------------------------------------------------------------------------------
void dumpMemData( uint16_t *area, uint16_t len, uint8_t itemsPerLine = 8, bool printAscii = false ) {

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

                    if ( isprint( ptr[ index + i ] >> 8  )) printf( "%c", ptr[ index + i ] >> 8 );
                    else                                    printf( "." );

                    if ( isprint( ptr[ index + i ] & 0xff )) printf( "%c ", ptr[ index + i ] & 0xff );
                    else                                     printf( ". " );
                }
            }
        }
       

        index += itemsPerLine;
        printf( "\n" );
    }
}

//------------------------------------------------------------------------------------------------------------
// List the NVM storage data. The function receives the absolute byte offset within the NVM area and the 
// length in bytes. The data is displayed in 16-bit quantities. Because the PICO uses little-endian format, 
// ASCII characters may appear reversed when interpreted directly.
//
//------------------------------------------------------------------------------------------------------------
void dumpNvmData( uint32_t start, uint32_t len, uint32_t itemsPerLine = 8, bool printAscii = false ) {

    uint8_t     rStat = ALL_OK;
    uint32_t    limit = start + len;
    uint16_t    val   = 0;

    while ( start < limit ) {

        printf( "0x%08x: ", start );

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = ( start + ( i * sizeof(uint16_t)));

            if ( ofs < limit ) {

                rStat = rtNvmGetWord( ofs, &val );
                if ( rStat == ALL_OK ) printf( "0x%04x ", val );
            }
        }

        if ( printAscii ) {

            if ( start + ( itemsPerLine * sizeof(uint16_t)) >= limit ) {

                int tmp = ( start + ( itemsPerLine * sizeof(uint16_t)) - limit ) / sizeof( uint16_t);
                for ( int i = 0; i < tmp; i++ ) printf( "       " );
            };

            printf( "  " );

            for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

                uint32_t ofs = start + ( i * sizeof(uint16_t));

                if ( ofs < limit ) {

                    rStat = rtNvmGetWord( ofs, &val );
                    if ( rStat == ALL_OK ) {

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

//------------------------------------------------------------------------------------------------------------
// List extension board NVM storage data. We are passed the absolute offset into the NVM area and the 
// length in bytes.
//
//------------------------------------------------------------------------------------------------------------
void dumpExtNvmData( uint8_t boardId, uint32_t start, uint32_t len, uint32_t itemsPerLine = 8 ) {

    uint8_t     rStat = ALL_OK;
    uint32_t    limit = start + len;
    uint16_t    val   = 0;

    while ( start < limit ) {

        printf( "0x%08x: ", start );

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = ( start + ( i * sizeof(uint16_t)));

            if ( ofs < limit ) {

                rStat = extNvmGetWord( boardId, ofs, &val );
                if ( rStat == ALL_OK ) printf( "0x%04x ", val );
            }
        }

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = ( start + ( i * sizeof(uint16_t)));

            if ( ofs < limit ) {

                rStat = extNvmGetWord( boardId, ofs, &val );
                if ( rStat == ALL_OK ) {
                    
                    if ( isprint( val >> 8  )) printf( "%c", val >> 8 );
                    else                       printf( "." );

                    if ( isprint( val & 0xff )) printf( "%c ", val & 0xFF );
                    else                        printf( ". " );
                }
            }
        }

        start = start + itemsPerLine * sizeof(uint16_t);
        printf( "\n" );
    }
}

//------------------------------------------------------------------------------------------------------------
// Routines to list contents of the various memory areas. Right now, we just dump out hex data. It would be 
// nice to show formatted data. Perhaps one day...
//
//------------------------------------------------------------------------------------------------------------
void printSummary( ) {

    printf( "LCS Node: \"" );
    for ( uint8_t i = 0; i < MAX_NODE_NAME_SIZE; i++ ) {
 
        if ( nodeMap.name[ i ] != 0 ) printf( "%c", nodeMap.name[ i ] );
    }
     
    printf( "\"\n" );
    printf( "LCS Library Version: %d.%d\n", nodeMap.nodeSwVersion >> 8, nodeMap.nodeSwVersion & 0xFF );
}

void dumpMemNodeMap( ) {

    printf( "MEM Node Map: \n\n" );
    dumpMemData((uint16_t *) &nodeMap, sizeof( LcsNodeMap ), 8, true);
    printf( "\n" );
}

void dumpMemCdcMap( ) {

    printf( "MEM CDC Map: \n\n" );
    // dumpMemData((uint16_t *) &nodeMap, sizeof( LcsNodeMap ));
    printf( "\n" );
}

void dumpMemPortMap( ) {

    printf( "MEM Port Map (Size: %d, Hwm: %d): \n\n", nodeMap.portMapEntries, nodeMap.portMapHwm );

    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        printf( "Port %d:\n", i + 1 );
        dumpMemData((uint16_t *) &portMap.map[ i ], sizeof( LcsPortMapEntry ), 8, true );
        printf( "\n" );
    }
}
 
void dumpMemNodeData( ) {

    printf( "MEM Node Data: \n\n" );

    for ( int i  = 0; i < MAX_NODE_DATA_BLOCKS; i++ ) {

        printf( "Port %d:\n", i );
        dumpMemData((uint16_t *) &nodeData.map[ i ], MAX_ATTR_MAP_ENTRIES * sizeof( uint16_t ));
        printf( "\n" );
    }
}

void dumpMemEventMap( ) {

    printf( "MEM Event Map (Size: %d, Hwm: %d): \n\n", nodeMap.eventMapEntries, nodeMap.eventMapHwm );
    dumpMemData((uint16_t *) &eventMap, sizeof( LcsEventMap ));
    printf( "\n" );
}

void dumpMemPendingReqMap( ) {

    printf( "MEM Pending Req Map: (Size: %d, Hwm: %d) \n\n", nodeMap.pendingMapEntries, nodeMap.pendingMapHwm );
    dumpMemData((uint16_t *) &pendingReqMap, sizeof( LcsPendingReqMap ));
    printf( "\n" );
}

void dumpMemCallbackMap( ) {

    printf( "MEM Callback Map: \n\n" );
    dumpMemData((uint16_t *) &callbackMap, sizeof( LcsCallbackMap ));
    printf( "\n" );
}

void dumpMemTaskMap( ) {

    printf( "MEM Task Map: (Size: %d, Hwm: %d) \n\n", nodeMap.taskMapEntries, nodeMap.taskMapHwm );
    dumpMemData((uint16_t *) &taskMap, sizeof( LcsTaskMap ));
    printf( "\n" );
}

void dumpMemDrvFuncMap( ) {

    printf( "MEM Driver Function Map: (Size: %d) \n\n", nodeMap.drvFuncMapEntries );

    for ( int i  = 0; i < MAX_DRV_TYPES; i++ ) {

        LcsDrvFuncEntry *entry = &drvFuncMap.map[ i ];
        printf( "%d: Type: %d, Func: %p\n", i, entry -> drvType, entry -> drvFunc );
    }

     printf( "\n" );
}

void dumpMemDrvMap( ) {

    printf( "MEM Driver Map: (Size: %d) \n\n", nodeMap.drvMapEntries );

    for ( int i  = 0; i < MAX_EXT_BOARDS; i++ ) {

        LcsDrvEntry *entry = &drvMap.map[ i ];

        printf( "Board %d: ( Flags: 0x%04x, LastErr: %d, Drv: %p\n", 
                i, entry -> flags, entry -> lastErr, entry -> drvFunc );
                
        dumpMemData(( uint16_t*) &drvMap.map[ i ].extBoard, sizeof( LcsDrvBoardDesc ), 8, true );
        printf( "\n" );
    }

     printf( "\n" );
}

void dumpMemRuntimeArea( ) {

    printf( "MEM Area Dump: \n\n" );
    dumpMemNodeMap( );
    dumpMemCdcMap( );
    dumpMemPortMap( );
    dumpMemEventMap( );
    dumpMemPendingReqMap( );
    dumpMemTaskMap( );
    dumpMemCallbackMap( );
    dumpMemDrvFuncMap( );
    dumpMemDrvMap( );
    printf( "\n" );
}

//------------------------------------------------------------------------------------------------------------
// Routines to list contents of the various NVM areas. Right now, we just dump out hex data. It would be 
// nice to show formatted data. Perhaps one day...
//
//------------------------------------------------------------------------------------------------------------
void dumpNvmNodeMap( ) {

    printf( "NVM Node Map Dump: \n\n" );
    dumpNvmData( NVM_NODE_MAP_START, sizeof( LcsNodeMap ), 8, true );
    printf( "\n" );
}

void dumpNvmCdcMap( ) {

    printf( "MEM CDC Map Dump: \n\n" );
    dumpNvmData( NVM_CDC_MAP_START, sizeof( CDC::CdcConfigDesc ));
    printf( "\n" );
}

void dumpNvmPortMap( ) {

    printf( "NVM Port Map Dump: \n\n" );
    
    for ( int i  = 0; i < MAX_PORT_MAP_ENTRIES; i++ ) {

        uint32_t ofs = NVM_PORT_MAP_START + ( i * sizeof( LcsPortMapEntry ));

        printf( "Port %d, NVM ofs: 0x%04x \n", i + 1, ofs );
        dumpNvmData( ofs, sizeof( LcsPortMapEntry ), 8, true );
        printf( "\n" );
    }

    printf( "\n" );
}

void dumpNvmNodeData( ) {

    printf( "NVM Port Map Dump: \n\n" );
    
    for ( int i  = 0; i < MAX_NODE_DATA_BLOCKS; i++ ) {

        uint32_t ofs = NVM_NODE_DATA_START + ( i * MAX_ATTR_MAP_ENTRIES  * sizeof( uint16_t ));

        printf( "Node data block: %d, NVM ofs: 0x%04x \n", i, ofs );
        dumpNvmData( ofs, MAX_ATTR_MAP_ENTRIES  * sizeof( uint16_t ));
        printf( "\n" );
    }

    printf( "\n" );
}

void dumpNvmEventMap( ) {

    printf( "NVM Node Event Dump: \n\n" );
    dumpNvmData( NVM_EVENT_MAP_START, sizeof( LcsEventMap ));
    printf( "\n" );
}

void dumpNvmRuntimeArea( ) {

    printf( "NVM Runtime Area Dump: \n\n" );
    dumpNvmData( 0, NVM_RUNTIME_AREA_SIZE , 8, true );
    printf( "\n" );
}

void dumpNvmDrvData( uint16_t boardId ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES ) {

        printf( "Invalid board ID\n" );
        return;
    }

    if ( drvMap.map[ boardId ].flags & BF_EXT_BOARD_PRESENT ) {

        printf( "NVM Driver Data( board: %d ): \n\n", boardId );
        dumpExtNvmData( boardId, 0, sizeof( LcsDrvBoardDesc ));
        printf( "\n" );
    }
    else printf( "No board found \n" );
}

void dumpNvmUserArea( ) {

    printf( "NVM Area Dump: \n\n" );
    dumpNvmData( NVM_USER_MAP_START, usrNvmGetSize( ), 8, true );
    printf( "\n" );
}

//------------------------------------------------------------------------------------------------------------
// "scanI2CBus" and "listDevicesI2C" are two routines that will list all chips found on the NVM and EXT bus.
//
//------------------------------------------------------------------------------------------------------------
void scanI2CBus( uint8_t sclPin ) {

    uint8_t rStat     = 0;
    uint8_t i2cAdr    = 0;
    uint8_t nDevices  = 0;
    uint8_t buf       = 0;

    for ( i2cAdr = 1; i2cAdr < 127; i2cAdr++ ) {

        rStat = CDC::i2cRead( sclPin, i2cAdr, &buf, 1 );
      
        if ( rStat == 0 ) {

            printf( "I2C device found at i2cAdr 0x%x\n", i2cAdr );
            nDevices ++;
        }
    }

    if ( nDevices == 0 )  printf( "No I2C devices found\n" );
    else                  printf( "Scan done\n" );
}

void listDevicesI2C( ) {

    if ( cdcMap.cfg.NVM_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) {

      printf( "Scanning NVM I2C Bus: scl:%d, sda: %d \n", cdcMap.cfg.NVM_I2C_SCL_PIN, cdcMap.cfg.NVM_I2C_SDA_PIN );
      scanI2CBus(  cdcMap.cfg.NVM_I2C_SCL_PIN );
      printf( "\n" );
    }

    if ( cdcMap.cfg.EXT_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) {

      printf( "Scanning EXT I2C Bus: scl:%d, sda: %d \n", cdcMap.cfg.EXT_I2C_SCL_PIN, cdcMap.cfg.EXT_I2C_SDA_PIN );
      scanI2CBus(  cdcMap.cfg.EXT_I2C_SCL_PIN );
      printf( "\n" );
    }
}

//------------------------------------------------------------------------------------------------------------
// Little helper functions.
//
//------------------------------------------------------------------------------------------------------------
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

uint8_t lowByte( uint16_t arg ) { 
    
    return( arg & 0xFF ); 
}

uint8_t highByte( uint16_t arg ) { 
    
    return( arg >> 8 ); 
}

//------------------------------------------------------------------------------------------------------------
// Helper routines for error status handling.
//
// ??? one day, combine all error strings in one routine and print them from there...
//------------------------------------------------------------------------------------------------------------
void errorArgList( ) {

    printf( "Argument list error, use \"?\" )for help\n" );
}

void errorStatusMsg( char *msg, uint8_t ret ) {

    printf( "Error: %s ( %d )\n", msg, ret );
}

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Routines in LCS name space.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "c" switches a node to CFG mode. For a local node command, we construct the LCS_OP_CFG message payload
// data and invoke the msg handler for switching the node mode. For any other node, we will just send a LCS 
// message.
//
//    c [ npId ]
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void switchToConfigCommand( char *s ) {

    uint16_t npId = 0;

    if ( sscanf( s, "%hu", &npId ) < 1 ) return( errorArgList( ));

    if ( nodeId( npId ) == 0 ) {

        uint8_t msg[ 8 ] = { LCS_OP_CFG };
        handleMsgLcsMgt( msg );
    }
    else {

        uint8_t ret = sendCfg( npId );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node send error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "o" switches the nodes to OPS mode. For a local node command, we construct the LCS_OP_OPS message payload
// data and invoke the msg handler for switching the node mode. For any other node, we will just send a LCS 
// message.
//
//    o [ npId ]
//
//------------------------------------------------------------------------------------------------------------
void switchToOperationsCommand( char *s ) {

    uint16_t npId = 0;

    if ( sscanf( s, "%hu", &npId ) < 1 ) return( errorArgList( ));

    if ( nodeId( npId ) == 0 ) {

        uint8_t msg[ 8 ] = { LCS_OP_OPS };
        handleMsgLcsMgt( msg );
    }
    else {

        uint8_t ret = sendOps( npId );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node send error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "a" adds an eventId / portId to the event map. If the portId is omitted, every port of the node will be 
// registered for the event. For a non-local npId we will send a message.
//
//      a npId eventId [ portId ]
//
//      npId      - the node and port Id for which the event is added.
//      eventId   - the eventId.
//      portId    - the port number.
//
//------------------------------------------------------------------------------------------------------------
void enterEventCommand( char *s ) {

    uint16_t  npId      = 0;
    uint16_t  eventId   = NIL_EVENT_ID;
    uint16_t  portId    = NIL_PORT_ID;

    if ( sscanf( s, "%hu %hu %hu ", &npId, &eventId, &portId ) < 2 ) return( errorArgList( ));

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        uint8_t ret = nodeReq( nodeId( npId ), ITEM_ID_ADD_EVENT_MAP_ENTRY, &eventId, &portId );
       if ( ret != ALL_OK ) errorStatusMsg((char *) "Node enter event error", ret );
    }
    else {

        uint8_t ret = sendReqNode( nodeId( npId ), ITEM_ID_ADD_EVENT_MAP_ENTRY, eventId, portId );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node send error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "r"  removes a eventId / portId combination from the event map. If the portId is omitted, all eventMap
// entries with the eventId are removed.
//
//      r npId eventId [ portId ]
//
//      npId      - the node and port Id for which the event is added.
//      eventId   - the eventId.
//      portId    - the port number.
//
//------------------------------------------------------------------------------------------------------------
void removeEventCommand( char *s ) {

    uint16_t  eventId   = NIL_EVENT_ID;
    uint16_t  portId    = NIL_PORT_ID;
    uint16_t  npId      =  0;

    if ( sscanf( s, "%hu %hu %hu ", &npId, &eventId, &portId ) < 1 ) return( errorArgList( ));

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        int ret = nodeReq( npId, ITEM_ID_DEL_EVENT_MAP_ENTRY, &eventId, &portId );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Node remove event error", ret );
    }
    else {

        uint8_t ret = sendReqNode( nodeMap.nodeId, ITEM_ID_DEL_EVENT_MAP_ENTRY, eventId, portId );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node send error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "f" searches the event map for the eventId / portId combination and returns the index if found. If the
// portId is omitted, the first event map entry with the matching eventId is returned. This is a local 
// command and cannot be called from a remote node.
//
//      f eventId [ portId ]
//
//      eventId   - the eventId.
//      portId    - the port number.
//
//------------------------------------------------------------------------------------------------------------
void findEventCommand( char *s ) {

    uint16_t  eventId   = NIL_EVENT_ID;
    uint16_t  portId    = NIL_PORT_ID;

    if ( sscanf( s, "%hu %hu ", &eventId, &portId ) < 1 ) return( errorArgList( ));

    int ret = searchEvent( eventId, portId );
    printf( "Event map index: %d", ret );
}

//------------------------------------------------------------------------------------------------------------
// "e" will send an event. We will broadcast a message and also simulates receiving an event on the local 
// node. Sending to ourselves is also quite useful for debugging event callback handlers.
//
//    e mode npId eventId [ arg ]
//
//    mode      - 0 - ON, 1 - OFF, 2 - DATA
//    npId      - the sending node / port Id
//    eventId   - the event Id
//    arg       - optional data argument for the event.
//
//------------------------------------------------------------------------------------------------------------
void sendEventCommand( char *s ) {

    uint8_t     msg[ 8 ]    = { };
    uint16_t    npId        = NIL_NODE_ID;
    uint16_t    eventId     = NIL_EVENT_ID;
    uint8_t     mode        = 0;
    uint16_t    arg         = 0;
    uint8_t     len         = 0;
    uint8_t     ret         = 0;

    len = sscanf( s, "%hhu %hu %hu %hu", &mode, &npId, &eventId, &arg );

    if ( len < 3 ) return( errorArgList( ));

    msg[ 0 ] = 0;
    msg[ 1 ] = highByte( npId );
    msg[ 2 ] = lowByte( npId );
    msg[ 3 ] = highByte( eventId );
    msg[ 4 ] = lowByte( eventId );
    msg[ 5 ] = highByte( arg );
    msg[ 6 ] = lowByte( arg );
    msg[ 7 ] = 0;

    if ( mode == 0 ) {

        msg[ 0 ] = LCS_OP_EVT_ON;
        ret = sendEventOn( npId, eventId ); 
    }
    else if ( mode == 0 ) {

        msg[ 0 ] = LCS_OP_EVT_OFF;
        ret = sendEventOn( npId, eventId ); 
    }
    else if ( mode == 2 ) {

        msg[ 0 ] = LCS_OP_EVT;
        ret = sendEvent( npId, eventId, arg ); 
    }

    if ( ret != ALL_OK ) errorStatusMsg((char *) "Send event error", ret );
}

//------------------------------------------------------------------------------------------------------------
// "g" handles the node/port attribute query command. If the node is our node, we call the local access 
// routines. Otherwise we send a message.
//
//    <!g npId item [ val1 [ val2 ]]>
//
//    npId      - the node/port Id.
//    item      - the node item to query, the result items will be listed in HEX format.
//    val1      - the argument 1 on input.
//    val2      - the argument 2 on input.
//
//------------------------------------------------------------------------------------------------------------
void getNodeCommand( char *s ) {

    uint16_t  npId    = 0;
    uint8_t   item    = 0;
    uint16_t  arg1    = 0;
    uint16_t  arg2    = 0;
    uint8_t   ret     = ALL_OK;

    if ( sscanf( s, "%hu %hhu %hu %hu", &npId, &item, &arg1, &arg2  ) < 2 ) return( errorArgList( ));

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        ret = nodeGet( npId, item, &arg1, &arg2 );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Node GET error", ret );
        else printf( "Node: 0x%x, item: %d, arg1: 0x%x, arg2: 0x%x\n", npId, item, arg1, arg2 );
    }
    else {

        ret = sendGetNode( npId, item, arg1, arg2 );
         if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node GET error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "p" handles the node or port attribute value set command. If the node is out node, we call the local 
// access routines. Otherwise we send a message.
//
//    <!p npId item [ val1 [ val2 ]]>
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//
//------------------------------------------------------------------------------------------------------------
void putNodeCommand( char *s ) {

    uint16_t  npId  = 0;
    uint8_t   item  = 0;
    uint16_t  val1  = 0;
    uint16_t  val2  = 0;
    uint8_t   ret   = ALL_OK;

    if ( sscanf(  s, "%hu %hhu %hu %hu", &npId, &item, &val1, &val2 ) < 2 ) return( errorArgList( ));

    if ( nodeId( npId ) == nodeMap.nodeId ) {
     
        ret = nodePut( npId, item, val1, val2 );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Node PUT error", ret );
        else printf( "Node: 0x%x, item: %d, val1: 0x%x, val2: 0x%x\n", npId, item, val1, val2 );
    }
    else {

        ret = sendSetNode( npId, item, val1, val2 );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node PUT error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "r" handles the node / port request command. If the node is out node, we call the local access routines.
// Otherwise we send a message.
//
//    r npId item [ val1 [ val2 ]]
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//
//------------------------------------------------------------------------------------------------------------
void reqNodeCommand( char *s ) {

    uint16_t  npId  = 0;
    uint8_t   item  = 0;
    uint16_t  val1  = 0;
    uint16_t  val2  = 0;
    uint8_t   ret   = ALL_OK;

    if ( sscanf(  s, "%hu %hhu %hu %hu", &npId, &item, &val1, &val2 ) < 2 ) return( errorArgList( ));

    if ( nodeId( npId ) == nodeMap.nodeId ) {
     
        ret = nodeReq( npId, item, &val1, &val2 );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Node REQ error", ret );
        else printf( "Node: 0x%x, item: %d, val1: 0x%x, val2: 0x%x\n", npId, item, val1, val2 );
    }
    else {

        ret = sendReqNode( npId, item, val1, val2 );
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Remote Node REQ error", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "B" broadcasts a LCS message. Mainly used for debugging purposes. Although most commands in the LCS
// console interface can also send messages to other nodes, not all messages are covered. This command sends
// any kind of message, even undefined ones. 
//
//    B byte1 [ byte2 ... byte8 ]
//
//    byte1 .. byte8   - the packet data in hexadecimal
//
//------------------------------------------------------------------------------------------------------------
void broadcastLcsMsgCommand( char *s ) {

    uint8_t b[ 8 ] = { 0 };
    uint8_t nBytes  = sscanf( s, "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                                b, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7 );

    if ( nBytes >= 1 && nBytes <= 8 ) {

        uint8_t ret = msgBus -> sendLcsMsg( b ); 
        if ( ret != ALL_OK ) errorStatusMsg((char *) "Can Bus send error", ret );
    }
    else errorArgList( );
}

//------------------------------------------------------------------------------------------------------------
// "G" sends a GET request to a driver. The commands will typically work on the MEM image of the driver data. 
// We will use the same idea of item ranges for MEM and NVM, except that the NVM range will work only if the 
// extension board is write-enabled.
//
//    G board item
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//
//------------------------------------------------------------------------------------------------------------
void drvGetCommand( char *s ) {

    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t arg      = 0;
    uint8_t  ret      = 0;

    if ( sscanf( s, "%hhu %hhu", &boardId, &item ) < 2 ) return( errorArgList( ));

    ret = drvGet( boardId, item, &arg );

    if ( ret != ALL_OK ) errorStatusMsg((char *) "Driver GET error", ret );
    else printf( "Board: %d, item: %d, arg: 0x%x\n", boardId, item, &arg );
}

//------------------------------------------------------------------------------------------------------------
// "P" sends a PUT request to a driver. The commands will typically work on the MEM image of the driver data. 
// We will use the same idea of item ranges for MEM and NVM, except that the NVM range will work only if the 
// extension board is write-enabled.
//
//    P board item arg
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//    arg   - the data argument to the driver.
//
//------------------------------------------------------------------------------------------------------------
void drvPutCommand( char *s ) {

    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t val      = 0;
    uint8_t  ret      = 0;

    if ( sscanf( s, "%hhu %hhu %hu", &boardId, &item, &val ) < 3 ) return( errorArgList( ));

    ret = drvPut( boardId, item, val );
    
    if ( ret != ALL_OK ) errorStatusMsg((char *) "Driver PUT error", ret );
    else printf( "Board: %d, item: %d, arg: 0x%x\n", boardId, item, &val );
}

//------------------------------------------------------------------------------------------------------------
// "R" sends a REQ request to a driver.
//
//    R board item arg1 [ arg 2 ]
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//    arg1  - the first argument to the driver.
//    arg2  - the optional second argument to the driver and also output from the driver.
//
//------------------------------------------------------------------------------------------------------------
void drvReqCommand( char *s ) {

    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t arg1     = 0;
    uint16_t arg2     = 0;
    uint8_t  ret      = 0;

    if ( sscanf( s, "%hhu %hhu %hu %hu", &boardId, &item, &arg1, &arg2 ) < 2 ) return( errorArgList( ));

    ret = drvReq( boardId, item, &arg1, &arg2 );

    if ( ret != ALL_OK ) errorStatusMsg((char *) "Driver REQ error", ret );
    else printf( "Board: %d, item: %d, arg1: 0x%x, arg2: 0x%x\n", boardId, item, &arg1, &arg2 );
}

//------------------------------------------------------------------------------------------------------------
// "s" lists status information. The level argument specifies the what and the detail level.
//
//    s [ level ]
//
//    returns:  NONE.
//
//------------------------------------------------------------------------------------------------------------
void listStatusCommand( char *s ) {

    int level = 0;

    if ( sscanf( s, " %d", &level ) > 0 ) {

        switch ( level ) {

            case 0:     printSummary( );            break;

            case 1:     dumpMemNodeMap( );          break;
            case 2:     dumpNvmCdcMap( );           break;
            case 3:     dumpMemPortMap( );          break;
            case 4:     dumpMemNodeData( );         break;
            case 5:     dumpMemEventMap( );         break;
            case 6:     dumpMemPendingReqMap( );    break;
            case 7:     dumpMemTaskMap( );          break;
            case 8:     dumpMemCallbackMap( );      break;
            case 9:     dumpMemDrvFuncMap( );       break;
            case 10:     dumpMemDrvMap( );           break;
            case 11:    dumpMemRuntimeArea( );      break;

            case 21:    dumpNvmNodeMap( );          break;
            case 22:    dumpNvmCdcMap( );           break;
            case 23:    dumpNvmPortMap( );          break;
            case 24:    dumpNvmNodeData( );         break;
            case 25:    dumpNvmEventMap( );         break;
            case 26:    dumpNvmRuntimeArea( );      break;

            case 30:    dumpNvmDrvData( 0 );        break;
            case 31:    dumpNvmDrvData( 1 );        break;
            case 32:    dumpNvmDrvData( 2 );        break;
            case 33:    dumpNvmDrvData( 3 );        break;

            case 40:    listDevicesI2C( );          break;   

            default: printf( "<Unknown help option, use '?' for help>" );
        }
    } 
    else printSummary( );
}

//------------------------------------------------------------------------------------------------------------
// "?" lists core library help information. We just list the available commands and a short description.
//
//    ?
//
//------------------------------------------------------------------------------------------------------------
void listCoreLibHelpCommand( ) {

    printf( "\nCommands: \n" );
    printf( "c [ npId ] - enter config mode\n" );
    printf( "o [ npId ] - enter operations mode\n" );

    printf( "a npId eventId [ npId ] - add an event to the event tab\n" );
    printf( "d npId eventId [ npId ] - remove an event from the event tab\n" );
    printf( "e npId eventId mode [ arg ] - simulate sending an event ( mode: 0 - ON, 1 - OFF, 2 - EVT\n" );
    printf( "f eventId [ npId ] - search an event on the event tab\n" );
    
    printf( "g npId item - gets a node attribute\n" );
    printf( "p npId item val1 [ val2 ] - puts a node attribute\n" );
    printf( "r npId item val1 [ val2 ] - request a node function\n" );

    printf( "B byte1 [ byte2 ... byte8 ] - broadcast a raw LCS message\n" );

    printf( "G board item - send a GET request to an extension board n\n" );
    printf( "P board item arg - send a PUT request to an extension board n\n" );
    printf( "R board item [ arg1 [ arg2 ]] - send a REQ request to an extension board n\n" );
   
    printf( "s [ level ] - list status, default is summary\n" );
    printf( "              " " -   0  - summary\n" );
    printf( "              " " -   1  - MEM Node Map\n" );
    printf( "              " " -   2  - MEM CDC Map\n" );
    printf( "              " " -   3  - MEM Port Map\n" );
    printf( "              " " -   4  - MEM Node Data\n" );
    printf( "              " " -   5  - MEM Event Map\n" );
    printf( "              " " -   6  - MEM Pending Request Map\n" );
    printf( "              " " -   7  - MEM Task Map\n" );
    printf( "              " " -   8  - MEM Callback Map\n" );
    printf( "              " " -   9  - MEM Driver Function Map\n" );
    printf( "              " " -  10  - MEM Driver Map\n" );
    printf( "              " " -  11  - MEM Runtime Area\n" );

    printf( "              " " -  21  - NVM Node Map\n" );
    printf( "              " " -  22  - NVM CDC Map\n" );
    printf( "              " " -  23  - NVM Port Map\n" );
    printf( "              " " -  24  - NVM Node Data\n" );
    printf( "              " " -  25  - NVM Event Map\n" );
    printf( "              " " -  26  - NVM Runtime Area\n" );

    printf( "              " " -  30  - NVM Extension Board 0\n" );
    printf( "              " " -  31  - NVM Extension Board 1\n" );
    printf( "              " " -  32  - NVM Extension Board 2\n" );
    printf( "              " " -  33  - NVM Extension Board 3\n" );

    printf( "              " " -  40  - I2C Devices\n" );
}

//------------------------------------------------------------------------------------------------------------
// "setupSerialCommand" initializes the serial interface. We use the PICO USB as console IO. The CDC lib
// contains functions for reading and writing to the console.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

    return ( CDC::configureConsoleIO( ));
}

//------------------------------------------------------------------------------------------------------------
// "handleSerialCommand" reads characters from the console. The command interpreter is a simple character
// based input. Note that this routine is called as part of the runtime loop. Consequently, it cannot not 
// block for IO. The interface is designed in a way that it assembles the character input when there are 
// characters until a carriage return is received. The first character is the command. If it is not a 
// command we know and there is a callback, the callback gets his chance to handle the input string.
// Since we are pretty basic on a character by character basis, we also add a bit of luxury and echo back 
// the  what was typed in as well as process the backspace character.
//
//------------------------------------------------------------------------------------------------------------
uint8_t handleSerialCommand( ) {

    char c;

    while (( c = CDC::getConsoleChar( )) > 0 ) {

        switch( c ) {

            case '\r': {

                printf( "\n" );

                if ( strlen( commandBuf) > 0 ) {

                    switch ( commandBuf[ 0 ] ) {

                        case 'C': switchToConfigCommand( commandBuf + 1 );        break;
                        case 'O': switchToOperationsCommand( commandBuf + 1 );    break;
                        
                        case 'a': enterEventCommand( commandBuf + 1 );          break;
                        case 'd': removeEventCommand( commandBuf + 1 );         break;
                        case 'f': findEventCommand( commandBuf + 1 );           break;
                        case 'e': sendEventCommand( commandBuf + 1 );           break;
                        
                        case 'g': getNodeCommand( commandBuf + 1 );             break;
                        case 'p': putNodeCommand( commandBuf + 1 );             break;
                        case 'r': reqNodeCommand( commandBuf + 1 );             break;

                        case 'B': broadcastLcsMsgCommand( commandBuf + 1 );     break;

                        case 'G': drvGetCommand( commandBuf + 1 );              break;
                        case 'P': drvPutCommand( commandBuf + 1 );              break;
                        case 'R': drvReqCommand( commandBuf + 1 );              break;

                        case 's': listStatusCommand( commandBuf + 1 );          break;
                        case '?': listCoreLibHelpCommand( );                    break;

                        default: {
                            
                            if ( callbackMap.cmdLineCallback != nullptr ) {

                                    callbackMap.cmdLineCallback( commandBuf );
                                }
                            else printf( "<Unknown command, use '?' for help>" );
                        }
                    }
                }
               
                commandBuf[ 0 ] = '\0';
                printf( "->");
            
            } break;

            case '\b': {

                printf( "\b \b" );
                if ( strlen( commandBuf ) > 0 ) commandBuf[ strlen( commandBuf ) - 1 ] = '\0';

            } break;

            default: {

                printf( "%c", c );
                if ( strlen( commandBuf ) < MAX_COMMAND_LINE_SIZE ) strncat( commandBuf, &c, 1 );
            }
        }
    }

    return( ALL_OK );
}

}; // namespace LCS
