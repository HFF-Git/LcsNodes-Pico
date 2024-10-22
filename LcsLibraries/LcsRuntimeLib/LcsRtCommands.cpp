//------------------------------------------------------------------------------------------------------------
//
// LCS Runtime - command line interface.
//
//------------------------------------------------------------------------------------------------------------
// Based on the Raspberry Pi PICO controller USB interface, the LCS node has an option to accept commands and
// display data. This interface is used for manual node and extension board configuration as well as debug
// and troubleshooting. The command syntax is rather simple and adopted from the original DCC++ work. The key
// reason for adopting the DCC++ command syntax that for example the JMRI community built tools that accept 
// DCC++ commands.
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
// "dumpMemData" lists the memory data content of the storage area passed. The data is listed in 16-bit 
// quantities.
//
//------------------------------------------------------------------------------------------------------------
void dumpMemData( uint16_t *area, uint16_t len, uint8_t itemsPerLine = 8 ) {

    uint16_t  index   = 0;
    uint16_t  limit   = ( len + 1 ) / 2; 
    uint16_t  *ptr    = area;

    while ( index < limit ) {

        printf( "0x%04x: ", index * sizeof( uint16_t ));

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            if ( index + i < limit ) printf( "0x%04x ", ptr[ index + i ] );
        }

        index += itemsPerLine;
        printf( "\n" );
    }
}

//------------------------------------------------------------------------------------------------------------
// List NVM storage data. We are passed the absolute byte offset into the NVM area and the length in bytes.
// The data is listed in 16-bit quantities.
//
//------------------------------------------------------------------------------------------------------------
void dumpNvmData( uint32_t start, uint32_t len, uint32_t itemsPerLine = 8 ) {

    uint32_t  limit = start + len;
    uint16_t  val   = 0;

    while ( start < limit ) {

        printf( "0x%08x: ", start );

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = ( start + ( i * sizeof(uint16_t)));

            if ( ofs < limit ) {

                rtNvmGetWord( ofs, &val );
                printf( "0x%04x ", val );
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

    uint32_t  limit = start + len;
    uint16_t  val   = 0;

    while ( start < limit ) {

        printf( "0x%08x: ", start );

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            uint32_t ofs = ( start + ( i * sizeof(uint16_t)));

            if ( ofs < limit ) {

                extNvmGetWord( boardId, ofs, &val );
                printf( "0x%04x ", val );
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
void dumpNodeMap( ) {

    printf( "MEM Node Map: \n" );
    dumpMemData((uint16_t *) &nodeMap, sizeof( LcsNodeMap ));
    printf( "\n" );
}

void dumpPortMap( ) {

    printf( "MEM Port Map (Size: %d, Hwm: %d): \n", nodeMap.portMapEntries, nodeMap.portMapHwm );
    dumpMemData((uint16_t *) &portMap.map, sizeof( LcsPortMap ));
    printf( "\n" );
}

void dumpNodeData( ) {

    printf( "MEM Node Data: \n" );
    dumpMemData((uint16_t *) &nodeData.map, sizeof( LcsNodeData ));
    printf( "\n" );
}

void dumpEventMap( ) {

    printf( "MEM Event Map (Size: %d, Hwm: %d): \n", nodeMap.eventMapEntries, nodeMap.eventMapHwm );
    dumpMemData((uint16_t *) &eventMap, sizeof( LcsEventMap ));
    printf( "\n" );
}

void dumpPendingReqMap( ) {

    printf( "MEM Pending Req Map: (Size: %d, Hwm: %d) \n", pendingReqMap.size, pendingReqMap.hwm );
    dumpMemData((uint16_t *) &pendingReqMap, sizeof( LcsPendingReqMap ));
    printf( "\n" );
}

void dumpCallbackMap( ) {

    printf( "Callback Map: \n" );
    dumpMemData((uint16_t *) &callbackMap, sizeof( LcsCallbackMap ));
    printf( "\n" );
}

void dumpTaskMap( ) {

    printf( "Task Map: (Size: %d, Hwm: %d) \n", nodeMap.taskMapEntries, nodeMap.taskMapHwm );
    dumpMemData((uint16_t *) &taskMap, sizeof( LcsTaskMap ));
    printf( "\n" );
}

void dumpDrvMap( ) {

    printf( "Driver Map: (Size: %d) \n", drvMap.size );
    dumpMemData((uint16_t *) &drvMap, sizeof( LcsDrvMap ));
    printf( "\n" );
}

void printSummary( ) {

    printf( "LCS Node: \"" );
    for ( uint8_t i = 0; i < MAX_NODE_NAME_SIZE; i++ ) {
 
        if ( nodeMap.name[ i ] != 0 ) printf( "%c", nodeMap.name[ i ] );
    }
     
    printf( "\"\n" );
    printf( "LCS Library Version: %d.%d\n", nodeMap.nodeSwVersion >> 8, nodeMap.nodeSwVersion & 0xFF );
}

void dumpMemArea( ) {

    printf( "MEM Area Dump: \n" );
    dumpNodeMap( );
    dumpPortMap( );
    dumpEventMap( );
    dumpPendingReqMap( );
    dumpTaskMap( );
    dumpCallbackMap( );
    dumpDrvMap( );
    printf( "\n" );
}

void dumpNvmRuntimeArea( ) {

    printf( "NVM Runtime Area Dump: \n" );
    dumpNvmData( 0, NVM_RUNTIME_AREA_SIZE );
    printf( "\n" );
}

void dumpNvmDrvData( uint16_t boardId  ) {

    printf( "NVM Driver Data: \n" );
    dumpExtNvmData( boardId, 0, extNvmGetSize( ));
    printf( "\n" );
}

void dumpNvmUserArea( ) {

    printf( "NVM Area Dump: \n" );
    dumpNvmData( NVM_USER_MAP_START, usrNvmGetSize( ));
    printf( "\n" );
}

void scanI2CBus( uint8_t sclPin, uint8_t sdaPin ) {

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

      printf( "Scanning NVM I2C Bus: slc:%d, sda: %d \n", cdcMap.cfg.NVM_I2C_SCL_PIN, cdcMap.cfg.NVM_I2C_SDA_PIN );
      scanI2CBus(  cdcMap.cfg.NVM_I2C_SCL_PIN, cdcMap.cfg.NVM_I2C_SDA_PIN );
      printf( "\n" );
    }

    if ( cdcMap.cfg.EXT_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) {

      printf( "Scanning EXT I2C Bus: slc:%d, sda: %d \n", cdcMap.cfg.EXT_I2C_SCL_PIN, cdcMap.cfg.EXT_I2C_SDA_PIN );
      scanI2CBus(  cdcMap.cfg.EXT_I2C_SCL_PIN, cdcMap.cfg.EXT_I2C_SDA_PIN );
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

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Routines in LCS name space.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "!c" switches a node to CFG mode. For a local node command, we construct the LCS_OP_CFG message payload
// data and invoke the msg handler for switching the node mode. For any other node, we will just send a LCS 
// message.
//
//    <!c [ npId ] >
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void switchToConfigCommand( char *s ) {

    uint16_t npId = 0;

    if ( sscanf( s, "%hu", &npId ) < 1 ) return;

    if ( nodeId( npId ) == 0 ) {

        uint8_t msg[ 8 ] = { LCS_OP_CFG };
        handleMsgLcsMgt( msg );
    }
    else {

        uint8_t ret = sendCfg( npId );
        printf( "<!c %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!o" switches the nodes to OPS mode. For a local node command, we construct the LCS_OP_OPS message payload
// data and invoke the msg handler for switching the node mode. For any other node, we will just send a LCS 
// message.
//
//    <!o [ npId ] >
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void switchToOperationsCommand( char *s ) {

    uint16_t npId = 0;

    if ( sscanf( s, "%hu", &npId ) < 1 ) return;

    if ( nodeId( npId ) == 0 ) {

        uint8_t msg[ 8 ] = { LCS_OP_OPS };
        handleMsgLcsMgt( msg );
    }
    else {

        uint8_t ret = sendOps( npId );
        printf( "<!o %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!a" adds an eventId / portId to the event map. If the portId is omitted, every port of the node will be 
// registered for the event. For a non-local npId we will send a message.
//
//      <!a npId eventId [ portId ]>
//
//      npId      - the node and port Id for which the event is added.
//      eventId   - the eventId.
//      portId    - the port number.
//
//    returns: <!a ret>
//
//------------------------------------------------------------------------------------------------------------
void enterEventCommand( char *s ) {

    uint16_t  npId      = 0;
    uint16_t  eventId   = NIL_EVENT_ID;
    uint16_t  portId    = NIL_PORT_ID;

    if ( sscanf( s, "%hu %hu %hu ", &npId, &eventId, &portId ) < 2 ) return;

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        int ret = nodeReq( nodeId( npId ), ITEM_ID_ADD_EVENT_MAP_ENTRY, &eventId, &portId );
        printf( "<!a %d>", ret );
    }
    else {

        uint8_t ret = sendReqNode( nodeId( npId ), ITEM_ID_ADD_EVENT_MAP_ENTRY, eventId, portId );
        printf( "<!a %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!r"  removes a eventId / portId combination from the event map. If the portId is omitted, all eventMap
// entries with the eventId are removed.
//
//      <!r npId eventId [ portId ]>
//
//      npId      - the node and port Id for which the event is added.
//      eventId   - the eventId.
//      portId    - the port number.
//
//      returns: <!r ret>
//
//------------------------------------------------------------------------------------------------------------
void removeEventCommand( char *s ) {

    uint16_t  eventId   = NIL_EVENT_ID;
    uint16_t  portId    = NIL_PORT_ID;
    uint16_t  npId      =  0;

    if ( sscanf( s, "%hu %hu %hu ", &npId, &eventId, &portId ) < 1 ) return;

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        int ret = nodeReq( npId, ITEM_ID_DEL_EVENT_MAP_ENTRY, &eventId, &portId );
        printf( "<!r %d>", ret );
    }
    else {

        uint8_t ret = sendReqNode( nodeMap.nodeId, ITEM_ID_DEL_EVENT_MAP_ENTRY, eventId, portId );
        printf( "<!r %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!f" searches the event map for the eventId / portId combination and returns the index if found. If the
// portId is omitted, the first event map entry with the matching eventId is returned. This is a local 
// command and cannot be called from a remote node.
//
//      <!f eventId [ portId ]>
//
//      eventId   - the eventId.
//      portId    - the port number.
//
//      returns: <!r ret> where "ret" is either the index of the event map entry or -1.
//
//------------------------------------------------------------------------------------------------------------
void findEventCommand( char *s ) {

    uint16_t  eventId   = NIL_EVENT_ID;
    uint16_t  portId    = NIL_PORT_ID;

    if ( sscanf( s, "%hu %hu ", &eventId, &portId ) < 1 ) return;

    int ret = searchEvent( eventId, portId );
    printf( "<!f %d>", ret );
}

//------------------------------------------------------------------------------------------------------------
// "!e" will send an event. We will broadcast a message and also simulates receiving an event on the local 
// node. Sending to ourselves is also quite useful for debugging event callback handlers.
//
//    <!e mode npId eventId [ arg ] >
//
//    mode      - 0 - ON, 1 - OFF, 2 - DATA
//    npId      - the sending node / port Id
//    eventId   - the sending node / port Id event Id
//    arg       - optional data argument for the event.
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void sendEventCommand( char *s ) {

    uint8_t     msg[ 8 ];
    uint16_t    npId    = NIL_NODE_ID;
    uint16_t    eventId = NIL_EVENT_ID;
    uint8_t     mode    = 0;
    uint16_t    arg     = 0;
    uint8_t     len     = 0;

    len = sscanf( s, "%hhu %hu %hu %hu", &mode, &npId, &eventId, &arg );

    if ( len < 3 ) return;

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

        sendEventOn( npId, eventId ); 
        handleMsgEvent( msg );
    }
    else if ( mode == 0 ) {

        msg[ 0 ] = LCS_OP_EVT_OFF;

        sendEventOn( npId, eventId ); 
        handleMsgEvent( msg );
    }
    else if ( mode == 2 ) {

        msg[ 0 ] = LCS_OP_EVT;
        sendEvent( npId, eventId, arg ); 
        handleMsgEvent( msg );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!g" handles the node/port attribute query command. If the node is our node, we call the local access 
// routines. Otherwise we send a message.
//
//    <!g npId item [ val1 [ val2 ]]>
//
//    npId      - the node/port Id.
//    item      - the node item to query, the result items will be listed in HEX format.
//    val1      - the argument 1 on input.
//    val2      - the argument 2 on input.
//
//    returns: <!g item val1 val2 ret>
//
//
//------------------------------------------------------------------------------------------------------------
void getNodeCommand( char *s ) {

    uint16_t  npId    = 0;
    uint8_t   item    = 0;
    uint16_t  arg1    = 0;
    uint16_t  arg2    = 0;
    uint8_t   ret     = ALL_OK;

    if ( sscanf( s, "%hu %hu %hu %hu", &npId, &item, &arg1, &arg2  ) != 2 ) return;

    if ( nodeId( npId ) == nodeMap.nodeId ) {

        ret = nodeGet( npId, item, &arg1, &arg2 );
        printf( "<!g 0x%x, %d 0x%x 0x%x %d>", npId, item, arg1, arg2, ret );
    }
    else {

        ret = sendGetNode( npId, item, arg1, arg2 );
        printf( "<!g %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!p" handles the node or port attribute value set command. If the node is out node, we call the local 
// access routines. Otherwise we send a message.
//
//    <!p npId item [ val1 [ val2 ]]>
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//
//    returns: <!p item val1 val2 ret>
//
//------------------------------------------------------------------------------------------------------------
void putNodeCommand( char *s ) {

    uint16_t  npId  = 0;
    uint8_t   item  = 0;
    uint16_t  val1  = 0;
    uint16_t  val2  = 0;
    uint8_t   ret   = ALL_OK;

    if ( sscanf(  s, "%hu %hhu %hu %hu", &npId, &item, &val1, &val2 ) < 2 ) return;

    if ( nodeId( npId ) == nodeMap.nodeId ) {
     
        ret = nodePut( npId, item, val1, val2 );
        printf( "<!p %d 0x%x 0x%x %d>", item, val1, val2, ret );
    }
    else {

        ret = sendSetNode( npId, item, val1, val2 );
        printf( "<!p %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!r" handles the node / port request command. If the node is out node, we call the local access routines.
// Otherwise we send a message.
//
//    <!r npId item [ val1 [ val2 ]]>
//
//    npId      - the node/port Id.
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//
//    returns: <!r item val1 val2 ret>
//
//------------------------------------------------------------------------------------------------------------
void reqNodeCommand( char *s ) {

    uint16_t  npId  = 0;
    uint8_t   item  = 0;
    uint16_t  val1  = 0;
    uint16_t  val2  = 0;
    uint8_t   ret   = ALL_OK;

    if ( sscanf(  s, "%hu %hhu %hu %hu", &npId, &item, &val1, &val2 ) < 2 ) return;

    if ( nodeId( npId ) == nodeMap.nodeId ) {
     
        ret = nodeReq( npId, item, &val1, &val2 );
        printf( "<!r %d 0x%x 0x%x %d>", item, val1, val2, ret );
    }
    else {

        ret = sendReqNode( npId, item, val1, val2 );
        printf( "<!r %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!B" broadcasts a LCS message. Mainly used for debugging purposes. Although most commands in the LCS
// console interface can also send messages to other nodes, not all messages are covered. This command sends
// any kind of message, even undefined ones. 
//
//    <!B byte1 [ byte2 ... byte8 ]>
//
//    byte1 .. byte8   - the packet data in hexadecimal
//
//    returns: CANBus Lib Status
//
//------------------------------------------------------------------------------------------------------------
void broadcastLcsMsgCommand( char *s ) {

    uint8_t b[ 8 ] = { 0 };
    uint8_t nBytes  = sscanf( s, "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                                b, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7 );

    if ( nBytes >= 1 && nBytes <= 8 ) {

        int ret = msgBus -> sendLcsMsg( b ); 

        printf( "<!B [ " );
        for ( int i = 0; i < 8; i++ ) printf( "0x%x ", b[ i ] );
        printf( "] : %d>", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!D" sends a GET request to a driver. The commands will typically work on the MEM image of the driver data. 
// We will use the same idea of item ranges for MEM and NVM, except that the NVM range will work only if the 
// extension board is write-enabled.
//
//    <!G board item >
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//    arg1  - the first argument to the driver.
//    arg2  - the optional second argument to the driver and also output from the driver.
//
//    returns:  <!G board item arg ret>
//
//------------------------------------------------------------------------------------------------------------
void drvGetCommand( char *s ) {

    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t arg      = 0;
    uint8_t  rStat    = 0;

    if ( sscanf( s, "%hhu %hhu", &boardId, &item ) < 2 ) return;

    rStat = drvGet( boardId, item, &arg );
                                  
    printf( "<!G %d %d %d %d>", boardId, item, arg, rStat );
}

//------------------------------------------------------------------------------------------------------------
// "!M" sends a PUT request to a driver. The commands will typically work on the MEM image of the driver data. 
// We will use the same idea of item ranges for MEM and NVM, except that the NVM range will work only if the 
// extension board is write-enabled.
//
//    <!M board item arg1 >
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//    arg   - the data argument to the driver.
//
//    returns:  <!M board item arg ret>
//
//------------------------------------------------------------------------------------------------------------
void drvPutCommand( char *s ) {
    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t arg      = 0;
    uint8_t  rStat    = 0;

    if ( sscanf( s, "%hhu %hhu %hu", &boardId, &item, &arg ) < 3 ) return;

    rStat = drvPut( boardId, item, arg );

    printf( "<!M %d %d %d %d>", boardId, item, arg, rStat );
}

//------------------------------------------------------------------------------------------------------------
// "!R" sends a REQ request to a driver.
//
//    <!R board item arg1 [ arg 2 ]>
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//    arg1  - the first argument to the driver.
//    arg2  - the optional second argument to the driver and also output from the driver.
//
//    returns:  <!D board item arg1 arg2 ret>
//
//------------------------------------------------------------------------------------------------------------
void drvReqCommand( char *s ) {

    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t arg1     = 0;
    uint16_t arg2     = 0;
    uint8_t  rStat    = 0;

    if ( sscanf( s, "%hhu %hhu %hu %hu", &boardId, &item, &arg1, &arg2 ) < 3 ) return;

    rStat = drvReq( boardId, item, &arg1, &arg2 );

    printf( "<!R %d %d %d %d %d>", boardId, item, arg1, arg2, rStat );
}

//------------------------------------------------------------------------------------------------------------
// "listStatus" list information. The level argument specifies the what and the detail level.
//
//    <!s [ level ]>
//
//    returns:  NONE.
//
//------------------------------------------------------------------------------------------------------------
void listStatusCommand( char *s ) {

    int level = 0;

    if ( sscanf( s, " %d", &level ) > 0 ) {

        switch ( level ) {

            case 0:     printSummary( );            break;
            case 1:     dumpNodeMap( );             break;
            case 2:     dumpPortMap( );             break;
            case 3:     dumpNodeData( );            break;
            case 4:     dumpEventMap( );            break;
            case 5:     dumpPendingReqMap( );       break;
            case 6:     dumpTaskMap( );             break;
            case 7:     dumpCallbackMap( );         break;
            case 8:     dumpDrvMap( );              break;
            case 10:    dumpMemArea( );             break;
            case 11:    dumpNvmRuntimeArea( );      break;
            case 20:    listDevicesI2C( );          break;   

            default: printf( "<Unknown help option, use '?' for help>" );
        }
    } 
    else printSummary( );
}

//------------------------------------------------------------------------------------------------------------
// "!?" lists core library help information. We just list the available commands and a short description.
//
//    <!?>
//
//    returns: a list of available commands for the core library
//
//------------------------------------------------------------------------------------------------------------
void listCoreLibHelpCommand( ) {

    printf( "\nCommands: \n" );
    printf( "<!c [ npId ] > - enter config mode\n" );
    printf( "<!o [ npId ] > - enter operations mode\n" );

    printf( "<!a npId eventId [ npId ] > - add an event to the event tab\n" );
    printf( "<!d npId eventId [ npId ] > - remove an event from the event tab\n" );
    printf( "<!e npId eventId mode [ arg ] > - simulate sending an event ( mode: 0 - ON, 1 - OFF, 2 - EVT\n" );
    printf( "<!f eventId [ npId ] > - search an event on the event tab\n" );
    
    printf( "<!g npId item > - gets a node attribute\n" );
    printf( "<!p npId item val1 [ val2 ] > - puts a node attribute\n" );
    printf( "<!r npId item val1 [ val2 ] > - request a node function\n" );

    printf( "<!B byte1 [ byte2 ... byte8 ] > - broadcast a raw LCS message\n" );

    printf( "<!G board item > - send a GET request to an extension board n\n" );
    printf( "<!P board item arg > - send a PUT request to an extension board n\n" );
    printf( "<!R board item [ arg1 [ arg2 ]] > - send a REQ request to an extension board n\n" );
   
    printf( " < !s [ level ] > - list status, default is summary\n" );
    printf( "              " " -  0  - summary\n" );
    printf( "              " " -  1  - Node Map\n" );
    printf( "              " " -  2  - Port Map\n" );
    printf( "              " " -  3  - Node Data\n" );
    printf( "              " " -  4  - Event Map\n" );
    printf( "              " " -  5  - Pending Request Map\n" );
    printf( "              " " -  6  - Task Map\n" );
    printf( "              " " -  7  - Callback Map\n" );
    printf( "              " " -  8  - Driver Map\n" );
    printf( "              " " - 10  - MEM Area\n" );
    printf( "              " " - 11  - NVM Area\n" );
    printf( "              " " - 20  - I2C Devices\n" );
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
// "handleSerialCommand" reads characters from the console. The command line syntax is modeled after the
// original DCC++ work. First character is a command, the rest are arguments. A command is bracketed by "<" 
// and ">". Once we encounter a closing ">" sign, the first characters in the bracketed string is used to 
// branch to the appropriate command handler. The command interface routine only handles the LCS commands, 
// which do start with a "!" followed by the command character after the opening bracket. Anything else is 
// passed to a command handler callback, if defined. The interface accepts more than one command, they are 
// just a list of "<...>" characters. Note that this routine is called as part of the runtime loop. 
// Consequently, it cannot not block for IO. The interface is designed in a way that it assembles the 
// character input when there are characters. Only when a valid "<...>" sequence assembled, the command 
// handler is invoked. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t handleSerialCommand( ) {

    char c;

    while (( c = CDC::getConsoleChar( )) > 0 ) {

        switch( c ) {

            case '<': commandBuf[ 0 ] = 0; break;

            case '>': {

                printf( "\n" );

                if ( commandBuf[ 0 ] == '!' ) {

                    switch ( commandBuf[ 1 ] ) {

                        case 'C': switchToConfigCommand( commandBuf + 2 );        break;
                        case 'O': switchToOperationsCommand( commandBuf + 2 );    break;
                        
                        case 'a': enterEventCommand( commandBuf + 2 );            break;
                        case 'd': removeEventCommand( commandBuf + 2 );           break;
                        case 'f': findEventCommand( commandBuf + 2 );             break;
                        case 'e': sendEventCommand( commandBuf + 2 );             break;
                        
                        case 'g': getNodeCommand( commandBuf + 2 );               break;
                        case 'p': putNodeCommand( commandBuf + 2 );               break;
                        case 'r': reqNodeCommand( commandBuf + 2 );               break;

                        case 'B': broadcastLcsMsgCommand( commandBuf + 2 );       break;

                        case 'G': drvGetCommand( commandBuf + 2 );                break;
                        case 'P': drvPutCommand( commandBuf + 2 );                break;
                        case 'R': drvReqCommand( commandBuf + 2 );                break;

                        case 's': listStatusCommand( commandBuf + 2 );            break;
                        case '?': listCoreLibHelpCommand( );                      break;

                        default: printf( "<Unknown !-command, use '!?' for help>" );
                    }
                }
                else if ( callbackMap.cmdLineCallback != nullptr ) {

                    callbackMap.cmdLineCallback( commandBuf );
                }
                else printf( "<Unknown command, use '?' for help>" );

            } break;

            default: if ( strlen( commandBuf ) < MAX_COMMAND_LINE_SIZE ) strncat( commandBuf, &c, 1 );
        }
    }

    return( ALL_OK );
}

}; // namespace LCS
