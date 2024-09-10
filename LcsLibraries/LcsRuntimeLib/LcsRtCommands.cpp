//------------------------------------------------------------------------------------------------------------
//
// LCS Runtime - command line interface.
//
//------------------------------------------------------------------------------------------------------------
// Based on the Raspberry Pi PICO controller, the LCS node has an option to accept commands and display
// data via the USB interface. This is very handy for initial debugging and troubleshooting in the field. 
// The command syntax is rather simple and adopted from the original DCC++ work. The key reason for adopting
// the DCC++ command syntax that for example the JMRI community built tools that accept DCC++ commands.
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
// External data structures.
//
//------------------------------------------------------------------------------------------------------------
extern LCS::LcsCdcDesc          cdcMap;
extern LCS::LcsNodeMap          nodeMap;
extern LCS::LcsNodeData         nodeData;
extern LCS::LcsPortMap          portMap;
extern LCS::LcsEventMap         eventMap;
extern LCS::LcsCallbackMap      callbackMap;
extern LCS::LcsTaskMap          taskMap;
extern LCS::LcsPendingReqMap    pendingReqMap;
extern LCS::LcsMsgBusCAN        *msgBus;

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
// "dumpMemData" lists the MEM data content of the storage area passed. The data is listed in 16-bit 
// quantities.
//
//------------------------------------------------------------------------------------------------------------
void dumpMemData( uint16_t *area, uint16_t len, uint8_t itemsPerLine = 8 ) {

    uint16_t  index   = 0;
    uint16_t  limit   = ( len + 1 ) / 2; 
    uint16_t  *ptr    = area;

    while ( index < limit ) {

        printf( "0x%4x: ", index * sizeof( uint16_t ));

        for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

            if ( index + i < limit ) printf( "0x%4x ", ptr[ index + i ] );
        }

        index += itemsPerLine;
        printf( "\n" );
    }
}

//------------------------------------------------------------------------------------------------------------
// List NVM storage data. We are passed the absolute offset into the NVM area and the length in bytes.
// The data is listed in 16-bit quantities.
//
//------------------------------------------------------------------------------------------------------------
void dumpNvmData( uint32_t start, uint32_t len, uint32_t itemsPerLine = 8 ) {

  uint32_t  index   = 0;
  uint32_t  limit   = ( len + 1 ) / 2; 
  uint16_t  val     = 0;

  while ( index < limit ) {

    printf( "0x%8x: ", index * sizeof( uint16_t ) );

    for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

      if ( index + i < limit ) {

        rtNvmGetWord(( index + i ) * 2, &val );
        printf( "0x%4x ", val );
      }
    }

    printf( "\n" );
  }
}

//------------------------------------------------------------------------------------------------------------
// List extension board NVM storage data. We are passed the absolute offset into the NVM area and the 
// length in bytes.
//
//------------------------------------------------------------------------------------------------------------
void dumpExtNvmData( uint8_t boardId, uint32_t start, uint32_t len, uint32_t itemsPerLine = 8 ) {

  uint32_t  index   = 0;
  uint32_t  limit   = ( len + 1 ) / 2; 
  uint16_t  val     = 0;

  while ( index < limit ) {

    printf( "0x%8x: ", index * sizeof( uint16_t ) );

    for ( uint16_t i = 0; i < itemsPerLine; i++ ) {

      if ( index + i < limit ) {

        extNvmGetWord( boardId, ( index + i ) * 2, &val );
        printf( "0x%4x ", val );
      }
    }

    printf( "\n" );
  }
}

//------------------------------------------------------------------------------------------------------------
// Routines to list contents of the various memory areas.
//
// ??? general_ should we rather format them ? A lot of work, but much more readable...
// ??? downside: garbage data needs to be checked...
//------------------------------------------------------------------------------------------------------------
void dumpNodeMap( ) {

    printf( "MEM Node Map: " );
    dumpMemData((uint16_t *) &nodeMap, sizeof( LcsNodeMap ));
    printf( "\n" );
}

void dumpPortMap( ) {

    printf( "MEM Port Map: (Hwm: %d) ( Entry Size: %d)\n", nodeMap.portMapHwm, sizeof( LcsPortMapEntry ));
    dumpMemData((uint16_t *) &portMap.map, sizeof( LcsPortMap ));
    printf( "\n" );
}

void dumpNodeData( ) {

    printf( "MEM Node Data:" );
    dumpMemData((uint16_t *) &nodeData.map, sizeof( LcsNodeData ));
    printf( "\n" );
}

void dumpEventMap( ) {

    printf( "MEM Event Map (Hwm: %d\n):", nodeMap.eventMapHwm );
    dumpMemData((uint16_t *) &eventMap, sizeof( LcsEventMap ));
    printf( "\n" );
}

void dumpPendingReqMap( ) {

    printf( "MEM Pending Req Map: (flags: 0x%x)(Hwm: %d)\n ", pendingReqMap.flags, 0 );
    dumpMemData((uint16_t *) pendingReqMap.map, MAX_PENDING_REQ_MAP_ENTRIES * sizeof( LcsPendingReqEntry ));
    printf( "\n" );
}

void dumpCallbackMap( ) {

    printf( "Callback Map: \n" );
    dumpMemData((uint16_t *) &callbackMap, sizeof( LcsCallbackMap ));
    printf( "\n" );
}

void dumpTaskMap( ) {

    printf( "Task Map:\n" );
    dumpMemData((uint16_t *) taskMap.map, MAX_TASK_MAP_ENTRIES * sizeof( LcsPTaskMapEntry ));
    printf( "\n" );
}

void dumpDrvMap( ) {

    printf( "Driver Map: (flags: 0x%x)(Size: %d)\n ", 0, 0 );
 
    // ??? get from the board....
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

    printf( "MEM Area Dump:\n" );
    dumpNodeMap( );
    dumpPortMap( );
    dumpEventMap( );
    dumpPendingReqMap( );
    dumpTaskMap( );
    dumpCallbackMap( );
    dumpDrvMap( );
    printf( "\n" );
}

void dumpNvmArea( ) {

    printf( "NVM Area Dump:\n" );
    dumpNvmData( 0, sizeof( LcsNodeMap ) + sizeof( LcsNodeData ) + sizeof ( LcsPortMap ) + sizeof( LcsEventMap ));
    printf( "\n" );
}

void dumpNvmDrvData( uint16_t boardId  ) {

    printf( "NVM Driver Data:\n" );

    // ??? get from the board....

}

void dumpNvmUserArea( ) {

    printf( "NVM Area Dump:\n" );
    dumpNvmData( NVM_USER_MAP_START, 0x100 );  // ??? fix ...
    printf( "\n" );
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
// "!c" switches a node to CFG mode. For a local command, we construct the LCS_OP_CFG message payload data 
// and invoke the msg handler for switching the node mode. For any other node, we will just send a message
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
        printf( "<!o %d >", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!o" switches the nodes to OPS mode. We construct the LCS_OP_CFG message payload data and invoke the msg
// handler for switching the node mode.
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
        printf( "<!o %d >", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!a" adds an eventId / portId to the event map. If the npId is omitted, every port on the local node will 
// be registered for the event. For a non-local npId we will send a message.
//
//    <!a eventId [ npId ]>
//
//    eventId   -   the eventId.
//    npId      -   the node and port Id for which the event is added.
//
//    returns: <!a ret>
//
//------------------------------------------------------------------------------------------------------------
void enterEventCommand( char *s ) {

    uint16_t  eventId = NIL_EVENT_ID;
    uint16_t  npId    = NIL_PORT_ID;

    if ( sscanf( s, "%hu %hu ", &npId, &portId ) < 1 ) return;

    if ( nodeId( npId ) == 0 ) {

        int ret = nodeReq( npId, NPI_ADD_EVENT_MAP_ENTRY, &eventId, &npId );
        printf( "<!a %d >", ret );
    }
    else {

        uint8_t ret = sendReqNode( npId, NPI_ADD_EVENT_MAP_ENTRY, eventId, npId );
        printf( "<!a %d >", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!r"  removes a eventId / portId combination from the event map. If the portId is omitted, all eventMap
// entries with the eventId are removed.
//
//    <!r eventId [ portId ]>
//
//    eventId   -   the eventId.
//    portId    -   the portId for which the event is removed.
//
//    returns: <!r ret>
//
//------------------------------------------------------------------------------------------------------------
void removeEventCommand( char *s ) {

    uint16_t  eventId = NIL_EVENT_ID;
    uint16_t  npId    = 0;

    if ( sscanf( s, "%hu %hu ", &eventId, &portId ) < 1 ) return;

    if ( nodeId( npId ) == 0 ) {

        int ret = nodeReq( npId, NPI_DEL_EVENT_MAP_ENTRY, &eventId, &npId );
        printf( "<!a %d >", ret );
    }
    else {

        uint8_t ret = sendReqNode( npId, NPI_DEL_EVENT_MAP_ENTRY, eventId, npId );
        printf( "<!a %d >", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "!f" searches the event map for the eventId / portId combination and returns the index if found. If the
// portId is omitted, the first event map entry with the matching eventId is returned.
//
//    <!f eventId [ portId ]>
//
//    eventId   -   the eventId.
//    portId    -   the portId for which the event is removed.
//
//    returns: <!r ret> where "ret" is either the index of the event map entry or -1.
//
//------------------------------------------------------------------------------------------------------------
void findEventCommand( char *s ) {

    uint16_t  eventId = NIL_EVENT_ID;
    uint16_t  portId  = NIL_PORT_ID;

    if ( sscanf( s, "%hu %hu ", &eventId, &portId ) < 1 ) return;

    int ret = searchEvent( eventId, portId );
    printf( "<!f %d >", ret );
}

//------------------------------------------------------------------------------------------------------------
// "!e" will send an event. We will send a message and also simulates receiving an event on the local node.
// Sending to ourselves is also quite useful for debugging event callback handlers.
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

    len = sscanf( s, "%hhu %hu %hu %hu", &mode, &nodeId, &eventId, &arg );

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
// "!g" handles the node/port attribute query command. If the node is out node, we call the local access 
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
// ??? how is the reply displayed ?
//------------------------------------------------------------------------------------------------------------
void getNodeCommand( char *s ) {

    uint16_t  npId    = 0;
    uint8_t   item    = 0;
    uint16_t  arg1    = 0;
    uint16_t  arg2    = 0;
    uint8_t   ret     = ALL_OK;

    if ( sscanf( s, "%hu %hu %hu %hu", &npId, &item, &arg1, &arg2  ) != 2 ) return;

    if ( nodeId( npId ) == 0 ) {

        ret = nodeGet( npId, item, &arg1, &arg2 );
        printf( "<!g %d|0x%x|0x%x|%d>", item, arg1, arg2, ret );
    }
    else {

        ret = sendQryNode( npId, item, arg1, arg2 );
        printf( "<!g %d >", ret );
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

    if ( nodeId( npId ) == 0 ) {
     
        ret = nodePut( npId, item, val1, val2 );
        printf( "<!r %d|0x%x|0x%x|%d>", item, val1, val2, ret );
    }
    else {

        ret = sendSetNode( npId, item, val1, val2 );
        printf( "<!g %d >", ret );
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

    if ( nodeId( npId ) == 0 ) {
     
        ret = nodeReq( npId, item, &val1, &val2 );
        printf( "<!r %d|0x%x|0x%x|%d>", item, val1, val2, ret );
    }
    else {

        ret = sendReqNode( npId, item, val1, val2 );
        printf( "<!g %d >", ret );
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
        printf( "] : %d >", ret );
    }
}

//------------------------------------------------------------------------------------------------------------
// "driver request" command. A driver is the piece of code that interfaces with an extension board. There is
// actually only one routine for talking to an extension board. This command will call a driver with the 
// respective arguments. Note that ITEMs which are used to set values in the extension board NVM will only
// work when the board is write-enabled. 
//
//    <!D board item arg1 [ arg 2]>
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which is the requested operation.
//    arg1  - the first argument to the driver.
//    arg2  - the optional second argument to the driver and also output from the driver.
//
//    returns:  <!D board item arg1 arg 2 ret>
//
//------------------------------------------------------------------------------------------------------------
void drvRequestCommand( char *s ) {

    uint8_t  boardId  = 0;
    uint8_t  item     = 0;
    uint16_t arg1     = 0;
    uint16_t arg2     = 0;

    if ( sscanf( s, "%hhu %hhu %hu %hu", &boardId, &item, &arg1, &arg2 ) < 4 ) return;

    int ret = drvReq( boardId, item, arg1, &arg2 );
    printf( "<#c %d %d %d %d %d >", boardId, item, arg1, arg2, ret );
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

            case 0:  printSummary( );       break;
            case 1:  dumpNodeMap( );        break;
            case 2:  dumpPortMap( );        break;
            case 3:  dumpNodeData( );       break;
            case 4:  dumpEventMap( );       break;
            case 5:  dumpPendingReqMap( );  break;
            case 6:  dumpTaskMap( );        break;
            case 7:  dumpCallbackMap( );    break;
            case 8:  dumpNvmArea( );        break;
            case 9:  dumpMemArea( );        break;

            case 10: dumpDrvMap( );         break;

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

    printf( "<!a eventId [ npId ] > - add an event to the event tab\n" );
    printf( "<!d eventId [ npId ] > - remove an event from the event tab\n" );
    printf( "<!f eventId [ npId ] > - search an event on the event tab\n" );
    printf( "<!e npId eventId mode [ arg ] > - simulate sending an event ( mode: 0 - ON, 1 - OFF, 2 - EVT\n" );
    
    printf( "<!g npId item > - gets a node attribute\n" );
    printf( "<!p npId item val1 [ val2 ] > - puts a node attribute\n" );
    printf( "<!r npId item val1 [ val2 ] > - request a node function\n" );

    printf( "<!B byte1 [ byte2 ... byte8 ] > - broadcast a raw LCS message\n" );
    printf( "<!D board item [ arg1 [ arg2 ]] > - send a request to an extension board n\n" );

    printf( " < !s [ level ] > - list status, default is summary\n" );
    printf( "              " " -  0 - summary\n" );
    printf( "              " " -  1 - Node Map\n" );
    printf( "              " " -  2 - Port Map\n" );
    printf( "              " " -  3 - Node Data\n" );
    printf( "              " " -  4 - Event Map\n" );
    printf( "              " " -  5 - Pending Request Map\n" );
    printf( "              " " -  6 - Task Map\n" );
    printf( "              " " -  7 - Callback Map\n" );
    printf( "              " " -  8 - NVM Area\n" );
    printf( "              " " -  9 - MEM Area\n" );
    printf( "              " " - 10 - Driver Area\n" );
}

//------------------------------------------------------------------------------------------------------------
// "setupSerialCommand" initializes the serial interface. We use the PICO USB as console IO.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

    return ( CDC::configureConsoleIO( ));
}

//------------------------------------------------------------------------------------------------------------
// "handleSerialCommand" reads characters from the console. The command line syntax is modeled after the
// original DCC++ work. First character is a command, the rest are arguments. A command is bracketed by "<" 
// and ">". Once we encounter a closing ">" sign, the first character in the bracketed string is used to 
// branch to the appropriate command handler. The command interface routine only handles the LCS commands, 
// which do start with a "!" after the opening bracket. Anything else is passed to a command handler callback,
// if defined. The interface accepts more than one command, they are just a list of "<...>" characters.
// Note that this routine is called as part of the runtime loop. Consequently, it cannot not block for IO. 
// The interface is designed in a way that it assembles the character input when there are characters. Only
// when there is a valid "<...>" sequence assembled, the command handler is invoked. 
//
//
// ??? what do we do about ACK and ERR return messages ? we cannot wait for an ACK / ERR to arrive, so we
// just issue a command and return. There needs to be a callback that will just print out the return 
// result to the console output.
//
// ??? next issue: what if the user also wants a callback message callback ? We would need to get in the 
// middle and then invoke the user callback if there is any ... 
//------------------------------------------------------------------------------------------------------------
uint8_t handleSerialCommand( ) {

  char c;

  while ( c = CDC::getConsoleChar( ) > 0 ) {

    switch( c ) {

      case '<': commandBuf[ 0 ] = 0; break;

      case '>': {

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
              case 'D': drvRequestCommand( commandBuf + 2 );            break;
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
