//------------------------------------------------------------------------------------------------------------
//
// LCS Runtime - command line interface.
//
//------------------------------------------------------------------------------------------------------------
// Based on the Raspberry Pi PICO controller, the LCS node has an option to accept commands and display
// data via the USB interface. This is very handy for initial debugging and later troubleshooting. The
// command syntax is rather simple and adopted from the original DCC++ work. The key reason for adopting
// the DCC++ command syntax that for exmaple the JMRI community built tools that accept DCC++ commands.
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
//
//
//------------------------------------------------------------------------------------------------------------  
char  commandBuf [ MAX_COMMAND_LINE_SIZE ];

//------------------------------------------------------------------------------------------------------------  
// Debug and Trace support. Instead of conditional cimpilation, we will print debug messages based on the
// settoin of the debiug level.
//------------------------------------------------------------------------------------------------------------ 
uint8_t debugLevel = 0;

//------------------------------------------------------------------------------------------------------------
// "dumpMemData" lists the MEM data content of the storage area passed. The data is listed in 16-bit 
// quantities.
//
//------------------------------------------------------------------------------------------------------------
void dumpMemData( uint8_t *area, uint16_t len, uint8_t itemsPerLine = 8 ) {

  uint16_t  index   = 0;
  uint16_t  limit   = ( len + 1 ) / 2; 
  uint16_t  *ptr    = (uint16_t *) area;

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
void dumpNvmData( uint32_t start, uint16_t len, uint8_t itemsPerLine = 8 ) {

  uint16_t  index   = 0;
  uint16_t  limit   = ( len + 1 ) / 2; 
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
void dumpExtNvmData( uint8_t boardId, uint32_t start, uint16_t len, uint8_t itemsPerLine = 8 ) {

  uint16_t  index   = 0;
  uint16_t  limit   = ( len + 1 ) / 2; 
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
//------------------------------------------------------------------------------------------------------------
void dumpNodeMap( ) {

  printf( "MEM Node Map:" );
  dumpMemData((uint8_t *) &nodeMap, sizeof( LcsNodeMap ));
  printf( "\n" );
}

void dumpNodeData( ) {

  printf( "MEM Node Data:" );
  dumpMemData((uint8_t *) &nodeData.map, sizeof( LcsNodeData ));
  printf( "\n" );
}

void dumpPortMap( ) {

  printf( "MEM Port Map: ( Entry Size: %d)\n", sizeof( LcsPortMapEntry ));
  dumpMemData((uint8_t *) &portMap, sizeof( LcsPortMap ));
  printf( "\n" );
}

void dumpEventMap( ) {

  printf( "MEM Event Map (hwm: %d\n):", nodeMap.eventMapHwm );
  dumpMemData((uint8_t *) &eventMap, sizeof( LcsEventMap ));
  printf( "\n" );
}

void dumpPendingReqMap( ) {

  printf( "MEM Pending Req Map:\n " );
  dumpMemData((uint8_t *) pendingReqMap.map, MAX_PENDING_REQ_MAP_ENTRIES * sizeof( LcsPendingReqEntry ));
  printf( "\n" );
}

void dumpCallbackMap( ) {

  printf( "Callback Map:\n" );
  dumpMemData((uint8_t *) callbackMap.map, ( MAX_PORT_MAP_ENTRIES + 1 ) * sizeof( LcsCallbackMapEntry ));
  printf( "\n" );
}

void dumpTaskMap( ) {

  printf( "Task Map:\n" );
  dumpMemData((uint8_t *) taskMap.map, MAX_TASK_MAP_ENTRIES * sizeof( LcsPTaskMapEntry ));
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

  printf( "MEM Area Dump:\n" );
  dumpNodeMap( );
  dumpPortMap( );
  dumpEventMap( );
  dumpPendingReqMap( );
  dumpTaskMap( );
  dumpCallbackMap( );
  printf( "\n" );
}

void dumpNvmArea( ) {

  printf( "NVM Area Dump:\n" );
  dumpNvmData( 0, sizeof( LcsNodeMap ) + sizeof( LcsNodeData ) + sizeof ( LcsPortMap ) + sizeof( LcsEventMap ));
  printf( "\n" );
}

void dumpNvmUserArea( ) {

  printf( "NVM Area Dump:\n" );
  dumpNvmData( NVM_USER_MAP_START, 0x100 );  // ??? fix ...
  printf( "\n" );
}

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Routines in LCS name sppace.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "!c" switches the node to CFG mode. We construct the LCS_OP_CFG message payload data and invoke the msg
// handler for switching the node mode.
//
//    <!c>
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void switchToConfigCommand( ) {

  uint8_t msg[ 8 ] = { LCS_OP_CFG };
  handleMsgLcsMgt( msg );
}

//------------------------------------------------------------------------------------------------------------
// "!o" switches the nodes to OPS mode. We construct the LCS_OP_CFG message payload data and invoke the msg
// handler for switching the node mode.
//
//    <!o>
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void switchToOperationsCommand( ) {

  uint8_t msg[ 8 ] = { LCS_OP_OPS };
  handleMsgLcsMgt( msg );
}

//------------------------------------------------------------------------------------------------------------
// "!a" adds an eventId / portId to the event map. If the portId is omitted, every port on the node will be
// registered for the event.
//
//    <!a eventId [ portId ]>
//
//    eventId   -   the eventId.
//    portId    -   the portId for which the event is added.
//
//    returns: <!a ret>
//
//------------------------------------------------------------------------------------------------------------
void enterEventCommand( char *s ) {

  uint16_t  eventId = NIL_EVENT_ID;
  uint16_t   portId = NIL_PORT_ID;

  if ( sscanf( s, "%hu %hu ", &eventId, &portId ) < 1 ) return;

  int ret = nodeControl( portId, NPC_ADD_EVENT_MAP_ENTRY, eventId, portId );
  printf( "<!a %d >", ret );
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
  uint16_t  portId  = NIL_PORT_ID;

  if ( sscanf( s, "%hu %hu ", &eventId, &portId ) < 1 ) return;

  int ret = nodeControl( portId, NPC_DEL_EVENT_MAP_ENTRY, eventId, portId );
  printf( "<!r %d >", ret );
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
// "!e" simulates receiving an event and will invoke the event processing routine for this event. This command
// is quite helpful in testing event callbacks.
//
//    <!e mode nodeId eventId [ arg ] >
//
//    mode      - 0 - ON, 1 - OFF, 2 - DATA
//    node      - the sending node Id
//    eventId   - the sending node event Id
//    arg       - optional data argument for the event.
//
//    returns: none
//
//------------------------------------------------------------------------------------------------------------
void sendEventCommand( char *s ) {

  uint8_t     msg[ 8 ];
  uint16_t    nodeId  = NIL_NODE_ID;
  uint16_t    eventId = NIL_EVENT_ID;
  uint8_t     mode    = 0;
  uint16_t    arg     = 0;
  uint8_t     len     = 0;

  len = sscanf( s, "%hhu %hu %hu %hu", &mode, &nodeId, &eventId, &arg );

  if ( len < 3 ) return;

  msg[ 0 ] = 0;
  msg[ 1 ] = highByte( nodeId );
  msg[ 2 ] = lowByte( nodeId );
  msg[ 3 ] = highByte( eventId );
  msg[ 4 ] = lowByte( eventId );
  msg[ 5 ] = highByte( arg );
  msg[ 6 ] = lowByte( arg );
  msg[ 7 ] = 0;

  if (( mode == 0 ) || ( mode == 1 )) {

    msg[ 0 ] = (( mode == 0 ) ? LCS_OP_EVT_ON : LCS_OP_EVT_OFF );
    handleMsgEvent( msg );
  }
  else if (( mode == 2 ) && ( len == 4 )) {

    msg[ 0 ] = LCS_OP_EVT;
    handleMsgEvent( msg );
  }
}

//------------------------------------------------------------------------------------------------------------
// "!n" handles the local node query command.
//
//    <!n port item [ val1 [ val2 ]]>
//
//    port      - the port. ( 0 - node, 1 .. n - port )
//    item      - the node item to query, the result items will be listed in HEX format.
//    val1      - the argument 1 on input.
//    val2      - the argument 2 on input.
//
//    returns: <!n item val1 val2 ret>
//
//------------------------------------------------------------------------------------------------------------
void queryNodeCommand( char *s ) {

  uint16_t  portId  = NIL_PORT_ID;
  uint8_t   item    = 0;
  uint16_t  arg1    = 0;
  uint16_t  arg2    = 0;
  uint8_t   ret     = ALL_OK;

  if ( sscanf( s, "%hu %hu %hu %hu", &portId, &item, &arg1, &arg2  ) != 2 ) return;

  ret = nodeInfo( portId, item, &arg1, &arg2 );

  printf( "<!n %d|0x%x|0x%x|%d>", item, arg1, arg2, ret );
}

//------------------------------------------------------------------------------------------------------------
// "!N" handles the node control command.
//
//    <!N item [ val1 [ val2 ]]>
//
//    port      - the port. ( 0 - node, 1 .. n - port )
//    item      - the port item to control
//    val1      - the item value 1
//    val2      - the item value 2 ( optional )
//
//    returns: <!N item val1 val2 ret>
//
//------------------------------------------------------------------------------------------------------------
void controlNodeCommand( char *s ) {

  uint16_t  portId  = NIL_PORT_ID;
  uint8_t   item    = 0;
  uint16_t  val1    = 0;
  uint16_t  val2    = 0;

  if ( sscanf(  s, "%hu %hhu %hu %hu", &portId, &item, &val1, &val2 ) < 2 ) return;

  uint8_t ret = nodeControl( portId, item, val1, val2 );

  printf( "<!N %d|0x%x|0x%x|%d>", item, val1, val2, ret );
}

//------------------------------------------------------------------------------------------------------------
// "!B" broadcasts a LCS message.
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

      case 0:  printSummary( );    break;
      case 1:  dumpNodeMap( );     break;
      case 2:  dumpNodeData( );    break;
      case 3:  dumpPortMap( );     break;
      case 4:  dumpEventMap( );    break;
      case 6:  dumpTaskMap( );     break;
      case 7:  dumpCallbackMap( ); break;
      case 8:  dumpNvmArea( );     break;
      case 9:  dumpMemArea( );     break;
    //  case 10: dumpDrvData( );     break;

      default: printf( "<Unknown help option, use '?' for help>" );
    }
    } else printSummary( );
}

//------------------------------------------------------------------------------------------------------------
// "driver request" command.
//
//    <!D board item arg1 [ arg 2]>
//
//    board - the extension board the driver handles.
//    item  - the driver specific item which isthe requested operation.
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
// "!?" lists core library help information.
//
//    <?>
//
//    returns: a list of available commands for the core library
//
//------------------------------------------------------------------------------------------------------------
void listCoreLibHelpCommand( ) {

  printf( "\nCommands: \n" );
  printf( "<!c > - enter config mode\n" );
  printf( "<!o > - enter operations mode\n" );
  printf( "<!a eventId [ portId ] > - add an event to the event tab\n" );
  printf( "<!r eventId [ portId ] > - remove an event from the event tab\n" );
  printf( "<!f eventId [ portId ] > - search an event on the event tab\n" );
  printf( "<!e mode nodeId eventId [ arg ] > - simulate sending an event ( mode: 0 - ON, 1 - OFF, 2 - EVT\n" );
  printf( "<!n portId item > - list a node attribute\n" );
  printf( "<!N portId item val1 [ val2 ] > - sets a node attribute\n" );
  printf( "<!B byte1 [ byte2 ... byte8 ] > - broadcast a raw LCS message\n" );
  printf( "<!D board item [ arg1 [ arg2 ]] > - send a request to an extension board n\n" );

  printf( " < !s [ level ] > - list status, default is summary\n" );
  printf( "              " " -  0 - summary\n" );
  printf( "              " " -  1 - Node Map\n" );
  printf( "              " " -  2 - Node Data\n" );
  printf( "              " " -  3 - Port Map\n" );
  printf( "              " " -  4 - Event Map\n" );
  printf( "              " " -  5 - Attribute Map\n" );
  printf( "              " " -  6 - Ptask Map\n" );
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
// "handleSerialCommand" reads characters from the console. The command line syntax is modelled after the
// original DCC++ work. First character is a command, the rest are arguments. A command is bracketed by "<" 
// and ">". Once we encounter a closing ">" sign, the first character in the bracketed string is used to 
// branch to the appropriate command handler. The command interface routine only handles the LCS commands, 
// which do start with a "!" after the opening bracket. Anything else is passed to a command handler callback,
// if defined. The interface accepts more than one command, they are just a list of "<...>" characters.
// Note that this routine is called as part of the runtime loop. Consequently, it cannot not block for IO. 
// The interface is designed in a way that it assembles the character input when there are characters. Only
// when there is a valid "<...>" sequence assembled, the command handler is invoked. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t handleSerialCommand( ) {

  char c;

  while ( c = CDC::getConsoleChar( ) > 0 ) {

    switch( c ) {

      case '<': commandBuf[ 0 ] = 0; break;

      case '>': {

          if ( commandBuf[ 0 ] == '!' ) {

            switch ( commandBuf[ 1 ] ) {

              case 'c': switchToConfigCommand( );                 break;
              case 'o': switchToOperationsCommand( );             break;
              case 'a': enterEventCommand( commandBuf + 2 );      break;
              case 'r': removeEventCommand( commandBuf + 2 );     break;
              case 'f': findEventCommand( commandBuf + 2 );       break;
              case 'e': sendEventCommand( commandBuf + 2 );       break;
              case 'n': queryNodeCommand( commandBuf + 2 );       break;
              case 'N': controlNodeCommand( commandBuf + 2 );     break;
              case 'B': broadcastLcsMsgCommand( commandBuf + 2 ); break;
              case 'D': drvRequestCommand( commandBuf + 2 );      break;
              case 's': listStatusCommand( commandBuf + 2 );      break;
              case '?': listCoreLibHelpCommand( );                break;
              default: printf( "<Unknown !-command, use '!?' for help>" );
            }
          }
          else if ( commandBuf[ 0 ] == '$' ) {

            switch ( commandBuf[ 1 ] ) {

              case 's':   break; // list content...
              case 'r':   break; // r ofs len show an area by ofs / len
              case 'w':   break; // w ofs data 0 ... 7 write up to eight words to memory
              case 'c':   break; // basic consistency checks..
              case '?':   break;

              default: printf( "<Unknown $-command, use '$?' for help>" );
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
