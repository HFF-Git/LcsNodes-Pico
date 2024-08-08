//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - nodeInfo and nodeControl routines.
//
//------------------------------------------------------------------------------------------------------------
// The file contains the part of the LCS Runtime that implements the node and port info and control access
// routines. Both accept an item Id as the command and up to two arguments for the call.
//
// There are four main groups of item numbers. The first group is the reserved list of node and port items.
// Typically these items will invoke an internal function in the runtime. The second and third group refer
// to the node or port attributes. An attribute is simply a variable that can hold a 16-bit value. The item
// number ranges in each group refer to the same attributes. For example item 64 and item 128 refer to 
// attribute 0. The difference is that the second group looks at the attribute as a volatile memory value
// while the third item group looks at an attribute as a memory and non-volatile combination. A read will 
// first copy the non volatile attribute value to memory and then return it. A write will first update the
//  memory and then also write the data to the non-volatile place. The fourth group of item numbers are 
// entirely user defined and will result in the invocation of a callback function that implements whatever 
// is associated with the item.
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

//------------------------------------------------------------------------------------------------------------
// The external global data structures defined in the "LcsRtCore" file.
//
//------------------------------------------------------------------------------------------------------------
extern LCS::LcsCdcDesc              cdcMap;
extern LCS::LcsNodeMap              nodeMap;
extern LCS::LcsNodeData             nodeData;
extern LCS::LcsPortMap              portMap;
extern LCS::LcsCallbackMap          callbackMap;

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  using namespace LCS;

  //----------------------------------------------------------------------------------------------------------  
  // Debug and Trace support. Instead of conditional cimpilation, we will print debug messages based on the
  // settoin of the debiug level.
  //---------------------------------------------------------------------------------------------------------- 
  uint8_t debugLevel = 0;

  //----------------------------------------------------------------------------------------------------------
  // The node or port name cannot be set with a single LCS message. We will store the parts in this temporary
  // buffer and set the name when all parts are received.
  //
  //----------------------------------------------------------------------------------------------------------
  char tempName[ MAX_NODE_NAME_SIZE + 1 ] = { 0 };

  //----------------------------------------------------------------------------------------------------------
  // Utility routines for number range check.
  //
  //----------------------------------------------------------------------------------------------------------
  static inline bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
  }

  //----------------------------------------------------------------------------------------------------------
  // "readAttrMem" gets a value from the node or port attribute map in MEM. As an internal function, we expect
  // a valid block and item argument.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t readAttrMem( uint8_t block, uint8_t item, uint16_t *arg ) {

    *arg = nodeData.map[ block ][ item - LCS::NPI_ATTR_MEM_RANGE_START ];
    return ( LCS::ALL_OK );
  }

  //----------------------------------------------------------------------------------------------------------
  // "writeAttrMem" stores a value to a node or port attribute map in MEM. As an internal function, we expect
  // a valid block and item argument.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t writeAttrMem( uint8_t block, uint8_t item, uint16_t arg ) {

    nodeData.map[ block ][ item - LCS::NPC_ATTR_MEM_RANGE_START ] = arg;
    return ( LCS::ALL_OK );
  }

  //----------------------------------------------------------------------------------------------------------
  // "readAttrNvm" gets a value from the node or port NVM attribute map. We read the value from the respective
  // attribute map, store it in the memory counterpart and then return it. For the NVM access, the byte offset
  // into the storage needs to be computed. As an internal function, we expect a valid block and item argument.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t readAttrNvm( uint8_t block, uint8_t item, uint16_t *arg ) {

    uint16_t index  = item - LCS:: NPI_ATTR_NVM_RANGE_START;
    uint16_t ofs    = ( block * MAX_ATTR_MAP_ENTRIES ) + ( index  * sizeof( uint16_t ));
    uint8_t rStat   = rtNvmGetWord( ofs, &nodeData.map[ block ][ index ] );

    if ( rStat == ALL_OK ) *arg = nodeData.map[ block ][ index ];
    return ( rStat );
  }

  //----------------------------------------------------------------------------------------------------------
  // "writeAttrNvm" stores an value to the node or port NVM attribute map. We first update the MEM attribute
  // map and then write the value to NVM attribute map. For the NVM access, the byte offset into the storage
  // needs to be computed. As an internal function, we expect a valid block and item argument.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t writeAttrNvm( uint8_t block, uint8_t item, uint16_t arg ) {

    uint16_t index  = item - LCS:: NPI_ATTR_NVM_RANGE_START;
    uint16_t ofs    = ( block * MAX_ATTR_MAP_ENTRIES ) + ( index  * sizeof( uint16_t ));

    nodeData.map[ block ][ index ] = arg;
    return ( rtNvmPutWord( ofs, arg ));
  }

} // namespace

//------------------------------------------------------------------------------------------------------------
// Namespace LCS routines.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "nodeInfo" is the entry point to obtain information about the node. 


// There is the situation that the request
// came from another node. 


// This routine does however not distinguish between a local or remote call. The item
// number range is divided into several ranges.
//
//   0        -> NIL Item.
//   1 -  63  -> Reserved items
//  64 - 127  -> Attribute returned from MEM ( normal  case )
// 128 - 191  -> Attributes first copied from NVM to MEM and then returned.
// 192 - 255  -> user defined items passed to the callback.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nodeInfo( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

  #if DEBUG_ATTRIBUTES == 1
  printf( "nodeInfo: %d:%d", portId, item  );
  if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
  if ( arg2 != nullptr ) printf( ":%d\n", *arg2 ); else printf( "null" );
  #endif

  if ( portId > MAX_PORT_MAP_ENTRIES ) return ( ERR_INVALID_PORT_ID );

  if ( isInRangeU( item, NPI_NODE_USER_DEFINED_START, NPI_MAX_ITEMS )) {

    if ( callbackMap.map[ portId ].ctrlItemCallback != nullptr ) {

      return ( callbackMap.map[ portId ].ctrlItemCallback( portId, item, *arg1, *arg2 ));

    } else return ( ERR_INVALID_ITEM_ID );
  }
  else if ( isInRangeU( item, NPI_ATTR_MEM_RANGE_START, NPI_ATTR_MEM_RANGE_END )) {

    return ( readAttrMem( portId, item, arg1 ));
  }
  else if ( isInRangeU( item, NPI_ATTR_NVM_RANGE_START, NPI_ATTR_NVM_RANGE_END )) {

    return ( readAttrNvm( portId, item, arg1 ));
  }
  else {

    switch ( item ) {

      case NPI_GET_OPTIONS: {

          if ( arg1 != nullptr ) *arg1 = nodeMap. options;
          return ( ALL_OK );
        }

      case NPI_GET_NODE_UID: {

          if ( arg1 != nullptr ) *arg1 = nodeMap.nodeUID >> 16;
          if ( arg2 != nullptr ) *arg2 = nodeMap.nodeUID & 0xFFFF;
          return ( ALL_OK );
        }

      case NPI_GET_NODE_ID: {

          if ( arg1 != nullptr ) *arg1 = nodeMap.nodeId;
          return ( ALL_OK );
        }

      case NPI_GET_PORT_MAP_ENTRIES: {

          if ( arg1 != nullptr ) *arg1 = MAX_PORT_MAP_ENTRIES;
          return ( ALL_OK );
        }

      case NPI_GET_EVENT_MAP_ENTRIES: {

          if ( arg1 != nullptr ) *arg1 = MAX_EVENT_MAP_ENTRIES;
          return ( ALL_OK );
        }

      case NPI_GET_ATTR_MAP_ENTRIES: {

          if ( arg1 != nullptr ) *arg1 = MAX_ATTR_MAP_ENTRIES;
          return ( ALL_OK );
        }

      case NPI_GET_RESTART_COUNT: {

          if ( arg1 != nullptr ) *arg1 = nodeMap. restartCnt;
          return ( ALL_OK );
        }

      case NPI_GET_EVENT_MAP_ENTRY: {

          return ( getMemEmapEntry( *arg1, arg1, arg2 ));
        }

      case NPI_GET_NODE_TYPE:  {

          if ( arg1 != nullptr ) *arg1 = nodeMap. nodeType;
          return ( ALL_OK );
        }

      case NPI_GET_PORT_TYPE:  {

          if ( arg1 != nullptr ) *arg1 = portMap.map[ portId - 1 ].type;
          return ( ALL_OK );
        }

      case NPI_GET_NODE_FLAGS:  {

          if ( arg1 != nullptr ) *arg1 = nodeMap. flags;
          return ( ALL_OK );
        }

      case NPI_GET_PORT_FLAGS:  {

          if ( arg1 != nullptr ) *arg1 = portMap.map[ portId - 1 ].flags;
          return ( ALL_OK );
        }

      case NPI_GET_NODE_NAME_1: {

          if ( arg1 != nullptr ) *arg1 = ((uint16_t) ( nodeMap.name[ 0 ] << 8  ) | nodeMap.name[ 1 ] );
          if ( arg2 != nullptr ) *arg2 = ((uint16_t) ( nodeMap.name[ 2 ] << 8  ) | nodeMap.name[ 3 ] );
          return ( ALL_OK );
        }

      case NPI_GET_NODE_NAME_2: {

          if ( arg1 != nullptr ) *arg1 = ((uint16_t) ( nodeMap.name[ 4 ] << 8  ) | nodeMap.name[ 5 ] );
          if ( arg2 != nullptr ) *arg2 = ((uint16_t) ( nodeMap.name[ 6 ] << 8  ) | nodeMap.name[ 7 ] );
          return ( ALL_OK );
        }

      case NPI_GET_NODE_NAME_3: {

          if ( arg1 != nullptr ) *arg1 = ((uint16_t) ( nodeMap.name[ 8 ] << 8  ) | nodeMap.name[ 9 ] );
          if ( arg2 != nullptr ) *arg2 = ((uint16_t) ( nodeMap.name[ 10 ] << 8  ) | nodeMap.name[ 11 ] );
          return ( ALL_OK );
        }

      case NPI_GET_NODE_NAME_4: {

          if ( arg1 != nullptr ) *arg1 = ((uint16_t) ( nodeMap.name[ 12 ] << 8  ) | nodeMap.name[ 13 ] );
          if ( arg2 != nullptr ) *arg2 = ((uint16_t) ( nodeMap.name[ 14 ] << 8  ) | nodeMap.name[ 15 ] );
          return ( ALL_OK );
        }

      case NPI_GET_EVENT_DELAY_TICKS: {

          if (( arg1 != nullptr ) && ( portId != NIL_PORT_ID )) *arg1 = portMap.map[ portId - 1 ].eventDelayTime;
          return ( ALL_OK );
        }

      default: return ( ERR_INVALID_NODE_INFO_ITEM );
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// "nodeControl" is the entry point to set information for the node and ports. There is the situation that the
// request came from another node. This routine does howeber not distinguish between a local or remote call.
// The item number range is divided into several ranges.
//
//   0        -> NIL Item.
//   1 - 63   -> Reserved items
//  64 - 127  -> Attribute returned from MEM ( normal  case )
// 128 - 191  -> Attributes first copied from NVM to MEM and then returned.
// 192 - 255  -> user defined items just passed to the callback.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nodeControl( uint8_t portId, uint8_t item, uint16_t val1, uint16_t val2 ) {

  #if DEBUG_ATTRIBUTES == 1
  printf( "nodeControl: %d:%d:%d:%d\n", portId, item, val1, val2  );
  #endif

  if ( portId > MAX_PORT_MAP_ENTRIES ) return ( ERR_INVALID_PORT_ID );

  if ( isInRangeU( item, NPC_NODE_USER_DEFINED_START, NPC_MAX_ITEMS )) {

    if ( callbackMap.map[ portId ].ctrlItemCallback != nullptr ) {

      return ( callbackMap.map[ portId ].ctrlItemCallback( portId, item, val1, val2 ));

    } else return ( ERR_INVALID_ITEM_ID );
  }
  else if ( isInRangeU( item, NPC_ATTR_MEM_RANGE_START, NPC_ATTR_MEM_RANGE_END )) {

    return ( writeAttrMem( portId, item, val1 ));
  }
  else if ( isInRangeU( item, NPC_ATTR_NVM_RANGE_START, NPC_ATTR_NVM_RANGE_END )) {

    return ( writeAttrNvm( portId, item, val1 ));
  }
  else if ( isInRangeU( item, NPI_NODE_MAP_RANGE_START, NPI_NODE_MAP_RANGE_END )) {

    switch ( item ) {

      case NPC_SET_READY_LED: {

          return ( CDC::writeDio( cdcMap.cfg.READY_LED_PIN, val1 ));
        }

      case NPC_SET_ACTIVITY_LED: {

          return ( CDC::writeDio( cdcMap.cfg.ACTIVE_LED_PIN, val1 ));
        }

      case NPC_TOGGLE_READY_LED:  {

          return ( CDC::toggleDio( cdcMap.cfg.READY_LED_PIN ));
        }

      case NPC_TOGGLE_ACTIVITY_LED: {

          return ( CDC::toggleDio( cdcMap.cfg.ACTIVE_LED_PIN ));
        }

      case NPC_BLINK_READY_LED:
      case NPC_BLINK_ACTIVITY_LED: {

          return ( ERR_NOT_IMPLEMENTED );
        }

      case NPC_RESET_NODE: {

          return ( resetNode( ));
        }

      case NPC_ADD_EVENT_MAP_ENTRY: {

          return ( addEvent( val1, val2 & 0xFF ));
        }

      case NPC_DEL_EVENT_MAP_ENTRY: {

          return ( removeEvent( val1, val2 & 0xFF ));
        }

      case NPC_SET_NODE_ID: {

          if ( isInRangeU( val1, MIN_NODE_ID, MAX_NODE_ID )) {

            nodeMap.nodeId = val1;
            rtNvmPutBytes( offsetof( LcsNodeMap, nodeId ), (uint8_t *) &nodeMap.nodeId, sizeof( uint16_t ));
            return ( ALL_OK );

          }
          else return ( ERR_INVALID_NODE_ID );
        }

      case NPC_SET_NODE_NAME_1: {

          tempName[ 0 ] = highByte( val1 );
          tempName[ 1 ] = lowByte( val1 );
          tempName[ 2 ] = highByte( val2 );
          tempName[ 3 ] = lowByte( val2 );

          memcpy((uint8_t *) nodeMap.name, (uint8_t *)tempName, MAX_NODE_NAME_SIZE );
          rtNvmPutBytes( offsetof( LcsNodeMap, name ), (uint8_t *)tempName, MAX_NODE_NAME_SIZE );
          return ( ALL_OK );
        }

      case NPC_SET_NODE_NAME_2: {

          tempName[ 4 ]   = highByte( val1 );
          tempName[ 5 ]   = lowByte( val1 );
          tempName[ 6 ]   = highByte( val2 );
          tempName[ 7 ]   = lowByte( val2 );
          return ( ALL_OK );
        }

      case NPC_SET_NODE_NAME_3: {

          tempName[ 8 ]   = highByte( val1 );
          tempName[ 9 ]   = lowByte( val1 );
          tempName[ 10 ]  = highByte( val2 );
          tempName[ 11 ]  = lowByte( val2 );
          return ( ALL_OK );
        }

      case NPC_SET_NODE_NAME_4: {

          memset( tempName, 0, MAX_NODE_NAME_SIZE );
          tempName[ 12 ]  = highByte( val1 );
          tempName[ 13 ]  = lowByte( val1 );
          tempName[ 14 ]  = highByte( val2 );
          tempName[ 15 ]  = lowByte( val2 );
          return ( ALL_OK );
        }

      case NPC_SET_NODE_TYPE: {

          nodeMap.nodeType = lowByte( val1 );
          rtNvmPutWord( offsetof( LcsNodeMap, nodeType ), nodeMap.nodeType );
          return ( ALL_OK );
        }

      case NPC_SET_PORT_TYPE: {

          if ( ! isInRangeU( portId, 1, MAX_PORT_MAP_ENTRIES )) return ( ERR_INVALID_PORT_ID );

          portMap.map[ portId - 1 ].type = lowByte( val1 );

          uint16_t ofs = offsetof( LcsPortMap, map ) + (( portId - 1 ) * sizeof( LcsPortMapEntry )) +
                         offsetof( LcsPortMapEntry, type );

          return ( rtNvmPutWord( ofs= portMap.map[ portId - 1 ].type, false ));
        }

      case NPC_ENABLE_EVENT_PROCESSING: {

          if ( ! isInRangeU( portId, 1, MAX_PORT_MAP_ENTRIES )) return ( ERR_INVALID_PORT_ID );

          if ( val1 ) portMap.map[ portId - 1 ].flags |= PF_PORT_EVENT_HANDLING_ENABLED;
          else        portMap.map[ portId - 1 ].flags &= ~ PF_PORT_EVENT_HANDLING_ENABLED;

          return ( ALL_OK );
        }

      case NPC_SET_EVENT_DELAY_TICKS: {

          if ( ! isInRangeU( portId, 1, MAX_PORT_MAP_ENTRIES )) return ( ERR_INVALID_PORT_ID );

          portMap.map[ portId - 1 ].eventDelayTime = val1;

          uint16_t ofs = offsetof( LcsPortMap, map ) + (( portId - 1 ) * sizeof( LcsPortMapEntry )) +
                         offsetof( LcsPortMapEntry, eventDelayTime );

          return ( rtNvmPutWord( ofs, val1 ));
        }

      default: return ( ERR_INVALID_NODE_CTRL_ITEM );
    }
  }
  else return ( ERR_INVALID_ITEM_ID );
}

}; // namespace LCS
