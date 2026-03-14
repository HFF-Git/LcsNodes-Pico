//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime items
//
//----------------------------------------------------------------------------------------
// A key concept in the LCS runtime is the idea of items. An item is an entity
// such as a data attribute of a port or a function callback related to an item.
// The item itself is a number. items are organized in ranges. Item 1 to 63 are
// reserved for runtime library attribute and functions, items 64 to 127 are 
// reserved for driver functions and items 128 to 255 are user defined items for
// attributes and function.
// 
// This file contains the LCS runtime routines that implement node attribute and
// function access. There are three routines that allow to manipulate node and
// port data as well as issue requests to a node or port. The "npId" will indicate
// which node and port the call refers to. The node portion is ignored, as the 
// calls in this module always refer to the local node.
//
// In addition there are extended attributes for the node. They are also indexed 
// by an item number. Item number 256 to NN are referring to them. "NN" depends
// on the actual size of the NVM on the board. 
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime items
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
// External declaration to global structures and routines in other files.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    using namespace CDC;

    extern uint16_t         debugMask;
    extern uint16_t         runtimeOptions;
    extern uint16_t         firmwareOptions;
    
    extern LcsNodeMap       nodeMap;
    extern LcsNodeData      nodeData;
    extern LcsPortMap       portMap;
    extern LcsEventMap      eventMap;

    extern uint8_t          syncEventMapToMem( );
    extern uint8_t          syncEventMapToNvm( );
    extern uint8_t          setEventMask( uint16_t eventId, uint16_t eventMask );
    extern uint8_t          removeEventMask( uint16_t eventId );
    extern int              searchEvent( uint16_t eventId );

    extern uint8_t          getMemEmapEntry( uint16_t index, 
                                             uint16_t *eventId, 
                                             uint16_t *eventMask );
    
    extern uint8_t          rtNvmPutWord( uint32_t ofs, uint16_t word );
    extern uint8_t          rtNvmGetWord( uint32_t ofs, uint16_t *word );

    extern uint8_t          rtNvmPutBytes( uint32_t ofs, 
                                           uint8_t *buf, 
                                           uint32_t len );
};

//----------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------
// "debugEnabled" and "retStat" are the debug support routines. We can easily 
// check whether debug is enabled at all. The return status routine will print 
// out a return status message when debugging is enabled. The macro "RET_STAT" 
// is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool itemDebugEnabled(  ) {

    return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_ITEMS )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( itemDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// "readAttrMem" gets a value from the node or port attribute map in MEM. As an
// internal function, we expect validated arguments. The "block" argument will 
// refer to the node and port data attributes. 
//
//----------------------------------------------------------------------------------------
uint8_t readAttrMem( uint16_t block, uint16_t item, uint16_t *arg ) {

    *arg = nodeData.map[ block ][ item - IR_USER_RANGE_START ];
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// "writeAttrMem" stores a value to a node or port attribute map in MEM. As an 
// internal function, we expect validated arguments. The "block" argument will
// refer to the node and port data attributes. 
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrMem( uint16_t block, uint16_t item, uint16_t arg ) {

    nodeData.map[ block ][ item - IR_USER_RANGE_START ] = arg;
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// "readAttrNvm" gets an attribute from the NVM storage. We read the value 
// from the NVM area. If successful, we also store it in the MEM counterpart 
// and then return it. This ensures that NVM and MEM are always in sync when 
//accessing NVM. For NVM access, the byte offset into the storage needs to 
// be computed. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrNvm( uint16_t block, uint16_t item, uint16_t *arg ) {

    uint16_t index  = item - IR_USER_RANGE_START;
    uint16_t ofs    = NVM_NODE_DATA_OFS + offsetof( LcsNodeData, map ) + 
        (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

    uint8_t rStat = rtNvmGetWord( ofs, arg );
    if ( rStat == NO_ERR ) rStat = writeAttrMem( block, item, *arg );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "writeAttrNvm" stores an attribute to the NVM storage. If the update is 
// successful, we also update the corresponding MEM attribute. This ensures 
// that NVM and MEM are always in sync when accessing the NVM. For the NVM 
// access, the byte offset into the storage needs to be computed. As an internal
// function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrNvm( uint16_t block, uint16_t item, uint16_t arg ) {

    uint16_t index  = item - IR_USER_RANGE_START;
    uint16_t ofs    = NVM_NODE_DATA_OFS + offsetof( LcsNodeData, map ) + 
        (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

    uint8_t rStat = rtNvmPutWord( ofs, arg );
    if ( rStat == NO_ERR ) rStat = writeAttrMem( block, item, arg );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "syncAttrToMem" will copy the NVM attribute value to the MEM counterpart. 
// All we do is just reading the NVM value to a dummy variable. As an internal 
// function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToMem( uint16_t block, uint16_t item ) {

    uint16_t arg = 0;
    return ( readAttrNvm( block, item, &arg ));
}

//----------------------------------------------------------------------------------------
// "syncAttrToNvm" will take the MEM attribute value of an item and writes it 
// to the NVM counterpart. As an internal function, we expect a valid block and
// item argument.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToNvm( uint16_t block, uint16_t item ) {

    uint16_t    arg     = 0;
    uint8_t     rStat   = readAttrMem( block, item, &arg );
    if ( rStat == NO_ERR ) rStat = writeAttrNvm( block, item, arg );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "readAttrMemExt" reads an attribute from the extended attribute map in main
// memory. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrMemExt( uint16_t item, uint16_t *arg ) {

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "writeAttrMemExt" updates an attribute in the extended attribute map in main
// memory. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrMemExt( uint16_t item, uint16_t arg ) {

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "readAttrNvmExt" reads an attribute from the extended attribute map in
// non volatile memory. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrNvmExt( uint16_t item, uint16_t *arg ) {

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "writeAttrNvmExt" writes an attribute to the extended attribute map in
// non volatile memory. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrNvmExt( uint16_t item, uint16_t arg ) {

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "syncAttrToMemExt" will copy the NVM extended attribute value to the MEM
// counterpart. All we do is just reading the NVM value to a dummy variable.
// As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToMemExt( uint16_t item ) {

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "syncAttrToNvmExt" will take the MEM attribute value of an item and writes it 
// to the NVM counterpart. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToNvmExt( uint16_t item ) {

    return ( ERR_NOT_IMPLEMENTED );
}

//----------------------------------------------------------------------------------------
// "rtLibGet" handles all items that directly refer to the node and port map.
// As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t rtLibGet( uint16_t npId, uint16_t item, uint16_t *arg ) {

    switch ( item ) {

        case ITEM_ID_DEBUG_MASK:            *arg = debugMask;               break;
        case ITEM_ID_RUNTIME_OPTIONS:       *arg = runtimeOptions;          break;
        case ITEM_ID_FIRMWARE_OPTIONS:      *arg = firmwareOptions;         break;
        case ITEM_ID_RT_LIB_VERSION:        *arg = LCS_RT_LIB_VERSION;      break;
        case ITEM_ID_RT_LIB_PATCH_LEVEL:    *arg = LCS_RT_LIB_PATCH_LEVEL;  break;
        case ITEM_ID_NODE_STATE:            *arg = nodeMap.nodeState;       break;
        case ITEM_ID_NODE_ID:               *arg = nodeMap.nodeId;          break;
        case ITEM_ID_RESTART_COUNT:         *arg = nodeMap.nodeRestartCnt;  break;
        case ITEM_ID_PORT_MAP_ENTRIES:      *arg = MAX_PORT_MAP_ENTRIES;    break;
        case ITEM_ID_PORT_MAP_HWM:          *arg = portMap.mapHwm;          break;
        case ITEM_ID_EVENT_MAP_ENTRIES:     *arg = MAX_EVENT_MAP_ENTRIES;   break;
        case ITEM_ID_EVENT_MAP_HWM:         *arg = eventMap.mapHwm;         break;
        case ITEM_ID_ATTR_MAP_ENTRIES:      *arg = MAX_ATTR_MAP_ENTRIES;    break;
        case ITEM_ID_LOOKUP_EVENT_ENTRY:    *arg = searchEvent( *arg );     break;

        case ITEM_ID_FLAGS: { 
            
            *arg = portMap.map[ portId( npId ) ].portFlags; 
            
        } break;

        case ITEM_ID_TYPE: { 
                        
            *arg = portMap.map[ portId( npId ) ].portType; 
            
        } break;

        case ITEM_ID_EVENT_DELAY_TICKS: {

            *arg = portMap.map[ portId( npId ) ].eventDelayTime;
            
        } break;

        default: return ( ERR_INVALID_ITEM_ID );
    }

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "rtLibSet" handles all items that directly refer to the node and port map.
// As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t rtLibSet( uint16_t npId, uint16_t item, uint16_t val ) {

    switch ( item ) {

        case ITEM_ID_DEBUG_MASK: {

            if ( usbIsConnected( )) debugMask = val | LCS_DBG_ENABLE;           
            else                    debugMask = val & ~ LCS_DBG_ENABLE;
            
            return ( LCS_OK );
        }

        case ITEM_ID_NODE_ID: {

            nodeMap.nodeId  = nodeId( val );
            uint32_t ofs    = NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId );
            return ( rtNvmPutWord( ofs, val ));
        }

        case ITEM_ID_FLAGS: {

            portMap.map[ portId( npId ) ].portFlags = val;
            return ( LCS_OK );
        }

        case ITEM_ID_TYPE: {

            portMap.map[ portId( npId ) ].portType = lowByte( val );
            return ( LCS_OK );
        }

        case ITEM_ID_EVENT_DELAY_TICKS: {

            portMap.map[ portId( npId ) ].eventDelayTime = val;
            return ( LCS_OK );
        }

        default: return ( ERR_INVALID_ITEM_ID );
    }
}

//----------------------------------------------------------------------------------------
// "rtLibRequest" handles the request items for the runtime library itself.
// As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t rtLibRequest( uint16_t npId, 
                      uint16_t item, 
                      uint16_t *arg1, 
                      uint16_t *arg2 ) {

    switch ( item ) {

        // ??? add OPS and CFG requests...

        case ITEM_ID_GET_NODE_UID: {

            *arg1 = nodeMap.nodeUID >> 16;
            *arg2 = nodeMap.nodeUID & 0xFFFF;
            return ( LCS_OK );
        }

        case ITEM_ID_GET_EVENT_MAP_ENTRY: {

            if ( arg2 == nullptr ) return ( ERR_INVALID_ATTR_ARG );
            return ( getMemEmapEntry( *arg1, arg1, arg2 ));
        }

        case ITEM_ID_RESET: {

            // ??? to do ...
            
            return ( ERR_NOT_IMPLEMENTED );
        }

        case ITEM_ID_SYNC_TO_MEM: {

            return ( syncAttrToMem( portId( npId ), *arg1 ));
        }

        case ITEM_ID_SYNC_TO_NVM: {

            return ( syncAttrToNvm( portId( npId ), *arg1 ));
        }

        case ITEM_ID_ADD_EVENT_MASK: {

            return ( setEventMask( *arg1, *arg2 ));
        }

        case ITEM_ID_REMOVE_EVENT_MASK: {

            return ( removeEventMask( *arg1 ));
        }

        case ITEM_ID_SYNC_EVENT_MAP_MEM: {

            return ( syncEventMapToMem( ));
        }

        case ITEM_ID_SYNC_EVENT_MAP_NVM: {

            return ( syncEventMapToNvm( ));
        }

        case ITEM_ID_ENABLE_EVENT_PROCESSING: {

            LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];

            if ( *arg1 ) pPtr -> portFlags |= NPF_PORT_EVENT_HANDLING_ENABLED;
            else         pPtr -> portFlags &= ~ NPF_PORT_EVENT_HANDLING_ENABLED;

            return ( LCS_OK );
        }

        case ITEM_ID_SET_ACTIVE_LED: {

            if ( *arg1 == 1 ) return ( writeDio( CDC_RN_ACTIVITY_LED, true ));
            else return ( writeDio( CDC_RN_ACTIVITY_LED, false ));
        }

        default: return ( RET_STAT( ERR_INVALID_ITEM_ID ));
    }
}

//----------------------------------------------------------------------------------------
// "rtLibI2cGet" reads a word from the peripheral based on the I2C address for 
// the port and channel passed. As an internal function, we expect validated 
// arguments.
//
//----------------------------------------------------------------------------------------
uint8_t rtLibI2cGet( uint16_t npId, uint16_t item, uint16_t *arg ) {

    uint8_t i2cAdr  = ( portId( npId ) * 8 ) + chanId( npId ) + 8; 
    uint8_t ofs     = item - IR_DRV_CHAN_START;
    uint8_t rStat   = 0;
    uint8_t buf[ 4 ];

    rStat = i2cWrite( CDC_RN_EXT_NVM, i2cAdr, &ofs, 1, true );
    return ( i2cRead( CDC_RN_EXT_NVM, i2cAdr, (uint8_t *) arg, 2, false ));
}

//----------------------------------------------------------------------------------------
// "rtLibI2cSet" writes a word to the peripheral based on the I2C address for 
// the port and channel passed. As an internal function, we expect validated 
// arguments.
//
//----------------------------------------------------------------------------------------
uint8_t rtLibI2cSet( uint16_t npId, uint16_t item, uint16_t val ) {

    uint8_t i2cAdr  = ( portId( npId ) * 8 ) + chanId( npId ) + 8; 
    uint8_t ofs     = item - IR_DRV_CHAN_START;
    uint8_t rStat   = 0;
    uint8_t buf[ 4 ];

    buf[ 0 ] = ofs;
    buf[ 1 ] = lowByte( val );
    buf[ 2 ] = highByte( val );

    return ( i2cWrite( CDC_RN_EXT_NVM, i2cAdr, (uint8_t *) &val, 2, false ));
}

//----------------------------------------------------------------------------------------
// User callback function invocation routine. Items 128 to 255 are user defined
// items. We will simply invoke a previously registered callback passing the 
// arguments. 
//
//----------------------------------------------------------------------------------------
uint8_t invokeUserItemCallback( uint16_t npId, 
                                uint16_t item, 
                                uint16_t *arg1, 
                                uint16_t *arg2 ) {

    LcsPortMapEntry *pPtr = & portMap.map[ portId( npId ) ];

    if ( pPtr -> reqCallback != nullptr ) {

        return ( pPtr -> reqCallback( portId( npId ), 
                                      item, 
                                      arg1, 
                                      arg2, 
                                      pPtr -> reqCallBackUdata ));
    }
    else return ( ERR_INVALID_ITEM_ID );
}

//----------------------------------------------------------------------------------------
// Driver request function invocation routine. Items 64 to 127 are reserved for 
// the driver request items. 
// 
//----------------------------------------------------------------------------------------
uint8_t invokeDrvItemCallback( uint16_t npId, 
                               uint16_t item, 
                               uint16_t *arg1, 
                               uint16_t *arg2 ) {

    LcsPortMapEntry *pPtr = & portMap.map[ portId( npId ) ];

    if ( pPtr -> drvReqCallback != nullptr ) {

        return ( pPtr -> drvReqCallback( npId, 
                                         item, 
                                         arg1, 
                                         arg2, 
                                         pPtr -> drvReqCallBackUdata ));
    }
    else return ( ERR_INVALID_ITEM_ID );
}
 
} // namespace


//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "nodeGet" will lookup a value from the header map, node map, port map or the 
// attribute data map. The "npId" argument contains the node and port Id. The 
// item argument determines which value we want to get. The data is returned in 
// the "arg" argument.
//
//----------------------------------------------------------------------------------------
uint8_t nodeGet( uint16_t npId, uint16_t item, uint16_t *arg ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeGet: npId: 0x%x, item: %d", npId, item  );
        if ( arg != nullptr ) printf( ":%d", *arg ); else printf( "null" );
        printf( "\n" );
    }

    if (( nodeMap.nodeState != NS_OPERATE) && 
        ( nodeMap.nodeState != NS_CONFIG ) && 
        ( nodeMap.nodeState != NS_INIT )) {

        return ( RET_STAT( ERR_LIB_NOT_READY ));
    }

    if ( arg == nullptr ) return ( RET_STAT( ERR_INVALID_ATTR_ARG )); 

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( rtLibGet( npId, item, arg )));
    }
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( rtLibI2cGet( npId, item, arg ) ));
    }
    else if ( isInRangeU16( item, IR_USER_RANGE_START, IR_ATTR_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {

            return ( RET_STAT( readAttrMem( portId( npId ), item, arg )));
        }
        else if ( nodeMap.nodeState == NS_CONFIG  ) {

            return ( RET_STAT( readAttrNvm( portId( npId ), item, arg )));
        }
        else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {

            return ( RET_STAT( readAttrMemExt( item, arg )));
        }
        else if ( nodeMap.nodeState == NS_CONFIG  ) {

            return ( RET_STAT( readAttrNvmExt( item, arg )));
        }
        else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "nodeSet" will write a value to the node map, port map or the attribute data 
// ranges. The "npId" argument contains the node, port and channel Id. For data
// attribute items the node state determines whether we just update the MEM 
// attribute or both MEM and NVM version. Node state CONFIG will update NVM too. 
//
//----------------------------------------------------------------------------------------
uint8_t nodeSet( uint16_t npId, uint16_t item, uint16_t val ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeSet: npId: 0x%x, item: %d, val:%d\n", npId, item, val  );
    }

    if (( nodeMap.nodeState != NS_OPERATE ) && 
        ( nodeMap.nodeState != NS_CONFIG )  && 
        ( nodeMap.nodeState != NS_INIT   )) {
        
        return ( RET_STAT( ERR_LIB_NOT_READY ));
    }

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( rtLibSet( npId, item, val )));
    }
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( rtLibI2cSet( npId, item, val ) ));
    }
    else if ( isInRangeU16( item, IR_USER_RANGE_START, IR_ATTR_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {

            return ( RET_STAT( writeAttrMem( portId( npId ), item, val )));
        }
        else if ( nodeMap.nodeState == NS_CONFIG ) { 

            return ( RET_STAT( writeAttrNvm( portId( npId ), item, val )));
        }
        else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE )); 
    } 
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {

            return ( RET_STAT( writeAttrMemExt( item, val )));
        }
        else if ( nodeMap.nodeState == NS_CONFIG ) { 

            return ( RET_STAT( writeAttrNvmExt( item, val )));
        }
        else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE )); 
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "nodeReq" will carry out a node, port or driver function request. A function 
// item defined in the user range, is handled by the firmware registered 
// callback function. Function items in the system range are handled by the 
// runtime library. 
//
//----------------------------------------------------------------------------------------
uint8_t nodeReq( uint16_t npId, uint16_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeReq: 0x%x:%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
        printf( "\n" );
    }

    if (( nodeMap.nodeState != NS_OPERATE ) && 
        ( nodeMap.nodeState != NS_CONFIG )) {
        
        return ( RET_STAT( ERR_LIB_NOT_READY ));
    }

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( rtLibRequest( npId, item, arg1, arg2 )));
    } 
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( invokeDrvItemCallback( npId, item, arg1, arg2 )));
    } 
    else if ( isInRangeU16( item, IR_USER_RANGE_START, IR_ATTR_RANGE_END )) {

        return ( RET_STAT( invokeUserItemCallback( npId, item, arg1, arg2 )));
    } 
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

} // namespace LCS
