//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime items
//
//----------------------------------------------------------------------------------------
// A key concept in the LCS runtime is the idea of items. An item is an entity
// such as a data attribute of a port or a requested function to perform. The
// item itself is a number. items are organized in ranges. Item 1 to 63 are
// reserved for runtime library attribute and functions, items 64 to 127 are 
// reserved for driver functions and items 128 to 255 are user defined items for
// attributes and function. In addition, there are global attributes, starting
// at itemId 256 up to the capacity of the NVM chip.
// 
// This file contains the LCS runtime routines that implement attribute and
// function access. Besides the item argument there is also the "npId" argument,
// which indicates the node and port the item refers to. The node portion of the
// npId is ignored, as the calls in this module always refer to the local node.
// The port portion of the npId is used to determine the port the item refers to.
// Finally, the channel portion of the npId is is used when the item is a driver
// function request.
//
// Note that the routines offered in this module are blocking calls. The caller
// will wait until the call is completed. The message system is responsible for
// receiving a message, calling this blocking functions in this module and 
// sending the reply back to the caller. 
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
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
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
    extern LcsPortDataMap   portDataMap;
    extern LcsPortMap       portMap;
    extern LcsEventMap      eventMap;

    extern int              searchEvent( uint16_t eventId );
    extern uint8_t          addEvent( uint16_t eventId, uint16_t eventMask );
    extern uint8_t          removeEvent( uint16_t eventId );
    extern uint8_t          updateEventMap( );

    extern uint8_t          getEventEntryByIndex( uint16_t index, 
                                                  uint16_t *eventId, 
                                                  uint16_t *eventMask );
    
    extern uint8_t          rtNvmPutWord( uint32_t ofs, uint16_t word );
    extern uint8_t          rtNvmGetWord( uint32_t ofs, uint16_t *word );

    extern uint8_t          rtNvmPutBytes( uint32_t ofs, 
                                           uint8_t *buf, 
                                           uint32_t len );

    extern uint8_t          rtNvmGetBytes( uint32_t ofs, 
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
// internal function, we expect validated arguments. The "port" argument will 
// refer to the node and port data attributes. 
//
//----------------------------------------------------------------------------------------
uint8_t readAttrMem( uint16_t port, uint16_t item, uint16_t *arg ) {

    *arg = portDataMap.map[ port ][ item - IR_PORT_ATTR_START ];
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// "writeAttrMem" stores a value to a node or port attribute map in MEM. As an 
// internal function, we expect validated arguments. The "port" argument will
// refer to the node and port data attributes. 
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrMem( uint16_t port, uint16_t item, uint16_t arg ) {

    portDataMap.map[ port ][ item - IR_PORT_ATTR_START ] = arg;
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// "attrNvmOffset" returns the NVM offset of the attribute item Id passed. The
// "npId" parameter contains the portId if the item is referring to a port item 
// range. The offset returned is the absolute byte offset into the NVM chip.
// It can be used to, for example, copy a range of data based in the starting
// item number.
// 
//----------------------------------------------------------------------------------------
uint8_t attrNvmOffset( uint16_t npId, uint16_t item, uint16_t *ofs ) {

    uint8_t  rStat = LCS_OK;
    uint16_t port  = portId( npId );  
    
    if (( portId( npId ) != 0 ) &&
        ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END ))) {

        uint16_t index  = item - IR_PORT_ATTR_START;
        *ofs = NVM_PORT_DATA_OFS + offsetof( LcsPortDataMap, map ) + 
            (( port * MAX_PORT_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        uint16_t index  = item - IR_GLOBAL_ATTR_START;
        *ofs = NVM_GLOBAL_DATA_OFS + sizeof( LcsGlobalDataMap ) + 
                         ( index * sizeof( uint16_t ));
    }
    else rStat = ERR_INVALID_ITEM_ID;

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "readAttrNvmRange" reads a range of attributes based on the starting item
// from NVM. The routine is typically used to read a whole set of data at node 
// startup.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrNvmRange( uint16_t npId, 
                          uint16_t startItem, 
                          uint16_t len, 
                          uint16_t *arg ) {

    uint8_t  rStat;
    uint16_t ofs;

    rStat = attrNvmOffset( npId, startItem, &ofs );

    if ( rStat == LCS_OK ) rStat = rtNvmGetBytes( ofs, 
                                                  (uint8_t *) arg, 
                                                  len * sizeof( uint16_t ));
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "writeAttrNvmRange" writes a range of attributes based on the starting item
// to NVM.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrNvmRange( uint16_t npId, 
                          uint16_t startItem, 
                          uint16_t len, 
                          uint16_t *arg ) {

    uint8_t  rStat;
    uint16_t ofs;

    rStat = attrNvmOffset( npId, startItem, &ofs );

    if ( rStat == LCS_OK ) rStat = rtNvmPutBytes( ofs, 
                                                  (uint8_t *) arg, 
                                                  len * sizeof( uint16_t ));
    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "readAttrNvm" gets an attribute from the NVM storage. We read the value 
// from the NVM area. If successful, we also store it in the MEM counterpart 
// and then return it. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrNvm( uint16_t port, uint16_t item, uint16_t *arg ) {

    uint16_t index  = item - IR_PORT_ATTR_START;
    uint16_t ofs    = NVM_PORT_DATA_OFS + offsetof( LcsPortDataMap, map ) + 
        (( port * MAX_PORT_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

    uint8_t rStat = rtNvmGetWord( ofs, arg );
    if ( rStat == NO_ERR ) rStat = writeAttrMem( port, item, *arg );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "writeAttrNvm" stores an attribute to the NVM storage. If the update is 
// successful, we also update the corresponding MEM attribute. As an internal
// function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrNvm( uint16_t port, uint16_t item, uint16_t arg ) {

    uint16_t index  = item - IR_PORT_ATTR_START;
    uint16_t ofs    = NVM_PORT_DATA_OFS + offsetof( LcsPortDataMap, map ) + 
        (( port * MAX_PORT_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

    uint8_t rStat = rtNvmPutWord( ofs, arg );
    if ( rStat == NO_ERR ) rStat = writeAttrMem( port, item, arg );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "syncAttrToMem" will copy the NVM attribute value to the MEM counterpart. 
// All we do is just reading the NVM value to a dummy variable. As an internal 
// function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToMem( uint16_t port, uint16_t item ) {

    uint16_t arg = 0;
    return ( readAttrNvm( port, item, &arg ));
}

//----------------------------------------------------------------------------------------
// "syncAttrToNvm" will take the MEM attribute value of an item and writes it 
// to the NVM counterpart. As an internal function, we expect a valid block and
// item argument.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToNvm( uint16_t port, uint16_t item ) {

    uint16_t    arg     = 0;
    uint8_t     rStat   = readAttrMem( port, item, &arg );
    if ( rStat == NO_ERR ) rStat = writeAttrNvm( port, item, arg );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "readAttrNvmExt" reads an attribute from the extended attribute map in
// non volatile memory. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrNvmExt( uint16_t item, uint16_t *arg ) {

    uint16_t index  = item - IR_GLOBAL_ATTR_START;
    uint16_t ofs    = NVM_GLOBAL_DATA_OFS + sizeof( LcsGlobalDataMap ) + 
                      ( index * sizeof( uint16_t ));

    return ( rtNvmGetWord( ofs, arg ));
}

//----------------------------------------------------------------------------------------
// "writeAttrNvmExt" writes an attribute to the extended attribute map in
// non volatile memory. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrNvmExt( uint16_t item, uint16_t arg ) {

    uint16_t index  = item - IR_GLOBAL_ATTR_START;
    uint16_t ofs    = NVM_GLOBAL_DATA_OFS + sizeof( LcsGlobalDataMap ) + 
                      ( index * sizeof( uint16_t ));

    return ( rtNvmPutWord( ofs, arg ));
}

//----------------------------------------------------------------------------------------
// "libItemGet" handles all items that directly refer to the node and port map.
// As an internal function, we expect validated arguments.
//
// ??? add new items for new nodeMap fields.
//----------------------------------------------------------------------------------------
uint8_t libItemGet( uint16_t npId, uint16_t item, uint16_t *arg ) {

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
        case ITEM_ID_ATTR_MAP_ENTRIES:      *arg = MAX_PORT_ATTR_MAP_ENTRIES;    break;

         case ITEM_ID_EVENT_MAP_HWM: {
            
            *arg = 0; // ??? fix, we need to get from NVM.        

        } break;

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
// "libItemSet" handles all items that directly refer to the node and port map.
// As an internal function, we expect validated arguments.
//
// ??? add new items for new nodeMap fields.
//----------------------------------------------------------------------------------------
uint8_t libItemSet( uint16_t npId, uint16_t item, uint16_t val ) {

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
// "libItemRequest" handles the request items for the runtime library itself.
// As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t libItemRequest( uint16_t npId, 
                        uint16_t item, 
                        uint16_t *arg1, 
                        uint16_t *arg2 ) {

    switch ( item ) {

        case ITEM_ID_RESET: {

            // ??? to do ...
            
            return ( ERR_NOT_IMPLEMENTED );
        }

        case ITEM_ID_GET_NODE_UID: {

            *arg1 = nodeMap.nodeUID >> 16;
            *arg2 = nodeMap.nodeUID & 0xFFFF;
            return ( LCS_OK );
        }

        case ITEM_ID_GET_EVENT_MAP_ENTRY: {

            if ( arg2 == nullptr ) return ( ERR_INVALID_ATTR_ARG );
            return ( getEventEntryByIndex( *arg1, arg1, arg2 ));
        }

        case ITEM_ID_SYNC_TO_MEM: {

            return ( syncAttrToMem( portId( npId ), *arg1 ));
        }

        case ITEM_ID_SYNC_TO_NVM: {

            return ( syncAttrToNvm( portId( npId ), *arg1 ));
        }

        case ITEM_ID_ADD_EVENT: {

            return ( addEvent( *arg1, *arg2 ));
        }

        case ITEM_ID_REMOVE_EVENT: {

            return ( removeEvent( *arg1 ));
        }

        case ITEM_ID_UPDATE_EVENT_MAP: {

            return ( updateEventMap( ));
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
// Get a port attribute. Depending on the node state, we will read the attribute
// from MEM or NVM. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t attrItemGet( int16_t npId, uint16_t item, uint16_t *arg  ) {

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( readAttrMem( portId( npId ), item, arg )));
    }
    else if ( nodeMap.nodeState == NS_CONFIG || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( readAttrNvm( portId( npId ), item, arg )));
    }
    else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
}

//----------------------------------------------------------------------------------------
// Set a port attribute. Depending on the node state, we will write the attribute
// to MEM or NVM. As an internal function, we expect validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t attrItemSet( int16_t npId, uint16_t item, uint16_t arg  ) {

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( writeAttrMem( portId( npId ), item, arg )));
    }
    else if ( nodeMap.nodeState == NS_CONFIG || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( writeAttrNvm( portId( npId ), item, arg )));
    }
    else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
}

//----------------------------------------------------------------------------------------
// Get a global attribute.
//
//----------------------------------------------------------------------------------------
uint8_t glbItemGet( int16_t npId, uint16_t item, uint16_t *arg  ) {

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( readAttrNvmExt( item, arg )));
    }
    else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
}

//----------------------------------------------------------------------------------------
// Get an extended attribute.
//
//----------------------------------------------------------------------------------------
uint8_t glbItemSet( int16_t npId, uint16_t item, uint16_t arg  ) {

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( writeAttrNvmExt( item, arg )));
    }
    else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
}

//----------------------------------------------------------------------------------------
// User callback function invocation routine. Items 128 to 255 are user defined
// items. We will simply invoke a previously registered callback passing the 
// arguments. 
//
//----------------------------------------------------------------------------------------
uint8_t userItemRequest( uint16_t npId, 
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
// "drvItemGet" reads a word from the peripheral based on the I2C address for 
// the port and channel passed. As an internal function, we expect validated 
// arguments. The item id is passed in the range for drivers ( 64 .. 127 ) but
// mapped to the driver board attribute numbers 0 .. 63.
//
//  We send the following data:
//
//      W: i2cAdr, item
//      R: i2cAdr, arg-h, arg-l
//
//----------------------------------------------------------------------------------------
uint8_t drvItemGet( uint16_t npId, uint16_t item, uint16_t *arg ) {

    uint8_t i2cAdr  = ( portId( npId ) * 8 ) + chanId( npId ) + 8; 
    uint8_t ofs     = item - IR_DRV_CHAN_START;
    uint8_t rStat   = 0;
    uint8_t buf[ 4 ];

    rStat = i2cWrite( CDC_RN_EXT_NVM, i2cAdr, &ofs, 1, true );
    return ( i2cRead( CDC_RN_EXT_NVM, i2cAdr, (uint8_t *) arg, 2, false ));
}

//----------------------------------------------------------------------------------------
// "drvItemSet" writes a word to the peripheral based on the I2C address for 
// the port and channel passed. The item id is passed in the range for drivers
// ( 64 .. 127 ) but mapped to the driver board attribute numbers 0 .. 63. As 
// an internal function, we expect validated arguments. 
//
// We send the following data:
// 
//      W: i2cAdr, item, arg-h, arg-l
//
//----------------------------------------------------------------------------------------
uint8_t drvItemSet( uint16_t npId, uint16_t item, uint16_t val ) {

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
// Driver request submit. We fill in the command data in the device. There are
// four fields, CMD, ARG1 and ARG2 and STATUS. 
// 
// We first send the request:
// 
//      W: i2cAdr, item, arg1-h, arg1-l, arg2-h, arg2-l
//
// And then poll for the reply:
//
//      W: i2cAdr, item
//      R: i2cAdr, stat, arg1-h, arg1-l, arg2-h, arg2-l
//
// The status field tells the outcome. We read the entire data fields and
// check the status field. A code of 255 is the "busy" code.
//
//----------------------------------------------------------------------------------------
uint8_t drvItemRequest( uint16_t npId, 
                        uint16_t item, 
                        uint16_t *arg1, 
                        uint16_t *arg2 ) {

    const int maxRetryCount = 100; // ??? for now ...

    uint16_t        index       = item - IR_DRV_CHAN_START;
    LcsPortMapEntry *pPtr       = & portMap.map[ portId( npId ) ];
    uint8_t         i2cAdr      = ( portId( npId ) * 8 ) + chanId( npId ) + 8;
    uint8_t         rStat       = 0;
    uint8_t         buf[ 16 ];
    
    buf[ 0 ] = lowByte( item );
    buf[ 1 ] = lowByte( *arg1 );
    buf[ 2 ] = highByte( *arg1 );
    buf[ 3 ] = lowByte( *arg2 );
    buf[ 4 ] = highByte( *arg2 );

    rStat = i2cWrite( CDC_RN_EXT_NVM, i2cAdr, buf, 6, false );
    if ( rStat == NO_ERR ) {

        for ( int i = 0; i < maxRetryCount; i++ ) {

            rStat = i2cRead( CDC_RN_EXT_NVM, i2cAdr, buf, 6, false );
            if ( rStat != NO_ERR ) break;
            if ( buf[ 0 ] != 0xFF ) break;
        }
    }

    return( rStat );
}

} // namespace


//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "nodeGet" will lookup a value from the various maps based on the item Id. 
// The "npId" argument contains the node/port/channel Id. 
//
//----------------------------------------------------------------------------------------
uint8_t nodeGet( uint16_t npId, uint16_t item, uint16_t *arg ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeGet: npId: 0x%x, item: %d", npId, item  );
        if ( arg != nullptr ) printf( ":%d", *arg ); else printf( "null" );
        printf( "\n" );
    }

    if ( arg == nullptr ) return ( RET_STAT( ERR_INVALID_ATTR_ARG )); 

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( libItemGet( npId, item, arg )));
    }
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( drvItemGet( npId, item, arg ) ));
    }
    else if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return( RET_STAT ( attrItemGet( npId, item, arg )));
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        return( RET_STAT ( glbItemGet( npId, item, arg )));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "nodeSet" will write a value into one of the various maps. The "npId"
// argument contains the node/port/channel Id. 
//
//----------------------------------------------------------------------------------------
uint8_t nodeSet( uint16_t npId, uint16_t item, uint16_t val ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeSet: npId: 0x%x, item: %d, val:%d\n", npId, item, val  );
    }

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( libItemSet( npId, item, val )));
    }
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( drvItemSet( npId, item, val ) ));
    }
    else if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return( RET_STAT ( attrItemSet( npId, item, val )));
    } 
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

       return( RET_STAT ( glbItemSet( npId, item, val )));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "nodeGetRange" is used to locally read a range of attributes for performance
// reasons. A npId of zero refers to the global attribute range, a npId with a
// port Id will refer to the attribute range for the port.
//
//----------------------------------------------------------------------------------------
uint8_t nodeGetRange( uint16_t npId, 
                      uint16_t itemStart, 
                      uint16_t len, 
                      uint16_t *argArray ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeGetRange: npId: 0x%x, itemStart: %d, len:%d\n", 
                npId, itemStart, len  );
    }

    uint8_t  rStat;
    uint32_t end = itemStart + len;

    if ( isInRangeU16( itemStart, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        if ( end > IR_PORT_ATTR_END ) return ( RET_STAT( ERR_INVALID_ITEM_ID ));
    }
    else if ( isInRangeU16( itemStart, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        if ( end > IR_GLOBAL_ATTR_END ) return ( RET_STAT( ERR_INVALID_ITEM_ID ));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));

    return( rStat );
}

//----------------------------------------------------------------------------------------
//
// ??? I am not sure we would really need this. the caller could also just 
// do a loop reading a word at a time.... let's see...
//----------------------------------------------------------------------------------------
uint8_t nodeSetRange( uint16_t npId, 
                      uint16_t itemStart, 
                      uint16_t len, 
                      uint16_t *argArray ) {

     if ( itemDebugEnabled( )) {

        printf( "nodeSetRange: npId: 0x%x, itemStart: %d, len:%d\n", 
                npId, itemStart, len  );
    }

    uint8_t rStat;
    uint32_t end  = itemStart + len;

    if ( isInRangeU16( itemStart, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        if ( end > IR_PORT_ATTR_END ) return ( RET_STAT( ERR_INVALID_ITEM_ID ));
    }
    else if ( isInRangeU16( itemStart, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        if ( end > IR_GLOBAL_ATTR_END ) return ( RET_STAT( ERR_INVALID_ITEM_ID ));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));

    return( RET_STAT( ERR_NOT_IMPLEMENTED )); // ??? for now ...
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
uint8_t nodeGetItemPtr( uint16_t npId,
                        uint16_t item,
                        void **itemPtr ) {

    if ( itemDebugEnabled( )) {

        printf( "nodeGetItemPtr: npId: 0x%x, item: %d\n", npId, item  );
    }

    if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return( RET_STAT( ERR_NOT_IMPLEMENTED )); // ??? for now ... 
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        return( RET_STAT( ERR_NOT_IMPLEMENTED )); // ??? for now ... 
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "nodeReq" will carry out a node, port or driver function request. The "npId"
// argument contains the node/port/channel Id. The "item" argument indicates the
// requested function. The "arg1" and "arg2" arguments are optional arguments 
// for the requested function. The actual meaning of the arguments depends on 
// the requested function.
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

        return ( RET_STAT( libItemRequest( npId, item, arg1, arg2 )));
    } 
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( drvItemRequest( npId, item, arg1, arg2 )));
    } 
    else if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return ( RET_STAT( userItemRequest( npId, item, arg1, arg2 )));
    } 
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

} // namespace LCS
