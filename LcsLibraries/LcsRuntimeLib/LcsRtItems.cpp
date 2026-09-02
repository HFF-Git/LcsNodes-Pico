//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime items
//
//----------------------------------------------------------------------------------------
// A key concept in the LCS runtime is the idea of items. An item is an entity
// such as a data attribute of a port or a requested function to perform. The
// item itself is a number and organized in ranges. Item 1 to 63 are reserved 
// for runtime library attribute and functions, items 64 to 127 are reserved 
// for driver functions. Items 128 to 255 are port data items used for user 
// defined attributes and functions. In addition, there are global attributes, 
// starting at itemId 256 up to the capacity of the NVM chip.
// 
// This file contains the LCS runtime routines that implement attribute and
// function access. Besides the item argument there is also the "npId" argument,
// which indicates the node, port and channel the item refers to. The routines 
// in this file implement the local access, the nodeId portion is ignored. The 
// port and channel portion of the npId are used to determine the port and
// channel if required. 
//
// Note that the routines offered in this module are blocking calls and run to
// completion. When we receive a request from another node, read and function
// request items are started and the result is returned via a reply message. 
// The "LcsCore.cpp" implementation file contains the routines that implement 
// the message handling from other nodes.
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
    extern LcsGlobalDataMap globalDataMap;
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
// Debug support routines. We can easily check whether debug is enabled at all.
// The return status routine will print out a return status message when 
// debugging is enabled. The macro "RET_STAT" is a nice helper that adds the 
// function name. The ENTER_FUNC macro is a helper to print out the function 
// name when entering a routine.
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

inline void enterFunc( const char *name ) {

    if ( itemDebugEnabled( )) printf( "--> %s\n", name );
}

#define ENTER_FUNC() enterFunc( __func__)
#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// "attrValidRange" ensures that the specified range is a valid for accessing
// attributes. We accept as valid item ranges only port and global attribute 
// items.
//
//----------------------------------------------------------------------------------------
bool attrValidRange( uint16_t item, uint16_t len ) {

    if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        if ( IR_PORT_ATTR_END - item + 1 < len ) return ( false );
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        if ( IR_GLOBAL_ATTR_END - item + 1 < len ) return ( false );
    }
    else return ( false );

    return ( true );
}

//----------------------------------------------------------------------------------------
// "attrMemAdr" returns the memory address of the first item and also validates
// that the range is actually valid memory.
//
//----------------------------------------------------------------------------------------
uint8_t attrMemAdr( uint16_t npId, 
                    uint16_t item, 
                    uint16_t **adr, 
                    uint16_t len ) {

    uint8_t  rStat;
    uint16_t port = portId( npId );

     if (( port != 0 ) &&
         ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END ))) {

        uint16_t index  = item - IR_PORT_ATTR_START;
        *adr = & portDataMap.map[ port ] [ index ];
        return ( LCS_OK );
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        uint16_t index  = item - IR_GLOBAL_ATTR_START;
        *adr = & globalDataMap.map[ index ];
        return ( LCS_OK );
    }
    else return ( ERR_INVALID_ITEM_ID );
}

//----------------------------------------------------------------------------------------
// "attrNvmOfs" returns the absolute NVM offset of the attribute item Id passed.
// The "npId" parameter contains the portId if the item is referring to a port
// data attribute. 
// 
//----------------------------------------------------------------------------------------
uint8_t attrNvmOfs( uint16_t npId, 
                    uint16_t item, 
                    uint16_t *ofs, 
                    uint16_t len ) {

    uint8_t  rStat;
    uint16_t port = portId( npId );

    if (( port != 0 ) &&
        ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END ))) {

        uint16_t index = item - IR_PORT_ATTR_START;
        *ofs = NVM_PORT_DATA_OFS + offsetof( LcsPortDataMap, map ) + 
            (( port * MAX_PORT_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        return( LCS_OK );
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        uint16_t index = item - IR_GLOBAL_ATTR_START;
        *ofs = NVM_GLOBAL_DATA_OFS + sizeof( LcsGlobalDataMap ) + 
                         ( index * sizeof( uint16_t ));
        return( LCS_OK );
    }
    else return( ERR_INVALID_ITEM_ID );
}

//----------------------------------------------------------------------------------------
// "readAttrMem" gets a range if items from the port attribute map or global
// data map in MEM. The "len" parameter indicates how many items to read. The 
// "arg" parameter is a pointer to the buffer where the read items will be 
// stored.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrMem( uint16_t npId, 
                     uint16_t item, 
                     uint16_t *arg, 
                     uint16_t len ) {

    uint16_t *adr;
    uint8_t  rStat = attrMemAdr( npId, item, &adr, len );
    if ( rStat != LCS_OK ) return( rStat );

    memcpy((uint8_t *) arg, (uint8_t *) adr, len * sizeof( uint16_t ));
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeAttrMem" stores a range if items to the port attribute map or global
// data map in MEM. The "len" parameter indicates how many items to write. The
// "arg" parameter is a pointer to the buffer where the items to write are
// stored.
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrMem( uint16_t npId, 
                      uint16_t item, 
                      uint16_t *arg, 
                      uint16_t len  ) {

    uint16_t *adr;
    uint8_t  rStat = attrMemAdr( npId, item, &adr, len );
    if ( rStat != LCS_OK ) return( rStat );

    memcpy((uint8_t *) adr, (uint8_t *) arg, len * sizeof( uint16_t ));
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "readAttrNvm" gets an attribute from the NVM storage. We read the value 
// from the NVM area. If successful, we also store the value in the MEM 
// counterpart and then return it.
//
//----------------------------------------------------------------------------------------
uint8_t readAttrNvm( uint16_t npId, 
                     uint16_t item, 
                     uint16_t *arg, 
                     uint16_t len ) {

    uint8_t  rStat;
    uint16_t port = portId( npId );
    uint16_t ofs;

    rStat = attrNvmOfs( npId, item, &ofs, len );
    if ( rStat != LCS_OK ) return ( ERR_INVALID_ITEM_ID );

    rStat = rtNvmGetBytes( ofs, (uint8_t *) arg, len * sizeof( uint16_t )); 
    if ( rStat != LCS_OK ) return( rStat );

    rStat = writeAttrMem( npId, item, arg, len );
    return( rStat );
}

//----------------------------------------------------------------------------------------
// "writeAttrNvm" stores an attribute to the NVM storage. If the update is 
// successful, we also update the corresponding MEM attribute. 
//
//----------------------------------------------------------------------------------------
uint8_t writeAttrNvm( uint16_t npId, 
                      uint16_t item, 
                      uint16_t *arg, 
                      uint16_t len  ) {

    uint8_t  rStat;
    uint16_t port = portId( npId );
    uint16_t ofs;

    rStat = attrNvmOfs( npId, item, &ofs, len );
    if ( rStat != LCS_OK ) return ( ERR_INVALID_ITEM_ID );

    rStat = rtNvmPutBytes( ofs, (uint8_t *) arg, len * sizeof( uint16_t ));
    if ( rStat != LCS_OK ) return ( rStat );

    rStat = writeAttrMem( npId, item, arg, len );
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "syncAttrToMem" copies the range of NVM attributes to their MEM counterpart. 
// All we do is just reading an attribute from NVM at a time. As a side effect,
// it also copied to MEM.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToMem( uint16_t npId, uint16_t item, uint16_t len ) {

    if ( ! attrValidRange( item, len )) return ( ERR_INVALID_ITEM_ID );

    for ( uint16_t i = 0; i < len; i++  ) {

        uint16_t arg;
        uint8_t rStat = readAttrNvm( npId, item + i, &arg, 1 );
        if ( rStat != LCS_OK ) return( rStat );
    }

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "syncAttrToNvm" copies the range of MEM attributes to their NVM counterpart.
//
//----------------------------------------------------------------------------------------
uint8_t syncAttrToNvm( uint16_t npId, uint16_t item, uint16_t len ) {

    if ( ! attrValidRange( item, len )) return ( ERR_INVALID_ITEM_ID );

    for ( uint16_t i = 0; i < len; i++  ) {

        uint16_t arg;

        uint8_t rStat = readAttrMem( npId, item + i, &arg, 1  );
        if ( rStat != LCS_OK ) return( rStat );

        rStat = writeAttrNvm( npId, item, &arg, 1 );
        if ( rStat != LCS_OK ) return( rStat );
    }

    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "getLibItem" handles all items that directly refer to the node and port map.
// As an internal function, we expect validated arguments.
//
// ??? add new items for new nodeMap fields.
//----------------------------------------------------------------------------------------
uint8_t getLibItem( uint16_t npId, uint16_t item, uint16_t *arg ) {

    switch ( item ) {

        case ITEM_ID_DEBUG_MASK: {

            *arg = debugMask;           

        } break;

        case ITEM_ID_RUNTIME_OPTIONS: {       
            
            *arg = runtimeOptions;          

        } break;
        
        case ITEM_ID_FIRMWARE_OPTIONS: {     
            
            *arg = firmwareOptions;        
            
        } break;

        case ITEM_ID_RT_LIB_VERSION: {       
            
            *arg = LCS_RT_LIB_VERSION;      
            
        } break;

        case ITEM_ID_RT_LIB_PATCH_LEVEL: {   
            
            *arg = LCS_RT_LIB_PATCH_LEVEL;  
            
        } break;

        case ITEM_ID_NODE_STATE: {

            *arg = nodeMap.nodeState;       
            
        } break;

        case ITEM_ID_NODE_ID: {              
            
            *arg = nodeMap.nodeId;         
        
        } break;
        
        case ITEM_ID_RESTART_COUNT: {         
            
            *arg = nodeMap.nodeRestartCnt;  
            
        } break;
        
        case ITEM_ID_PORT_MAP_ENTRIES: {    
            
            *arg = MAX_PORT_MAP_ENTRIES; 
        
        } break;

        case ITEM_ID_PORT_MAP_HWM: {           
            
            *arg = portMap.mapHwm;         
        
        }  break;
        
        case ITEM_ID_EVENT_MAP_ENTRIES: { 
            
            *arg = MAX_EVENT_MAP_ENTRIES;   
            
        } break;
        
        case ITEM_ID_ATTR_MAP_ENTRIES: {     
            
            *arg = MAX_PORT_ATTR_MAP_ENTRIES;   
            
        } break;

         case ITEM_ID_EVENT_MAP_HWM: { 
            
            *arg = eventMap.mapHwm; 
        
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
// "setLibItem" handles all items that directly refer to the node and port map.
// As an internal function, we expect validated arguments.
//
// ??? add new items for new nodeMap fields.
//----------------------------------------------------------------------------------------
uint8_t setLibItem( uint16_t npId, uint16_t item, uint16_t val ) {

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
// "reqLibItem" handles the request items for the runtime library itself.
//
//----------------------------------------------------------------------------------------
uint8_t reqLibItem( uint16_t npId, 
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

            return ( syncAttrToMem( portId( npId ), *arg1, *arg2 ));
        }

        case ITEM_ID_SYNC_TO_NVM: {

            return ( syncAttrToNvm( portId( npId ), *arg1, *arg2 ));
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
// Get a port or global attribute range. Depending on the node state, we will 
// read the attribute from MEM or NVM. As an internal function, we expect 
// validated arguments.
//
//----------------------------------------------------------------------------------------
uint8_t getAttrItem( int16_t  npId, 
                     uint16_t item, 
                     uint16_t *arg, 
                     uint16_t len = 1  ) {

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( readAttrMem( npId, item, arg, len )));
    }
    else if ( nodeMap.nodeState == NS_CONFIG || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( readAttrNvm( npId, item, arg, len )));
    }
    else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
}

//----------------------------------------------------------------------------------------
// Set a port attribute. Depending on the node state, we will write the attribute
// to MEM or NVM.
//
//----------------------------------------------------------------------------------------
uint8_t setAttrItem( int16_t npId, 
                     uint16_t item, 
                     uint16_t *arg, 
                     uint16_t len = 1 ) {

    if (( nodeMap.nodeState == NS_OPERATE ) || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( writeAttrMem( npId, item, arg, len )));
    }
    else if ( nodeMap.nodeState == NS_CONFIG || ( nodeMap.nodeState == NS_INIT )) {

        return ( RET_STAT( writeAttrNvm( npId, item, arg, len )));
    }
    else return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
}

//----------------------------------------------------------------------------------------
// User callback function invocation routine. Items 128 to 255 are user defined
// items. We will simply invoke a previously registered callback passing the 
// arguments. 
//
//----------------------------------------------------------------------------------------
uint8_t userReqItemRequest( uint16_t npId, 
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
// "getDrvItem" reads a word from the peripheral based on the I2C address for 
// the port and channel passed. The i2cAdr is calculated from the port and 
// channel Id. The driver item range is 64 to 127, which we map to the peripheral
// item range 0 to 63.
//
//  We send the following data:
//
//      W: i2cAdr, item
//      R: i2cAdr, arg-h, arg-l
//
// ??? check transfer order to arg ...
//----------------------------------------------------------------------------------------
uint8_t getDrvItem( uint16_t npId, uint16_t item, uint16_t *arg ) {

    uint8_t i2cAdr  = ( portId( npId ) * 8 ) + chanId( npId ) + 8; 
    uint8_t index   = item - IR_DRV_CHAN_START;
    uint8_t rStat   = LCS_OK;
    uint8_t buf[ 4 ];

    rStat = i2cWrite( CDC_RN_EXT_NVM, i2cAdr, &index, 1, true );
    if ( rStat != LCS_OK ) return ( rStat );

    rStat = i2cRead( CDC_RN_EXT_NVM, i2cAdr, buf, 2, false );
    if ( rStat != LCS_OK ) return ( rStat );

    *arg = ( buf[ 1 ] << 8 ) | buf[ 0 ];
    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "setDrvItem" writes a word to the peripheral based on the I2C address for 
// the port and channel passed. The i2cAdr is calculated from the port and 
// channel Id. The driver item range is 64 to 127, which we map to the peripheral
// item range 0 to 63.
//
// We send the following data:
// 
//      W: i2cAdr, item, arg-h, arg-l
//
// ??? check transfer order of val ...
//----------------------------------------------------------------------------------------
uint8_t setDrvItem( uint16_t npId, uint16_t item, uint16_t val ) {

    uint8_t i2cAdr  = ( portId( npId ) * 8 ) + chanId( npId ) + 8; 
    uint8_t index   = item - IR_DRV_CHAN_START;
    uint8_t rStat   = LCS_OK;
    uint8_t buf[ 4 ];

    buf[ 0 ] = index;
    buf[ 1 ] = lowByte( val );
    buf[ 2 ] = highByte( val );

    return ( i2cWrite( CDC_RN_EXT_NVM, i2cAdr, buf, 3, false ));
}

//----------------------------------------------------------------------------------------
// Driver request submit. The i2cAdr is calculated from the port and channel 
// Id. The driver item range is 64 to 127, which we map to the peripheral
// item range 0 to 63. We fill in the command data in the device. There are 
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
// The status field in the reply message tells the outcome. We read the entire 
// data fields and check the status field. A code of 255 is the "busy" code.
//
//----------------------------------------------------------------------------------------
uint8_t reqDrvItem( uint16_t npId, 
                    uint16_t item, 
                    uint16_t *arg1, 
                    uint16_t *arg2 ) {

    const int maxRetryCount = 20; // ??? for now ...

    uint16_t    index       = item - IR_DRV_CHAN_START;
    uint8_t     i2cAdr      = ( portId( npId ) * 8 ) + chanId( npId ) + 8;
    uint8_t     rStat       = 0;
    uint8_t     buf[ 16 ];
    
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

        if ( rStat == LCS_OK ) {

            *arg1 = ( buf[ 2 ] << 8 ) | buf[ 1 ];
            *arg2 = ( buf[ 4 ] << 8 ) | buf[ 3 ];
        }
    }

    return ( rStat );
}

} // namespace


//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "getItem" will lookup a value from the various maps based on the item Id. 
// The "npId" argument contains the node/port/channel Id. 
//
//----------------------------------------------------------------------------------------
uint8_t getItem( uint16_t npId, uint16_t item, uint16_t *arg, uint16_t len ) {

    if ( itemDebugEnabled( )) {

        printf( "getItem: npId: 0x%x, item: %d, len: %d", npId, item, len  );
        if ( arg != nullptr ) printf( ":%d", *arg ); else printf( "null" );
        printf( "\n" );
    }

    if ( arg == nullptr ) return ( RET_STAT( ERR_INVALID_ATTR_ARG )); 

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( getLibItem( npId, item, arg )));
    }
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( getDrvItem( npId, item, arg ) ));
    }
    else if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return ( RET_STAT ( getAttrItem( npId, item, arg, len )));
    }
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

        return ( RET_STAT ( getAttrItem( npId, item, arg, len )));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "setItem" will write a value into one of the various maps. The "npId"
// argument contains the node/port/channel Id. 
//
//----------------------------------------------------------------------------------------
uint8_t setItem( uint16_t npId, uint16_t item, uint16_t *val, uint16_t len ) {

    if ( itemDebugEnabled( )) {

        printf( "setItem: npId: 0x%x, item: %d, len:%d\n", npId, item, len  );
    }

    if ( isInRangeU16( item, IR_LIB_MAP_RANGE_START, IR_LIB_MAP_RANGE_END )) {

        return ( RET_STAT( setLibItem( npId, item, *val )));
    }
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( setDrvItem( npId, item, *val ) ));
    }
    else if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return ( RET_STAT ( setAttrItem( npId, item, val, 1 )));
    } 
    else if ( isInRangeU16( item, IR_GLOBAL_ATTR_START, IR_GLOBAL_ATTR_END )) {

       return ( RET_STAT ( setAttrItem( npId, item, val, 1 )));
    }
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

//----------------------------------------------------------------------------------------
// "getItemPtr" will return a void pointer to the MEM offset, where the
// item can be found. This is a useful but also dangerous function. The pointer
// returned is just the address of the item. When the function is used to get
// the address in memory of a larger data structure implemented as a range of 
// items, it is up to the firmware programmer to ensure that the accessed
// memory is the valid one with the correct size. We do however check that 
// the intended range is actually valid memory.
//
//----------------------------------------------------------------------------------------
uint8_t getItemPtr( uint16_t npId,
                    uint16_t item,
                    uint16_t **itemPtr,
                    uint16_t len ) {

    if ( itemDebugEnabled( )) {

        printf( "getItemPtr: npId: 0x%x, item: %d, len: %d\n", 
                npId, item, len  );
    }

    return( attrMemAdr( npId, item, itemPtr, len ));
}

//----------------------------------------------------------------------------------------
// "reqItem" will carry out a node, port or driver function request. The "npId"
// argument contains the node/port/channel Id. The "item" argument indicates the
// requested function. The "arg1" and "arg2" arguments are optional arguments 
// for the requested function. The actual meaning of the arguments depends on 
// the requested function.
//
//----------------------------------------------------------------------------------------
uint8_t reqItem( uint16_t npId, uint16_t item, uint16_t *arg1, uint16_t *arg2 ) {

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

        return ( RET_STAT( reqLibItem( npId, item, arg1, arg2 )));
    } 
    else if ( isInRangeU16( item, IR_DRV_CHAN_START, IR_DRV_CHAN_END )) {

        return ( RET_STAT( reqDrvItem( npId, item, arg1, arg2 )));
    } 
    else if ( isInRangeU16( item, IR_PORT_ATTR_START, IR_PORT_ATTR_END )) {

        return ( RET_STAT( userReqItemRequest( npId, item, arg1, arg2 )));
    } 
    else return ( RET_STAT( ERR_INVALID_ITEM_ID ));
}

} // namespace LCS
