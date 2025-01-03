//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - node access routines.
//
//------------------------------------------------------------------------------------------------------------
// The file contains the LCS runtime routines that implement node access. There are three routines that allow
// to manipulate node and port data as well as issue requests to a node or port. The key are the node/port ID
// and the item number. The "npId" will indicate which node and port the call refers to. The node portion is 
// typically our own node Id, the port Id refers to a ports on the node, with a port Id of zero referring to
// the node itself. Any node can access another node. In this case request comes via a message and the 
// message handler will call the local routines in this file. 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Core Library
// Copyright (C) 2021 - 2025  Helmut Fieres
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
// External declaration to global structures defined in "LcsRtSetup".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t             debugMask;
    extern LcsNvmHeaderMap      nvmHeaderMap;
    extern LcsCdcMap            cdcMap;
    extern LcsNodeMap           nodeMap;
    extern LcsNodeData          nodeData;
    extern LcsPortMap           portMap;
   // extern LcsCallbackMap       callbackMap;
};

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

    using namespace LCS;

    //--------------------------------------------------------------------------------------------------------
    // The node or port name cannot be set with a single LCS message. We will store the parts in this 
    // temporary buffer and set the name when all parts are received.
    //
    //--------------------------------------------------------------------------------------------------------
    char tempName[ MAX_NODE_NAME_SIZE + 1 ] = { 0 };

    //--------------------------------------------------------------------------------------------------------
    // Utility routines.
    //
    //--------------------------------------------------------------------------------------------------------
    bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

        return (( val >= lower ) && ( val <= upper ));
    }

    uint8_t lowByte( uint16_t arg ) { 
        
        return( arg & 0xFF ); 
    }
    
    uint8_t highByte( uint16_t arg ) { 
        
        return( arg >> 8 ); 
    }

     uint16_t nodeId( uint16_t arg ) {

        return( arg >> 4 );
    }

    uint16_t portId( uint16_t arg ) {

        return( arg & 0xF);
    }

    //--------------------------------------------------------------------------------------------------------
    // "readAttrMem" gets a value from the node or port attribute map in MEM. As an internal function, we 
    // expect a valid block and item argument. The "block" argument will refer to the node and port data
    // attributes. Block 0 is the node, all others the port. 
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t readAttrMem( uint8_t block, uint8_t item, uint16_t *arg ) {

        *arg = nodeData.map[ block ][ item - IR_ATTR_RANGE_START ];
        return ( LCS::ALL_OK );
    }

    //----------------------------------------------------------------------------------------------------------
    // "writeAttrMem" stores a value to a node or port attribute map in MEM. As an internal function, we 
    // expect a valid block and item argument. The "block" argument will refer to the node and port data
    // attributes. Block 0 is the node, all others the port. 
    //
    //----------------------------------------------------------------------------------------------------------
    uint8_t writeAttrMem( uint8_t block, uint8_t item, uint16_t arg ) {

        nodeData.map[ block ][ item - IR_ATTR_RANGE_START ] = arg;
        return ( LCS::ALL_OK );
    }

    //--------------------------------------------------------------------------------------------------------
    // "readAttrNvm" gets an attribute from the NVM storage. We read the value from NVM, store it in the MEM 
    // counterpart and then return it. For the NVM access, the byte offset into the storage needs to be 
    // computed. As an internal function, we expect a valid block and item argument.
    //
    //----------------------------------------------------------------------------------------------------------
    uint8_t readAttrNvm( uint8_t block, uint8_t item, uint16_t *arg ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

            printf( "readAttrNvm: block: 0x%x, item: %d\n", block, item );
        }

        uint16_t index  = item - IR_ATTR_RANGE_START;
        uint16_t ofs    = nodeMap.nvmNodeDataOfs + (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        printf( "Ofs: 0x%x\n", ofs );

        uint8_t rStat = rtNvmGetWord( ofs, &nodeData.map[ block ][ index ] );

        printf( "rStat: 0x%x\n", rStat );

        *arg = (( rStat == ALL_OK ) ? nodeData.map[ block ][ index ] : 0 );
        return ( rStat );
    }

    //--------------------------------------------------------------------------------------------------------
    // "writeAttrNvm" stores an attribute to the NVM storage. We first update the corresponding MEM attribute
    // and then write the value to NVM storage. For the NVM access, the byte offset into the storage needs to
    // be computed. As an internal function, we expect a valid block and item argument.
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t writeAttrNvm( uint8_t block, uint8_t item, uint16_t arg ) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

            printf( "readAttrNvm: block: 0x%x, item: %d\n", block, item );
        }

        uint16_t index  = item - IR_ATTR_RANGE_START;
        uint16_t ofs    = nodeMap.nvmNodeDataOfs + (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        printf( "Ofs: 0x%x\n", ofs );

        nodeData.map[ block ][ index ] = arg;
        return ( rtNvmPutWord( ofs, arg ));
    }

    //--------------------------------------------------------------------------------------------------------
    // User callback function invocation routine. Items 64 to 127 are user defined items. We will simply
    // invoke a previously registered callback passing the arguments.
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t invokeUserItemCallback( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

        if (( portId == 0 ) && ( nodeMap.reqCallback != nullptr )) { 

            return ( nodeMap.reqCallback( portId, item, arg1, arg2 ));
        }
        else if ( isInRangeU( portId, MIN_PORT_ID, MAX_PORT_ID )) {

            if ( portMap.map[ portId ].reqCallbackFunc != nullptr ) {

                return ( portMap.map[ portId ].reqCallbackFunc( portId, item, arg1, arg2 ));
            }
            else return( ERR_INVALID_ITEM_ID );
        }
        else return( ERR_INVALID_ITEM_ID );
    }
    
} // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "nodeGet" will lookup a value from the node, port or the attribute data map. The "npId" argument contains
// the node and port Id. However, we will only use the portId portion, which represents the block index.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nodeGet( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

        printf( "nodeGet: 0x%x:%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
    }

    if ( isInRangeU( item, IR_ATTR_RANGE_START, IR_ATTR_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {
            
            return ( readAttrMem( portId( npId ), item, arg1 ));
        }
        else if ( nodeMap.nodeState == NS_CONFIG ) {
        
            return ( readAttrNvm( portId( npId ), item, arg1 ));
        }
        else return( ERR_INVALID_OP_FOR_NODE_STATE );
        
    } else {

        switch ( item ) {

            case ITEM_ID_DEBUG_MASK: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );  
                *arg1 = debugMask;
                return( ALL_OK );
            }

            case ITEM_ID_OPTIONS: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );  

                *arg1 = nodeMap.nodeOptions;
                return ( ALL_OK );
            }

            case ITEM_ID_FLAGS: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 )   *arg1 = nodeMap.nodeFlags;
                else                         *arg1 = portMap.map[ portId( npId ) - 1 ].flags;

                return ( ALL_OK );
            }

            case ITEM_ID_VERSION: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                *arg1 = nodeMap.nodeSwVersion;
                return ( ALL_OK );          
            }

            case ITEM_ID_BOARD_VERSION: {

                // ??? board version ...
                
                return( 255 );
            } break;

            case ITEM_ID_TYPE: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 )   *arg1 = nodeMap. nodeType;
                else                         *arg1 = portMap.map[ portId( npId ) - 1 ].type;

                return ( ALL_OK );
            }

            case ITEM_ID_CONTROLLER_FAMILY: {

                // ??? controller family should actually become part of the version and board version ...

                 if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nvmHeaderMap.map[ 0 ].controllerFamily;
                return ( ALL_OK );
            }

            case ITEM_ID_NODE_STATE: {

                 if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.nodeState;
                return ( ALL_OK );
            }
        
            case ITEM_ID_NODE_ID: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.nodeId;
                return ( ALL_OK );
            }

            case ITEM_ID_NODE_UID: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );  
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                *arg1 = nodeMap.nodeUID >> 16;
                *arg2 = nodeMap.nodeUID & 0xFFFF;
                return ( ALL_OK );
            }

            case ITEM_ID_RESTART_COUNT: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.nodeRestartCnt;
                return ( ALL_OK );
            }

            case ITEM_ID_PORT_MAP_ENTRIES: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.portMapEntries;
                return ( ALL_OK );
            }

            case ITEM_ID_EVENT_MAP_ENTRIES: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.eventMapEntries;
                return ( ALL_OK );
            }

            case ITEM_ID_ATTR_MAP_ENTRIES: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                
                *arg1 = MAX_ATTR_MAP_ENTRIES;
                return ( ALL_OK );
            }

            case ITEM_ID_GET_EVENT_MAP_ENTRY: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );  
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                return ( getMemEmapEntry( *arg1, arg1, arg2 ));
            }

            case ITEM_ID_NAME_1: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 ) {
                
                    *arg1 = ((uint16_t) ( nodeMap.name[ 0 ] << 8  ) | nodeMap.name[ 1 ] );
                    *arg2 = ((uint16_t) ( nodeMap.name[ 2 ] << 8  ) | nodeMap.name[ 3 ] );
                }
                else {

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) -1 ];

                    *arg1 = ((uint16_t) ( pEntry -> name[ 0 ] << 8  ) | pEntry -> name[ 1 ] );
                    *arg2 = ((uint16_t) ( pEntry -> name[ 2 ] << 8  ) | pEntry -> name[ 3 ] );
                }

                return ( ALL_OK );
            }

            case ITEM_ID_NAME_2: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 ) {
                
                    *arg1 = ((uint16_t) ( nodeMap.name[ 4 ] << 8  ) | nodeMap.name[ 5 ] );
                    *arg2 = ((uint16_t) ( nodeMap.name[ 6 ] << 8  ) | nodeMap.name[ 7 ] );
                }
                else {

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) - 1 ];

                    *arg1 = ((uint16_t) ( pEntry -> name[ 4 ] << 8  ) | pEntry -> name[ 5 ] );
                    *arg2 = ((uint16_t) ( pEntry -> name[ 6 ] << 8  ) | pEntry -> name[ 7 ] );
                }

                return ( ALL_OK );
            }

            case ITEM_ID_NAME_3: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 ) {
                
                    *arg1 = ((uint16_t) ( nodeMap.name[ 8 ] << 8  ) | nodeMap.name[ 9 ] );
                    *arg2 = ((uint16_t) ( nodeMap.name[ 10 ] << 8  ) | nodeMap.name[ 11 ] );
                }
                else {

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) - 1 ];

                    *arg1 = ((uint16_t) ( pEntry -> name[ 8 ] << 8  ) | pEntry -> name[ 9 ] );
                    *arg2 = ((uint16_t) ( pEntry -> name[ 10 ] << 8  ) | pEntry -> name[ 11 ] );
                }

                return ( ALL_OK );
            }

            case ITEM_ID_NAME_4: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 ) {
                
                    *arg1 = ((uint16_t) ( nodeMap.name[ 12 ] << 8  ) | nodeMap.name[ 13 ] );
                    *arg2 = ((uint16_t) ( nodeMap.name[ 14 ] << 8  ) | nodeMap.name[ 15 ] );
                }
                else {

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) - 1 ];

                    *arg1 = ((uint16_t) ( pEntry -> name[ 12 ] << 8  ) | pEntry -> name[ 13 ] );
                    *arg2 = ((uint16_t) ( pEntry -> name[ 14 ] << 8  ) | pEntry -> name[ 15 ] );
                }

                return ( ALL_OK );
            }

            case ITEM_ID_NVM_PROTECTED_ACCESS: {

                // ??? access to protected NVM data areas.
                // ??? arg 1 -> offset
                // ??? arg 2 -> value

                return ( ALL_OK );

            } break;

            case ITEM_ID_EVENT_DELAY_TICKS: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
              
                *arg1 = portMap.map[ portId( npId ) - 1 ].eventDelayTime;
                return ( ALL_OK );
            }

            default: return ( ERR_INVALID_ITEM_ID );
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// "nodePut" will write a value to the node, port or the attribute data map. The "npId" argument contains
// the node and port Id. However, we will only use the portId portion.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nodePut( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

        printf( "nodePut: 0x%x:%d:%d:%d\n", npId, item, val1, val2  );
    }

    if ( isInRangeU( item, IR_ATTR_RANGE_START, IR_ATTR_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {
            
             return ( writeAttrMem( portId( npId ), item, val1 ));
        }
        else if ( nodeMap.nodeState == NS_CONFIG ) {
        
            return ( writeAttrNvm( portId( npId ), item, val1 ));
        }
        else return ( ERR_INVALID_OP_FOR_NODE_STATE );
        
    } else {

        switch ( item ) {

            case ITEM_ID_DEBUG_MASK: {

                if ( CDC::isConsoleConnected( ))    debugMask = val1 | DBG_CONFIG;           
                else                                debugMask = val1 & ~ DBG_CONFIG;
              
                return( ALL_OK );
            }

            case ITEM_ID_VERSION: {

                nodeMap.nodeSwVersion = val1;
                return( rtNvmPutWord( nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeSwVersion ), val1 ));
            }

            case ITEM_ID_BOARD_VERSION: {

                // ??? board version setting ...

                return( 255 );
            } break;

            case ITEM_ID_CONTROLLER_FAMILY: {

                // ??? controller family should actually become part of the version and board version ...

                return( 255 );
            } break;

            case ITEM_ID_OPTIONS: {

                nodeMap.nodeOptions = val1;
                return( rtNvmPutWord( nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeOptions ), val1 ));
            }

            case ITEM_ID_FLAGS: {

                nodeMap.nodeFlags = val1;
                return( rtNvmPutWord( nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeFlags ), val1 ));
            }

            case ITEM_ID_NODE_ID: {

                nodeMap.nodeId = val1;
                return( rtNvmPutWord( nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeId ), nodeMap.nodeId ));
            }

            case ITEM_ID_TYPE: {

                if ( portId( npId ) == 0 ) {

                    nodeMap.nodeType = lowByte( val1 );
                    return( rtNvmPutWord( nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeType ), val1 ));
                }
                else {

                    portMap.map[ portId( npId ) - 1 ].type = lowByte( val1 );

                    uint16_t ofs =  nodeMap.nvmPortMapOfs +
                                    offsetof( LcsPortMap, map ) + 
                                    (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                    offsetof( LcsPortMapEntry, type );

                    return ( rtNvmPutWord( ofs, portMap.map[ portId( npId ) - 1 ].type ));
                }
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                if ( isInRangeU ( portId( npId ) - 1, 0, MAX_PORT_MAP_ENTRIES )) { 

                    portMap.map[ portId( npId ) - 1 ].eventDelayTime = val1;

                    uint16_t ofs =  nodeMap.nvmPortMapOfs +
                                    offsetof( LcsPortMap, map ) + 
                                    (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                    offsetof( LcsPortMapEntry, eventDelayTime );

                    return ( rtNvmPutWord( ofs, val1 ));
                }
                else return( ERR_INVALID_PORT_ID );
            }
            
            case ITEM_ID_NAME_1: {

                tempName[ 0 ] = highByte( val1 );
                tempName[ 1 ] = lowByte( val1 );
                tempName[ 2 ] = highByte( val2 );
                tempName[ 3 ] = lowByte( val2 );

                if ( portId( npId ) == 0 ) {

                    memcpy((uint8_t *) nodeMap.name, (uint8_t *)tempName, MAX_NODE_NAME_SIZE );
                    return( rtNvmPutBytes(  nodeMap.nvmNodeMapOfs  + offsetof( LcsNodeMap, name ), 
                                            (uint8_t *)tempName, 
                                            MAX_NODE_NAME_SIZE ));
                }
                else {

                    memcpy((uint8_t *) portMap.map[ portId( npId ) ].name, (uint8_t *)tempName, MAX_PORT_NAME_SIZE );
                    uint16_t ofs =  nodeMap.nvmPortMapOfs  +
                                    offsetof( LcsPortMap, map ) + 
                                    (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                    offsetof( LcsPortMapEntry, name );
                    return( rtNvmPutBytes( ofs, (uint8_t *)tempName, MAX_PORT_NAME_SIZE ));
                }
            }

            case ITEM_ID_NAME_2: {

                tempName[ 4 ]   = highByte( val1 );
                tempName[ 5 ]   = lowByte( val1 );
                tempName[ 6 ]   = highByte( val2 );
                tempName[ 7 ]   = lowByte( val2 );
                return ( ALL_OK );
            }

            case ITEM_ID_NAME_3: {

                tempName[ 8 ]   = highByte( val1 );
                tempName[ 9 ]   = lowByte( val1 );
                tempName[ 10 ]  = highByte( val2 );
                tempName[ 11 ]  = lowByte( val2 );
                return ( ALL_OK );
            }

            case ITEM_ID_NAME_4: {

                memset( tempName, 0, MAX_NODE_NAME_SIZE );
                tempName[ 12 ]  = highByte( val1 );
                tempName[ 13 ]  = lowByte( val1 );
                tempName[ 14 ]  = highByte( val2 );
                tempName[ 15 ]  = lowByte( val2 );
                return ( ALL_OK );
            }

            default: return ( ERR_INVALID_ITEM_ID );
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// "nodeReq" will carry out a node or port function. A function, represented by an item, can be a node or port
// defined item, or a user defined item. For the latter we will invoke the user defined callback, if any. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t nodeReq( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

        printf( "nodeReq: 0x%x:%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
    }

     if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        // ??? if the port is associated to a driver... different call ....

        return( invokeUserItemCallback( npId, item, arg1, arg2 ));
    
    } else {

        switch ( item ) {

            case ITEM_ID_RESET: {

                // ??? watchDog business ?

                return( 255 );
            }

            case ITEM_ID_FORMAT: {

                // ??? we need a way to format the extension board area, when needed. 
                // ??? applies to ports 1 to 4 when they are mapped to a driver...

                return( 255 );
            } break;

            case ITEM_ID_ADD_EVENT_MAP_ENTRY: {

                return ( addEvent( *arg1, *arg2 ));
            }

            case ITEM_ID_DEL_EVENT_MAP_ENTRY: {

                return ( removeEvent( *arg1 ));
            }

            case ITEM_ID_SYNC: {

                // ??? options what to sync ? For now it is only the event map...
                // ??? use arg 1 as an option number... ?
                return( syncEventMap( ));
            }

            case ITEM_ID_NODE_ID: {

                if ( isInRangeU( *arg1, MIN_NODE_ID, MAX_NODE_ID )) {

                    nodeMap.nodeId = nodeId( *arg1 );
                    return( rtNvmPutBytes(  nodeMap.nvmNodeMapOfs + offsetof( LcsNodeMap, nodeId ), 
                                            (uint8_t *) &nodeMap.nodeId, 
                                            sizeof( uint16_t )));
                }
                else return ( ERR_INVALID_NODE_ID );
            }

            case ITEM_ID_ENABLE_EVENT_PROCESSING: {

                if ( *arg1 )    portMap.map[ portId( npId ) - 1 ].flags |= PF_PORT_EVENT_HANDLING_ENABLED;
                else            portMap.map[ portId( npId ) - 1 ].flags &= ~ PF_PORT_EVENT_HANDLING_ENABLED;

                return ( ALL_OK );
            }

            case ITEM_ID_SET_READY_LED: {

                return ( CDC::writeDio( cdcMap.cfg.READY_LED_PIN, *arg1 ));
            }

            case ITEM_ID_SET_ACTIVITY_LED: {

                return ( CDC::writeDio( cdcMap.cfg.ACTIVE_LED_PIN, *arg1 ));
            }

            case ITEM_ID_TOGGLE_READY_LED:  {

                return ( CDC::toggleDio( cdcMap.cfg.READY_LED_PIN ));
            }

            case ITEM_ID_TOGGLE_ACTIVITY_LED: {

                return ( CDC::toggleDio( cdcMap.cfg.ACTIVE_LED_PIN ));
            }

            default: return ( ERR_INVALID_ITEM_ID );
        }
    }
}

//------------------------------------------------------------------------------------------------------------
//
// ??? how about a routine to update a mask ? We often have the case to set / clear a bit in a mask. We 
// could pass the mask with the bit position set to set or clear the mask word.
//
//------------------------------------------------------------------------------------------------------------

} // namespace LCS
