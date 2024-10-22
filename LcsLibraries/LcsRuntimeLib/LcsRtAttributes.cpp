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
// External declaration to global structures defined in "LcsRtSetup".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t                     debugMask;
    extern LCS::LcsCdcDesc              cdcMap;
    extern LCS::LcsNodeMap              nodeMap;
    extern LCS::LcsNodeData             nodeData;
    extern LCS::LcsPortMap              portMap;
    extern LCS::LcsCallbackMap          callbackMap;
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

        *arg = nodeData.map[ block ][ item - IR_ATTR_MEM_RANGE_START ];
        return ( LCS::ALL_OK );
    }

    //----------------------------------------------------------------------------------------------------------
    // "writeAttrMem" stores a value to a node or port attribute map in MEM. As an internal function, we 
    // expect a valid block and item argument. The "block" argument will refer to the node and port data
    // attributes. Block 0 is the node, all others the port. 
    //
    //----------------------------------------------------------------------------------------------------------
    uint8_t writeAttrMem( uint8_t block, uint8_t item, uint16_t arg ) {

        nodeData.map[ block ][ item - IR_ATTR_MEM_RANGE_START ] = arg;
        return ( LCS::ALL_OK );
    }

    //--------------------------------------------------------------------------------------------------------
    // "readAttrNvm" gets an attribute from the NVM storage. We read the value from NVM, store it in the MEM 
    // counterpart and then return it. For the NVM access, the byte offset into the storage needs to be 
    // computed. As an internal function, we expect a valid block and item argument.
    //
    //----------------------------------------------------------------------------------------------------------
    uint8_t readAttrNvm( uint8_t block, uint8_t item, uint16_t *arg ) {

        uint16_t index  = item - IR_ATTR_NVM_RANGE_START;
        uint16_t ofs    = (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );
        uint8_t  rStat  = rtNvmGetWord( ofs, &nodeData.map[ block ][ index ] );

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

        uint16_t index  = item - IR_ATTR_NVM_RANGE_START;
        uint16_t ofs    = (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        nodeData.map[ block ][ index ] = arg;
        return ( rtNvmPutWord( ofs, arg ));
    }

    //--------------------------------------------------------------------------------------------------------
    // User callback function invocation routine. Items 64 to 127 are user defined items. We will simply
    // invoke a previously registered callback passing the arguments.
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t invokeUserItemCallback( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

        if ( callbackMap.reqCallback != nullptr ) {

            return ( callbackMap.reqCallback( portId, item, arg1, arg2 ));
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

    if ( isInRangeU( item, IR_ATTR_MEM_RANGE_START, IR_ATTR_MEM_RANGE_END )) {

        return ( readAttrMem( portId( npId ), item, arg1 ));
    }
    else if ( isInRangeU( item, IR_ATTR_NVM_RANGE_START, IR_ATTR_NVM_RANGE_END )) {

        return ( readAttrNvm( portId( npId ), item, arg1 ));
    }
    else {

        switch ( item ) {

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

            case ITEM_ID_NODE_UID: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );  
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                *arg1 = nodeMap.nodeUID >> 16;
                *arg2 = nodeMap.nodeUID & 0xFFFF;
                return ( ALL_OK );
            }

            case ITEM_ID_NODE_ID: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.nodeId;
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

            case ITEM_ID_RESTART_COUNT: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = nodeMap.nodeRestartCnt;
                return ( ALL_OK );
            }

            case ITEM_ID_GET_EVENT_MAP_ENTRY: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );  
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                return ( getMemEmapEntry( *arg1, arg1, arg2 ));
            }

            case ITEM_ID_TYPE: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                if ( portId( npId ) == 0 )   *arg1 = nodeMap. nodeType;
                else                         *arg1 = portMap.map[ portId( npId ) - 1 ].type;

                return ( ALL_OK );
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

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) -1 ];

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

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) -1 ];

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

                    LcsPortMapEntry *pEntry = &portMap.map[ portId( npId ) -1 ];

                    *arg1 = ((uint16_t) ( pEntry -> name[ 12 ] << 8  ) | pEntry -> name[ 13 ] );
                    *arg2 = ((uint16_t) ( pEntry -> name[ 14 ] << 8  ) | pEntry -> name[ 15 ] );
                }

                return ( ALL_OK );
            }

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

   if ( isInRangeU( item, IR_ATTR_MEM_RANGE_START, IR_ATTR_MEM_RANGE_END )) {

        return ( writeAttrMem( portId( npId ), item, val1 ));
    }
    else if ( isInRangeU( item, IR_ATTR_NVM_RANGE_START, IR_ATTR_NVM_RANGE_END )) {

        return ( writeAttrNvm( portId( npId ), item, val1 ));
    }
    else {

        switch ( item ) {

            case ITEM_ID_TYPE: {

                if ( portId( npId ) == 0 ) {

                    nodeMap.nodeType = lowByte( val1 );
                    return( rtNvmPutWord( NVM_NODE_MAP_START + offsetof( LcsNodeMap, nodeType ), nodeMap.nodeType ));
                }
                else {

                    portMap.map[ portId( npId ) - 1 ].type = lowByte( val1 );

                    uint16_t ofs =  NVM_PORT_MAP_START +
                                    offsetof( LcsPortMap, map ) + 
                                    (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                    offsetof( LcsPortMapEntry, type );

                    return ( rtNvmPutWord( ofs, portMap.map[ portId( npId ) - 1 ].type ));
                }
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                portMap.map[ portId( npId ) - 1 ].eventDelayTime = val1;

                uint16_t ofs =  NVM_PORT_MAP_START +
                                offsetof( LcsPortMap, map ) + 
                                (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, eventDelayTime );

                return ( rtNvmPutWord( ofs, val1 ));
            }
            
            case ITEM_ID_NAME_1: {

                tempName[ 0 ] = highByte( val1 );
                tempName[ 1 ] = lowByte( val1 );
                tempName[ 2 ] = highByte( val2 );
                tempName[ 3 ] = lowByte( val2 );

                if ( portId( npId ) == 0 ) {

                    memcpy((uint8_t *) nodeMap.name, (uint8_t *)tempName, MAX_NODE_NAME_SIZE );
                    return( rtNvmPutBytes(  NVM_NODE_MAP_START + offsetof( LcsNodeMap, name ), 
                                            (uint8_t *)tempName, 
                                            MAX_NODE_NAME_SIZE ));
                }
                else {

                    memcpy((uint8_t *) portMap.map[ portId( npId ) ].name, (uint8_t *)tempName, MAX_PORT_NAME_SIZE );
                    uint16_t ofs =  NVM_PORT_MAP_START +
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
// ??? have an option to set the debug level ?
//------------------------------------------------------------------------------------------------------------
uint8_t nodeReq( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

        printf( "nodeReq: 0x%x:%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
    }

     if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        return( invokeUserItemCallback( npId, item, arg1, arg2 ));
    }
    else {

        switch ( item ) {

            case ITEM_ID_RESET: {

                debugMask = *arg1;
                return ( resetNode( npId ));
            }

            case ITEM_ID_ADD_EVENT_MAP_ENTRY: {

                return ( addEvent( *arg1, *arg2 ));
            }

            case ITEM_ID_DEL_EVENT_MAP_ENTRY: {

                return ( removeEvent( *arg1, *arg2 ));
            }

            case ITEM_ID_SYNC: {

                // ??? options what to sync ? For now it is only the event map...
                // ??? use arg 1 as an option number... ?
                return( syncEventMap( ));
            }

            case ITEM_ID_NODE_ID: {

                if ( isInRangeU( *arg1, MIN_NODE_ID, MAX_NODE_ID )) {

                    nodeMap.nodeId = nodeId( *arg1 );
                    return( rtNvmPutBytes(  NVM_NODE_MAP_START +
                                            offsetof( LcsNodeMap, nodeId ), 
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

            case ITEM_ID_BLINK_READY_LED:
            case ITEM_ID_BLINK_ACTIVITY_LED: {

                return ( ERR_NOT_IMPLEMENTED );
            }

            default: return ( ERR_INVALID_ITEM_ID );
        }
    }
}

} // namespace LCS
