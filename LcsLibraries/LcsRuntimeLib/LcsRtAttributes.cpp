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
    extern LcsEventMap          eventMap;

    extern uint8_t              addEvent( uint16_t eventId, uint16_t eventMask );
    extern uint8_t              removeEvent( uint16_t eventId );
    extern uint8_t              syncEventMap( );
    extern uint8_t              getMemEmapEntry( uint16_t index, uint16_t *eventId, uint16_t *eventMask );
    extern uint8_t              rtNvmPutWord( uint32_t ofs, uint16_t word );
    extern uint8_t              rtNvmGetWord( uint32_t ofs, uint16_t *word );
    extern uint8_t              rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
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
    char tempName[ MAX_NODE_PORT_NAME_SIZE + 1 ] = { 0 };

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
        uint16_t ofs    = NVM_NODE_DATA_OFS + offsetof( LcsNodeData, map ) + 
                            (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        printf( "Ofs: 0x%x\n", ofs );
        uint8_t rStat = rtNvmGetWord( ofs, &nodeData.map[ block ][ index ] );
       
        *arg = (( rStat == ALL_OK ) ? nodeData.map[ block ][ index ] : 0 );

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

            printf( "readAttrNvm: ret: %d\n", rStat );
        }

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

            printf( "writeAttrNvm: block: 0x%x, item: %d\n", block, item );
        }

        uint16_t index  = item - IR_ATTR_RANGE_START;
        uint16_t ofs    = NVM_NODE_DATA_OFS + offsetof( LcsNodeData, map ) + 
                            (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        printf( "Ofs: 0x%x\n", ofs );
        nodeData.map[ block ][ index ] = arg;
        uint8_t rStat = rtNvmPutWord( ofs, arg );

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_ATTRIBUTES )) {

            printf( "writeAttrNvm: ret: %d\n", rStat );
        }

        return( rStat );
    }

    //--------------------------------------------------------------------------------------------------------
    // User callback function invocation routine. Items 64 to 127 are user defined items. We will simply
    // invoke a previously registered callback passing the arguments. Note that a user call back and a driver
    // call for extension boards mapped to P1 .. P4 are the same.
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t invokeUserItemCallback( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

        if ( isInRangeU( portId, 0, MAX_PORT_ID )) {

            if ( portMap.map[ portId ].reqCallback != nullptr ) {

                return ( portMap.map[ portId ].reqCallback( portId, item, arg1, arg2 ));
            }
            else return( ERR_INVALID_ITEM_ID );
        }
        else return( ERR_INVALID_ITEM_ID );
    }

    //--------------------------------------------------------------------------------------------------------
    //
    //
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t handleSyncCommand( uint8_t portId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

        // ??? options what to sync ? For now it is only the event map...
        // ??? use arg 1 as an option number... ?

        // arg1 -> sync command ( e.g. sync eventMap or sync to NVM )
        // arg2 -> 128 to 255 variable item

        syncEventMap( ); // just a quick test ....

        return( ALL_OK );
    }
    
} // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "nodeGet" will lookup a value from the node, port or the attribute data map. The "npId" argument contains
// the node and port Id. However, we will only use the portId portion, which represents the block index. For
// data attributes the node state determines whether we just access the MEM attribute or sync the MEM with 
// the NVM version of the data first. For the other node or port reserved attributes the MEM version is used.
// The remainder of items to get are straightforward, every item can be read regardless of node state. 
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
            
            return ( readAttrMem(  portId( npId ), item, arg1 ));
        }
        else if ( nodeMap.nodeState == NS_CONFIG ) {
        
            return ( readAttrNvm(  portId( npId ), item, arg1 ));
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

                *arg1 = portMap.map[  portId( npId ) ].options;
                return ( ALL_OK );
            }

            case ITEM_ID_FLAGS: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = portMap.map[  portId( npId ) ].flags;
                return ( ALL_OK );
            }

            case ITEM_ID_TYPE: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = portMap.map[  portId( npId ) ].flags;
                return ( ALL_OK );
            }

            case ITEM_ID_VERSION: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                *arg1 = nodeMap.nodeSwVersion;
                return ( ALL_OK );          
            }

            case ITEM_ID_BOARD_VERSION: {

                if (( arg1 == nullptr ) || ( ! isInRangeU( *arg1, 0, 4 ))) return( ERR_INVALID_ATTR_ARG );

                *arg1 = nvmHeaderMap.map[ *arg1 ].boardVersion ;
                return ( ALL_OK );  

            } break;

            case ITEM_ID_CONTROLLER_FAMILY: {

                if (( arg1 == nullptr ) || ( ! isInRangeU( *arg1, 0, 4 ))) return( ERR_INVALID_ATTR_ARG );

                *arg1 = nvmHeaderMap.map[ *arg1 ].controllerFamily;
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
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = MAX_PORT_MAP_ENTRIES;
                *arg2 = portMap.mapHwm;
                return ( ALL_OK );
            }

            case ITEM_ID_EVENT_MAP_ENTRIES: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                *arg1 = MAX_EVENT_MAP_ENTRIES;
                *arg2 = eventMap.mapHwm;
                return ( ALL_OK );
            }

            case ITEM_ID_ATTR_MAP_ENTRIES: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                
                *arg1 = MAX_ATTR_MAP_ENTRIES;
                *arg2 = MAX_ATTR_MAP_ENTRIES;
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

                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 0 ] << 8  ) | pPtr -> name[ 1 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 2 ] << 8  ) | pPtr -> name[ 3 ] );
                return ( ALL_OK );
            }

            case ITEM_ID_NAME_2: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG );

                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 4 ] << 8  ) | pPtr -> name[ 5 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 6 ] << 8  ) | pPtr -> name[ 7 ] );
                return ( ALL_OK );
            }

            case ITEM_ID_NAME_3: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 8 ] << 8  )  | pPtr -> name[ 9 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 10 ] << 8  ) | pPtr -> name[ 11 ] );
                return ( ALL_OK );
            }

            case ITEM_ID_NAME_4: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
                if ( arg2 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 

                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 12 ] << 8  ) | pPtr -> name[ 13 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 14 ] << 8  ) | pPtr -> name[ 15 ] );
                return ( ALL_OK );
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                if ( arg1 == nullptr ) return( ERR_INVALID_ATTR_ARG ); 
              
                *arg1 = portMap.map[  portId( npId ) ].eventDelayTime;
                return ( ALL_OK );
            }

            default: return ( ERR_INVALID_ITEM_ID );
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// "nodePut" will write a value to the node, port or the attribute data map. The "npId" argument contains
// the node and port Id. For data attributes the node state determines whether we also update the NVM slot.
// For the remaining items the update of NVM is dependent on the meaning of the particular item.
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

            case ITEM_ID_OPTIONS: {

                portMap.map[ 0 ].options = val1;

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, options );

                return( rtNvmPutWord( ofs, val1 ));
            }

            case ITEM_ID_FLAGS: {

                portMap.map[ 0 ].flags = val1;

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, options );

                return( rtNvmPutWord( ofs, val1 ));
            }

            case ITEM_ID_TYPE: {

                portMap.map[ portId( npId ) - 1 ].type = lowByte( val1 );

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, type );

                return ( rtNvmPutWord( ofs, portMap.map[ portId( npId ) - 1 ].type ));
            }



            
            case ITEM_ID_VERSION: {

                nodeMap.nodeSwVersion = val1;
                return( rtNvmPutWord( NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeSwVersion ), val1 ));
            }

            case ITEM_ID_BOARD_VERSION: {

                // ??? board version setting ...

                return( 255 );
            } break;

            // case ITEM_ID_BOARD_TYPE: {

                // ??? board version setting ...

               // return( 255 );
            // }

            case ITEM_ID_CONTROLLER_FAMILY: {

                // ??? controller family should actually become part of the version and board version ...

                return( 255 );
            }

            case ITEM_ID_NODE_ID: {

                nodeMap.nodeId = val1;
                return( rtNvmPutWord( NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId ), nodeMap.nodeId ));
            }

            

            case ITEM_ID_EVENT_DELAY_TICKS: {

                portMap.map[ portId( npId ) - 1 ].eventDelayTime = val1;

                uint16_t ofs =  NVM_PORT_MAP_OFS +
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

                memcpy((uint8_t *) portMap.map[ portId( npId ) ].name, (uint8_t *)tempName, MAX_NODE_PORT_NAME_SIZE );
                uint16_t ofs =  NVM_PORT_MAP_OFS  +
                                offsetof( LcsPortMap, map ) + 
                                (( portId( npId ) - 1 ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, name );
                
                return( rtNvmPutBytes( ofs, (uint8_t *)tempName, MAX_NODE_PORT_NAME_SIZE ));
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

                memset( tempName, 0, MAX_NODE_PORT_NAME_SIZE );
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

                return( handleSyncCommand( npId, item, *arg1, *arg2 ));
            }

            case ITEM_ID_NODE_ID: {

                if ( isInRangeU( *arg1, MIN_NODE_ID, MAX_NODE_ID )) {

                    nodeMap.nodeId = nodeId( *arg1 );
                    return( rtNvmPutBytes(  NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId ), 
                                            (uint8_t *) &nodeMap.nodeId, 
                                            sizeof( uint16_t )));
                }
                else return ( ERR_INVALID_NODE_ID );
            }

            case ITEM_ID_ENABLE_EVENT_PROCESSING: {

                if ( *arg1 )    portMap.map[ portId( npId ) - 1 ].flags |= NPF_PORT_EVENT_HANDLING_ENABLED;
                else            portMap.map[ portId( npId ) - 1 ].flags &= ~ NPF_PORT_EVENT_HANDLING_ENABLED;

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
