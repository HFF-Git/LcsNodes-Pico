//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime attribute management
//
//----------------------------------------------------------------------------------------
// This file contains the LCS runtime routines that implement node attribute access. 
// There are  three routines that allow to manipulate node and port data as well as  
// issue requests to a node or port. The "npId" will indicate which node and port the 
// call refers to. The node portion is typically our own node Id, the port Id refers
// to a ports on the node, with a port Id of zero referring to the node itself. Any 
// node can access another node. In this case request come via a message and the 
// message handler will call the local routines in this file. 
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime attribute management
// Copyright (C) 2022 - 2025 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should
// have received a copy of the GNU General Public License along with this program. 
// If not, see <http://www.gnu.org/licenses/>.
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
    extern LcsHeaderMap     headerMap;
    extern LcsNodeMap       nodeMap;
    extern LcsNodeData      nodeData;
    extern LcsPortMap       portMap;
    extern LcsEventMap      eventMap;

    extern uint8_t          addEvent( uint16_t eventId, uint16_t eventMask );
    extern uint8_t          removeEvent( uint16_t eventId );

    extern uint8_t          getMemEmapEntry( uint16_t index, 
                                             uint16_t *eventId, 
                                             uint16_t *eventMask );
    extern uint8_t          syncEventMap( );

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

    //------------------------------------------------------------------------------------
    // The node or port name cannot be set with a single LCS message. We will 
    // store the parts in this temporary buffer and set the name when all parts
    // are received.
    //
    //------------------------------------------------------------------------------------
    char tempName[ MAX_NODE_PORT_NAME_SIZE + 1 ] = { 0 };

    //------------------------------------------------------------------------------------
    // Utility routines.
    //
    //------------------------------------------------------------------------------------
    bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

        return (( val >= lower ) && ( val <= upper ));
    }

    uint8_t lowByte( uint16_t arg ) { 
        
        return ( arg & 0xFF ); 
    }
    
    uint8_t highByte( uint16_t arg ) { 
        
        return ( arg >> 8 ); 
    }

     uint16_t nodeId( uint16_t arg ) {

        return ( arg >> 4 );
    }

    uint16_t portId( uint16_t arg ) {

        return ( arg & 0xF );
    }

    uint8_t errStat( uint8_t errId ) {

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "Ret: %d\n", errId );
        }

        return ( errId );
    }

    //------------------------------------------------------------------------------------
    // "readAttrMem" gets a value from the node or port attribute map in MEM. As an 
    // internal function, we expect a valid block and item argument. The "block"
    // argument will refer to the node and port data attributes. 
    //
    //------------------------------------------------------------------------------------
    uint8_t readAttrMem( uint8_t block, uint8_t item, uint16_t *arg ) {

        *arg = nodeData.map[ block ][ item - IR_USER_RANGE_START ];

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "readAttrMem: block: 0x%x, item: %d, data: 0x%x\n", 
                    block, item, *arg );
        }

        return ( errStat( ALL_OK ));
    }

    //------------------------------------------------------------------------------------
    // "writeAttrMem" stores a value to a node or port attribute map in MEM. As an 
    // internal function, we expect a valid block and item argument. The "block" 
    // argument will refer to the node and port data attributes. 
    //
    //------------------------------------------------------------------------------------
    uint8_t writeAttrMem( uint8_t block, uint8_t item, uint16_t arg ) {

        nodeData.map[ block ][ item - IR_USER_RANGE_START ] = arg;

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "writeAttrMem: block: 0x%x, item: %d, data: 0x%x\n", 
                    block, item, arg );
        }

        return ( errStat( ALL_OK ));
    }

    //------------------------------------------------------------------------------------
    // "readAttrNvm" gets an attribute from the NVM storage. We read the value from
    // the NVM area. If successful, we also store it in the MEM counterpart and then 
    // return it. This ensures that NVM and MEM are always in sync when accessing NVM.
    // For NVM access, the byte offset into the storage needs to be computed. As an 
    // internal function, we expect a valid block and item argument.
    //
    //------------------------------------------------------------------------------------
    uint8_t readAttrNvm( uint8_t block, uint8_t item, uint16_t *arg ) {

        uint16_t index  = item - IR_USER_RANGE_START;
        uint16_t ofs    = NVM_NODE_DATA_OFS + offsetof( LcsNodeData, map ) + 
            (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "readAttrNvm: block: 0x%x, item: %d, "
                    "nvm-ofs: 0x%x, data: 0x%x\n",
                     block, item, ofs, *arg );
        }

        uint8_t rStat = rtNvmGetWord( ofs, arg );
        if ( rStat == ALL_OK ) rStat = writeAttrMem( block, item, *arg );
        
        return ( errStat( rStat ));
    }

    //------------------------------------------------------------------------------------
    // "writeAttrNvm" stores an attribute to the NVM storage. If the update is 
    // successful, we also update the corresponding MEM attribute. This ensures that 
    // NVM and MEM are always in sync when accessing the NVM. For the NVM access, the
    // byte offset into the storage needs to be computed. As an  internal function, 
    // we expect a valid block and item argument.
    //
    //------------------------------------------------------------------------------------
    uint8_t writeAttrNvm( uint8_t block, uint8_t item, uint16_t arg ) {

        uint16_t index  = item - IR_USER_RANGE_START;
        uint16_t ofs    = NVM_NODE_DATA_OFS + offsetof( LcsNodeData, map ) + 
            (( block * MAX_ATTR_MAP_ENTRIES ) + index  ) * sizeof( uint16_t );
        
        if (( debugMask & LCS_DBG_ATTRIBUTES ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "writeAttrNvm: block: 0x%x, item: %d,"
                    " nvm-ofs: 0x%x, data: 0x%x\n",
                    block, item, ofs, arg );
        }

        uint8_t rStat = rtNvmPutWord( ofs, arg );
        if ( rStat == ALL_OK ) rStat = writeAttrMem( block, item, arg );
        
        return ( errStat( rStat ));
    }

    //------------------------------------------------------------------------------------
    // "syncAttrToMem" will copy the NVM attribute value to the MEM counterpart. 
    // All we do is just reading the NVM value to a dummy variable.
    //
    //------------------------------------------------------------------------------------
    uint8_t syncAttrToMem( uint8_t block, uint8_t item ) {

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "syncAttrToMem: block: 0x%x, item: %d\n", block, item );
        }

        uint16_t arg = 0;
        return ( errStat( readAttrNvm( block, item, &arg )));
    }

    //------------------------------------------------------------------------------------
    // "syncAttrToNvm" will take the MEM attribute value of an item and writes it 
    // to the NVM counterpart.
    //
    //------------------------------------------------------------------------------------
    uint8_t syncAttrToNvm( uint8_t block, uint8_t item ) {

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "syncAttrToNvm: block: 0x%x, item: %d\n", block, item );
        }

        uint16_t    arg     = 0;
        uint8_t     rStat   = readAttrMem( block, item, &arg );
        if ( rStat == ALL_OK ) rStat = writeAttrNvm( block, item, arg );
        return ( errStat( rStat ));
    }

    //------------------------------------------------------------------------------------
    // User callback function invocation routine. Items 64 to 127 are user defined
    // items. We will simply invoke a previously registered callback passing the 
    // arguments. 
    //
    //------------------------------------------------------------------------------------
    uint8_t invokeUserItemCallback( uint8_t npId, 
                                    uint8_t item, 
                                    uint16_t *arg1, 
                                    uint16_t *arg2 ) {

        if ( portMap.map[ portId( npId ) ].reqCallback != nullptr ) {

            return ( errStat( portMap.map[ portId( npId ) ].
                                    reqCallback( portId( npId ), item, arg1, arg2 )));
        }
        else return ( errStat( ERR_INVALID_ITEM_ID ));
    }

    //------------------------------------------------------------------------------------
    // "handleSyncCommand" is the handler for SYNC options. Arg1 will contain the 
    // command, arg2 an optional argument.
    //
    //------------------------------------------------------------------------------------
    uint8_t handleSyncCommand( uint8_t npId, uint16_t arg1, uint16_t arg2 ) {

        if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

            printf( "handleSyncCommand: npId: 0x%x, arg1: %d, arg2: %d\n", 
                    npId, arg1, arg2 );
        }

        switch ( arg1 ) {

            case 1:  return ( errStat( syncEventMap( ))); 

            case 2: {

                if ( isInRangeU( arg2, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

                   return ( errStat( syncAttrToMem( portId( npId ), arg2 )));
                } 
                else return ( errStat( ERR_INVALID_ITEM_ID ));
            }

            case 3: {

                if ( isInRangeU( arg2, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

                   return ( errStat( syncAttrToNvm( portId( npId ), arg2 )));
                } 
                else return ( errStat( ERR_INVALID_ITEM_ID ));
            }

            default: return ( errStat( 255 ));
        }
    }
    
} // namespace


//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "nodeGet" will lookup a value from the node, port or the attribute data map. The 
// "npId" argument contains the node and port Id. However, we will only use the portId
// portion, which represents the block index. For data attribute items the node state
// determines whether we just access the MEM attribute or the NVM version of the data. 
// For the other node or port reserved attributes the MEM version is used.
//
//----------------------------------------------------------------------------------------
uint8_t nodeGet( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

        printf( "nodeGet: npId: 0x%x, item: :%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
    }

    if (( nodeMap.nodeState != NS_OPERATE) && ( nodeMap.nodeState != NS_CONFIG )) {

        return ( errStat( ERR_LIB_NOT_READY ));
    }

    if ( arg1 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));  
    
    if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) 
            return ( errStat( readAttrMem( portId( npId ), item, arg1 )));
        else if ( nodeMap.nodeState == NS_CONFIG  ) 
            return ( errStat( readAttrNvm( portId( npId ), item, arg1 )));
        else                                        
            return ( errStat( ERR_INVALID_OP_FOR_NODE_STATE ));
        
    } else {

        switch ( item ) {

            case ITEM_ID_DEBUG_MASK: {   
                
                *arg1 = debugMask; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_OPTIONS: {      
                
                *arg1 = portMap.map[ portId( npId ) ].options; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_FLAGS: { 
                
                *arg1 = portMap.map[ portId( npId ) ].flags; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_TYPE: { 
                         
                *arg1 = portMap.map[ portId( npId ) ].type; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_RT_LIB_VERSION: {    
                
                *arg1 = nodeMap.rtLibSwVersion; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_RT_LIB_PATCH_LEVEL: {    
                
                *arg1 = nodeMap.rtLibSwPatchLevel; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NODE_STATE: {   
                
                *arg1 = nodeMap.nodeState; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NODE_ID: {      
                
                *arg1 = nodeMap.nodeId; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_RESTART_COUNT: {
                
                *arg1 = nodeMap.nodeRestartCnt; 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_BOARD_VERSION: {

                if ( isInRangeU( *arg1, 0, 4 )) {

                    *arg1 = headerMap.map[ *arg1 ].boardVersion ;
                    return ( errStat( ALL_OK ));
                }
                else return ( errStat( ERR_INVALID_ATTR_ARG ));
            }

            case ITEM_ID_NODE_UID: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                *arg1 = nodeMap.nodeUID >> 16;
                *arg2 = nodeMap.nodeUID & 0xFFFF;
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_PORT_MAP_ENTRIES: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                *arg1 = MAX_PORT_MAP_ENTRIES;
                *arg2 = portMap.mapHwm;
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_EVENT_MAP_ENTRIES: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                *arg1 = MAX_EVENT_MAP_ENTRIES;
                *arg2 = eventMap.mapHwm;
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_ATTR_MAP_ENTRIES: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                *arg1 = MAX_ATTR_MAP_ENTRIES;
                *arg2 = MAX_ATTR_MAP_ENTRIES;
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_GET_EVENT_MAP_ENTRY: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                return ( getMemEmapEntry( *arg1, arg1, arg2 ));
            }

            case ITEM_ID_NAME_1: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 0 ] << 8  ) | pPtr -> name[ 1 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 2 ] << 8  ) | pPtr -> name[ 3 ] );
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NAME_2: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 4 ] << 8  ) | pPtr -> name[ 5 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 6 ] << 8  ) | pPtr -> name[ 7 ] );
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NAME_3: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG )); 
                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 8 ] << 8  )  | pPtr -> name[ 9 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 10 ] << 8  ) | pPtr -> name[ 11 ] );
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NAME_4: {

                if ( arg2 == nullptr ) return ( errStat( ERR_INVALID_ATTR_ARG ));
                LcsPortMapEntry *pPtr = &portMap.map[ portId( npId ) ];
                *arg1 = ((uint16_t) ( pPtr -> name[ 12 ] << 8  ) | pPtr -> name[ 13 ] );
                *arg2 = ((uint16_t) ( pPtr -> name[ 14 ] << 8  ) | pPtr -> name[ 15 ] );
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                *arg1 = portMap.map[ portId( npId ) ].eventDelayTime;
                return ( errStat( ALL_OK ));
            }

            case ITEM_USER_MAP_AREA: {

                // ??? fetch the data word from the user area...
                return ( errStat( ALL_OK ));
            }

            default: return ( errStat( ERR_INVALID_ITEM_ID ));
        }
    }
}

//----------------------------------------------------------------------------------------
// "nodePut" will write a value to the node, port or the attribute data map. The 
// "npId" argument contains the node and port Id. For data attributes the node state
// determines whether we also update the NVM slot. For the remaining items the update
// of NVM is dependent on the meaning of the particular item.
//
//----------------------------------------------------------------------------------------
uint8_t nodePut( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

        printf( "nodePut: npId: 0x%x, item: %d, val1:%d, val2: %d\n",
                npId, item, val1, val2  );
    }

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) {
        
        return ( errStat( ERR_LIB_NOT_READY ));
    }
    
    if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE )
            return ( writeAttrMem( portId( npId ), item, val1 ));
        else if ( nodeMap.nodeState == NS_CONFIG )  
            return ( writeAttrNvm( portId( npId ), item, val1 ));
        else                                       
             return ( errStat( ERR_INVALID_OP_FOR_NODE_STATE ));
        
    } else {

        switch ( item ) {

            case ITEM_ID_DEBUG_MASK: {

                if ( isConsoleConnected( )) debugMask = val1 | LCS_DBG_CONFIG;           
                else                        debugMask = val1 & ~ LCS_DBG_CONFIG;
              
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_OPTIONS: {

                portMap.map[ portId( npId ) ].options = val1;

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, options );

                return ( errStat( rtNvmPutWord( ofs, val1 )));
            }

            case ITEM_ID_FLAGS: {

                portMap.map[ portId( npId ) ].flags = val1;

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, flags );

                return ( errStat( rtNvmPutWord( ofs, val1 )));
            }

            case ITEM_ID_TYPE: {

                portMap.map[ portId( npId ) ].type = lowByte( val1 );

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, type );

                return ( errStat( rtNvmPutWord( ofs, 
                    portMap.map[ portId( npId ) ].type )));
            }

            case ITEM_ID_NODE_ID: {

                nodeMap.nodeId = nodeId( val1 );
                return ( errStat( 
                            rtNvmPutWord( 
                                NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId ), 
                                val1 )));
            }

            case ITEM_ID_RT_LIB_VERSION: {

                nodeMap.rtLibSwVersion = val1;
                return ( errStat( 
                            rtNvmPutWord( 
                                NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, rtLibSwVersion ), 
                                val1 )));
            }

            case ITEM_ID_BOARD_VERSION: {

                if ( ! isInRangeU( val1, 0, 4 )) return ( errStat( ERR_INVALID_ATTR_ARG ));

                // ??? board version setting ...
                // ??? only in config mode ?
                // ??? what would we exactly do for the NVM header changes / config ?

                return ( errStat( 255 ));
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                portMap.map[ portId( npId ) ].eventDelayTime = val1;

                uint16_t ofs =  NVM_PORT_MAP_OFS +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, eventDelayTime );

                return ( errStat( rtNvmPutWord( ofs, val1 )));
            }
            
            case ITEM_ID_NAME_1: {

                memset( tempName, 0, MAX_NODE_PORT_NAME_SIZE );
                tempName[ 0 ] = highByte( val1 );
                tempName[ 1 ] = lowByte( val1 );
                tempName[ 2 ] = highByte( val2 );
                tempName[ 3 ] = lowByte( val2 );
            }

            case ITEM_ID_NAME_2: {

                tempName[ 4 ]   = highByte( val1 );
                tempName[ 5 ]   = lowByte( val1 );
                tempName[ 6 ]   = highByte( val2 );
                tempName[ 7 ]   = lowByte( val2 );
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NAME_3: {

                tempName[ 8 ]   = highByte( val1 );
                tempName[ 9 ]   = lowByte( val1 );
                tempName[ 10 ]  = highByte( val2 );
                tempName[ 11 ]  = lowByte( val2 );
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_NAME_4: {

                tempName[ 12 ]  = highByte( val1 );
                tempName[ 13 ]  = lowByte( val1 );
                tempName[ 14 ]  = highByte( val2 );
                tempName[ 15 ]  = lowByte( val2 );

                memcpy((uint8_t *) portMap.map[ portId( npId ) ].name, 
                       (uint8_t *)tempName, 
                       MAX_NODE_PORT_NAME_SIZE );

                uint16_t ofs =  NVM_PORT_MAP_OFS  +
                                offsetof( LcsPortMap, map ) + 
                                ( portId( npId ) * sizeof( LcsPortMapEntry )) +
                                offsetof( LcsPortMapEntry, name );
                
                return ( errStat( 
                            rtNvmPutBytes( ofs, 
                                        (uint8_t *)tempName, 
                                        MAX_NODE_PORT_NAME_SIZE + 1 )));

                return ( errStat( ALL_OK ));
            }

            case ITEM_USER_MAP_AREA: {

                // ??? set the data word in the user area...
                return ( errStat( ALL_OK ));
            }

            default: return ( errStat( ERR_INVALID_ITEM_ID ));
        }
    }
}

//----------------------------------------------------------------------------------------
// "nodeReq" will carry out a node or port function. A function, represented by an
// item, can be a node or port defined item, a extension board driver defined item 
// or a user defined item.
//
//----------------------------------------------------------------------------------------
uint8_t nodeReq( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_ATTRIBUTES )) {

        printf( "nodeReq: 0x%x:%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
    }

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) {
        
        return ( errStat( ERR_LIB_NOT_READY ));
    }
    
    if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        return ( errStat( invokeUserItemCallback( npId, item, arg1, arg2 )));
    
    } else {

        switch ( item ) {

            case ITEM_ID_RESET: {

                // pass the watchdog timer ... is there a better way ?
                sleepMillis( 10000 ); 
                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_FORMAT: {

                // ??? we need a way to format the extension board area, when needed. 
                // ??? applies to ports 1 to 4 when they are mapped to a driver...

                // val1 = type ?
                // val2 = ?

                return ( errStat( 255 ));
            }

            case ITEM_ID_ADD_EVENT_MAP_ENTRY: {

                return ( errStat( addEvent( *arg1, *arg2 )));
            }

            case ITEM_ID_DEL_EVENT_MAP_ENTRY: {

                return ( errStat( removeEvent( *arg1 )));
            }

            case ITEM_ID_SYNC: {

                return ( errStat( handleSyncCommand( npId, *arg1, *arg2 )));
            }

            case ITEM_ID_ENABLE_EVENT_PROCESSING: {

                if ( *arg1 )    
                    portMap.map[ portId( npId ) - 1 ].flags |= 
                                            NPF_PORT_EVENT_HANDLING_ENABLED;
                else            
                    portMap.map[ portId( npId ) - 1 ].flags &= 
                                            ~ NPF_PORT_EVENT_HANDLING_ENABLED;

                return ( errStat( ALL_OK ));
            }

            case ITEM_ID_ACTIVE_LED: {

                if ( *arg1 == 1 )  
                    return ( errStat( writeDio( CDC_RN_ACTIVITY_LED, true )));
                else if ( *arg1 == 2 )  
                    return ( errStat( toggleDio( CDC_RN_ACTIVITY_LED )));
                else                    
                    return ( errStat( writeDio( CDC_RN_ACTIVITY_LED, false )));
            }

            default: return ( errStat( ERR_INVALID_ITEM_ID ));
        }
    }
}

} // namespace LCS
