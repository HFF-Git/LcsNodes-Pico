//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime attribute management
//
//----------------------------------------------------------------------------------------
// This file contains the LCS runtime routines that implement node attribute access. 
// There are three routines that allow to manipulate node and port data as well as  
// issue requests to a node or port. The "npId" will indicate which node and port 
// the call refers to. The node portion is typically our own node Id, the port Id 
// refers to a ports on the node, with a port Id of zero referring to the node 
// itself. Any node can access another node. In this case request come via a message
// and the message handler will call the local routines in this file. 
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime attribute management
// Copyright (C) 2022 - 2025 Helmut Fieres
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
    
    extern LcsHeaderMap     headerMap;
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

    //------------------------------------------------------------------------------------
    // "debugEnabled" and "retStat" are the debug support routines. We can easily 
    // check whether debug is enabled at all. The return status routine will print 
    // out a return status message when debugging is enabled. The macro "RET_STAT" 
    // is a nice helper that adds the function name to the message.
    // 
    //------------------------------------------------------------------------------------
    inline bool attrDebugEnabled(  ) {

        return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_ATTRIBUTES )); 
    }

    inline uint8_t retStat( char *name, uint8_t errId ) {

        if ( attrDebugEnabled( )) {

            if ( errId == LCS_OK )  printf( "%s: OK\n", name );
            else                    printf( "%s: %d\n", name, errId );
        }

        return ( errId );
    }

    #define RET_STAT(x) retStat((char *) __func__, ( x ))

    //------------------------------------------------------------------------------------
    // "readAttrMem" gets a value from the node or port attribute map in MEM. As an 
    // internal function, we expect a valid block and item argument. The "block"
    // argument will refer to the node and port data attributes. 
    //
    //------------------------------------------------------------------------------------
    uint8_t readAttrMem( uint8_t block, uint8_t item, uint16_t *arg ) {

        *arg = nodeData.map[ block ][ item - IR_USER_RANGE_START ];
        return ( NO_ERR );
    }

    //------------------------------------------------------------------------------------
    // "writeAttrMem" stores a value to a node or port attribute map in MEM. As an 
    // internal function, we expect a valid block and item argument. The "block" 
    // argument will refer to the node and port data attributes. 
    //
    //------------------------------------------------------------------------------------
    uint8_t writeAttrMem( uint8_t block, uint8_t item, uint16_t arg ) {

        nodeData.map[ block ][ item - IR_USER_RANGE_START ] = arg;
        return ( NO_ERR );
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

        uint8_t rStat = rtNvmGetWord( ofs, arg );
        if ( rStat == NO_ERR ) rStat = writeAttrMem( block, item, *arg );
        
        return ( rStat );
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

        uint8_t rStat = rtNvmPutWord( ofs, arg );
        if ( rStat == NO_ERR ) rStat = writeAttrMem( block, item, arg );
        
        return ( rStat );
    }

    //------------------------------------------------------------------------------------
    // "syncAttrToMem" will copy the NVM attribute value to the MEM counterpart. 
    // All we do is just reading the NVM value to a dummy variable.
    //
    //------------------------------------------------------------------------------------
    uint8_t syncAttrToMem( uint8_t block, uint8_t item ) {

        if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

            uint16_t arg = 0;
            return ( readAttrNvm( block, item, &arg ));
        }
        else return ( ERR_INVALID_ITEM_ID );
    }

    //------------------------------------------------------------------------------------
    // "syncAttrToNvm" will take the MEM attribute value of an item and writes it 
    // to the NVM counterpart.
    //
    //------------------------------------------------------------------------------------
    uint8_t syncAttrToNvm( uint8_t block, uint8_t item ) {

        if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

            uint16_t    arg     = 0;
            uint8_t     rStat   = readAttrMem( block, item, &arg );
            if ( rStat == NO_ERR ) rStat = writeAttrNvm( block, item, arg );
            return ( rStat );
        }
        else return ( ERR_INVALID_ITEM_ID );
    }

    //------------------------------------------------------------------------------------
    // User callback function invocation routine. Items 128 to 255 are user defined
    // items. We will simply invoke a previously registered callback passing the 
    // arguments. 
    //
    //------------------------------------------------------------------------------------
    uint8_t invokeUserItemCallback( uint8_t npId, 
                                    uint8_t item, 
                                    uint16_t *arg1, 
                                    uint16_t *arg2 ) {

        if ( portMap.map[ portId( npId ) ].reqCallback != nullptr ) {

            return ( portMap.map[ portId( npId ) ].reqCallback( portId( npId ), 
                                                                item, 
                                                                arg1, 
                                                                arg2 ));
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
// "nodeGet" will lookup a value from the node, port or the attribute data map. The 
// "npId" argument contains the node and port Id. However, we will only use the portId
// portion, which represents the block index. For data attribute items the node state
// determines whether we just access the MEM attribute or the NVM version of the data
// synced with the memory counterpart. A node state of CONFIG will access NVM, since 
// you are configuring a node. For the other node or port reserved attributes the MEM
// version is used.
//
//----------------------------------------------------------------------------------------
uint8_t nodeGet( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    
    if ( attrDebugEnabled( )) {

        printf( "nodeGet: npId: 0x%x, item: %d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
        printf( "\n" );
    }

    if (( nodeMap.nodeState != NS_OPERATE) && 
        ( nodeMap.nodeState != NS_CONFIG )) {

        return ( RET_STAT( ERR_LIB_NOT_READY ));
    }

    if ( arg1 == nullptr ) {
            
        return ( RET_STAT( ERR_INVALID_ATTR_ARG )); 
    }
    
    if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {

            return ( RET_STAT( readAttrMem( portId( npId ), item, arg1 )));
        }
        else if ( nodeMap.nodeState == NS_CONFIG  ) {

            return ( RET_STAT( readAttrNvm( portId( npId ), item, arg1 )));
        }
        else                                        
            return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
        
    } else {

        switch ( item ) {

            case ITEM_ID_DEBUG_MASK: {   
                
                *arg1 = debugMask; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_RUNTIME_OPTIONS: {      
                
                *arg1 = runtimeOptions; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_FIRMWARE_OPTIONS: {

                *arg1 = firmwareOptions;
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_RT_LIB_VERSION: {    
                
                *arg1 = nodeMap.rtLibSwVersion; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_RT_LIB_PATCH_LEVEL: {    
                
                *arg1 = nodeMap.rtLibSwPatchLevel; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_NODE_STATE: {   
                
                *arg1 = nodeMap.nodeState; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_NODE_ID: {      
                
                *arg1 = nodeMap.nodeId; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_NODE_UID: {

                if ( arg2 == nullptr ) {
                    
                    return ( RET_STAT( ERR_INVALID_ATTR_ARG ));
                }

                *arg1 = nodeMap.nodeUID >> 16;
                *arg2 = nodeMap.nodeUID & 0xFFFF;
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_RESTART_COUNT: {
                
                *arg1 = nodeMap.nodeRestartCnt; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_BOARD_VERSION: {

                if ( isInRangeU( *arg1, 0, MAX_EXT_BOARD_MAP_ENTRIES )) {

                    *arg1 = headerMap.map[ *arg1 ].boardVersion ;
                    return ( RET_STAT( LCS_OK ));
                }
                else return ( RET_STAT( ERR_INVALID_ATTR_ARG ));
            }

            case ITEM_ID_PORT_MAP_ENTRIES: {

                if ( arg2 == nullptr ) {
                    
                    return ( RET_STAT( ERR_INVALID_ATTR_ARG ));
                }

                *arg1 = MAX_PORT_MAP_ENTRIES;
                *arg2 = portMap.mapHwm;
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_EVENT_MAP_ENTRIES: {

                if ( arg2 == nullptr ) {
                    
                    return ( RET_STAT( ERR_INVALID_ATTR_ARG ));
                }

                *arg1 = MAX_EVENT_MAP_ENTRIES;
                *arg2 = eventMap.mapHwm;
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_ATTR_MAP_ENTRIES: {

                if ( arg2 == nullptr ) {
                    
                    return ( RET_STAT( ERR_INVALID_ATTR_ARG ));
                }

                *arg1 = MAX_ATTR_MAP_ENTRIES;
                *arg2 = MAX_ATTR_MAP_ENTRIES;
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_FLAGS: { 
                
                *arg1 = portMap.map[ portId( npId ) ].flags; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_TYPE: { 
                         
                *arg1 = portMap.map[ portId( npId ) ].type; 
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_LOOKUP_EVENT_ENTRY: {

                *arg1 = searchEvent( *arg1 );
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_GET_EVENT_MAP_ENTRY: {

                if ( arg2 == nullptr ) {
                    
                    return ( RET_STAT(  ERR_INVALID_ATTR_ARG ));
                }

                return ( getMemEmapEntry( *arg1, arg1, arg2 ));
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                *arg1 = portMap.map[ portId( npId ) ].eventDelayTime;
                return ( RET_STAT( LCS_OK ));
            }

            default: return ( RET_STAT( ERR_INVALID_ITEM_ID ));
        }
    }
}

//----------------------------------------------------------------------------------------
// "nodePut" will write a value to the node, port or the attribute data map. The 
// "npId" argument contains the node and port Id. However, we will only use the 
// portId portion, which represents the block index. For data attribute items the 
// node state determines whether we just update the MEM attribute or both MEM and
// NVM version. Node state CONFIG will update NVM too. 
//
//----------------------------------------------------------------------------------------
uint8_t nodePut( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2 ) {

    if ( attrDebugEnabled( )) {

        printf( "nodePut: npId: 0x%x, item: %d, val1:%d, val2: %d\n",
                npId, item, val1, val2  );
    }

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) {
        
        return ( RET_STAT( ERR_LIB_NOT_READY ));
    }
    
    if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        if ( nodeMap.nodeState == NS_OPERATE ) {

            return ( RET_STAT( writeAttrMem( portId( npId ), item, val1 )));
        }
        else if ( nodeMap.nodeState == NS_CONFIG ) { 

            return ( RET_STAT( writeAttrNvm( portId( npId ), item, val1 )));
        }
        else                                       
             return ( RET_STAT( ERR_INVALID_OP_FOR_NODE_STATE ));
        
    } 
    else {

        switch ( item ) {

            case ITEM_ID_DEBUG_MASK: {

                if ( usbIsConnected( )) debugMask = val1 | LCS_DBG_ENABLE;           
                else                    debugMask = val1 & ~ LCS_DBG_ENABLE;
              
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_NODE_ID: {

                nodeMap.nodeId = nodeId( val1 );
                return ( RET_STAT( rtNvmPutWord( 
                                    NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, nodeId ), 
                                        val1 )));
            }

            case ITEM_ID_RT_LIB_VERSION: {

                nodeMap.rtLibSwVersion = val1;
                return ( RET_STAT( rtNvmPutWord( 
                            NVM_NODE_MAP_OFS + offsetof( LcsNodeMap, rtLibSwVersion ), 
                                val1 )));
            }

            case ITEM_ID_FLAGS: {

                portMap.map[ portId( npId ) ].flags = val1;
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_TYPE: {

                portMap.map[ portId( npId ) ].type = lowByte( val1 );
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_EVENT_DELAY_TICKS: {

                portMap.map[ portId( npId ) ].eventDelayTime = val1;
                return ( RET_STAT( LCS_OK ));
            }

            default: return ( RET_STAT( ERR_INVALID_ITEM_ID ));
        }
    }
}

//----------------------------------------------------------------------------------------
// "nodeReq" will carry out a node or port function. A function, represented by an
// item identifier, can be a node or port defined item, an extension board driver 
// defined item or a user defined item. For LCS node or port related functions, the
// library handles the request, for a user defined item, a user callback is invoked.
//
//----------------------------------------------------------------------------------------
uint8_t nodeReq( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if ( attrDebugEnabled( )) {

        printf( "nodeReq: 0x%x:%d", npId, item  );
        if ( arg1 != nullptr ) printf( ":%d", *arg1 ); else printf( "null" );
        if ( arg2 != nullptr ) printf( ":%d", *arg2 ); else printf( "null" );
        printf( "\n" );
    }

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) {
        
        return ( RET_STAT( ERR_LIB_NOT_READY ));
    }
    
    if ( isInRangeU( item, IR_USER_RANGE_START, IR_USER_RANGE_END )) {

        return ( RET_STAT( invokeUserItemCallback( npId, item, arg1, arg2 )));
    
    } else {

        switch ( item ) {

            case ITEM_ID_RESET: {

                // ??? we have a routine to do resets via messages... 
                // ??? should we even have a way to invoke via REQ calls ?
                
                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_FORMAT_EXT: {

                // ??? we need a way to format the extension board area, when needed. 
                // ??? applies to ports 1 to 4 when they are mapped to a driver...
                // val1 = type ?
                // val2 = boardId
               
                return ( RET_STAT( 255 ));
            }

            case ITEM_ID_SYNC_TO_MEM: {

                return( RET_STAT( syncAttrToMem( portId( npId ), *arg1 )));
            }

            case ITEM_ID_SYNC_TO_NVM: {

                return( RET_STAT( syncAttrToNvm( portId( npId ), *arg1 )));
            }

            case ITEM_ID_SET_EVENT_MASK: {

                return ( RET_STAT( setEventMask( *arg1, *arg2 )));
            }

            case ITEM_ID_REMOVE_EVENT_MASK: {

                return ( RET_STAT( removeEventMask( *arg1 )));
            }

            case ITEM_ID_SYNC_EVENT_MAP_MEM: {

                return ( RET_STAT( syncEventMapToMem( )));
            }

            case ITEM_ID_SYNC_EVENT_MAP_NVM: {

                return ( RET_STAT( syncEventMapToNvm( )));
            }

            case ITEM_ID_ENABLE_EVENT_PROCESSING: {

                if ( *arg1 ) {

                    portMap.map[ portId( npId ) - 1 ].flags |= 
                                            NPF_PORT_EVENT_HANDLING_ENABLED;
                }
                else {

                    portMap.map[ portId( npId ) - 1 ].flags &= 
                                            ~ NPF_PORT_EVENT_HANDLING_ENABLED;
                }

                return ( RET_STAT( LCS_OK ));
            }

            case ITEM_ID_ACTIVE_LED: {

                if ( *arg1 == 1 ) {

                    return ( RET_STAT( writeDio( CDC_RN_ACTIVITY_LED, true )));
                }
                else if ( *arg1 == 2 ) { 

                    return ( RET_STAT( toggleDio( CDC_RN_ACTIVITY_LED )));
                }
                else                    
                    return ( RET_STAT( writeDio( CDC_RN_ACTIVITY_LED, false )));
            }

            default: return ( RET_STAT( ERR_INVALID_ITEM_ID ));
        }
    }
}

} // namespace LCS
