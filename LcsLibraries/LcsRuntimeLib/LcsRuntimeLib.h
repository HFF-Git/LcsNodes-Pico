//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library include file
//
//------------------------------------------------------------------------------------------------------------
// At the heart of layout control, LCS, is the runtime library implementing the basic functions. Please refer
// to the document for information on concepts and implementation notes. This is the exterbal include file for
// the firmware programmer. All external definitions of key contants and types are included here.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Runtime Library
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
#ifndef LCS_RT_LIB_h
#define LCS_RT_LIB_h

//------------------------------------------------------------------------------------------------------------
// Include files.
//
//------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <inttypes.h>
#include "LcsCdcLib.h"

//------------------------------------------------------------------------------------------------------------
// All LCS Library definitions are in a separate name space "LCS".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//-----------------------------------------------------------------------------------------------------------
// A node is identified through the node number. Node numbers start with one. The nodeId of zero represents
// the NIL node Id. The node Id is a 12-bit number, so up to 4095 nodes can be addressed. The nodeId, a
// unique Id for the LCS nodes in a layout, is also used as the canId used for the CAN bus. Keep in mind
// that a CAN bus nevertheless can reasonably handle about 127 nodes at the same time.
//
//------------------------------------------------------------------------------------------------------------
enum LcsNodeId : uint16_t {

    NIL_NODE_ID   = 0,
    MIN_NODE_ID   = 1,
    MAX_NODE_ID   = 4095
};

//-----------------------------------------------------------------------------------------------------------
// A node type can be assigned to a node. NodeId types start with one. The nodeType of zero represents the
// NIL node type. A node type is arbitrarily defied by the firmware programmer.
//
//------------------------------------------------------------------------------------------------------------
enum LcsNodeTypeId : uint8_t {

    NIL_NODE_TYPE     = 0,
    MIN_NODE_TYPE_ID  = 1,
    MAX_NODE_TYPE_ID  = 255
};

//------------------------------------------------------------------------------------------------------------
// The port Id identifies the port on a node. Port numbers start with one. The port number 0 represesents
// the NIL port number and usually referes to the node itself. A node can have up to 15 ports.
//
//------------------------------------------------------------------------------------------------------------
enum LcsPortId : uint8_t {

    NIL_PORT_ID   = 0,
    MIN_PORT_ID   = 1,
    MAX_PORT_ID   = 15
};

//------------------------------------------------------------------------------------------------------------
// A port type can be assigned to a port. Port types start with one. The portType of zero represents the
// NIL port type. A port type is arbitrarily defied by the firmware programmer.
//
//------------------------------------------------------------------------------------------------------------
  enum LcsPortTypeId : uint8_t {

    NIL_PORT_TYPE     = 0,
    MIN_PORT_TYPE_ID  = 1,
    MAX_PORT_TYPE_ID  = 255
};

//-----------------------------------------------------------------------------------------------------------
// Events are just numbers assigned to an event by a configuration tool. Event id numbers start with one, 
// the 0 number represents the NIL event number. The maximum event id number is 65535. Perhaps a configuration
// tool will assign number ranges to group event types.
//
//------------------------------------------------------------------------------------------------------------
enum LcsEventId : uint16_t {

    NIL_EVENT_ID  = 0,
    MIN_EVENT_ID  = 1,
    MAX_EVENT_ID  = 65535
};

//------------------------------------------------------------------------------------------------------------
// Each locomotive has a type. There are STEAM, DIESEL and ELECTRIC engines so far.
//
// ??? additional types ?
//------------------------------------------------------------------------------------------------------------
enum DccLocoType : uint8_t {

    LOC_T_NIL       = 0,
    LOC_T_STEAM     = 1,
    LOC_T_DIESEL    = 2,
    LOC_T_ELECTRIC  = 3
};

//------------------------------------------------------------------------------------------------------------
// The base station maintains the locomotive sessions. Session Ids start with 1, up to 255 simultaneous
// sessions are supported.
//
//------------------------------------------------------------------------------------------------------------
enum DccSessionId : uint8_t {

    NIL_LOCO_SESSION_ID  = 0,
    MIN_LOCO_SESSION_ID  = 1,
    MAX_LOCO_SESSION_ID  = 255
};

//------------------------------------------------------------------------------------------------------------
// The cabId is the locomotive number or address. For DCC type locomotives, there is a short and long address
// for a decoder. The short address ranges from 1 .. 127, the long address from 1 .. to 10239. However, most 
// base stations support just up to 9999 locomotives IDs and we do too.
//
//------------------------------------------------------------------------------------------------------------
enum DccCabId : uint16_t {

    NIL_CAB_ID  = 0,
    MIN_CAB_ID  = 1,
    MAX_CAB_ID  = 9999
};

//------------------------------------------------------------------------------------------------------------
// A DCC decoder has configuration variables, called CVs. CVs are numbered starting with 1, the maximum
// number is 1024.
//
//------------------------------------------------------------------------------------------------------------
enum DccCvId : uint16_t {

    NIL_DCC_CV_ID  = 0,
    MIN_DCC_CV_ID  = 1,
    MAX_DCC_CV_ID  = 1024
};

//------------------------------------------------------------------------------------------------------------
// A locomotive decoder features up to 69 functions Ids. They are numbered from 0 to 68.
//
//------------------------------------------------------------------------------------------------------------
enum DccFuncId : uint8_t {

    NIL_DCC_FUNC_ID   = 255,
    MIN_DCC_FUNC_ID   = 0,
    MAX_DCC_FUNC_ID   = 68
};

//------------------------------------------------------------------------------------------------------------
// DCC decoder functions are grouped in ten groups, labelled from 1 to 10 according to the DCC standard.
//
//------------------------------------------------------------------------------------------------------------
enum DccFuncGroupId : uint8_t {

    MIN_DCC_FUNC_GROUP_ID   = 1,
    MAX_DCC_FUNC_GROUP_ID   = 10
};

//------------------------------------------------------------------------------------------------------------
// DCC decoder function mapping Ids. The LCS system defines a set of functions used by the handlhelds such
// as horn, lights and so on. These identifier are mapped to the DCC functions available for a given DCC
// decoder.
//
// ??? define our standard available function IDs here..... we may not need 69 slots.
//------------------------------------------------------------------------------------------------------------
enum LcsDccFuncId : uint8_t {

    NIL_LCS_DCC_FUNC_ID = 0,
    MIN_LCS_DCC_FUNC_ID = 1,
    MAX_LCS_DCC_FUNC_ID = 68
};

//------------------------------------------------------------------------------------------------------------
// The DCC standard defines several speed step modes. Today, 128 speed steps is the one used in all new
// decoders. The other speed steps are mapped to the 128 value range.
//
//------------------------------------------------------------------------------------------------------------
enum DccSpeedSteps : uint8_t {

    DCC_SPEED_STEPS_14    = 1,
    DCC_SPEED_STEPS_28    = 2,
    DCC_SPEED_STEPS_128   = 3
};

//------------------------------------------------------------------------------------------------------------
// The DCC locomotive decoder speed. The range is defined for a 128 speed step decoder, from 0 to 127. The
// speed of 1 represents the emergency speed stop. In normal operations, speed steos would thus go from 2
// to 0 and back. For analog engines, we keep this scheme and map it to the respective power levels.
//
//------------------------------------------------------------------------------------------------------------
enum DccLocSpeed : uint8_t {

    MIN_DCC_SPEED    = 0,
    DCC_ESTOP_SPEED  = 1,
    MAX_DCC_SPEED    = 127
};

//--------------------------------------------------------------------------------------------------------------
// DCC locomotive direction.
//
//--------------------------------------------------------------------------------------------------------------
enum DccLocDirection : uint8_t {

    DCC_DIR_LOCO_NEUTRAL  = 0,
    DCC_DIR_LOCO_FORWARD  = 1,
    DCC_DIR_LOCO_REVERSE  = 2
};

//------------------------------------------------------------------------------------------------------------
// "LocSessionModes" specify the options when creating a session for the loco. Besides creating a normal
// session an existing session can be taken over or even shared among multiple handhelds.
//
//------------------------------------------------------------------------------------------------------------
enum DccLocSessionModes : uint8_t {

    LSM_NORMAL  = 1,
    LSM_STEAL   = 2,
    LSM_SHARED  = 3
};

//------------------------------------------------------------------------------------------------------------
// "CvModeOptions" is used by the DCC CV variables access routines to specify the access mode. Only options
// 0 and 1 are supported. The others are there for historic reasons, the functionlity was found in older
// decoders and should not be supported anymore.
//
//------------------------------------------------------------------------------------------------------------
enum DccCvModeOptions : uint8_t {

    CVM_BYTE      = 0,
    CVM_BIT       = 1,
    CVM_PAGE      = 2,
    CVM_REGISTER  = 3,
    CVM_ADR_ONLY  = 4
};

//--------------------------------------------------------------------------------------------------------------
// The defined board types and sub types. When the runtime is initialized, the firmware will pass the type
// and subtype to specify what board it expects.
//
//--------------------------------------------------------------------------------------------------------------
enum LcsBoardType : uint16_t {

    BT_NIL                = 0,
    BT_MAIN_CONTROLLER    = 1,
    BT_BASE_STATION       = 2,
    BT_BLOCK_CONTROLLER   = 3,
    BT_CAB_HANDHELD       = 4,

    BT_EXT_OCC_DETECT     = 11,
    BT_EXT_SERVO          = 12,
    BT_EXT_GPIO           = 13
};

enum LcsBoardSubType : uint16_t {

    BT_ST_NIL             = 0

};

//--------------------------------------------------------------------------------------------------------------
// The defined chip families. There are controller chip families such as the controller family RP2040, or 
// chip familes for the NVM chips used, and so on.
//
//--------------------------------------------------------------------------------------------------------------
enum LcsControllerFamilyType : uint16_t {

    CF_FAM_NIL                = 0,
    CF_FAM_RPICO_2040         = 1,
    CF_FAM_MICROCHIP          = 2,
    CF_FAM_NXP                = 3
};

//------------------------------------------------------------------------------------------------------------
// The configuration descriptor and node map have an option field. The following constants define the
// options that can be set.
//
//  NOPT_SKIP_NODE_ID_CONFIG - during startup, skip the nodeId configuration protocol.
//  NOPT_SKIP_NODE_INIT_STEP - during startup, skip the node initialization step.
//  NOPT_SKIP_PORT_INIT_STEP - during startup, sjip the port initialization step.
//
//------------------------------------------------------------------------------------------------------------
enum NodeOptions : uint16_t {

    NOPT_SKIP_NODE_ID_CONFIG  = 0x0001,
    NOPT_SKIP_NODE_INIT_STEP  = 0x0002,
    NOPT_SKIP_PORT_INIT_STEP  = 0x0004
};

//------------------------------------------------------------------------------------------------------------
// Node Flags.
//
//  NFLAGS_EXT_PRESENT  - extension boards are present.
//
//------------------------------------------------------------------------------------------------------------
enum NodeFlags : uint16_t {

    NFLAGS_EXT_PRESENT        = 0x0001,

};
//------------------------------------------------------------------------------------------------------------
// Node and port data are accessed with the nodeInfo and node control function. One of the arguments is
// the info item, which specifies what data to access. The item nuber range is divided as follows:
//
//   0          - NIL item, not used
//   1  ..  63  - Node reseved area, global items.
//  64  .. 127  - User defined items, passed to the registered callback routine.
// 128  .. 191  - Node/Port Attributes returned from MEM
// 192  .. 255  - Node/Port Attributes copied from NVM to MEM and then returned. Mirrors items 128 - 191.
//
// Items may refer to node or port specific data. The portId specified in the call will indicate whether we
// access the node or a port on the node. A portId of NIP-NIL means that we refer to the node itself.
//
// ??? note: this list is work in progress, please us always the names rather than the numbers.
//------------------------------------------------------------------------------------------------------------
enum NodeAndPortInfoItems : uint8_t {

    NPI_NIL                         = 0,

    NPI_NODE_MAP_RANGE_START        = 1,
    NPI_NODE_MAP_RANGE_END          = 63,

    NPI_NODE_USER_DEFINED_START    = 64,
    NPI_NODE_USER_DEFINED_END      = 127,

    NPI_ATTR_MEM_RANGE_START        = 128,
    NPI_ATTR_MEM_RANGE_END          = 191,

    NPI_ATTR_NVM_RANGE_START        = 192,
    NPI_ATTR_NVM_RANGE_END          = 255,

    NPI_MAX_ITEMS                   = 255,


    // ??? also add DRV related items...
    // ??? board info, name, version, chan info

    // ??? need extension board state info...


    NPI_GET_OPTIONS                 = 1,
    NPI_GET_VERSION                 = 2,
    NPI_GET_NODE_FLAGS              = 3,
    NPI_GET_NODE_TYPE               = 4,
    NPI_GET_NODE_ID                 = 5,
    NPI_GET_NODE_UID                = 6,
    NPI_GET_RESTART_COUNT           = 7,

    NPI_GET_PORT_MAP_ENTRIES        = 9,
    NPI_GET_EVENT_MAP_ENTRIES       = 10,
    NPI_GET_ATTR_MAP_ENTRIES        = 11,
    NPI_GET_EVENT_MAP_ENTRY         = 12,
    NPI_GET_NODE_NAME_1             = 13,
    NPI_GET_NODE_NAME_2             = 14,
    NPI_GET_NODE_NAME_3             = 15,
    NPI_GET_NODE_NAME_4             = 16,

    NPI_GET_PORT_FLAGS              = 32,
    NPI_GET_PORT_TYPE               = 33,
    NPI_GET_EVENT_DELAY_TICKS       = 34,
};

enum NodeAndPortControlItems : uint8_t {

    NPC_NIL                         = 0,

    NPC_NODE_MAP_RANGE_START        = 1,
    NPC_NODE_MAP_RANGE_END          = 63,

    NPC_NODE_USER_DEFINED_START    = 64,
    NPC_NODE_USER_DEFINED_END      = 127,

    NPC_ATTR_MEM_RANGE_START        = 128,
    NPC_ATTR_MEM_RANGE_END          = 191,

    NPC_ATTR_NVM_RANGE_START        = 192,
    NPC_ATTR_NVM_RANGE_END          = 255,

    NPC_MAX_ITEMS                   = 255,


    // ??? also add DRV related items...
    // ??? a RESET, ...
    // ??? add stop and enable periodic processing ?
    


    NPC_RESET_NODE                  = 1,
    NPC_SYNC_NODE                   = 2,
    NPC_SET_NODE_ID                 = 3,
    NPC_ADD_EVENT_MAP_ENTRY         = 5,
    NPC_DEL_EVENT_MAP_ENTRY         = 6,
    NPC_SET_READY_LED               = 7,
    NPC_SET_ACTIVITY_LED            = 8,
    NPC_TOGGLE_READY_LED            = 9,
    NPC_TOGGLE_ACTIVITY_LED         = 10,
    NPC_BLINK_READY_LED             = 11,
    NPC_BLINK_ACTIVITY_LED          = 12,
    NPC_SET_NODE_FLAGS              = 13,
    NPC_SET_NODE_TYPE               = 20,
    NPC_SET_NODE_NAME_1             = 21,
    NPC_SET_NODE_NAME_2             = 22,
    NPC_SET_NODE_NAME_3             = 23,
    NPC_SET_NODE_NAME_4             = 24,

    NPC_SET_PORT_FLAGS              = 32,
    NPC_SET_PORT_TYPE               = 33,
    NPC_ENABLE_EVENT_PROCESSING     = 34,
    NPC_SET_EVENT_DELAY_TICKS       = 35,

    NPC_SYNC_EVENT_MAP              = 36,
};

//----------------------------------------------------------------------------------------------------------
// The portMap entry has a flag field. The constants defined here indicate the bit positions and fields
// defined.
//
//  PF_PORT_ENABLED                 - the port is initialized and active
//  PF_PORT_EVENT_HANDLING_ENABLED  - the port has event handling enabled
//  PF_PORT_EVENT_DIRECTION         - if set, this is an outbound port, else an inbound port.
//  PF_EVENT_PENDING                - an event has been received for this port and is pending.
//
//
// ??? what exactly is an outbond port ?
// ??? should we have a föag for a request pending ?
// ??? a flag for a timed out request ? 
//----------------------------------------------------------------------------------------------------------
enum PortFlags : uint16_t {

    PF_PORT_ENABLED                 = 0x8000,
    PF_PORT_EVENT_HANDLING_ENABLED  = 0x4000,
    PF_PORT_EVENT_DIRECTION         = 0x2000,
    PF_EVENT_PENDING                = 0x1000
};

//----------------------------------------------------------------------------------------------------------
// The port event action. When an event is received, it will be of the type shown below. There is a port
// specific time delay configured between the actual receipt of an event message and the invocation of the
// event callback.
//
//  PEA_EVENT_IDLE                - the port is idle.
//  PEA_EVENT_ON                  - an "ON" event was received.
//  PEA_EVENT_OFF                 - an "OFF" event was received.
//  PEA_EVENT_EVT                 - an event wth additional arguments was received.
//
//----------------------------------------------------------------------------------------------------------
enum PortEventAction : uint8_t {

    PEA_EVENT_IDLE    = 0,
    PEA_EVENT_ON      = 1,
    PEA_EVENT_OFF     = 2,
    PEA_EVENT_EVT     = 3
};

//----------------------------------------------------------------------------------------------------------
// The "mode" parameter in the attribute map access routines define whether the access concerns the MEM or
// NVM or both areas. For a read operation the SYNC option will first copy the NVM data to the MEM data.
// For the write operation the SYNC option will first write to the MEM data and then update the NVM data.
//
//  ACC_MEM                   - the acces is memory only.
//  ACC_NVM                   - the access is NVM only.
//  ACC_SYNC                  - the acces will for reads first read NVM to MEM for writes flush MEM to NVM.
//
//----------------------------------------------------------------------------------------------------------
enum AttrDataAccessOptions : uint8_t {

    ACC_MEM   = 0,
    ACC_NVM   = 1,
    ACC_SYNC  = 2
};

//---------------------------------------------------------------------------------------------------------
// The opCode identifies the LCS Bus message. It is always the first data byte of the message. We encode
// the number of payload data bytes in the first three bits of the opCode. For each message length there is
// a maximum of 32 opCode possible. This scheme is adopted from the Merg CBUS. The constant list below is
// organized by instruction length. The OPC  macro helps to define the opcodes. The first argument is the
// length of the data bytes, the second the opcodeId within the group.
//
// ??? note: this list is work in progress, please us always the names rather than the numbers.
//----------------------------------------------------------------------------------------------------------
#define OPC( len, id ) ((uint8_t) (( len << 5 ) + ( id & 0x1F )))

enum LcsMsgOpCodes : uint8_t {

    LCS_OP_NO_MSG         = OPC( 0, 0 ),

    LCS_OP_CFG            = OPC( 0, 2 ),
    LCS_OP_OPS            = OPC( 0, 3 ),
    LCS_OP_BON            = OPC( 0, 4 ),
    LCS_OP_BOF            = OPC( 0, 5 ),
    LCS_OP_REQ_TON        = OPC( 0, 6 ),
    LCS_OP_REQ_TOF        = OPC( 0, 7 ),
    LCS_OP_REQ_ESTP       = OPC( 0, 8 ),
    LCS_OP_TON            = OPC( 0, 9 ),
    LCS_OP_TOF            = OPC( 0, 10 ),
    LCS_OP_ESTP           = OPC( 0, 11 ),

    LCS_OP_REL_LOC        = OPC( 1, 1 ),
    LCS_OP_QRY_LOC        = OPC( 1, 2 ),
    LCS_OP_KEEP_LOC       = OPC( 1, 3 ),

    LCS_OP_PING           = OPC( 2, 1 ),
    LCS_OP_ACK            = OPC( 2, 2 ),
    LCS_OP_DCC_ACK        = OPC( 2, 3 ),
    LCS_OP_SET_LSPD       = OPC( 2, 4 ),
    LCS_OP_SET_LMOD       = OPC( 2, 5 ),
    LCS_OP_LOC_FON        = OPC( 2, 6 ),
    LCS_OP_LOC_FOF        = OPC( 2, 7 ),
    LCS_OP_BACC           = OPC( 2, 8 ),
    LCS_OP_EACC           = OPC( 2, 9 ),

    LCS_OP_RESET          = OPC( 3, 1 ),
    LCS_OP_REQ_LOC        = OPC( 3, 2 ),
    LCS_OP_SET_LCON       = OPC( 3, 3 ),
    LCS_OP_LOC_FGRP       = OPC( 3, 4 ),
    LCS_OP_SEND_DCC3      = OPC( 3, 5 ),
    LCS_OP_DCC_ERR        = OPC( 3, 6 ),
    LCS_OP_REQ_CVS        = OPC( 3, 7 ),

    LCS_OP_EVT_ON         = OPC( 4, 1 ),
    LCS_OP_EVT_OFF        = OPC( 4, 2 ),
    LCS_OP_SEND_DCC4      = OPC( 4, 3 ),
    LCS_OP_REP_CVS        = OPC( 4, 4 ),
    LCS_OP_SET_CVS        = OPC( 4, 5 ),

    LCS_OP_ERR            = OPC( 5, 1 ),
    LCS_OP_SET_CVM        = OPC( 5, 2 ),
    LCS_OP_SEND_DCC5      = OPC( 5, 3 ),

    LCS_OP_EVT            = OPC( 6, 1 ),
    LCS_OP_SEND_DCC6      = OPC( 6, 2 ),
    LCS_OP_NCOL           = OPC( 6, 6 ),

    LCS_OP_REQ_NID        = OPC( 7, 1 ),
    LCS_OP_REP_NID        = OPC( 7, 2 ),
    LCS_OP_SET_NID        = OPC( 7, 3 ),
    LCS_OP_QRY_NODE       = OPC( 7, 4 ),
    LCS_OP_REQ_NODE       = OPC( 7, 5 ),
    LCS_OP_REP_NODE       = OPC( 7, 6 ),
    LCS_OP_REP_LOC        = OPC( 7, 7 )
};

//----------------------------------------------------------------------------------------------------------
// LCS Core Library Error codes. The status code is used as a return value from most of the library methods.
// The numbers are grouped in a LCS library portion and a user firmware portion. The LCS library portion
// ranges from 1 to 127, the user portion from 128 to 255. The vaue of zero is generally a "OK".
//
// ??? add NVM errors, also CDC errors ?
//----------------------------------------------------------------------------------------------------------
enum LcsErrorCodes : uint8_t {

    ALL_OK                              = 0,
    ERR_NOT_IMPLEMENTED                 = 1,
    ERR_NOT_SUPPORTED                   = 2,

    ERR_CDC_SETUP                       = 10,
    ERR_NVM_SETUP                       = 11,
    ERR_CAN_SETUP                       = 12,
    ERR_NVM_NODE_MAP_CORRUPT            = 13,



    ERR_NVM_SIZE_EXCEEDED               = 14,
    ERR_MEM_SIZE_EXCEEDED               = 15,

    ERR_NVM_OP_FAILED                   = 16,

    ERR_NODE_NOT_OPS_STATE              = 18,
    ERR_NODE_NOT_CONFIG_STATE           = 19,
    ERR_NODE_OUTSTANDING_REQ_LIMIT      = 20,
    ERR_TASK_MAP_SIZE_EXCEEDED          = 21,

    ERR_INVALID_ITEM_ID                 = 39,

    ERR_INVALID_NODE_ID                 = 30,
    ERR_INVALID_NODE_INFO_ITEM          = 31,
    ERR_INVALID_NODE_CTRL_ITEM          = 32,

    ERR_INVALID_PORT_ID                 = 40,
    ERR_INVALID_PORT_INFO_ITEM          = 41,
    ERR_INVALID_PORT_CTRL_ITEM          = 42,

    ERR_INVALID_EVENT_ID                = 50,
    ERR_INVALID_EVENT_MAP_INDEX         = 51,
    ERR_EVENT_MAP_FULL                  = 52,
    ERR_PENDING_REQ_MAP_FULL            = 53,

    ERR_INVALID_SESSION_ID              = 60,
    ERR_INVALID_CAB_ID                  = 61,
    ERR_INVALID_LOCO_SPEED              = 62,
    ERR_INVALID_FGROUP_ID               = 63,
    ERR_INVALID_FUNC_ID                 = 64,
    ERR_INVALID_CV_ID                   = 65,
    ERR_INVALID_CV_MODE                 = 66,
    ERR_CV_OP_NO_ACK                    = 67,
    ERR_INVALID_BIT_POS                 = 68,
    ERR_INVALID_PACKET_LEN              = 69,
    ERR_INVALID_REPEATS                 = 70,

    ERR_CAN_BUS_INIT                    = 81,
    ERR_CAN_INVALID_MODE                = 82,
    ERR_CAN_BUS_MSG_SIZE                = 83,
    ERR_CAN_MSG_SEND                    = 84,
    ERR_CAN_MSG_RECV                    = 85,
    ERR_CAN_MSG_NO_MSG                  = 86,
    ERR_CAN_ID_COLLISION                = 87,
    ERR_CAN_ID_CHANGED                  = 88,

    // ??? for now .... 
    ERR_INVALID_BOARD_ID = 255,

    ERR_INVALID_DRV_ITEM                = 100,

    ERR_NODE_SPECIFIC_BASE              = 128
};


//------------------------------------------------------------------------------------------------------------
// All drivers support a common set of items. The item numbers are in the range of 1 to 63. Driver specific
// items should be allocated in the range 192 to 255.
//
// ??? generic part become of node items ?
//------------------------------------------------------------------------------------------------------------
enum LcsNodeDriverItems {

    DI_NIL                = 0,
    DI_RESET              = 1,
    DI_GET_BOARD_TYPE     = 2,
    DI_GET_BOARD_SUBTYPE  = 3,
    DI_GET_BOARD_NAME     = 4,
    DI_GET_BOARD_VERSION  = 5,

    DI_DRIVER_ITEM_BASE   = 192
};

//----------------------------------------------------------------------------------------------------------
// Core library callback function signatures.
//
// ??? flags on init call back indicate wteher STARTUP, RESTE or PFAIL...
// ??? do we need to pass portID for item callback ?
// 
//----------------------------------------------------------------------------------------------------------
extern "C" {

    typedef void    ( *LcsMsgCallback ) ( uint8_t *msg );
    typedef uint8_t ( *LcsCommandCallback ) ( char *cmdLine );
    typedef void    ( *LcsTaskCallback ) ( );

    typedef uint8_t ( *LcsInitCallback )  ( uint16_t nodeId, uint8_t portId, uint16_t flags );
    typedef uint8_t ( *LcsInfoItemCallback ) ( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 );
    typedef uint8_t ( *LcsCtrlItemCallback ) ( uint8_t portId, uint8_t item, uint16_t arg1, uint16_t arg2 );
    typedef void    ( *LcsItemReqRepCallback ) ( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t val1, uint16_t arg2 );
    typedef void    ( *LcsPortEventCallback) ( uint16_t nodeId, uint8_t portId, uint8_t eAction, uint16_t eId, uint16_t eData );
}


// ??? !!!!!!! think about how to best check that the system is ready for a particular call....

//------------------------------------------------------------------------------------------------------------
// Library functions. The main function are the initialization and start of the LCS runtime.
// 
//------------------------------------------------------------------------------------------------------------
uint8_t             initRuntime( CDC::CdcPinConfig *cfg );
void                startRuntime( );

//----------------------------------------------------------------------------------------------------------
// Access the node.
//
//----------------------------------------------------------------------------------------------------------
uint8_t             nodeInfo( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 = nullptr );
uint8_t             nodeControl( uint8_t portId, uint8_t item, uint16_t val1 = 0, uint16_t val2 = 0 );

//----------------------------------------------------------------------------------------------------------
// Register callbacks for messages and tasks.
//
//----------------------------------------------------------------------------------------------------------
void                registerLcsMsgCallback( LcsMsgCallback functionId );
void                registerDccMsgCallback( LcsMsgCallback functionId );
void                registerCommandCallback( LcsCommandCallback functionId );
void                registerReqRepCallback( LcsItemReqRepCallback handler );
void                registerPortEventCallback( LcsPortEventCallback functionId );

uint8_t             registerInitCallback( uint8_t portId, LcsInitCallback handler );
uint8_t             registerInfoCallback( uint8_t portId, LcsInfoItemCallback handler );
uint8_t             registerCtrlCallback( uint8_t portId, LcsCtrlItemCallback handler );

uint8_t             registerPeriodicTask( LcsTaskCallback task, uint32_t interval = 0 );

//----------------------------------------------------------------------------------------------------------
// A set of convenience functions to send an LCS message.
//
//----------------------------------------------------------------------------------------------------------
uint8_t             sendReset( uint16_t nodeId, uint8_t portId, uint8_t flags );
uint8_t             sendBusOn( );
uint8_t             sendBusOff( );
uint8_t             sendPing( uint16_t nodeId );
uint8_t             sendAck( uint16_t nodeId );
uint8_t             sendErr( uint16_t nodeId, uint8_t errCode, uint8_t arg1 = 0, uint8_t arg2 = 0 );

uint8_t             sendReqNodeId( uint16_t nodeId, uint32_t nodeUID, uint8_t flags );
uint8_t             sendRepNodeId( uint16_t nodeId, uint32_t nodeUID );
uint8_t             sendSetNodeId( uint16_t nodeId, uint32_t nodeUID );
uint8_t             sendNodeIdCollision( uint16_t nodeId, uint32_t nodeUID );

uint8_t             sendQryNode( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t arg1 = 0, uint16_t arg2 = 0 );
uint8_t             sendRepNode( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t val1, uint16_t val2  );
uint8_t             sendReqNode( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t val1, uint16_t val2  );

uint8_t             sendEventOn( uint16_t nodeId, uint16_t eventId );
uint8_t             sendEventOff( uint16_t nodeId, uint16_t eventId );
uint8_t             sendEvent( uint16_t nodeId, uint16_t eventId, uint16_t arg );

uint8_t             sendReqTrackOn( );
uint8_t             sendTrackOn( );
uint8_t             sendReqTrackOff( );
uint8_t             sendTrackOff( );
uint8_t             sendReqEstop( );
uint8_t             sendEstop( );

// ??? rework the sendXXX routines to only have the arguments that need to be passed. I.e. nodeID is often our own nodeId.

uint8_t             sendReqLoc( uint16_t locAdr, uint8_t flags  );
uint8_t             sendRelLoc( uint8_t sId  );
uint8_t             sendRepLoc( uint8_t sId, uint16_t locAdr, uint8_t spDir, uint8_t fn1 = 0, uint8_t fn2 = 0, uint8_t fn3 = 0 );
uint8_t             sendLocConsist( uint8_t sId, uint8_t consId, uint8_t flags );
uint8_t             sendQueryLoc( uint8_t sId  );
uint8_t             sendKeepLoc( uint8_t sId  );
uint8_t             sendSetLocSpDir( uint8_t sId, uint8_t spDir );
uint8_t             sendSetLocMode( uint8_t sId, uint8_t mode );
uint8_t             sendSetLocFuncOn( uint8_t sId, uint8_t fNum );
uint8_t             sendSetLocFuncOff( uint8_t sId, uint8_t fNum );
uint8_t             sendSetLocFgroup( uint8_t sId, uint8_t fGroup, uint8_t data );

uint8_t             sendSetLocCvMain( uint8_t sId, uint16_t cvId, uint8_t mode, uint8_t val );
uint8_t             sendSetLocCvProg( uint16_t cvId, uint8_t mode, uint8_t val );
uint8_t             sendReqLocCvProg( uint16_t cvId, uint8_t mode );
uint8_t             sendRepLocCvProg( uint16_t cvId, uint8_t val );

uint8_t             sendSetBacc( uint16_t accAdr, uint8_t flags  );
uint8_t             sendSetEacc( uint16_t accAdr, uint8_t val  );

uint8_t             sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3 );
uint8_t             sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4 );
uint8_t             sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5 );
uint8_t             sendDccPacket( uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4, uint8_t arg5, uint8_t arg6 );

uint8_t             sendDccAck( );
uint8_t             sendDccErr( uint8_t errCode, uint8_t arg1 = 0, uint8_t arg2 = 0 );

uint8_t             sendRawMsg( uint8_t *msgBuf );

//----------------------------------------------------------------------------------------------------------
// The driver interface.
//
//----------------------------------------------------------------------------------------------------------
uint8_t             drvReq( uint8_t boardId, uint8_t item, uint16_t arg1 = 0, uint16_t *arg2 = nullptr );

//----------------------------------------------------------------------------------------------------------
// The User Map interface.
//
//----------------------------------------------------------------------------------------------------------
uint8_t             usrNvmPutWord( uint32_t ofs, uint16_t word );
uint8_t             usrNvmGetWord( uint32_t ofs, uint16_t *word );
uint8_t             usrNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
uint8_t             usrNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
uint8_t             usrNvmInitArea( uint32_t ofs, uint32_t len, uint8_t val);
uint32_t            usrNvmGetSize( );

}; // LCS NameSpace

#endif
