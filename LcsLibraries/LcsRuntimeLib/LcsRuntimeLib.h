//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library include file
//
//------------------------------------------------------------------------------------------------------------
// At the heart of the layout control system, LCS, is the runtime library implementing the basic functions. 
// Please refer to the document for information on concepts and implementation notes. This is the external 
// include file for the firmware programmer. All external definitions of key constants and types are 
// included here.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Runtime Library
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
// All LCS Library definitions are in a name space "LCS". You can prefix each constant, type or function 
// with the "LCS::" prefix, or declare using the namespace in your code. 
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//---------------------------------------------------------------------------.--------------------------------
// A node is identified through the node number. Node numbers start with one. The nodeId of zero represents
// the NIL node Id. The node Id is a 12-bit number, so up to 4095 nodes can be addressed. The nodeId, a
// unique Id for the LCS nodes in a layout, is also used as the canId used for the CAN bus. Keep in mind
// that a CAN bus can reasonably handle about 127 nodes at the same time.
//
//------------------------------------------------------------------------------------------------------------
enum LcsNodeId : uint16_t {

    NIL_NODE_ID   = 0,
    MIN_NODE_ID   = 1,
    MAX_NODE_ID   = 4095
};

//------------------------------------------------------------------------------------------------------------
// Nodes have ports. The port Id identifies the port on a given node. Port numbers start with one. The port 
// number zero represents the NIL port number and usually refers to the node itself. A node can have up to
// 15 ports. Often the library functions expect a "node/portId". Which is the concatenation of the 12-bit 
// node Id with the 4-bit port Id.
//
//------------------------------------------------------------------------------------------------------------
enum LcsPortId : uint8_t {

    MIN_PORT_ID   = 0,
    MAX_PORT_ID   = 15
};

//------------------------------------------------------------------------------------------------------------
// Events are just numbers assigned to an event by a configuration tool. Event id numbers start with one. 
// The  event number zero represents the NIL event number. The maximum event id number is 65535. 
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
//------------------------------------------------------------------------------------------------------------
enum DccLocoType : uint8_t {

    LOC_T_NIL       = 0,
    LOC_T_STEAM     = 1,
    LOC_T_DIESEL    = 2,
    LOC_T_ELECTRIC  = 3
};

//------------------------------------------------------------------------------------------------------------
// The base station maintains the locomotive sessions. A session is assigned by the base station and commands
// for the locomotive use this session number. Session Ids start with 1, up to 255 simultaneous sessions are
//  supported.
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
// base stations support just up to 9999 locomotives IDs and so do we. Analog engines do not really have a 
// cabId. Still, they should have a cabId assigned distinct from any DCC capId used. Refer to the book for 
// the details.
//
//------------------------------------------------------------------------------------------------------------
enum DccCabId : uint16_t {

    NIL_CAB_ID  = 0,
    MIN_CAB_ID  = 1,
    MAX_CAB_ID  = 9999
};

//------------------------------------------------------------------------------------------------------------
// A DCC decoder features configuration variables, called CVs. CVs are numbered starting with 1, the maximum
// number is 1024.
//
//------------------------------------------------------------------------------------------------------------
enum DccCvId : uint16_t {

    NIL_DCC_CV_ID  = 0,
    MIN_DCC_CV_ID  = 1,
    MAX_DCC_CV_ID  = 1024
};

//------------------------------------------------------------------------------------------------------------
// "CvModeOptions" is used by the DCC CV variables access routines to specify the access mode. Only options
// 0 and 1 are supported. The others are there for historic reasons, the functionality was found in older
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

//------------------------------------------------------------------------------------------------------------
// A locomotive decoder features up to 69 functions Ids. They are numbered from 0 to 68. The function Id
// 255 is used to indicate a NIL function Id. 
//
//------------------------------------------------------------------------------------------------------------
enum DccFuncId : uint8_t {

    NIL_DCC_FUNC_ID   = 255,
    MIN_DCC_FUNC_ID   = 0,
    MAX_DCC_FUNC_ID   = 68
};

//------------------------------------------------------------------------------------------------------------
// According to the DCC standard, DCC decoder functions are grouped in ten groups, labelled from 1 to 10 .
//
//------------------------------------------------------------------------------------------------------------
enum DccFuncGroupId : uint8_t {

    MIN_DCC_FUNC_GROUP_ID   = 1,
    MAX_DCC_FUNC_GROUP_ID   = 10
};

//------------------------------------------------------------------------------------------------------------
// DCC decoder function mapping Ids. The LCS system defines a set of functions used by the handhelds such
// as horn, lights and so on. These identifiers are standardized for our handhelds and mapped to the DCC 
// function. Note that decoders also feature a function map. This is not to be confused with this mapping. 
//
// ??? align with the cabHandheld functions buttons, etc.
//------------------------------------------------------------------------------------------------------------
enum LcsDccFuncId : uint8_t {

    NIL_LCS_DCC_FUNC_ID     = 0,
    MIN_LCS_DCC_FUNC_ID     = 1,

    LCS_DCC_FUNC_ID_HORN    = 1,
    LCS_DCC_FUNC_ID_BELL    = 2,
    LCS_DCC_FUNC_ID_LIGHTS  = 3,

    // ??? function IDs go here...

    MAX_LCS_DCC_FUNC_ID     = 68
};

//------------------------------------------------------------------------------------------------------------
// The DCC standard defines several speed step modes. Today, the 28 speed step option is the one used in all
// new decoders. The other speed steps are mapped to the 128 value range.
//
//------------------------------------------------------------------------------------------------------------
enum DccSpeedSteps : uint8_t {

    DCC_SPEED_STEPS_14    = 1,
    DCC_SPEED_STEPS_28    = 2,
    DCC_SPEED_STEPS_128   = 3
};

//------------------------------------------------------------------------------------------------------------
// The locomotive decoder speed. The range is defined for a DCC 128 speed step decoder, from 0 to 127. The
// speed of 1 represents the emergency speed stop. In normal operations, speed stops would thus go from 2
// to 0 and back. For analog engines, we keep this scheme and map it to the respective power levels.
//
//------------------------------------------------------------------------------------------------------------
enum LocoSpeed : uint8_t {

    MIN_LOCO_SPEED      = 0,
    ESTOP_LOCO_SPEED    = 1,
    MAX_LOCO_SPEED      = 127
};

//------------------------------------------------------------------------------------------------------------
// Locomotive direction. 
//
//------------------------------------------------------------------------------------------------------------
enum LocoDirection : uint8_t {

    LOCO_DIR_LOCO_NEUTRAL  = 0,
    LOCO_DIR_LOCO_FORWARD  = 1,
    LOCO_DIR_LOCO_REVERSE  = 2
};

//------------------------------------------------------------------------------------------------------------
// "LocSessionModes" specify the options when creating a session for the loco. Besides creating a normal
// session an existing session can be taken over or even shared among multiple handhelds. The base station
// session management will keep track of the session mode.
//
//------------------------------------------------------------------------------------------------------------
enum LocoSessionModes : uint8_t {

    LSM_NORMAL  = 1,
    LSM_STEAL   = 2,
    LSM_SHARED  = 3
};


// ??? merge node and port types ? can a user set this type ?
// a 16-bit word: 4bits controller family, 6 bits major and 6 bits minor version.

//------------------------------------------------------------------------------------------------------------
// A port type can be assigned to a port. Port types start with one. The portType of zero represents the
// NIL port type. A port type is arbitrarily defined by the firmware programmer. The port type for port 
// zero represents the node type.
//
//------------------------------------------------------------------------------------------------------------
  enum LcsPortTypeId : uint16_t {

    NIL_PORT_TYPE     = 0,
    MIN_PORT_TYPE_ID  = 1,
    MAX_PORT_TYPE_ID  = 255
};

// ??? should we generalize this to just type for nodes and ports ?
//------------------------------------------------------------------------------------------------------------
// The defined board types. When the runtime is initialized, the firmware will pass the type to specify what 
// board it expects. This value is compared to what is actually stored in the NVM of the main controller 
// board. If they don't match, it is considered an error and the NVM needs to be configured to support the 
// firmware. 
//
//------------------------------------------------------------------------------------------------------------
enum LcsBoardType : uint16_t {

    BT_NIL                  = 0,
    BT_MAIN_CONTROLLER      = 1,
    BT_BASE_STATION         = 2,
    BT_BLOCK_CONTROLLER     = 3,
    BT_CAB_HANDHELD         = 4,

    BT_EXT_NIL              = 10,
    BT_EXT_OCC_DETECT       = 11,
    BT_EXT_SERVO            = 12,
    BT_EXT_GPIO             = 13
};

//------------------------------------------------------------------------------------------------------------
// The defined chip families. There are controller chip families such as the controller family RP2040, or 
// chip families for the NVM chips used, and so on.
//
//------------------------------------------------------------------------------------------------------------
enum LcsControllerFamilyType : uint16_t {

    CF_FAM_NIL              = 0,
    CF_FAM_RPICO            = 1,
    CF_FAM_MICROCHIP        = 3,
    CF_FAM_NXP              = 4
};

//------------------------------------------------------------------------------------------------------------
// The node and ports have field in the portMap for configuration options that can be set. Most options apply
// only to port zero, which is the node itself. The constants defined here indicate the bit positions and fields
// defined.
//
//  NPO_SKIP_NODE_ID_CONFIG - during startup, skip the nodeId configuration protocol.
//
//  NPO_SKIP_PORT_INIT_STEP - during startup, skip the port initialization step.
//
//  NPO_DEBUG_DURING_SETUP  - during startup print debug info until we use the mask of nodeMap
//
//  NPO_DISABLE_WATCHDOG    - disable the watch dog timer.
//
//  NPO_FORMAT_RUNTIME      - format the non-volatile runtime structures in any case.
//
//------------------------------------------------------------------------------------------------------------
enum LcsNodePortOptions : uint16_t {

    NPO_NIL                     = 0,
    NPO_SKIP_NODE_ID_CONFIG     = ( 1 << 0 ),
    NPO_DEBUG_DURING_SETUP      = ( 1 << 1 ),
    NPO_DISABLE_WATCHDOG        = ( 1 << 2 ),
    NPO_FORMAT_RUNTIME          = ( 1 << 3 ),

};

//----------------------------------------------------------------------------------------------------------
// The portMap entry has a flag field. Again, the flags in port zero refers to the node flags. The constants
// defined here indicate the bit positions and fields defined.
//
//  NPF_PORT_ENABLED                    -   the port is initialized and active. P0 is always enabled.
//
//  NPF_PORT_EVENT_HANDLING_ENABLED     -   the port has event handling enabled.
//
//  NPF_EVENT_PENDING                   -   an event has been received for this port and is pending.
//
//  NPF_EXT_BOARD_PRESENT               -   there is an extension board associated with the port. For P0 
//                                          this fag indicates that there is an extension board at all.
//                                          
//  NPF_EXT_BOARD_VALID                 -   there is a valid extension board associated with the port. 
//                                          This flag only applies to P1 .. P4.
//
//  NPF_EXT_BOARD_READY                 -   there is a valid extension board ready to be used. This flag
//                                          only applies to P1 .. P4.
//
//----------------------------------------------------------------------------------------------------------
enum LcsNodePortFlags : uint16_t {

    NPF_NIL                             = 0,

    NPF_PORT_PRESENT                    = ( 1 << 12 ),            
    NPF_PORT_ENABLED                    = ( 1 << 11 ),
    NPF_PORT_EVENT_HANDLING_ENABLED     = ( 1 << 10 ),
    
    NPF_EVENT_PENDING                   = ( 1 << 9  ),

    NPF_EXT_BOARD_PRESENT               = ( 1U << 8 ),
    NPF_EXT_BOARD_VALID                 = ( 1U << 7 ),
    NPF_EXT_BOARD_READY                 = ( 1U << 6 )
};

//----------------------------------------------------------------------------------------------------------
// The port event action. When an event is received, it will be of the type shown below. 
//
//  PEA_EVENT_IDLE                - the port is idle.
//  PEA_EVENT_ON                  - an "ON" event was received.
//  PEA_EVENT_OFF                 - an "OFF" event was received.
//  PEA_EVENT_EVT                 - an event with additional arguments was received.
//
//----------------------------------------------------------------------------------------------------------
enum LcsPortEventAction : uint8_t {

    PEA_EVENT_IDLE    = 0,
    PEA_EVENT_ON      = 1,
    PEA_EVENT_OFF     = 2,
    PEA_EVENT_EVT     = 3
};

//------------------------------------------------------------------------------------------------------------
// Nodes, ports and drivers are accessed with three main routines, GET, SET and REQ. 
//
// GET - the get routine will use the item numbers to retrieve the data labelled by the item. 
//
// SET - the set routine will use the item numbers to set the value. Note that not all items that can be 
// read can also be written to. An attempt will result in an error return.
//
// REQ - the request call will transmit the request parameters to the node / port / driver where a registered 
// callback or the driver entry point will be invoked. The result is returned via the parameters.
//
// One argument is the item. Items range from  0 ... 255 and are defined as follows: 
//
//   0          -   NIL item, not used
//   1  ..  63  -   Node / port / driver reserved area items, global items for GET/SET/REQ requests.
//  64  .. 127  -   User or driver defined items, specific meaning, accessed via the REQ routine.
// 128  .. 255  -   Node / port / driver data attributes.
//
// The following declarations just list the item numbers defined. The ranges are defined in the internal
// include file. The ranges as well as the reserved items defined here should not be tampered with.
//
// ??? to be sorted when more stable...
// ??? how about an item that allows to set / clear a bit in a mask ?
//------------------------------------------------------------------------------------------------------------
enum LcsItems : uint8_t {

    ITEM_ID_DEBUG_MASK                  = 1,    
    ITEM_ID_OPTIONS                     = 2,
    ITEM_ID_FLAGS                       = 3,

    ITEM_ID_HW_VERSION                  = 5,
    ITEM_ID_SW_VERSION                  = 4,
    ITEM_ID_TYPE                        = 6,
   
    ITEM_ID_CONTROLLER_FAMILY           = 7,
    ITEM_ID_NVM_CHIP_FAMILY             = 8,
   
    ITEM_ID_NODE_STATE                  = 10,
    ITEM_ID_NODE_ID                     = 11,
    ITEM_ID_NODE_UID                    = 12,
    ITEM_ID_RESTART_COUNT               = 13,

    ITEM_ID_PORT_MAP_ENTRIES            = 14,
    ITEM_ID_EVENT_MAP_ENTRIES           = 15,
    ITEM_ID_ATTR_MAP_ENTRIES            = 16,

    ITEM_ID_NAME_1                      = 17,
    ITEM_ID_NAME_2                      = 18,
    ITEM_ID_NAME_3                      = 19,
    ITEM_ID_NAME_4                      = 20,

    ITEM_ID_RESET                       = 22,
    ITEM_ID_SYNC                        = 23,
    ITEM_ID_FORMAT                      = 24,
    
    ITEM_ID_ADD_EVENT_MAP_ENTRY         = 25,
    ITEM_ID_DEL_EVENT_MAP_ENTRY         = 26,
    ITEM_ID_GET_EVENT_MAP_ENTRY         = 27,
    ITEM_ID_EVENT_DELAY_TICKS           = 21,
    ITEM_ID_ENABLE_EVENT_PROCESSING     = 40,

    ITEM_ID_ACTIVE_LED                  = 34,
    
    // ??? add stop and enable periodic processing ?
};

//------------------------------------------------------------------------------------------------------------
// The debug mask. The library has a debug mask where each major part of the library has a flag. There could 
// also be flags reserved for the firmware. There is an ITEM request code to read and set this mask. Wherever
// debugging is needed, the bit mask will be used to determine whether to print debugging data or not. From
// a performance perspective, the test will take just a few instructions. In other words we do not take out
// debugging code when going into production. Never liked this approach of conditional debug code anyway.
//
// The usage of the debug mask is generally: 
//
//      if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_xxx )) ....
// 
// The DBG_CONFIG bit allows for the entire debugging messages to be enabled or disabled. This feature will 
// also be used when we test whether we even have a console or not. If there is no console, all the prints
// will not be executed.
//
//------------------------------------------------------------------------------------------------------------
enum DebugOtions : uint16_t {

    LCS_DBG_CONFIG          = ( 1U << 15 ),
    LCS_DBG_SETUP           = ( 1U << 0 ),
    LCS_DBG_NVM_ACCESS      = ( 1U << 1 ),
    LCS_DBG_CAN_BUS         = ( 1U << 2 ),
    LCS_DBG_MSG_BUS         = ( 1U << 3 ),
    LCS_DBG_ATTRIBUTES      = ( 1U << 4 ),
    LCS_DBG_EVENTS          = ( 1U << 5 )
};

//---------------------------------------------------------------------------------------------------------
// The message operation code identifies the LCS bus message. It is always the first data byte of the 
// message. We encode the number of payload data bytes in the first three bits of the opCode. For each 
// message length there is a maximum of 32 opCode possible. The OPC  macro helps to define the opcodes. 
// The first argument is the length of the data bytes, the second the opcodeId within the group.
//
// ??? note: this list is work in progress, please us always the names rather than the numbers.
//----------------------------------------------------------------------------------------------------------
#define OPC( len, id ) ((uint8_t) (( len << 5 ) + ( id & 0x1F )))

enum LcsMsgOpCodes : uint8_t {

    LCS_OP_NO_MSG           = OPC( 0, 0 ),
    LCS_OP_CFG              = OPC( 0, 1 ),
    LCS_OP_OPS              = OPC( 0, 2 ),
    LCS_OP_BON              = OPC( 0, 3 ),
    LCS_OP_BOF              = OPC( 0, 4 ),
   
    LCS_OP_REL_LOC          = OPC( 1, 1 ),
    LCS_OP_QRY_LOC          = OPC( 1, 2 ),
    LCS_OP_KEEP_LOC         = OPC( 1, 3 ),
    LCS_OP_ESTP             = OPC( 1, 4 ),

    LCS_OP_PING             = OPC( 2, 1 ),
    LCS_OP_ACK              = OPC( 2, 2 ),
    LCS_OP_DCC_ACK          = OPC( 2, 3 ),
    LCS_OP_SET_LSPD         = OPC( 2, 4 ),
    LCS_OP_SET_LMOD         = OPC( 2, 5 ),
    LCS_OP_LOC_FON          = OPC( 2, 6 ),
    LCS_OP_LOC_FOF          = OPC( 2, 7 ),
    LCS_OP_BACC             = OPC( 2, 8 ),
    LCS_OP_EACC             = OPC( 2, 9 ),
    LCS_OP_TON              = OPC( 2, 10 ),
    LCS_OP_TOF              = OPC( 2, 11 ),

    LCS_OP_RESET            = OPC( 3, 1 ),
    LCS_OP_SYNC             = OPC( 3, 2 ),
    LCS_OP_REQ_LOC          = OPC( 3, 3 ),
    LCS_OP_SET_LCON         = OPC( 3, 4 ),
    LCS_OP_LOC_FGRP         = OPC( 3, 5 ),
    LCS_OP_SEND_DCC3        = OPC( 3, 6 ),
    LCS_OP_DCC_ERR          = OPC( 3, 7 ),
    LCS_OP_REQ_CVS          = OPC( 3, 8 ),
    
    LCS_OP_EVT_ON           = OPC( 4, 1 ),
    LCS_OP_EVT_OFF          = OPC( 4, 2 ),
    LCS_OP_SEND_DCC4        = OPC( 4, 3 ),
    LCS_OP_REP_CVS          = OPC( 4, 4 ),
    LCS_OP_SET_CVS          = OPC( 4, 5 ),
    LCS_SYS_TIME            = OPC( 4, 6 ),

    LCS_OP_ERR              = OPC( 5, 1 ),
    LCS_OP_SET_CVM          = OPC( 5, 2 ),
    LCS_OP_SEND_DCC5        = OPC( 5, 3 ),

    LCS_OP_EVT              = OPC( 6, 1 ),
    LCS_OP_SEND_DCC6        = OPC( 6, 2 ),
    LCS_OP_NCOL             = OPC( 6, 6 ),

    LCS_OP_REQ_NID          = OPC( 7, 1 ),
    LCS_OP_REP_NID          = OPC( 7, 2 ),
    LCS_OP_SET_NID          = OPC( 7, 3 ),
    LCS_OP_NODE_GET         = OPC( 7, 4 ),
    LCS_OP_NODE_PUT         = OPC( 7, 5 ),
    LCS_OP_NODE_REQ         = OPC( 7, 6 ),
    LCS_OP_NODE_REP         = OPC( 7, 7 ),
    LCS_OP_REP_LOC          = OPC( 7, 8 ),
    LCS_INFO                = OPC( 7, 9 )
};

//----------------------------------------------------------------------------------------------------------
// LCS Core Library Error codes. The status code is used as a return value from most of the library methods.
// The numbers are grouped in a LCS library portion and a user firmware portion. The LCS library portion
// ranges from 1 to 127, the user portion from 128 to 255. The value of zero is generally an "OK".
//
//----------------------------------------------------------------------------------------------------------
enum LcsErrorCodes : uint8_t {

    ALL_OK                              = 0,
    ERR_NOT_IMPLEMENTED                 = 1,
    ERR_NOT_SUPPORTED                   = 2,
    ERR_LIB_NOT_INITIALIZED             = 3,
    ERR_LIB_NOT_READY                   = 4,

    ERR_CDC_SETUP                       = 10,
    ERR_NVM_SETUP                       = 11,
    ERR_MEM_SETUP                       = 12,
    ERR_CAN_SETUP                       = 13,

    ERR_NVM_CHIP_SIZE_DETECT            = 14,
    ERR_NVM_NODE_MAP_CORRUPT            = 15,
    ERR_NVM_SIZE_EXCEEDED               = 16,
    ERR_MEM_SIZE_EXCEEDED               = 17,
    ERR_NVM_OP_FAILED                   = 18,

    ERR_INVALID_OP_FOR_NODE_STATE       = 20,
    ERR_NODE_NOT_OPS_STATE              = 21,
    ERR_NODE_NOT_CONFIG_STATE           = 22,
    ERR_NODE_OUTSTANDING_REQ_LIMIT      = 23,
    ERR_TASK_MAP_SIZE_EXCEEDED          = 24,

    ERR_INVALID_NODE_ID                 = 30,
    ERR_INVALID_PORT_ID                 = 31,
    ERR_INVALID_ITEM_ID                 = 32,
    ERR_INVALID_EVENT_ID                = 33,
    ERR_INVALID_BOARD_ID                = 34,
    ERR_INVALID_DRV_ITEM                = 35,
    ERR_INVALID_ATTR_ARG                = 36,

    ERR_INVALID_EVENT_MAP_INDEX         = 51,
    ERR_EVENT_MAP_FULL                  = 52,
    ERR_PENDING_REQ_MAP_FULL            = 53,
    ERR_REQ_TIMEOUT                     = 54,

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

    ERR_DRV_FUNC_MAP_FULL               = 75,    
    ERR_DRV_PUT_ERR                     = 76,
    ERR_DRV_GET_ERR                     = 77,

    ERR_CAN_BUS_INIT                    = 81,
    ERR_CAN_INVALID_MODE                = 82,
    ERR_CAN_BUS_MSG_SIZE                = 83,
    ERR_CAN_MSG_SEND                    = 84,
    ERR_CAN_MSG_RECV                    = 85,
    ERR_CAN_MSG_NO_MSG                  = 86,
    ERR_CAN_ID_COLLISION                = 87,
    ERR_CAN_ID_CHANGED                  = 88,

    ERR_EXT_BOARD_NOT_VALID             = 90,

    ERR_USER_SPECIFIC_BASE              = 128
};

//------------------------------------------------------------------------------------------------------------
// The CAN bus mode. The PICO_PIO_xxx modes use the Raspberry Pi Pico "can2040" library, which is a software
// implementation of the CAN bus. The "can2040" library could run on the same or on the separate processor 
// core. Technically, the PICO could also run the MCP2515 via the SPI interface, but so far we just use the
// software version and avoid the additional controller hardware.
//
//------------------------------------------------------------------------------------------------------------
enum CanBusControllerMode : uint8_t {

    CAN_BUS_LIB_PICO_PIO_125K               = 1,
    CAN_BUS_LIB_PICO_PIO_250K               = 2,
    CAN_BUS_LIB_PICO_PIO_500K               = 3,
    CAN_BUS_LIB_PICO_PIO_1000K              = 4,

    CAN_BUS_LIB_PICO_PIO_125K_M_CORE        = 11,
    CAN_BUS_LIB_PICO_PIO_250K_M_CORE        = 12,
    CAN_BUS_LIB_PICO_PIO_500K_M_CORE        = 13,
    CAN_BUS_LIB_PICO_PIO_1000K_M_CORE       = 14,
};

//------------------------------------------------------------------------------------------------------------
// "MsgPriority" defines the values for the message priority. It tracks the general definition found in the
// sendMsg routines of the LCS library. For the CAN bus, the priority is encoded in the CAN address field.
// A CAN Id consists of the CAN Id number and the priority. Messages start out with a hard coded priority and
// on message timeout are raised in their priority. This done transparently to the firmware programmer.
//
//------------------------------------------------------------------------------------------------------------
enum MsgPriority : uint8_t {

    MSG_PRI_VERY_HIGH   = 0,
    MSG_PRI_HIGH        = 1,
    MSG_PRI_NORMAL      = 2,
    MSG_PRI_LOW         = 3
};

//------------------------------------------------------------------------------------------------------------
// Core library callback function signatures. The entire runtime works with a set of APIs to invoke a 
// function and callbacks to get any results. Callbacks are registered by the firmware at setup time.
//
//      LcsInitCallback     -   a callback called at initialization time, as part of "startRuntime". The
//                              npId is passed so that the callback can detect wether a port or node is 
//                              the target.
//
//      LcsPfailCallback    -   a callback when a power fail situation is detected. The npId is passed 
//                              so that the callback can detect wether a port or node is the target.
// 
//      LcsMsgCallback      -   is called with a LCS management message received.
//
//      LcsCmdCallback      -   when the command interpreter detects a non LCS command, the command line 
//                              is passed on to the callback. 
//      LcsTaskCallback     -   a callback for a previously registered task. The callback is invoked on 
//                              the configured periodic basis.
//
//      LcsReqCallback      -   a callback to invoke for a user request message. The callback is passed 
//                              the item and a reference to the two input / output arguments. A request
//                              callback is associated with a port and is either a user defined callback
//                              or in case of a port associated with a port a driver request function.
//      
//      LcsRepCallback      -   a callback to return the reply message for a previous LCS message sent. The
//                              reply can be a data reply, an ACK or NACK or a timeout error. The arguments 
//                              are the item that was requested, the arguments and the return status of the
//                              operation.
//
//      LcsEventCallback    -   a callback for a received event. The arguments are the issuing npId, the
//                              event type and the optional arguments.
//          
// All callback functions need to return a status, which is ALL_OK if the callback was successful.
// 
//------------------------------------------------------------------------------------------------------------
extern "C" {

    typedef uint8_t ( *LcsInitCallback ) ( uint16_t npId );
    typedef uint8_t ( *LcsPfailCallback ) ( uint16_t npId );
    typedef uint8_t ( *LcsMsgCallback ) ( uint8_t *msg );
    typedef uint8_t ( *LcsCmdCallback ) ( char *cmdLine );
    typedef uint8_t ( *LcsTaskCallback ) ( void );
    
    typedef uint8_t ( *LcsReqCallback ) ( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 );
    typedef uint8_t ( *LcsRepCallback ) ( uint16_t npId, uint8_t item, uint16_t arg1, uint16_t arg2, uint8_t ret );

    typedef uint8_t ( *LcsEventCallback ) ( uint16_t npId, uint16_t eId, uint8_t eAction, uint16_t eData );
}

//------------------------------------------------------------------------------------------------------------
// Library functions. The main function are the initialization and start of the LCS runtime. Between "init"
// and "start", the firmware should do its own setup and register the necessary callbacks. We will not return
// from the "start" routine. All calls are a plain C style library calls. 
//
// The "initRuntime" routine is passed the resource map. Important. All there is to know about the particular 
// board and resources to configure comes from this map.
// 
//------------------------------------------------------------------------------------------------------------
uint8_t             initRuntime( CDC::CdcResourceMap *cMap );
uint8_t             startRuntime( );

//------------------------------------------------------------------------------------------------------------
// Routines to access the node/port GET/SET/REQ items. The first argument is the node/port Id. A node Id of 
// zero refers to the local node and the calls are direct procedure calls. A non-zero node will refer to
// another node, and a message is broadcasted.
//
// ??? howe about a scheme where we have blocking calls ? When we send the request, the reply will contain
// the reply node and port. So, we could filter on that node and port. It would mean however that you will
// wait ( perhaps with a timeout ) for the outstanding request.
//------------------------------------------------------------------------------------------------------------
uint8_t             nodeGet( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 = nullptr );
uint8_t             nodePut( uint16_t npId, uint8_t item, uint16_t arg1, uint16_t arg2 = 0 );
uint8_t             nodeReq( uint16_t npId, uint8_t item, uint16_t *arg1 = nullptr, uint16_t *arg2 = nullptr );

//------------------------------------------------------------------------------------------------------------
// Function registration routines for callbacks, tasks, driver types, etc.
//
//------------------------------------------------------------------------------------------------------------
uint8_t             registerInitCallback( LcsInitCallback handler );
uint8_t             registerPfailCallback( LcsPfailCallback handler );
uint8_t             registerLcsMsgCallback( LcsMsgCallback functionId );
uint8_t             registerDccMsgCallback( LcsMsgCallback functionId );
uint8_t             registerCmdCallback( LcsCmdCallback functionId );
uint8_t             registerTaskCallback( LcsTaskCallback task, uint32_t interval = 0 );
uint8_t             registerEventCallback( LcsEventCallback functionId, uint16_t portMask = 0xFFFF );
uint8_t             registerReqCallback( LcsReqCallback handler, uint16_t portMask = 0xFFFF );
uint8_t             registerRepCallback( LcsRepCallback handler, uint16_t portMask = 0xFFFF );
uint8_t             registerDrvFunc(  uint16_t drvType, LcsReqCallback drvReqFunction );

//------------------------------------------------------------------------------------------------------------
// A set of convenience functions to send an LCS message.
//
//------------------------------------------------------------------------------------------------------------
uint8_t             sendCfg( uint16_t npId );
uint8_t             sendOps( uint16_t npId );
uint8_t             sendReset( uint16_t npId );
uint8_t             sendBusOn( );
uint8_t             sendBusOff( );
uint8_t             sendPing( uint16_t npId );
uint8_t             sendAck( uint16_t npId );
uint8_t             sendErr( uint16_t npId, uint8_t errCode, uint8_t arg1 = 0, uint8_t arg2 = 0 );

uint8_t             sendReqNodeId( uint16_t npId, uint32_t nodeUID, uint8_t flags );
uint8_t             sendRepNodeId( uint16_t npId, uint32_t nodeUID );
uint8_t             sendSetNodeId( uint16_t npId, uint32_t nodeUID );
uint8_t             sendNodeIdCollision( uint16_t npId, uint32_t nodeUID );

uint8_t             sendGetNode( uint16_t npId, uint8_t item, uint16_t arg1 = 0, uint16_t arg2 = 0 );
uint8_t             sendSetNode( uint16_t npId, uint8_t item, uint16_t arg1 = 0, uint16_t arg2 = 0 );
uint8_t             sendRepNode( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2  );
uint8_t             sendReqNode( uint16_t npId, uint8_t item, uint16_t val1, uint16_t val2  );

uint8_t             sendEventOn( uint16_t npId, uint16_t eventId );
uint8_t             sendEventOff( uint16_t npId, uint16_t eventId );
uint8_t             sendEvent( uint16_t npId, uint16_t eventId, uint16_t arg );

uint8_t             sendTrackOn( );
uint8_t             sendTrackOff( );
uint8_t             sendEstop( );

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
void                printLcsMs( uint8_t *msgBuf );

//----------------------------------------------------------------------------------------------------------
// The User Map interface. The LCS library offers a set of routines for the firmware to access the user
// NVM area. The size is dependent on what the actual chip on the board offers. The meaning of this data
// area is entirely firmware specific. Note that there are also routines for accessing the runtime data 
// area as well as the individual extension board areas. They are declared in the internal include file.
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
