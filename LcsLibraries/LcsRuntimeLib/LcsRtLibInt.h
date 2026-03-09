//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library internal include file
//
//----------------------------------------------------------------------------------------
// The LCS library internal definitions are all grouped in this include file. A
// firmware writer only includes the external include file. There is nothing in
// here that is required outside.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library internals include file
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
#pragma once

//----------------------------------------------------------------------------------------
// Include files. Besides the standard C libraries, there is the external LCS runtime
// include file, and the dependent code library include file.
//
//----------------------------------------------------------------------------------------
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>
#include <inttypes.h>
#include <cstring>

#include "LcsRtLibVersion.h"
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//----------------------------------------------------------------------------------------
// Namespace LCS declarations
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// The LCS Runtime needs to maintain a couple of internal data structures. As a 
// general concept, most of the data areas are stored in the NVM. Upon reset or 
// power up the memory areas are initialized from their NVM counter parts. 
// 
// Data that needs to be changed permanently is flushed from memory to NVM so that
// it is the initial value on the next restart. All data is stored in controller 
// native endianness. Only the messages exchanged via the LcsMsgBus are transmitted
// in big endian order.
//
// The NVM layout is a fixed one. We have the nodeMap starting at NVM_NODE_MAP_START
// offset. All data areas follow. Their starting offset is the sum of the size of 
// the previous structures starting. The system area needs to be at least the size 
// of the declared structures up to the use map. The optional user map occupies all
// the remaining bytes in the NVM. 
//
//          :-------------------------------------------:       NVM_NODE_MAP_START 
//          :                                           :
//          :       NVM Header Map                      :
//          :                                           :
//          :-------------------------------------------:       + sizeof( LcsBoardDesc ) 
//          :                                           :
//          :       Node Map                            :
//          :                                           :
//          :-------------------------------------------:       + sizeof( NodeMap )
//          :                                           :
//          :       Mode Map Data                       :
//          :                                           :
//          :-------------------------------------------:       + sizeof( NodeMapData )
//          :                                           :
//          :       Event Map                           :
//          :                                           :
//          :-------------------------------------------:       + sizeof( EventMap )
//          :                                           :
//          :                                           :
//          :                                           :
//          :       Optional User Map                   :
//          :                                           :
//          :                                           :
//          :                                           :
//          :-------------------------------------------:       0xNNNN  
//
// All data areas have a fixed size which is the maximum size for the particular map.
// For example, there is room for 16 ports, even if the firmware would only use let's
// say 4. In general, each of the runtime map also could have been designed in a way 
// that they are dynamically configurable in size. Considering the size and price of
// NVM chips as well as the memory size of the supported controller platforms, using
// fixed sizes for each map avoids configuration complexity.
//
//----------------------------------------------------------------------------------------
const uint16_t  MAX_NVM_HEADER_MAP_ENTRIES      = 5;
const uint16_t  MAX_PORT_MAP_ENTRIES            = 8;
const uint16_t  MAX_CHANNEL_MAP_ENTRIES         = 8;
const uint16_t  MAX_EVENT_MAP_ENTRIES           = 768;
const uint16_t  MAX_TASK_MAP_ENTRIES            = 16;
const uint16_t  MAX_EXT_BOARD_MAP_ENTRIES       = 4;
const uint16_t  MAX_NODE_DATA_BLOCKS            = MAX_PORT_MAP_ENTRIES;
const uint16_t  MAX_ATTR_MAP_ENTRIES            = 128;
const uint8_t   MAX_DRV_TYPE_MAP_ENTRIES        = 8;

//----------------------------------------------------------------------------------------
// Other common constants.
//
//----------------------------------------------------------------------------------------
const uint16_t  MAX_LCS_MSG_SIZE                = 8;
const uint16_t  MAX_COMMAND_LINE_SIZE           = 256;
const uint16_t  EVENT_DELAY_TICK_MILLIS         = 32;

//----------------------------------------------------------------------------------------
// The default sizes for the chips on the board. For the main boards, we will figure
// out the real size during startup. For the extension boards, the size is fixed 
// for now.
//
//----------------------------------------------------------------------------------------
const int       NVM_MAIN_BOARD_DEF_SIZE         = 16 * 1024;
const int       NVM_EXT_BOARD_DEF_SIZE          = 4 * 1024;

//----------------------------------------------------------------------------------------
// The maps have as their first word a magic word, which is just a special constant. We 
// simply read in that word and check it for being a valid word for the particular map. 
// If valid, the area was configured before and we can do further checking. It would be 
// quite unlikely that a random NVM content has this word at the right spot. 
//
//----------------------------------------------------------------------------------------
const uint32_t NVM_MWORD_MAIN           = (uint32_t) ( 'L' << 24 ) | 
                                                     ( 'C' << 16 ) | 
                                                     ( 'S' << 8 );

const uint32_t NVM_MWORD_EXTENSION      = (uint32_t) ( 'L' << 24 ) | 
                                                     ( 'C' << 16 ) | 
                                                     ( 'E' << 8 );

const uint32_t NVM_MWORD_NODE_HEADER    = NVM_MWORD_MAIN | 0x01;
const uint32_t NVM_MWORD_NODE_MAP       = NVM_MWORD_MAIN | 0x02;
const uint32_t NVM_MWORD_NODE_DATA_MAP  = NVM_MWORD_MAIN | 0x03;
const uint32_t NVM_MWORD_EVENT_MAP      = NVM_MWORD_MAIN | 0x04;

const uint32_t NVM_MWORD_EXT_HEADER     = NVM_MWORD_EXTENSION | 0x01;

//----------------------------------------------------------------------------------------
// The node states. Essentially, the node runtime is a big state machine. The node 
// starts in the INIT state and once all is initialized and registered ends up in the
// OPS or CFG mode.
//
//  NS_NIL          -   NIL.
//  NS_FAIL         -   The startup failed.
//  NS_PFAIL        -   The startup detected that we restarted after a power failure.
//  NS_INIT         -   The node entered the startup state.
//  NS_REGISTER     -   The node entered the node register state to get the nodeId.
//  NS_COLLISION    -   The node detected a nodeId collision on the LCS bus and stopped.
//  NS_HALTED       -   The node was halted.
//  NS_CONFIG       -   The node is in configuration mode.
//  NS_OPERATE      -   The node is on operations mode.
//
//----------------------------------------------------------------------------------------
enum LcsNodeState : uint16_t {
    
    NS_NIL              = 0,
    NS_FAIL             = 1,
    NS_PFAIL            = 2,
    NS_INIT             = 3,
    NS_REGISTER         = 4,
    NS_COLLISION        = 5,
    NS_HALTED           = 6,
    NS_CONFIG           = 7,
    NS_OPERATE          = 8
};

//----------------------------------------------------------------------------------------
// Node, port and extension board driver attributes and functions are accessed with 
// three main routines, GET, SET and REQ. The specific items are defined in the 
// external include file. This part here defined the boundaries for internal checking.
// The first 64 items are predefined attributes and reserved for the runtime itself. 
// Item 64 to 127 are reserved for library and firmware functions. Item 128
// to 255 are user definable attributes.
//
// ??? we should use a 16-bit quantity and add extended attributes range.
//----------------------------------------------------------------------------------------
enum ItemRanges : uint8_t {

    IR_NIL                      = 0,

    IR_LIB_MAP_RANGE_START      = 1,
    IR_LIB_MAP_RANGE_END        = 63,

    IR_DRV_FUNCTION_START       = 64,
    IR_DRV_FUNCTION_END         = 127,

    IR_USER_RANGE_START         = 128,
    IR_USER_RANGE_END           = 255,
 
    IR_MAX_ITEMS                = 255,
};

//----------------------------------------------------------------------------------------
// "MsgPriority" defines the values for the message priority. It tracks the general 
// definition found in the sendMsg routines of the LCS library. For the CAN bus, 
// the priority is encoded in the CAN address field. A CAN Id consists of the CAN Id 
// number and the priority. Messages start out with a hard coded priority and on 
// message timeout are raised in their priority. This done transparently to the 
// firmware programmer.
//
//----------------------------------------------------------------------------------------
enum MsgPriority : uint8_t {

    MSG_PRI_VERY_HIGH   = 0,
    MSG_PRI_HIGH        = 1,
    MSG_PRI_NORMAL      = 2,
    MSG_PRI_LOW         = 3
};

//----------------------------------------------------------------------------------------
// "LcsMsgBusCAN" is the CAN bus interface. The two key routines are the send and 
// receive routines. A CAN bus message will use the nodeId as the canBus Id.
//
//----------------------------------------------------------------------------------------
struct LcsMsgBusCAN {

    public:

    uint8_t     init(   uint8_t  rxPin, 
                        uint8_t  txPin, 
                        uint32_t baudRate   = 125000, 
                        bool     twoCores   = false );

    uint8_t     sendLcsMsg ( uint16_t sendingNpId, 
                             uint8_t *msgBuf, 
                             uint8_t msgPri = MSG_PRI_NORMAL );

    uint8_t     receiveLcsMsg( uint16_t *senderNpId, 
                               uint8_t *msg );

    void        setNodeId( uint8_t nodeId );

    private: 

    uint8_t localNodeId = 0;
};

//----------------------------------------------------------------------------------------
// The CdcBoardDescMap structure defines what the board actually represents. It is 
// also the first structure that can be found on the controller board NVM as well 
// as the extension board NVM. For the smart extension boards that have a controller
// on board themselves, the controller firmware is expected to return this data
// just as if we read it from the NVM. An Atmega Attiny controller board also has
// the nice property of a serial number. We use it for I2C bus collision detection.
//
// The header structure is 32 bytes long.
//
//----------------------------------------------------------------------------------------
struct LcsBoardDesc {

    uint32_t            boardMword;
    uint16_t            boardInfo;                      // type/subtype
    uint16_t            boardCtrlInfo;                  // family / cType
    uint16_t            boardVersion;                   // major / sub version
    uint16_t            reserved[ 7 ];                  // future use
    uint16_t            serialNum1;                     // serial number part 1
    uint16_t            serialNum2;                     // serial number part 2
    uint16_t            serialNum3;                     // serial number part 3
    uint16_t            serialNum4;                     // serial number part 4 
};

//----------------------------------------------------------------------------------------
// The NVM header map stores the NVM headers of the node board and the optional 
// extension boards found. It is a MEM only structure and will be filled though 
// reading the NVM headers at startup time. There should be at least the main 
// controller board NVM header stored and optional up to four extension board
// NVM headers. 
//
// ??? why would we need it, if we can get to the boards anytime ?
//----------------------------------------------------------------------------------------
struct LcsHeaderMap {

    LcsBoardDesc map[ MAX_NVM_HEADER_MAP_ENTRIES ] = { 0 };
};

//----------------------------------------------------------------------------------------
// The nodeMap is the heart of all data on the node. When bringing up a node, we 
// first read in the NVM header and then the node map. Once read in from the NVM 
// storage, several validity checks are performed. The most important check is to 
// compare the size of the various data structures with the runtime data size that 
// we know from the compilation. That is the reason that each NVM area starts with 
// a  magic word. Only when they match can we somewhat trust the rest of the data 
// maps to read.
//
// During configuration and operation state, update to nodeMap fields can also be 
// forwarded to their NVM counterpart. Since a node has port zero as the port for
// node wide operations, the portMap entry 0 is also considered part of the node map.
//
//----------------------------------------------------------------------------------------
struct LcsNodeMap {

    uint32_t            magicWord                       = NVM_MWORD_NODE_MAP;
    uint32_t            nvmOfs                          = 0;
    uint32_t            nvmSize                         = sizeof( LcsNodeMap );
    uint16_t            rtLibSwVersion                  = LCS_RT_LIB_VERSION;
    uint16_t            rtLibSwPatchLevel               = LCS_RT_LIB_PATCH_LEVEL;

    uint16_t            nodeState                       = NS_NIL;
    uint16_t            nodeId                          = NIL_NODE_ID;
    uint32_t            nodeUID                         = 0L;
    uint16_t            nodeRestartCnt                  = 0;
    uint32_t            nodeSystemTime                  = 0;
   
    LcsInitCallback     initCallback                    = nullptr;
    void                *initCallBackUdata              = nullptr;

    LcsPfailCallback    pfailCallback                   = nullptr;
    void                *pfailCallBackUdata             = nullptr;

    LcsMsgCallback      lcsMsgCallback                  = nullptr;
    void                *lcsMsgCallBackUdata            = nullptr;

    LcsMsgCallback      dccMsgCallback                  = nullptr;
    void                *dccMsgCallBackUdata            = nullptr;

    LcsCmdCallback      cmdLineCallback                 = nullptr;
    void                *cmdLineCallBackUdata           = nullptr;
};

//----------------------------------------------------------------------------------------
// Node and ports each have an area of variables that are in memory as well as in
// the node NVM. Typical usage examples are configuration items such as a limit 
// value. Upon power up the header structure is validated and the node data from 
// the NVM area is copied to the MEM counterpart. Although the node and port 
// attributes are logically part of the portMap and nodeMap, they are kept in this
// separate structure. This way, index calculation and access becomes rather
// straightforward.
//
//----------------------------------------------------------------------------------------
struct LcsNodeData {

    uint32_t    magicWord   = NVM_MWORD_NODE_DATA_MAP;
    uint32_t    nvmOfs      = 0;
    uint32_t    nvmSize     = sizeof( LcsNodeData );    

    uint16_t    map[ MAX_PORT_MAP_ENTRIES ][ MAX_ATTR_MAP_ENTRIES ] = { 0 };
};

//----------------------------------------------------------------------------------------
// The event map entry contains the mapping from eventId to portId. When a node 
// or port is interested in an event, there will be an entry with event Id and 
// the port mask, where each port has a bit. The map is sorted by event Id and 
// searched for an incoming event to find the ports that are interested in the 
// event. 
//
//----------------------------------------------------------------------------------------
struct LcsEventMapEntry {

    uint16_t eventId    = NIL_EVENT_ID;
    uint16_t eventMask  = 0;
};

struct LcsEventMap {

    uint32_t            magicWord   = NVM_MWORD_EVENT_MAP;
    uint32_t            nvmOfs      = 0;
    uint32_t            nvmSize     = sizeof( LcsEventMap );
    uint32_t            mapHwm      = 0;

    LcsEventMapEntry    map[ MAX_EVENT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// The port map contains an array of ports, each described by a port map entry. 
// There are 8 entries in the port map. Each port has an area of attributes, 
// which are stored in the data block area. They map to ITEM numbers 128 to 255 
// and are accessed via GET/SET calls. 
//
// The item numbers 1 to 63 are reserved for node/port specific purposes, the 
// item numbers 64 to 127 are reserved for driver library functions. 
// 
// A port support up to 8 channels. Each channel corresponds to an I2C address
// and is our way to access hardware, such as an extension board or a satellite
// board. All channels on a port must have the same channel type.
//
// The portMap entry furthermore contains the fields that deal with the actual 
// event received that the port is interested in. There are fields for the sending
// node, the event itself and its action. An event can also be invoked with a 
// delay time.
//
// When a request os send, the target npId and a request time limit timestamp
// are stored in the port map entry. This way, when a reply comes in, we can check
// if it is expected and invoke the reply callback. On a port, one request can
// be pending at a time. If another request is sent before the reply, it is an
// error.
//
// ??? should we support that all channels can have an outstanding request ?
//----------------------------------------------------------------------------------------
struct LcsPortMapEntry {

    uint16_t            flags                       = 0;
    uint16_t            type                        = 0;
    uint16_t            lastErr                     = 0;
   
    LcsReqCallback      reqCallback                 = nullptr;
    void                *reqCallBackUdata           = nullptr;

    LcsRepCallback      repCallback                 = nullptr;
    void                *repCallBackUdata           = nullptr;

    LcsRepCallback      drvReqCallback              = nullptr;
    void                *drvReqCallBackUdata        = nullptr;

    LcsEventCallback    eventCallback               = nullptr;
    void                *eventCallBackUdata         = nullptr;

    uint16_t            eventNpId                   = 0;
    uint16_t            eventId                     = NIL_EVENT_ID;
    uint16_t            eventValue                  = 0;
    uint16_t            eventAction                 = PEA_EVENT_IDLE;
    uint16_t            eventDelayTime              = 0;
    uint32_t            eventTimeStamp              = 0L;

    uint16_t            targetNpId                  = 0;
    uint32_t            targetReqTs                 = 0; 

    uint16_t            channelMask                 = 0;

    // ??? do we need a kind of channel state ?
};

struct LcsPortMap {

    uint32_t        mapHwm = 0;
    LcsPortMapEntry map[ MAX_PORT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// The core library maintains an array of periodic tasks. The entry maintains the
// task procedure label, the time it ran the last time, and the minimum interval
// between invocations. Note that the timing is not very accurate, but it is 
// guaranteed that a task will eventually run when the interval is reached.
//
//----------------------------------------------------------------------------------------
struct LcsPTaskMapEntry {

    LcsTaskCallback     task        = nullptr;
    void                *uData      = nullptr; 
    uint32_t            timeStamp   = 0;
    uint32_t            interval    = 0;
};

struct LcsTaskMap {

    uint32_t            mapHwm      = 0;
    LcsPTaskMapEntry    map[ MAX_TASK_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// An extension board is associated with a port and thus has attributes and request 
// items. Attributes are naturally accessed via the GET/PUT calls. The extension 
// board specific request functions are accessed via the REQ calls. The firmware is
// required to register a callback, i.e. driver, for each driver type used. The type
// and function label for extension boards are kept in the driver function map. 
//
//----------------------------------------------------------------------------------------
struct LcsDrvFuncEntry {

    uint16_t        drvType = CDC_BT_NIL;
    LcsReqCallback  drvFunc = nullptr;
    void            *uData  = nullptr;
};

struct LcsDrvFuncMap {

    uint32_t        mapHwm = 0;
    LcsDrvFuncEntry map[ MAX_DRV_TYPE_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// The layout of the NVM storage is fixed. There are the header and the maps 
// allocated in a given order and size. We can easily define the relevant offsets
// and sizes as constants.
//
//----------------------------------------------------------------------------------------
const uint32_t  NVM_BOARD_DESC_SIZE         =   sizeof( LcsBoardDesc );
const uint32_t  NVM_NODE_MAP_SIZE           =   sizeof( LcsNodeMap );
const uint32_t  NVM_NODE_DATA_SIZE          =   sizeof( LcsNodeData );
const uint32_t  NVM_EVENT_MAP_SIZE          =   sizeof( LcsEventMap );

const uint32_t  NVM_MAP_STORAGE_START       =   0;

const uint32_t  NVM_RUNTIME_MAPS_SIZE       =   NVM_BOARD_DESC_SIZE + 
                                                NVM_NODE_MAP_SIZE   +
                                                NVM_NODE_DATA_SIZE  +
                                                NVM_EVENT_MAP_SIZE;

const uint32_t  NVM_HEADER_MAP_OFS          =   NVM_MAP_STORAGE_START;  

const uint32_t  NVM_NODE_MAP_OFS            =   NVM_MAP_STORAGE_START + 
                                                NVM_BOARD_DESC_SIZE;

const uint32_t  NVM_NODE_DATA_OFS           =   NVM_MAP_STORAGE_START + 
                                                NVM_BOARD_DESC_SIZE   +  
                                                NVM_NODE_MAP_SIZE;
    
const uint32_t  NVM_EVENT_MAP_OFS           =   NVM_MAP_STORAGE_START + 
                                                NVM_BOARD_DESC_SIZE   +
                                                NVM_NODE_MAP_SIZE     +
                                                NVM_NODE_DATA_SIZE;
    
const uint32_t  NVM_USER_MAP_OFS            =   NVM_MAP_STORAGE_START + 
                                                NVM_BOARD_DESC_SIZE   + 
                                                NVM_NODE_MAP_SIZE     +
                                                NVM_NODE_DATA_SIZE    +
                                                NVM_EVENT_MAP_SIZE;

} // namespace LCS
