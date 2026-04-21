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
#include "LcsUtilLib.h"

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
// Data that needs to be kept permanently is flushed from memory to NVM so 
// that it is the initial value on the next restart. All data is stored in 
// controller native endianness. Only the messages exchanged via the LcsMsgBus 
// are transmitted in big endian order.
//
// The NVM layout is a fixed one. The layout starts with 16 bytes board data. 
// This area is right now primarily used for the magic word. Following this 
// data area is the nodeMAp, the central structure for a node. It contains the 
// nodeId, the node state and so on. Following the nodeMap is the node data 
// area, which contains the port attributes. Each port has a set of 128 16-bit
// words. Following this area is the event map, which contains the eventId to
// portId mapping. The last area is the optional extended attribute map, which
// can be used for any user defined data. It is an array of attributes global
// to the node.
// 
//
//          :-----------------------------------:   NVM_MAP_STORAGE_START 
//          :                                   :
//          :   NVM Header                      :
//          :                                   :
//          :-----------------------------------:   NVM_NODE_MAP_START
//          :                                   :
//          :   Node Map                        :
//          :                                   :
//          :-----------------------------------:   + sizeof( NodeMap )
//          :                                   :
//          :   Mode Map Data                   :
//          :                                   :
//          :-----------------------------------:   + sizeof( NodeMapData )
//          :                                   :
//          :   Event Map                       :
//          :                                   :
//          :-----------------------------------:   + sizeof( EventMap )
//          :                                   :
//          :                                   :
//          :                                   :
//          :   Optional Ext Attribute Map      :
//          :                                   :
//          :                                   :
//          :                                   :
//          :-----------------------------------:   0xNNNN  
//
// All data areas have a fixed size which is the maximum size for the particular 
// map. For example, there is room for 8 ports, even if the firmware would only 
// use let's say 4. In general, each of the runtime map also could have been 
// designed in a way that they are dynamically configurable in size. Considering
// the size and price of NVM chips as well as the memory size of the supported 
// controller platforms, using fixed sizes for each map avoids configuration 
// complexity.
//
//----------------------------------------------------------------------------------------
const uint16_t  MAX_PORT_MAP_ENTRIES            = 8;
const uint16_t  MAX_CHANNEL_MAP_ENTRIES         = 8;
const uint16_t  MAX_EVENT_MAP_ENTRIES           = 768;
const uint16_t  MAX_ATTR_MAP_ENTRIES            = 128;
const uint16_t  MAX_TASK_MAP_ENTRIES            = 16;

//----------------------------------------------------------------------------------------
// Other common constants.
//
//----------------------------------------------------------------------------------------
const uint16_t  MAX_LCS_MSG_SIZE                = 8;
const uint16_t  MAX_COMMAND_LINE_SIZE           = 256;
const uint16_t  MAX_ITEMS_PER_LINE              = 8;
const uint16_t  EVENT_DELAY_TICK_MILLIS         = 32;
const uint8_t   I2C_ADDRESS_OFFSET              = 8;

//----------------------------------------------------------------------------------------
// The default sizes for the chips on the board. For the main boards, we will figure
// out the real size during startup. For the extension boards, the size is fixed 
// for now.
//
//----------------------------------------------------------------------------------------
const uint32_t  NVM_MAIN_BOARD_DEF_SIZE         = 16 * 1024;

//----------------------------------------------------------------------------------------
// The maps have as their first word a magic word, which is just a special 
// constant. We simply read in that word and check it for being a valid word 
// for the particular map. If valid, the area was configured before and we can
// do further checking. It would be  quite unlikely that a random NVM content 
// has this word at the right spot. 
//
//----------------------------------------------------------------------------------------
const uint32_t  NVM_MWORD_NODE_HEADER           = ( 0xa5a5 << 16 ) | 1L;
const uint32_t  NVM_MWORD_NODE_MAP              = ( 0xa5a5 << 16 ) | 2L;
const uint32_t  NVM_MWORD_NODE_DATA_MAP         = ( 0xa5a5 << 16 ) | 3L;
const uint32_t  NVM_MWORD_NODE_EXT_DATA_MAP     = ( 0xa5a5 << 16 ) | 4L;
const uint32_t  NVM_MWORD_NODE_EVENT_MAP        = ( 0xa5a5 << 16 ) | 5L;

//----------------------------------------------------------------------------------------
// The node states. Essentially, the node runtime is a big state machine. The 
// node starts in the INIT state and once all is initialized and registered ends
// up in the OPS or CFG mode.
//
//  NS_NIL          -   NIL.
//  NS_FAIL         -   The startup failed.
//  NS_PFAIL        -   The startup detected a power failure restart.
//  NS_INIT         -   The node entered the startup state.
//  NS_REGISTER     -   The node entered the node register state.
//  NS_COLLISION    -   The node detected a nodeId collision and stopped.
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
// Node, port and channel attributes and functions are accessed with three main 
// routines, GET, SET and REQ. The specific items are defined in the external 
// include file. This part here defined the boundaries for internal checking.
// The first 64 items are predefined attributes and reserved for the runtime 
// itself. Item 64 to 127 are reserved for the channel attributes and functions.
// Item 128 to 255 are the user definable port attributes.
//
// Node level global attributes occupy the number range 256 to the maximum what 
// the board NVM chip can offer. Since we support a maximum of 64Kbyte chips, 
// the maximum node level global attributes limit is 32768 - 256.
//
//----------------------------------------------------------------------------------------
enum ItemRanges : uint16_t {

    IR_NIL                      = 0,

    IR_LIB_MAP_RANGE_START      = 1,
    IR_LIB_MAP_RANGE_END        = 63,

    IR_DRV_CHAN_START           = 64,
    IR_DRV_CHAN_END             = 127,

    IR_ATTR_RANGE_START         = 128,
    IR_ATTR_RANGE_END           = 255,
 
    IR_GLOBAL_ATTR_START        = 256,
    IR_GLOBAL_ATTR_END          = 32768
};

//----------------------------------------------------------------------------------------
// "MsgPriority" defines the values for the message priority. It tracks the 
// general definition found in the sendMsg routines of the LCS library. For the
// CAN bus, the priority is encoded in the CAN address field. A CAN Id consists 
// of the CAN Id number and the priority. Messages start out with a hard coded 
// priority and on message timeout are raised in their priority. This done 
// transparently to the firmware programmer.
//
//----------------------------------------------------------------------------------------
enum MsgPriority : uint8_t {

    MSG_PRI_VERY_HIGH   = 0,
    MSG_PRI_HIGH        = 1,
    MSG_PRI_NORMAL      = 2,
    MSG_PRI_LOW         = 3
};

//----------------------------------------------------------------------------------------
// "LcsMsgBusCAN" is the CAN bus interface. The two key routines are the send 
// and receive routines. A CAN bus message will use the nodeId as the canBus Id.
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

    uint8_t localNodeId;
};

//----------------------------------------------------------------------------------------
// The LcsNvmHeader structure is the first area in the NVM storage. It contains
// the magic word and some space reserved for future use. The magic word is used
// to check that the NVM storage was initialized before and that the data is 
// valid.
// 
//----------------------------------------------------------------------------------------
struct LcsNvmHeader {

    uint32_t magicWord;   
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
};

//----------------------------------------------------------------------------------------
// The nodeMap is the heart of all data on the node. When bringing up a node, we 
// first read in the NVM header to check the validity of the NVM data and then
// the node map. Several validity checks for the nodeMap data are performed. T
// he most important check is to compare the size of the various data structures
// with the runtime data size that we know from the compilation. That is the 
// reason that each NVM area also starts with a magic word. Only when they match
// can we somewhat trust the rest of the data maps.
//
//----------------------------------------------------------------------------------------
struct LcsNodeMap {

    uint32_t            magicWord;              // magic word of the node map
    uint32_t            nvmOfs;                 // byte offset in NVM
    uint32_t            nvmSize;                // size of the node map in bytes   

    uint16_t            nodeOptions;            // node options
    uint16_t            nodeFlags;              // node flags           
    uint16_t            nodeLastErr;            // laster code on the node
    uint16_t            nodeType;               // node type, e.g. main board
       
    uint16_t            boardType;              // board type, e.g. LCS_MAIN_BOARD  
    uint16_t            boardVersion;           // board  version, e.g. 1.0
    uint64_t            serialNum;              // board serial number

    uint16_t            nodeState;              // node state, e.g. NS_INIT;
    uint16_t            nodeId;                 // node Id
    uint32_t            nodeUID;                // node unique ID        
    uint16_t            nodeRestartCnt;         // restart counter  
    uint32_t            nodeSystemTime;         // system time in milliseconds
   
    LcsInitCallback     initCallback;           // init callback
    void                *initCallBackUdata;     // init callback user data

    LcsPfailCallback    pfailCallback;          // power failure callback
    void                *pfailCallBackUdata;    // power failure callback user data

    LcsMsgCallback      lcsMsgCallback;         // LCS message callback
    void                *lcsMsgCallBackUdata;   // LCS message callback user data

    LcsMsgCallback      dccMsgCallback;         // DCC message callback
    void                *dccMsgCallBackUdata;   // DCC message callback user data

    LcsCmdCallback      cmdLineCallback;        // command line callback
    void                *cmdLineCallBackUdata;  // command line callback user data

    LcsRepCallback      repCallback;            // LCS msg reply callback
    void                *repCallBackUdata;      // LCS msg reply callback user data
};

//----------------------------------------------------------------------------------------
// Node and ports each have an area of attributes that are in memory as well as
// in the node NVM. Typical usage examples are configuration items such as a 
// limit value. Upon power up the header structure is validated and the node 
// data from the NVM area is copied to the MEM counterpart. 
//
//----------------------------------------------------------------------------------------
struct LcsNodeData {

    uint32_t    magicWord;
    uint32_t    nvmOfs;
    uint32_t    nvmSize;    
    uint16_t    map[ MAX_PORT_MAP_ENTRIES ][ MAX_ATTR_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// A node offers an area of extended or global attributes. The sizeof this area
// is the available space in the NVM minus the storage requirements of the 
// runtime data. Upon power up the header structure is validated just like all
// the other NVM areas. In contrast to node and port attributes, the extended 
// attributes are directly accessed form the NVM. The idea us that these 
// attributes are not modified very often and thus we can do with the slower 
// access time of the NVM.
//
//----------------------------------------------------------------------------------------
struct LcsNodeExtData {

    uint32_t    magicWord;
    uint32_t    nvmOfs;
    uint32_t    nvmSize;      
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

    uint16_t eventId;
    uint16_t eventMask;
};

struct LcsEventMap {

    uint32_t            magicWord;
    uint32_t            nvmOfs;
    uint32_t            nvmSize;
    uint32_t            mapHwm;
    LcsEventMapEntry    map[ MAX_EVENT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// Events map entries are loaded into a memory map for faster access. The table
// is a hash table.
//
//----------------------------------------------------------------------------------------
const int               MAX_CAB_HASH_TAB_ENTRIES = 2048;

struct LcsEventHashMap {

    uint32_t           numEntries;
    LcsEventMapEntry   map[ MAX_CAB_HASH_TAB_ENTRIES ];
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
// A port supports up to 8 channels. Each port/channel combo corresponds to an 
// I2C address and is our way to access hardware, such as an extension board or
// a satellite board. All channels on a port must have the same channel type.
//
// The portMap entry furthermore contains the fields that deal with the actual 
// event received that the port is interested in. There are fields for the 
// sending node, the event itself and its action. An event can also be invoked 
// with a delay time.
//
// When a request is send, the target npId and a request time limit timestamp
// are stored in the port map entry. This way, when a reply comes in, we can 
// check if it is expected and invoke the reply callback. On a port, one request
// can be pending at a time. If another request is sent before the reply, it is
// an error.
//
//----------------------------------------------------------------------------------------
struct LcsPortMapEntry {

    uint16_t            portOptions;
    uint16_t            portFlags;
    uint16_t            portType;
    uint16_t            portLastErr;
   
    LcsReqCallback      reqCallback;;
    void                *reqCallBackUdata;

    LcsEventCallback    eventCallback;
    void                *eventCallBackUdata;

    uint16_t            eventNpId;
    uint16_t            eventId;
    uint16_t            eventValue;
    uint16_t            eventAction;
    uint16_t            eventDelayTime;
    uint32_t            eventTimeStamp;

    uint16_t            targetNpId;
    uint32_t            targetReqTs; 
    LcsRepCallback      targetRepCallback;
    void                *targetRepCallBackUdata;

    uint16_t            channelMap; // ??? needed ?
};

struct LcsPortMap {

    uint32_t        mapHwm;
    LcsPortMapEntry map[ MAX_PORT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// The core library maintains an array of periodic tasks. The entry maintains 
// the task procedure label, the time it ran the last time, and the minimum 
// interval between invocations. Note that the timing is not very accurate, but
// it is guaranteed that a task will eventually run when the interval is reached.
//
//----------------------------------------------------------------------------------------
struct LcsPTaskMapEntry {

    LcsTaskCallback     task;
    void                *uData; 
    uint32_t            timeStamp;
    uint32_t            interval;
};

struct LcsTaskMap {

    uint32_t            mapHwm;
    LcsPTaskMapEntry    map[ MAX_TASK_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// The layout of the NVM storage is fixed. There are the header and the maps 
// allocated in a given order and size. We can easily define the relevant offsets
// and sizes as constants.
//
//----------------------------------------------------------------------------------------
const uint32_t  NVM_BOARD_DESC_SIZE         =   sizeof( LcsNvmHeader );
const uint32_t  NVM_NODE_MAP_SIZE           =   sizeof( LcsNodeMap );
const uint32_t  NVM_NODE_DATA_SIZE          =   sizeof( LcsNodeData );
const uint32_t  NVM_EVENT_MAP_SIZE          =   sizeof( LcsEventMap );

const uint32_t  NVM_RUNTIME_MAPS_SIZE       =   NVM_BOARD_DESC_SIZE + 
                                                NVM_NODE_MAP_SIZE   +
                                                NVM_NODE_DATA_SIZE  +
                                                NVM_EVENT_MAP_SIZE;

const uint32_t  NVM_MAP_STORAGE_START       =   0;

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
    
const uint32_t NVM_EXT_NODE_DATA_OFS        =   NVM_MAP_STORAGE_START +
                                                NVM_BOARD_DESC_SIZE   +
                                                NVM_NODE_MAP_SIZE     +
                                                NVM_NODE_DATA_SIZE    +
                                                NVM_EVENT_MAP_SIZE;

} // namespace LCS
