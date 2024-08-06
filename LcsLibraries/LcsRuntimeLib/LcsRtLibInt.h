//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library inernals include file
//
//------------------------------------------------------------------------------------------------------------
//
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
#ifndef LCS_RT_LIB_INT_h
#define LCS_RT_LIB_INT_h

//------------------------------------------------------------------------------------------------------------
// Include files. Besides the standard C libraries, there is the external LCS runtime include file, tand he 
// dependent code library include file.
//
//------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsDrvOccDetectLib.h"


// ??? add other driver libs...


namespace LCS {

// ??? this should go to CDC ?
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))


//------------------------------------------------------------------------------------------------------------
//
// ??? should they rather be just variables, we always have debugging code included...
//------------------------------------------------------------------------------------------------------------
#define   DEBUG_CONFIG      1
#define   DEBUG_NVM         1
#define   DEBUG_CAN_BUS     1
#define   DEBUG_ATTRIBUTES  1
#define   DEBUG_EVENTS      1

// ??? this should actually be a set of variables.
// ??? each module should have a function to set the module ( file ) local debug level ...



//------------------------------------------------------------------------------------------------------------
// The LCS Runtime needs to maintain a couple of internal data structures. As a general concept, most of the
// data areas are stored in the NVM and shadowed by a memory copy. Upon reset or power up the memory areas
// initialized from their NVM counter parts. Data that needs to be changed permanently is flushed from memory
// to NVM so that it is the initial value on the next restart. All data is stored in controller native 
// endianess. Only the messages exchanged via the LcsMsgBus are transmitted in big endian order.
//
// The NVM layout is a fixed one. We have the nodeMap starting at offset zero, the portMap starting at 
// offset 0x100 and the eventMap at offset 0x1000. The system area is 8 Kbytes. The optional user map is
// all the remaining bytes in the NVM and starts at 0x2000 then. All access routines are by default accessing
// the user area. A user can also access the system are but needs to set the access parameter explicitly in
// the calling routine. Note that dangerous things can be done when modifying the systen area.
//
// In general each of the runtime areas could have also been designed in a way that they are dynmically 
// configurable in size. For example, a port map could be up to 15 ports but also less. The current 
// implementation does however use fixed sizes. Why ? It turns out that the memory requirements are well
//  within the capabilities of the NVM chips and also the PICO memory size. It is not worth the complexity.
//
//----------------------------------------------------------------------------------------------------------
const uint16_t  MAX_ATTR_MAP_ENTRIES          = 64;
const uint16_t  MAX_PORT_MAP_ENTRIES          = 15;
const uint16_t  MAX_EVENT_MAP_ENTRIES         = 1022;
const uint16_t  MAX_TASK_MAP_ENTRIES          = 16;

const uint16_t  MAX_NODE_NAME_SIZE            = 16;
const uint16_t  MAX_PORT_NAME_SIZE            = 16;
const uint16_t  MAX_COMMAND_LINE_SIZE         = 256;
const uint16_t  MAX_LCS_MSG_SIZE              = 8;

const uint16_t  MAX_EXT_BOARD_MAP_ENTRIES     = 4;
const uint16_t  MAX_PENDING_REQ_MAP_ENTRIES   = 8;
const uint16_t  EVENT_DELAY_TICK_MILLIS       = 32;

const uint16_t  NVM_NODE_MAP_START            = 0;
const uint16_t  NVM_PORT_MAP_START            = 0x100;
const uint16_t  NVM_EVENT_MAP_START           = 0x1000;
const uint16_t  NVM_USER_MAP_START            = 0x2000;

const uint8_t   MAX_EXT_BOARDS                = 4;
const uint8_t   MAX_DRIVER_DATA_SIZE          = 64;

//----------------------------------------------------------------------------------------------------------
// The nodeMap on NVM has two locations with a "magic" word. We simply read in a nodeMap and check these
// locations for the magic words. If found, the area was configured before. It would be quite unlikely
// that a random NVM content has these two words at the right spot. In a similar way, we have two magic 
// words for the NVM in an extension board. Same ide, same logic.
//
//----------------------------------------------------------------------------------------------------------
const uint16_t NODE_MWORD_1 = ( 'L' << 8 ) + 'C';
const uint16_t NODE_MWORD_2 = ( 'S' << 8 ) + 'R';

const uint16_t EXT_MWORD_1  = ( 'L' << 8 ) + 'C';
const uint16_t EXT_MWORD_2  = ( 'S' << 8 ) + 'E';

//------------------------------------------------------------------------------------------------------------
// The CAN bus mode. The PICO_PIO_xxx modes use the Raspberry Pi Pico "can2040" library, which is a software
// implementation of the CAN bus. The software version could run on the same or on the seprate processor core.
// Technically, the PICO could also run the MCP2515 via the SPI interface, but so far we just use the software
// version and avoid the additonal controller hardware.
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

//----------------------------------------------------------------------------------------------------------
  // The node states. The node starts in the INIT state and once all is initialized and registered ends up in
  // the OPS or CFG mode.
  //
  //  NS_NIL            -
  //  NS_FAIL           -
  //  NS_INIT           -
  //  NS_REGISTER       -
  //  NS_COLLISION      -
  //  NS_HALTED         -
  //  NS_CONFIG         -
  //  NS_OPERATE        -
  //
  //----------------------------------------------------------------------------------------------------------
  enum LcsNodeState : uint16_t {

    NS_NIL            = 0,
    NS_FAIL           = 1,
    NS_INIT           = 2,
    NS_REGISTER       = 3,
    NS_COLLISION      = 4,
    NS_HALTED         = 5,
    NS_CONFIG         = 6,
    NS_OPERATE        = 7
  };


//------------------------------------------------------------------------------------------------------------
// "LcsMsgBusCAN" is the CAN bus interface. The two key routines are the send and receive routines.
//
//------------------------------------------------------------------------------------------------------------
struct LcsMsgBusCAN {

  public:

    uint8_t   init( uint16_t canId, uint8_t pinRx, uint8_t pinTx, uint8_t fMode = CAN_BUS_LIB_PICO_PIO_125K );

    uint8_t   sendLcsMsg ( uint8_t *msgBuf, uint8_t msgPri = MSG_PRI_NORMAL );
    uint8_t   receiveLcsMsg( uint8_t *msg );

  private:

    uint16_t  canId = 0;
};

//----------------------------------------------------------------------------------------------------------
// Every LCS board uses the CDC layer to access the controller hardware. The CDC descriptor contains the
// pin configuration data. 
//
// ??? currently, the CDC config data is set directly by the application. One day, we may store this data
// in the descriptor. So far, this is more of a place holder.
//----------------------------------------------------------------------------------------------------------
struct LcsCdcDesc {

  uint16_t          flags;
  CDC::CdcPinConfig cfg;
};


//----------------------------------------------------------------------------------------------------------
// The port map contains an array of ports, each described by a port map entry. Besides the port flags,
// name and type there are the port attributes. The portMap entry also contains the fields that deal with
// the actual event received. There are fields for the sending node, the event and its action. An event 
// can also be invoked with a delay time. The are fifteen entries in the port map. The portMap starts
// fixed at NVM offset 0x1000.
//
//----------------------------------------------------------------------------------------------------------
struct LcsPortMapEntry {

  uint16_t  flags                           = 0;
  uint16_t  type                            = 0;

  uint16_t  nodeId                          = NIL_NODE_ID;
  uint16_t  eventId                         = NIL_EVENT_ID;
  uint16_t  eventValue                      = 0;
  uint16_t  eventAction                     = PEA_EVENT_IDLE;
  uint16_t  eventDelayTime                  = 0;
  uint32_t  eventTimeStamp                  = 0L;

  char      name[ MAX_PORT_NAME_SIZE  ]     = { 0 };
  uint16_t  map[ MAX_ATTR_MAP_ENTRIES ]     = { 0 };
};

struct LcsPortMap {

  uint16_t        flags                     = 0;
  uint16_t        options                   = 0;
  uint16_t        size                      = MAX_PORT_MAP_ENTRIES;
  uint16_t        reserved                  = 0;
  
  LcsPortMapEntry map[ MAX_PORT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------------------------
// The event map entry contains the mapping from eventId to portId. Every port interested in a certain event
// will have an entry in the event map. It is a sorted table of event and port pairs. This table is searched
// for an incoming event to find the ports that are interested in the event. The high water mark defines the
// actual number of entries used.
//
//----------------------------------------------------------------------------------------------------------
struct LcsEventMapEntry {

  uint16_t eventId                        = NIL_EVENT_ID;
  uint16_t portId                         = NIL_PORT_ID;
};

struct LcsEventMap {

  uint16_t            flags               = 0;
  uint16_t            options             = 0;
  uint16_t            size                = MAX_EVENT_MAP_ENTRIES;
  uint16_t            hwm                 = 0;

  LcsEventMapEntry    map[ MAX_EVENT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------------------------
// The LCS runtime communicates back to the firmware via callbacks. There are global callbacks for message
// receipt and events as well as callbacks for the node and ports that can be registered. Callbacks for
// node and ports are kept in an array of callback entry structures, called the callback map. Entry zero
// is the node itself all others are one for each port. The map thus has the number of ports plus the node
// itself.
//
//----------------------------------------------------------------------------------------------------------
struct LcsCallbackMapEntry {

  LcsInitCallback     initCallback      = nullptr;
  LcsInfoItemCallback infoItemCallback  = nullptr;
  LcsCtrlItemCallback ctrlItemCallback  = nullptr;
};

struct LcsCallbackMap {

  uint16_t                  flags;
  uint16_t                  size;

  LcsMsgCallback            lcsMsgCallback                    = nullptr;
  LcsMsgCallback            dccMsgCallback                    = nullptr;
  LcsCommandCallback        cmdLineCallback                   = nullptr;
  LcsPortEventCallback      portEventCallback                 = nullptr;
  LcsItemReqRepCallback     itemReqRepCallback                = nullptr;

  LcsCallbackMapEntry map[ MAX_PORT_MAP_ENTRIES + 1 ];
};

//----------------------------------------------------------------------------------------------------------
// The core library maintains an array of periodic task items. They will be run on a repeating bases. To
// balance the needs of other core library internal periodic tasks, such as checking for incoming messages,
// the periodic tasks are run one at a time, round robin, with the other internal tasks interleaving. The
// structure maintains the task procedure label, the time it ran the last time, and the interval between
// invocations. Note that the timing is not very accurate, but is guaranteed that a task will evetntually
// when the interval is reached.
//
//----------------------------------------------------------------------------------------------------------
struct LcsPTaskMapEntry {

  LcsTaskCallback   task          = nullptr;
  uint32_t          timeStamp     = 0;
  uint32_t          interval      = 0;

  void reset( ) {

    task      = nullptr;
    timeStamp = 0;
    interval  = 0;
  }
};

struct LcsTaskMap {

  uint16_t          flags;
  uint16_t          size;

  LcsPTaskMapEntry  *hwm    = nullptr;
  LcsPTaskMapEntry  *next   = nullptr;

  LcsPTaskMapEntry  map[ MAX_TASK_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------------------------
// The pending request map keeps track of outstanding requests to another node. We add an entry when our
// node sends a "REQ" type packet and clear the entry when the reply comes in.
//
//----------------------------------------------------------------------------------------------------------
struct LcsPendingReqEntry {

  uint16_t nodeId;
  uint16_t timeoutTimeStamp;
};

struct LcsPendingReqMap {

  uint16_t            flags;
  LcsPendingReqEntry  *hwm;

  LcsPendingReqEntry map[ MAX_PENDING_REQ_MAP_ENTRIES ];
};

extern "C" {

  typedef uint8_t ( *LcsDrvReqFunc ) ( uint8_t boardId, uint8_t item, uint16_t arg1, uint16_t *arg2 );  
} 

//------------------------------------------------------------------------------------------------------------
// Each extension board will have a NVM to store the board configuration data. Similar to the node map of the
// controller board, this extension board will have a data structure that is read at initialization time. The
// structure of this data is rather simple. We have the magic words bracketing the data, a board type and
// subtype, a name and an area which contains driver relevant information. The driver information will tell
// the driver what capabilities the board has. This data is entirely driver specific and the meaning is only
// know to the driver software. At startup time, all we have to do then is locate the board type, load the 
// respective driver and let the driver code do whatever needs to be done accorind to the data area content.
//
// Note that the extension board NVM data is "read only". It needs to be set when there is a vanilla board
// by a utility program or commands in the runtime lib user interface. After this setting, the data will 
// not change again. This does however not mean that that data once it is loaded cannot be changed during
// oerations. For example, the driver area is also the "working area" for the driver to keep temporary
// values. At node restart, all data is set back to the NVM data configured on the extension bard chip.
//
//------------------------------------------------------------------------------------------------------------
struct LcsDrvBoardDesc {

  uint16_t  magicWord1;

  uint16_t  options;
  uint16_t  flags;
  uint16_t  boardType;
  uint32_t  boardUID;

  uint16_t  driverData[ MAX_DRIVER_DATA_SIZE ];

  uint16_t  magicWord2;
};

//----------------------------------------------------------------------------------------------------------
// The runtime libary maintains a driver table. A driver is a library that manages a particular extension 
// board. During startup all extension boards will be located, if any. For each board the correct driver
// will be stored in the driver map. The driver obbject is a set of defined methods and a reference to the
// driver data area. This is the area that was read in when we located the extension board.
//
//----------------------------------------------------------------------------------------------------------
struct LcsDrvEntry {

    uint16_t        flags     = 0;   
    LcsDrvReqFunc   drvFunc   = nullptr;
    LcsDrvBoardDesc *extBoard = nullptr;
};

//----------------------------------------------------------------------------------------------------------
// The LCS driver map is a memory structure created at setup time. When an extension board is connected to
// the controller board it is assigned by hardware an index number. The first board has an index of zero. 
// 
//
// ??? when you change the order of the boards, it does not matter to the drivers, but perhaps to the 
// firmware on top...
//----------------------------------------------------------------------------------------------------------
struct LcsDrvMap {

  uint16_t          flags;
  uint16_t          size;
  
  LcsDrvEntry       map[ MAX_EXT_BOARDS ];
};


//----------------------------------------------------------------------------------------------------------
// The node map. At the first locations of the NVM area on the controller board NVM chip is the nodeMap, 
// which is read in at controller  reset. It is the heart of all data on the node.
//
// Creating the nodeMap at controller startup is a two step process. First we read in the map, which 
// contains fields that are set either configured opr predefiuned values. Some fields are however bound
// to be overwritten during the startup process.
//
// ??? describe the fields...
//----------------------------------------------------------------------------------------------------------
struct LcsNodeMap {

  uint16_t  magicWord1                      = NODE_MWORD_1;

  uint16_t  options                         = 0;
  uint16_t  flags                           = 0;
  
  uint16_t  nodeId                          = NIL_NODE_ID;
  uint32_t  nodeUID                         = 0L;
  uint16_t  nodeType                        = NIL_NODE_TYPE;   

  uint16_t  boardType                       = BT_NIL;
  uint16_t  controllerFamily                = CF_FAM_RPICO_2040;
  uint16_t  nvmChipFamily                   = CF_FAM_MICROCHIP;
  
  uint16_t  nodeSwVersion                   = 0;
  uint16_t  nodeSwPatchLevel                = 0;
  uint16_t  restartCnt                      = 0;

  uint16_t  nodeMapNvmOfs                   = NVM_NODE_MAP_START;
  uint16_t  portMapNvmOfs                   = NVM_PORT_MAP_START;
  uint16_t  eventMapNvmOfs                  = NVM_EVENT_MAP_START;
  uint16_t  userMapNvmOfs                   = NVM_USER_MAP_START;
  uint16_t  nvmMemSizeInBlocks              = 0;


// ??? phase out ...
  uint16_t  nvmChipI2CAdrRoot               = 0x50;
  uint16_t  nvmMemSize0                     = 0;
  uint16_t  nvmMemSize1                     = 0;
  uint16_t  nvmMemSize2                     = 0;
  uint16_t  nvmMemSize3                     = 0;
  uint32_t  totalNvmSize                    = 0;

  
  char      name[ MAX_NODE_NAME_SIZE ]      = { 0 };
  uint16_t  map[ MAX_ATTR_MAP_ENTRIES ]     = { 0 };

  uint16_t  magicWord2                      = NODE_MWORD_2;
};



//----------------------------------------------------------------------------------------------------------
// The LCS runtime internal routines used by other files of the runtime library.
//
//
//----------------------------------------------------------------------------------------------------------

uint8_t       configNvm( CDC::CdcPinConfig *ci );

uint8_t       rtNvmPutWord( uint32_t ofs, uint16_t word );
uint8_t       rtNvmGetWord( uint32_t ofs, uint16_t *word );
uint8_t       rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
uint8_t       rtNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
uint32_t      rtNvmGetSize( );

uint8_t       extNvmPutWord( uint8_t boardId, uint32_t ofs, uint16_t word );
uint8_t       extNvmGetWord( uint8_t boardId, uint32_t ofs, uint16_t *word );
uint8_t       extNvmPutBytes( uint8_t boardId, uint32_t ofs, uint8_t *buf, uint32_t len );
uint8_t       extNvmGetBytes( uint8_t boardId, uint32_t ofs, uint8_t *buf, uint32_t len );
uint32_t      extNvmGetSize( );


uint8_t       resetNode( );
uint8_t       resetPort( uint8_t portId );

uint8_t       syncEventMap( );
uint8_t       addEvent( uint16_t eventId, uint16_t portId = NIL_PORT_ID );
uint8_t       removeEvent( uint16_t eventId, uint16_t portId = NIL_PORT_ID );
int           searchEvent( uint16_t eventId, uint16_t portId = NIL_PORT_ID );
uint8_t       getMemEmapEntry( uint16_t index, uint16_t *evId, uint16_t *pId );

void          handleMsgLcsMgt( uint8_t *msg );
void          handleMsgEvent( uint8_t *msg );

uint8_t       setupSerialCommand( );
uint8_t       handleSerialCommand( );

void          handleNodeStateInit( );
void          handleNodeStateFail( );
void          handleNodeStateRegister( );
void          handleNodeStateCollision( );
void          handleNodeStateHalted( );
void          handleNodeStateConfig( );
void          handleNodeStateOperations( );

void          handlePeriodicTasks( );
void          handleNodePortEvents( );


};

#endif
