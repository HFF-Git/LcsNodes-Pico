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
#include <stdio.h>
#include <cstring>

#include "LcsRuntimeLib.h"
#include "../LcsCdcLib/LcsCdcLib.h"

// ??? this should go to CDC ?
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))


//------------------------------------------------------------------------------------------------------------
//
// ??? shold they rather be just variables, we always have debugging code included...
//------------------------------------------------------------------------------------------------------------
#define   DEBUG_CONFIG      1
#define   DEBUG_NVM         1
#define   DEBUG_CAN_BUS     1
#define   DEBUG_ATTRIBUTES  1
#define   DEBUG_EVENTS      1


//----------------------------------------------------------------------------------------------------------
// Common constants to define limits for names, attributes and so on. Our current approaxch is to build the
// various data structures with fixed sizes described by these constants.
//
//----------------------------------------------------------------------------------------------------------
const uint16_t  MAX_ATTR_MAP_ENTRIES          = 64;
const uint16_t  MAX_PORT_MAP_ENTRIES          = 15;
const uint16_t  MAX_EVENT_MAP_ENTRIES         = 1024;
const uint16_t  MAX_TASK_MAP_ENTRIES          = 16;

const uint16_t  MAX_NODE_NAME_SIZE            = 16;
const uint16_t  MAX_PORT_NAME_SIZE            = 16;
const uint16_t  MAX_COMMAND_LINE_SIZE         = 256;
const uint16_t  MAX_LCS_MSG_SIZE              = 8;

const uint16_t  MAX_EXT_BOARD_MAP_ENTRIES     = 8;

const uint16_t  MAX_PENDING_REQ_MAP_ENTRIES   = 8;

const uint16_t  EVENT_DELAY_TICK_MILLIS       = 32;


//----------------------------------------------------------------------------------------------------------
// The NVM layout is a fixed one. We have the nodeMap starting at offset zero, the portMap starting at 
// offset 0x100 and the eventMap at offset 0x1000. The system area is 8 Kbytes. The optional user map is
// all the remaining bytes in the NVM and starts at 0x2000 then. All access routines are by default accessing
// the user area. A user can also access the system are but needs to set the access parameter explicitly in
// the calling routine. Note that dangerous things can be done when modifying the systen area.
//
//----------------------------------------------------------------------------------------------------------
const uint16_t NVM_NODE_MAP_START             = 0;
const uint16_t NVM_PORT_MAP_START             = 0x100;
const uint16_t NVM_EVENT_MAP_START            = 0x1000;
const uint16_t NVM_USER_MAP_START             = 0x2000;


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

//------------------------------------------------------------------------------------------------------------
// "LcsBus" is the CAN bus interface. The two key routines are the send and receive routines.
//
//------------------------------------------------------------------------------------------------------------
struct LcsMsgBusCAN {

  public:

    uint8_t   init( uint16_t canId, uint8_t pinRx, uint8_t pinTx, uint8_t fMode = CAN_BUS_LIB_PICO_PIO_500K );

    uint8_t   sendLcsMsg ( uint8_t *msgBuf, uint8_t msgPri = MSG_PRI_NORMAL );
    uint8_t   receiveLcsMsg( uint8_t *msg );

  private:

    uint16_t  canId = 0;
};


//------------------------------------------------------------------------------------------------------------
// The LCS Runtime needs to maintain a couple of internal data structures. As a general concept, there are
// two data areas which are stored in the node NVM and shadowed by a memory copy. Upon reset or power up
// the memory areas are copied form their NVM counter parts. Data that needs to be changed permanently is
// flushed from memory to NVM so that it is the initial value on the next restart. All data is stored in
// controller native endianess. Only the messages exchanged via the LcsMsgBus are transmitted in big endian
// order.
//
// The runtime area is a fixed size structure with a few sub areas. The first structure in any NVM is a
// board descriptor, which gives basic information on the board type and the NVM type. This information is
// key to correctly ddress the NVM. Before reading this area, we just use a generic attempt to read this
// subarea.
//
// The LCS runtime will check at startup time that the firmwware's board type expectation matches what
// is sored in the NVM on the board. Naturally, the NVM needs to get that data once when the board initially
// configured with this infomration.
//
// The next sub area is the HW descriptor area. We maintain a set of fixed identifiers for HW resources.
// For example, there are GPIO pins D0 to D15 or ADC pins ADC0 to ADC4. These resource IDs will be set
// with real HW pins and some flags according to the schematics of the Board.
//
// Further sub areas are the node map, the port map and the event map round up the runtime maps. The Node Map
// contains node global data and attributes. he port map contains a  set of port map entries, each describing
// a port. A port has a type, a name and like the node a set of attributes. It contains the event to port
// mapping for incoming events.
//
// The entire runtime area is shadowed by a memory area of the same type. At system startup or reset the
// runtime NVM area is copied to the memory area. During operation, some runtime values may change. They
// can only update memory or both memory and NVM.
//
// In genereal each of the runtime areas could be designed in a way that they are dynmically configurable
// in size. For example, a port map could be up to 15 ports but also less. The current implementation does
// however use fixed sizes. Why ? It turns out that the memory requirements are well within the capabilties
// of the NVM chips and also the PICO memory size.
//
// The remainder of the NVM is user defined. It is just the rest of the NVM storage. A firmware can only
// access the User NVM area through dedicated runtime library routines. Likewise, a firmware programmer can
// only access the runtime area through the nodeXXX runtime access access routines.
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// The board descriptor sub area. Every LCS hardware board has a unique controller, type and version. The
// data can be found in the first locations of the NVM chip on the board. The same structure is also used
// for an extension board NVM, although not all fields are used. The area is bracketed by two "magic" words,
// which are just constants to check that indeed the area is a descriptor area. The constants are choosen so
// that it is quite unlikely to find two such words in a new NVM.
//
// The board controller contains 4 NVM slots, which means we can have up to four NVM chips. The current chip
// family is the AA24XXX family if I2C NVM chips, which a capacity of 4K up to 64K. The chips sizes can be
// mixed. When the controller starts up, it will just attemot to read this structure form the first NVM
// locations. The size of the chip does not matter at startup, we only read this structure. When the board
// descritor has reasonabe valus, the rest can be read safely.
//
// The total size if this structure is 64bytes. We will pad it to this size.
//
//------------------------------------------------------------------------------------------------------------


struct LcsBoardDesc {

  uint16_t magicWord1;

  uint16_t ControllerFamily;
  uint16_t BoardType;
  uint16_t BoardVersion;

  uint16_t nvmChipFamily;
  uint16_t nvmChipI2CAdrRoot;
  uint16_t nvmMemSize0;
  uint16_t nvmMemSize1;
  uint16_t nvmMemSize2;
  uint16_t nvmMemSize3;
  uint32_t totalNvmSize;

  uint16_t reserved[ 19 ]; // for new items, it is under construction ...


};


//----------------------------------------------------------------------------------------------------------
// Every LCS board uses the CDC layer to access the controller hardware. The CDC descriptor contains the
// pin configuration data. When used by the schematic, the configuration process will set this field with
// the hardware pin.
//
// ??? not clear what else should be in here....
// ??? currently, the CDC config data is set directly by the application. One day, we may store this data
// in this descriptor. So far, this is more of a place holder.
//----------------------------------------------------------------------------------------------------------
struct LcsCdcDesc {

  uint16_t          flags;
  CDC::CdcPinConfig cfg;

};


//----------------------------------------------------------------------------------------------------------
// The node map

// contains items such as the node UID, the node ID, and so on. There is also the node name
// and type. Finally the node map is the place to store the node attributes. The structure can directly be
// read from NVM into a memory structure of the same type.
//
//----------------------------------------------------------------------------------------------------------
const uint16_t MWORD_1 = 0x010b;
const uint16_t MWORD_2 = 0x0a02;

struct LcsNodeMap {

  uint16_t magicWord1;

  uint16_t  options                     = 0;
  uint16_t  flags                       = 0;
  uint32_t  uid                         = 0L;
  uint16_t  id                          = NIL_NODE_ID;
  uint16_t  type                        = NIL_NODE_TYPE;
  uint16_t  restartCnt                  = 0;
  uint16_t  nodeMajorVersion            = 0;
  uint16_t  nodeMinorVersion            = 0;

  uint16_t ControllerFamily;
  uint16_t BoardType;
  uint16_t BoardVersion;

  uint16_t nvmChipFamily;
  uint16_t nvmChipI2CAdrRoot;
  uint16_t nvmMemSize0;
  uint16_t nvmMemSize1;
  uint16_t nvmMemSize2;
  uint16_t nvmMemSize3;
  uint32_t totalNvmSize;


  char      name[ MAX_NODE_NAME_SIZE ]  = { 0 };
  uint16_t  map[ MAX_ATTR_MAP_ENTRIES ] = { 0 };

  uint16_t magicWord2;
};

//----------------------------------------------------------------------------------------------------------
// The port map contains an array of ports, each descriobed by a port map entry. Besides the port flags,
// name and type and there are the port attributes. The portMap entry also contains the fields that deal
// with the actual event received. There are fields for the sending node, the event and its action. An
// event can also be invoked with a delay time.
//
//----------------------------------------------------------------------------------------------------------
struct LcsPortMapEntry {

  uint16_t  flags                         = 0;
  uint16_t  type                          = 0;

  uint16_t  nodeId                        = NIL_NODE_ID;
  uint16_t  eventId                       = NIL_EVENT_ID;
  uint16_t  eventValue                    = 0;
  uint16_t  eventAction                   = PEA_EVENT_IDLE;
  uint16_t  eventDelayTime                = 0;
  uint32_t  eventTimeStamp                = 0L;

  char      name[ MAX_PORT_NAME_SIZE ]    = { 0 };
  uint16_t  map[ MAX_ATTR_MAP_ENTRIES ]   = { 0 };
};

//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
struct LcsPortMap {

  uint16_t        flags;
  uint16_t        size;
  LcsPortMapEntry map[ MAX_PORT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------------------------
// The event map entry contains the mapping from eventId to portId. Every port interested in a certain event
// will have an entry in the event map. It is a sorted table of event and port pairs. This table is searched
// for an incoming event to find the ports that are interested in the event.
//
//----------------------------------------------------------------------------------------------------------
struct LcsEventMapEntry {

  uint16_t eventId  = NIL_EVENT_ID;
  uint16_t portId   = NIL_PORT_ID;
};

struct LcsEventMap {

  uint16_t            flags;
  uint16_t            size;
  uint16_t            hwm;
  LcsEventMapEntry    map[ MAX_EVENT_MAP_ENTRIES ];
};

//----------------------------------------------------------------------------------------------------------
// The user map describes the remaining area of the NVM. Most importantly there are the offset and size
// of this area. The usage and meaning is entirely up to the firmware programmer. The LCS runtime will
// however offer routines to access this area as the only way to get to it.
//
// ??? what else to store here ?
//----------------------------------------------------------------------------------------------------------
struct LcsUserMap {

  uint16_t  flags;
  uint16_t  size;
  uint32_t  ofs;
};

//----------------------------------------------------------------------------------------------------------
// The LCS runtime structure contains the runtime data found in the NVM. All values and fields are aligned
// to be on a 16-bit or 32-bit basis to avoid any alignment issues. The data is stored in the endianess of
// the controller used. This is not an issues as the data is created on the same controller. Only LCS
// messages insist on the big endian format regardless of the controller endianess. The runtime map is
// read in two parts. The first just gets the board section and validates that the runtime map makes sense.
// Next, all other parts can be read in one swoop. Easy and simple.
//
//----------------------------------------------------------------------------------------------------------
struct LcsRuntimeMap {

  LcsBoardDesc      board;
  LcsCdcDesc        cdcMap;
  LcsNodeMap        nodeMap;
  LcsPortMap        portMap;
  LcsEventMap       eventMap;
  LcsUserMap        userMap;
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

  void reset( ) {

    initCallback = nullptr;
    infoItemCallback  = nullptr;
    ctrlItemCallback  = nullptr;
  }
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

  void reset( ) {

    flags = 0;
    size  = MAX_PORT_MAP_ENTRIES + 1;

    lcsMsgCallback                    = nullptr;
    dccMsgCallback                    = nullptr;
    cmdLineCallback                   = nullptr;
    portEventCallback                 = nullptr;
    itemReqRepCallback                = nullptr;

    for ( int i = 0; i <= MAX_PORT_MAP_ENTRIES; i++ ) map[ i ].reset( );
  }
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

  void reset( ) {

    flags = 0;
    size  = MAX_TASK_MAP_ENTRIES;

    for ( int i = 0; i < size - 1; i++ ) map[ i ].reset( );
  }

};

//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
struct LcsPendingReqMap {

  uint16_t flags;

  uint16_t map[ MAX_PENDING_REQ_MAP_ENTRIES ];

};

const uint8_t ERR_INVALID_BOARD_ID = 255;

const uint8_t MIN_BOARD_ID = 1;
const uint8_t MAX_BOARD_ID = 4;


//------------------------------------------------------------------------------------------------------------
// Each extension board will have a NVM to store the board configuration data.
//
//
//
// ??? this may be a bit tricky to read and write. We need to decide how to actually store the data on the
// NVM. Is it little endian ? On the node NVM, this is not an issue, as this structure is rebuilt on the
// particular node PCB board. But the extension could be connected to different controller families....
//------------------------------------------------------------------------------------------------------------
struct LcsDrvBoardDesc {

  uint16_t  magicWord;
  uint16_t  boardType;
  uint32_t  boardUID;

  uint16_t  descHeaderSize;

  uint16_t  numOfChips;
  uint16_t  chipTabEntrySize;

  uint16_t  numOfEndPoints;
  uint16_t  endPointEntrySize;

  char      extBoardName[ 16 ];
};



//----------------------------------------------------------------------------------------------------------
// The core libary maintains a driver table. A driver is a library that manages a particular extension board.
// During startup all extension boards will be located, if any. For each extension board the correct driver
// will be stored in the driver map. There area at most four extension boards on a controller type board.
//
// ??? actually, the entry is just a refernce to the DRV object ... 
// ??? This entry struct should be the base class of all drivers...
//
// REWORK !!!!!!
//
//----------------------------------------------------------------------------------------------------------
struct LcsDrvEntry {

  public:

    virtual uint8_t init( uint16_t flags ) = 0;
    virtual uint8_t control( uint8_t padId, uint8_t item, uint16_t arg1, uint16_t arg2 = 0 ) = 0;
    virtual uint8_t info( uint8_t padId, uint8_t item, uint16_t *arg1, uint16_t *arg2 = nullptr ) = 0;
    virtual uint8_t read( uint8_t padId, uint16_t *arg ) = 0;
    virtual uint8_t write( uint8_t padId, uint16_t arg ) = 0;
    virtual uint8_t read( uint8_t padId, uint8_t *buf, uint8_t *bufLen ) = 0;
    virtual uint8_t write( uint8_t padId, uint8_t *buf, uint8_t bufLen ) = 0;

  private:

    LcsDrvBoardDesc *extBoard = nullptr;
};


//----------------------------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------------------------
struct LcsDrvMap {

  uint16_t          flags;
  uint16_t          size;
  uint16_t          *hwm = nullptr;

  LcsDrvEntry       *map[ 4 ];  // fix ....

  void reset( ) { }

};


//----------------------------------------------------------------------------------------------------------
// The LCS runtime internal routines used by other files of the runtime library.
//
//
//----------------------------------------------------------------------------------------------------------
uint8_t       resetNode( );
uint8_t       resetPort( uint8_t portId );

uint8_t       syncEventMap( );
uint8_t       addEvent( uint16_t eventId, uint16_t portId = NIL_PORT_ID );
uint8_t       removeEvent( uint16_t eventId, uint16_t portId = NIL_PORT_ID );
int           searchEvent( uint16_t eventId, uint16_t portId = NIL_PORT_ID );
uint8_t       getMemEmapEntry( uint16_t index, uint16_t *evId, uint16_t *pId );

void          handleMsgLcsMgt( uint8_t *msg );
void          handleMsgEvent( uint8_t *msg );

void          handleNodeSerialCommand( );

#endif
