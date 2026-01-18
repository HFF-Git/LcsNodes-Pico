//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Include file
//
//----------------------------------------------------------------------------------------
//
// ??? this is a first cut at the block controller software. It remains to be seen
// what we should factor out and use across base station and block controller.
//
//
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller
// Copyright (C) 2020 - 2026  Helmut Fieres
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
//----------------------------------------------------------------------------------------
#pragma once

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsDrvOccDetectLib.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
//
// A block is an abstract object with n WEST docking points and m EAST docking
// points. The block is managed by a block controller and represents a single
// power domain. Within a block there may be sections, turnouts, and signals.
// The simplest form is a block with one WEST and one EAST docking point and
// no turnouts.
//
//                        <--- direction
//                  :---------------------------:
//                  :         Block ID          :
//              WEST_1                      EAST_1
//                  :                           :
//   Neighbors      :                           :    Neighbors
//                  :                           :
//              WEST_n                      EAST_m
//                  :                           :
//                  :---------------------------:
//
// The block controller manages all associated resources, including sections,
// turnouts, and signals. The intent is that a layout can be controlled using
// high-level commands that the block interprets locally. For example, a route
// reservation causes the block to set its turnouts and signals accordingly.
// On automated routes, the block also controls engine speed within its domain.
//
// All blocks publish events describing their current state. Blocks also
// monitor neighboring blocks by subscribing to and processing their events.
//
// A block can operate in either automatic or manual mode. Automatic mode
// provides a straightforward block control scheme: routes are reserved and
// trains are dispatched along those routes. In manual mode, a set of blocks
// (for example, an entire route or a subset of blocks) is reserved for direct
// manual operation.
//
// A block does not cross-check whether the currently assigned CabId matches
// the actual engine on the track. While such verification is possible in
// digital mode using RailCom, analog engines cannot be uniquely identified.
// The block only assumes that an engine is present, initially associated
// with a CabId, and manages that engine until it leaves the block. When an
// engine enters the next block, the receiving block queries its predecessor
// for the CabId. This simple scheme works for both DCC and analog operation.
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// The block controller maintains a set of debug flags. The overall concept is 
// very similar to the LCS runtime library debug mask. Then following debug flags
// are defined:
//
//      DBG_BC_CONFIG                   -   DEBUG base station enabled
//      DBG_BC_SETUP                    -   show the setup steps
//      DBG_BC_NODE                     -   show the node related activity
//      DBG_BC_BLOCK                    -   show the block state changes
//      DBG_BC_TRACK                    -   show the track power measurement data
//      DBG_BC_OCCUPANCY                -   show the occupancy detection activity
//      DBG_BC_TURNOUTS                 -   show the turnout activity
//      DBG_BC_SIGNALS                  -   show the signal activity
//      DBG_BC_RAILCOM                  -   show the RailCom activity
//      DBG_BC_ALL                      -   all debug enabled
//
// The way to use these flags is for example:
//
//      if (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_SESSION )) 
//
//----------------------------------------------------------------------------------------
enum BlockControllerDebugFlags : uint16_t {

    DBG_BC_CONFIG           = 1 << 15,   
    DBG_BC_SETUP            = 1 << 1,    
    DBG_BC_NODE             = 1 << 2,    
    DBG_BC_BLOCK            = 1 << 3,    
    DBG_BC_TRACK            = 1 << 4,    
    DBG_BC_OCCUPANCY        = 1 << 5,    
    DBG_BC_TURNOUTS         = 1 << 6,    
    DBG_BC_SIGNALS          = 1 << 7,    
    DBG_BC_RAILCOM          = 1 << 8,
    DBG_BC_ALL              = 0xFFFF
};

//----------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user number
// range of errors defined in the LCS runtime library. 
//
//----------------------------------------------------------------------------------------
enum BlockControllerErrors : uint8_t {

    BLOCK_CONTROLLER_ERR_BASE       = 128,

    ERR_MSG_INTERFACE_SETUP         = BLOCK_CONTROLLER_ERR_BASE + 10,
    ERR_DCC_TRACK_CONFIG            = BLOCK_CONTROLLER_ERR_BASE + 11,
    ERR_RNUM_CONFIG                 = BLOCK_CONTROLLER_ERR_BASE + 12,
    ERR_TRACK_CONFIG                = BLOCK_CONTROLLER_ERR_BASE + 13,

    ERR_NVM_HW_SETUP                = BLOCK_CONTROLLER_ERR_BASE + 15,
    ERR_PIO_HW_SETUP                = BLOCK_CONTROLLER_ERR_BASE + 16
};

//----------------------------------------------------------------------------------------
// Setup options to set for the DCC track. They are set when the track object is 
// created.
//
//  DT_OPT_SERVICE_MODE_TRACK  - The track is a PROG track.
//  DT_OPT_RAILCOM             - The track support Railcom detection.
//
//----------------------------------------------------------------------------------------
enum BlockControllerTrackOptions : uint16_t {

    BT_OPT_DEFAULT_SETTING      = 1 << 0,
    BT_OPT_RAILCOM              = 1 << 1
};

//----------------------------------------------------------------------------------------
// Nodes and blocks is accessed with the LCS routines GET,SET and REQ. These 
// routines expect an item number as one of their parameters. This enumeration
// will define all item numbers. See the comments what they are and how an item
// is used.
//
// ??? assign item numbers...
//----------------------------------------------------------------------------------------
enum BlockControllItems2 : uint8_t {

    //------------------------------------------------------------------------------------
    // Node items: These items apply to all blocks on the node.
    //
    // ??? we could map some of the LCS common items with a meaningful name 
    // here, just to be clearer... .e.g Node Name or so...
    //------------------------------------------------------------------------------------
    BCI_NUM_OF_BLOCKS                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_UPDATE_DATA_INTERVAL            = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Block Items. There is a ton of items. The two key items are the block state
    // which is an encoded attribute that give an overview for other nodes with one
    // GET call. The block command item is used for a REQ call to issues a command 
    // to the block. The command codes themselves are also items defined for a block.
    //
    //------------------------------------------------------------------------------------
    BCI_BLOCK_STATE                     = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_COMMAND                   = ITEM_ID_USER_START + 0,   // REQ

    BCI_INITIAL_BLOCK_STATE             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_INITIAL_ROUTE_STATE             = ITEM_ID_USER_START + 0,   // GET / SET 

    BCI_BLOCK_MAX_SPEED                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_ITEM_SPEED_SLOW                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_ITEM_SPEED_MIDDLE               = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_ITEM_SPEED_HIGH                 = ITEM_ID_USER_START + 0,   // GET / SET
    
    //------------------------------------------------------------------------------------
    // Block Id and our neighbors. A block ID is the LCS node and the port Id. The
    // port numbers used are the ports 5 to 8. Port 0 is the node itself, and there
    // can be up to 4 extension boards to address. Hence Port 5 to 8 is where the
    // blocks are assigned to.
    // 
    // The neighbors are organized in a west and an east group. For a simple block
    // with no turnouts, WEST_1 and EAST_1 are used. Add a turnout to the west side
    // WEST_1 and WEST_2 are on the west side, EAST_1 on the east side.
    //
    // All turnouts are part of the block power domain. The neighbor blocks imply
    // that there is at least one entry. The most common setting is perhaps one
    // entry and one exit.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_BLOCK_ID                        = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_1                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_2                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_3                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_4                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_5                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_6                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_7                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_WEST_8                 = ITEM_ID_USER_START + 0,   // GET / SET

    BCI_BLOCK_ID_EAST_1                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_2                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_3                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_4                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_5                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_6                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_7                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ID_EAST_8                 = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Block length and section length measure in Centimeters. We can handle up 
    // to 8 sections for a block.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_BLOCK_LEN                       = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_1                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_2                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_3                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_4                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_5                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_6                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_7                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_LEN_8                   = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Other static block attributes.
    //
    // Configuration data.
    //-----------------------------------------------------------------------------------
    BCI_BLOCK_SLOPE                     = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // The time stamps when we detect a section being entered or left. The 
    // direction does not matter, except when forward ( west ) block entry is 
    // east and exit is west, else vice versa. The timestamp is a 10ms resolution
    // 16-bit value. Overflows in about 10 minutes.
    //
    // Dynamic data.
    //------------------------------------------------------------------------------------
    BCI_BLOCK_ENTRY_TS                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_1              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_2              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_3              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_4              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_5              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_6              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_7              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SECTION_ENTRY_TS_8              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_EXIT_TS                   = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Signal Channels. Each signal is defined with its type, initial state and 
    // where to find it in the hardware. There is an extension board and a channel
    // number which uniquely defines the turnout.
    // 
    // A block can have NO signal, 1, 2 or up to eight. No matter how many 
    // signals are defined, they all belong to the same block power module. The
    // signal configuration does not specify where they are used in the track
    // plan.
    //
    // Configuration attribute ( type, initial state, address )
    //
    //  - type:             servo, nil          ( 4bits )
    //  - initial state:    tbd                 ( 4bits )
    //  - address:          ext-board, channel  (  2 + 6 bits )
    //
    // we need to fit this in a 16-bit word.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_SIGNAL_CHAN_1                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_2                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_3                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_4                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_5                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_6                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_7                   = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_SIGNAL_CHAN_8                   = ITEM_ID_USER_START + 0,   // GET / SET

    BCI_SIGNAL_CHAN_WEST                = BCI_SIGNAL_CHAN_1,        // GET / SET
    BCI_SIGNAL_CHAN_EAST                = BCI_SIGNAL_CHAN_2,        // GET / SET

    //------------------------------------------------------------------------------------
    // Turnout Channels. Each turnout is defined with its type, initial state 
    // and where to find it in the hardware. There is an extension board and a 
    // channel number which uniquely defines the turnout.
    // 
    // A block can have NO turnout, 1, 2 or up to eight. No matter how many 
    // turnouts are defined, they all belong to the same block power module. The
    // turnout configuration does not specify where they are used in the track
    // plan and their relation to each other.
    //
    // Configuration attribute ( type, initial state, address )
    //
    //  - type:             servo, nil          ( 4bits )
    //  - initial state:    normal / thrown     ( 4bits )
    //  - address:          ext-board, channel  (  2 + 6 bits )
    //
    // we need to fit this in a 16-bit word.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_TURNOUT_CHAN_1                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_2                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_3                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_4                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_5                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_6                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_7                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_CHAN_8                  = ITEM_ID_USER_START + 0,   // GET / SET

    BCI_TURNOUT_CHAN_WEST               = BCI_TURNOUT_CHAN_1,       // GET / SET
    BCI_TURNOUT_CHAN_EAST               = BCI_TURNOUT_CHAN_2,       // GET / SET

    //------------------------------------------------------------------------------------
    // Turnout Route Mask. A turnout route is the setting of the turnouts for a 
    // route from input block to exit block. A route is defined 
    // 
    //  input:      the entry block ( 4 bit )
    //. output:     the exit block  ( 4 bit )
    //  mask:       the turnout settings. ( 1 bit per turnout: N/T )
    //
    // Note: there are more possible routes than we can realistically configure.
    // An 8 by 8 hyper block would need 64 slots to record all combinations. 
    // Let's define up to 16 internal routes and see if this is enough. After
    // all a large hyper block can be divided into several hyper blocks at the 
    // expense of being a separate block.
    //
    // Dynamic data.
    //------------------------------------------------------------------------------------
    BCI_BLOCK_ROUTE_MASK_1              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_2              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_3              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_4              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_5              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_6              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_7              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_8              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_9              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_10             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_11             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_12             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_13             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_14             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_15             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_ROUTE_MASK_16             = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Current state items.
    //
    // Dynamic data.
    //------------------------------------------------------------------------------------
    BCI_CURRENT_BLOCK_MODE              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_BLOCK_EAST              = ITEM_ID_USER_START + 0,   // GET
    BCI_CURRENT_BLOCK_WEST              = ITEM_ID_USER_START + 0,   // GET
    BCI_CURRENT_TARGET_SPEED            = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_ROUTE_ID                = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_ROUTE_MASK              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_CAB_ID                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_CAB_SPEED               = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_OCCUPANCY_MASK          = ITEM_ID_USER_START + 0,   // GET
    BCI_CURRENT_TURNOUT_STATE_MASK      = ITEM_ID_USER_START + 0,   // GET
    BCI_CURRENT_POWER_CONSUMPTION       = ITEM_ID_USER_START + 0,   // GET

    //------------------------------------------------------------------------------------
    // Event Ids to configure. These event Ids are used when the block controller
    // sends an event to the system.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_BLOCK_SECTION_ENTRY_1           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_2           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_3           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_4           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_5           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_6           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_7           = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_SECTION_ENTRY_8           = ITEM_ID_USER_START + 0,   // GET / SET

    BCI_BLOCK_EVENT_ENTRY               = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_BLOCK_EVENT_EXIT                = ITEM_ID_USER_START + 0,   // GET / SET

    BLI_BLOCK_EVENT_POWER               = ITEM_ID_USER_START + 0,   // GET / SET
   
    //------------------------------------------------------------------------------------
    // Block Power Module. The power module manages the hardware track.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_INIT_CURRENT_MA                 = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_LIMIT_CURRENT_MA                = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_MAX_CURRENT_MA                  = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_MILLI_VOLT_PER_AMP              = ITEM_ID_USER_START + 0,   // GET / SET

    BCI_START_TIME_THRESHOLD            = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_STOP_TIME_THRESHOLD             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_OVL_TIME_THRESHOLD              = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_OVL_EVENT_THRESHOLD             = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_OVL_RESTART_THRESHOLD           = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Turnout setting parameters. A turnout of type servo is controlled by two
    // positions, a delay when to apply and so on. These attributes are part of
    // of the extension board attributes where the turnouts are configured. 
    // Nevertheless, their item value is defined here. There is one set of 
    // attributes per turnout.
    //
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_TURNOUT_TYPE                    = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_SERVO_LEFT_POS          = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_SERVO_RIGHT_POS         = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_TURNOUT_SERVO_DELAY             = ITEM_ID_USER_START + 0,   // GET / SET

    //------------------------------------------------------------------------------------
    // Signal settings parameters. A signal is managed through a set of digital 
    // outputs. These attributes are part of the extension board attributes where 
    // the signals are configured. There is one set of attributes per turnout.
    // 
    // Configuration data.
    //------------------------------------------------------------------------------------
    BCI_SIGNAL_TYPE                     = ITEM_ID_USER_START + 0,   // GET / SET
    
    // ??? tbd... what do we need for a signal ?

    //------------------------------------------------------------------------------------
    // Block command codes. A block can be managed by a control system either via
    // high-level commands or explicitly via low-level commands that operate the
    // individual components of the block. The latter command set is used, for
    // example, by train control systems such as RocRail, which assume a central
    // command station and send their commands to that station.
    //
    // To support such systems, an LCS gateway node is part of the architecture.
    // It accepts these commands and translates them into LCS node messages for
    // the corresponding block controller.
    //
    // An alternative approach is layout management using higher-level commands.
    // In this scenario, the block is treated as a high-level entity that manages
    // turnouts, signals, and the currently assigned engine in coordination with
    // its neighboring blocks. The train master then issues abstract commands such
    // as "reserve route" or "send train".
    //
    // Managing hybrid systems (i.e., analog and digital engines on the same
    // layout) becomes considerably more complex when the control system explicitly
    // manages the layout. Since block controllers typically do not have access to
    // the control system’s configuration data, commands such as "engine n speed m"
    // cannot be decoded for analog engines.
    //
    // Operating at the high-level block abstraction is therefore much simpler.
    // With the necessary configuration data available, block controllers are able
    // to manage any type of train within their sections.
    // 
    // ??? is there a general command structure:
    //
    // REQ <item> <sub-item> <value> ?
    // example: REQ BCI_BLOCK_SET_INNER_ROUTE <2,5>
    // example: REQ BCI_TURNOUT_SERVO_LEFT_POS <1,100>
    //
    // ??? explain the commands more detailed...
    //
    // Command Items.
    //------------------------------------------------------------------------------------
    BCI_BLOCK_RESERVE_ROUTE             = ITEM_ID_USER_START + 0,   // REQ - high level 
    BCI_BLOCK_RELEASE_ROUTE             = ITEM_ID_USER_START + 0,   // REQ - high level    
    BCI_BLOCK_SET_TRACK_MODE            = ITEM_ID_USER_START + 0,   // REQ - high level   
    BCI_BLOCK_SET_INNER_ROUTE           = ITEM_ID_USER_START + 0,   // REQ - high level 
    BCI_BLOCK_SET_POWER                 = ITEM_ID_USER_START + 0,   // REQ - any level 
    BCI_BLOCK_SET_SPEED                 = ITEM_ID_USER_START + 0,   // REQ - any level 
    BCI_BLOCK_SET_TURNOUT               = ITEM_ID_USER_START + 0,   // REQ - any level 
    BCI_BLOCK_SET_SIGNAL                = ITEM_ID_USER_START + 0,   // REQ - any level 
    
};

//----------------------------------------------------------------------------------------
// The block track object has a set of flags to indicate its current status.
//
//  DT_F_POWER_ON             - The track is under power.
//  DT_F_POWER_OVERLOAD       - An overload situation was detected.
//  DT_F_MEASUREMENT_ON       - The power measurement is enabled.
//  DT_F_CONFIG_ERROR         - The configuration descriptor is invalid.
//
//----------------------------------------------------------------------------------------
enum TrackFlags : uint16_t {

    BT_F_DEFAULT_SETTING      = 0,
    BT_F_POWER_ON             = 1 << 0,
    BT_F_POWER_OVERLOAD       = 1 << 1,
    BT_F_MEASUREMENT_ON       = 1 << 2,
    BT_F_CONFIG_ERROR         = 1 << 15
};

//----------------------------------------------------------------------------------------
// The following constants are for the current consumption RMS measurement. The 
// idea is to record the measured ADC values in a circular buffer, every time a 
// certain amount of milliseconds has passed. This work is done by the DCC track 
// state machine as part of the power on state.
//
//----------------------------------------------------------------------------------------
const uint8_t   PWR_SAMPLE_BUF_SIZE               = 64;
const uint32_t  PWR_SAMPLE_TIME_INTERVAL_MILLIS   = 16;

//----------------------------------------------------------------------------------------
// The track state machine runs at a time interval.
//
//----------------------------------------------------------------------------------------
const uint32_t TRACK_STATE_TIME_INTERVAL  = 10;

//----------------------------------------------------------------------------------------
// A block track can be in four states.
//
//----------------------------------------------------------------------------------------
enum BlockTrackMode : uint16_t {

    BT_MODE_OFF        = 0,
    BT_MODE_PWM_FWD    = 1,
    BT_MODE_PWM_REV    = 2,
    BT_MODE_DCC        = 3
};

//----------------------------------------------------------------------------------------
// The block controller manages up to four blocks. Each block track is described
// by an LcsBlockTrackDesc descriptor, which specifies the associated hardware
// resource IDs. In addition, current consumption limits are defined, all
// expressed in milliamps.
//
// The initial current defines the current limit applied immediately after a
// track is powered on. The limit current specifies the normal operating value
// used to detect a track current overload condition. The maximum current
// defines an absolute limit that the power module must never exceed.
//
// For current measurement to work correctly, the power module must provide a
// voltage proportional to the current drawn by the track. This proportionality
// factor is specified in millivolts per ampere.
//
// Finally, threshold times are defined to control overload handling and track
// restart behavior.
//
//----------------------------------------------------------------------------------------
struct LcsBlockTrackDesc {

    uint16_t    options                         = BT_OPT_DEFAULT_SETTING;

    uint8_t     rNumControl                     = 0;
    uint8_t     rNumSense                       = 0;
    
    uint16_t    pwmFrequency                    = 70;
    uint16_t    initialTrackMode                = BT_MODE_OFF;
    uint16_t    initialTrackSpeed               = 0;

    uint16_t    initCurrentMilliAmp             = 0;
    uint16_t    limitCurrentMilliAmp            = 0;
    uint16_t    maxCurrentMilliAmp              = 0;
    uint16_t    milliVoltPerAmp                 = 0;

    uint16_t    startTimeThresholdMillis        = 0;
    uint16_t    stopTimeThresholdMillis         = 0;
    uint16_t    overloadTimeThresholdMillis     = 0;
    uint16_t    overloadEventThreshold          = 0;
    uint16_t    overloadRestartThreshold        = 0;
};

//----------------------------------------------------------------------------------------
// The "LcsBlockTrack" manages the track of a block. This primarily the power 
// management and control of the H-Bridge settings. There is one object per track
// block. At the heart of the object is a state machine that is executed very often
// for measuring the power consumption and overload detection logic. The tack can 
// operate in digital or analog mode. In digital mode, the DCC signal from the LCS
// bus is routed though to the H-Bridge, in analog mode a PWM signal is used to set
// the H-Bridge emitting a PWM signal with a positive or negative voltage.
//
//----------------------------------------------------------------------------------------
struct LcsBlockTrack {

    public:

    LcsBlockTrack( );

    void        getDefaultTrackDesc( LcsBlockTrackDesc *tDesc );
    void        setStartTimeThresholdMillis( LcsBlockTrackDesc *tDesc, uint16_t val );
    void        setStopTimeThresholdMillis( LcsBlockTrackDesc *tDesc, uint16_t val );
    void        setOverloadTimeThresholdMillis( LcsBlockTrackDesc *tDesc, uint16_t val );
    void        setOverloadEventThreshold( LcsBlockTrackDesc *tDesc, uint16_t val ); 
    void        setOverloadRestartThreshold(LcsBlockTrackDesc *tDesc, uint16_t val );    
    void        setInitCurrentMilliAmp( LcsBlockTrackDesc *tDesc, uint16_t val );                        
    void        setLimitCurrentMilliAmp( LcsBlockTrackDesc *tDesc, uint16_t val );
    void        setMaxCurrentMilliAmp( LcsBlockTrackDesc *tDesc, uint16_t val );
    void        setMilliVoltPerAmp( LcsBlockTrackDesc *tDesc, uint16_t val );
    uint8_t     setupBlockTrack( LcsBlockTrackDesc* trackDesc );

    uint16_t    getFlags( );
    uint16_t    getOptions( );
    uint16_t    getLimitCurrentMilliAmp( );
    uint16_t    getActualCurrentMilliAmp( );
    uint16_t    getInitCurrentMilliAmp( );
    uint16_t    getMaxCurrentMilliAmp( );
    uint16_t    getRMSCurrentMilliAmp( );
    uint32_t    getPowerSamplesTaken( );
    uint16_t    getPowerSamplesPerSec( );

    uint8_t     setTrackState( uint16_t state );
    uint8_t     setTrackMode( uint16_t mode, uint8_t speed = 0 );
    uint8_t     setPwmFrequency( uint32_t frequency );

    void        powerStart( );
    void        powerStop( );
    bool        isPowerOn( );
    bool        isPowerOverload( );

    void        checkOverload( );
    void        powerMeasurement( );
    void        syncPwm( );
    void        samplePowerMeasurement( );
    
    void                runTrackStateMachine( );
    void                printTrackConfig( );
    void                printTrackStatus( );
   
    private:

    uint16_t            options                         = BT_OPT_DEFAULT_SETTING;
    volatile uint16_t   flags                           = BT_F_DEFAULT_SETTING;

    volatile uint16_t   trackState                      = 0;
    volatile uint16_t   trackMode                       = 0;
    volatile uint16_t   trackSpeed                      = 0;      
    volatile uint32_t   trackTimeStamp                  = 0;
    volatile uint8_t    overloadEventCount              = 0;
    volatile uint8_t    overloadRestartCount            = 0;

    uint8_t             rNumEnable                      = 0;
    uint8_t             rNumControl                     = 0;
    uint8_t             rNumSense                       = 0;

    uint16_t            pwmFrequency                    = 0;
    uint16_t            initialTrackMode                = 0;
    uint16_t            initialTrackSpeed               = 0;
    uint16_t            initCurrentMilliAmp             = 0;
    uint16_t            limitCurrentMilliAmp            = 0;
    uint16_t            maxCurrentMilliAmp              = 0;

    uint16_t            startTimeThreshold              = 0;
    uint16_t            stopTimeThreshold               = 0;
    uint16_t            overloadTimeThreshold           = 0;
    uint16_t            overloadEventThreshold          = 0;
    uint16_t            overloadRestartThreshold        = 0;
    uint16_t            milliVoltPerAmp                 = 0;
    uint16_t            digitsPerAmp                    = 0;

    volatile uint16_t   actualCurrentDigitValue         = 0;
    volatile uint16_t   highWaterMarkDigitValue         = 0;
    volatile uint16_t   limitCurrentDigitValue          = 0;

    volatile uint32_t   totalPowerSamplesTaken                  = 0;
    uint32_t            lastPowerSampleTimeStamp                = 0;

    uint32_t            lastPowerSamplePerSecTaken              = 0;
    uint32_t            lastPowerSamplePerSecTimeStamp          = 0;
    uint32_t            powerSamplesPerSec                      = 0;

    uint8_t             powerSampleBufIndex                     = 0;
    uint16_t            powerSampleBuf[ PWR_SAMPLE_BUF_SIZE ]   = { 0 };

};

//----------------------------------------------------------------------------------------
// "LcsOccDetect" manages an Occupancy detector extension board. The track power 
// output of a block controller track is routed to an extension board which 
// implements a set of current detectors. The extension board is access via the
// extension I2C bus.
//
//----------------------------------------------------------------------------------------
struct LcsOccDetect {

    public:

    LcsOccDetect( );

    uint8_t setupOccDetect( uint16_t extBoardId );

    uint8_t getOccDetectMask( uint16_t *mask );
        
    private:
    
    uint16_t extBoardId;

};

//----------------------------------------------------------------------------------------
// "LcsSignal" manages a signal. A block has a signal for each direction to 
// indicate the state of the next block in a route.
//
//----------------------------------------------------------------------------------------
struct LcsSignalControl {

    public:

    LcsSignalControl( );

    uint8_t setupSignalControl( uint16_t extBoardId );

    private:

    uint16_t extBoardId;

};

//----------------------------------------------------------------------------------------
// "LcsTurnout" manages the optional turnouts at the end of a block.
//
//
//----------------------------------------------------------------------------------------
struct LcsTurnoutControl {

    public:

    LcsTurnoutControl( );

    uint8_t setupTurnoutControl( uint16_t extBoardId );

    private:

    uint16_t extBoardId;
};

//----------------------------------------------------------------------------------------
// "LcsRailComDetect" manages the optional RailCom interface for the block.
//
//----------------------------------------------------------------------------------------
struct LcsRailComDetect {

    public:

    LcsRailComDetect( );

    uint8_t setupRailComDetect( uint16_t extBoardId );

    private:

};

//----------------------------------------------------------------------------------------
// "LcsBlockControl" manages a block. A block consists mainly of the tack itself 
// and the optional elements detectors, signal and turnouts. The block logic, i.e.
// what to do when the next block is occupied, is handled here.
//
//
// ??? runs the block logic
// ??? how to assign occ detect an signals to the block ?
// ??? should message handling be a separate part ?
// 
//
// Most likely a state machine with sensors, events and current state as inputs.
// Sets the next state, turnouts, signals, block power and so on.
//
// It will be a set of rules to evaluate many times / per second.
//
// Example: "if ( actual speed > target speed ) decelerate;"
//          "if ( actual speed == target speed  ) do nothing;"
//          "if ( target speed > MAX ) target speed = MAX;"
//          "if ( target < 0 ) emergency stop"
//
// and so on....
//----------------------------------------------------------------------------------------
struct LcsBlockControl {

    LcsBlockControl(  );

    uint8_t setupBlockControl(  );    

    private:

    // ??? handles to detect, signal and turnout object.

};

//----------------------------------------------------------------------------------------
// A LCS block controller node can host up to four blocks. This object manages 
// the configured blocks on the node. The block controllers themselves manage 
// are stored in the block controller map array. Up to four blocks can be
// configured on a block controller node.
//
// The lcs block controller node implements the LCS node callbacks to handle any
// LCS message that is sent to the block controller node. LCS requests, replies
// and events are handled here and forwarded to the respective block controller
// object.
//
//----------------------------------------------------------------------------------------
struct LcsBlockControllerNode {

    public: 

    LcsBlockControllerNode( );

    uint8_t setupBockController( );

    uint8_t handleInitCallback( uint16_t npId );
    uint8_t handleResetCallback( uint16_t npId );
    uint8_t handlePfailCallback( uint16_t npId );
    uint8_t handleLcsMsgCallback( uint8_t *msg );

    uint8_t handleLcsReqCallback( uint16_t npId, 
                                  uint8_t item, 
                                  uint16_t *arg1, 
                                  uint16_t *arg2 );

    uint8_t handleLcsRepCallback( uint16_t npId, 
                                  uint8_t item, 
                                  uint16_t arg1, 
                                  uint16_t arg2, 
                                  uint8_t ret );

    uint8_t handleLcsEventCallback( uint16_t npId, 
                                    uint16_t eId, 
                                    uint8_t eAction, 
                                    uint16_t eData );

    private:

    uint16_t    options     = 0;
    uint16_t    flags       = 0;
    uint16_t    hwm         = 0;

    LcsBlockControl map[ 4 ];
};