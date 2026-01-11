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


//----------------------------------------------------------------------------------------
//
// A block is an abstract object with n WEST docking points and m EAST docking 
// points. The block is manage by a block controller, it is a power domain.
// Inside a block there are sections, turnouts and signals. The most simple form
// is a block with one WEST and one EAST dock and no turnouts.
//
//                        <--- direction
//                  :---------------------------:
//                  :         Block Id          :
//              WEST_1                      EAST_1  
//                  :                           :
//   Neighbors      :                           :    Neighbors 
//                  :                           :
//              WEST_n                      EAST_m 
//                  :                           :
//                  :---------------------------:
//
// The block controller manages all associated resources, turnout and signals. It
// is the intent that a layout can be managed by high level commands that the 
// block interprets. Fore example, a route reservation results in the block setting
// turnouts and signals accordingly. On automated routes, the block controls the 
// engine speed in its domain. 
// 
// All  blocks send events about their state. All blocks monitor their neighbors 
// via listening to events. 
// 
// A block can be operated in automatic and manual mode. Automatic mode is a 
// straightforward block control scheme. A route is reserved and trains can be 
// sent via this route. In manual mode a set of blocks, i.e. a route or a set of 
// blocks are reserved for manual operation.
// 
// A block does not cross check that the currently assigned CabId matches the
// engine on the track. While this could be done in digital mode via Railcom 
// support, an analog engine is not identifiable. All the block cares about is 
// that there is an engine, initially assigned to a cabId, and manages that 
// engine until it leaves the block. A receiving block queries its predecessor
// block for the cabId received. This simple scheme works for DCC and analog too.
//
//----------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------
// The block controller maintains a set of debug flags. The overall concept is 
// very similar to the LCS runtime library debug mask. Then following debug flags
// are defined:
//
//      DBG_BC_CONFIG                   -   DEBUG base station enabled
//      DBG_BC_SETUP                    -   show the setup steps
//      DBG_BC_LCS_MSG_INTERFACE        -   show the incoming LCS messages
//      DBG_BC_TRACK_POWER_MGMT         -   show the track power measurement data
//      DBG_BC_RAILCOM                  -   show the RailCom activity
//
// The way to use these flags is for example:
//
//      if (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_SESSION )) 
//
//----------------------------------------------------------------------------------------
enum BlockControllerDebugFlags : uint16_t {

    DBG_BC_CONFIG                  = 1 << 15,   // DEBUG enabled
    DBG_BC_SETUP                   = 1 << 1,    // show setup steps
    DBG_BC_LCS_MSG_INTERFACE       = 1 << 2,    // show incoming LCS messages
    DBG_BC_TRACK_POWER_MGMT        = 1 << 3,    // show track power data
    DBG_BC_RAILCOM                 = 1 << 4     // show the RailCom activity
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
    //  - initial state:    normal / thrown     ( 4bits )
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
    BCI_CURRENT_TARGET_SPEED            = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_ROUTE_ID                = ITEM_ID_USER_START + 0,   // GET / SET
    BCI_CURRENT_ROUTE_SET               = ITEM_ID_USER_START + 0,   // GET / SET
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
    // Block command codes. The block is managed by a control system via high 
    // level command or explicitly via low level commands that manage the 
    // individual components of a block. The latter command set is used for example 
    // by train control systems such as RocRail, that assume a central command 
    // station, sending their commands to that station. For support such systems,
    // a LCS gateway node is part of the system which accepts these commands and 
    // translates them into LCS node messages for the respective block controller. 
    // 
    // An alternative is the management of the layout via higher level commands. 
    // In this scenario, the block is a high level entity and manages turnouts,
    // signals and the current engine in alignment with its neighboring blocks. 
    // The train master just issues commands such as "reserve route", "send train"
    // and so on.
    //
    // It is a bit of a headache to manage hybrid systems, i.e analog and digital
    // engine on the layout, when the control system explicitly manages the layout.
    // Since the block controllers normally do not have the configuration data
    // of the control system, commands such as "engine n speed m" cannot decoded
    // for analog engines. Working with in high level block is much easier, 
    // since the configuration data puts block controllers in a position to 
    // manager any kind of train their sections.
    //
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
// Base station errors. Note that they need to be in the assigned to the user number
// range of errors defined in the LCS runtime library. 
//
//----------------------------------------------------------------------------------------
enum BlockControllerErrors : uint8_t {

    BLOCK_CONTROLLER_ERR_BASE       = 128,

    ERR_MSG_INTERFACE_SETUP         = BLOCK_CONTROLLER_ERR_BASE + 10,
    ERR_DCC_TRACK_CONFIG            = BLOCK_CONTROLLER_ERR_BASE + 11,
    ERR_PIN_CONFIG                  = BLOCK_CONTROLLER_ERR_BASE + 12,
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
// The block track object has a set of flags to indicate its current status.
//
//  DT_F_POWER_ON             - The track is under power.
//  DT_F_POWER_OVERLOAD       - An overload situation was detected.
//  DT_F_MEASUREMENT_ON       - The power measurement is enabled.
//  DT_F_SERVICE_MODE_ON      - The track is currently in service mode, i.e. is a PROG track.
//  DT_F_CUTOUT_MODE_ON       - The track has the cutout generation enabled.
//  DT_F_RAILCOM_MODE_ON      - The track has the railcom detect enabled.
//  DT_F_RAILCOM_MSG_PENDING  - If railcom is enabled, a received datagram is indicated.
//  DT_F_CONFIG_ERROR         - The passed configuration descriptor has invalid options configured.
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
// The block controller can contain up to four blocks. Each block track is described
// by the LcsBlockDesc descriptor. There are the hardware pins sel1Pin1, selPin2, 
// sensePin and uartRxPin. In addition there are the limits for current consumption
// values, all specified in milliAmps. The initial current sets the current 
// consumption limit after the track is turned on. The limit current consumption 
// specifies the actual configured value that is checked for a track current overload
// situation. The maximum current defines what current the power module should never
// exceed. For the measurements to work, the power module needs to deliver a voltage
// that corresponds to the current drawn on the track. The value is measured in 
// milliVolt per Ampere drawn. Finally, there are threshold times for managing the
// track overload and restart capability.
//
//----------------------------------------------------------------------------------------
struct LcsBlockTrackDesc {

    uint16_t    options;

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

    uint8_t                     setupBlockTrack( LcsBlockTrackDesc* trackDesc );
    uint8_t                     setTrackState( uint16_t state );
    uint8_t                     setTrackMode( uint16_t mode, uint8_t speed = 0 );
    uint8_t                     setPwmFrequency( uint32_t frequency );

    uint16_t                    getFlags( );
    uint16_t                    getOptions( );

    void                        runTrackStateMachine( );

    void                        powerStart( );
    void                        powerStop( );
    bool                        isPowerOn( );
    bool                        isPowerOverload( );
  
    void                        setLimitCurrent( uint16_t val );
    uint16_t                    getLimitCurrent( );
    uint16_t                    getActualCurrent( );
    uint16_t                    getInitCurrent( );
    uint16_t                    getMaxCurrent( );
    uint16_t                    getRMSCurrent( );

    void                        checkOverload( );
    void                        powerMeasurement( );

    uint32_t                    getPwrSamplesTaken( );
    uint16_t                    getPwrSamplesPerSec( );

    void                        printTrackConfig( );
    void                        printTrackStatus( );

    private:

    uint16_t                    options                         = BT_OPT_DEFAULT_SETTING;
    volatile uint16_t           flags                           = BT_F_DEFAULT_SETTING;

    volatile uint16_t           trackState                      = 0;
    volatile uint16_t           trackMode                       = 0;
    volatile uint16_t           trackSpeed                      = 0;      
    volatile uint32_t           trackTimeStamp                  = 0;
    volatile uint8_t            overloadEventCount              = 0;
    volatile uint8_t            overloadRestartCount            = 0;

    uint8_t                     rNumEnable                      = 0;
    uint8_t                     rNumControl                     = 0;
    uint8_t                     rNumSense                       = 0;

    uint16_t                    pwmFrequency                    = 0;
    uint16_t                    initialTrackMode                = 0;
    uint16_t                    initialTrackSpeed               = 0;
    uint16_t                    initCurrentMilliAmp             = 0;
    uint16_t                    limitCurrentMilliAmp            = 0;
    uint16_t                    maxCurrentMilliAmp              = 0;

    uint16_t                    startTimeThreshold              = 0;
    uint16_t                    stopTimeThreshold               = 0;
    uint16_t                    overloadTimeThreshold           = 0;
    uint16_t                    overloadEventThreshold          = 0;
    uint16_t                    overloadRestartThreshold        = 0;

    uint16_t                    milliVoltPerAmp                 = 0;
    uint16_t                    digitsPerAmp                    = 0;
    volatile uint16_t           actualCurrentDigitValue         = 0;
    volatile uint16_t           highWaterMarkDigitValue         = 0;
    volatile uint16_t           limitCurrentDigitValue          = 0;

    volatile uint32_t           totalPwrSamplesTaken            = 0;
    uint32_t                    lastPwrSampleTimeStamp          = 0;

    uint32_t                    lastPwrSamplePerSecTaken        = 0;
    uint32_t                    lastPwrSamplePerSecTimeStamp    = 0;
    uint32_t                    pwrSamplesPerSec                = 0;

    uint8_t                     pwrSampleBufIndex                       = 0;
    uint16_t                    pwrSampleBuf[ PWR_SAMPLE_BUF_SIZE ]     = { 0 };

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

    uint8_t getOccDetectMask( uint16_t *mask );

    private:    

    // ??? need to remember the extension board ID.

};

//----------------------------------------------------------------------------------------
// "LcsSignal" manages a signal. A block has a signal for each direction to 
// indicate the state of the next block in a route.
//
//----------------------------------------------------------------------------------------
struct LcsSignalControl {

    public:

    LcsSignalControl( );


    private:

    // ??? need to remember the extension board ID.

};

//----------------------------------------------------------------------------------------
// "LcsTurnout" manages the optional turnouts at the end of a block.
//
//
//----------------------------------------------------------------------------------------
struct LcsTurnoutControl {

    public:

    LcsTurnoutControl( );

    private:

    // ??? need to remember the extension board ID.
};

//----------------------------------------------------------------------------------------
// "LcsRailComDetect" manages the optional RailCom interface for the block.
//
//----------------------------------------------------------------------------------------
struct LcsRailComDetect {

    public:

    LcsRailComDetect( );

    private:

    // ??? need to remember the extension board ID.
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

    uint8_t handleLcsRequest( uint8_t *msg );

   
    private:

    // ??? handles to detect, signal and turnout object.

};

//----------------------------------------------------------------------------------------
// A LCS block controller node can host up to four blocks. This object is the main
// object that manages the blocks on the node.
//
// ??? the node descriptor is an array of block descriptors. They are kept in the NVM ?
// ??? manages the LCS messages and forwards them to the target block.
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