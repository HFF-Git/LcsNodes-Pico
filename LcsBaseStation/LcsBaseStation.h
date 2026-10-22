//----------------------------------------------------------------------------------------
//
// LCS Base Station - Include file
//
//----------------------------------------------------------------------------------------
//
// LCS - Base Station
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
#include "LcsDccTrackLib.h"
#include "LcsLocoSessionLib.h"

using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// The base station maintains a set of debug flags. The overall concept is very 
// similar to the LCS runtime library debug mask. Then following debug flags are 
// defined:
//
//      DBG_BS_CONFIG                   -   DEBUG base station enabled
//      DBG_BS_SESSION                  -   show the session management actions
//      DBG_BS_LCS_MSG_INTERFACE        -   show the incoming LCS messages
//      DBG_BS_TRACK_POWER_MGMT         -   show the track power measurement data
//      DBG_BS_DCC_ACK_DETECT           -   display decoder ACK power measurements
//      DBG_BS_CHECK_ALIVE_SESSIONS     -   display check for inactive sessions
//      DBG_BS_RAILCOM                  -   show the RailCom activity
//
// The way to use these flags is for example:
//
//      if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_SESSION )) 
//
// ??? should have a command to set the debug mask on the fly...
//----------------------------------------------------------------------------------------
enum BaseStationDebugFlags : uint16_t {

   // DBG_BS_CONFIG                  = 1 << 15,   

    DBG_BS_SESSION                 = 1 << 0,    
    DBG_BS_LCS_MSG_INTERFACE       = 1 << 1,    
    DBG_BS_TRACK_POWER_MGMT        = 1 << 2,        
    DBG_BS_DCC_ACK_DETECT          = 1 << 3,        
    DBG_BS_CHECK_ALIVE_SESSIONS    = 1 << 4,        
    DBG_BS_RAILCOM                 = 1 << 5,
    
    DBG_BS_ENABLED                  = 1 << 15
};

//----------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user 
// number range of errors defined in the LCS runtime library. The first 128 error 
// codes are reserved for the LCS library.
//
//----------------------------------------------------------------------------------------
enum BaseStationErrors : uint8_t {

    BASE_STATION_ERR_BASE           = ERR_USER_SPECIFIC_BASE,

    ERR_BASE_STATION_CMD_INIT       = BASE_STATION_ERR_BASE + 1,
    ERR_BASE_STATION_MSG_INIT       = BASE_STATION_ERR_BASE + 2,

    ERR_LOCO_NOT_FOUND              = BASE_STATION_ERR_BASE + 4,
    ERR_SESSION_NOT_FOUND           = BASE_STATION_ERR_BASE + 5,
    ERR_LOCO_SESSION_ALLOCATE       = BASE_STATION_ERR_BASE + 6,
    ERR_LOCO_SESSION_CANCELLED      = BASE_STATION_ERR_BASE + 7,

    
    ERR_MSG_INTERFACE_SETUP         = BASE_STATION_ERR_BASE + 10,
  
    ERR_DCC_PIN_CONFIG              = BASE_STATION_ERR_BASE + 12,

    ERR_NVM_HW_SETUP                = BASE_STATION_ERR_BASE + 15,
    ERR_PIO_HW_SETUP                = BASE_STATION_ERR_BASE + 16,

    ERR_INVALID_COMMAND             = BASE_STATION_ERR_BASE + 17,
    ERR_INVALID_CHAN                = BASE_STATION_ERR_BASE + 17,
};

//----------------------------------------------------------------------------------------
// The base station items for attributes and request LCS calls. 
//
// ??? document them here ?
//----------------------------------------------------------------------------------------
enum BaseStationItems : uint8_t {

    // or use GET in all constants

    BS_ITEM_BASE                    = ITEM_ID_USER_START,

    BS_ITEM_SESSION_MAP_OPTIONS     = ITEM_ID_USER_START + 0,
    BS_ITEM_SESSION_MAP_FLAGS       = ITEM_ID_USER_START + 1,
    BS_ITEM_MAX_SESSIONS            = ITEM_ID_USER_START + 2,
    BS_ITEM_ACTIVE_SESSIONS         = ITEM_ID_USER_START + 3,

    BS_ITEM_INIT_CURRENT_VAL        = ITEM_ID_USER_START + 4,
    BS_ITEM_LIMIT_CURRENT_VAL       = ITEM_ID_USER_START + 5,
    BS_ITEM_MAX_CURRENT_VAL         = ITEM_ID_USER_START + 6,
    BS_ITEM_ACTUAL_CURRENT_VAL      = ITEM_ID_USER_START + 7,

    // thresholds

    // eventID to send for events ?

};

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
extern uint8_t  initSerialCommand( );
extern uint8_t  handleSerialCommand( uint8_t chan, char *s );

extern uint8_t  initLcsMsgInterface( );
extern void     handleLcsMsg( uint8_t *msg );
