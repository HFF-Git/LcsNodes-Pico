///---------------------------------------------------------------------------------------
//
// LCS - Loco Session Lib
//
///---------------------------------------------------------------------------------------
// 
//
///---------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include File
// Copyright (C) 2020 - 2026  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#pragma once

#include "LcsUtilLib.h"
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsDccTrackLib.h"

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// The base station maintains a set of debug flags. The overall concept is very 
// similar to the LCS runtime library debug mask. Then following debug flags are 
// defined:
//
//      DBG_LS_CONFIG                   -   DEBUG base station enabled
//      DBG_LS_SESSION                  -   show the session management actions
//      DBG_LS_CHECK_ALIVE_SESSIONS     -   display check for inactive sessions
//
// The way to use these flags is for example:
//
//      if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_SESSION )) 
//
// ??? should have a command to set the debug mask on the fly...
//----------------------------------------------------------------------------------------
enum LcsLocoSessionsDebugFlags : uint16_t {

    DBG_LS_CONFIG                  = 1 << 15,   

    DBG_LS_SESSION                 = 1 << 0,         
    DBG_LS_CHECK_ALIVE_SESSIONS    = 1 << 1,          
};

//----------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user 
// number range of errors defined in the LCS runtime library. The first 128 error 
// codes are reserved for the LCS library.
//
//----------------------------------------------------------------------------------------
enum LcsLocoSessionErrors : uint8_t {

    LOCO_SESSIONS_ERR_BASE            = ERR_USER_SPECIFIC_BASE,

    ERR_NO_SVC_MODE                   = LOCO_SESSIONS_ERR_BASE + 1,
    ERR_CV_OP_FAILED                  = LOCO_SESSIONS_ERR_BASE + 2,

    ERR_LOCO_SESSION_ALLOCATE         = LOCO_SESSIONS_ERR_BASE + 6,
    ERR_LOCO_SESSION_CANCELLED        = LOCO_SESSIONS_ERR_BASE + 7,

    ERR_SESSION_SETUP                 = LOCO_SESSIONS_ERR_BASE + 9,
};

//----------------------------------------------------------------------------------------
// DCC Session timeout value. These timeouts are used to determine when a session
// has timed out and needs to be removed from the session map.
//
//----------------------------------------------------------------------------------------
const uint32_t  DCC_SESSION_TIMEOUT_MILLIS = 2000;

//----------------------------------------------------------------------------------------
// The session map options. These are options initially set when the base station 
// starts. They are used to set the flags, which are then used for processing the 
// the actual settings. 
//
//  SM_KEEP_ALIVE_CHECKING  -   enable keep alive checking. When enabled, the engine
//                              session need to receive a keep alive LCS message 
//                              periodically.
//  SM_ENABLE_REFRESH       -   refresh the session data. This will send the engine
//                              speed and direction as well as the function flags
//                              periodically in a round robin processing of the
//
//  SM_SESSION_AUTO_CREATE  -   when an engine command comes in and there is no 
//                              session assigned to the engine, we will create a 
//                              session. If there is no room in the session table,
//                              the oldest entry that is not currently active will
//                              be removed to make room for the new session.
//
//----------------------------------------------------------------------------------------
enum SessionMapOptions : uint16_t {

    SM_OPT_DEFAULT_SETTING      = 0,
    SM_OPT_KEEP_ALIVE_CHECKING  = 1 << 0,
    SM_OPT_ENABLE_REFRESH       = 1 << 1,
    SM_SESSION_AUTO_CREATE      = 1 << 3,
};

//----------------------------------------------------------------------------------------
// The session map flags. The apply to all sessions in the session map. The initial
// values are copied from session option initial values.
//
//  SM_F_KEEP_ALIVE_CHECKING  - enable keep alive checking. When enabled, the engine
//                              session need to receive a keep alive LCS message 
//                              periodically.
//  SM_F_ENABLE_REFRESH       - refresh the session data. This will send the engine
//                              speed and direction as well as the function flags 
//                              periodically in a round robin fashion.
//
//----------------------------------------------------------------------------------------
enum SessionMapFlags : uint16_t {

    SM_F_DEFAULT_SETTING        = 0,
    SM_F_KEEP_ALIVE_CHECKING    = 1 << 0,
    SM_F_ENABLE_REFRESH         = 1 << 1
};

//----------------------------------------------------------------------------------------
// Each session map entry has a set of flags.
//
//  SME_ALLOCATED           - the session is allocated, the entry valid.
//  SME_COMBINED_REFRESH    - locomotive speed/dir and functions are refreshed.
//  SME_SPDIR_REFRESH       - locomotive speed/dir are refreshed.
//  SME_FUNC_REFRESH        - locomotive functions are refreshed. 
//  SME_DISPATCHED          -
//  SME_SHARED              -
// 
//----------------------------------------------------------------------------------------
enum SessionMapEntryFlags : uint16_t {

    SME_DEFAULT_SETTING     = 0,
    SME_ALLOCATED           = 1 << 0,
    SME_COMBINED_REFRESH    = 1 << 1,
    SME_SPDIR_REFRESH       = 1 << 2,
    SME_FUNC_REFRESH        = 1 << 3,
    SME_DISPATCHED          = 1 << 4,
    SME_SHARED              = 1 << 5
};



//----------------------------------------------------------------------------------------
// Timeout intervals for various base station tasks. Measured in milliseconds.
//
//----------------------------------------------------------------------------------------

const uint32_t  SESSION_REFRESH_TASK_INTERVAL   = 50;


//----------------------------------------------------------------------------------------
// Maximum number of cab sessions supported by the base station.
//
// ??? goes into Session File...
//----------------------------------------------------------------------------------------
const uint16_t  MAX_CAB_SESSIONS  = 128;

//----------------------------------------------------------------------------------------
// For creating the Loco Session object the session map object is described by the
// following descriptor.
//
// ??? we may not need this, just options...
//----------------------------------------------------------------------------------------
struct LcsBaseStationSessionMapDesc {

    uint16_t    options       = SM_OPT_DEFAULT_SETTING;
    uint16_t    maxSessions   = MAX_CAB_SESSIONS;
};







//----------------------------------------------------------------------------------------
// Every allocated loco session is described by the cabMap structure. There 
// are the engine cab Id, speed, direction and function information. There is also
// a field that indicates when we received information for this session from a cab 
// control handheld. The function flags are stored in an array, where each byte 
// represents a DCC function group. So function 0 to 7 are in byte 0, function 8
// to 15 in byte 1 and so on, up to function 80 in byte 9. This makes it easy to 
// set a group. Most of the fields are actually used for a DCC type locomotive. 
// When the locomotive is an analog engine, only a subset of the fields is actually
// used. Nevertheless, even for an analog engine we will have a session. The base 
// station will however not generate packets for this engine.
//
//----------------------------------------------------------------------------------------
struct LcsCabEntry {

  uint16_t  flags               = SME_DEFAULT_SETTING;
  uint16_t  cabId               = NIL_CAB_ID;
  uint8_t   speed               = 0;
  uint8_t   speedSteps          = 128;
  uint8_t   direction           = 0;
  uint8_t   engineState         = 0;
  uint8_t   nextRefreshStep     = 0;
  uint32_t  lastKeepAliveTime   = 0;
  uint8_t   functions[ MAX_DCC_FUNC_GROUP_ID ] = { 0 };
};

//----------------------------------------------------------------------------------------
// The loco sessions object is the central data structure for the base station 
// loco management. 
//
// 
// ??? some rework... 
//----------------------------------------------------------------------------------------
struct LcsLocoSessions {

  public:

    LcsLocoSessions( );

    uint8_t             setup( uint16_t options, 
                               LcsDccTrack *mainTrack,
                               LcsDccTrack *progTrack
                             );

    uint16_t            getOptions( );
    uint16_t            getFlags( );

   
    void                emergencyStopAll( );

    uint8_t             setThrottle( uint16_t cabId, 
                                     uint8_t speed, 
                                     uint8_t direction );

    uint8_t             setDccFunctionBit( uint16_t cabId,
                                           uint8_t funcNum, 
                                           uint8_t val );

    uint8_t             setDccFunctionGroup( uint16_t cabId, 
                                             uint8_t fGroup, 
                                             uint8_t dccByte );

    uint8_t             writeCVMain( uint16_t cabId, 
                                     uint16_t cvId, 
                                     uint8_t mode, 
                                     uint8_t val );

    uint8_t             writeCVByteMain( uint16_t cabId, 
                                         uint16_t cvId,
                                         uint8_t val );

    uint8_t             writeCVBitMain( uint16_t cabId, 
                                        uint16_t cvId, 
                                        uint8_t bitPos, 
                                        uint8_t val );

    uint8_t             readCV( uint16_t cvId, uint8_t mode, uint8_t *val );
    uint8_t             readCVByte( uint16_t cvId, uint8_t *val );
    uint8_t             readCVBit( uint16_t cvId, uint8_t bitPos, uint8_t *val );

    uint8_t             writeCV( uint16_t cvId, uint8_t mode, uint8_t val );
    uint8_t             writeCVByte( uint16_t cvId, uint8_t val );
    uint8_t             writeCVBit( uint16_t cvId, uint8_t bitPos, uint8_t val );

    uint8_t             writeDccPacketMain( uint8_t *buf,  
                                            uint8_t len, 
                                            uint8_t nRepeat );

    uint8_t             writeDccPacketProg( uint8_t *buf,  
                                            uint8_t len, 
                                            uint8_t nRepeat );

    void                refreshActiveSessions( );
    uint8_t             markSessionAlive( uint16_t cabId );
    uint32_t            getSessionKeepAliveInterval( );


    void                printConfig( );
    void                printCabInfo( LcsCabEntry *csPtr );
    void                printCabMap( );

    LcsCabEntry         *lookupCabEntry( uint16_t cabId );
  
    private:

    uint16_t            options                 = DT_OPT_DEFAULT_SETTING;
    uint16_t            flags                   = DT_F_DEFAULT_SETTING;
    uint32_t            lastAliveCheckTime      = 0L;
    uint32_t            refreshAliveTimeOutVal  = DCC_SESSION_TIMEOUT_MILLIS;

    LcsDccTrack         *mainTrack              = nullptr;
    LcsDccTrack         *progTrack              = nullptr;

    uint16_t            cabCount                = 0;
    uint16_t            cabIndexMap[ MAX_CAB_SESSIONS ];
    LcsCabEntry         cabMap[ MAX_CAB_SESSIONS ];

    void                setupCabEntry( LcsCabEntry *entry, uint16_t cabId );
    LcsCabEntry         *allocateCabEntry( uint16_t cabId );
    void                deallocateCabEntry( uint16_t cabId );
};
