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
//  SME_ACTIVE              - 
//  SME_COMBINED_REFRESH    - locomotive speed/dir and functions are refreshed.
//  SME_SPDIR_REFRESH       - locomotive speed/dir are refreshed.
//  SME_FUNC_REFRESH        - locomotive functions are refreshed. 

// 
// ??? rework a little ...
//----------------------------------------------------------------------------------------
enum SessionMapEntryFlags : uint16_t {

    SME_DEFAULT_SETTING     = 0,
    SME_ACTIVE              = 1 << 0,
    SME_COMBINED_REFRESH    = 1 << 1,
    SME_SPDIR_REFRESH       = 1 << 2,
    SME_FUNC_REFRESH        = 1 << 3,
   
};


//----------------------------------------------------------------------------------------
// Maximum number of cabMap entries. The maximum size size depends on the size
// of the NVM chip on the base station board. 
// 
// ??? for now 128... to get going ...
// ??? for active sessions, no real mem limit, but what is realistic ?
//----------------------------------------------------------------------------------------
const uint16_t  MAX_CAB_DICT_SESSIONS   = 128;
const uint16_t  MAX_CAB_ACTIVE_SESSIONS = 32;


//----------------------------------------------------------------------------------------
// Every locomotive know to the system has an entry in the cabMap. Loading the
// cabMap results in a sorted array of cabMap entries.
//
// Cab data is stored in an 16-word entry. It contains the configured initial
// data for the cab. The entries are accessed from the runtime NVM using the 
// getAttr / setAttr functions.
//
// ??? actually, we do not really have a struct on the NVM. So, perhaps these
// fields are just offsets...
//
// The following table shows the dictionary word layout for digital and analog
// locomotives.
//
//              DCC Mode                            Analog Mode
//
//          :---------------------------:       :---------------------------:   
//      0   :   cabId                   :       :   cabId                   :
//          :---------------------------:       :---------------------------:
//      1   :   flags                   :       :   flags                   :  
//          :---------------------------:       :---------------------------:
//      2   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      3   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      4   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      5   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      6   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      7   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      8   :   Speed map MIN,s1        :       :   Speed map MIN,s1        : 
//          :---------------------------:       :---------------------------:
//      9   :   Speed map s2, s3        :       :   Speed map s2, s3        : 
//          :---------------------------:       :---------------------------:
//     10   :   Speed map s4, MAX       :       :   Speed map s4, MAX       : 
//          :---------------------------:       :---------------------------:
//     11   :   DCC function map 0      :       :   PWM Frequency           : 
//          :---------------------------:       :---------------------------:
//     12   :   DCC function map 1      :       :                           : 
//          :---------------------------:       :---------------------------:
//     13   :   DCC function map 2      :       :                           : 
//          :---------------------------:       :---------------------------:
//     14   :   DCC function map 3      :       :                           : 
//          :---------------------------:       :---------------------------:
//     15   :   DCC function map 4      :       :                           : 
//          :---------------------------:       :---------------------------:
   
// DCC function map 4 uses 4 bits for F64 ... F68, remaining 11 bits are used
// for function group change flag.

// PWM frequency is encoded ....

// Speed map contains the MIN, s1, s2, s3, s4, MAX settings. In DCC mode this 
// is a speed step, in analog a PWM width or also a speed setting encoded as
// 128 steps...

// still need type, and some other data ... ?
// ??? need a word or two for block data, e.g. a block reports on the loco...

// 


//----------------------------------------------------------------------------------------
struct LcsCabEntry {

    // ??? build a union with a struct for each ?
    // ??? just keep cabId and flags outside ... ?

    uint16_t  cabId               = NIL_CAB_ID;
    uint16_t  flags               = SME_DEFAULT_SETTING;
    
    uint8_t   speed               = 0;
    uint8_t   speedSteps          = 128;
    uint8_t   direction           = 0;
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

    uint8_t             setupLocoSessions( uint16_t options, 
                                           LcsDccTrack *mainTrack,
                                           LcsDccTrack *progTrack
                                         );

    uint8_t             loadCabMap( );
    uint8_t             loadCabMapEntry( uint16_t cabId );
    uint8_t             addCabEntry( LcsCabEntry *entry );
    uint8_t             removeCabEntry( uint16_t cabId );
    LcsCabEntry         *lookupCabEntry( uint16_t cabId );
    
    uint16_t            getOptions( );
    uint16_t            getFlags( );
    uint16_t            getCabCount( );

    void                emergencyStopAll( );

    uint8_t             setThrottle( uint16_t cabId, 
                                     uint8_t speed, 
                                     uint8_t direction );

    uint8_t             setThrottle( LcsCabEntry *cabEntryPtr, 
                                     uint8_t speed, 
                                     uint8_t direction );

    uint8_t             setDccFunctionBit( uint16_t cabId,
                                           uint8_t funcNum, 
                                           uint8_t val );

    uint8_t             setDccFunctionGroup( uint16_t cabId, 
                                             uint8_t fGroup, 
                                             uint8_t dccByte );

    uint8_t             setDccFunctionGroup( LcsCabEntry *cPtr, 
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

    uint8_t             markCabAlive( uint16_t cabId );
    uint8_t             refreshActiveSessions( );
    
    void                printConfig( );
    void                printCabInfo( LcsCabEntry *csPtr );
    void                printCabMap( );

    private:

    uint16_t            options                 = DT_OPT_NIL;
    uint16_t            flags                   = DT_F_NIL;
    uint32_t            lastAliveCheckTime      = 0L;
    uint32_t            refreshAliveTimeOutVal  = DCC_SESSION_TIMEOUT_MILLIS;

    LcsDccTrack         *mainTrack              = nullptr;
    LcsDccTrack         *progTrack              = nullptr;

    uint16_t            nvmCabCount             = 0;
    uint16_t            cabCount                = 0;
    uint16_t            activeCabCount          = 0;
    uint16_t            cabRefreshIndex         = 0;
    LcsCabEntry         cabMap[ MAX_CAB_DICT_SESSIONS ];
    int                 activeCabList[ MAX_CAB_ACTIVE_SESSIONS ];

    LcsCabEntry         *activateCabEntry( uint16_t cabId );
    void                deactivateCabEntry( uint16_t cabId );

    void                addActiveCab( uint16_t locoIndex );
    void                removeActiveCab( uint16_t locoIndex );
    void                refreshActiveCabs( );
    void                refreshActiveCabEntry( LcsCabEntry *cab );
};
