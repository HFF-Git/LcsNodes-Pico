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
const uint16_t NVM_CAB_MAP_OFS = 256;  // ??? for now ... it is items !!!

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

    SM_F_NIL                    = 0,
    SM_F_KEEP_ALIVE_CHECKING    = 1 << 0,
    SM_F_ENABLE_REFRESH         = 1 << 1
};

//----------------------------------------------------------------------------------------
// Maximum number of cabMap entries. The maximum size size depends on the size
// of the NVM chip on the base station board. 
// 
//----------------------------------------------------------------------------------------
const uint16_t  MAX_CAB_DICT_SESSIONS   = 128;
const uint16_t  MAX_CAB_ACTIVE_SESSIONS = 32;

//----------------------------------------------------------------------------------------
// Each cab map entry has a set of flags. 
//
//  CMAP_F_ACTIVE                - the cab is active and in the active list.
//  CMAP_F_ALIVE                 - the cab is marked alive.
//  CMAP_F_COMBINED_REFRESH      - a DCC cab support the combined refresh option. 
//  CMAP_F_SPDIR_REFRESH         - a DCC cab refreshes speed/dir.
//  CMAP_F_FUNC_REFRESH          - a DCC cab refreshes functions. 
// 
// 
//----------------------------------------------------------------------------------------
enum CabMapEntryFlags : uint16_t {

    CMAP_F_NIL                      = 0,
    CMAP_F_ACTIVE                   = 1 << 0,
    CMAP_F_ALIVE                    = 1 << 1,
    CMAP_F_COMBINED_REFRESH         = 1 << 2,
    CMAP_F_SPDIR_REFRESH            = 1 << 3,
    SME_FUNC_REFRESH                = 1 << 4,
   
};

//----------------------------------------------------------------------------------------
// The Cab Map data structure is an array of cab entries. Cab entries are a set
// of 16 attributes, i.e. the size is 32bytes. Entry number zero is the cab map 
// header. The first 8 words are the NVM header info, which is also used in the 
// runtime library. It contains the magic word, the NVM location and size, and
// the current number of configured cabs in the array. 
//
// ??? we my keep also some other info here... convenient.
//----------------------------------------------------------------------------------------
struct LcsCabDictHead {

    uint32_t    magicWord;                                      // word 0 - 1 
    uint32_t    nvmOfs;                                         // word 2 - 3
    uint32_t    nvmSize;                                        // word 4 - 5
    uint32_t    rsv1;                                           // word 6 - 7
    uint16_t    maxCabCount;                                    // word 8
    uint16_t    currentCabCount;                                // word 9                    
    uint16_t    rsv2[ 6 ];                                      // word 10 - 15 
};

//----------------------------------------------------------------------------------------
// Every locomotive know to the system has an entry in the cabMap. Loading the
// cabMap results in a sorted array of cabMap entries. Cab data is stored in a 
// 16-word entry. It contains the configured initial data for the cab. The 
// entries are accessed from the runtime NVM using the getAttr / setAttr 
// functions.
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
//      4   :   SpeedInfo               :       :   SpeedInfo               : 
//          :---------------------------:       :---------------------------:
//      5   :   Speed map MIN,s1        :       :   Speed map MIN,s1        : 
//          :---------------------------:       :---------------------------:
//      6   :   Speed map s2, s3        :       :   Speed map s2, s3        : 
//          :---------------------------:       :---------------------------:
//      7   :   Speed map s4, MAX       :       :   Speed map s4, MAX       : 
//          :---------------------------:       :---------------------------:
//      8   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      9   :                           :       :                           : 
//          :===========================:       :===========================:
//     10   :   DCC Info.               :       :   Analog Info             : 
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
//
//
//  cabId           -   the cab assigned number. This is typically the DCC 
//                      address. Analog cabs also get a number, as well as 
//                      cab engine consists. 
// 
//  flags           -   ...
//
//  speedInfo       -   the speed and direction. 
//
//  speedMap        -   a set of 8 speed values.
//
//  dFlags          -   DCC cab specific 
//
//  functions       -   the set of DCC function groups.
// 
//  aFlags          -   analog cab specific 
//
//  PwmFrequency    -   the PWM option for the analog engine
// 
//                      
// still need type, and some other data ... ?
// ??? need a word or two for block data, e.g. a block reports on the loco...
// 
//----------------------------------------------------------------------------------------
struct LcsCabEntry {

    uint16_t    cabId;                                          // word 0 
    uint16_t    flags;                                          // word 1
    uint16_t    rsv1;                                           // word 2
    uint16_t    rsv2;                                           // word 3

    uint16_t    speedInfo;                                      // word 4
    uint8_t     speedMap[ 6 ];                                  // word 5 - 7 

    uint16_t    rsv3;                                           // word 8
    uint16_t    rsv4;                                           // word 9
    
    union {

        struct {

            uint16_t    dFlags;                                 // word 10
            uint8_t     functions[ MAX_DCC_FUNC_GROUP_ID ];     // word 11 - 15

        } d;

        struct {

            uint16_t    aFlags;                                 // word 10
            uint16_t    pwmFrequency;                           // word 11
            uint16_t    rsv1;                                   // word 12
            uint16_t    rsv2;                                   // word 13
            uint16_t    rsv3;                                   // word 14
            uint16_t    rsv4;                                   // word 15

        } a;
    };
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

    uint8_t             formatCabMap( );
    uint8_t             loadCabMap( );
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

    uint8_t             setThrottle( LcsCabEntry *cePtr, 
                                     uint8_t speed, 
                                     uint8_t direction );

    uint8_t             setDccFunctionBit( uint16_t cabId,
                                           uint8_t funcNum, 
                                           uint8_t val );

    uint8_t             setDccFunctionGroup( uint16_t cabId, 
                                             uint8_t fGroup, 
                                             uint8_t dccByte );

    uint8_t             setDccFunctionGroup( LcsCabEntry *cePtr, 
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
    void                printCabInfo( LcsCabEntry *cePtr );
    void                printCabMap( );

    private:

    uint16_t            debugMask               = 0;
    uint16_t            options                 = DT_OPT_NIL;
    uint16_t            flags                   = DT_F_NIL;
    uint32_t            lastAliveCheckTime      = 0L;
    uint32_t            refreshAliveTimeOutVal  = DCC_SESSION_TIMEOUT_MILLIS;

    LcsDccTrack         *mainTrack              = nullptr;
    LcsDccTrack         *progTrack              = nullptr;

    
    uint16_t            cabCount                = 0;
    uint16_t            activeCabCount          = 0;
    uint16_t            cabRefreshIndex         = 0;

    LcsCabDictHead      cabMaphead;
    LcsCabEntry         cabMap[ MAX_CAB_DICT_SESSIONS ];
    int                 activeCabList[ MAX_CAB_ACTIVE_SESSIONS ];

    LcsCabEntry         *activateCabEntry( uint16_t cabId );
    void                deactivateCabEntry( uint16_t cabId );

    void                addActiveCab( uint16_t locoIndex );
    void                removeActiveCab( uint16_t locoIndex );
    void                refreshActiveCabs( );
    void                refreshActiveCabEntry( LcsCabEntry *cab );
};
