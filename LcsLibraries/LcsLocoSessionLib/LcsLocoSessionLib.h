//----------------------------------------------------------------------------------------
//
// LCS - Loco Session Lib
//
//----------------------------------------------------------------------------------------
// The loc session library contains the code for the base station locomotive
// session functionality. 
//
//----------------------------------------------------------------------------------------
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
// Name spaces.
//
//----------------------------------------------------------------------------------------
using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// Fundamental constants for the loco session library. 
// 
//----------------------------------------------------------------------------------------
constexpr uint16_t  MAX_CAB_DICT_SESSIONS       = 128;
constexpr uint16_t  MAX_CAB_ACTIVE_SESSIONS     = 32;

constexpr uint16_t  CAB_ENTRY_ITEM_SIZE         = 32;
constexpr uint16_t  CAB_ITEM_LOCO_NAME_LEN      = 16;
constexpr uint16_t  CAB_ITEM_TRAIN_NAME_LEN     = 16;

constexpr uint32_t  CAB_MAP_MAGIC_WORD          = 0xC0DECAFE;
constexpr uint16_t  CAB_MAP_ITEM_ID_START       = ITEM_ID_USER_START;
constexpr uint16_t  CAB_MAP_ITEM_ID_MAP_START   = CAB_MAP_ITEM_ID_START + 
                                                  CAB_ENTRY_ITEM_SIZE;

constexpr uint32_t  DCC_SESSION_TIMEOUT_MILLIS  = 2000;

//----------------------------------------------------------------------------------------
// The base station maintains a set of debug flags. The overall concept is very 
// similar to the LCS runtime library debug mask. The following debug flags are 
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
//----------------------------------------------------------------------------------------
enum LcsLocoSessionsDebugFlags : uint16_t {

    DBG_LS_CONFIG                  = 1 << 15,   

    DBG_LS_SESSION                 = 1 << 0,         
    DBG_LS_CHECK_ALIVE_SESSIONS    = 1 << 1,          
};

//----------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user 
// number range of errors defined in the LCS runtime library. 
//
//----------------------------------------------------------------------------------------
enum LcsLocoSessionErrors : uint8_t {

    LOCO_SESSIONS_ERR_BASE          = ERR_USER_SPECIFIC_BASE,
    ERR_SESSION_SETUP               = LOCO_SESSIONS_ERR_BASE + 0,
    ERR_NO_SVC_MODE                 = LOCO_SESSIONS_ERR_BASE + 1,
    ERR_CV_OP_FAILED                = LOCO_SESSIONS_ERR_BASE + 2,
    ERR_CAB_MAP_FULL                = LOCO_SESSIONS_ERR_BASE + 3
};

//----------------------------------------------------------------------------------------
// The session map options. These are options initially set when the base 
// station starts. They are used to set the flags, which are then used for
// processing the the actual settings. 
//
//  SM_KEEP_ALIVE_CHECKING  -   enable keep alive checking. When enabled, the 
//                              engine session need to receive a keep alive LCS 
//                              message periodically.
//                              
//  SM_ENABLE_REFRESH       -   refresh the session data. This will send the 
//                              engine speed and direction as well as the
//                              function flags periodically in a round robin 
//                              processing of the
//
//----------------------------------------------------------------------------------------
enum SessionMapOptions : uint16_t {

    SM_OPT_DEFAULT_SETTING      = 0,
    SM_OPT_KEEP_ALIVE_CHECKING  = 1 << 0,
    SM_OPT_ENABLE_REFRESH       = 1 << 1,
};

//----------------------------------------------------------------------------------------
// The session map flags. The apply to all sessions in the session map. The 
// initial values are copied from session option initial values.
//
//  SM_F_CAB_MAP_VALID        - the NVM cab map is a valid structure.
//
//  SM_F_KEEP_ALIVE_CHECKING  - enable keep alive checking. When enabled, the 
//                              engine session need to receive a keep alive LCS
//                              message periodically.
//
//  SM_F_ENABLE_REFRESH       - refresh the session data. This will send the 
//                              engine speed and direction as well as the 
//                              function flags periodically in a round robin 
//                              fashion.
//
//----------------------------------------------------------------------------------------
enum CabMapFlags : uint16_t {

    SM_F_NIL                    = 0,
    SM_F_CAB_MAP_VALID          = 1 << 0, 
    SM_F_KEEP_ALIVE_CHECKING    = 1 << 1,
    SM_F_ENABLE_REFRESH         = 1 << 2
};

//----------------------------------------------------------------------------------------
// Each cab map entry has a set of flags. 
//
//  CMAP_F_ACTIVE             - the cab is in the active list.
//
//  CMAP_F_COMBINED_REFRESH   - a DCC cab supports the combined refresh option.
//                              There is a DCC packet code for this. 
//
//  CMAP_F_SPDIR_REFRESH      - a DCC cab only refreshes speed/dir.
//
//  CMAP_F_FUNC_REFRESH       - a DCC cab also refreshes functions. 
// 
//----------------------------------------------------------------------------------------
enum CabMapEntryFlags : uint16_t {

    CMAP_F_NIL                      = 0,
    CMAP_F_ACTIVE                   = 1 << 0,
    CMAP_F_COMBINED_REFRESH         = 1 << 1,
    CMAP_F_SPDIR_REFRESH            = 1 << 2,
    SME_FUNC_REFRESH                = 1 << 3
};

//----------------------------------------------------------------------------------------
// Every locomotive know to the system has an entry in the cabMap. Loading the
// cabMap results in a sorted array of cabMap entries. Cab data is stored in a 
// 16-word entry. It contains the configured initial data for the cab. The 
// entries are accessed from the runtime NVM using the getAttr / setAttr 
// functions. Just like the header, we will read the array of cab entries at 
// module init and store it in a local data structure. To the outside world, 
// we present the items. 
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
//      2   :   speed info              :       :   speed info              : 
//          :---------------------------:       :---------------------------:
//      3   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      4   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      5   :                           :       :                           : 
//          :---------------------------:       :---------------------------:
//      6   :                           :       :                           : 
//          :===========================:       :===========================:
//      7   :   Speed map MIN,s1        :       :   Speed map MIN,s1        : 
//          :---------------------------:       :---------------------------:
//      8   :   Speed map s2, s3        :       :   Speed map s2, s3        : 
//          :---------------------------:       :---------------------------:
//      9   :   Speed map s4, MAX       :       :   Speed map s4, MAX       :   
//          :===========================:       :===========================:
//     10   :   DCC Info.               :       :   Analog Info             : 
//          :---------------------------:       :---------------------------:
//     11   :   DCC function map 0      :       :   PWM Frequency           : 
//          :---------------------------:       :---------------------------:
//     12   :   DCC function map 1      :       :   0                       : 
//          :---------------------------:       :---------------------------:
//     13   :   DCC function map 2      :       :   0                       : 
//          :---------------------------:       :---------------------------:
//     14   :   DCC function map 3      :       :   0                       : 
//          :---------------------------:       :---------------------------:
//     15   :   DCC function map 4      :       :   0                       : 
//          :---------------------------:       :---------------------------:
//     16   :                           :       :                           :
//          :   Loco Name               :       :   Loco Name               :
//          :                           :       :                           :
//          :---------------------------:       :---------------------------:
//     24   :                           :       :                           :
//          :   Train Name              :       :   Train Name              :
//          :                           :       :                           :
//          :---------------------------:       :---------------------------:
//
//  cabId           -   the cab assigned number. This is typically the DCC 
//                      address. Analog cabs also get a number, as well as 
//                      cab engine consists. A NIL cab entry is a a zero.
// 
//  flags           -   flags for the cab entry.
//
//  speedInfo       -   the speed and direction. 
//
//  speedMap        -   a set of 8 speed values.
//
//  DCC Info        -   DCC cab specific 
//
//  functions       -   the set of DCC function groups.
// 
//  Analog Info     -   analog cab specific 
//
//  PwmFrequency    -   the PWM option for the analog engine
//
//  Loco Name       -   a string of 15 characters plus a zero byte.
// 
//  Train Name       -   a string of 15 characters plus a zero byte.
// 
//----------------------------------------------------------------------------------------
struct LcsCabEntry {

    uint16_t    cabId;                                          // word 0 
    uint16_t    flags;                                          // word 1
    uint16_t    speedInfo;                                      // word 2
    uint16_t    rsv[ 4 ];                                       // word 3 - 5
    uint8_t     speedMap[ 6 ];                                  // word 7 - 9 

    union {

        struct {

            uint16_t    dFlags;                                 // word 10
            uint8_t     funcMap[ MAX_DCC_FUNC_GROUP_ID ];       // word 11 - 15

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

    char locoName[ CAB_ITEM_LOCO_NAME_LEN ];                    // word 16 - 23
    char trainName[ CAB_ITEM_TRAIN_NAME_LEN ];                  // word 24 - 31
};

//----------------------------------------------------------------------------------------
// The Cab Map data structure is a header and an array of cab entries. The
// header has a magic word, so we know that the data structure is valid. The
// data is stored in global items. On module initialization we just read the
// items in a local data structure. To the outside world, we present the items.
// We reserve the first items for the "header". The header has the same size
// as a cab entry.
//----------------------------------------------------------------------------------------
struct LcsCabMap {

    uint32_t        magicWord;                                  // item 0 - 1
    uint16_t        flags;                                      // item 2
    uint16_t        maxCabCount;                                // item 3
    uint16_t        currentCabCount;                            // item 4  
    uint16_t        cabEntryItemSize;                           // item 5   
    uint16_t        filler[ 26 ];                               // item 6 - 31

    LcsCabEntry     map[ MAX_CAB_DICT_SESSIONS ];               // item 31 - nn
};

//----------------------------------------------------------------------------------------
// Active cabs are maintained the active cab map. This a simple array of cabId
// and a timestamp when it was last refreshed. There is no order in the map,
// the active cab count is the high water mark. The DCC standard requires to 
// periodically refresh the DCC command. The refresh index is a round robin
// index that we use to do the refresh one locomotive at a time. We cannot 
// refresh all at once, since then the LCS system would be non-responsive for
// a considerable while.
//
//----------------------------------------------------------------------------------------
struct LcsActiveCabMap {

     uint16_t   activeCabCount;
     uint16_t   refreshIndex;
     int        map[ MAX_CAB_ACTIVE_SESSIONS ];
     uint32_t   tsMap[ MAX_CAB_ACTIVE_SESSIONS ];
};

//----------------------------------------------------------------------------------------
// The loco sessions object is the central data structure for the base station 
// loco management. The base station creates exactly one instance. The session
// object works closely with the DCC track objects.
//
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
    uint8_t             updateCabMap( );

    uint8_t             addCabEntry( LcsCabEntry *entry );
    uint8_t             removeCabEntry( uint16_t cabId );
    LcsCabEntry         *lookupCabEntry( uint16_t cabId );
    
    uint16_t            getOptions( );
    uint16_t            getFlags( );
    uint16_t            getCabCount( );
    uint16_t            getMaxCabCount( );

    void                emergencyStopAll( );

    uint8_t             setThrottle( uint16_t cabId, 
                                     uint8_t speed, 
                                     uint8_t direction );

    uint8_t             setThrottle( LcsCabEntry *cPtr, 
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

    uint8_t             verifyCV( uint16_t cvId, uint8_t mode, uint8_t val );
    uint8_t             verifyCVByte( uint16_t cvId, uint8_t val );
    uint8_t             verifyCVBit( uint16_t cvId, uint8_t bitPos, uint8_t val );

    uint8_t             refreshActiveSessions( );

    uint8_t             writeDccPacketMain( uint8_t *packet, uint8_t len );
    uint8_t             writeDccPacketProg( uint8_t *packet, uint8_t len );
    
    void                printCabMap( );
    void                printActiveCabMap( );
    void                printActiveMapCabEntry( LcsCabEntry *aPtr, bool detail = false );
    void                printCabMapEntry( LcsCabEntry *cPtr, bool detail = false );
    
    private:

    uint16_t            debugMask               = 0;
    uint16_t            options                 = DT_OPT_NIL;
    uint16_t            flags                   = DT_F_NIL;

    LcsDccTrack         *mainTrack              = nullptr;
    LcsDccTrack         *progTrack              = nullptr;

    LcsCabMap           cabMap;
    LcsActiveCabMap     activeCabMap;

    LcsCabEntry         *activateCabEntry( uint16_t cabId );
    void                deactivateCabEntry( uint16_t cabId );

    void                addActiveCab( uint16_t locoIndex );
    void                removeActiveCab( uint16_t locoIndex );
    void                rebuildActiveCabMap( );
    void                refreshActiveCabs( );
    void                refreshActiveCabEntry( LcsCabEntry *cab );
};
