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
#ifndef LcsBaseStation_h
#define LcsBaseStation_h

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

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

    DBG_BS_CONFIG                  = 1 << 15,   

    DBG_BS_SESSION                 = 1 << 0,    
    DBG_BS_LCS_MSG_INTERFACE       = 1 << 1,    
    DBG_BS_TRACK_POWER_MGMT        = 1 << 2,        
    DBG_BS_DCC_ACK_DETECT          = 1 << 3,        
    DBG_BS_CHECK_ALIVE_SESSIONS    = 1 << 4,        
    DBG_BS_RAILCOM                 = 1 << 5   
};

//----------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user 
// number range of errors defined in the LCS runtime library. The first 128 error 
// codes are reserved for the LCS library.
//
//----------------------------------------------------------------------------------------
enum BaseStationErrors : uint8_t {

    BASE_STATION_ERR_BASE             = ERR_USER_SPECIFIC_BASE,

    ERR_NO_SVC_MODE                   = BASE_STATION_ERR_BASE + 1,
    ERR_CV_OP_FAILED                  = BASE_STATION_ERR_BASE + 2,

    ERR_LOCO_NOT_FOUND                = BASE_STATION_ERR_BASE + 4,
    ERR_SESSION_NOT_FOUND             = BASE_STATION_ERR_BASE + 5,
    ERR_LOCO_SESSION_ALLOCATE         = BASE_STATION_ERR_BASE + 6,
    ERR_LOCO_SESSION_CANCELLED        = BASE_STATION_ERR_BASE + 7,

    ERR_SESSION_SETUP                 = BASE_STATION_ERR_BASE + 9,
    ERR_MSG_INTERFACE_SETUP           = BASE_STATION_ERR_BASE + 10,
    ERR_DCC_TRACK_CONFIG              = BASE_STATION_ERR_BASE + 11,
    ERR_DCC_PIN_CONFIG                = BASE_STATION_ERR_BASE + 12,

    ERR_NVM_HW_SETUP                  = BASE_STATION_ERR_BASE + 15,
    ERR_PIO_HW_SETUP                  = BASE_STATION_ERR_BASE + 16
};

//----------------------------------------------------------------------------------------
// DCC packet definition. A DCC packet is the payload data without the checksum. 
// Besides the length in bytes and the buffer, there is a repeat counter to specify
// how often this packet will be repeatedly transmitted after the first transmission.
// Currently, a DCC packet is at most 15 bytes long, excluding the checksum byte. 
// This is true for XPOM and DCC-A support, otherwise it is historically a maximum 
// of 6 bytes.
//
//----------------------------------------------------------------------------------------
const uint8_t DCC_PACKET_SIZE = 16;

struct DccPacket {

    uint8_t len;
    uint8_t repeat;
    uint8_t buf[ DCC_PACKET_SIZE ];
};

//----------------------------------------------------------------------------------------
// DCC packet payload data definitions we need often, so these constants come in 
// handy.
//
//----------------------------------------------------------------------------------------
const uint8_t   idleDccPacketData[ ]    = { 0xFF, 0x00 };
const uint8_t   resetDccPacketData[ ]   = { 0x00, 0x00 };
const uint8_t   eStopDccPacketData[ ]   = { 0x00, 0x01 };

//----------------------------------------------------------------------------------------
// Setup options to set for the DCC track. They are set when the track object is
// created.
//
//  DT_OPT_SERVICE_MODE_TRACK  - The track is a DCC PROG track.
//  DT_OPT_CUTOUT              - The track is configured to emit a cutout.
//  DT_OPT_RAILCOM             - The track support Railcom detection.
//
//----------------------------------------------------------------------------------------
enum DccTrackOptions : uint16_t {

    DT_OPT_NIL      = 0,
    DT_OPT_SERVICE_MODE_TRACK   = 1 << 0,
    DT_OPT_CUTOUT               = 1 << 1,
    DT_OPT_RAILCOM              = 1 << 2
};

//----------------------------------------------------------------------------------------
// The DCC track object has a set of flags to indicate its current status.
//
//  DT_F_POWER_ON             - The track is under power.
//  DT_F_POWER_OVERLOAD       - An overload situation was detected.
//  DT_F_MEASUREMENT_ON       - The power measurement is enabled.
//  DT_F_SERVICE_MODE_ON      - The track is currently in service mode.
//  DT_F_CUTOUT_MODE_ON       - The track has the cutout generation enabled.
//  DT_F_RAILCOM_MODE_ON      - The track has the railcom detect enabled.
//  DT_F_RAILCOM_MSG_PENDING  - A Railcom received datagram is indicated.
//  DT_F_CONFIG_ERROR         - The configuration descriptor has invalid options.
//
//----------------------------------------------------------------------------------------
enum DccTrackFlags : uint16_t {

    DT_F_NIL      = 0,
    DT_F_POWER_ON             = 1 << 0,
    DT_F_POWER_OVERLOAD       = 1 << 1,
    DT_F_MEASUREMENT_ON       = 1 << 2,
    DT_F_SERVICE_MODE_ON      = 1 << 3,
    DT_F_CUTOUT_MODE_ON       = 1 << 4,
    DT_F_RAILCOM_MODE_ON      = 1 << 5,
    DT_F_DCC_PACKET_PENDING   = 1 << 6,
    DT_F_RAILCOM_MSG_PENDING  = 1 << 7,
    DT_F_CONFIG_ERROR         = 1 << 15
};

//----------------------------------------------------------------------------------------
// The following constants are for the current consumption RMS measurement. The 
// idea is to record the measured ADC values in a circular buffer, every time a 
// certain amount of milliseconds has passed. This work is done by the DCC track 
// state machine as part of the power on state.
//
//----------------------------------------------------------------------------------------
const uint8_t   PWR_SAMPLE_BUF_SIZE             = 64;
const uint32_t  PWR_SAMPLE_TIME_INTERVAL_MILLIS = 16;

//----------------------------------------------------------------------------------------
// The RailCom buffer size. During the cutout period up to eight bytes of raw data
// are sent by the decoder if the Railcom option is enabled.
//
//----------------------------------------------------------------------------------------
const uint8_t   RAILCOM_BUF_SIZE = 8;

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
// Timeout intervals for various base station tasks. Measured in milliseconds.
//
//----------------------------------------------------------------------------------------
const uint32_t  MAIN_TRACK_STATE_TIME_INTERVAL  = 10;
const uint32_t  PROG_TRACK_STATE_TIME_INTERVAL  = 10;
const uint32_t  SESSION_REFRESH_TASK_INTERVAL   = 50;

//----------------------------------------------------------------------------------------
// For creating the DCC track object, the track is described by the data structure 
// below. In addition to the hardware resources, there are the limits for current 
// consumption values, all specified in milliAmps. The initial current sets the 
// current consumption limit after the track is turned on. The limit current 
// consumption specifies the actual configured value that is checked for a track 
// current overload situation. The maximum current defines what current the power 
// module should never exceed. For the measurements to work, the power module needs
// to deliver a voltage that corresponds to the current drawn on the track. The 
// value is measured in milliVolt per Ampere drawn. Finally, there are threshold 
// times for managing the track overload and restart capability.
//
//----------------------------------------------------------------------------------------
struct LcsDccTrackDesc {

    uint16_t    options                         = SM_OPT_DEFAULT_SETTING;

    uint8_t     rNumEnable                      = 0; 
    uint8_t     rNumControl                     = 0;
    uint8_t     rNumSense                       = 0;
    uint8_t     rNumUartRx                      = 0;

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
// DCC track definition. The DCC track object is responsible for managing track 
// power as well as building and sending the DCC packet bit stream. It is the 
// heart of the DCC signal generation. 
//
// A DCC packet consists of the preamble bits, the postamble bit, the data bytes
// separated with a ZERO bit, followed by the checksum byte. The DCC track object
// builds the packet bit stream from the DCC packet data structure and then sends
// it to the track. Sending the bit stream is done with a state machine that is
// invoked every 29 microseconds via a hardware timer interrupt. This state machine
// creates the actual DCC signal on the track output pin.
//
// The DCC track object also manages the repeated sending of DCC packets. When a
// packet is loaded into the DCC track object, it can be specified how often the 
// packet is to be repeated. The DCC track object will then take care of sending
// the packet the specified number of times.
//
// The DCC signal state machine finally invokes follow up actions that measure the 
// actual power consumption, read in a railcom message and so on. There is also 
// a DCC log facility which records internal events for testing and debugging. 
// 
// The other state machine will manage the actual track power. This state machine 
// is responsible for the periodic checking of power consumption and resulting power 
// control. In contrast to the DCC signal state machine, this machine is not driven
// by a periodic interrupt but invoked periodically via the LCS runtime task manager.
//
// For a base station, there will be two track objects. One is the MAIN track and 
// the other one is the PROG track. Each track has a DCC track object associated 
// with it. In addition to the two track objects, there are class level static 
// routines to manage the timer hardware functions, the analog signal read for 
// current measurement and the serial IO for optional RailCom message processing. 
//
//----------------------------------------------------------------------------------------
struct LcsDccTrack {

    public:

    LcsDccTrack( );

    uint8_t             setupDccTrack( LcsDccTrackDesc* trackDesc );
    void                loadPacket( const uint8_t *packet, 
                                    uint8_t len, 
                                    uint8_t repeat = 0 );

    uint16_t            getFlags( );
    uint16_t            getOptions( );

    bool                isServiceModeOn( );
    void                serviceModeOn( );
    void                serviceModeOff( );

    void                runDccTrackStateMachine( );
    void                powerStart( );
    void                powerStop( );
    bool                isPowerOn( );
    bool                isPowerOverload( );

    void                cutoutOn( );
    void                cutoutOff( );
    bool                isCutoutOn( );

    void                railComOn( );
    void                railComOff( );
    bool                isRailComOn( );

    void                setLimitCurrent( uint16_t val );
    uint16_t            getLimitCurrent( );
    uint16_t            getActualCurrent( );
    uint16_t            getInitCurrent( );
    uint16_t            getMaxCurrent( );
    uint16_t            getRMSCurrent( );

    uint16_t            decoderAckBaseline( uint8_t resetPacketsToSend );
    bool                decoderAckDetect( uint16_t baseValue, uint8_t retries );
    void                checkOverload( );

    void                runDccSignalStateMachine( 
                            volatile uint8_t *timeToInterrupt, 
                            uint8_t *followUpAction 
                        );

    void                getNextBit( );
    void                getNextPacket( );
    void                powerMeasurement( );

    void                startRailComIO( );
    void                stopRailComIO( );
    uint8_t             handleRailComMsg( );
    uint8_t             getRailComMsg( uint8_t *buf, uint8_t bufLen );

    uint32_t            getDccPacketsSend( );
    uint32_t            getPwrSamplesTaken( );
    uint16_t            getPwrSamplesPerSec( );

    void                printDccTrackConfig( );
    void                printDccTrackStatus( );

    void                enableLog( bool arg );
    void                beginLog( );
    void                endLog( );
    void                printLog( );

    void                writeLogData( uint8_t id, uint8_t *buf, uint8_t len );
    void                writeLogId( uint8_t id );
    void                writeLogTs( );
    void                writeLogVal( uint8_t valId, uint16_t val );

    private:

    uint16_t            options                         = DT_OPT_NIL;
    volatile uint16_t   flags                           = DT_F_NIL;

    volatile uint8_t    trackState                      = 0;
    volatile uint8_t    signalState                     = 0;

    volatile uint32_t   trackTimeStamp                  = 0;
    volatile uint8_t    overloadEventCount              = 0;
    volatile uint8_t    overloadRestartCount            = 0;

    uint8_t             rNumEnable                      = 0;
    uint8_t             rNumControl                     = 0;
    uint8_t             rNumSense                       = 0;
    uint8_t             rNumUartRx                      = 0;

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
    uint16_t            ackThresholdDigitValue          = 0;

    uint32_t            totalPwrSamplesTaken            = 0;
    uint32_t            lastPwrSampleTimeStamp          = 0;

    uint32_t            lastPwrSamplePerSecTaken        = 0;
    uint32_t            lastPwrSamplePerSecTimeStamp    = 0;
    uint32_t            pwrSamplesPerSec                = 0;

    uint8_t             preambleLen                     = 0;
    uint8_t             postambleLen                    = 0;
    volatile bool       currentBit                      = false;
    volatile uint8_t    bytesSent                       = 0;
    volatile uint8_t    bitsSent                        = 0;
    volatile uint8_t    preambleSent                    = 0;
    volatile uint8_t    postambleSent                   = 0;
    uint32_t            dccPacketsSend                  = 0;

    DccPacket           dccBuf1                         = { 0 }; 
    DccPacket           dccBuf2                         = { 0 };
    DccPacket           *activeBufPtr                   = nullptr;
    DccPacket           *pendingBufPtr                  = nullptr;

    // ??? add base station capabilities according to RCN200 - 4 16 bit words
    // sample values per second for samples and dcc packets
    // ??? add buffers for POM / XPOM data
    // ??? add queue for POM / XPOM commands

    uint8_t             railComBufIndex                     = 0;
    uint8_t             railComMsgBuf[ RAILCOM_BUF_SIZE ]   = { 0 };

    uint8_t             pwrSampleBufIndex                    = 0;
    uint16_t            pwrSampleBuf[ PWR_SAMPLE_BUF_SIZE ]  = { 0 };

    public:

    static void         startDccProcessing( );
};


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
// We need also a way of mapping a cabId to a sessionId. The structure below 
// defines an entry in the mapping table and the mapping table itself.
//
// ??? this should go local to session file ?
//----------------------------------------------------------------------------------------
struct LcsCabSessionEntry {

    uint16_t cabId;  
    uint8_t  sessionId;   
};

struct LcsCabSessionMapTable {

    LcsCabSessionEntry map[ MAX_CAB_SESSIONS ];
    uint16_t           hwm;

};




//----------------------------------------------------------------------------------------
// Every allocated loco session is described by the sessionMap structure. There 
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
struct SessionMapEntry {

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
// The loco session object is the central data structure for the base station 
// engine management. For a DCC type engine it manages the loco sessions and 
// assembles the DCC packets and drives the DCC track objects to send out the 
// relevant DCC packages. For an analog engine it will just manage the session 
// entry and communicate via the LCS bus with the block controller that actually 
// owns the engine at the moment.
//
// 
// ??? some rework... 
//----------------------------------------------------------------------------------------
struct LcsBaseStationLocoSession {

  public:

    LcsBaseStationLocoSession( );

    uint8_t setupSessionMap(

        LcsBaseStationSessionMapDesc  *sessionMapDesc,
        LcsDccTrack        *mainTrack,
        LcsDccTrack        *progTrack
    );

    uint8_t                   requestSession( uint16_t cabId, 
                                              uint8_t mode, 
                                              uint8_t *sId );

    uint8_t                   releaseSession( uint8_t sId );
    uint8_t                   updateSession( uint8_t sId, uint8_t flags );

    uint8_t                   markSessionAlive( uint8_t sId );
    void                      refreshActiveSessions( );
    uint32_t                  getSessionKeepAliveInterval( );

    uint16_t                  getOptions( );
    uint16_t                  getFlags( );
    uint8_t                   getSessionMapHwm( );
    uint8_t                   getActiveSessions( );
    uint8_t                   getSessionIdByCabId( uint16_t cabId );
    void                      emergencyStopAll( );

    uint8_t                   setThrottle( uint8_t sId, 
                                           uint8_t speed, 
                                           uint8_t direction );

    uint8_t                   setDccFunctionBit( uint8_t sId, 
                                                 uint8_t funcNum, 
                                                 uint8_t val );

    uint8_t                   setDccFunctionGroup( uint8_t sId, 
                                                   uint8_t fGroup, 
                                                   uint8_t dccByte );

    uint8_t                   writeCVMain( uint8_t sId, 
                                           uint16_t cvId, 
                                           uint8_t mode, 
                                           uint8_t val );

    uint8_t                   writeCVByteMain( uint8_t sId, 
                                               uint16_t cvId,
                                               uint8_t val );

    uint8_t                   writeCVBitMain( uint8_t sId, 
                                              uint16_t cvId, 
                                              uint8_t bitPos, 
                                              uint8_t val );

    uint8_t                   readCV( uint16_t cvId, uint8_t mode, uint8_t *val );
    uint8_t                   readCVByte( uint16_t cvId, uint8_t *val );
    uint8_t                   readCVBit( uint16_t cvId, uint8_t bitPos, uint8_t *val );

    uint8_t                   writeCV( uint16_t cvId, uint8_t mode, uint8_t val );
    uint8_t                   writeCVByte( uint16_t cvId, uint8_t val );
    uint8_t                   writeCVBit( uint16_t cvId, uint8_t bitPos, uint8_t val );

    uint8_t                   writeDccPacketMain( uint8_t *buf,  
                                                  uint8_t len, 
                                                  uint8_t nRepeat );

    uint8_t                   writeDccPacketProg( uint8_t *buf,  
                                                  uint8_t len, 
                                                  uint8_t nRepeat );

    void                      printSessionMapConfig( );
    void                      printSessionMapInfo( );

    SessionMapEntry           *lookupSessionEntry( uint16_t cabId );
    SessionMapEntry           *getSessionMapEntryPtr( uint8_t sId );

    private:

    uint8_t                   setThrottle(  SessionMapEntry *csptr, 
                                            uint8_t speed, 
                                            uint8_t direction );

    uint8_t                   setDccFunctionGroup( SessionMapEntry *csPtr, 
                                                   uint8_t fGroup, 
                                                   uint8_t dccByte );

    SessionMapEntry           *allocateSessionEntry( uint16_t cabId );
    void                      deallocateSessionEntry( SessionMapEntry *csPtr );
    void                      refreshSessionEntry( SessionMapEntry *csPtr );
    void                      initSessionEntry( SessionMapEntry *csPtr );
    void                      printSessionEntry( SessionMapEntry *csPtr );

    private:

    LcsDccTrack    *mainTrack              = nullptr;
    LcsDccTrack    *progTrack              = nullptr;

    uint16_t                  options                 = DT_OPT_NIL;
    uint16_t                  flags                   = DT_F_NIL;
    uint32_t                  lastAliveCheckTime      = 0L;
    uint32_t                  refreshAliveTimeOutVal  = DCC_SESSION_TIMEOUT_MILLIS;

    // ??? rework this... 

    // ??? add hash table... ?
    // ??? HWM a simple int ?
    // ??? next refresh a simple int ?
    // ??? limit is a constant ?

    SessionMapEntry           *sessionMap             = nullptr;
    SessionMapEntry           *sessionMapNextRefresh  = nullptr;
    SessionMapEntry           *sessionMapHwm          = nullptr;
    SessionMapEntry           *sessionMapLimit        = nullptr; // ??? goes away...
};

//----------------------------------------------------------------------------------------
// One of the key duties of the base station is to listen and react to DCC commands
// coming via the LCS bus. The interface works very closely with the loco session 
// management and the two DCC track objects.
//
//----------------------------------------------------------------------------------------
struct LcsBaseStationMsgInterface {

    public:

    LcsBaseStationMsgInterface( );

    uint8_t setupLcsMsgInterface( LcsBaseStationLocoSession   *locoSessions,
                                  LcsDccTrack      *mainTrack,
                                  LcsDccTrack      *progTrack
                                );

    void handleLcsMsg( uint8_t *msg );

    private:

    LcsBaseStationLocoSession   *locoSessions   = nullptr;
    LcsDccTrack      *mainTrack      = nullptr;
    LcsDccTrack      *progTrack      = nullptr;

};

//----------------------------------------------------------------------------------------
// The base station implements a serial IO command interface. The command interface
// uses the classic DCC++ syntax ( updated according to the DCC-EX syntax ) of a 
// command line and where it is a original DCC++ command it implements them in a 
// compatible way. The idea is to one day connect to the programs of the JMRI world,
// which support the DCC++ style command interface.
//
//----------------------------------------------------------------------------------------
struct LcsBaseStationCommand {

    public:

    LcsBaseStationCommand( );

    uint8_t setupSerialCommand( LcsBaseStationLocoSession  *locoSessions,
                                LcsDccTrack     *mainTrack,
                                LcsDccTrack     *progTrack );

    void handleSerialCommand( char *s );

    private:

    void openSessionCmd( char *s );
    void closeSessionCmd( char *s );

    void setThrottleCmd( char *s );
    void setFunctionBitCmd( char *s );
    void setFunctionGroupCmd( char *s );
    void emergencyStopCmd( );

    void readCVCmd( char *s );
    void writeCVByteCmd( char *s );
    void writeCVBitCmd( char *s );
    void writeCVByteMainCmd( char *s );
    void writeCVBitMainCmd( char *s );

    void writeDccPacketMainCmd( char *s );
    void writeDccPacketProgCmd( char *s );

    void setTrackOptionCmd( char *s );
    void turnPowerOnAllCmd( );
    void turnPowerOnMainCmd( );
    void turnPowerOnProgCmd( );
    void turnPowerOffAllCmd( );

    void printStatusCmd( char *s );
    void printTrackCurrentCmd( char *s );
    void printBaseStationConfigCmd( );
    void printHelpCmd( );
    void printVersionInfo( );
    void printConfiguration( );
    void printSessionMap( );
    void printTrackStatusMain( );
    void printTrackStatusProg( );

    void printDccLogCommand( char *s ); 

    private:

    LcsBaseStationLocoSession *locoSessions = nullptr;
    LcsDccTrack    *mainTrack    = nullptr;
    LcsDccTrack    *progTrack    = nullptr;
};

#endif
