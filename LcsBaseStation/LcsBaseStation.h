//------------------------------------------------------------------------------------------------------------
//
// LCS Base Station - Include file
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
// Copyright (C) 2019 - 2024  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
// Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along with this program. If not, see
// http://www.gnu.org/licenses
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//------------------------------------------------------------------------------------------------------------
#ifndef LcsBaseStation_h
#define LcsBaseStation_h

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//------------------------------------------------------------------------------------------------------------
// The base station maintains a set of debug flags. The overall concept is very similar to the LCS runtime
// library debug mask. Then following debug flags are defined:
//
//      DBG_BS_CONFIG                   -   DEBUG base station enabled
//      DBG_BS_SESSION                  -   show the session management actions
//      DBG_BS_LCS_MSG_INTERFACE        -   show the incoming LCS messages
//      DBG_BS_TRACK_POWER_MGMT         -   show the track power measurement data
//      DBG_BS_DCC_ACK_DETECT           -   display decoder ACK power measurements
//      DBG_BS_CHECK_ALIVE_SESSIONS     -   displays that a session seems no longer be alive
//      DBG_BS_RAILCOM                  -   show the RailCom activity
//
// The way to use these flags is for example:
//
//      if (( debugMask & DBG_BS_CONFIG ) && ( debugMask & DBG_BS_SESSION )) 
//
// ??? should have a command to set the debug mask on the fly...
//------------------------------------------------------------------------------------------------------------
#define DEBUG_SESSION                   1 // show the session management actions ...
#define DEBUG_LCS_MSG_INTERFACE         1 // show the incoming LCS messages...
#define DEBUG_CURRENT_MEASUREMENT       1 // show the actual measurement data ...
#define DEBUG_DCC_ACK_DETECT            1 // display decoder ACK power measurements ...
#define DEBUG_RAILCOM                   0 // show the RailCom activity
#define DEBUG_CHECK_ALIVE_SESSIONS      0 // displays that a session seems no longer be alive ...
#define DEBUG_WAVE_FORM                 0 // slows down the waveform timing so you can see it blinking ...

enum BaseStationDebugFlags : uint16_t {

    DBG_CONFIG                     = 1 << 15,       // DEBUG base station enabled

    DBG_BS_SESSION                 = 1 << 0,        // show the session management actions
    DBG_BS_LCS_MSG_INTERFACE       = 1 << 1,        // show the incoming LCS messages
    DBG_BS_TRACK_POWER_MGMT        = 1 << 2,        // show the track power measurement data
    DBG_BS_DCC_ACK_DETECT          = 1 << 3,        // display decoder ACK power measurements
    DBG_BS_CHECK_ALIVE_SESSIONS    = 1 << 4,        // displays that a session seems no longer be alive
    DBG_BS_RAILCOM                 = 1 << 5         // show the RailCom activity

};

//------------------------------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user number range of errors defined 
// in the LCS runtime library. 
//
//------------------------------------------------------------------------------------------------------------
enum BaseStationErrors : uint8_t {

    BASE_STATION_ERR_BASE             = 128,

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

//------------------------------------------------------------------------------------------------------------
// DCC packet definition. A DCC packet is the payload data without the checksum. Besides the length in bytes
// and the buffer, there is a repeat counter to specify how often this packet will be repeatedly transmitted
// after the first transmission. Currently, a DCC packet is at most 15 bytes long, excluding the checksum 
// byte. This is true for XPOM and DCC-A support, otherwise it is historically a maximum of 6 bytes.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t DCC_PACKET_SIZE = 16;

struct DccPacket {

    uint8_t len;
    uint8_t repeat;
    uint8_t buf[ DCC_PACKET_SIZE ];
};

//----------------------------------------------------------------------------------------------------------
// DCC packet payload data definitions we need often, so these constants come in handy.
//
//----------------------------------------------------------------------------------------------------------
const uint8_t   idleDccPacketData[ ]    = { 0xFF, 0x00 };
const uint8_t   resetDccPacketData[ ]   = { 0x00, 0x00 };
const uint8_t   eStopDccPacketData[ ]   = { 0x00, 0x01 };

//------------------------------------------------------------------------------------------------------------
// Setup options to set for the DCC track. They are set when the track object is created.
//
//  DT_OPT_SERVICE_MODE_TRACK  - The track is a PROG track.
//  DT_OPT_CUTOUT              - The track is configured to emit a cutout during the DCC packet preamble.
//  DT_OPT_RAILCOM             - The track support Railcom detection.
//
//------------------------------------------------------------------------------------------------------------
enum DccTrackOptions : uint16_t {

    DT_OPT_DEFAULT_SETTING      = 0,
    DT_OPT_SERVICE_MODE_TRACK   = 1 << 0,
    DT_OPT_CUTOUT               = 1 << 1,
    DT_OPT_RAILCOM              = 1 << 1
};

//------------------------------------------------------------------------------------------------------------
// The DCC track object has a set of flags to indicate its current status.
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
//------------------------------------------------------------------------------------------------------------
enum DccTrackFlags : uint16_t {

    DT_F_DEFAULT_SETTING      = 0,
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

//------------------------------------------------------------------------------------------------------------
// The following constants are for the current consumption RMS measurement. The idea is to record the measured
// ADC values in a circular buffer, every time a certain amount of milliseconds has passed. This work is done
// by the DCC track state machine as part of the power on state.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t   PWR_SAMPLE_BUF_SIZE               = 64;
const uint32_t  PWR_SAMPLE_TIME_INTERVAL_MILLIS   = 16;

//------------------------------------------------------------------------------------------------------------
// The RailCom buffer size. During the cutout period up to eight bytes of raw data are sent by the decoder if
// the Railcom option is enabled.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t   RAILCOM_BUF_SIZE = 8;

//------------------------------------------------------------------------------------------------------------
// The session map options. These are options initially set when the base station starts. They are used to 
// set the flags, which are then used for processing the the actual settings. 
//
//  SM_KEEP_ALIVE_CHECKING  - enable keep alive checking. When enabled, the locomotive session need to receive
//                            a keep alive LCS message periodically.
//  SM_ENABLE_REFRESH       - refresh the session data. This will send the locomotive speed and direction as
//                            well as the function flags periodically in a round robin processing of the
//
//------------------------------------------------------------------------------------------------------------
enum SessionMapOptions : uint16_t {

    SM_OPT_DEFAULT_SETTING        = 0,
    SM_OPT_KEEP_ALIVE_CHECKING    = 1 << 0,
    SM_OPT_ENABLE_REFRESH         = 1 << 1
};

//------------------------------------------------------------------------------------------------------------
// The session map flags. The apply to all sessions in the session map. The initial values are copied from 
// session option initial values.
//
//  SM_F_KEEP_ALIVE_CHECKING  - enable keep alive checking. When enabled, the locomotive session need to receive
//                            a keep alive LCS message periodically.
//  SM_F_ENABLE_REFRESH       - refresh the session data. This will send the locomotive speed and direction as
//                            well as the function flags periodically in a round robin processing of the
//
//------------------------------------------------------------------------------------------------------------
enum SessionMapFlags : uint16_t {

    SM_F_DEFAULT_SETTING        = 0,
    SM_F_KEEP_ALIVE_CHECKING    = 1 << 0,
    SM_F_ENABLE_REFRESH         = 1 << 1
};

//------------------------------------------------------------------------------------------------------------
// Each session map entry has a set of flags.
//
//  SME_ALLOCATED           - the session is allocated, the entry valid.
//  SME_COMBINED_REFRESH    - locomotive speed/dir and functions are refreshed using the combined DCC packet.
//  SME_SPDIR_REFRESH       - locomotive speed/dir are refreshed.
//  SME_FUNC_REFRESH        - locomotive functions are refreshed. 
//  SME_DISPATCHED          -
//  SME_SHARED              -
// 
//
// ??? when the base station has a config value of using the DCC spdir/func command, these flags need to be
// named slightly different. Should we still have the option to enable or disable it even though the base
// station can do it ? A decoder might not support this packet type...
//------------------------------------------------------------------------------------------------------------
enum SessionMapEntryFlags : uint16_t {

    SME_DEFAULT_SETTING     = 0,
    SME_ALLOCATED           = 1 << 0,
    SME_COMBINED_REFRESH    = 1 << 1,
    SME_SPDIR_ONLY_REFRESH  = 1 << 2, // ??? phase out...
    SME_SPDIR_REFRESH       = 1 << 2,
    SME_FUNC_REFRESH        = 1 << 3,
    SME_DISPATCHED          = 1 << 4,
    SME_SHARED              = 1 << 5
};

//------------------------------------------------------------------------------------------------------------
// The base station items for nodeInfo and nodeControl calls .... tbd
//
// ??? the are mapped in the MEM / NVM range as well as in the USER range.
// ??? how to do it consistently and understandably ?
//------------------------------------------------------------------------------------------------------------
enum BaseStationInfoItems : uint8_t {

    // or use GET in all constants

    BS_ITEM_SESSION_MAP_OPTIONS   = 128,
    BS_ITEM_SESSION_MAP_FLAGS     = 129,
    BS_ITEM_MAX_SESSIONS          = 130,
    BS_ITEM_ACTIVE_SESSIONS       = 131,

    BS_ITEM_INIT_CURRENT_VAL      = 140,
    BS_ITEM_LIMIT_CURRENT_VAL     = 140,
    BS_ITEM_MAX_CURRENT_VAL       = 140,
    BS_ITEM_ACTUAL_CURRENT_VAL     = 140,

    // thresholds

    // eventID to send for events ?

};

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
const uint32_t  MAIN_TRACK_STATE_TIME_INTERVAL  = 10;
const uint32_t  PROG_TRACK_STATE_TIME_INTERVAL  = 10;
const uint32_t  SESSION_REFRESH_TASK_INTERVAL   = 50;

const uint16_t  MAX_CAB_SESSIONS                = 64;

//------------------------------------------------------------------------------------------------------------
// For creating the Loco Session object the session map object is described by the following descriptor.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBaseStationSessionMapDesc {

    uint16_t    options       = SM_OPT_DEFAULT_SETTING;
    uint16_t    maxSessions   = MAX_CAB_SESSIONS;
};

//------------------------------------------------------------------------------------------------------------
// For creating the DCC track object, the track is described by the data structure below. In addition to the
// hardware pins enablePin, dcc1Pin1, dccPin2 and sensePin, there are the limits for current consumption
// values, all specified in milliAmps. The initial current sets the current consumption limit after the track
// is turned on. The limit current consumption specifies the actual configured value that is checked for a
// track current overload situation. The maximum current defines what current the power module should never
// exceed. For the measurements to work, the power module needs to deliver a voltage that corresponds to the
// current drawn on the track. The value is measured in milliVolt per Ampere drawn. Finally, there are
// threshold times for managing the track overload and restart capability.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBaseStationTrackDesc {

    uint16_t  options                       = SM_OPT_DEFAULT_SETTING;

    uint8_t   enablePin                     = CDC::UNDEFINED_PIN;
    uint8_t   dccSigPin1                    = CDC::UNDEFINED_PIN;
    uint8_t   dccSigPin2                    = CDC::UNDEFINED_PIN;
    uint8_t   sensePin                      = CDC::UNDEFINED_PIN;
    uint8_t   uartRxPin                     = CDC::UNDEFINED_PIN;

    uint16_t  initCurrentMilliAmp           = 0;
    uint16_t  limitCurrentMilliAmp          = 0;
    uint16_t  maxCurrentMilliAmp            = 0;
    uint16_t  milliVoltPerAmp               = 0;

    uint16_t  startTimeThresholdMillis      = 0;
    uint16_t  stopTimeThresholdMillis       = 0;
    uint16_t  overloadTimeThresholdMillis   = 0;
    uint16_t  overloadEventThreshold        = 0;
    uint16_t  overloadRestartThreshold      = 0;
};

//------------------------------------------------------------------------------------------------------------
// DCC track definition. The DCC track object is responsible for managing the track power as well as building
// and sending the DCC packet bit stream. A packet consists of the preamble bits, the postamble bit, the data
// bytes separated with a ZERO bit and a checksum byte. Creating the DCC bit stream is done with the signal
// generation routines. The signal state machine, running on a 29 microsecond tick, takes a DCC packet and
// gets it out to the track. The DCC signal state machine also invokes follow up actions that measure the 
// actual power consumption, read in a railcom message and so on. There is also a DCC log facility which 
// records internal events for testing and debugging.
//
// The other state machine will manage the actual track power. This machine is responsible for the periodic 
// checking of power consumption and resulting power control. In contrast to the DCC signal state machine,
// this machine is not driven by a periodic interrupt but invoked periodically via the LCS runtime task 
// manager.
//
// For a base station, there will be two track objects. One is the MAIN track and the other one is the PROG
// track. Each track has a DCC track object associated with it. In addition to the two track objects, there
// are class level static routines to manage the timer hardware functions, the analog signal read for current
// measurement and the serial IO for the optional RailCom message processing. The current version is AtMega
// specific.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBaseStationDccTrack {

    public:

    LcsBaseStationDccTrack( );

    uint8_t                     setupDccTrack( LcsBaseStationTrackDesc* trackDesc );
    void                        loadPacket( const uint8_t *packet, uint8_t len, uint8_t repeat = 0 );

    uint16_t                    getFlags( );
    uint16_t                    getOptions( );

    bool                        isServiceModeOn( );
    void                        serviceModeOn( );
    void                        serviceModeOff( );

    void                        runDccTrackStateMachine( );
    void                        powerStart( );
    void                        powerStop( );
    bool                        isPowerOn( );
    bool                        isPowerOverload( );

    void                        cutoutOn( );
    void                        cutoutOff( );
    bool                        isCutoutOn( );

    void                        railComOn( );
    void                        railComOff( );
    bool                        isRailComOn( );

    void                        setLimitCurrent( uint16_t val );
    uint16_t                    getLimitCurrent( );
    uint16_t                    getActualCurrent( );
    uint16_t                    getInitCurrent( );
    uint16_t                    getMaxCurrent( );
    uint16_t                    getRMSCurrent( );

    uint16_t                    decoderAckBaseline( uint8_t resetPacketsToSend );
    bool                        decoderAckDetect( uint16_t baseValue, uint8_t retries );
    void                        checkOverload( );

    void                        runDccSignalStateMachine( volatile uint8_t *timeToInterrupt, uint8_t *followUpAction );

    void                        getNextBit( );
    void                        getNextPacket( );
    void                        powerMeasurement( );

    void                        startRailComIO( );
    void                        stopRailComIO( );
    uint8_t                     handleRailComMsg( );
    uint8_t                     getRailComMsg( uint8_t *buf, uint8_t bufLen );

    uint32_t                    getDccPacketsSend( );
    uint32_t                    getPwrSamplesTaken( );
    uint16_t                    getPwrSamplesPerSec( );

    void                        printDccTrackConfig( );
    void                        printDccTrackStatus( );

    void                        enableLog( bool arg );
    void                        beginLog( );
    void                        endLog( );
    void                        printLog( );

    void                        writeLogData( uint8_t id, uint8_t *buf, uint8_t len );
    void                        writeLogId( uint8_t id );
    void                        writeLogTs( );
    void                        writeLogVal( uint8_t valId, uint16_t val );

    private:

    uint16_t                    options                       = DT_OPT_DEFAULT_SETTING;
    volatile uint16_t           flags                         = DT_F_DEFAULT_SETTING;

    volatile uint8_t            trackState                    = 0;
    volatile uint8_t            signalState                   = 0;

    volatile uint32_t           trackTimeStamp                = 0;
    volatile uint8_t            overloadEventCount            = 0;
    volatile uint8_t            overloadRestartCount          = 0;

    uint8_t                     enablePin                     = CDC::UNDEFINED_PIN;
    uint8_t                     dccSigPin1                    = CDC::UNDEFINED_PIN;
    uint8_t                     dccSigPin2                    = CDC::UNDEFINED_PIN;
    uint8_t                     sensePin                      = CDC::UNDEFINED_PIN;
    uint8_t                     uartRxPin                     = CDC::UNDEFINED_PIN;

    uint16_t                    initCurrentMilliAmp           = 0;
    uint16_t                    limitCurrentMilliAmp          = 0;
    uint16_t                    maxCurrentMilliAmp            = 0;

    uint16_t                    startTimeThreshold            = 0;
    uint16_t                    stopTimeThreshold             = 0;
    uint16_t                    overloadTimeThreshold         = 0;
    uint16_t                    overloadEventThreshold        = 0;
    uint16_t                    overloadRestartThreshold      = 0;

    uint16_t                    milliVoltPerAmp               = 0;
    uint16_t                    digitsPerAmp                  = 0;
    volatile uint16_t           actualCurrentDigitValue       = 0;
    volatile uint16_t           highWaterMarkDigitValue       = 0;
    volatile uint16_t           limitCurrentDigitValue        = 0;
    uint16_t                    ackThresholdDigitValue        = 0;

    uint32_t                    totalPwrSamplesTaken          = 0;
    uint32_t                    lastPwrSampleTimeStamp        = 0;

    uint32_t                    lastPwrSamplePerSecTaken      = 0;
    uint32_t                    lastPwrSamplePerSecTimeStamp  = 0;
    uint32_t                    pwrSamplesPerSec              = 0;

    uint8_t                     preambleLen                   = 0;
    uint8_t                     postambleLen                  = 0;
    volatile bool               currentBit                    = false;
    volatile uint8_t            bytesSent                     = 0;
    volatile uint8_t            bitsSent                      = 0;
    volatile uint8_t            preambleSent                  = 0;
    volatile uint8_t            postambleSent                 = 0;
    uint32_t                    dccPacketsSend                = 0;

    DccPacket                   dccBuf1;
    DccPacket                   dccBuf2;
    DccPacket                   *activeBufPtr   = nullptr;
    DccPacket                   *pendingBufPtr  = nullptr;

    // ??? to add....
    // base station capabilities according to RCN200 - 4 16 bit words
    // sample values per second for samples and dcc packets
    // buffers for POM / XPOM data
    // queue for POM / XPOM commands

    uint8_t                     railComBufIndex                   = 0;
    uint8_t                     railComMsgBuf[ RAILCOM_BUF_SIZE ] = { 0 };

    uint8_t                     pwrSampleBufIndex                    = 0;
    uint16_t                    pwrSampleBuf[ PWR_SAMPLE_BUF_SIZE ]      = { 0 };

    public:

    static void                 startDccProcessing( );

};

//------------------------------------------------------------------------------------------------------------
// Every allocated loco session is described by the sessionMap structure. There are the engine cab Id, speed,
// direction and function information. There is also a field that indicates when we received information for
// this session from a cab control handheld. The function flags are stored in an array, each byte representing
// a group. Most of the fields are actually used for a DCC type locomotive. When the locomotive is an analog
// engine, only a subset of the fields is actually used. Nevertheless, even for an analog engine we will 
// have a session. The base station will however not generate packets for this engine.
//
//------------------------------------------------------------------------------------------------------------
struct SessionMapEntry {

  uint16_t        flags               = SME_DEFAULT_SETTING;
  uint16_t        cabId               = LCS::NIL_CAB_ID;
  uint8_t         speed               = 0;
  uint8_t         speedSteps          = 128;
  uint8_t         direction           = 0;
  uint8_t         engineState         = 0;
  uint8_t         nextRefreshStep     = 0;
  unsigned long   lastKeepAliveTime   = 0;
  uint8_t         functions[ LCS::MAX_DCC_FUNC_GROUP_ID ] = { 0 };

};

//------------------------------------------------------------------------------------------------------------
// The loco session object is the central data structure for the base station locomotive management. For a 
// DCC type engine it manages the loco sessions and assembles the DCC packets and drives the DCC track objects
// to send out the relevant DCC packages. For an analog engine it will just manage the session entry and 
// communicate via the LCS bus with the block controller that actually owns the engine at the moment.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBaseStationLocoSession {

  public:

    LcsBaseStationLocoSession( );

    uint8_t setupSessionMap(

        LcsBaseStationSessionMapDesc  *sessionMapDesc,
        LcsBaseStationDccTrack        *mainTrack,
        LcsBaseStationDccTrack        *progTrack
    );

    uint8_t                   requestSession( uint16_t cabId, uint8_t mode, uint8_t *sId );
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

    uint8_t                   setThrottle( uint8_t sId, uint8_t speed, uint8_t direction );
    uint8_t                   setDccFunctionBit( uint8_t sId, uint8_t funcNum, uint8_t val );
    uint8_t                   setDccFunctionGroup( uint8_t sId, uint8_t fGroup, uint8_t dccByte );

    uint8_t                   writeCVMain( uint8_t sId, uint16_t cvId, uint8_t mode, uint8_t val );
    uint8_t                   writeCVByteMain( uint8_t sId, uint16_t cvId, uint8_t val );
    uint8_t                   writeCVBitMain( uint8_t sId, uint16_t cvId, uint8_t bitPos, uint8_t val );

    uint8_t                   readCV( uint16_t cvId, uint8_t mode, uint8_t *val );
    uint8_t                   readCVByte( uint16_t cvId, uint8_t *val );
    uint8_t                   readCVBit( uint16_t cvId, uint8_t bitPos, uint8_t *val );

    uint8_t                   writeCV( uint16_t cvId, uint8_t mode, uint8_t val );
    uint8_t                   writeCVByte( uint16_t cvId, uint8_t val );
    uint8_t                   writeCVBit( uint16_t cvId, uint8_t bitPos, uint8_t val );

    uint8_t                   writeDccPacketMain( uint8_t *buf,  uint8_t len, uint8_t nRepeat );
    uint8_t                   writeDccPacketProg( uint8_t *buf,  uint8_t len, uint8_t nRepeat );

    void                      printSessionMapConfig( );
    void                      printSessionMapInfo( );

    SessionMapEntry           *lookupSessionEntry( uint16_t cabId );
    SessionMapEntry           *getSessionMapEntryPtr( uint8_t sId );

    private:

    uint8_t                   setThrottle( SessionMapEntry *csptr, uint8_t speed, uint8_t direction );
    uint8_t                   setDccFunctionGroup( SessionMapEntry *csPtr, uint8_t fGroup, uint8_t dccByte );

    SessionMapEntry           *allocateSessionEntry( uint16_t cabId );
    void                      deallocateSessionEntry( SessionMapEntry *csPtr );
    void                      refreshSessionEntry( SessionMapEntry *csPtr );
    void                      initSessionEntry( SessionMapEntry *csPtr );
    void                      printSessionEntry( SessionMapEntry *csPtr );

    private:

    LcsBaseStationDccTrack    *mainTrack              = nullptr;
    LcsBaseStationDccTrack    *progTrack              = nullptr;

    uint16_t                  options                 = DT_OPT_DEFAULT_SETTING;
    uint16_t                  flags                   = DT_F_DEFAULT_SETTING;
    uint32_t                  lastAliveCheckTime      = 0L;
    uint32_t                  refreshAliveTimeOutVal  = 2000L;  // ??? a constant name ...

    SessionMapEntry           *sessionMap             = nullptr;
    SessionMapEntry           *sessionMapNextRefresh  = nullptr;
    SessionMapEntry           *sessionMapHwm          = nullptr;
    SessionMapEntry           *sessionMapLimit        = nullptr;

};

//------------------------------------------------------------------------------------------------------------
// One of the key duties of the base station is to listen and react to DCC commands coming via the LCS bus.
// The interface works very closely with the session management and the two DCC track objects.
//
// ??? how about we make the handleLcsMsg handler a routine vs. an object ?
// ??? would make the any REQ/REP scheme easier ?
//------------------------------------------------------------------------------------------------------------
struct LcsBaseStationMsgInterface {

    public:

    LcsBaseStationMsgInterface( );

    uint8_t setupLcsMsgInterface( LcsBaseStationLocoSession   *locoSessions,
                                  LcsBaseStationDccTrack      *mainTrack,
                                  LcsBaseStationDccTrack      *progTrack
                                );

    void handleLcsMsg( uint8_t *msg );

    private:

    LcsBaseStationLocoSession   *locoSessions   = nullptr;
    LcsBaseStationDccTrack      *mainTrack      = nullptr;
    LcsBaseStationDccTrack      *progTrack      = nullptr;

};

//------------------------------------------------------------------------------------------------------------
// The base station implements a serial IO command interface. The command interface uses the DCC++ syntax of
// a command line and where it is a original DCC++ command it implements them in a compatible way. The idea
// is to one day connect to the programs of the JMRI world, which support the DCC++ style command interface.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBaseStationCommand {

    public:

    LcsBaseStationCommand( );

    uint8_t setupSerialCommand( LcsBaseStationLocoSession  *locoSessions,
                                LcsBaseStationDccTrack     *mainTrack,
                                LcsBaseStationDccTrack     *progTrack );

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
    LcsBaseStationDccTrack    *mainTrack    = nullptr;
    LcsBaseStationDccTrack    *progTrack    = nullptr;
};

#endif
