//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller - Include file
//
//------------------------------------------------------------------------------------------------------------
//
// ??? this is a first cut at the block controller software. It remains to be seen what we should factor out
// and use across base station and block controller.
//
//
//
//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller
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
#ifndef LcsBlockController_h
#define LcsBlockController_h

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//------------------------------------------------------------------------------------------------------------
//
// Ideas how to use the node data:
//
// There is a static data portion, which describes the block. This is data is entered when the block is configured.
//
//  - block ID
//  - block length
//  - block name
//  - previous block(s)
//  - next block(s)
//
//  - number of sections
//  - section lengths
//  - speed level - slow, middle, high ... 
//  - support DCC and analog flag
//  - max current limit
//  - periodic time to send data
//  - timeout values of all kinds ?
//
//
// There is a dynamic data portion, which contains the data about the block current state
//
//  - mode ( DCC or analog or off )
//  - actual current
//  - section occupancy
//  - section enter / leave timestamps
//  - 
//
// ??? what is retrieved from the dynamic data on a restart ?
//
// The node attributes contains data about how many blocks this node contains ( nodeId + portId -> blockId )
// 
// Most of the data is stored in attributes for the port.
// 
//
// Finally, there are items that represent commands to the block. 
// 
//  - emergency stop
//  - switch to DCC or analog mode
//  - block on or off
//  - signals setting
//  - turnout setting
//  - ...

// There are predefined events that the controller node will send.
// 
//  - block state change
//  - section occupied
//  - section entered
//  - section left
//  - 
//  
//
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
// The block controller maintains a set of debug flags. The overall concept is very similar to the LCS runtime
// library debug mask. Then following debug flags are defined:
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
//------------------------------------------------------------------------------------------------------------
enum BlockControllerDebugFlags : uint16_t {

    DBG_BC_CONFIG                  = 1 << 15,       // DEBUG enabled
    DBG_BC_SETUP                   = 1 << 1,        // show the setup steps
    DBG_BC_LCS_MSG_INTERFACE       = 1 << 2,        // show the incoming LCS messages
    DBG_BC_TRACK_POWER_MGMT        = 1 << 3,        // show the track power measurement data
    DBG_BC_RAILCOM                 = 1 << 4         // show the RailCom activity
};

//------------------------------------------------------------------------------------------------------------
// The base station items for nodeInfo and nodeControl calls .... tbd
//
// ??? the are mapped in the MEM / NVM range as well as in the USER range.
// ??? how to do it consistently and understandably ?
//------------------------------------------------------------------------------------------------------------
enum BlockControllerItems : uint8_t {

    BC_ITEM_SET_TRACK_STATE         = 64,

    BC_ITEM_INIT_CURRENT_VAL        = 140,
    BC_ITEM_LIMIT_CURRENT_VAL       = 141,
    BC_ITEM_MAX_CURRENT_VAL         = 142,
    BC_ITEM_ACTUAL_CURRENT_VAL      = 143,

    // thresholds
    // eventID to send for events ?
};

//------------------------------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user number range of errors defined 
// in the LCS runtime library. 
//
//------------------------------------------------------------------------------------------------------------
enum BlockControllerErrors : uint8_t {

    BLOCK_CONTROLLER_ERR_BASE       = 128,

    ERR_MSG_INTERFACE_SETUP         = BLOCK_CONTROLLER_ERR_BASE + 10,
    ERR_DCC_TRACK_CONFIG            = BLOCK_CONTROLLER_ERR_BASE + 11,
    ERR_PIN_CONFIG                  = BLOCK_CONTROLLER_ERR_BASE + 12,
    ERR_TRACK_CONFIG                = BLOCK_CONTROLLER_ERR_BASE + 13,

    ERR_NVM_HW_SETUP                = BLOCK_CONTROLLER_ERR_BASE + 15,
    ERR_PIO_HW_SETUP                = BLOCK_CONTROLLER_ERR_BASE + 16
};

//------------------------------------------------------------------------------------------------------------
// Setup options to set for the DCC track. They are set when the track object is created.
//
//  DT_OPT_SERVICE_MODE_TRACK  - The track is a PROG track.
//  DT_OPT_RAILCOM             - The track support Railcom detection.
//
//------------------------------------------------------------------------------------------------------------
enum BlockControllerTrackOptions : uint16_t {

    BT_OPT_DEFAULT_SETTING      = 0,
    BT_OPT_RAILCOM              = 1 << 1
};

//------------------------------------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------------------------------------
enum TrackFlags : uint16_t {

    BT_F_DEFAULT_SETTING      = 0,
    BT_F_POWER_ON             = 1 << 0,
    BT_F_POWER_OVERLOAD       = 1 << 1,
    BT_F_MEASUREMENT_ON       = 1 << 2,
    BT_F_CONFIG_ERROR         = 1 << 15
};

//------------------------------------------------------------------------------------------------------------
// The following constants are for the current consumption RMS measurement. The idea is to record the measured
// ADC values in a circular buffer, every time a certain amount of milliseconds has passed. This work is done
// by the DCC track state machine as part of the power on state.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t   PWR_SAMPLE_BUF_SIZE               = 64;
const uint32_t  PWR_SAMPLE_TIME_INTERVAL_MILLIS   = 16;

//----------------------------------------------------------------------------------------------------------
// The track state machine runs at a time interval.
//
//----------------------------------------------------------------------------------------------------------
const uint32_t TRACK_STATE_TIME_INTERVAL  = 10;

//----------------------------------------------------------------------------------------------------------
// A block track can be in four states.
//
//----------------------------------------------------------------------------------------------------------
enum BlockTrackMode : uint16_t {

    BT_MODE_OFF        = 0,
    BT_MODE_PWM_FWD    = 1,
    BT_MODE_PWM_REV    = 2,
    BT_MODE_DCC        = 3
};

//------------------------------------------------------------------------------------------------------------
// The block controller can contain up to four blocks. Each block track is described by the LcsBlockDesc
// descriptor. There are the hardware pins sel1Pin1, selPin2, sensePin and uartRxPin. In addition there are
// the limits for current consumption values, all specified in milliAmps. The initial current sets the current
// consumption limit after the track is turned on. The limit current consumption specifies the actual 
// configured value that is checked for a track current overload situation. The maximum current defines what 
// current the power module should never exceed. For the measurements to work, the power module needs to 
// deliver a voltage that corresponds to the current drawn on the track. The value is measured in milliVolt 
// per Ampere drawn. Finally, there are threshold times for managing the track overload and restart 
// capability.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBlockTrackDesc {

    uint16_t    options;
    uint8_t     selPin1                         = CDC::UNDEFINED_PIN;
    uint8_t     selPin2                         = CDC::UNDEFINED_PIN;
    uint8_t     sensePin                        = CDC::UNDEFINED_PIN;

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

//------------------------------------------------------------------------------------------------------------
// The "LcsBlockTrack" manages the track of a block. This primarily the power management and control of the 
// H-Bridge settings. There is one object per track block. At the heart of the object is a state machine that
// is executed very often for measuring the power consumption and overload detection logic. The tack can 
// operate in digital or analog mode. In digital mode, the DCC signal from the LCS bus is routed though to
// the H-Bridge, in analog mode a PWM signal is used to set the H-Bridge emitting a PWM signal with a 
// positive or negative voltage.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBlockTrack {

    public:

    LcsBlockTrack( );

    uint8_t                     setupBlockTrack( LcsBlockTrackDesc* trackDesc );
    uint8_t                     setTrackState( uint16_t state );
    uint8_t                     setTrackMode( uint16_t mode, uint8_t speed = 0 );
    
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

    uint8_t                     selPin1                         = CDC::UNDEFINED_PIN;
    uint8_t                     selPin2                         = CDC::UNDEFINED_PIN;
    uint8_t                     sensePin                        = CDC::UNDEFINED_PIN;
   
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

//------------------------------------------------------------------------------------------------------------
// "LcsOccDetect" manages an Occupancy detector extension board. The track power output of a block controller
// track is routed to an extension board which implements a set of current detectors. The extension board is
// access via the extension I2C bus.
//
//------------------------------------------------------------------------------------------------------------
struct LcsOccDetect {

    public:

    LcsOccDetect( );

    uint8_t getOccDetectMask( uint16_t *mask );

    private:    

    // ??? need to remember the extension board ID.

};

//------------------------------------------------------------------------------------------------------------
// "LcsSignal" manages a signal. A block has a signal for each direction to indicate the state of the next
// block in a route.
//
//------------------------------------------------------------------------------------------------------------
struct LcsSignalControl {

    public:

    LcsSignalControl( );


    private:

    // ??? need to remember the extension board ID.

};

//------------------------------------------------------------------------------------------------------------
// "LcsTurnout" manages the optional turnouts at the end of a block.
//
//
//------------------------------------------------------------------------------------------------------------
struct LcsTurnoutControl {

    public:

    LcsTurnoutControl( );

    private:

    // ??? need to remember the extension board ID.
};

//------------------------------------------------------------------------------------------------------------
// "LcsRailComDetect" manages the optional RailCom interface for the block.
//
//------------------------------------------------------------------------------------------------------------
struct LcsRailComDetect {

    public:

    LcsRailComDetect( );

    private:

    // ??? need to remember the extension board ID.
};






//------------------------------------------------------------------------------------------------------------
// "LcsBlockControllerLogic" manages a set of blocks available in the block controller hardware.
//
//
// ??? runs the block logic
// ??? how to manage one to four blocks ?
// ??? how to assign occ detect an signals to the block ?
// ??? should message handling be a separate part ?
// ??? can we build the control logic in such a way that it is configurable via ITEMs ?
//------------------------------------------------------------------------------------------------------------
struct LcsBlockControllerLogic {


    LcsBlockControllerLogic(  );

    uint8_t handleLcsRequest( uint8_t *msg );

    private:

    
};


//------------------------------------------------------------------------------------------------------------
// Do we need an object that encompasses all blocks ?
//
// ??? or just an array of block controller logic objects ?
//------------------------------------------------------------------------------------------------------------


#endif
