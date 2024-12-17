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
// Copyright (C) 2024 - 2024  Helmut Fieres
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
//
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
// The way to interact with nodes and ports is via ITEM numbers used in GET/PUT/REQ calls. There are defined
// items at the node level which apply to all ports. Ports represent a block. The majority of items refer to
// a block. 
//
// There is a static item portion, which configures the block. This is data is entered when the block is
// configured and stored in the NVM.
//
// The dynamic item part will contain values that initially are fed from the NVM but change during operations.
//
// The request item part maps to user defined items that control the block operation.
//
//------------------------------------------------------------------------------------------------------------
enum BlockControllerItems : uint8_t {

    //--------------------------------------------------------------------------------------------------------
    // Configuration items. Static. Store in NVM.
    //
    // Option, flags, name come from the port portion...
    //--------------------------------------------------------------------------------------------------------
    BC_ITEM_BLOCK_NAME                  = 0,   // maps to port item.
    BC_ITEM_BLOCK_OPTIONS               = 0,   // maps to port item.
    BC_ITEM_BLOCK_FLAGS                 = 0,   // maps to port item.

    BC_ITEM_BLOCK_ID                    = 0,        
    BC_ITEM_BLOCK_ID_EAST_1             = 0,
    BC_ITEM_BLOCK_ID_EAST_2             = 0,
    BC_ITEM_BLOCK_ID_WEST_1             = 0,
    BC_ITEM_BLOCK_ID_WEST_2             = 0,

    BC_ITEM_BLOCK_LENGTH                = 0,
    BC_ITEM_SECTIONS                    = 0,
    BC_ITEM_SECTION_LENGTH_1            = 0,  // more than one necessary ? how to encode more ?
    BC_ITEM_SECTION_LENGTH_2            = 0, 
    BC_ITEM_SECTION_LENGTH_3            = 0, 
    BC_ITEM_SECTION_LENGTH_4            = 0, 
    BC_ITEM_SECTION_LENGTH_5            = 0,
    BC_ITEM_SECTION_LENGTH_6            = 0, 
    BC_ITEM_SECTION_LENGTH_7            = 0, 
    BC_ITEM_SECTION_LENGTH_8            = 0,  

    BC_ITEM_OCC_DETECT_PATH_1           = 0,  // up to two OCC detect boards ? max 8 sections per block ?
    BC_ITEM_OCC_DETECT_PATH_2           = 0,

    BC_ITEM_SIGNAL_PATH_EAST_1          = 0,   // a path: boardId:resourceId
    BC_ITEM_SIGNAL_PATH_EAST_2          = 0,
    BC_ITEM_SIGNAL_PATH_WEST_1          = 0,
    BC_ITEM_SIGNAL_PATH_WEST_2          = 0,

    BC_ITEM_TURNOUT_PATH_EAST_1         = 0,
    BC_ITEM_TURNOUT_PATH_EAST_2         = 0,
    BC_ITEM_TURNOUT_PATH_WEST_1         = 0,
    BC_ITEM_TURNOUT_PATH_WEST_2         = 0,

    BC_ITEM_INITIAL_BLOCK_STATE         = 0,
    BC_ITEM_INITIAL_ROUTE_STATE         = 0,

    BC_ITEM_SPEED_SLOW                  = 0,
    BC_ITEM_SPEED_MIDDLE                = 0,
    BC_ITEM_SPEED_HIGH                  = 0,

    BC_UPDATE_DATA_INTERVAL             = 0,

    BC_ITEM_EVENT_ID_STATE_CHANGE       = 0,
    BC_ITEM_EVENT_ID_OCC_CHANGE         = 0,
    BC_ITEM_EVENT_ID_OCC_ENTER          = 0,
    BC_ITEM_EVENT_ID_OCC_EXIT           = 0,
    BC_ITEM_EVENT_ID_BLOCK_ENTER        = 0,
    BC_ITEM_EVENT_ID_BLOCK_EXIT         = 0,
    BC_ITEM_EVENT_ID_BLOCK_OVL          = 0,

    //--------------------------------------------------------------------------------------------------------
    //
    // ??? on a per node basis...
    //--------------------------------------------------------------------------------------------------------
    BC_ITEM_INIT_CURRENT_MA             = 0,
    BC_ITEM_LIMIT_CURRENT_MA            = 0,
    BC_ITEM_MAX_CURRENT_MA              = 0,
    BC_ITEM_MILLI_VOLT_PER_AMP          = 0,

    BC_ITEM_START_TIME_THRESHOLD        = 0,
    BC_ITEM_STOP_TIME_THRESHOLD         = 0,
    BC_ITEM_OVL_TIME_THRESHOLD          = 0,
    BC_ITEM_OVL_EVENT_THRESHOLD         = 0,
    BC_ITEM_OVL_RESTART_THRESHOLD       = 0,

    //--------------------------------------------------------------------------------------------------------
    // Dynamic GET items.
    //
    // ??? most of the items will be the result of a REQ item and reflect the state of the block.
    // ??? we should not use PUT but rather use REQ with callbacks...
    //--------------------------------------------------------------------------------------------------------
    BC_ITEM_BLOCK_STATE                 = 64,  // gets state  / REQ to control
    BC_ITEM_BLOCK_ROUTE                 = 0,   // gets state  / REQ to control

    BC_ITEM_ENTER_BLOCK_TIMESTAMP       = 0,   // get
    BC_ITEM_EXIT_BLOCK_TIMESTAMP        = 0,   // get 
    BC_ITEM_BLOCK_OCC_MASK              = 0,   // get 

    BC_ITEM_CURRENT_BLOCK_EAST          = 0,   // get
    BC_ITEM_CURRENT_BLOCK_WEST          = 0,   // get 

    BC_ITEM_SIGNAL_EAST_1               = 0,   // get 
    BC_ITEM_SIGNAL_EAST_2               = 0,   // get
    BC_ITEM_SIGNAL_WEST_1               = 0,   // get
    BC_ITEM_SIGNAL_WEST_2               = 0,   // get

    BC_ITEM_TURNOUT_EAST_1              = 0,   // get
    BC_ITEM_TURNOUT_EAST_2              = 0,   // get
    BC_ITEM_TURNOUT_WEST_1              = 0,   // get
    BC_ITEM_TURNOUT_WEST_2              = 0,   // get

    BC_ITEM_ACTUAL_CURRENT_VAL          = 143, // get 


    // thresholds
    // dynamic values
    //  - timeout values of all kinds ?

    //--------------------------------------------------------------------------------------------------------
    // Request items.
    //
    // ??? rather make the slots for GET/PUT ?
    //--------------------------------------------------------------------------------------------------------
   
   
   // ??? I am not even clear whether we need the signals ?

    BC_ITEM_SET_SIGNAL_EAST_1           = 0,
    BC_ITEM_SET_SIGNAL_EAST_2           = 0,
    BC_ITEM_SET_SIGNAL_WEST_1           = 0,
    BC_ITEM_SET_SIGNAL_WEST_2           = 0,

    // necessary for configuring a route.

    BC_ITEM_SET_TURNOUT_EAST_1          = 0,
    BC_ITEM_SET_TURNOUT_EAST_2          = 0,
    BC_ITEM_SET_TURNOUT_WEST_1          = 0,
    BC_ITEM_SET_TURNOUT_WEST_2          = 0,

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
// "LcsBlockControl" manages a block. A block consists mainly of the tack itself and the optional elements
// detectors, signal and turnouts. The block logic, i.e. what to do when the next block is occupied, is 
// handled here.
//
//
// ??? runs the block logic
// ??? how to assign occ detect an signals to the block ?
// ??? should message handling be a separate part ?
// ??? can we build the control logic in such a way that it is configurable via ITEMs ?
//------------------------------------------------------------------------------------------------------------
struct LcsBlockControl {

    LcsBlockControl(  );

    uint8_t handleLcsRequest( uint8_t *msg );

   
    private:

    // ??? handles to detect, signal and turnout object.

};

//------------------------------------------------------------------------------------------------------------
// A LCS block controller node can host up to four blocks. This object is the main object that manages the
// blocks on the node.
//
// ??? the node descriptor is an array of block descriptors. They are kept in the NVM ?
// ??? manages the LCS messages and forwards them to the target block.
//------------------------------------------------------------------------------------------------------------
struct LcsBlockControllerNode {

    public: 

    LcsBlockControllerNode( );

    uint8_t setupBockController( );

    uint8_t handleInitCallback( uint16_t npId );
    uint8_t handleResetCallback( uint16_t npId );
    uint8_t handlePfailCallback( uint16_t npId );
    uint8_t handleLcsMsgCallback( uint8_t *msg );
    uint8_t handleLcsReqCallback( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 );
    uint8_t handleLcsRepCallback( uint16_t npId, uint8_t item, uint16_t arg1, uint16_t arg2, uint8_t ret );
    uint8_t handleLcsEventCallback( uint16_t npId, uint16_t eId, uint8_t eAction, uint16_t eData );

    private:

    uint16_t    options     = 0;
    uint16_t    flags       = 0;
    uint16_t    hwm         = 0;

    LcsBlockControl map[ 4 ];
};

#endif
