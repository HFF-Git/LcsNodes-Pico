//----------------------------------------------------------------------------------------
//
// LCS - Block Controller
//
//----------------------------------------------------------------------------------------
// This source file contains ...
//
//
//----------------------------------------------------------------------------------------
//
// LCS - Block Controller - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsBlockController.h"

using namespace LCS;
using namespace CDC;


// ??? we initialize the runtime and create the block objects.
// ??? the init callback handler will do the block setup.
// ??? we register the tasks for state machine updates.
// ??? what we do here is to call each object handler from within the callback.

// ??? the callback functions will pass the the callback data to the respective
// block or to all, for node commands...

// ??? the data is mostly stored in the attributes, except for the hardware
// related items. How do we get them to the track object ?



//----------------------------------------------------------------------------------------
// Block Controller global data. We feature quite a few objects. There is one 
// object each for occupancy detect, turnout and signal handling. Next, there
// are four track objects and four block objects. Depending on the configuration 
// not all objects are created and used.
//
// ??? should "track" be local to "block" ?
// ??? should "occDetect", "turnout" and "signal" be part of "node" ?
//----------------------------------------------------------------------------------------
uint16_t                debugMask = DBG_BC_CONFIG | 
                                    DBG_BC_SETUP  | 
                                    DBG_BC_TRACK;

CdcResourceDescMap      dMap;

LcsBlockNode            *bcNode     = nullptr;
LcsOccDetect            *occDetect  = nullptr;
LcsTurnoutControl       *turnout    = nullptr;
LcsSignalControl        *signal     = nullptr;

LcsTrackControl         *track1     = nullptr;
LcsTrackControl         *track2     = nullptr;
LcsTrackControl         *track3     = nullptr;
LcsTrackControl         *track4     = nullptr;

LcsBlockControl         *block1     = nullptr;
LcsBlockControl         *block2     = nullptr;
LcsBlockControl         *block3     = nullptr;
LcsBlockControl         *block4     = nullptr;


// ??? goes away ...
LcsBlockTrackDesc       block1Desc;
LcsBlockTrackDesc       block2Desc;


//----------------------------------------------------------------------------------------
// Debug support routines. We can easily check whether debug is enabled at all. 
// The return status routines will print out a return status message when 
// debugging is enabled. The macro "RET_STAT" is a nice helper that adds the
// function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool debugSetupEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_SETUP )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( debugSetupEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))



//----------------------------------------------------------------------------------------
//
// ??? this concept will be reworked...
//----------------------------------------------------------------------------------------
uint8_t setupBlockDesc1( ) {

    block1Desc.options                         = 0;
    block1Desc.rNumControl                     = RNUM_CONTROL_BLK_0;
    block1Desc.rNumSense                       = RNUM_ADC_BLK_0;

    block1Desc.pwmFrequency                    = DEF_PWM_FREQUENCY;

    block1Desc.initCurrentMilliAmp             = 500;
    block1Desc.limitCurrentMilliAmp            = 1500;
    block1Desc.maxCurrentMilliAmp              = 2000;
    block1Desc.milliVoltPerAmp                 = 100 * 11; // ??? opAmp has Factor eleven ...

    block1Desc.startTimeThresholdMillis        = 1000;
    block1Desc.stopTimeThresholdMillis         = 500;
    block1Desc.overloadTimeThresholdMillis     = 500;
    block1Desc.overloadEventThreshold          = 10;
    block1Desc.overloadRestartThreshold        = 5;

    return( NO_ERR );
}

uint8_t setupBlockDesc2( ) {

    block2Desc.options                         = 0;
    block2Desc.rNumControl                     = RNUM_CONTROL_BLK_1;
    block2Desc.rNumSense                       = RNUM_ADC_BLK_1;

    block2Desc.pwmFrequency                    = DEF_PWM_FREQUENCY;

    block2Desc.initCurrentMilliAmp             = 500;
    block2Desc.limitCurrentMilliAmp            = 1500;
    block2Desc.maxCurrentMilliAmp              = 2000;
    block2Desc.milliVoltPerAmp                 = 100 * 11; // ??? opAmp has Factor eleven ...

    block2Desc.startTimeThresholdMillis        = 1000;
    block2Desc.stopTimeThresholdMillis         = 500;
    block2Desc.overloadTimeThresholdMillis     = 500;
    block2Desc.overloadEventThreshold          = 10;
    block2Desc.overloadRestartThreshold        = 5;

    return( NO_ERR );
}





//----------------------------------------------------------------------------------------
// Fire up the base station. First all base station modules are initialized. If OK, 
// the DCC tack signal generation is enabled, i.e. the interrupt driven DCC packet
// broadcasting starts. Finally, the track power is turned on and we give control 
// to the LCS runtime for processing events and requests.
//
//----------------------------------------------------------------------------------------
uint8_t startBlockNode( ) {

    printf( "Start Block Node\n" );

    uint8_t rStat = NO_ERR; 

    setupBlockDesc1( );
    setupBlockDesc2( );
    
    bcNode          = new LcsBlockNode( );
    track1          = new LcsTrackControl( );
    track2          = new LcsTrackControl( );

    printf( "Configure Block 1\n" );
    rStat = track1 -> setupTrackControl( &block1Desc );
    if ( rStat != NO_ERR ) RET_STAT( rStat );

    printf( "Configure Block 2\n" );
    rStat = track2 -> setupTrackControl( &block2Desc );
    if ( rStat != NO_ERR ) RET_STAT( rStat );

    printf( "Block 1 Config:\n" );
    if ( track1 != nullptr ) track1 -> printTrackConfig( );

    printf( "Block 2 Config:\n" );
    if ( track2 != nullptr ) track2 -> printTrackConfig( );

    if ( rStat == NO_ERR ) {

        printf( "Ready...\n" );
        startRuntime( );
    }

    return( NO_ERR );
}


//----------------------------------------------------------------------------------------
// We need to run the track state machines on a periodic basis. 
//
//----------------------------------------------------------------------------------------
uint8_t periodicTrackTasks( void *uData ) {

    if ( track1 != nullptr ) track1 -> runTrackStateMachine( );
    if ( track2 != nullptr ) track2 -> runTrackStateMachine( );
    if ( track3 != nullptr ) track3 -> runTrackStateMachine( );
    if ( track4 != nullptr ) track4 -> runTrackStateMachine( );

    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// We need to run the block state machines on a periodic basis. 
//
//----------------------------------------------------------------------------------------
uint8_t periodicBlockTasks( void *uData ) {

    if ( occDetect != nullptr ) occDetect -> runOccDetectStateMachine( );

    if ( block1 != nullptr ) block1 -> runBlockStateMachine( );
    if ( block2 != nullptr ) block2 -> runBlockStateMachine( );
    if ( block3 != nullptr ) block3 -> runBlockStateMachine( );
    if ( block4 != nullptr ) block4 -> runBlockStateMachine( );

    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Init the Runtime.
//
//----------------------------------------------------------------------------------------
uint8_t initBlockNode( ) {

    if ( debugSetupEnabled( )) printf( "initBlockNode\n" );

    dMap = LCS_BLOCK_CONTROLLER_DUAL_BOARD_DESC_B_02_00;

    uint8_t rStat = initRuntime( &dMap );

    if ( debugSetupEnabled( )) printResourceDescMap( &dMap );
    if ( debugSetupEnabled( )) printResourceMap( );

    bcNode      = new LcsBlockNode( );
    occDetect   = new LcsOccDetect( );
    turnout     = new LcsTurnoutControl( );
    signal      = new LcsSignalControl( );

    return( RET_STAT( rStat ));
}

#if 0
//----------------------------------------------------------------------------------------
// After the initial setup of the runtime library, the callbacks are registered.
//
//----------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {

    if ( debugSetupEnabled( )) printf( "Registering Callbacks\n" );

    registerLcsMsgCallback( lcsMsgCallback );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    registerTaskCallback( periodicTrackTasks, TRACK_STATE_TIME_INTERVAL_MS );
    registerTaskCallback( periodicBlockTasks, BLOCK_STATE_TIME_INTERVAL_MS );

    return( RET_STAT( NO_ERR ));
}
#endif

//----------------------------------------------------------------------------------------
// Setup the drivers for extension boards.
//
//----------------------------------------------------------------------------------------
uint8_t registerDrvFunctions( ) {

    if ( debugSetupEnabled( ))  printf( "registerDrvFunctions\n" );

    uint8_t ret = registerDrvFunc( lcsDrvOccDetect, CDC_BT_EXT_OCC_DETECT );
    
    return( RET_STAT( ret ));
}

//----------------------------------------------------------------------------------------
//
//
// 
//----------------------------------------------------------------------------------------
uint8_t setupTrackControl( ) {

    if ( debugSetupEnabled( )) printf( "setupTrackControl\n" );

    uint8_t rStat = NO_ERR;



    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
//
//
// 
//----------------------------------------------------------------------------------------
uint8_t setupOccDetect( ) {

    if ( debugSetupEnabled( )) printf( "setupOccDetect\n" );

    uint8_t rStat = NO_ERR;



    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
//
//
// 
//----------------------------------------------------------------------------------------
uint8_t setupTurnoutControl( ) {

    if ( debugSetupEnabled( )) printf( "setupTurnoutControl\n" );

    uint8_t rStat = NO_ERR;



    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
//
//
// 
//----------------------------------------------------------------------------------------
uint8_t setupSignalControl( ) {

    if ( debugSetupEnabled( )) printf( "setupSignalControl\n" );

    uint8_t rStat = NO_ERR;



    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
//
//
// 
//----------------------------------------------------------------------------------------
uint8_t setupBlockControl( ) {

    if ( debugSetupEnabled( )) printf( "setupBlockControl\n" );

    uint8_t rStat = NO_ERR;



    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// The main program. Setup the runtime, register the callbacks, and get the show 
// on the road.
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = NO_ERR;

    if ( rStat == NO_ERR ) rStat = initBlockNode( );
    if ( rStat == NO_ERR ) rStat = registerDrvFunctions( );
    if ( rStat == NO_ERR ) rStat = setupTrackControl( );
    if ( rStat == NO_ERR ) rStat = setupOccDetect( );
    if ( rStat == NO_ERR ) rStat = setupTurnoutControl( );
    if ( rStat == NO_ERR ) rStat = setupSignalControl( );
    if ( rStat == NO_ERR ) rStat = setupBlockControl( );
    if ( rStat == NO_ERR ) return( startBlockNode( ));
}
