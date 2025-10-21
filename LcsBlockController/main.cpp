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
// Copyright (C) 2024 - 2024 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should
// have received a copy of the GNU General Public License along with this program. 
// If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsBlockControllerBoardDesc.h"
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsBlockController.h"

using namespace LCS;
using namespace CDC;




//----------------------------------------------------------------------------------------
// Block Controller global data.
//
//----------------------------------------------------------------------------------------
uint16_t                        debugMask = DBG_BC_CONFIG | DBG_BC_SETUP | DBG_BC_TRACK_POWER_MGMT;

CdcResourceDescMap              dMap;
LcsBlockTrackDesc               block1Desc;
LcsBlockTrackDesc               block2Desc;

LcsBlockControllerNode          *bcNode         = nullptr;
LcsBlockControl                 *blockControl   = nullptr;
LcsBlockTrack                   *block1         = nullptr;
LcsBlockTrack                   *block2         = nullptr;

//----------------------------------------------------------------------------------------------------------
// Setup the resource configuration data and the CDC library.
//
// ??? current config - dual block controller
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

    dMap = LCS_BLOCK_CONTROLLER_DUAL_BOARD_DESC_B_02_00;
   // dMap.options |= NPO_SKIP_NODE_ID_CONFIG | NPO_DEBUG_DURING_SETUP;
    
    cdcInit( &dMap );
    configureConsoleIO( );
    sleepMillis( 2000 );
    printf( "Test LCS Controller dependent code library\n" );
}


//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
uint8_t setupBlockDesc1( ) {

    block1Desc.options                         = 0;
    block1Desc.rNumControl                     = RNUM_CONTROL_BLK_0;
    block1Desc.rNumSense                       = RNUM_ADC_BLK_0;

    block1Desc.pwmFrequency                    = PWM_FREQUENCY;

    block1Desc.initCurrentMilliAmp             = 500;
    block1Desc.limitCurrentMilliAmp            = 1500;
    block1Desc.maxCurrentMilliAmp              = 2000;
    block1Desc.milliVoltPerAmp                 = 100 * 11; // ??? opAmp has Factor eleven ...

    block1Desc.startTimeThresholdMillis        = 1000;
    block1Desc.stopTimeThresholdMillis         = 500;
    block1Desc.overloadTimeThresholdMillis     = 500;
    block1Desc.overloadEventThreshold          = 10;
    block1Desc.overloadRestartThreshold        = 5;

    return( ALL_OK );
}

uint8_t setupBlockDesc2( ) {

    block2Desc.options                         = 0;
    block2Desc.rNumControl                     = RNUM_CONTROL_BLK_1;
    block2Desc.rNumSense                       = RNUM_ADC_BLK_1;

    block2Desc.pwmFrequency                    = PWM_FREQUENCY;

    block2Desc.initCurrentMilliAmp             = 500;
    block2Desc.limitCurrentMilliAmp            = 1500;
    block2Desc.maxCurrentMilliAmp              = 2000;
    block2Desc.milliVoltPerAmp                 = 100 * 11; // ??? opAmp has Factor eleven ...

    block2Desc.startTimeThresholdMillis        = 1000;
    block2Desc.stopTimeThresholdMillis         = 500;
    block2Desc.overloadTimeThresholdMillis     = 500;
    block2Desc.overloadEventThreshold          = 10;
    block2Desc.overloadRestartThreshold        = 5;

    return( ALL_OK );
}

//----------------------------------------------------------------------------------------
// Some little helper functions.
//
//----------------------------------------------------------------------------------------
uint8_t printStatus (uint8_t status ) {

  printf( "Status: " );
  if ( status == LCS::ALL_OK ) printf( "OK\n" );
  else printf ( "FAILED: %d\n", status );
  return ( status );
}

//----------------------------------------------------------------------------------------------------------
// The LCS runtime callback forwards. We register these routines with the runtime. All they do is to 
// dispatch the incoming callback to the block controller node object, which in turn will dispatch to the
// correct block object.
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsInitCallback( uint16_t npId ) {

    return ( bcNode -> handleInitCallback( npId ));
}

uint8_t lcsPfailCallback( uint16_t npId ) {

    return ( bcNode -> handlePfailCallback( npId ));
}

uint8_t lcsMsgCallback( uint8_t *msg ) {

    return ( bcNode -> handleLcsMsgCallback( msg ));
}

uint8_t lcsReqCallback( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    return( bcNode -> handleLcsReqCallback( npId, item, arg1, arg2 ));
}

uint8_t lcsRepCallback( uint16_t npId, uint8_t item, uint16_t arg1, uint16_t arg2, uint8_t ret ) {

    return ( bcNode -> handleLcsRepCallback( npId, item, arg1, arg2, ret ));
}

uint8_t lcsEventCallback( uint16_t npId, uint16_t eId, uint8_t eAction, uint16_t eData ) {

    return ( bcNode -> handleLcsEventCallback( npId, eId, eAction, eData ));
}

//----------------------------------------------------------------------------------------
// We need to run the track state machines on a periodic basis.
//
//
// ??? we actually need an array of track machines ?
// ??? or should we register each one individually ?
//----------------------------------------------------------------------------------------
uint8_t trackStateMachine( ) {

    if ( block1 != nullptr ) block1 -> runTrackStateMachine( );
    if ( block2 != nullptr ) block2 -> runTrackStateMachine( );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// Init the Runtime.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initThrottle( ) {

    printf( "LCS Block Controller\n" );
    printf( "initLcsRuntime\n" );

    setupConfigInfo( );

    uint8_t rStat = initRuntime( &dMap );

    printResourceDescMap( &dMap );
    printResourceMap( );
    return( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------------------------
// After the initial setup of the runtime library, the callback are registered.
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {

    printf( "Registering Callbacks\n" );

    registerLcsMsgCallback( lcsMsgCallback );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
    
    
    //registerReqCallback( lcsReqCallback );
    
    
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    registerTaskCallback( trackStateMachine, TRACK_STATE_TIME_INTERVAL );

    return( printStatus( ALL_OK ));
}

//----------------------------------------------------------------------------------------------------------
// Setup the drivers for extension boards.
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerLcsDrvFunctions( ) {

    printf( "Register Extension Board Drivers\n" );

    uint8_t ret = registerDrvFunc( CDC_BT_EXT_OCC_DETECT, lcsDrvOccDetect );
    if ( ret != ALL_OK )  printf( "Registration failed: %d\n, ret ");

    return( ret );
}

//----------------------------------------------------------------------------------------------------------
// Fire up the base station. First all base station modules are initialized. If this is OK, the DCC tack
// signal generation is enabled, i.e. the interrupt driven DCC packet broadcasting starts. Finally, the 
// track power is turned on and we give control to the LCS runtime for processing events and requests.
//
//----------------------------------------------------------------------------------------------------------
uint8_t startBlockController( ) {

    printf( "Start Block controller\n" );

    uint8_t rStat = ALL_OK; 

    setupBlockDesc1( );
    setupBlockDesc2( );
    
    bcNode          = new LcsBlockControllerNode( );
    blockControl    = new LcsBlockControl( );
    block1          = new LcsBlockTrack( );
    block2          = new LcsBlockTrack( );

    printf( "Configure Block 1\n" );
    rStat = block1 -> setupBlockTrack( &block1Desc );
    if ( rStat != ALL_OK ) printStatus( rStat );

    printf( "Configure Block 2\n" );
    rStat = block2 -> setupBlockTrack( &block2Desc );
    if ( rStat != ALL_OK ) printStatus( rStat );

    printf( "Block 1 Config:\n" );
    if ( block1 != nullptr ) block1 -> printTrackConfig( );

    printf( "Block 2 Config:\n" );
    if ( block2 != nullptr ) block2 -> printTrackConfig( );

    if ( rStat == ALL_OK ) {

        printf( "Ready...\n" );
        startRuntime( );
    }

    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// The main program. Setup the runtime, register the callbacks, and get the show on the road.
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = ALL_OK;

    if ( rStat == ALL_OK ) rStat = initThrottle( );
    if ( rStat == ALL_OK ) rStat = registerCallbacks( );
    if ( rStat == ALL_OK ) rStat = registerLcsDrvFunctions( );
    if ( rStat == ALL_OK ) return( startBlockController( ));
}




#if 0
//----------------------------------------------------------------------------------------
// Configure a Bridge PIO instance.
// 
// ??? not used yet, under development....
// ??? we need to get the right PIO program mix...
//----------------------------------------------------------------------------------------
void setupPioProgramInstance ( int index ) {

    printf( "setupPioProgramInstance: index: %d\n", index );

    uint offset = pio_add_program( pio, & dcc_h_bridge_control_program );
    sm[ index ] = pio_claim_unused_sm( pio, true );

   
    start_dcc_booster_program( pio, uint sm, uint pin_in, uint pin_out)

    dcc_h_bridge_control_program_init(  pio, 
                                        sm[ index ], 
                                        offset, 
                                        OUTPUT_PINS[ index ][ 0 ], 
                                        OUTPUT_PINS[ index ][ 1], 
                                        INPUT_PINS[ index ][ 0 ], 
                                        INPUT_PINS[ index ][ 1] 
                                    );

}

//----------------------------------------------------------------------------------------
// Switching routines when going from PIO control to PWM control of a pin and back.
//
//----------------------------------------------------------------------------------------
void switchToPwm( uint gpio ) {

    printf( "switchToPwm: %d/n", gpio );
    gpio_set_function( gpio, GPIO_FUNC_PWM );
}

void switchToPio( uint gpio ) {

    printf( "switchToPio: %d/n", gpio );
    gpio_set_function( gpio, GPIO_FUNC_PIO0 );
}

//----------------------------------------------------------------------------------------
// Control the Bridge. We have to claim the PWM if needed and release it later on.
//
//----------------------------------------------------------------------------------------
void setPioSelect( int index, int sel ) {

    printf( "setPioSelect: index: %d, sel: %d\n", index, sel );

    if      ( sel == 1 ) switchToPwm( OUTPUT_PINS[ index % 2 ] [ 0 ] );
    else if ( sel == 2 ) switchToPwm( OUTPUT_PINS[ index % 2 ] [ 1 ] );
       
    pio_sm_put( pio, sm[ index % 2 ], sel % 2 ); 

    if      ( sel == 1 ) switchToPio( OUTPUT_PINS[ index % 2 ] [ 0 ] );
    else if ( sel == 2 ) switchToPio( OUTPUT_PINS[ index % 2 ] [ 1 ] );
}

//----------------------------------------------------------------------------------------
// Globals. Example.
//
//----------------------------------------------------------------------------------------
const int   MAX_BRIDGE_INSTANCES        = 4;
PIO         pio                         = pio0;
uint        sm[ MAX_BRIDGE_INSTANCES ]  = { 0 };

const uint OUTPUT_PINS[ MAX_BRIDGE_INSTANCES ][ 2 ] = {

    {6, 7}, {8, 9}, {18,19}, {20, 21}
};

const uint INPUT_PINS[ MAX_BRIDGE_INSTANCES ][ 2 ] = {
    
    {4, 5}, {4, 5}, {4, 5}, {4, 5}  
};


#endif

                            
