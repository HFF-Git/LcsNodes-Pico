//------------------------------------------------------------------------------------------------------------
//
// LCS - Block Controller
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Block Controller - Raspberry PI Pico Implementation
// Copyright (C) 2024 - 2024 Helmut Fieres
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
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsBlockController.h"

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Block Controller global data.
//
//------------------------------------------------------------------------------------------------------------
uint16_t                        debugMask = DBG_BC_CONFIG | DBG_BC_SETUP | DBG_BC_TRACK_POWER_MGMT;
CDC::CdcConfigDesc              cdcConfig;
LCS::LcsConfigDesc              lcsConfig;
LcsBlockTrackDesc               block1Desc;
LcsBlockTrackDesc               block2Desc;

LcsBlockControllerNode          *bcNode         = nullptr;
LcsBlockControl                 *blockControl   = nullptr;
LcsBlockTrack                   *block1         = nullptr;
LcsBlockTrack                   *block2         = nullptr;

// ??? other BC specific global data ...

//----------------------------------------------------------------------------------------------------------
// Setup the configuration of the HW board. The CDC config contains the HW pin mapping. The dual bridge pins
// for enabling the bridge and controlling its direction. The pins are mapped to the CDC pin names DIO2 to 
// DIO5 as shown below.
//
//      cdcConfig.DIO_PIN_0     -> undefined
//      cdcConfig.DIO_PIN_1     -> undefined
//      cdcConfig.DIO_PIN_2     -> Select-0-1 
//      cdcConfig.DIO_PIN_3     -> Select-0-2     
//      cdcConfig.DIO_PIN_4     -> Select-1-1    
//      cdcConfig.DIO_PIN_5     -> Select-1-2     
//      cdcConfig.DIO_PIN_6     -> undefined 
//      cdcConfig.DIO_PIN_7     -> Cut-Signal
//
// Current mapping: Dual Block Controller Board B.00.01 - PICO - newest version.
//
//      cdcConfig.DIO_PIN_2     = 21; 
//      cdcConfig.DIO_PIN_3     = 20;       
//      cdcConfig.DIO_PIN_4     = 19;  
//      cdcConfig.DIO_PIN_5     = 18;    
//      cdcConfig.DIO_PIN_7     = 4;    
//
// In addition, the HW pins for I2C, analog inputs and so on are set. Check the schematic for the board 
// to see all pin assign,ents.
//
// ??? one day we will have several base station versions. Although they will perhaps differ, their the CDC
// pin names used should not change. But we would need to come up with an idea which configuration to use
// when preparing an image for the base station board.
//----------------------------------------------------------------------------------------------------------
uint8_t setupConfigInfo( ) {

    cdcConfig = CDC::getConfigDefault( );
    lcsConfig = LCS::getConfigDefault( );

    cdcConfig.ADC_PIN_0             = 26;
    cdcConfig.ADC_PIN_1             = 27;

    cdcConfig.PFAIL_PIN             = 5;
    cdcConfig.EXT_INT_PIN           = 22;
    cdcConfig.READY_LED_PIN         = 14;
    cdcConfig.ACTIVE_LED_PIN        = 15;

    cdcConfig.DIO_PIN_2             = 21; 
    cdcConfig.DIO_PIN_3             = 20;       
    cdcConfig.DIO_PIN_4             = 19;  
    cdcConfig.DIO_PIN_5             = 18;    
    cdcConfig.DIO_PIN_7             = 4;    

    cdcConfig.PWM_PIN_0             = 21;
    cdcConfig.PWM_PIN_1             = 20;
    cdcConfig.PWM_PIN_2             = 19;
    cdcConfig.PWM_PIN_3             = 18;

    // ??? more PWM channels ?

    cdcConfig.UART_RX_PIN_1         = 13;
    cdcConfig.UART_RX_PIN_2         = 9;

    cdcConfig.NVM_I2C_SCL_PIN       = 3;
    cdcConfig.NVM_I2C_SDA_PIN       = 2;
    cdcConfig.NVM_I2C_ADR_ROOT      = 0x50;

    cdcConfig.EXT_I2C_SCL_PIN       = 17;
    cdcConfig.EXT_I2C_SDA_PIN       = 16;
    cdcConfig.EXT_I2C_ADR_ROOT      = 0x50;

    cdcConfig.CAN_BUS_RX_PIN        = 0;
    cdcConfig.CAN_BUS_TX_PIN        = 1;
    cdcConfig.CAN_BUS_CTRL_MODE     = CAN_BUS_LIB_PICO_PIO_125K_M_CORE;
    cdcConfig.CAN_BUS_DEF_ID        = 100;

    cdcConfig.NODE_NVM_SIZE         = 8192;
    cdcConfig.EXT_NVM_SIZE          = 512;

    lcsConfig.options               |= NOPT_SKIP_NODE_ID_CONFIG | NOPT_DEBUG_DURING_SETUP;

    return( ALL_OK );
}

const uint16_t PWM_FREQUENCY = 20000;

uint8_t setupBlockDesc1( ) {

    block1Desc.options                         = 0;
    block1Desc.selPin1                         = cdcConfig.PWM_PIN_0;
    block1Desc.selPin2                         = cdcConfig.PWM_PIN_1;
    block1Desc.sensePin                        = cdcConfig.ADC_PIN_0;

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
    block2Desc.selPin1                         = cdcConfig.PWM_PIN_2;
    block2Desc.selPin2                         = cdcConfig.PWM_PIN_3;
    block2Desc.sensePin                        = cdcConfig.ADC_PIN_1;

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

//------------------------------------------------------------------------------------------------------------
// Some little helper functions.
//
//------------------------------------------------------------------------------------------------------------
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

uint8_t lcsResetCallback( uint16_t npId ) {

    return ( bcNode -> handleResetCallback( npId ));
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

//------------------------------------------------------------------------------------------------------------
// We need to run the track state machines on a periodic basis.
//
//
// ??? we actually need an array of track machines ?
// ??? or should we register each one individually ?
//------------------------------------------------------------------------------------------------------------
uint8_t trackStateMachine( ) {

    if ( block1 != nullptr ) block1 -> runTrackStateMachine( );
    if ( block2 != nullptr ) block2 -> runTrackStateMachine( );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// Init the Runtime.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    printf( "LCS Block Controller\n" );
    printf( "initLcsRuntime\n" );

    setupConfigInfo( );

    uint8_t rStat = initRuntime( &lcsConfig, &cdcConfig );

    CDC::printConfigInfo( &cdcConfig );
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
    registerResetCallback( lcsResetCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
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

    uint8_t ret = registerDrvFunc( BT_EXT_OCC_DETECT, lcsDrvOccDetect );
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

    if ( rStat == ALL_OK ) rStat = initLcsRuntime( );
    if ( rStat == ALL_OK ) rStat = registerCallbacks( );
    if ( rStat == ALL_OK ) rStat = registerLcsDrvFunctions( );
    if ( rStat == ALL_OK ) return( startBlockController( ));
}
