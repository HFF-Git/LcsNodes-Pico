//------------------------------------------------------------------------------------------------------------
//
// LCS - Runtime Library - Test Program
//
//------------------------------------------------------------------------------------------------------------
// This source file contains a simple wrapper for the runtime library. The runtime library features a simple
// command interpreter, which will be used to test the library functions. So, all we need to do is to register
// any callbacks, initialize the runtime and the just start it.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2025 Helmut Fieres
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
#include "LcsCdcDescMapDefaults.h"
#include "LcsRuntimeLib.h"
#include "LcsDrvOccDetectLib.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CdcResourceDescMap dMap = RES_MAP_RP_2040;

const uint8_t ADC_0 = CDC_RN_FIRST_USER_RN + 0;
const uint8_t ADC_1 = CDC_RN_FIRST_USER_RN + 1;
const uint8_t DIO_0 = CDC_RN_FIRST_USER_RN + 2;
const uint8_t DIO_1 = CDC_RN_FIRST_USER_RN + 3;
const uint8_t DIO_2 = CDC_RN_FIRST_USER_RN + 4;
const uint8_t DIO_3 = CDC_RN_FIRST_USER_RN + 5;
const uint8_t DIO_4 = CDC_RN_FIRST_USER_RN + 6;
const uint8_t DIO_5 = CDC_RN_FIRST_USER_RN + 7;
const uint8_t DIO_6 = CDC_RN_FIRST_USER_RN + 8;
const uint8_t DIO_7 = CDC_RN_FIRST_USER_RN + 9;

const uint8_t PWM_0 = CDC_RN_FIRST_USER_RN + 10;
const uint8_t PWM_1 = CDC_RN_FIRST_USER_RN + 11;

const uint8_t DIO_P_0 = CDC_RN_FIRST_USER_RN + 12;
const uint8_t DIO_P_1 = CDC_RN_FIRST_USER_RN + 13;
const uint8_t DIO_P_2 = CDC_RN_FIRST_USER_RN + 14;
const uint8_t DIO_P_3 = CDC_RN_FIRST_USER_RN + 15;

const uint8_t PWM_P_0 = CDC_RN_FIRST_USER_RN + 16;


//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library. We get a default CDC config structure and fill in the the additional
// pins for the main controller board we use for testing the library. The runtime initialization will 
// enable the debugging, as we want see as much as possible what is happening. Note that for debugging 
// the various parts of the library, the debug mask needs to be set with a LCS command. 
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    dMap.map[ ADC_0 ].type          = CDC_RT_ADC;
    dMap.map[ ADC_0 ].adc.pin       = 26;
    dMap.map[ ADC_0 ].adc.adcNum    = 0;

    dMap.map[ ADC_1 ].type          = CDC_RT_ADC;
    dMap.map[ ADC_1 ].adc.pin       = 27;
    dMap.map[ ADC_1 ].adc.adcNum    = 1;

    dMap.map[ DIO_0 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_0 ].gpio.pinA     = 9;
    dMap.map[ DIO_0 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_0 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_1 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_1 ].gpio.pinA     = 8;
    dMap.map[ DIO_1 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_1 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_2 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_2 ].gpio.pinA     = 10;
    dMap.map[ DIO_2 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_2 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_3 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_3 ].gpio.pinA     = 11;
    dMap.map[ DIO_3 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_3 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_4 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_4 ].gpio.pinA     = 21;
    dMap.map[ DIO_4 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_4 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_5 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_5 ].gpio.pinA     = 20;
    dMap.map[ DIO_5 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_5 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_6 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_6 ].gpio.pinA     = 19;
    dMap.map[ DIO_6 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_6 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_7 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_7 ].gpio.pinA     = 18;
    dMap.map[ DIO_7 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_7 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_0 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_0 ].gpio.pinA     = 8;
    dMap.map[ DIO_P_0 ].gpio.pinB     = 9;
    dMap.map[ DIO_P_0 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_1 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_1 ].gpio.pinA     = 10;
    dMap.map[ DIO_P_1 ].gpio.pinB     = 11;
    dMap.map[ DIO_P_1 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_2 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_2 ].gpio.pinA     = 21;
    dMap.map[ DIO_P_2 ].gpio.pinB     = 20;
    dMap.map[ DIO_P_2 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_3 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_3 ].gpio.pinA     = 19;
    dMap.map[ DIO_P_3 ].gpio.pinB     = 18;
    dMap.map[ DIO_P_3 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ PWM_0 ].type          = CDC_RT_PWM;
    dMap.map[ PWM_0 ].pwm.pinA      = 20;
    dMap.map[ PWM_0 ].pwm.pinB      = UNDEFINED_PIN;
    dMap.map[ PWM_0 ].pwm.frequency = 100;

    dMap.map[ PWM_1 ].type          = CDC_RT_PWM;
    dMap.map[ PWM_1 ].pwm.pinA      = 21;
    dMap.map[ PWM_1 ].pwm.pinB      = UNDEFINED_PIN;
    dMap.map[ PWM_1 ].pwm.frequency = 100;

    dMap.map[ PWM_P_0 ].type          = CDC_RT_PWM;
    dMap.map[ PWM_P_0 ].pwm.pinA      = 20;
    dMap.map[ PWM_P_0 ].pwm.pinB      = 21;
    dMap.map[ PWM_P_0 ].pwm.frequency = 100;

    dMap.options |= NPO_SKIP_NODE_ID_CONFIG;

    cdcInit( &dMap );
    configureConsoleIO( );
    sleepMillis( 2000 );
    printf( "Test LCS Controller dependent code library\n\n" );

    printResourceDescMap( &dMap );
    printf( "\n" );
    
    // resMap.extNvmSize               = 8192;
    // resMap.EXT_NVM_SIZE             = 4096;
 
    uint8_t rStat = initRuntime( &dMap );
 
    if ( rStat == ALL_OK ) {
 
        printf( "Init runtime, configuration: \n" );
        printResourceMap( );
    }
 
    return( rStat );
}

//----------------------------------------------------------------------------------------------------------
// When a main controller board is used to drive an extension board, the DIO pins need to be set to 
// the value 1 for both pins. This is equivalent to leaving the extension boards select input pins open.
//
//----------------------------------------------------------------------------------------------------------
uint8_t setupPinsForExtBoardTests( ) {

    uint8_t rStat = ALL_OK;
    if ( rStat == ALL_OK ) configureDio( DIO_0, CDC_DIO_OUT );
    if ( rStat == ALL_OK ) configureDio( DIO_1, CDC_DIO_OUT );
    if ( rStat == ALL_OK ) writeDio( DIO_0, false  );
    if ( rStat == ALL_OK ) writeDio( DIO_1, true );

    printf( "Setup DIO pins 0 and 1 for Extension Board Test, stat: %d \n", rStat );
    return( rStat );
}

//----------------------------------------------------------------------------------------------------------
// Callbacks. All we do is to list their invocation.
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg ) {

    printf( "MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) printf( "0x%2x ");
    printf( "\n" );
    return( ALL_OK );
}

uint8_t lcsCmdCallback( char *cmdLine ) {

    printf( "Command Line Callback: %s\n", cmdLine );
    return( ALL_OK );
}

uint8_t lcsTaskCallback1( ) {

    //printf( "Task Callback1...\n" );
    return( ALL_OK );    
}
uint8_t lcsTaskCallback2( ) {

    //printf( "Task Callback2...\n" );
    return( ALL_OK );    
}

uint8_t lcsInitCallback( uint16_t npId ) {

    printf( "Init Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsPfailCallback( uint16_t npId ) {

    printf( "Pfail Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsReqCallback( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr ) printf( ", arg1: %d, ", *arg1 ); else printf( ", arg1: null" );
    if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); else printf( ", arg2: null" );
    return( ALL_OK );
}

uint8_t lcsRepCallback( uint16_t npId, uint8_t item, uint16_t arg1, uint16_t arg2, uint8_t ret ) {

    printf( "REP callback: npId: 0x%x, item: %d, arg1: %d, arg2: %d, ret: %d", npId, item, arg1, arg2, ret );
    return( ALL_OK );
}

uint8_t lcsEventCallback( uint16_t npId, uint16_t eId, uint8_t eAction, uint16_t eData ) {

    printf( "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", npId, eId, eAction, eData );
    return( ALL_OK );
}

uint8_t lcsDccMsgCallback( uint8_t *msg ) {

    printf( "DCC MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) printf( "0x%2x ");
    printf( "\n" );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// The runtime features a rich set of callbacks. We will register all possible callbacks for testing 
// purposes.
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerLcsCallbacks( ) {

    printf( "Registering Callbacks\n" );

    registerLcsMsgCallback( lcsMsgCallback );
    registerDccMsgCallback( lcsDccMsgCallback );
    registerCmdCallback( lcsCmdCallback );
    registerTaskCallback( lcsTaskCallback1, 1000 );
    registerTaskCallback( lcsTaskCallback2, 2000 );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    return( ALL_OK );
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
// This is the last routine we call when the setup worked fine. We actually never return.
//
//----------------------------------------------------------------------------------------------------------
void startLcsRuntime( ) {

    printf( "Start runtime\n" );
    startRuntime( );
}

//----------------------------------------------------------------------------------------------------------
// Main. Set up the hardware, register the callbacks and just start the show.
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = ALL_OK;

    if ( rStat == ALL_OK ) rStat = initLcsRuntime( );
    if ( rStat == ALL_OK ) rStat = setupPinsForExtBoardTests( );
    if ( rStat == ALL_OK ) rStat = registerLcsCallbacks( );
    if ( rStat == ALL_OK ) rStat = registerLcsDrvFunctions( );
    if ( rStat == ALL_OK ) startLcsRuntime( );
    return( ALL_OK );
}