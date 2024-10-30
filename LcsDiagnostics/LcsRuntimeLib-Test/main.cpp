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
// Copyright (C) 2022 - 2024 Helmut Fieres
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

using namespace LCS;

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcConfigDesc  cdcConfig;
LCS::LcsConfigDesc  lcsConfig;

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library. We get a default CDC config structure and fill in the the additional
// pins for the main controller board we use for testing the library. The runtime initialization will 
// enable the debugging, as we want see as much as possible what is happening. Note that for debugging 
// the various parts of the library, the debug mask needs to be set with a LCS command. 
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint8_t rStat;

    cdcConfig = CDC::getConfigDefault( );

    cdcConfig.READY_LED_PIN         = 14;
    cdcConfig.ACTIVE_LED_PIN        = 15;
    
    cdcConfig.ADC_PIN_0             = 26;
    cdcConfig.ADC_PIN_1             = 27;

    cdcConfig.DIO_PIN_0             = 9;
    cdcConfig.DIO_PIN_1             = 8;
    cdcConfig.DIO_PIN_2             = 10;
    cdcConfig.DIO_PIN_3             = 11;
    cdcConfig.DIO_PIN_4             = 21;
    cdcConfig.DIO_PIN_5             = 20;
    cdcConfig.DIO_PIN_6             = 19;
    cdcConfig.DIO_PIN_7             = 18;

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

    rStat = LCS::initRuntime( &lcsConfig, &cdcConfig );

    if ( rStat == ALL_OK ) {

        printf( "Init runtime, configuration: \n" );
        CDC::printConfigInfo( &cdcConfig );
    }

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

uint8_t lcsTaskCallback( ) {

    // printf( "Task Callback...\n" );
    return( ALL_OK );    
}

uint8_t lcsInitCallback( uint16_t npId ) {

    printf( "Init Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsResetCallback( uint16_t npId ) {

    printf( "Reset Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsPfailCallback( uint16_t npId ) {

    printf( "Pfail Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsReqCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr ) printf( ", arg1: %d, ", *arg1 ); else printf( ", arg1: null" );
    if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); else printf( ", arg2: null" );
    return( ALL_OK );
}

uint8_t lcsRepCallback( uint8_t npId, uint8_t item, uint16_t arg1, uint16_t arg2, uint8_t ret ) {

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
    registerTaskCallback( lcsTaskCallback, 1000 );
    registerInitCallback( lcsInitCallback );
    registerResetCallback( lcsResetCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    return( ALL_OK );
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
    if ( rStat == ALL_OK ) rStat = registerLcsCallbacks( );
    if ( rStat == ALL_OK ) startLcsRuntime( );
    return( ALL_OK );
}