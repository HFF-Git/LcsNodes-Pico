//------------------------------------------------------------------------------------------------------------
//
// LCS - Runtime Library - Test Program
//
//------------------------------------------------------------------------------------------------------------
// This source file contains a simple wrapper for the runtime library. The runtime library features a simple
// command interpreter, which we will use to test the library functions. So, all we need to do is to register
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

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg;

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library. 
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    cfg.ADC_PIN_0             = 26;
    cfg.ADC_PIN_1             = 27;

    cfg.DIO_PIN_0             = 9;
    cfg.DIO_PIN_1             = 8;
    cfg.DIO_PIN_2             = 10;
    cfg.DIO_PIN_3             = 11;
    cfg.DIO_PIN_4             = 21;
    cfg.DIO_PIN_5             = 20;
    cfg.DIO_PIN_6             = 19;
    cfg.DIO_PIN_7             = 18;

    printf( "Init LCS runtime, configuration: \n" );
    CDC::printConfigInfo( &cfg );

    return( LCS::initRuntime( &cfg, LCS::NOPT_DEBUG_DURING_SETUP ));
}

//----------------------------------------------------------------------------------------------------------
//
//
// ??? callbacks.....
//----------------------------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg ) {

    return( 0 );
}

uint8_t lcsCmdCallback( char *cmdLine ) {

    return( 0 );
}

uint8_t lcsTaskCallback( ) {

    return( 0 );    
}

uint8_t lcsInitCallback( uint16_t npId ) {

  return( 0 );
}

uint8_t lcsResetCallback( uint16_t npId ) {

  return( 0 );
}

uint8_t lcsPfailCallback( uint16_t npId ) {

    return( 0 );
}

uint8_t lcsReqCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    return( 0 );
}

uint8_t lcsRepCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    return( 0 );
}

uint8_t lcsEventCallback( uint16_t npId, uint8_t eAction, uint16_t eId, uint16_t eData ) {

    return( 0 );
}

uint8_t lcsDccMsgCallback( uint8_t *msg ) {

    return( 0 );
}

//----------------------------------------------------------------------------------------------------------
// The runtime features a rich set of callbacks. We will register all possible callbacks for testing 
// purposes.
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerLcsCallbacks( ) {

    printf( "Registering Callbacks\n" );

    LCS::registerLcsMsgCallback( lcsMsgCallback );
    LCS::registerDccMsgCallback( lcsDccMsgCallback );
    LCS::registerCmdCallback( lcsCmdCallback );
    LCS::registerTaskCallback( lcsTaskCallback, 1000 );
    LCS::registerInitCallback( lcsInitCallback );
    LCS::registerResetCallback( lcsResetCallback );
    LCS::registerPfailCallback( lcsPfailCallback );
    LCS::registerReqCallback( lcsReqCallback );
    LCS::registerRepCallback( lcsRepCallback );
    LCS::registerEventCallback( lcsEventCallback );
    return( LCS::ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// This is the last routine we call when the setup worked fine. We actually never return.
//
//----------------------------------------------------------------------------------------------------------
void startLcsRuntime( ) {

    printf( "Start LCS runtime\n" );
    LCS::startRuntime( );
}

//----------------------------------------------------------------------------------------------------------
// Main. Set up the hardware, register the callbacks, set up the runtime and then just start the show.
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = LCS::ALL_OK;

    if ( rStat == LCS::ALL_OK ) rStat = registerLcsCallbacks( );
    if ( rStat == LCS::ALL_OK ) rStat = initLcsRuntime( );
    if ( rStat == LCS::ALL_OK ) startLcsRuntime( );
    return( LCS::ALL_OK );
}