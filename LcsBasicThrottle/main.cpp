//------------------------------------------------------------------------------------------------------------
//
// LCS - Basic Throttle
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
// ??? both throttle could share a throttle lib with common code...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Code - Raspberry PI Pico Implementation
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

// #include "LcsCdcLib.h"
// #include "LcsRuntimeLib.h"
// #include "LcsUIElements.h"
#include "LcsBasicThrottle.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CdcResourceDescMap      dMap;

//----------------------------------------------------------------------------------------------------------
// Externals.
//
//----------------------------------------------------------------------------------------------------------
extern UIDisplay        *oled;
extern UIEncoder        *encoder;
extern CabStack         *cabStack;
extern CabMsgBus        *msgBus;

extern uint8_t          setupMsgBus( );
extern uint8_t          setupUIElements( );
extern uint8_t          setupScreens( );
extern uint8_t          setupCabStack( );

//----------------------------------------------------------------------------------------------------------
// "printStatus" is a little helper function for the initialization routines protocol printing. If there is
// a serial IO, these routines will list the success of the particular setup operation.
//
// ??? may be a bit of an overkill ?
//----------------------------------------------------------------------------------------------------------
uint8_t printStatus( uint8_t status ) {

    if ( status == ALL_OK ) printf( "-> OK\n" );
    else                    printf( "-> FAILED: %d\n", status );
       
    return ( status );
}

//----------------------------------------------------------------------------------------------------------
// Setup the resource configuration data and the CDC library.
//
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

    dMap = RES_MAP_RP_2040;

    dMap.map[ RNUM_MENU_BUTTON ].type               = CDC_RT_GPIO;
    dMap.map[ RNUM_MENU_BUTTON ].gpio.pinA          = 6;
    dMap.map[ RNUM_MENU_BUTTON ].gpio.pinB          = UNDEFINED_PIN;
    dMap.map[ RNUM_MENU_BUTTON ].gpio.pinMode       = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_SELECT_BUTTON ].type             = CDC_RT_GPIO;
    dMap.map[ RNUM_SELECT_BUTTON ].gpio.pinA        = 8;
    dMap.map[ RNUM_SELECT_BUTTON ].gpio.pinB        = UNDEFINED_PIN;
    dMap.map[ RNUM_SELECT_BUTTON ].gpio.pinMode     = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_UP_BUTTON ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_UP_BUTTON ].gpio.pinA            = 7;
    dMap.map[ RNUM_UP_BUTTON ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_UP_BUTTON ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_DOWN_BUTTON ].type               = CDC_RT_GPIO;
    dMap.map[ RNUM_DOWN_BUTTON ].gpio.pinA          = 9;
    dMap.map[ RNUM_DOWN_BUTTON ].gpio.pinB          = UNDEFINED_PIN;
    dMap.map[ RNUM_DOWN_BUTTON ].gpio.pinMode       = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_HORN_BUTTON ].type               = CDC_RT_GPIO;
    dMap.map[ RNUM_HORN_BUTTON ].gpio.pinA          = 22;
    dMap.map[ RNUM_HORN_BUTTON ].gpio.pinB          = UNDEFINED_PIN;
    dMap.map[ RNUM_HORN_BUTTON ].gpio.pinMode       = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_HORN_BUTTON ].type               = CDC_RT_GPIO;
    dMap.map[ RNUM_HORN_BUTTON ].gpio.pinA          = 15;
    dMap.map[ RNUM_HORN_BUTTON ].gpio.pinB          = UNDEFINED_PIN;
    dMap.map[ RNUM_HORN_BUTTON ].gpio.pinMode       = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_FWD_BUTTON ].type                = CDC_RT_GPIO;
    dMap.map[ RNUM_FWD_BUTTON ].gpio.pinA           = 10;
    dMap.map[ RNUM_FWD_BUTTON ].gpio.pinB           = UNDEFINED_PIN;
    dMap.map[ RNUM_FWD_BUTTON ].gpio.pinMode        = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_REV_BUTTON ].type                = CDC_RT_GPIO;
    dMap.map[ RNUM_REV_BUTTON ].gpio.pinA           = 11;
    dMap.map[ RNUM_REV_BUTTON ].gpio.pinB           = UNDEFINED_PIN;
    dMap.map[ RNUM_REV_BUTTON ].gpio.pinMode        = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_F1_BUTTON ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_F1_BUTTON ].gpio.pinA            = 18;
    dMap.map[ RNUM_F1_BUTTON ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_F1_BUTTON ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_F2_BUTTON ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_F2_BUTTON ].gpio.pinA            = 19;
    dMap.map[ RNUM_F2_BUTTON ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_F2_BUTTON ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_F3_BUTTON ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_F3_BUTTON ].gpio.pinA            = 20;
    dMap.map[ RNUM_F3_BUTTON ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_F3_BUTTON ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_F4_BUTTON ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_F4_BUTTON ].gpio.pinA            = 21;
    dMap.map[ RNUM_F4_BUTTON ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_F4_BUTTON ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_ENCODER_BUTTON ].type            = CDC_RT_GPIO;
    dMap.map[ RNUM_ENCODER_BUTTON ].gpio.pinA       = 14;
    dMap.map[ RNUM_ENCODER_BUTTON ].gpio.pinB       = UNDEFINED_PIN;
    dMap.map[ RNUM_ENCODER_BUTTON ].gpio.pinMode    = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_ENCODER_A ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_ENCODER_A ].gpio.pinA            = 12;
    dMap.map[ RNUM_ENCODER_A ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_ENCODER_A ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.map[ RNUM_ENCODER_B ].type                 = CDC_RT_GPIO;
    dMap.map[ RNUM_ENCODER_B ].gpio.pinA            = 13;
    dMap.map[ RNUM_ENCODER_B ].gpio.pinB            = UNDEFINED_PIN;
    dMap.map[ RNUM_ENCODER_B ].gpio.pinMode         = CDC_DIO_IN_PULLUP;

    dMap.options                                    |= NPO_SKIP_NODE_ID_CONFIG | NPO_DEBUG_DURING_SETUP;

    cdcInit( &dMap );
    configureConsoleIO( );
    sleepMillis( 2000 );
}

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
// ??? the pin naming --- confusing ?
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint8_t rStat = ALL_OK;

    setupConfigInfo( );

    printf( "LCS Basic Throttle\n" );

    if ( rStat == ALL_OK ) {

        printf( "Init RuntimeLib " );
        rStat = initRuntime( &dMap );
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup Msg Bus " );
        rStat = printStatus( setupMsgBus( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup UI Elements " );
        rStat = printStatus( setupUIElements( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup Screens " );
        rStat = printStatus( setupScreens( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup Cab Stack " );
        rStat = printStatus( setupCabStack ( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Start Screen\n" );
        registerTaskCallback( UIElements::tick, 10  ); // 10ms tick ?
        UIScreen::setup( );   
    }

    return( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
void startLcsRuntime( ) {

    printf( "BasicThrottle, start runtime\n" );
    startRuntime( );
}

//----------------------------------------------------------------------------------------------------------
// Main. Initialize, register and start the show.
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = initLcsRuntime( );
    if ( rStat == ALL_OK ) startLcsRuntime( );
    return( 0 );
}