//------------------------------------------------------------------------------------------------------------
//
// LCS - Diesel Throttle
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
//
// ??? both throttle should share a throttle lib with common code...
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

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

using namespace LCS;

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg;

//----------------------------------------------------------------------------------------------------------
// "printStatus" is a little helper function for the initialization routines protocol printing. 
//
//----------------------------------------------------------------------------------------------------------
uint8_t printStatus( uint8_t status ) {

    if ( status == ALL_OK ) printf( "-> OK" );
    else                    printf( "-> FAILED: %d\n", status );
       
    return ( status );
}

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values. 
//
//----------------------------------------------------------------------------------------------------------
uint8_t setupConfigInfo( CDC::CdcPinConfig *cfg ) {

    printf( "Setup Config Info\n" );

    *cfg = CDC::getConfigDefault( );

    cfg -> ADC_PIN_0             = 26;
    cfg -> ADC_PIN_1             = 27;

    cfg -> PWM_PIN_0             = 20;
    cfg -> PWM_PIN_1             = 21;

    cfg -> PFAIL_PIN             = 7;
    cfg -> EXT_INT_PIN           = 22;
    cfg -> READY_LED_PIN         = 14;
    cfg -> ACTIVE_LED_PIN        = 15;

    cfg -> DIO_PIN_0             = 9;
    cfg -> DIO_PIN_1             = 8;
    cfg -> DIO_PIN_2             = 10;
    cfg -> DIO_PIN_3             = 11;
    cfg -> DIO_PIN_4             = 21;
    cfg -> DIO_PIN_5             = 20;
    cfg -> DIO_PIN_6             = 19;
    cfg -> DIO_PIN_7             = 18;

    cfg -> NVM_I2C_SCL_PIN       = 17;
    cfg -> NVM_I2C_SDA_PIN       = 16;
    cfg -> NVM_I2C_ADR_ROOT      = 0x50;

    cfg -> EXT_I2C_SCL_PIN       = 3;
    cfg -> EXT_I2C_SDA_PIN       = 2;
    cfg -> EXT_I2C_ADR_ROOT      = 0x50;

    CDC::printConfigInfo( cfg );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint8_t rStat = ALL_OK;

    printf( "LCS Diesel Cab Throttle\n" );

    rStat = setupConfigInfo( &cfg );
    
    if ( rStat == ALL_OK ) {

        printf( "Setup Msg Bus " );
        // rStat = printStatus( setupMsgBus( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup UI Elements " );
        // rStat = printStatus( setupUIElements( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup Screens " );
        // rStat = printStatus( setupScreens( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Setup Cab Stack " );
        // rStat = printStatus( setupCabStack ( ));
    }

    if ( rStat == ALL_OK ) {

        printf( "Ready..." );
        // UIScreen::setup( );    
    }

    return( rStat );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {


    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t startLcsRuntime( ) {


    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = ALL_OK;
    if ( rStat == ALL_OK ) rStat = initLcsRuntime( );
    if ( rStat == ALL_OK ) rStat = registerCallbacks( );
    if ( rStat == ALL_OK ) rStat = startLcsRuntime( );
    return( ALL_OK );
}