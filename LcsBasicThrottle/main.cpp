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

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcConfigDesc      cdcConfig;
LCS::LcsConfigDesc      lcsConfig;

//----------------------------------------------------------------------------------------------------------
// Externals.
//
//----------------------------------------------------------------------------------------------------------
extern UIDisplay        *oled;
extern UIEncoder        *encoder;
extern CabStack         *cabStack;
extern CabMsgBus        *msgBus;

extern uint8_t          setupLcsLib( );
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
// Init the CDC and Runtime library...
//
// ??? the pin naming --- confusing ?
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint8_t rStat = ALL_OK;

    cdcConfig = CDC::getConfigDefault( );

    cdcConfig.ADC_PIN_0             = 26;
    cdcConfig.ADC_PIN_1             = 27;

    cdcConfig.PWM_PIN_0             = 20;
    cdcConfig.PWM_PIN_1             = 21;

    cdcConfig.PFAIL_PIN             = 7;
    cdcConfig.EXT_INT_PIN           = 22;
    cdcConfig.READY_LED_PIN         = 14;
    cdcConfig.ACTIVE_LED_PIN        = 15;

    cdcConfig.DIO_PIN_0             = 9;
    cdcConfig.DIO_PIN_1             = 8;
    cdcConfig.DIO_PIN_2             = 10;
    cdcConfig.DIO_PIN_3             = 11;
    cdcConfig. DIO_PIN_4            = 21;
    cdcConfig.DIO_PIN_5             = 20;
    cdcConfig.DIO_PIN_6             = 19;
    cdcConfig.DIO_PIN_7             = 18;

    cdcConfig.NVM_I2C_SCL_PIN       = 3;
    cdcConfig.NVM_I2C_SDA_PIN       = 2;
    cdcConfig.NVM_I2C_ADR_ROOT      = 0x50;

    cdcConfig.EXT_I2C_SCL_PIN       = 17;
    cdcConfig.EXT_I2C_SDA_PIN       = 16;
    cdcConfig.EXT_I2C_ADR_ROOT      = 0x3C;

    cdcConfig.CAN_BUS_RX_PIN        = 0;
    cdcConfig.CAN_BUS_TX_PIN        = 1;
    cdcConfig.CAN_BUS_CTRL_MODE     = CAN_BUS_LIB_PICO_PIO_125K_M_CORE;
    cdcConfig.CAN_BUS_DEF_ID        = 100;

    cdcConfig.NODE_NVM_SIZE         = 8192;
    cdcConfig.EXT_NVM_SIZE          = 512;

    lcsConfig.options               |= NPO_SKIP_NODE_ID_CONFIG | NPO_DEBUG_DURING_SETUP;

    printf( "LCS Basic Throttle\n" );

    if ( rStat == ALL_OK ) {

        printf( "Init runtime, configuration: \n" );
        CDC::printConfigInfo( &cdcConfig );
    }

    if ( rStat == ALL_OK ) {

        printf( "Init RuntimeLib " );
        rStat = initRuntime( &lcsConfig, &cdcConfig );
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

        printf( "Ready..." );
        UIScreen::setup( );   
    }

    return( rStat );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {

    printf( "BasicThrottle, register callbacks\n" );

    registerTaskCallback( UIElements::tick, 10  ); // 10ms tick ?

    return( LCS::ALL_OK );
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

    uint8_t rStat = ALL_OK;

    rStat = initLcsRuntime( );
    if ( rStat == ALL_OK ) registerCallbacks( );
    if ( rStat == ALL_OK ) startLcsRuntime( );
    return( 0 );
}