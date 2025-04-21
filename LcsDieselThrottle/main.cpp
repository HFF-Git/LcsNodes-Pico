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
using namespace CDC;

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CdcResourceMap resMap;

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
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint8_t rStat = ALL_OK;

    printf( "LCS Diesel Cab Throttle\n" );

    resMap = getDefaultResourceMap( );

    resMap.ledPin                   = 15;

    resMap.i2cSclPin_0              = 3;
    resMap.i2cSdaPin_0              = 2;
  //  resMap.NVM_I2C_ADR_ROOT         = 0x50;

    resMap.canPinRx                 = 0;
    resMap.canPinTx                 = 1;
    resMap.canBaudRate              = 125000;
    resMap.canTwoCores              = true;
  
  //  resMap.CAN_BUS_DEF_ID        = 100;

    resMap.extNvmSize               = 32 * 1024;
    

    resMap.options               |= NPO_SKIP_NODE_ID_CONFIG | NPO_DEBUG_DURING_SETUP;


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