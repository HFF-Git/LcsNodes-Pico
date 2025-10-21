//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle
//
//----------------------------------------------------------------------------------------
// This source file contains ...
//
// ??? both throttle could share a throttle lib with common code...
//
//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2024 Helmut Fieres
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

// #include "LcsCdcLib.h"
// #include "LcsRuntimeLib.h"
// #include "LcsUIElements.h"
#include "LcsBasicThrottle.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap      dMap;

//----------------------------------------------------------------------------------------
// Externals.
//
//----------------------------------------------------------------------------------------
extern UIDisplay        *oled;
extern UIEncoder        *encoder;
extern CabStack         *cabStack;
extern CabMsgBus        *msgBus;

extern uint8_t          setupMsgBus( );
extern uint8_t          setupUIElements( );
extern uint8_t          setupScreens( );
extern uint8_t          setupCabStack( );

//----------------------------------------------------------------------------------------
// "printStatus" is a little helper function for the initialization routines protocol
// printing. If there is a serial IO, these routines will list the success of the 
// particular setup operation.
//
//----------------------------------------------------------------------------------------
uint8_t printStatus( uint8_t status ) {

    if ( status == ALL_OK ) printf( "-> OK\n" );
    else                    printf( "-> FAILED: %d\n", status );
       
    return ( status );
}

//----------------------------------------------------------------------------------------
// Setup the resource configuration data and the CDC library.
//
//----------------------------------------------------------------------------------------
void setupConfigInfo( ) {

    dMap = LCS_BASIC_THROTTLE_BOARD_DESC_B_02_00;
   // dMap.options |= NPO_SKIP_NODE_ID_CONFIG | NPO_DEBUG_DURING_SETUP;

    cdcInit( &dMap );
    configureConsoleIO( );
    sleepMillis( 2000 );
}

//----------------------------------------------------------------------------------------
// Init the throttle runtime.
//
//----------------------------------------------------------------------------------------
uint8_t initThrottle( ) {

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

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
void startThrottle( ) {

    printf( "BasicThrottle, start runtime\n" );
    startRuntime( );
}

//----------------------------------------------------------------------------------------
// Main. Initialize, register and start the show.
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = initThrottle( );
    if ( rStat == ALL_OK ) startThrottle( );
    return( 0 );
}