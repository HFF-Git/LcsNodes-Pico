//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle
//
//----------------------------------------------------------------------------------------
// This source file contains ...
//
// ??? both throttle could share a throttle lib with common code...
//
//
// ??? there are no sessions anymore.
// ??? all we need is an array of cabIds and one cabEntry to manage the actual
// loco.
// ??? when cabId is entered, REQ_LOC asks for the data.
// ??? REP_LOC is stored in the current cabEntry structure.
// ??? refresh is either LOC_KEEP of current loco, or simply another REQ_LOC
// ??? sharing is simply two throttles managing the same loco. There is no
// takeover.
// ??? the base station is the single source of truth. 
// ??? all throttles monitor LCS DCC commands for his current loco to ensure
// that the current loco data are in sync with the base station.
// ??? the base station is unaware of many throttles for the same loco.
// 
//
//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Code - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
#include "LcsBasicThrottle.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap  dMap;
CabStack            *cabStack   = nullptr;
CabMsgBus           *msgBus     = nullptr;

//----------------------------------------------------------------------------------------
// Externals.
//
//----------------------------------------------------------------------------------------
extern uint8_t      setupMsgBus( );
extern uint8_t      setupUIElements( );
extern uint8_t      setupScreens( );
extern uint8_t      setupCabStack( );

// ??? adopt the errStat scheme from runtime lib.
//----------------------------------------------------------------------------------------
// "errStat" is a little helper function for the initialization routines protocol
// printing. If there is a serial IO, these routines will list the success of the 
// particular setup operation.
//
//----------------------------------------------------------------------------------------
uint8_t errStat( uint8_t errId, char *msg ) {

    if ( errId == 0 ) printf( "%s -> OK\n", msg );
    else              printf( "%s -> %d\n", msg, errId );
    return ( errId );
}

//----------------------------------------------------------------------------------------
// Init the throttle runtime.
//
//----------------------------------------------------------------------------------------
uint8_t initThrottle( ) {

    uint8_t rStat = NO_ERR;

    dMap = LCS_BASIC_THROTTLE_BOARD_DESC_B_02_00;

    rStat = errStat( initRuntime( &dMap, 
                                  NPO_SKIP_NODE_ID_CONFIG, 
                                  0 ),
                     (char *) "initRuntime");

    sleepMillis( 2000 );    

    if ( rStat == NO_ERR ) 
        rStat = errStat( setupMsgBus( ), (char *) "setupMsgBus" );

    if ( rStat == NO_ERR ) 
        rStat = errStat( setupUIElements( ), (char *) "setupUIElements" );

    if ( rStat == NO_ERR ) 
        rStat = errStat( setupScreens( ), (char *) "setupScreens" );

    if ( rStat == NO_ERR ) 
        rStat = errStat( setupCabStack( ), (char *) "setupCabStack" );
    
    if ( rStat == NO_ERR ) {

        registerTaskCallback( UIElements::tick, 10  ); // 10ms tick ?
        UIScreen::setup( );   
    }

    return( errStat( rStat, (char *) "initThrottle" ));
}

//----------------------------------------------------------------------------------------
// Once all is configured, we start the runtime.
//
//----------------------------------------------------------------------------------------
void startThrottle( ) {

    printf( "start throttle\n" );
    startRuntime( );
}

//----------------------------------------------------------------------------------------
// Main. Initialize, register and start the show.
//
//----------------------------------------------------------------------------------------
int main( ) {

    if ( initThrottle( ) == NO_ERR ) startThrottle( );
    return( 0 );
}