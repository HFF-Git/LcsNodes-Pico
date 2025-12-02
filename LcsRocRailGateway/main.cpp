//----------------------------------------------------------------------------------------
//
// LCS - RocRail Gateway
//
//----------------------------------------------------------------------------------------
// 
// 
//----------------------------------------------------------------------------------------
//
// LCS - RocRail Gateway
// Copyright (C) 2025 - 2025  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsRocRailGateway.h"
#include "LcsRocRailGatewayBoardDesc.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap dMap;

//----------------------------------------------------------------------------------------
// "errStat" is a little helper function for the initialization routines protocol
// printing. If there is a serial IO, these routines will list the success of the 
// particular setup operation.
//
//----------------------------------------------------------------------------------------
uint8_t errStat( uint8_t errId, char *msg ) {

    if ( errId == 0 ) usbIoPrintf( 0, "%s -> OK\n", msg );
    else              usbIoPrintf( 0, "%s -> %d\n", msg, errId );
    return ( errId );
}

//----------------------------------------------------------------------------------------
// This routine handles an incoming RocNet message. Assumption: the message is in ASCII
// format. The message starts with "@" followed by an 8byte header and optional data. 
//
// ??? test if we get the data correctly ...
//----------------------------------------------------------------------------------------
void handleRocNetMessage( char *line ) {




    // ??? to be implemented ...
    usbIoPrintf( 0, "RocNet Msg: %s\n", line );

    // ??? analyze the line. it starts with a "@".
    // ??? next an 8byte header. 
    // ??? byte 7 () last byte ) has payload length if required.

    int msgId;

    switch( msgId ) {

        case 0: {

            // ??? 
        } break;

        default: {

        } break;

    }
}

//----------------------------------------------------------------------------------------
// LCS message callback. We register this callback to get the LCS messages and check
// what we need to pass on to RocRail.
//
//----------------------------------------------------------------------------------------
bool lcsMessageCallback( uint8_t *msg ) {

    usbIoPrintf( 0, "LCS message Callback\n" );

    // ??? to be implemented ...
    return ( false );
}

//----------------------------------------------------------------------------------------
// Init ...
//
//----------------------------------------------------------------------------------------
uint8_t initGateway( ) {

    uint8_t rStat = NO_ERR;

    dMap = LCS_ROCRAIL_GATEWAY_BOARD_DESC_B_02_00;

    rStat = errStat( initRuntime( &dMap, NPO_SKIP_NODE_ID_CONFIG, 0 ),
                     (char *) "initGateway");

    // ??? register LCS message callback ...



    // setup the two USB interfaces...
    // ??? does it even have to become a part of runtime lib ?
   

    return( rStat );
}

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
uint8_t runGateway( ) {

    usbIoPrintf( 0, "Run RocRail Gateway\n" );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Here we go ...
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = NO_ERR;
    if ( rStat == NO_ERR ) rStat = initGateway( );
    return( runGateway( ));
}