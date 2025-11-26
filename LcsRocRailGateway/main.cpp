//----------------------------------------------------------------------------------------
//
// LCS - Diesel Throttle
//
//----------------------------------------------------------------------------------------
// This source file contains ...
//
//
// ??? both throttle should share a throttle lib with common code...
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
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsRocRailGateway.h"
#include "LcsRocRailGatewayBoardDesc.h"
#include "tusb.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap dMap;

// ??? need a routine to print to the USB debug channel....
//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void debugPrintf( const char *fmt, ... ) {

    char buf[128];
    va_list args;

    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // ??? interferes with our console IO ?
    tud_cdc_n_write_str( USB_INTERFACE_DEBUG, buf );
    tud_cdc_n_write_flush( USB_INTERFACE_DEBUG );
}

//----------------------------------------------------------------------------------------
// "errStat" is a little helper function for the initialization routines protocol
// printing. If there is a serial IO, these routines will list the success of the 
// particular setup operation.
//
//----------------------------------------------------------------------------------------
uint8_t errStat( uint8_t errId, char *msg ) {

    if ( errId == 0 ) debugPrintf( "%s -> OK\n", msg );
    else              debugPrintf( "%s -> %d\n", msg, errId );
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
    debugPrintf( "RocNet Msg: %s\n", line );

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

    debugPrintf( "LCS message Callback\n" );

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

    while ( true ) {

        tud_task();

        if (tud_cdc_n_available( USB_INTERFACE_ROCNET)) {
            char ch = tud_cdc_n_read_char( USB_INTERFACE_ROCNET );

        }
    }
    
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








#if 0 


    // ??? this is a write which works for all channels use in CDC ?

    tud_cdc_n_write_str(CDC1, "@1 OK\n"); 
    tud_cdc_n_write_flush(CDC1);


    // ??? this is a read 

    if (tud_cdc_n_available(CDC1)) {
        char ch = tud_cdc_n_read_char(CDC1);

    // ??? we need a tudTask( ); to advance the state machines...
    // ??? this is a res with tudTask( ); incorporated. Important.
    
    tud_task() ; 
    if (tud_cdc_n_available(CDC1)) {
        char ch = tud_cdc_n_read_char(CDC1);
    }


    // ??? offer a generic printf alike routine in CDC

    int cdcPrintf( int cdc, const char *fmt, ... ) {

    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf( buf, sizeof(buf), fmt, args );
    va_end(args);

    tud_cdc_n_write( cdc, buf, n );
    tud_cdc_n_write_flush( cdc );
}

    

#endif