//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - TinyUSB Implementation
//
//----------------------------------------------------------------------------------------
// Console IO section via USB. We set up the stdio via the USB connector. As 
// part  of the cdcInit call, the console configure call should be done rather
// early, so that we can print out debug messages. In normal LCS node operation
// there is no  USB device connected. Detecting a connection helps to decide 
// whether we can report an error or need to resort to a fatal error call at 
// startup. 
//
// The CDC library also supports two USB channels. One channel will always be 
// the console port, which we connect to a terminal on a laptop, or so. The 
// other channel is a port used for programs that send and receive text from
// the LCS libraries. We will use this feature for the base station and other
// gateways to connect a computer program to these components. There are many
// layout control systems on the market, which communicate via USB with a 
// base station.
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

// #include "pico/stdio_usb.h"

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;

char outBuffer [ 256 ];

} // namespace

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// Configure the USB console IO. This routine sets up the stdio via the USB
// connector. The routine needs to be called rather early during the setup
// sequence so that debug messages can be printed out. For two USB ports, the
// "tusb_init" routine from the tinyUSB library needs to be called. We also
// initialize the internal GPIO pin as input pin, so we can detect that power
// is present on the USB connector. This is of course not a guarantee that 
// there is actually a console on the other side, but it is a simple test.
//
//----------------------------------------------------------------------------------------
uint8_t configureUsbIO( ) {

    gpio_init( PICO_VBUS_PIN );
    gpio_set_dir( PICO_VBUS_PIN, GPIO_IN );

    // ??? what to do for multi USB-CDC ?
    // tusb_init( );

    stdio_init_all( );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Check whether there is a USB connection. The routine checks the VBUS pin
// to see whether there is power on the USB connector. If yes, we assume there 
// is a connection. Another way would be to ask tinyUSB, bit that may cause 
// timing issues at controller start. We run the risk of just a USB power cable
// and no USB data.
//
//----------------------------------------------------------------------------------------
bool usbIoIsConnected( ) {

    return ( gpio_get( PICO_VBUS_PIN ));
}

//----------------------------------------------------------------------------------------
// "usbIoGetChar" attempts to read a character from the USB console. If there 
// is no character available, the routine returns 0. When we run two USB ports
// we also need to advance the tinyUSB state machines. In the shielded version
// of just one USB port, the PICO SDK libraries take care of all this. Now it 
// is our responsibility. Sigh.
//
//----------------------------------------------------------------------------------------
char usbIoGetChar( int chan ) {

    if ( ! usbIoIsConnected( )) return ( 0 );

    // tud_task();

    if ( chan == 0 ) {

        int ch = getchar_timeout_us( 0 );
        return (( ch == PICO_ERROR_TIMEOUT ) ? 0 : ch );
    }
    else if ( chan == 1 ) {

        // ??? to do ...
        // if ( ! tud_cdc_n_available( 1 )) return ( -1 );
        // return ( tud_cdc_n_read_char( 1 ));
        return( 0 ); // ??? for now ...
    }
    else return( 0 );
}      

//----------------------------------------------------------------------------------------
// "usbIoPrintf" is a printf alike routine to the USB console IO. The routine 
// prints formatted data to the USB console. The routine returns the number of 
// characters printed.
//
//----------------------------------------------------------------------------------------
int usbIoPrintf( int chan, const char *fmt, ... ) {

    if ( ! usbIoIsConnected( )) return ( 0 );

    va_list args;
    va_start( args, fmt );
    int len = vsnprintf( outBuffer, sizeof( outBuffer ), fmt, args );
    va_end( args );

    if ( len <= 0 ) return ( 0 );

    if ( chan == 0 ) {

        len = printf( "%s", outBuffer );
    }
    else {

        // if ( ! tud_cdc_n_connected( 1 )) return( 0 );

        // ???to do ...
        // tud_cdc_n_write( 1, data, len );
        // tud_cdc_n_write_flush( 1 );
        // return( len );
    }
    
    return ( len );
}

} // namespace CDC
