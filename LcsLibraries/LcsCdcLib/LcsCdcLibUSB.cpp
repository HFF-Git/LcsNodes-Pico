//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - TinyUSB Implementation
//
//----------------------------------------------------------------------------------------
// Console IO section via UDB. We set up the stdio via the USB connector. As part 
// of the cdcInit call, the console configure call should be done rather early, so
// that we can print out debug messages. In normal LCS node operation there is no 
// USB device connected. Detecting a connection helps to decide whether we can 
// report an error or need to resort to a fatal error call at startup. 
//
// There are two basic ways to detect an USB connection. The first is to simply 
// check if there is power on the USB port. The PICO features an internal GPIO pin
// for this purpose. Using this method still does not mean that we have a computer 
// connected to the USB, but just that there is a cable with power. Well, good 
// enough for us. The second method truly detects that there is a USB host connected.
// This check is provided via the PICO libraries which in turn use the tinyUSB 
// library. However, there could be a timing problem where the USB stack is not 
// ready yet and we conclude wrongly that there is no USB connection. For now, 
// let's rather go with the crude approach to check if there is power on the VBUS
// pin, at the risk that there is just power on the USB connector and no data.
//
// Finally, there is a routine to get a character for the command interfaces. Since 
// the function just reads in a character, optionally with a timeout how long to 
// wait for any input.
//
// PS: The USB check way would be implemented as "return ( stdio_usb_connected( ));" 
// instead of the internal GPIO pin check.
//
// PS: Also, for a gateway style node, two USB channels would be quite useful. One for
// debug and one for an ASCCI style interface, such as the RocNet ASCII interface. 
// The key challenge is how the tinyUSB code is integrated into the PICO. Shielding 
// two channels at the CDC library level is a nightmare, since the PICO stdio layer
// expects a config file only defined at the program level. NOT the library level. 
// It sounds like we need to rely on the CDC library handling channel zero, and do
// some rather tinyUSB low level coding in a gateway. I had hoped to avoid this
// ugly exposure of detail. Additional channels are handled in the firmware that
// use it. Sigh.
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
// sequence so that debug messages can be printed out.
//
//----------------------------------------------------------------------------------------
uint8_t configureUsbIO( ) {

    stdio_init_all( );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Check whether there is a USB connection. The routine checks the VBUS pin
// to see whether there is power on the USB connector. If yes, we assume there 
// is a connection.
//
//----------------------------------------------------------------------------------------
bool usbIsConnected( ) {

    gpio_init( PICO_VBUS_PIN );
    gpio_set_dir( PICO_VBUS_PIN, GPIO_IN );

    return ( gpio_get( PICO_VBUS_PIN ));
}

//----------------------------------------------------------------------------------------
// Get a character from the USB console IO. The routine attempts to read a 
// character from the USB console. If there is no character available, the routine 
// will wait up to the specified timeout value in microseconds. If no character is
// received within the timeout period, the routine returns 0. If timeoutVal is 0, 
// the routine returns immediately.
//
//----------------------------------------------------------------------------------------
char usbIoGetChar( int chan, uint32_t timeoutVal ) {

    // ??? initial test... we just read as before.

    int ch = getchar_timeout_us( timeoutVal );
    return (( ch == PICO_ERROR_TIMEOUT ) ? 0 : ch );
}      

//----------------------------------------------------------------------------------------
// Printf alike routine to the USB console IO. The routine prints formatted data
// to the USB console. The routine returns the number of characters printed.
//
//----------------------------------------------------------------------------------------
int usbIoPrintf( int chan, const char *fmt, ... ) {

    if ( ! usbIsConnected( )) return ( 0 );

    va_list args;
    va_start( args, fmt );
    int len = vsnprintf( outBuffer, sizeof( outBuffer ), fmt, args );
    va_end( args );

    if ( len <= 0 ) return ( 0 );

    printf( "%s", outBuffer );

    return ( len );
}

} // namespace CDC
