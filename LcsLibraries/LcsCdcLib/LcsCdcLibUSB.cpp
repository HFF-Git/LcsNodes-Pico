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
// The CDC library supports two USB channels. One channel will always be the 
// console port, which we connect to a terminal on a laptop, or so. The other
// channel is a port used for programs that send and receive text from the LCS
// libraries. We will use this feature for the base station and other gateways
// to connect a computer program to these components. There are many layout 
// control systems on the market, which communicate via USB with a base station.
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

//----------------------------------------------------------------------------------------
// Compile-time checks ensuring that the TinyUSB stack is enabled and configured
// properly. These prevent subtle build errors when project configuration is 
// wrong.
//----------------------------------------------------------------------------------------
#ifndef CFG_TUD_CDC
#error "tusb_config.h was NOT loaded!"
#endif

#if !CFG_TUD_ENABLED
#error "TinyUSB device stack is NOT enabled!"
#endif

#if CFG_TUD_CDC != 2
#error "Expected exactly 2 CDC interfaces!"
#endif

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// Buffer for incoming data (not yet used for message assembly)
//
//----------------------------------------------------------------------------------------
char outBuffer [ 256 ];

//----------------------------------------------------------------------------------------
// USB Device Descriptor. This descriptor tells the host:
//
//   - What kind of USB device we are
//   - Vendor/product IDs
//   - Class/subclass of the device
//   - USB specification version
//   - Packet size of endpoint 0
//   - String descriptor indices
//
// The host reads this first when enumerating the device.
//----------------------------------------------------------------------------------------
tusb_desc_device_t const desc_device = {

    .bLength            = sizeof(tusb_desc_device_t),   // Size of this structure
    .bDescriptorType    = TUSB_DESC_DEVICE,             // Indicates "device descriptor"
    .bcdUSB             = 0x0200,                       // USB 2.0 compliant

    // USB classes for composite devices using Interface Association Descriptor (IAD)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,       // EP0 packet size
    .idVendor           = 0xCafe,                       // Vendor ID (placeholder)
    .idProduct          = 0x4001,                       // Product ID
    .bcdDevice          = 0x0100,                       // Device version 1.0

    // String descriptor indices
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 1                             // Only 1 configuration
};

//----------------------------------------------------------------------------------------
// USB Configuration Descriptor. This defines:
//
//   - Total size of the configuration
//   - Number of interfaces
//   - Power requirements
//   - Interface descriptors ( CDC control + CDC data )
//   - Endpoint descriptors ( IN, OUT, Notification )
//
// This device exposes exactly ONE CDC channel.
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// Interface numbers for the CDC function
//----------------------------------------------------------------------------------------
enum {
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,

    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,

    ITF_NUM_TOTAL
};

//----------------------------------------------------------------------------------------
// Endpoint addresses:
//----------------------------------------------------------------------------------------
#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

#define EPNUM_CDC_1_NOTIF   0x84
#define EPNUM_CDC_1_OUT     0x05
#define EPNUM_CDC_1_IN      0x85

//----------------------------------------------------------------------------------------
// Descriptor.
// Full configuration descriptor blob
// Includes:
//   - Configuration descriptor
//   - CDC interface descriptors
//   - CDC endpoints
//----------------------------------------------------------------------------------------
#define CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[ ] = {

    TUD_CONFIG_DESCRIPTOR(
        1,                                      // Configuration number
        ITF_NUM_TOTAL,                          // Number of interfaces
        0,                                      // String index
        CONFIG_TOTAL_LEN,                       // Total size
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,     // Attributes
        100 ),                                  // Max power (in 2 mA units → 200 mA)

    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC_0,                          // Control interface number
        4,                                      // String index
        EPNUM_CDC_0_NOTIF, 8,                   // Notification EP + packet size
        EPNUM_CDC_0_OUT,                        // Data OUT
        EPNUM_CDC_0_IN,                         // Data IN
        64),                                    // Data endpoint packet size

    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC_1,
        5,
        EPNUM_CDC_1_NOTIF, 8,
        EPNUM_CDC_1_OUT,
        EPNUM_CDC_1_IN,
        64)
};

//----------------------------------------------------------------------------------------
// USB String Descriptors. USB uses UTF-16 strings. TinyUSB wants an array of C 
// strings, which it then converts to UTF-16 at runtime.
//
// string_desc_arr[] is a table:
//   [0] = language codes (0x0409 = English-US)
//   [1] = Manufacturer string
//   [2] = Product string
//   [3] = Serial Number string
//   [4] = Additional descriptor (here: "RocNet Interface")
//
// _desc_str[ ] is the actual UTF-16 output buffer used by TinyUSB.
//
//----------------------------------------------------------------------------------------
char const *string_desc_arr[] = {

    (const char[]) { 0x09, 0x04 },   // 0: language
    "LcsNodes",                      // 1: manufacturer
    "LcsNodes USB",                  // 2: product
    "B.00.01",                       // 3: serial
    "Console",                       // 4: CDC 0
    "LcsNodes Interface"             // 5: CDC 1
};

static uint16_t _desc_str[ 32 ];  // UTF-16 buffer for TinyUSB to return

} // namespace

//----------------------------------------------------------------------------------------
// TinyUSB callbacks. They are invoked from the tinyUSB library.
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// TinyUSB callback returning the Device Descriptor.
//
//----------------------------------------------------------------------------------------
uint8_t const *tud_descriptor_device_cb( void ) {

    return ((uint8_t const *) &desc_device );
}

//----------------------------------------------------------------------------------------
// TinyUSB callback returning the Configuration Descriptor.
//
//----------------------------------------------------------------------------------------
uint8_t const *tud_descriptor_configuration_cb( uint8_t index ) {

    (void) index;
    return desc_configuration;
}

//----------------------------------------------------------------------------------------
// TinyUSB callback converting ASCII to UTF-16 as needed.
//
//----------------------------------------------------------------------------------------
uint16_t const *tud_descriptor_string_cb( uint8_t index, uint16_t langid ) {

    (void) langid;
    uint8_t chr_count;

    if ( index == 0 ) {
        // Return language ID directly
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else {
        // Convert ASCII → UTF-16 at runtime
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    // UTF-16 string header: descriptor type + total length in bytes
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file. These routines build our
// USB interface.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// Configure the USB console IO. This routine sets up the stdio via the USB
// connector. The routine needs to be called rather early during the setup
// sequence so that debug messages can be printed out. For two USB ports, the
// "tusb_init" routine from the tinyUSB library needs to be called. The 
// "stdio_init_all" tells the PICO SDK that we will use CDC0 as the console
// channel.
//
// We also initialize the internal GPIO pin as input pin, so we can detect that 
// power is present on the USB connector. This is of course not a guarantee that 
// there is actually a console on the other side, but it is a simple test.
//
//----------------------------------------------------------------------------------------
uint8_t configureUsbIO( ) {

    gpio_init( PICO_VBUS_PIN );
    gpio_set_dir( PICO_VBUS_PIN, GPIO_IN );

    tusb_init( );
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
// ??? rather do a channel version ?
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

    tud_task();

    if ( chan == 0 ) {

        int ch = getchar_timeout_us( 0 );
        return (( ch == PICO_ERROR_TIMEOUT ) ? 0 : ch );
    }
    else if ( chan == 1 ) {

        if ( ! tud_cdc_n_available( 1 )) return ( 0 );
        return ( tud_cdc_n_read_char( 1 ));
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

    tud_task( );

    va_list args;
    va_start( args, fmt );
    int len = vsnprintf( outBuffer, sizeof( outBuffer ), fmt, args );
    va_end( args );

    if ( len <= 0 ) return ( 0 );

    if ( chan == 0 ) {

        len = printf( "%s", outBuffer );
    }
    else {

        if ( ! tud_cdc_n_connected( 1 )) return( 0 );

        tud_cdc_n_write( 1, outBuffer, len );
        tud_cdc_n_write_flush( 1 );
        return( len );
    }

    return ( len );
}

} // namespace CDC
