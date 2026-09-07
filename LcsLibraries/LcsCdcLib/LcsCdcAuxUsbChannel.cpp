//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Auxiliary USB Channel
//
//----------------------------------------------------------------------------------------
// Some LCS applications need a second USB channel. The base station or gateways
// may be connected to a computer to exchange data packets. While this is also 
// possible via the USB console interface, leaving the console available for 
// commands and debug options requires a second USB channel.
//
// The PICO uses the tinyUSB library for USB. We build the second channel also
// using this library.
// 
//----------------------------------------------------------------------------------------
//
// LCS - RocRail Gateway - CanBus Message Interface
// Copyright (C) 2020 - 2026  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

//----------------------------------------------------------------------------------------
// Compile-time checks ensuring TinyUSB is enabled and configured properly.
// These prevent subtle build errors when project configuration is wrong.
//----------------------------------------------------------------------------------------
#ifndef CFG_TUD_CDC
#error "tusb_config.h was NOT loaded!"
#endif

#if !CFG_TUD_ENABLED
#error "TinyUSB device stack is NOT enabled!"
#endif

#if CFG_TUD_CDC != 1
#error "Expected exactly 1 CDC interfaces!"
#endif

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

// using namespace CDC;

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

// Interface numbers for the CDC function
enum {

    ITF_NUM_CDC = 0,       // CDC control interface
    ITF_NUM_CDC_DATA,      // CDC data interface
    ITF_NUM_TOTAL          // Total number of interfaces
};

// Endpoint addresses:
//   OUT  endpoint = 0x02
//   IN   endpoint = 0x82  (0x80 = IN direction)
//   NOTIFICATION endpoint = 0x83
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define EPNUM_CDC_NOTIF 0x83

// Full configuration descriptor blob
// Includes:
//   - Configuration descriptor
//   - CDC interface descriptors
//   - CDC endpoints
uint8_t const desc_configuration[ ] = {

    // Top-level configuration descriptor
    TUD_CONFIG_DESCRIPTOR(
        1,                      // Configuration number
        ITF_NUM_TOTAL,          // Number of interfaces
        0,                      // String index
        TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN,    // Total size
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,        // Attributes
        100),                   // Max power (in 2 mA units → 200 mA)

    // CDC interface descriptor set
    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC,            // Control interface number
        4,                      // String index
        EPNUM_CDC_NOTIF, 8,     // Notification EP + packet size
        EPNUM_CDC_OUT,          // Data OUT
        EPNUM_CDC_IN,           // Data IN
        64)                     // Data endpoint packet size
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
char const *string_desc_arr[ ] = {

    (const char[]) { 0x09, 0x04 }, // Language ID: English (0x0409)
    "LcsNodes",                    // Manufacturer
    "Rocnet - USB Channel 1",      // Product
    "B.00.01",                     // Serial number
    "RocNet Interface"             // Additional descriptor
};

static uint16_t _desc_str[ 32 ];  // UTF-16 buffer for TinyUSB to return

} // namespace

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

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
// "configureAuxUsbChannel" sets up the USB channel.
//
//----------------------------------------------------------------------------------------
uint8_t configureAuxUsbChannel( ) {

    tusb_init( );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// TinyUSB internal state machine. We need to call this routine often to advance
// the USB state machine.
//
// ??? isn't that already done through the PICO USB console IO functions ?
//----------------------------------------------------------------------------------------
void auxUsbChannelTick( ) {
 
    tud_task( );
}

//----------------------------------------------------------------------------------------
// "auxChannelAvailable" checks if there is data available on the USB channel.
//
//----------------------------------------------------------------------------------------
bool auxChannelAvailable( ) {

    return ( tud_cdc_available( ));   
}

//----------------------------------------------------------------------------------------
// "auxUsbChannelReadChar" reads one character from the USB channel.
//
//----------------------------------------------------------------------------------------
int  auxUsbChannelReadChar( ) {

    if ( ! tud_cdc_n_available( 1 )) return ( -1 );
    
    return ( tud_cdc_n_read_char( 1 ));
}

//----------------------------------------------------------------------------------------
// "auxUsbChannelWrite" writes a data buffer to the USB channel.
//
//----------------------------------------------------------------------------------------
uint32_t  auxUsbChannelWrite( const void *data, uint32_t len ) {

    if ( tud_cdc_connected( )) {

         tud_cdc_n_write( 1, data, len );
         tud_cdc_n_write_flush( 1 );
    }

    return( len );
}

//----------------------------------------------------------------------------------------
// "auxUsbChannelPrintf" implements a printf style output function.
//
//----------------------------------------------------------------------------------------
uint32_t auxUsbChannelPrintf( int chan, const char *fmt, ... ) {

     if ( ! tud_cdc_connected( )) return( 0 );

    va_list args;
    va_start( args, fmt );
    int len = vsnprintf( outBuffer, sizeof( outBuffer ), fmt, args );
    va_end( args );

    if ( len <= 0 ) return ( 0 );

    return ( auxUsbChannelWrite( outBuffer, len ));
}

} // namespace CDC