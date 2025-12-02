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
#include "tusb.h"

// ??? add to the RocRail side source file ?

// --------------------------------------------------------------------
// Device Descriptor
// --------------------------------------------------------------------

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCafe,
    .idProduct          = 0x4001,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 1
};

uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

// --------------------------------------------------------------------
// Configuration Descriptor with ONE CDC interface
// --------------------------------------------------------------------

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define EPNUM_CDC_NOTIF 0x83

uint8_t const desc_configuration[] =
{
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUD_CONFIG_DESC_LEN +
        TUD_CDC_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // CDC Channel 1
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;
    return desc_configuration;
}

// --------------------------------------------------------------------
// Strings
// --------------------------------------------------------------------

char const *string_desc_arr[] =
{
    (const char[]) { 0x09, 0x04 }, // LangID
    "Helmut",                       // Manufacturer
    "CDC Channel 1",                // Product
    "123456",                       // Serial number
    "CDC Interface"                 // CDC Interface string
};

static uint16_t _desc_str[32];

uint16_t const * tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;

    uint8_t chr_count;

    if (index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}
