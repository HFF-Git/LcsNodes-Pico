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
#include "tusb_config.h"
#include "tusb.h"
#include "LcsUsbChannel1.h"

// ??? this could be come the file for the RocRail side processing ... ???


#if !CFG_TUD_ENABLED
#error "TinyUSB device stack is NOT enabled!"
#endif

#if !CFG_TUD_CDC
#error "CDC is NOT enabled in TinyUSB!"
#endif

#ifndef CFG_TUD_CDC
#error "tusb_config.h was NOT loaded!"
#endif

#if CFG_TUD_CDC != 1
#error "CDC is DISABLED!"
#endif

// Buffer for incoming data
static uint8_t ch1_rx_buf[256];

// Initialize channel 1 USB
void usb_ch1_init(void)
{
    tusb_init();
}

void usb_ch1_task(void)
{
    tud_task();  // TinyUSB engine
}

// -------------------- CDC API --------------------

bool usb_ch1_available(void)
{
    return tud_cdc_available();
}

int usb_ch1_read(void)
{
    if (tud_cdc_available())
        return tud_cdc_read_char();
    return -1;
}

void usb_ch1_write(const void *data, uint32_t len)
{
    if (tud_cdc_connected())
    {
        tud_cdc_write(data, len);
        tud_cdc_write_flush();
    }
}

void usb_ch1_write_str(const char *s)
{
    usb_ch1_write(s, strlen(s));
}
