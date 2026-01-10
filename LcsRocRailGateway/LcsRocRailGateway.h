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
// Copyright (C) 2020 - 2026  Helmut Fieres
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
#pragma once
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

using namespace CDC;


// ??? NOTES:

// ??? wee need an AUTO CREATE mode for DCC sessions, Rocrail does not have a 
// concept of sessions.

// ??? We need a callback for LCS messages

// ??? and we need to read in the USB input to assemble a Rocnet message. 

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------



void usb_ch1_init(void);
void usb_ch1_task(void);

bool usb_ch1_available(void);
int  usb_ch1_read(void);

void usb_ch1_write(const void *data, uint32_t len);
void usb_ch1_write_str(const char *s);