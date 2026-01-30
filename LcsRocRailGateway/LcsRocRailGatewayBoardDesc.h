//----------------------------------------------------------------------------------------
//
// LCS - RocRail Gateway Board Descriptor File
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

using namespace CDC;

//----------------------------------------------------------------------------------------
// Each board is described by a resource descriptor, which contains information
// about the hardware family, controller type, controller attributes, and the
// hardware resources available on the board. A resource descriptor represents
// an actual hardware entity and primarily maps hardware pins to their assigned
// functions. The order of the resources in the map does not matter.
//
// A typical example of a resource is a GPIO pin, including whether it is
// configured as an input or output. Each resource entry in the resource map
// has a type, a unique ID, and a set of attributes specific to that resource
// type.
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// Board Descriptor for RocRail Gateway Board Version: B.02.00
//
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_ROCRAIL_GATEWAY_BOARD_DESC_B_02_00 = {

    //------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //------------------------------------------------------------------------------------
    .boardInfo      = CDC_BT_MAIN_CONTROLLER, 
    .boardCtrlInfo  = CDC_CF_RP_PICO,
    .boardVersion   = (( 2U << 8 ) | 0 ),  

    //------------------------------------------------------------------------------------
    // The resource map. It is a simple array of resource entries. The values set
    // reflect the board for which the resources are defined.
    // 
    // ??? to be completed ...
    //------------------------------------------------------------------------------------
    .map {

        {   .type = CDC_RT_GPIO, .resId = CDC_RN_ACTIVITY_LED,
            .gpio { .pinA = 15, .pinB = UNDEFINED_PIN,  .pinMode = CDC_DIO_OUT }   
        },

        {   .type = CDC_RT_GPIO, .resId = CDC_RN_PFAIL,
            .gpio { .pinA = UNDEFINED_PIN, .pinB = UNDEFINED_PIN, 
            .pinMode = CDC_DIO_IN_PULLUP }   
        },

        {   .type = CDC_RT_CAN_BUS, .resId = CDC_RN_CAN_BUS,
            .can { .rxPin = 0, .txPin = 1, .baudRate = 125000, .twoCores   = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c { .sclPin = 3, .sdaPin = 2, .baudRate = 100000, .i2cTimeoutMs = 25 }
        }
    }
};
