//----------------------------------------------------------------------------------------
//
// LCS - Diesel Throttle Board Descriptor File
//
//----------------------------------------------------------------------------------------
// The base station descriptor file contains the definitions for the hardware 
// configuration values of a basic throttle board.
// 
//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Board Descriptor File
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
// The button and switch assignments for the Cab Handheld Development Platform. 
// The current handheld is a board based on the PICO platform. All buttons, switches 
// and encoders are directly connected to the PICO GPIO pins. The CDC resource 
// descriptor map contains the configuration data for the board. In addition, the
// HW pins for I2C, analog inputs and so on are set from the current RPico Defaults.
// Check the schematic for the board to see all pin assignments.
//
// One day we will have several handheld versions. Although they will perhaps differ,
// their the CDC resource names used should not change. 
//----------------------------------------------------------------------------------------
const uint8_t RNUM_MENU_BUTTON      = CDC_RN_FIRST_USER_RN + 0;
const uint8_t RNUM_SELECT_BUTTON    = CDC_RN_FIRST_USER_RN + 1;
const uint8_t RNUM_UP_BUTTON        = CDC_RN_FIRST_USER_RN + 2;
const uint8_t RNUM_DOWN_BUTTON      = CDC_RN_FIRST_USER_RN + 3;
const uint8_t RNUM_HORN_BUTTON      = CDC_RN_FIRST_USER_RN + 4;
const uint8_t RNUM_BELL_BUTTON      = CDC_RN_FIRST_USER_RN + 5;
const uint8_t RNUM_FWD_BUTTON       = CDC_RN_FIRST_USER_RN + 6;
const uint8_t RNUM_REV_BUTTON       = CDC_RN_FIRST_USER_RN + 7;
const uint8_t RNUM_F1_BUTTON        = CDC_RN_FIRST_USER_RN + 8;
const uint8_t RNUM_F2_BUTTON        = CDC_RN_FIRST_USER_RN + 9;
const uint8_t RNUM_F3_BUTTON        = CDC_RN_FIRST_USER_RN + 10;
const uint8_t RNUM_F4_BUTTON        = CDC_RN_FIRST_USER_RN + 11;
const uint8_t RNUM_ENCODER_BUTTON   = CDC_RN_FIRST_USER_RN + 12;
const uint8_t RNUM_ENCODER_A        = CDC_RN_FIRST_USER_RN + 13;
const uint8_t RNUM_ENCODER_B        = CDC_RN_FIRST_USER_RN + 14;

//----------------------------------------------------------------------------------------
// Board Descriptor for Basic Throttle Controller Board Version: B.02.00.
//
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BASIC_THROTTLE_BOARD_DESC_B_02_00 = {

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
        },
        
        {   .type = CDC_RT_GPIO, .resId = RNUM_MENU_BUTTON,
            .gpio { .pinA = 6, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_SELECT_BUTTON,
            .gpio { .pinA = 8, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_UP_BUTTON,
            .gpio { .pinA = 7, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_DOWN_BUTTON,
            .gpio { .pinA = 9, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_HORN_BUTTON,
            .gpio { .pinA = 22, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_BELL_BUTTON,
            .gpio { .pinA = 15, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_FWD_BUTTON,
            .gpio { .pinA = 10, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_REV_BUTTON,
            .gpio { .pinA = 11, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_F1_BUTTON,
            .gpio { .pinA = 18, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_F2_BUTTON,
            .gpio { .pinA = 19, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_F3_BUTTON,
            .gpio { .pinA = 20, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_F4_BUTTON,
            .gpio { .pinA = 21, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENCODER_BUTTON,
            .gpio { .pinA = 14, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENCODER_A,
            .gpio { .pinA = 12, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENCODER_B,
            .gpio { .pinA = 13, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        }
    }
};