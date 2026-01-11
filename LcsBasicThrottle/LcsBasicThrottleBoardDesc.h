//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Board Descriptor File
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
//----------------------------------------------------------------------------------------
#ifndef LcsBasicThrottleBoardDesc_h
#define LcsBasicThrottleBoardDesc_h
 
#include "LcsCdcLib.h"

using namespace CDC;

//----------------------------------------------------------------------------------------
// The button and switch assignments for the Cab Handheld Development Platform.
// The current handheld is a board based on the PICO platform. All buttons, switches 
// and encoders are directly connected to the PICO's GPIO pins. The CDC resource 
// descriptor map contains the configuration data for the board.
//
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
const uint8_t RNUM_ENCODER_KNOB     = CDC_RN_FIRST_USER_RN + 13;

//----------------------------------------------------------------------------------------
// Each board is described by a resource descriptor, which contains information 
// about the hardware family, controller type, controller attributes and hardware 
// resources available on the board. A resource itself described the actual 
// hardware entity that is available. It the resource primarily maps the hardware
// pins and their function. A GPIO pin and whether it is input output pin is a 
// typical example for such a resource. A resource entry in the resource map has
// a type and unique Id and the attributes for the particular resource type. The
// order in the map does not matter, but when accessing the resource, the array 
// index is used. Applications need to map resource entries to their index. The 
// CDC library provides support for this mapping.
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
    .boardName      = "LCS_BASIC_THROTTLE_BOARD_DESC_B_02_00",
            
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
            .gpio { .pinA = UNDEFINED_PIN, 
                    .pinB = UNDEFINED_PIN, 
                    .pinMode = CDC_DIO_IN_PULLUP }   
        },

        {   .type = CDC_RT_CAN_BUS, .resId = CDC_RN_CAN_BUS,
            .can {  .rxPin = 0, .txPin = 1, .baudRate = 125000, .twoCores = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c {  .sclPin = 3, 
                    .sdaPin = 2, 
                    .baudRate = 100000, 
                    .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c { .sclPin = 17, 
                   .sdaPin = 16, 
                   .baudRate = 100000, 
                   .i2cTimeoutMs = 25 }
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

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENCODER_KNOB,
            .gpio { .pinA = 12, .pinB = 13, .pinMode = CDC_DIO_IN_PULLUP } 
        }
    }
};

#endif