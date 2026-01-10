//----------------------------------------------------------------------------------------
//
// LCS - Block Controller Board Descriptor File
//
//----------------------------------------------------------------------------------------
// The block controller descriptor file contains the definitions for the hardware 
// configuration values of a block controller board.
// 
//----------------------------------------------------------------------------------------
//
// LCS - Base Station Board Descriptor File
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
#ifndef LcsBlockControllerBoardDesc_h
#define LcsBlockControllerBoardDesc_h
 
#include "LcsCdcLib.h"

using namespace CDC;

//----------------------------------------------------------------------------------------
// Setup the configuration of the HW board. The CDC resource descriptor map contains
// the configuration data for the board. In addition, the HW pins for I2C, analog 
// inputs and so on are set from the current RPico Defaults. Check the schematic 
// for the board to see all pin assignments.
//
// One day we will have several block controller versions. Although they will 
// perhaps differ, their the CDC resource names used should not change. 
//----------------------------------------------------------------------------------------
const uint8_t   RNUM_CONTROL_BLK_0    = CDC_RN_FIRST_USER_RN + 0;
const uint8_t   RNUM_ADC_BLK_0        = CDC_RN_FIRST_USER_RN + 1;
const uint8_t   RNUM_UART_RX_0        = CDC_RN_FIRST_USER_RN + 2;

const uint8_t   RNUM_CONTROL_BLK_1    = CDC_RN_FIRST_USER_RN + 3;
const uint8_t   RNUM_ADC_BLK_1        = CDC_RN_FIRST_USER_RN + 4;
const uint8_t   RNUM_UART_RX_1        = CDC_RN_FIRST_USER_RN + 5;

const uint8_t   RNUM_CONTROL_BLK_2    = CDC_RN_FIRST_USER_RN + 6;
const uint8_t   RNUM_ADC_BLK_2        = CDC_RN_FIRST_USER_RN + 7;
const uint8_t   RNUM_UART_RX_2        = CDC_RN_FIRST_USER_RN + 8;

const uint8_t   RNUM_CONTROL_BLK_3    = CDC_RN_FIRST_USER_RN + 9;
const uint8_t   RNUM_ADC_BLK_3        = CDC_RN_FIRST_USER_RN + 10;
const uint8_t   RNUM_UART_RX_3        = CDC_RN_FIRST_USER_RN + 11;

const uint8_t   RNUM_CUT_SIGNAL       = CDC_RN_FIRST_USER_RN + 12;
const uint16_t  PWM_FREQUENCY        = 20000;

//----------------------------------------------------------------------------------------
// Each board is described by a resource descriptor, which contains information 
// about the hardware family, controller type, controller attributes and 
// hardware resources available on the board. A resource itself described the 
// actual hardware entity that is available. It the resource primarily maps the
// hardware pins and their function. A GPIO pin and whether it is input output 
// pin is a typical example for such a resource. A resource entry in the 
// resource map has a type and unique Id and the attributes for the particular
// resource type. The order in the map does not matter, but when accessing the 
// resource, the array index is used. Applications need to map resource entries
// to their index. The CDC library provides support for this mapping.
//
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BLOCK_CONTROLLER_DUAL_BOARD_DESC_B_02_00 = {

    //------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //------------------------------------------------------------------------------------
    .boardInfo      = CDC_BT_BLOCK_CONTROLLER, 
    .boardCtrlInfo  = CDC_CF_RP_PICO,
    .boardVersion   = (( 2U << 8 ) | 0 ),  
    .boardName      = "LCS_BLOCK_CONTROLLER_DUAL_BOARD_DESC_B_02_00",
            
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

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c { .sclPin = 17, .sdaPin = 16, .baudRate = 100000, .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_PWM, .resId = RNUM_CONTROL_BLK_0,
            .pwm { .pinA = 21, .pinB = 20, .frequency = PWM_FREQUENCY } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_BLK_0,
            .adc { .adcPin = 26, .adcNum = 0 }
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_BLK_0,
            .adc { .adcPin = 27, .adcNum = 1 }
        },

        {   .type = RNUM_UART_RX_0, .resId = RNUM_UART_RX_0,
            .uart {  .rxPin = 12, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        },

        {   .type = CDC_RT_PWM, .resId = RNUM_CONTROL_BLK_1,
            .pwm {  .pinA = 19, .pinB = 18, .frequency = PWM_FREQUENCY } 
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_1,
            .uart { .rxPin = 12, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_CUT_SIGNAL,
            .gpio { .pinA = 4, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP }
        }
    }
};

#endif