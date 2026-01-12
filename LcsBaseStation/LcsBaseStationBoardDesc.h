//----------------------------------------------------------------------------------------
//
// LCS - Base Station Board Descriptor File
//
//----------------------------------------------------------------------------------------
// The base station descriptor file contains the definitions for the hardware 
// configuration values of a base station board.
// 
//----------------------------------------------------------------------------------------
//
// LCS - Base Station Board Descriptor File
// Copyright (C) 2020 - 2026  Helmut Fieres
//
/// This program is free software: you can redistribute it and/or modify it under 
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
// Board Descriptor for Base Station Controller Board.
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// Defined Resource Numbers for the base station specific hardware resources.
//
//----------------------------------------------------------------------------------------
const uint8_t RNUM_TIMER_0      = CDC_RN_FIRST_USER_RN + 0;
const uint8_t RNUM_TIMER_1      = CDC_RN_FIRST_USER_RN + 1;

const uint8_t RNUM_ENABLE_MAIN  = CDC_RN_FIRST_USER_RN + 2;
const uint8_t RNUM_CONTROL_MAIN = CDC_RN_FIRST_USER_RN + 3;
const uint8_t RNUM_ADC_MAIN     = CDC_RN_FIRST_USER_RN + 4;

const uint8_t RNUM_ENABLE_PROG  = CDC_RN_FIRST_USER_RN + 5;
const uint8_t RNUM_CONTROL_PROG = CDC_RN_FIRST_USER_RN + 6;
const uint8_t RNUM_ADC_PROG     = CDC_RN_FIRST_USER_RN + 7;

const uint8_t RNUM_UART_RX_MAIN = CDC_RN_FIRST_USER_RN + 8;
const uint8_t RNUM_UART_RX_PROG = CDC_RN_FIRST_USER_RN + 9;

const uint8_t RNUM_DIO_0        = CDC_RN_FIRST_USER_RN + 10;
const uint8_t RNUM_DIO_1        = CDC_RN_FIRST_USER_RN + 11;

//----------------------------------------------------------------------------------------
// Board descriptor for Board Version: B.01.00
//
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BASE_STATION_BOARD_DESC_B_01_00 = {

    //------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //------------------------------------------------------------------------------------
    .boardInfo      = CDC_BT_BASE_STATION, 
    .boardCtrlInfo  = CDC_CF_RP_PICO,
    .boardVersion   = (( 2U << 8 ) | 0 ),  
    .boardName      = "LCS_BASE_STATION_BOARD_DESC_B_01_00",

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
            .gpio { .pinA = 5, .pinB = UNDEFINED_PIN, 
                    .pinMode = CDC_DIO_IN_PULLUP }   
        },

        {   .type = CDC_RT_TIMER, .resId = RNUM_TIMER_0,
            .timer { .timerVal = 0 }
        },

        {   .type = CDC_RT_TIMER, .resId = RNUM_TIMER_1,   
            .timer { .timerVal = 0 }           
        },

        {   .type = CDC_RT_CAN_BUS, .resId = CDC_RN_CAN_BUS,
            .can {  .rxPin = 0, .txPin = 1, .baudRate = 125000, .twoCores = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c { .sclPin = 3, .sdaPin = 2, .baudRate = 100000, .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c { .sclPin = 17, .sdaPin = 16, .baudRate = 100000, .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENABLE_MAIN,
            .gpio { .pinA = 6, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_CONTROL_MAIN,
            .gpio { .pinA = 21, .pinB = 20, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_MAIN,
            .adc { .adcPin = 26, .adcNum = 0 }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_MAIN,
            .uart { .rxPin = 13, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENABLE_PROG,
            .gpio { .pinA = 7, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_CONTROL_PROG,
            .gpio { .pinA = 19, .pinB = 18, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_PROG,
            .adc { .adcPin = 27, .adcNum = 1 }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_PROG,
            .uart { .rxPin = 9, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        }
    }
};

//----------------------------------------------------------------------------------------
// Board descriptor for Board Version: B.02.00
//
//
// ???? to be reviewed !!!!
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BASE_STATION_BOARD_DESC_B_02_00 = {

    //------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //------------------------------------------------------------------------------------
    .boardInfo      = CDC_BT_BASE_STATION, 
    .boardCtrlInfo  = CDC_CF_RP_PICO,
    .boardVersion   = (( 1U << 8 ) | 0 ),  
    .boardName      = "LCS_BASE_STATION_BOARD_DESC_B_02_00",

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
            .gpio { .pinA = 5, .pinB = UNDEFINED_PIN, 
                    .pinMode = CDC_DIO_IN_PULLUP }   
        },

        {   .type = CDC_RT_TIMER, .resId = RNUM_TIMER_0,
            .timer { .timerVal = 0 }
        },

        {   .type = CDC_RT_TIMER, .resId = RNUM_TIMER_1,   
            .timer { .timerVal = 0 }           
        },

        {   .type = CDC_RT_CAN_BUS, .resId = CDC_RN_CAN_BUS,
            .can {  .rxPin = 0, .txPin = 1, .baudRate = 125000, .twoCores = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c { .sclPin = 3, .sdaPin = 2, .baudRate = 100000, .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c { .sclPin = 17, .sdaPin = 16, .baudRate = 100000, .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENABLE_MAIN,
            .gpio { .pinA = 6, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_CONTROL_MAIN,
            .gpio { .pinA = 21, .pinB = 20, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_MAIN,
            .adc { .adcPin = 26, .adcNum = 0 }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_MAIN,
            .uart { .rxPin = 13, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENABLE_PROG,
            .gpio { .pinA = 7, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_CONTROL_PROG,
            .gpio { .pinA = 19, .pinB = 18, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_PROG,
            .adc { .adcPin = 27, .adcNum = 1 }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_PROG,
            .uart { .rxPin = 9, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_DIO_0,
            .gpio { .pinA = 8, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_DIO_0,
            .gpio { .pinA = 12, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP } 
        }
    }
};
