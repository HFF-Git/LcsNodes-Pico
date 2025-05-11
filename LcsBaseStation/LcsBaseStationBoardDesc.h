//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station Board Descriptor File
//
//------------------------------------------------------------------------------------------------------------
// The base station descriptor file contains the definitions for the hardware configuration values of a base
// station board.
// 
//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station Board Descriptor File
// Copyright (C) 2025 - 2025  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
// Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along with this program. If not, see
// http://www.gnu.org/licenses
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//------------------------------------------------------------------------------------------------------------
#ifndef LcsBaseStationBoardDesc_h
#define LcsBaseStationBoardDesc_h
 
#include "LcsCdcLib.h"

using namespace CDC;

//----------------------------------------------------------------------------------------------------------
// Defined Resource Numbers for the base station specific hardware resources.
//
//----------------------------------------------------------------------------------------------------------
const uint8_t RNUM_ENABLE_MAIN  = CDC_RN_FIRST_USER_RN + 0;
const uint8_t RNUM_CONTROL_MAIN = CDC_RN_FIRST_USER_RN + 1;
const uint8_t RNUM_ADC_MAIN     = CDC_RN_FIRST_USER_RN + 2;

const uint8_t RNUM_ENABLE_PROG  = CDC_RN_FIRST_USER_RN + 3;
const uint8_t RNUM_CONTROL_PROG = CDC_RN_FIRST_USER_RN + 4;
const uint8_t RNUM_ADC_PROG     = CDC_RN_FIRST_USER_RN + 5;

const uint8_t RNUM_UART_RX_MAIN = CDC_RN_FIRST_USER_RN + 6;
const uint8_t RNUM_UART_RX_PROG = CDC_RN_FIRST_USER_RN + 7;

//------------------------------------------------------------------------------------------------------------
// Each board is described by a resource descriptor, which contains information about the hardware family,
// controller type, controller attributes and hardware resources available on the board. A resource itself
// described the actual hardware entity that is available. It the resource primarily maps the hardware 
// pins and their function. A GPIO pin and whether it is input output pin is a typical example for such a
// resource. A resource entry in the resource map has a type and unique Id and the attributes for the 
// particular resource type. The order in the map does not matter, but when accessing the resource, the 
// array index is used. Applications need to map resource entries to their index. The CDC library provides
// support for this mapping.
//
//------------------------------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BASE_STATION_BOARD_DESC_B_02_00 = {

    //--------------------------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //  OPTION              - option flags for the board. They are set by the application.
    //  DEBUG MASK          - debug options. They are set by the application.
    //  CFAMILY             - controller chip family.
    //  CTYPE               - the controller chip.
    //  CPU CORES           - the number of CPU cores in the chip.
    //  MEMORY SIZE         - the main memory size of the controller chip.
    //  EEPROM SIZE         - the non volatile memory size of the controller chip.
    //  WATCHDOG INTERVAL   - the watchdog timer value in milliseconds.
    //  ADC REF VOLTAGE     - the reference voltage for the ADC in milli volt.
    //  ADC DIGIT RANGE     - the range of ADC conversion result. 
    //  NAME                - the board name.
    //
    //--------------------------------------------------------------------------------------------------------
    .options                    = 0,
    .debugMask                  = 0,
    .cFamily                    = CDC_CF_C_UNDEFINED,
    .cType                      = CDC_CF_C_UNDEFINED,
    .cpuCores                   = 1,
    .memorySize                 = 0,
    .eepromSize                 = 0,
    .watchDogIntervallMillis    = 2000,
    .adcRefVoltageMillis        = 3300,
    .adcDigitRange              = 1024,
    .name                       = "LCS_BASE_STATION_BOARD_DESC_B_02_00",

    //--------------------------------------------------------------------------------------------------------
    // The resource map. It is a simple array of resource entries. The values set reflect the board for which 
    // the resources are defined.
    // 
    //--------------------------------------------------------------------------------------------------------
    .map {

        {   .type = CDC_RT_GPIO, .resId = CDC_RN_ACTIVITY_LED,
            .gpio { .pinA = 15, .pinB = UNDEFINED_PIN,  .pinMode = CDC_DIO_OUT }   
        },

        {   .type = CDC_RT_TIMER, .resId = CDC_RN_TIMER_0  
        },

        {   .type = CDC_RT_TIMER, .resId = CDC_RN_TIMER_1              
        },

        {   .type = CDC_RT_GPIO, .resId = CDC_RN_PFAIL,
            .gpio { .pinA = UNDEFINED_PIN, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP }   
        },

        {   .type = CDC_RT_CAN_BUS, .resId = CDC_RN_CAN_BUS,
            .can {  .rxPin = 0, .txPin = 1, .baudRate = 125000, .twoCores = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c {  .sclPin         = 3,
                    .sdaPin         = 2,
                    .baudRate       = 100000,
                    .i2cTimeoutMs   = 25
                 }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c {  .sclPin         = 17,
                    .sdaPin         = 16,
                    .baudRate       = 100000,
                    .i2cTimeoutMs   = 25
                 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENABLE_MAIN,
            .gpio { .pinA = 6, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_CONTROL_MAIN,
            .gpio { .pinA = 21, .pinB = 20, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_MAIN,
            .adc { .pin = 26, .adcNum = 0 }
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_ENABLE_PROG,
            .gpio { .pinA = 6, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_GPIO, .resId = RNUM_CONTROL_PROG,
            .gpio { .pinA = 19, .pinB = 18, .pinMode = CDC_DIO_OUT } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_PROG,
            .adc { .pin = 27, .adcNum = 1 }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_MAIN,
            .uart { .rxPin = 8, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_PROG,
            .uart { .rxPin = 12, .txPin = UNDEFINED_PIN, .baudRate = 250000 }
        }
    }
};

#endif