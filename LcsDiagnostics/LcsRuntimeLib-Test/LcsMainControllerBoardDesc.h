//------------------------------------------------------------------------------------------------------------
//
// LCS - Main Controller Board Descriptor File
//
//------------------------------------------------------------------------------------------------------------
// The main controller descriptor file contains the definitions for the hardware configuration values of a 
// main controller board.
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
#ifndef LcsDccMonitorBoardDesc_h
#define LcsDccMonitorBoardDesc_h
 
#include "LcsCdcLib.h"

using namespace CDC;

//----------------------------------------------------------------------------------------------------------
// Setup the configuration of the HW board. The CDC resource descriptor map contains the configuration 
// data for the board. In addition, the HW pins for I2C, analog inputs and so on are set from the current
// RPico Defaults. Check the schematic for the board to see all pin assignments.
//
//----------------------------------------------------------------------------------------------------------
const uint8_t RNUM_ADC_0    = CDC_RN_FIRST_USER_RN + 0;
const uint8_t RNUM_ADC_1    = CDC_RN_FIRST_USER_RN + 1;

const uint8_t RNUM_DIO_0    = CDC_RN_FIRST_USER_RN + 2;
const uint8_t RNUM_DIO_1    = CDC_RN_FIRST_USER_RN + 3;
const uint8_t RNUM_DIO_2    = CDC_RN_FIRST_USER_RN + 4;
const uint8_t RNUM_DIO_3    = CDC_RN_FIRST_USER_RN + 5;
const uint8_t RNUM_DIO_4    = CDC_RN_FIRST_USER_RN + 6;
const uint8_t RNUM_DIO_5    = CDC_RN_FIRST_USER_RN + 7;
const uint8_t RNUM_DIO_6    = CDC_RN_FIRST_USER_RN + 8;
const uint8_t RNUM_DIO_7    = CDC_RN_FIRST_USER_RN + 9;
const uint8_t RNUM_DIO_8    = CDC_RN_FIRST_USER_RN + 10;
const uint8_t RNUM_DIO_9    = CDC_RN_FIRST_USER_RN + 11;
const uint8_t RNUM_DIO_10   = CDC_RN_FIRST_USER_RN + 12;
const uint8_t RNUM_DIO_11   = CDC_RN_FIRST_USER_RN + 13;

const uint8_t RNUM_PWM_0    = CDC_RN_FIRST_USER_RN + 14;
const uint8_t RNUM_PWM_1    = CDC_RN_FIRST_USER_RN + 15;

const uint8_t RNUM_DIO_P_0  = CDC_RN_FIRST_USER_RN + 16;
const uint8_t RNUM_DIO_P_1  = CDC_RN_FIRST_USER_RN + 17;
const uint8_t RNUM_DIO_P_2  = CDC_RN_FIRST_USER_RN + 18;
const uint8_t RNUM_DIO_P_3  = CDC_RN_FIRST_USER_RN + 19;

const uint8_t RNUM_PWM_P_0  = CDC_RN_FIRST_USER_RN + 20;


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
const CdcResourceDescMap LCS_MAIN_CONTROLLER_BOARD_DESC_B_02_00 = {

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
    .name                       = "LCS_DCC_MONITOR_BOARD_DESC_B_02_00",

    //--------------------------------------------------------------------------------------------------------
    // The resource map. It is a simple array of resource entries. The values set reflect the board for which 
    // the resources are defined.
    // 
    //--------------------------------------------------------------------------------------------------------
    .map {

        {   .type = CDC_RT_GPIO, .resId = CDC_RN_ACTIVITY_LED,
            .gpio { .pinA = 15, .pinB = UNDEFINED_PIN,  .pinMode = CDC_DIO_OUT }   
        },

        {   .type = CDC_RT_GPIO, .resId = CDC_RN_PFAIL,
            .gpio { .pinA = UNDEFINED_PIN, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN_PULLUP }   
        },

        {   .type = CDC_RT_CAN_BUS, .resId = CDC_RN_CAN_BUS,
            .can {  .rxPin = 0, .txPin = 1, .baudRate = 125000, .twoCores   = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c {  .sclPin = 3, .sdaPin = 2, .baudRate = 100000, .i2cTimeoutMs = 25 }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c {  .sclPin = 17, .sdaPin = 16, .baudRate = 100000, .i2cTimeoutMs = 25 }  
        },

        {
            .type = CDC_RT_ADC, .resId = RNUM_ADC_0,
            .adc { .adcPin = 26, .adcNum = 0 }
        },

        {
            .type = CDC_RT_ADC, .resId = RNUM_ADC_1,
            .adc { .adcPin = 27, .adcNum = 1 }
        },
        
        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_0,
            .gpio { .pinA = 8, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_1,
            .gpio { .pinA = 9, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_2,
            .gpio { .pinA = 10, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },
    
        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_3,
            .gpio { .pinA = 11, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_4,
            .gpio { .pinA = 21, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },
    
        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_5,
            .gpio { .pinA = 20, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },
    
        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_6,
            .gpio { .pinA = 19, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_7,
            .gpio { .pinA = 18, .pinB = UNDEFINED_PIN, .pinMode = CDC_DIO_IN }
        },
    
        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_P_0,
            .gpio { .pinA = 8, .pinB = 9, .pinMode = CDC_DIO_IN }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_P_1,
            .gpio { .pinA = 10, .pinB = 11, .pinMode = CDC_DIO_IN }
        },

        {   
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_P_2,
            .gpio { .pinA = 21, .pinB = 20, .pinMode = CDC_DIO_IN }
        },
    
        {   
            .type = CDC_RT_GPIO, .resId = RNUM_DIO_P_3,
            .gpio { .pinA = 19, .pinB = 18, .pinMode = CDC_DIO_IN }
        },
    
        {   
            .type = CDC_RT_PWM, .resId = RNUM_PWM_0,
            .pwm { .pinA = 20, .pinB = UNDEFINED_PIN, .frequency = 100 }
        },

        {   
            .type = CDC_RT_PWM, .resId = RNUM_PWM_1,
            .pwm { .pinA = 21, .pinB = UNDEFINED_PIN, .frequency = 100 }
        },

        {   
            .type = CDC_RT_PWM, .resId = RNUM_PWM_P_0,
            .pwm { .pinA = 20, .pinB = 21, .frequency = 100 }
        }
    }
};

#endif