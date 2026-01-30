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
#pragma once
 
#include "LcsCdcLib.h"

using namespace CDC;

// ??? how do we keep the SW informed about what actual board it runs on ?
// ??? would we need to create two different programs after all ? 

//----------------------------------------------------------------------------------------
// Setup the configuration of the HW board. The CDC resource descriptor map contains
// the configuration data for the board. In addition, the HW pins for I2C, analog 
// inputs and so on are set from the current RPico Defaults. Check the schematic 
// for the board to see all pin assignments.
//
//----------------------------------------------------------------------------------------
const uint8_t   RNUM_CONTROL_BLK_0      = CDC_RN_FIRST_USER_RN + 0;
const uint8_t   RNUM_ADC_BLK_0          = CDC_RN_FIRST_USER_RN + 1;
const uint8_t   RNUM_UART_RX_0          = CDC_RN_FIRST_USER_RN + 2;


const uint8_t   RNUM_CONTROL_BLK_1      = CDC_RN_FIRST_USER_RN + 3;
const uint8_t   RNUM_ADC_BLK_1          = CDC_RN_FIRST_USER_RN + 4;
const uint8_t   RNUM_UART_RX_1          = CDC_RN_FIRST_USER_RN + 5;

const uint8_t   RNUM_CONTROL_BLK_2      = CDC_RN_FIRST_USER_RN + 6;
const uint8_t   RNUM_ADC_BLK_2          = CDC_RN_FIRST_USER_RN + 7;
const uint8_t   RNUM_UART_RX_2          = CDC_RN_FIRST_USER_RN + 8;

const uint8_t   RNUM_CONTROL_BLK_3      = CDC_RN_FIRST_USER_RN + 9;
const uint8_t   RNUM_ADC_BLK_3          = CDC_RN_FIRST_USER_RN + 10;
const uint8_t   RNUM_UART_RX_3          = CDC_RN_FIRST_USER_RN + 11;

const uint8_t   RNUM_CUT_SIGNAL         = CDC_RN_FIRST_USER_RN + 12;
const uint8_t   RNUM_ADC_MUX            = CDC_RN_FIRST_USER_RN + 13;

const uint32_t  DEF_CAN_BUS_BAUDRATE    = 125000;
const uint32_t  DEF_I2C_NVM_BAUDRATE    = 100000;
const uint32_t  DEF_I2C_BUS_BAUDRATE    = 100000;
const uint32_t  DEF_UART_BAUDRATE       = 250000;
const uint16_t  DEF_I2C_TIMEOUT         = 25;
const uint16_t  DEF_PWM_FREQUENCY       = 20000;


//----------------------------------------------------------------------------------------
// The block controller board contains two H-Bridges.
//
//----------------------------------------------------------------------------------------




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
// Dual Block Controller Board Descriptor Version: B.02.00
//
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BLOCK_CONTROLLER_DUAL_BOARD_DESC_B_02_00 = {

    //------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //------------------------------------------------------------------------------------
    .boardInfo      = CDC_BT_BLOCK_CONTROLLER, 
    .boardCtrlInfo  = CDC_CF_RP_PICO,
    .boardVersion   = (( 2U << 8 ) | 0U ),  
   
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
            .can { .rxPin = 0, .txPin = 1, 
                   .baudRate = DEF_CAN_BUS_BAUDRATE, .twoCores   = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c { .sclPin = 3, .sdaPin = 2, 
                   .baudRate = DEF_I2C_NVM_BAUDRATE, .i2cTimeoutMs = DEF_I2C_TIMEOUT }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c { .sclPin = 17, .sdaPin = 16, 
                   .baudRate = DEF_I2C_BUS_BAUDRATE, .i2cTimeoutMs = DEF_I2C_TIMEOUT }
        },

        {   .type = CDC_RT_PWM, .resId = RNUM_CONTROL_BLK_0,
            .pwm { .pinA = 21, .pinB = 20, .frequency = DEF_PWM_FREQUENCY } 
        },

        {   .type = CDC_RT_PWM, .resId = RNUM_CONTROL_BLK_1,
            .pwm {  .pinA = 19, .pinB = 18, .frequency = DEF_PWM_FREQUENCY } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_BLK_0,
            .adc { .adcPin = 26, .adcNum = 0 }
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_BLK_1,
            .adc { .adcPin = 27, .adcNum = 1 }
        },

        {   .type = RNUM_UART_RX_0, .resId = RNUM_UART_RX_0,
            .uart {  .rxPin = 12, .txPin = UNDEFINED_PIN, 
                     .baudRate = DEF_UART_BAUDRATE }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_1,
            .uart { .rxPin = 12, .txPin = UNDEFINED_PIN, 
                    .baudRate = DEF_UART_BAUDRATE }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_CUT_SIGNAL,
            .gpio { .pinA = 4, .pinB = UNDEFINED_PIN, 
                    .pinMode = CDC_DIO_IN_PULLUP }
        }
    }
};

//----------------------------------------------------------------------------------------
// Quad Block Controller Board Descriptor Version: B.02.00
//
//----------------------------------------------------------------------------------------
const CdcResourceDescMap LCS_BLOCK_CONTROLLER_QUAD_BOARD_DESC_B_02_00 = {

    //------------------------------------------------------------------------------------
    // Controller configuration and common data.
    //
    //------------------------------------------------------------------------------------
    .boardInfo      = CDC_BT_BLOCK_CONTROLLER, 
    .boardCtrlInfo  = CDC_CF_RP_PICO,
    .boardVersion   = (( 2U << 8 ) | 10U ),  
   
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
            .can { .rxPin = 0, .txPin = 1, 
                   .baudRate = DEF_CAN_BUS_BAUDRATE, .twoCores   = true }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_NVM,
            .i2c { .sclPin = 3, .sdaPin = 2, 
                   .baudRate = DEF_I2C_NVM_BAUDRATE, .i2cTimeoutMs = DEF_I2C_TIMEOUT }
        },

        {   .type = CDC_RT_I2C, .resId = CDC_RN_EXT_NVM,
            .i2c { .sclPin = 17, .sdaPin = 16, 
                   .baudRate = DEF_I2C_BUS_BAUDRATE, .i2cTimeoutMs = DEF_I2C_TIMEOUT }
        },

        {   .type = CDC_RT_PWM, .resId = RNUM_CONTROL_BLK_0,
            .pwm { .pinA = 21, .pinB = 20, .frequency = DEF_PWM_FREQUENCY } 
        },

        {   .type = CDC_RT_PWM, .resId = RNUM_CONTROL_BLK_1,
            .pwm {  .pinA = 19, .pinB = 18, .frequency = DEF_PWM_FREQUENCY } 
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_BLK_0,
            .adc { .adcPin = 26, .adcNum = 0 }
        },

        {   .type = CDC_RT_ADC, .resId = RNUM_ADC_BLK_1,
            .adc { .adcPin = 27, .adcNum = 1 }
        },

        {   .type = RNUM_UART_RX_0, .resId = RNUM_UART_RX_0,
            .uart {  .rxPin = 12, .txPin = UNDEFINED_PIN, 
                     .baudRate = DEF_UART_BAUDRATE }
        },

        {   .type = CDC_RT_UART, .resId = RNUM_UART_RX_1,
            .uart { .rxPin = 12, .txPin = UNDEFINED_PIN, 
                    .baudRate = DEF_UART_BAUDRATE }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_CUT_SIGNAL,
            .gpio { .pinA = 4, .pinB = UNDEFINED_PIN, 
                    .pinMode = CDC_DIO_IN_PULLUP }
        },

        {
            .type = CDC_RT_GPIO, .resId = RNUM_ADC_MUX,
            .gpio { .pinA = UNDEFINED_PIN, .pinB = UNDEFINED_PIN, 
                    .pinMode = CDC_DIO_IN_PULLUP }
        }
    }
};
