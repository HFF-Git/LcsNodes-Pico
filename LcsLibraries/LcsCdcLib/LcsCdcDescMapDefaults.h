//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file - DescMap defaults
//
//------------------------------------------------------------------------------------------------------------
//
// 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file - DescMap defaults
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
#ifndef LcsCdcDescMapDefaults_h
#define LcsCdcDescMapDefaults_h
 
#include "LcsCdcLib.h"

using namespace CDC;

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
const CdcResourceDescMap RES_MAP_RP_2040 = {

    //--------------------------------------------------------------------------------------------------------
    //
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
             
    .name                       = "Resource Map - RP2040",

    //--------------------------------------------------------------------------------------------------------
    //
    //
    //--------------------------------------------------------------------------------------------------------
    .map {
        
        // 0 - CDC_RN_ACTIVITY_LED
        {   .type = CDC_RT_GPIO,
            .gpio { .pinA       = 15, 
                    .pinB       = UNDEFINED_PIN, 
                    .pinMode    = CDC_DIO_OUT }   
        },

        // 1 - CDC_RT_TIMER_0
        {   .type = CDC_RT_TIMER,
              
        },

        // 2 - CDC_RT_TIMER_1
        {   .type = CDC_RT_TIMER,
              
        },

        // 3 - CDC_RN_PFAIL
        {   .type = CDC_RT_GPIO,
            .gpio { .pinA       = UNDEFINED_PIN,
                    .pinB       = UNDEFINED_PIN, 
                    .pinMode    = CDC_DIO_IN_PULLUP }   
        },

        // 4 - CDC_RN_CAN_BUS
        {   .type = CDC_RT_CAN_BUS,
            .can {  .rxPin      = 0, 
                    .txPin      = 1,
                    .baudRate   = 125000,
                    .canId      = 100,
                    .twoCores   = true
                 }
        },

        // 5 - CDC_RN_NVM
        {   .type = CDC_RT_I2C,
            .i2c {  .sclPin         = 3,
                    .sdaPin         = 2,
                    .baudRate       = 100000,
                    .i2cAdrRoot     = 0,
                    .i2cTimeoutMs   = 25
                 }
        },

         // 6 - CDC_RN_EXT_NVM
        {   .type = CDC_RT_I2C,
            .i2c {  .sclPin         = 17,
                    .sdaPin         = 16,
                    .baudRate       = 100000,
                    .i2cAdrRoot     = 0,
                    .i2cTimeoutMs   = 25
                 }
        },

        // 7 - CDC_RN_RESERVED
        { .type = CDC_RT_UNDEFINED,
              
        }
    }
};

#endif