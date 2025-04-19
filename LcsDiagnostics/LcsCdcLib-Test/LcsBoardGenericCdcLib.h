//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Generic Runtime Lib
//
//------------------------------------------------------------------------------------------------------------
// Board descriptors define the controller / board pin and function mapping. This file contains the basic
// configuration values for the controller, the CAN bus and the I2C channels. The descriptor allows to 
// initialize a runtime library without any further function configured.
//
//------------------------------------------------------------------------------------------------------------
//
//  LCS - Board Descriptors - Generic Runtime Lib
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
#ifndef Lcs_BoardTGenericRtLib_h
#define Lcs_BoardTGenericRtLib_h

#include "LcsCdcLib.h"

using namespace CDC;

//------------------------------------------------------------------------------------------------------------
// Resource Id number which is used to find the resource in the resource table build from the resource 
// descriptor data. The number directly indexes into the resource table entry used when configuring the
// particular resource. The firmware will use the resource Id number to access the resource after it is 
// configured. The numbers can be arbitrary, but cannot exceed the size of the table.
//
//------------------------------------------------------------------------------------------------------------
enum CdcResourceIdNum : uint8_t {

    CDC_RID_UNDEFINED       = 0,
    CDC_RID_CONTROLLER      = 1,
    CDC_RID_CAN_BUS         = 2,
    CDC_RID_NVM_I2C         = 3,
    CDC_RID_EXT_I2C         = 4,

    CDC_RID_TIMER_0         = 5,
    CDC_RID_TIMER_1         = 6,
    
    CDC_RID_ADC_0           = 7,
    CDC_RID_ADC_1           = 8,
   
    CDC_RID_DIO_0           = 9,
    CDC_RID_DIO_1           = 10,
    CDC_RID_DIO_2           = 11,
    CDC_RID_DIO_3           = 12,
    CDC_RID_DIO_4           = 13,
    CDC_RID_DIO_5           = 14,
    CDC_RID_DIO_6           = 15,
    CDC_RID_DIO_7           = 16,
    
    CDC_RID_PWM_0           = 17,
    CDC_RID_PWM_1           = 18
};

//------------------------------------------------------------------------------------------------------------
// Resource configuration data. Each resource has an entry in this structure and is defined by the resource
// Id Number and resource type. The remaining data is resource type specific.
//
//------------------------------------------------------------------------------------------------------------
const CdcResourceDescMap defaultRtLib = {

    .options    = 0,
    .name       = "CDC Test Program resource map",

    .map {

        {   .resId  = CDC_RID_CONTROLLER,
            .type   = CDC_RT_CONTROLLER,
            
            .ctl {
    
                .controllerFamily           = CDC_CF_RP_PICO,
                .controllerChip             = CDC_CF_C_RP_2040,
                .cpuCores                   = 2,
                .memorySize                 = 260*1024,
                .internalNvmSize            = 0,
                .watchDogIntervallMillis    = 2000,
                .adcRefVoltageMillis        = 3300,
                .adcDigitRange              = 1024,
                .ledPin                     = 14,
                .pFailPin                   = 15
            }
        },
    
        {   .resId  = CDC_RID_CAN_BUS,
            .type   = CDC_RT_CAN_BUS,
    
            .can {
    
                .canPinRx       = 0,
                .canPinTx       = 1,
                .baudRate       = 125 * 1000,
                .twoCores       = true
            }
        },
    
        {   .resId  = CDC_RID_NVM_I2C,
            .type   = CDC_RT_I2C,
    
            .i2c {
    
                .sclPin         = 2,
                .sdaPin         = 3,
                .baudRate       = 100 * 1000,
                .timeoutValMs   = 25
            }
        },
    
        {
            .resId  = CDC_RID_EXT_I2C,
            .type   = CDC_RT_I2C,
    
            .i2c {
    
                .sclPin         = 17,
                .sdaPin         = 16,
                .baudRate       = 100 * 1000,
                .timeoutValMs   = 25
            }
        },

        {
            .resId = CDC_RID_TIMER_0,
            .type  = CDC_RT_TIMER,

            .timer {
        
                .intervalMillis = 2000
            }

        },

        {
            .resId = CDC_RID_TIMER_1,
            .type  = CDC_RT_TIMER,

            .timer {
        
                .intervalMillis = 1000
            }
        },

        {   .resId  = CDC_RID_ADC_0,
            .type   = CDC_RT_ADC,
    
            .adc {

                .adcPin = 26
            }
        },

        {   .resId  = CDC_RID_ADC_1,
            .type   = CDC_RT_ADC,
    
            .adc {

                .adcPin = 27
            }
        },

        {   .resId  = CDC_RID_DIO_0,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_1,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_2,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_3,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_4,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_5,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_6,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_DIO_7,
            .type   = CDC_RT_GPIO,
    
            .gpio {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .pinMode    = CDC_DIO_IN_PULLUP
            }
        },

        {   .resId  = CDC_RID_PWM_0,
            .type   = CDC_RT_PWM,
    
            .pwm {

                .pinA       = UNDEFINED_PIN,
                .pinB       = UNDEFINED_PIN,
                .freqency   = 100
            }
        },

        {   .resId  = CDC_RID_PWM_1,
            .type   = CDC_RT_PWM,
    
            .pwm {

                .pinA        = UNDEFINED_PIN,
                .pinB        = UNDEFINED_PIN,
                .freqency    = 100
            }
        }
    }
};

#endif // Lcs_BoardTGenericRtLib_h
