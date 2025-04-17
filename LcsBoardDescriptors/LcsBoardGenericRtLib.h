//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Generic Runtime Lib
//
//------------------------------------------------------------------------------------------------------------
// Board descriptors define the controller / board pin and function mapping. This file contains the basic
// configuration values for the controller, the CAN bus and the I2C channels. The desrciptor allows to 
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

namespace CDC {

const CdcResourceDescMap defaultRtLib = {

    .options    = 0,
    .entries    = 4,
    .name       = "Default LCS Runtime Resource Map",

    .map {

        {   .resId  = 0,
            .type   = CDC_IT_CONTROLLER,
            
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
    
        {   .resId  = 1,
            .type   = CDC_IT_CAN_BUS,
    
            .can {
    
                .canPinRx       = 0,
                .canPinTx       = 1,
                .baudRate       = 125 * 1000,
                .twoCores       = true
            }
        },
    
        {   .resId  = 2,
            .type   = CDC_IT_I2C,
    
            .i2c {
    
                .sclPin         = 2,
                .sdaPin         = 3,
                .baudRate       = 100 * 1000,
                .timeoutValMs   = 25
            }
        },
    
        {
            .resId  = 3,
            .type   = CDC_IT_I2C,
    
            .i2c {
    
                .sclPin         = 17,
                .sdaPin         = 16,
                .baudRate       = 100 * 1000,
                .timeoutValMs   = 25
            }
        },
    }
};

}; // namespace

#endif // Lcs_BoardTGenericRtLib_h
