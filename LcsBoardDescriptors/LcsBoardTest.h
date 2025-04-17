//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Test file...
//
//------------------------------------------------------------------------------------------------------------
// Board descriptors define the controller / board pin and function mapping. While the CDC layer abstracts
// the various hardware functions, the board descriptor table defines the resource mapping and a few other 
// values for the particular board. The hardware functions are called resources and define the pins and other 
// attributes used. For example, a UART resource needs the receive and transmit pins, as well as what data
// length, stop bits, and so on are set. There are also software resources such as a repeating timer. Each
// board has a type and a unique ID by which the correct descriptor map can be located. Upon library start,
// each resource is configured from this descriptor data.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Include file
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
#ifndef Lcs_BoardTest_h
#define Lcs_BoardTest_h

namespace CDC {

const CdcResourceDesc test[ ] = {

    {   .resId  = 0,
        .type   = CDC_IT_CONTROLLER,
        
        .ctl {

            .controllerFamily           = 0,
            .controllerChip             = 0,
            .cpuCores                   = 0,
            .memorySize                 = 0,
            .internalNvmSize            = 0,
            .watchDogIntervallMillis    = 0,
            .adcRefVoltageMillis        = 3300,
            .adcDigitRange              = 1024,
            .ledPin                     = UNDEFINED_PIN,
            .pFailPin                   = UNDEFINED_PIN
        }
    },

    {   .resId  = 1, 
        .type   = CDC_IT_GPIO,
        
        .gpio = {

            .pinA       = UNDEFINED_PIN,
            .pinB       = UNDEFINED_PIN,
            .pinMode    = CDC_DIO_IN
        }
    },

    {   
        .resId   = 15,
        .type   = CDC_IT_ADC,
        
        .adc    =  { .adcPin = 26 }   
    },

    {   
        .resId   = 16,
        .type   = CDC_IT_ADC,
        
        .adc    =  { .adcPin = 27 }   
    },
};


}; // namespace

#endif
