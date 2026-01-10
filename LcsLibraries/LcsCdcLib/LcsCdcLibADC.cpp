//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;

} // namespace

//----------------------------------------------------------------------------------------
// Global variables for the CDC lib. Declared in "LcsCdcLib.cpp".
//
//----------------------------------------------------------------------------------------
namespace CDC {

    extern uint16_t                debugMask;
    extern uint16_t                options;

    extern CdcResourceDescMap      dMap;
    extern CdcResourceMap          rMap;

    extern CdcResource *lookupResource( uint8_t rNum, uint8_t type );
    extern CdcResource *allocateResourceType( uint8_t rNum, uint8_t type );
}

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// ADC section. The analog input channel represented by the pin is configured. At 
// initialization, the ADC pin number is validated and the ADC subsystem is 
// initialized. The PICO does an analog read in about 2us. This is so fast, it is
// sufficient for our purpose, so it does not make much sense to implement an 
// asynchronous option. The PICO support up to three ADC pins at the dedicated 
// HW pins numbers 26, 27 and 28. They also need to be mapped an ADC select number
// for selecting the ADC hardware.
//
//----------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_ADC );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureAdc( rNum, dPtr -> adc.adcPin, dPtr -> adc.adcNum ));
}

//----------------------------------------------------------------------------------------
// Configure the ADC channel.
//
//----------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t rNum, uint8_t adcPin, uint8_t adcNum ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_ADC );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( adcPin == 26 ) {

        rPtr -> adc.adcPin = 26;
        rPtr -> adc.adcNum = 0;
    }
    else  if ( adcPin == 27 ) {

        rPtr -> adc.adcPin = 27;
        rPtr -> adc.adcNum = 1;
    }
    else  if ( adcPin == 28 ) {

        rPtr -> adc.adcPin = 28;
        rPtr -> adc.adcNum = 2;
    }
    else return ( ADC_PIN_ERR );

    adc_init( );
    adc_gpio_init( rPtr -> adc.adcPin );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Read the ADC value. The resolution is 12-bits.
//
//----------------------------------------------------------------------------------------
uint8_t readAdc( uint8_t rNum, uint16_t *val ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_ADC );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    adc_select_input( rPtr -> adc.adcNum );
    *val = adc_read( );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
uint16_t getAdcRefVoltage( ) {

    return ( ADC_REF_VOLTAGE_MILLIS );
}

uint16_t getAdcDigitRange( ) {

    return ( ADC_DIGIT_RANGE );
}

} // namespace CDC
