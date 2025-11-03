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
// Copyright (C) 2022 - 2025 Helmut Fieres
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
//----------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <cstring>

#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;


}

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
// PWM section. The PICO is quite flexible when it comes to PWM signals. We support
// a simple PWM capability. There is the frequency which set during configuration 
// and there is the write operation which set the duty cycle. The calculations are 
// best described in the PICO C++ SDK. Note that although the PICO is quite flexible,
// the wrap and phase parameters are set for the slice and not a single channel. The
// same is true for the signal inverter. This is normally not an issue unless you 
// want to have separate values for PWM pins on the same slice. 
//
// The "writePwm" function will just manipulate the duty cycle. When we need to change
// the frequency we need to configure again. The "syncPwm" function will reset the
// wrap count, which is used to implement the sync function for H-Bridges emitting a
// PWM signal.
// 
//----------------------------------------------------------------------------------------
uint8_t configurePwm( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
   
    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_PWM );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configurePwm( rNum, 
                           dPtr -> pwm.pinA, 
                           dPtr -> pwm.pinB, 
                           dPtr -> pwm.frequency ));
}

//----------------------------------------------------------------------------------------
// Configure the PWM channel.
//
//----------------------------------------------------------------------------------------
uint8_t configurePwm( uint8_t rNum, 
                      uint8_t pinA, 
                      uint8_t pinB, 
                      uint32_t frequency ) {

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_PWM )) {

        printf( "Configure Pwm: rNum: %d, pinA: %d, pinB: %d, f: %d\n",
                rNum, pinA, pinB, frequency );
    }

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> pwm.pinA            = pinA;
    rPtr -> pwm.pinB            = pinB;
    rPtr -> pwm.phaseCorrect    = true;
    rPtr -> pwm.inverted        = false;     
    rPtr -> pwm.sliceNum        = pwm_gpio_to_slice_num( rPtr -> pwm.pinA );
    rPtr -> pwm.frequency       = frequency;              

    if ( rPtr -> pwm.pinB != UNDEFINED_PIN ) {

        if ( pwm_gpio_to_slice_num( rPtr -> pwm.pinA ) != 
             pwm_gpio_to_slice_num( rPtr -> pwm.pinB )) {

            return ( PWM_PIN_ERR );
        }
    }

    if ( rPtr -> pwm.phaseCorrect ) {
        
        rPtr -> pwm.frequency = rPtr -> pwm.frequency * 2;
    }

    uint32_t sysClock = clock_get_hz( clk_sys );
    uint32_t clkDiv   = sysClock / rPtr -> pwm.frequency / 4096 + 
                        ( sysClock % ( rPtr -> pwm.frequency * 4096 ) != 0 );
    if ( clkDiv / 16 == 0 ) clkDiv = 16;

    rPtr -> pwm.wrap = sysClock * 16 / clkDiv / rPtr -> pwm.frequency - 1;
   
    pwm_config pwmConfig = pwm_get_default_config( );
    gpio_set_function( rPtr -> pwm.pinA, GPIO_FUNC_PWM );
    
    if ( rPtr -> pwm.pinB != UNDEFINED_PIN )  
        gpio_set_function( rPtr -> pwm.pinB, GPIO_FUNC_PWM );
   
    pwm_config_set_wrap( &pwmConfig, rPtr -> pwm.wrap );
    pwm_config_set_phase_correct( &pwmConfig, rPtr -> pwm.phaseCorrect );

    pwm_config_set_output_polarity( &pwmConfig, 
                                    rPtr -> pwm.inverted, 
                                    rPtr -> pwm.inverted );

    pwm_init( rPtr -> pwm.sliceNum, &pwmConfig, false );
    pwm_set_clkdiv_int_frac( rPtr -> pwm.sliceNum, clkDiv / 16, clkDiv & 0xF );
    pwm_set_enabled( rPtr -> pwm.sliceNum, true );

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_PWM )) {
   
        printf( "pinA: % d, pinB: %d, fPwm: % d, phase: % d, inverted: % d, " 
                "clkDiv: % d, wrap: %d, sliceNum: %d\n",
                rPtr -> pwm.pinA, 
                rPtr -> pwm.pinB, 
                rPtr -> pwm.frequency, 
                rPtr -> pwm.phaseCorrect, 
                rPtr -> pwm.inverted,
                clkDiv, 
                rPtr -> pwm.wrap, 
                rPtr -> pwm.sliceNum );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Set the PWM frequency.
//
//----------------------------------------------------------------------------------------
uint8_t setPwmFrequency( uint8_t rNum, uint32_t frequency ) {

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_PWM )) {
        
        printf( "Set PWMFrequency: rNum: %d, f: %d\n", rNum, frequency );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    return ( configurePwm( rNum, rPtr -> pwm.pinA, rPtr -> pwm.pinB, frequency ));
}

//----------------------------------------------------------------------------------------
// Write the PWM duty cycles.
//
//----------------------------------------------------------------------------------------
uint8_t writePwm( uint8_t rNum, uint8_t dutyCycleA, uint8_t dutyCycleB ) {

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_PWM )) {
        
        printf( "Write PWM: rNum: %d, dutyA: %d, dutyB: %d\n", 
                rNum, dutyCycleA, dutyCycleB );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> pwm.pinB == UNDEFINED_PIN ) {

        pwm_set_gpio_level( rPtr -> pwm.pinA, dutyCycleA );
    }
    else {
                            
        // printf( "Write Pwm: Slice: %d\n", rPtr -> pwm.sliceNum );
        pwm_set_both_levels( rPtr -> pwm.sliceNum, dutyCycleA, dutyCycleB );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Sync the PWM channel, i.e. reset the PWM slice counter.
//
//----------------------------------------------------------------------------------------
uint8_t syncPwm( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    pwm_set_counter( rPtr -> pwm.sliceNum, 0 );
    return ( NO_ERR );
}

} // namespace CDC
