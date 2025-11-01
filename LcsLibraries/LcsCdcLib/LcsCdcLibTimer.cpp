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

//----------------------------------------------------------------------------------------
// Global Interrupt handlers. The hardware and low level library will call these 
// handlers, which in turn will invoke the respective callback function if configured. 
//
// The repeating timer alarm will handle timer interrupts. We stored the respective 
// timer resource in the "user_data" field, so that we can get to the interrupt 
// handler configured.
//
// The GPIO interrupt handler manages the handler for all possible IO pins. The PICO 
// can only have one interrupt routine, so we feature an array of handlers where a 
// handler for a GPIO pin can be registered. 
// 
// The UART handlers will handle receive interrupts of the UART hardware blocks. 
// There is no easy way to get to the resource structure where the input buffer is.
// We therefore maintain two global variables in this file to store the configured 
// resource for each UART HW block.
// 
//----------------------------------------------------------------------------------------
bool repeatingTimerAlarm( repeating_timer_t *rt ) {

    CdcResource *ptr = (CdcResource *) rt -> user_data;

    if ( ptr -> timer.timerCallback != nullptr ) {

        ptr -> timer.timerCallback((uint32_t)
                                    ( - ptr -> timer.timerData.delay_us ));       
    }
    
    return ( true );
}



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
// Timer section. The CDC library features a repeating timer with a microsecond 
// resolution. There are routines to start and stop the timer as well as to allow to
// set a new limit. The PICO offers a high level function that schedules a repeating
// timer with the property of measuring the interval also from the start of the 
// callback invocation. 
//
// ??? add priority for handler ?
//----------------------------------------------------------------------------------------
uint8_t configureTimer( uint8_t rNum, uint8_t pri, TimerCallback functionId ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
  
    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_TIMER );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    CdcResource *ptr = allocateResourceType( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    ptr -> timer.timerVal         = dPtr -> timer.timerVal;
    ptr -> timer.timerCallback    = functionId;
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Start a timer.
//
// ??? where to set priority ?
// ??? we would need a different alarm pool with a higher priority for our PRI alarms.
// We would need two different alarm pools. ( or three ? )
// to which we add the timers...
//----------------------------------------------------------------------------------------
uint8_t startRepeatingTimer( uint8_t rNum, uint32_t val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    add_repeating_timer_us( - limit, 
                            repeatingTimerAlarm, 
                            ptr, 
                            &ptr -> timer.timerData );

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Stop a timer.
//
//----------------------------------------------------------------------------------------
uint8_t stopRepeatingTimer( uint8_t rNum ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    cancel_repeating_timer( &ptr -> timer.timerData );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Return the upper limit value for the timer.
//
//----------------------------------------------------------------------------------------
uint8_t getRepeatingTimerLimit( uint8_t rNum, uint32_t *val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    *val = (uint32_t) ( - ptr -> timer.timerData.delay_us );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Set a new timer limit.
//
//----------------------------------------------------------------------------------------
uint8_t setRepeatingTimerLimit( uint8_t rNum, uint32_t val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    ptr -> timer.timerData.delay_us = ((int64_t) - limit );
    return ( NO_ERR );
}

}








