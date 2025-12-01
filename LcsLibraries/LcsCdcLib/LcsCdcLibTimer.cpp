//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// CDC features timers. They are used for implementing periodical tasks. There are
// two pools for timers. The default priority pool is used almost all of the timer
// demands. In addition, there is a high priority pool where timer handlers are 
// kept that interrupt also the default timer handlers. They should only be used 
// for truly timer critical tasks running in the microseconds range.
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
//----------------------------------------------------------------------------------------
bool repeatingTimerAlarm( repeating_timer_t *rt ) {

    CdcResource *ptr = (CdcResource *) rt -> user_data;

    if ( ptr -> timer.timerCallback != nullptr ) {

        ptr -> timer.timerCallback((uint32_t)
                        ( - ptr -> timer.timerData.delay_us ));       
    }
    
    return ( true );
}

//----------------------------------------------------------------------------------------
// We will support two basic pools of alarms. The first the default pool. The other
// pool is a high priority pool. A high priority alarm would then interrupt a default
// priority alarm too.
//
//----------------------------------------------------------------------------------------
alarm_pool  *highPriAlarmPool   = nullptr;
alarm_pool  *lowPriAlarmPool    = nullptr;

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
// CDC features two alarm pools. There is the default alarm pool for most timers. 
// In addition, there is a high priority alarm pool, which will interrupt also a 
// timer handler of a default alarm pool timer. The idea is that we have time 
// critical timer, which need to run even if another timer is served. An example
// would be the DCC signal state machine timer. We will use the default alarm pool
// and create an additional high priority timer. 
//
// -> Create an alarm pool on hardware alarm 2, avoid default pool alarms 0..1. 
// -> Next, find the hardware alarm used by this pool (0..3).
// -> Compute the IRQ for that alarm and set its NVIC priority ( 0 = highest ).
// 
//----------------------------------------------------------------------------------------
uint8_t setupAlarmPools( ) {

    lowPriAlarmPool = alarm_pool_get_default( );

    highPriAlarmPool = alarm_pool_create( 2, 8 );
    if ( ! highPriAlarmPool ) 
        fatalError( 7, (char *) "Cannot create high pri alarm pool\n" );
        
    uint hw_alarm = alarm_pool_hardware_alarm_num( highPriAlarmPool );
    uint irq = TIMER_IRQ_0 + hw_alarm;
    irq_set_priority( irq, 0 );

    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Timer section. The CDC library features a repeating timer with a microsecond 
// resolution. There are routines to start and stop the timer as well as to allow to
// set a new limit. The PICO offers a high level function that schedules a repeating
// timer with the property of measuring the interval also from the start of the 
// callback invocation. 
//
//----------------------------------------------------------------------------------------
uint8_t configureTimer( uint8_t rNum, TimerCallback functionId,  bool pri  ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_TIMER );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    CdcResource *ptr = allocateResourceType( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( RES_NUM_ERR );

    ptr -> timer.timerVal         = dPtr -> timer.timerVal;
    ptr -> timer.timerCallback    = functionId;
    ptr -> timer.timerHighPri     = pri;
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Start a timer.
//
//----------------------------------------------------------------------------------------
uint8_t startRepeatingTimer( uint8_t rNum, uint32_t val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( RES_NUM_ERR );

    int64_t    limit = val;
    alarm_pool *aPtr = nullptr;

    if ( ptr -> timer.timerHighPri ) aPtr = highPriAlarmPool;
    else                             aPtr = lowPriAlarmPool;

    return(
        ( alarm_pool_add_repeating_timer_us( aPtr,
                                             - limit,
                                             repeatingTimerAlarm,  
                                             ptr, 
                                             &ptr -> timer.timerData )) ? 
                                                        NO_ERR : TIMER_ERR );
}

//----------------------------------------------------------------------------------------
// Stop a timer.
//
//----------------------------------------------------------------------------------------
uint8_t stopRepeatingTimer( uint8_t rNum ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( RES_NUM_ERR );

    return (( cancel_repeating_timer( &ptr -> timer.timerData )) ? 
                                                    NO_ERR : TIMER_ERR );
}

//----------------------------------------------------------------------------------------
// Return the upper limit value for the timer.
//
//----------------------------------------------------------------------------------------
uint8_t getRepeatingTimerLimit( uint8_t rNum, uint32_t *val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( RES_NUM_ERR );

    *val = (uint32_t) ( - ptr -> timer.timerData.delay_us );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Set a new timer limit.
//
//----------------------------------------------------------------------------------------
uint8_t setRepeatingTimerLimit( uint8_t rNum, uint32_t val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( RES_NUM_ERR );

    int64_t limit = val;
    ptr -> timer.timerData.delay_us = ((int64_t) - limit );
    return ( NO_ERR );
}

} // namespace CDC
