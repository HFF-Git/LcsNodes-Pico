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
// Button stuff ...
//
//----------------------------------------------------------------------------------------
uint16_t                debounceMillis      = DEFAULT_DEBOUNCE_MILLIS;
uint16_t                clickMillis         = DEFAULT_CLICK_MILLIS;
uint16_t                pressMillis         = DEFAULT_PRESS_MILLIS;

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
// Button Section.
//
//----------------------------------------------------------------------------------------
uint8_t configureButton( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_BUTTON );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );

    return( configureButton( rNum, dPtr -> button.pin, dPtr -> button.activeLow ));
}

uint8_t configureButton( uint8_t rNum, uint8_t hwPin, bool aLow ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> button.pin                  = hwPin;
    rPtr -> button.buttonState          = 0;
    rPtr -> button.longPressed          = false;
    rPtr -> button.activeLow            = aLow;
    rPtr -> button.startTime            = 0;
    rPtr -> button.stopTime             = 0;
    rPtr -> button.clickFunc            = nullptr;
    rPtr -> button.doubleClickFunc      = nullptr;
    rPtr -> button.longPressStartFunc   = nullptr;
    rPtr -> button.longPressStopFunc    = nullptr;
    rPtr -> button.duringLongPressFunc  = nullptr;

    return ( NO_ERR );
}

uint8_t attachClick( uint8_t rNum, ButtonCallback functionId ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    rPtr -> button.clickFunc = functionId;
    return ( NO_ERR );
}

uint8_t attachDoubleClick( uint8_t rNum, ButtonCallback functionId ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    rPtr -> button.doubleClickFunc = functionId;
    return ( NO_ERR );
}

uint8_t attachLongPressStart( uint8_t rNum, ButtonCallback functionId ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    rPtr -> button.longPressStartFunc = functionId;
    return ( NO_ERR );
}

uint8_t attachLongPressStop( uint8_t rNum, ButtonCallback functionId ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    rPtr -> button.longPressStopFunc = functionId;
    return ( NO_ERR );
}

uint8_t attachDuringLongPress( uint8_t rNum, ButtonCallback functionId )  {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    rPtr -> button.duringLongPressFunc = functionId;
    return ( NO_ERR );
}
 
bool isLongPressed( uint8_t rNum )  {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> button.longPressed );
}

uint32_t getDurationPressedMillis( uint8_t rNum )  {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> button.stopTime - rPtr -> button.startTime  );
}

uint32_t getPressedMillisSinceStart( uint8_t rNum )  {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_BUTTON );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( getMillis( ) - rPtr -> button.startTime );
}

//----------------------------------------------------------------------------------------
// "buttonTick" is the heart of managing a button. It is essentially a finite state 
// machine running off the current state machine state and the last value read for
// the button. A value matching the the defined active level is considered an active
// value for the button. The state machine has the following states:
//
//    State 0     - the button becomes active. Remember the starting time and set 
//                  the state to 1.
//
//    State 1     - if the button is inactive before the debouncing time window, it 
//                  is perhaps a glitch, ignore, go back to state 0.
//
//                  else if the button is inactive remember the stopping time and 
//                  set the state to 2.
//
//                  else if the button is active and the time for a long press is 
//                  exceeded, mark a long pressed event and invoke the start and
//                  during long press handlers, if any. Set the new state to 4.
//
//                  else set the sate to 1.
//
//    State 2     - if there are no double click handlers defined or the elapsed
//                  time for a click is reached, invoke a click function and set 
//                  the state to 0.
//
//                  else if the button is active again set the state to 3 and 
//                  remember the starting time.
//
//    State 3     - if the button is not active and the elapsed time is larger than
//                  the debouncing time, it is another click. Coming from state 2 
//                  this is considered a double click event. Invoke the double click
//                  handler, if any, remember the stopping time and set the state 
//                  to 0.
//
//    State 4     - the previous start was a recognized long press. If the button 
//                  became non active, remember the stopping time, invoke the end
//                  of long press handler and set the state to 0.
//
//                  else if the button is still active, invoke the during long press 
//                  state handler, if any, and set the state to 6.
//
//----------------------------------------------------------------------------------------
uint8_t buttonTick( CdcResource *rPtr ) {

    bool val    = gpio_get( rPtr -> button.pin );
    bool active = (( rPtr -> button.activeLow ) ? !val : val );

    #if 0 // use this when you suspect that the activeLow setting is messed up.
    printf( "HwId: %d, val: %d, active: %d\n",
            rPtr -> button.pin, val, active );
    #endif

    uint32_t  now         = getMillis( );
    uint32_t  elapsedTime = now - rPtr -> button.startTime;
    uint8_t   curState    = rPtr -> button.buttonState;
    uint8_t   nextState   = 0;

    if ( curState == 0 ) {

        if ( active ) {

            rPtr -> button.startTime  = now;
            nextState  = 1;
        }
        else nextState = 0;
    }
    else if ( curState == 1 ) {

        if (( ! active ) && ( elapsedTime < debounceMillis )) {

            nextState = 0;
        }
        else if (( ! active ) && ( elapsedTime >= debounceMillis )) {

            rPtr -> button.stopTime  = now;
            nextState = 2;
        }
        else if (( active ) && ( elapsedTime > pressMillis )) {

            rPtr -> button.longPressed = true;
            nextState   = 4;

            if ( rPtr -> button.longPressStartFunc ) 
                rPtr -> button.longPressStartFunc( rPtr -> resId );
        }
        else nextState = 1;
    }
    else if ( curState == 2 ) {

        if (( rPtr -> button.doubleClickFunc == NULL ) || 
            ( elapsedTime > clickMillis )) {

            if ( rPtr -> button.clickFunc ) 
                rPtr -> button.clickFunc( rPtr -> resId );
        
            nextState = 0;
        }
        else if (( active ) && ( elapsedTime > debounceMillis )) {

            rPtr -> button.startTime = now;
            nextState = 3;
        }
        else nextState = 2;
    }
    else if ( curState == 3 ) {

        if (( ! active ) && ( elapsedTime > debounceMillis )) {

            rPtr -> button.stopTime  = now;
            nextState = 0;

            if ( rPtr -> button.doubleClickFunc ) 
                rPtr -> button.doubleClickFunc( rPtr -> resId );
        }
        else nextState = 3;
    }
    else if ( curState == 4 ) {

        if ( ! active ) {

            rPtr -> button.longPressed = false;
            rPtr -> button.stopTime    = now;
            nextState   = 0;

            if ( rPtr -> button.longPressStopFunc ) 
                rPtr -> button.longPressStopFunc( rPtr -> resId );
        }
        else {

            rPtr -> button.longPressed = true;
            nextState   = 4;

            if ( rPtr -> button.duringLongPressFunc ) 
                rPtr -> button.duringLongPressFunc( rPtr -> resId );
        }
    }
    else nextState = 0;

    rPtr -> button.buttonState = nextState;

    return( NO_ERR );
}



}








