//----------------------------------------------------------------------------------------
//
// UIButton - implementation file
//
//----------------------------------------------------------------------------------------
// Buttons are one of the most common UI Element. Each button is essentially a 
// state machine with a set of defined event callback functions. There is a
// callback function for obtaining the actual button value. The time for debouncing,
// detecting a click or a long press is set for all buttons. A switch is just a
// button with long press characteristic. 
//
//----------------------------------------------------------------------------------------
//
// UIButtonElements
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
#include "LcsUIElements.h"

using namespace CDC;

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

    //------------------------------------------------------------------------------------
    // The default time intervals. The debounce time value will determine when we 
    // consider a button pushed. The click value defines how ling we press a button 
    // for a click and the press value defines what is considered a long push. There
    // is also the option to detect a double click. Care needs to be taken that the
    // click interval is not too long, which would result in a long press, and not
    // too short, which would lead to not ever consider a double click. The default
    // values are the result of testing some common tactical switch buttons.
    //
    //------------------------------------------------------------------------------------
    const uint16_t  DEFAULT_DEBOUNCE_MILLIS   = 50;
    const uint16_t  DEFAULT_CLICK_MILLIS      = 50;
    const uint16_t  DEFAULT_PRESS_MILLIS      = 500;

    uint16_t        debounceMillis = DEFAULT_DEBOUNCE_MILLIS;
    uint16_t        clickMillis    = DEFAULT_CLICK_MILLIS;
    uint16_t        pressMillis    = DEFAULT_PRESS_MILLIS;

} // namespace

//========================================================================================
//
// UIButton Object Section.
//
//========================================================================================

//----------------------------------------------------------------------------------------
// The routines for setting the time intervals. We set them for all button objects
// to the same value.
//
//----------------------------------------------------------------------------------------
void UIButton::setDebounceMillis( uint32_t ticks ) {

    debounceMillis = ticks;
}

void UIButton::setClickMillis( uint32_t ticks ) {

    clickMillis = ticks;
}

void UIButton::setPressMillis( uint32_t ticks ) {

    pressMillis = ticks;
}

//----------------------------------------------------------------------------------------
// The UI Button object. 
//
//----------------------------------------------------------------------------------------
UIButton::UIButton( uint8_t rNum, bool activeLow ) {
  
    this -> rNum        = rNum;
    this -> activeLow   = activeLow;
    reset( );
}

void UIButton::reset( ) {

    buttonState = 0;
    longPressed = false;
    startTime   = 0;
    stopTime    = 0;
}

void UIButton::attachClick( UIButtonCbFunction functionId ) {

    clickFunc = functionId;
}

void UIButton::attachDoubleClick( UIButtonCbFunction functionId ) {

    doubleClickFunc = functionId;
}

void UIButton::attachLongPressStart( UIButtonCbFunction functionId ) {

    longPressStartFunc = functionId;
}

void UIButton::attachLongPressStop( UIButtonCbFunction functionId ) {

    longPressStopFunc = functionId;
}

void UIButton::attachDuringLongPress( UIButtonCbFunction functionId ) {

    duringLongPressFunc = functionId;
}

void UIButton::attachGetDataFunction( UIGetDataFunction functionId ) {

    getDataFunc = functionId;
}

bool UIButton::isLongPressed( ) {

    return ( longPressed );
}

uint32_t UIButton::getDurationPressedMillis(  ) {

    return ( stopTime - startTime  );
}

uint32_t UIButton::getPressedMillisSinceStart( ) {

    return ( getMillis( ) - startTime );
}

uint8_t UIButton::getResId( ) {

    return ( rNum );
}

//----------------------------------------------------------------------------------------
// "processTick" is the heart of managing a button. It is essentially a finite 
// state machine using the current state and the last value read for the button. 
// A value matching the the defined active level is considered an active value 
// for the button. The state machine has the following states:
//
//    State 0     - the button becomes active. Remember the starting time and set 
//                  the state to 1.
//
//    State 1     - if the button is inactive before the debouncing time window, 
//                  it is perhaps a glitch, ignore, go back to state 0.
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
//    State 3     - if the button is not active and the elapsed time is larger 
//                  than the debouncing time, it is another click. Coming from 
//                  state 2 this is considered a double click event. Invoke the
//                  double click handler, if any, remember the stopping time and 
//                  set the state to 0.
//
//    State 4     - the previous start was a recognized long press. If the button 
//                  became non active, remember the stopping time, invoke the end
//                  of long press handler and set the state to 0.
//
//                  else if the button is still active, invoke the during long 
//                  press state handler, if any, and set the state to 6.
//
//----------------------------------------------------------------------------------------
void UIButton::processTick( ) {

    bool val = false;

    if ( getDataFunc != nullptr ) getDataFunc( rNum, &val );

    bool active = (( activeLow ) ? !val : val );

    #if 0 // use this when you suspect that the activeLow setting is messed up.
    Serial.print( "HwId: " );
    Serial.print( hwId );
    Serial.print( ":" );
    Serial.print( val );
    Serial.print( ":" );
    Serial.println( active );
    #endif

    uint32_t  now         = getMillis( );
    uint32_t  elapsedTime = now - startTime;
    uint8_t   curState    = buttonState;
    uint8_t   nextState   = 0;

    if ( curState == 0 ) {

        if ( active ) {

            startTime  = now;
            nextState  = 1;
        }
        else nextState = 0;
    }
    else if ( curState == 1 ) {

        if (( ! active ) && ( elapsedTime < debounceMillis )) {

            nextState = 0;
        }
        else if (( ! active ) && ( elapsedTime >= debounceMillis )) {

            stopTime  = now;
            nextState = 2;
        }
        else if (( active ) && ( elapsedTime > pressMillis )) {

            longPressed = true;
            nextState   = 4;

            if ( longPressStartFunc ) longPressStartFunc( this );
        }
        else nextState = 1;
    }
    else if ( curState == 2 ) {

        if (( doubleClickFunc == NULL ) || ( elapsedTime > clickMillis )) {

            if ( clickFunc ) clickFunc( this );
            nextState = 0;
        }
        else if (( active ) && ( elapsedTime > debounceMillis )) {

            startTime = now;
            nextState = 3;
        }
        else nextState = 2;
    }
    else if ( curState == 3 ) {

        if (( ! active ) && ( elapsedTime > debounceMillis )) {

            stopTime  = now;
            nextState = 0;

            if ( doubleClickFunc ) doubleClickFunc( this );
        }
        else nextState = 3;
    }
    else if ( curState == 4 ) {

        if ( ! active ) {

            longPressed = false;
            stopTime    = now;
            nextState   = 0;

            if ( longPressStopFunc ) longPressStopFunc( this );
        }
        else {

            longPressed = true;
            nextState   = 4;

            if ( duringLongPressFunc ) duringLongPressFunc( this );
        }
    }
    else nextState = 0;

    buttonState = nextState;
}
