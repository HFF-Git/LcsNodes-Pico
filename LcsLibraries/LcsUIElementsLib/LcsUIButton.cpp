//----------------------------------------------------------------------------------------
//
// UIButton - implementation file
//
//----------------------------------------------------------------------------------------
// Buttons are one  of the most used UI Elements. Each button is essentially a state 
// machine with a set of defined callback functions. There is also a callback for 
// obtaining the actual button value. The time for debouncing, detecting a click or a 
// long press is set for all buttons. A switch is also just a button with long press
// characteristic. All this comfort comes with a price though. The button object has a
// size of roundabout 24 bytes. So, an array of 256 buttons would occupy quite some
// memory storage. But for the typical case of 8 - 32 buttons, the array will do just
// fine.
//
//----------------------------------------------------------------------------------------
//
// UIButtonElements
// Copyright (C) 2019 - 2025  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should
// have received a copy of the GNU General Public License along with this program. 
// If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsUIElements.h"

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

    //------------------------------------------------------------------------------------
    // UIButton state. To save memory, we encode in the button state the active level 
    // logic, the long pressed detection flag and the state for the button state machine.
    //
    //------------------------------------------------------------------------------------
    enum buttonState : uint8_t {

        BS_ACTIVE_LOW     = 0x80,
        BS_LONG_PRESSED   = 0x40,
        BS_STATE          = 0x0F
     };

    //------------------------------------------------------------------------------------
    // The default time intervals. The debounce value will determine when we consider a 
    // button pushed. The click value defines how ling we press a button for a click and
    // the press value defines what is considered a long push. There is also the option 
    // to detect a double click. Care needs to be tale that the click interval is not too
    // long, which would result in a long press, and not too short, which would not ever
    // consider a double click. The default values are the result of testing some common
    // tactical switch buttons.
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
// The static routines for setting the time intervals. We set them for all buttons objects
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
// The UI Button object. Each button has a resource ID. This could be directly the pin 
// number but also any other number by which the button is identified in callbacks. A 
// button can also be active low or high.
//
//----------------------------------------------------------------------------------------
UIButton::UIButton( uint8_t hwId, bool activeLow ) {
  
  this -> hwId  = hwId;

  if ( activeLow )  buttonState |= BS_ACTIVE_LOW;
  else              buttonState &= ~ BS_ACTIVE_LOW;

  reset( );
}

void UIButton::reset( ) {

  buttonState   &= ~ BS_STATE;
  buttonState   &= ~ BS_LONG_PRESSED;
  startTime     = 0;
  stopTime      = 0;
}

void UIButton::attachClick( UIButtonCallBackFunction functionId ) {

  clickFunc = functionId;
}

void UIButton::attachDoubleClick( UIButtonCallBackFunction functionId ) {

  doubleClickFunc = functionId;
}

void UIButton::attachLongPressStart( UIButtonCallBackFunction functionId ) {

  longPressStartFunc = functionId;
}

void UIButton::attachLongPressStop( UIButtonCallBackFunction functionId ) {

  longPressStopFunc = functionId;
}

void UIButton::attachDuringLongPress( UIButtonCallBackFunction functionId ) {

  duringLongPressFunc = functionId;
}

void UIButton::attachGetDataFunction( UIGetDataFunction functionId ) {

  getDataFunc = functionId;
}

bool UIButton::isLongPressed( ) {

  return ( buttonState & BS_LONG_PRESSED );
}

uint32_t UIButton::getDurationPressedMillis(  ) {

  return ( stopTime - startTime  );
}

uint32_t UIButton::getPressedMillisSinceStart( ) {

  return ( CDC::getMillis( ) - startTime );
}

uint8_t UIButton::getHwId( ) {

  return ( hwId );
}

//----------------------------------------------------------------------------------------
// "processTick" is the heart of managing a button. It is essentially a finite state 
// machine running off the current state machine state and the last value read for the
// button. A value matching the the defined active level is considered an active value 
// for the button. The state machine has the following states:
//
//    State 0     - the button becomes active. Remember the starting time and set the 
//                  state to 1.
//
//    State 1     - if the button is inactive before the debouncing time window, it is 
//                  perhaps a glitch, ignore, go back to state 0.
//
//                  else if the button is inactive remember the stopping time and set 
//                  the state to 2.
//
//                  else if the button is active and the time for a long press is 
//                  exceeded, mark a long pressed event and invoke the start and during
//                  long press handlers, if any. Set the new state to 4.
//
//                  else set the sate to 1.
//
//    State 2     - if there are no double click handlers defined or the elapsed time 
//                  for a click is reached, invoke a click function and set the state to 0.
//
//                  else if the button is active again set the state to 3 and remember 
//                  the starting time.
//
//    State 3     - if the button is not active and the elapsed time is larger than the
//                  debouncing time, it is another click. Coming from state 2 this is 
//                  considered a double click event. Invoke the double click handler, 
//                  if any, remember the stopping time and set the state to 0.
//
//    State 4     - the previous start was a recognized long press. If the button became
//                  non active, remember the stopping time, invoke the end of long press
//                  handler and set the state to 0.
//
//                  else if the button is still active, invoke the during long press 
//                  state handler, if any, and set the state to 6.
//
//----------------------------------------------------------------------------------------
void UIButton::processTick( ) {

  bool val    = (( getDataFunc != nullptr ) ? getDataFunc( hwId ) : false );
  bool active = (( buttonState & BS_ACTIVE_LOW ) ? !val : val );

  #if 0 // use this when you suspect that the activeLow setting is messed up.
  Serial.print( "HwId: " );
  Serial.print( hwId );
  Serial.print( ":" );
  Serial.print( val );
  Serial.print( ":" );
  Serial.println( active );
  #endif

  uint32_t  now         = CDC::getMillis( );
  uint32_t  elapsedTime = now - startTime;
  uint8_t   curState    = buttonState & BS_STATE;
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

      buttonState |= BS_LONG_PRESSED;
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

      buttonState &= ~ BS_LONG_PRESSED;
      stopTime    = now;
      nextState   = 0;

      if ( longPressStopFunc ) longPressStopFunc( this );
    }
    else {

      buttonState |= BS_LONG_PRESSED;
      nextState   = 4;

      if ( duringLongPressFunc ) duringLongPressFunc( this );
    }
  }
  else nextState = 0;

  buttonState = ( buttonState & ~ BS_STATE ) + nextState;
}
