//------------------------------------------------------------------------------------------------------------
//
// UITimer - implementation file.
//
//------------------------------------------------------------------------------------------------------------
// Sometimes we need a primitive timer. The UITimer class implements a simple timer that can be set to
// invoke a callback after the entered time interval. This timer is not very accurate. It will be part of
// the UI elements processTick mechanism and depending how often the tick runs, the timer is a bit off.
// Anyway, for a crude delayed callback invocation, it will do.
//
//------------------------------------------------------------------------------------------------------------
//
// UITimer
// Copyright (C) 2019 - 2023  Helmut Fieres
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
#include "UIElements.h"

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
UITimer::UITimer( ) {

  timerEnabled    = false;
}

void UITimer::setTimer( uint32_t val ) {

  timerInterval = val;
  startTimerVal = millis( );
  timerEnabled  = ( val > 0 );
}

void UITimer::attachTimer( UITimerCallBackFunction functionId ) {

  timerCallback = functionId;
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void UITimer::processTick( ) {

  if ( timerEnabled ) {

    uint32_t currentTimeVal = millis( );

    if (( currentTimeVal - startTimerVal ) > timerInterval ) {

      startTimerVal = currentTimeVal;

      if ( timerCallback != nullptr ) timerCallback( this );
    }
  }

} // UITimer::processTick
