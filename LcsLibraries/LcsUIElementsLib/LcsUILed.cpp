//----------------------------------------------------------------------------------------
//
// UILedElements - implementation file.
//
//----------------------------------------------------------------------------------------
// LEDs are UIElements that can be turned on and off, toggled and blink. They are 
// rather straightforward. There is a callback function to retrieve the data for 
// the LED.
//
//----------------------------------------------------------------------------------------
//
// UILedElements
// Copyright (C) 2019 - 2025  Helmut Fieres
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
#include "LcsUIElements.h"

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// LEDs can blink. All LEDs will be configured with the same blink interval value. 
// Another option would be to spend each LED its own interval, but this would just 
// ramp up the storage requirement and perhaps rarely used. So for now, all LEDs 
// have the  same blink interval.
//
//----------------------------------------------------------------------------------------
const uint16_t DEFAULT_BLINK_TICKS    = 1000;
uint32_t       blickIntervalInMillis  = DEFAULT_BLINK_TICKS;

} // namespace

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void UILed::setBlinkIntervalMillis( uint32_t val ) {

    blickIntervalInMillis = val;
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
UILed::UILed( uint8_t rNum ) {

    this -> rNum = rNum;
}

void UILed::attachSetDataFunction( UISetDataFunction functionId ) {

    setDataFunc = functionId;
}

bool UILed::isOn( ) {

     return ( ledOn );
}

bool UILed::isOff( ) {

    return ( ! ledOn );
}

void UILed::setOn( ) {

    ledOn    = true;
    ledBlink = false;
}

void UILed::setOff( ) {

    ledOn    = false;
    ledBlink = false;
}

void UILed::setVal( bool val ) {

    ledOn    = val;
    ledBlink = false;
}

void UILed::toggle( ) {

    ledOn = ! ledOn;
    ledBlink = false;
}

void UILed::blink( ) {

    ledOn    = true;
    ledBlink = true;
}

void UILed::processTick( ) {

    if ( ledBlink ) {

        if (( getMillis( ) - lastChange ) > blickIntervalInMillis ) {

            lastChange  = getMillis( );
            ledOn       = ! ledOn;
        }
    }

    setDataFunc( rNum, ledOn );
}
