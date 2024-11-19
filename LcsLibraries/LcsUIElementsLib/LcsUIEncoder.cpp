//------------------------------------------------------------------------------------------------------------
//
// UIEncoderElements -implementation file.
//
//------------------------------------------------------------------------------------------------------------
// Rotary encoder are the second active UI element. They do have two ports and depending on the direction
// turned, reading the values for the two ports tells the direction. Whenever the encoder is turned the
// new position is passed via a callback. Some rotary encoders also have a push button built into the knob.
// This will not be handled here, it is just a button for which we have the UIButton object.
//
//------------------------------------------------------------------------------------------------------------
//
// UIEncoderElements
// Copyright (C) 2019 - 2024  Helmut Fieres
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
#include "LcsUIElements.h"

//------------------------------------------------------------------------------------------------------------
// The constructor. We are passed the hardware pins and the optional initial limits.
//
//------------------------------------------------------------------------------------------------------------
UIEncoder::UIEncoder( uint8_t hwIdA, uint8_t hwIdB, int lower, int upper, bool activeLow ) {

    this -> hwIdA     = hwIdA;
    this -> hwIdB     = hwIdB;
    this -> activeLow = activeLow;

    lowerLimit = (( lower < upper ) ? lower : INT_MIN );
    upperLimit = (( upper > lower ) ? upper : INT_MAX );

    reset( );
}

//------------------------------------------------------------------------------------------------------------
// The encoder access routines. The encoder has a position, an lower and upper limit, a time delta between
// two turns and most importantly a callback routine to invoke when the position changed.
//
//------------------------------------------------------------------------------------------------------------
void UIEncoder::reset( ) {

    oldState        = (( getDataFunc != nullptr ) ? getDataFunc( hwIdA ) : false );
    position        = 0;
    positionPrev    = 0;
}

uint8_t UIEncoder::getHwIdA( ) {

    return ( hwIdA );
}

uint8_t UIEncoder::getHwIdB( ) {

    return ( hwIdB );
}

int UIEncoder::getPosition( ) {

    return ( position );
}

int UIEncoder::getUpperLimit( ) {

    return ( upperLimit );
}

int UIEncoder::getLowerLimit( ) {

    return ( lowerLimit );
}

uint32_t UIEncoder::getMillisBetweenRotations( ) {

    return ( positionTime - positionTimePrev );
}

void UIEncoder::setLimits( int lower, int upper ) {

    if ( lower < upper ) {

        lowerLimit = lower;
        upperLimit = upper;
    }
}

void UIEncoder::setPosition( int newPosition, bool suppressCallback ) {

    if (( newPosition >= lowerLimit ) && ( newPosition <= upperLimit )) {

        position      = newPosition;
        positionPrev  = newPosition;

        if (( newPosition != position ) && ( positionChangedFunc != nullptr ) && ( ! suppressCallback ))
        positionChangedFunc( this );
    }
}

void UIEncoder::attachPositionChanged( UIEncoderCallBackFunction functionId ) {

    positionChangedFunc = functionId;
}

void UIEncoder::attachGetDataFunction( UIGetDataFunction functionId ) {

    getDataFunc = functionId;
}

void UIEncoder::processTick( ) {

    if ( getDataFunc != nullptr ) {

        idAVal = getDataFunc( hwIdA );
        idBVal = getDataFunc( hwIdB );

        if ( activeLow ) {

        idAVal = !idAVal;
        idBVal = !idBVal;
        }
    }
    else return;

    if (( idAVal != oldState ) && ( idAVal == true )) {

        position = (( idBVal != idAVal ) ? position - 1 : position + 1 );

        if ( position > upperLimit ) position = upperLimit;
        if ( position < lowerLimit ) position = lowerLimit;

        if (( positionPrev != position ) && ( positionChangedFunc != nullptr ))
        positionChangedFunc( this );

        positionPrev      = position;
        positionTimePrev  = positionTime;
        positionTime      = CDC::getMillis( );
    }

    oldState = idAVal;
}
