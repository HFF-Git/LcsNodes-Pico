//----------------------------------------------------------------------------------------
//
// UI Elements - implementation file
//
//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
//
// UIElements
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
// Class static variable. "resList" is the head of the linked list of UI elements. Each
// UI element created
// is added.
//
//----------------------------------------------------------------------------------------
static UIElements*  resList = NULL;

//----------------------------------------------------------------------------------------
// UI Element constructor. Every UI element we create has this class as a parent and is
// added to the global linked list. This is necessary for processing the ticks, which is
// essentially just running down that list and calling the respective handler in the UI
// element.
//
//----------------------------------------------------------------------------------------
UIElements::UIElements( bool atHead ) {

    if ( atHead ) insert( this );
    else          append( this );
}

//----------------------------------------------------------------------------------------
// Resource ID getter/setter.
//
//----------------------------------------------------------------------------------------
int UIElements::getResId( ) { 
  
    return( resId );
}

void UIElements::setResId( int arg ) {

    resId = arg;
}

//----------------------------------------------------------------------------------------
// "setup" is the static routine to place in the program setup routine. So far, there is
// nothing to do.
//
//----------------------------------------------------------------------------------------
uint8_t UIElements::setup( ) {

    return ( 0 );
}

//----------------------------------------------------------------------------------------
// "insert" and "append" will add the newly created UI element to the global resource
// list. "insert" will insert at the head, "append" will append to the list.
//
//----------------------------------------------------------------------------------------
void UIElements::insert( UIElements* res ) {

    if ( resList != NULL ) {

        res -> next = resList;
        resList = res;
    }
    else resList = res;
}

void UIElements::append( UIElements* res ) {

    if ( resList != NULL ) {

        UIElements *temp = resList;
        while ( temp -> next != nullptr ) temp = temp -> next;

        temp -> next = res;
        res -> next = nullptr;
    }
    else resList = res;
}

//----------------------------------------------------------------------------------------
// "tick" is the static routine to be called to advance the UI elements state machine.
//
//----------------------------------------------------------------------------------------
uint8_t UIElements::tick( ) {

    UIElements* res = resList;

    while ( res != NULL ) {

        res -> processTick( );
        res = res -> next;
    }

    return( 0 );
}