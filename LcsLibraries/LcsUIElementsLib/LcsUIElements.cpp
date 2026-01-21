//----------------------------------------------------------------------------------------
//
// UI Elements - implementation file
//
//----------------------------------------------------------------------------------------
// ALl UI elements are kept in a linked list. The "tick" function advanced all 
// elements on the list by calling their state machines.
//
//----------------------------------------------------------------------------------------
//
// UIElements
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

//----------------------------------------------------------------------------------------
// Class static variable. "resList" is the head of the linked list of UI elements. 
// Each UI element created is added.
//
//----------------------------------------------------------------------------------------
static UIElements*  resList = NULL;

//----------------------------------------------------------------------------------------
// UI Element constructor. Every UI element we create has this class as a parent 
// and is added to the global linked list. This is necessary for processing the 
// ticks.
//
//----------------------------------------------------------------------------------------
UIElements::UIElements( bool atHead ) {

    if ( atHead ) insert( this );
    else          append( this );
}

//----------------------------------------------------------------------------------------
// "insert" and "append" will add the newly created UI element to the global 
// resource list. "insert" will insert at the head, "append" will append to the list.
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
// "tick" is the static routine to be called to advance the individual UI elements
// state machine. The routine is expected to return a return code, which is always
// success in this case.
//
//----------------------------------------------------------------------------------------
uint8_t UIElements::tick( void *uData ) {

    UIElements* res = resList;

    while ( res != NULL ) {

        res -> processTick( );
        res = res -> next;
    }

    return( NO_ERR );
}


// ??? should we have our own timer for the state machines ? The benefit
// is that we would not to do it outside of the UI elements library,
//
// ??? the timer would need to be at lest at a resolution that we set for 
// buttons and alike.
//
// ??? we would need a start option routine to start UI Element processing.