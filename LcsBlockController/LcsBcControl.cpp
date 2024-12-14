//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller - Control Logic
//
//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller
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

#include "LcsBlockController.h"

// ??? contains the main code, the setup, the message handler, etc.

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
LcsBlockControl::LcsBlockControl(  ) {

   

}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
 uint8_t LcsBlockControl::handleLcsRequest( uint8_t *msg ) {

    switch ( msg[ 0 ] ) {

        // ??? define a few request for testing ...

        default: ;
    }

    return( ALL_OK );
 }




