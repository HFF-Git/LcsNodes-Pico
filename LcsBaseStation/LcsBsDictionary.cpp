//----------------------------------------------------------------------------------------
//
// LCS Base Station - Locomotive Dictionary - implementation file
//
//----------------------------------------------------------------------------------------
// The base station is the main node that manages among other things the active 
// locomotive session. When a session is established, it is necessary to get the 
// default configuration for the particular locomotive. The locomotive dictionary 
// is the part of the base station firmware that holds this kind of information.
// The locomotive dictionary is access by using the cabId as the key. The data 
// for a locomotive is entered via dedicated node control items.
//
//----------------------------------------------------------------------------------------
//
// LCS - Base Station
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
//----------------------------------------------------------------------------------------
#include "LcsBaseStation.h"

using namespace LCS;

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {


}; // namespace

//----------------------------------------------------------------------------------------
// ??? not clear what to do about this ... We would need to have an idea of the
// locomotive data. But does it have to be in the base station ?
//
// To be defined ...
//----------------------------------------------------------------------------------------
