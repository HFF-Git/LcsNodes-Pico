//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect boards - Include file
//
//------------------------------------------------------------------------------------------------------------
// 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect boards
// Copyright (C) 2023 - 2024  Helmut Fieres
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
#ifndef LcsDvrServoLib_h
#define LcsDvrServoLib_h

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//------------------------------------------------------------------------------------------------------------
// Driver items.
//
//------------------------------------------------------------------------------------------------------------
enum LcsDrvOccDetectItems : uint8_t {

   DVR_OCC_DETECT_INIT = 1,
   DVR_OCC_DETECT_RESET = 2,
   
    // ...
  
};

//------------------------------------------------------------------------------------------------------------
// Driver function.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcdDrvOccDetect( uint8_t boardId, uint8_t item, uint16_t arg1, uint16_t *arg2 );

#endif
