//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for SERVO boards - Include file
//
//------------------------------------------------------------------------------------------------------------
// 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include File
// Copyright (C) 2022 - 2023  Helmut Fieres
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
#include "LcsRtLibInt.h"
#include "LcsRuntimeLib.h"

namespace LCS {

//------------------------------------------------------------------------------------------------------------
// Driver items.
//
//------------------------------------------------------------------------------------------------------------
enum LcsDrvServoItems : uint8_t {

    DRV_SERVO_xxx = IR_USER_RANGE_START + 0,
    
  
};

//------------------------------------------------------------------------------------------------------------
// Driver function.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsDrvServo( uint16_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 );

} // namespace

#endif
