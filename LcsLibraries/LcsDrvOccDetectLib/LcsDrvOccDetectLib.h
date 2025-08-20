///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect boards - Include file
//
///---------------------------------------------------------------------------------------
// The occupancy detect extension board is a simple board that detects the presence of
// an engine in a track section. 
//
///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect boards
// Copyright (C) 2023 - 2025  Helmut Fieres
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
#ifndef LcsDvrOccDetectLib_h
#define LcsDvrOccDetectLib_h

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

namespace LCS {

///---------------------------------------------------------------------------------------
// Driver items. They are allocated in the user defined item range. Their meaning is
// board type specific.
//
///---------------------------------------------------------------------------------------
enum LcsDrvOccDetectItems : uint8_t {

  DRV_OCC_READ_MASK    = IR_USER_RANGE_START + 0,
  DRV_OCC_yyy          = IR_USER_RANGE_START + 1,
  
};

///---------------------------------------------------------------------------------------
// Driver function. This function is called when there is a "drvReq" call.
//
///---------------------------------------------------------------------------------------
uint8_t lcsDrvOccDetect( uint16_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 );

} // namespace 

#endif
