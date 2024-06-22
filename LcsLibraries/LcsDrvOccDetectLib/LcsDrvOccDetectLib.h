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

#include "../LcsCdcLib/LcsCdcLib.h"

//------------------------------------------------------------------------------------------------------------
// Driver items...
//
//------------------------------------------------------------------------------------------------------------
enum DrvItems {

  
  
};



//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
struct LcsDrvOccDetect  {

  public:

  LcsDrvOccDetect( );

  uint8_t init( uint16_t flags );
  uint8_t reset( uint16_t flags );
  
  uint8_t config( uint8_t padId, uint8_t item, uint16_t arg1, uint16_t arg2 = 0 );
  uint8_t control( uint8_t padId, uint8_t item, uint16_t arg1, uint16_t arg2 = 0 );
  uint8_t info( uint8_t padId, uint8_t item, uint16_t *arg1, uint16_t *arg2 = nullptr );
  
  uint8_t read( uint8_t padId, uint16_t *arg );
  uint8_t write( uint8_t padId, uint16_t arg );
  uint8_t read( uint8_t padId, uint8_t *buf, uint8_t *bufLen );
  uint8_t write( uint8_t padId, uint8_t *buf, uint8_t bufLen );

  private:
  
};

#endif
