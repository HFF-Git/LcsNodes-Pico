//------------------------------------------------------------------------------------------------------------
//
// LCS - Block Controller
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Block Controller - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2024 Helmut Fieres
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

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg;


//----------------------------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

  cfg = CDC::getConfigDefault( );

  //--------------------------------------------------------------------------------------------------------
  // Current mapping: Main Controller Board B.01.00 - PICO - newest version.
  //
  //--------------------------------------------------------------------------------------------------------
  cfg.ADC_PIN_0             = 26;
  cfg.ADC_PIN_1             = 27;

  cfg.PWM_PIN_0             = 20;
  cfg.PWM_PIN_1             = 21;

  cfg.PFAIL_PIN             = 7;
  cfg.EXT_INT_PIN           = 22;
  cfg.READY_LED_PIN         = 14;
  cfg.ACTIVE_LED_PIN        = 15;

  cfg.DIO_PIN_0             = 9;
  cfg.DIO_PIN_1             = 8;
  cfg.DIO_PIN_2             = 10;
  cfg.DIO_PIN_3             = 11;
  cfg.DIO_PIN_4             = 21;
  cfg.DIO_PIN_5             = 20;
  cfg.DIO_PIN_6             = 19;
  cfg.DIO_PIN_7             = 18;

  cfg.NVM_I2C_SCL_PIN       = 17;
  cfg.NVM_I2C_SDA_PIN       = 16;
  cfg.NVM_I2C_ADR_ROOT      = 0x50;

  cfg.EXT_I2C_SCL_PIN       = 3;
  cfg. EXT_I2C_SDA_PIN      = 2;
  cfg.EXT_I2C_ADR_ROOT      = 0x50;

  CDC::printConfigInfo( &cfg );
}

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

  setupConfigInfo( );
 
  uint8_t rStat = CDC::init( &cfg );

  if ( rStat != ALL_OK ) {


  }


  printf( "LCS Block Controller\n" );

  
  if ( rStat != 0 )  printf( "Err code: %d\n", rStat );
  else printf( "OK\n" );

  return( 0 );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {


  return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t startLcsRuntime( ) {


  return( ALL_OK );
}



//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    initLcsRuntime( );
    registerCallbacks( );
    startLcsRuntime( );
    return( 0 );
}