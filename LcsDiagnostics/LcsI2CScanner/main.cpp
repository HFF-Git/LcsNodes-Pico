//------------------------------------------------------------------------------------------------------------
//
// LCS - I2C Bus Scanner
//
//------------------------------------------------------------------------------------------------------------
// We need a program to see what is on the I2C bus...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
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

//----------------------------------------------------------------------------------------------------------
// The default CDC configuration data.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg = CDC::getConfigDefault( );

//----------------------------------------------------------------------------------------------------------
// Init the CDC library.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initCdcLib( ) {

  CDC::sleepMillis( 2000 );
  CDC::configureConsoleIO( );

  cfg.NVM_I2C_SCL_PIN = 3;
  cfg.NVM_I2C_SDA_PIN = 2;

  cfg.EXT_I2C_SCL_PIN = 17;
  cfg.EXT_I2C_SDA_PIN = 16;

  return( CDC::init( &cfg ));
}

//----------------------------------------------------------------------------------------------------------
// Configure the two I2C channels.
//
//----------------------------------------------------------------------------------------------------------
uint8_t configI2C( uint8_t sclPin, uint8_t sdaPin ) {

  printf( "Configuring the I2C Bus, slcPin: %d, sdaPin: %d\n", sclPin, sdaPin );

  return( CDC::configureI2C( sclPin, sdaPin ));
}

//----------------------------------------------------------------------------------------------------------
// "scanI2CBus" is the loop through all possible I2C addresses in an I2C bus. If a valid one is found, the 
// address is printed.
//
//----------------------------------------------------------------------------------------------------------
void scanI2CBus( uint8_t sclPin, uint8_t sdaPin ) {

  uint8_t rStat     = 0;
  uint8_t i2cAdr    = 0;
  uint8_t nDevices  = 0;
  uint8_t buf       = 0;

  printf( "Scanning for I2C Bus, sclPin: %d, sdaPin: %d\n", sclPin, sdaPin );

  for ( i2cAdr = 1; i2cAdr < 127; i2cAdr++ ) {

    rStat = CDC::i2cRead( sclPin, i2cAdr, &buf, 1 );

    if ( rStat == 0 ) {

      printf( "I2C device found at i2cAdr 0x%x\n", i2cAdr );
      nDevices ++;
    }
  }

  if ( nDevices == 0 )  printf( "No I2C devices found\n" );
  else                  printf( "Scan done\n" );
}

//----------------------------------------------------------------------------------------------------------
// Main. After we initialized the CDC layer, there is a loop which alternatively checks for the NVM or EXT
// I2C bus.
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

  uint8_t rStat = initCdcLib( ); 

  CDC::sleepMillis( 5000 );

  if ( rStat != 0 ) {
    
    printf( "Init CDC Library, Err code: %d\n", rStat );
    return( -1 );
  }

  rStat = configI2C( cfg.NVM_I2C_SCL_PIN, cfg.NVM_I2C_SDA_PIN );
  if ( rStat != CDC::NO_ERR ) {

    printf( "Configuring NVM I2C, error: %d\n", rStat );
  }

  rStat = configI2C( cfg.EXT_I2C_SCL_PIN, cfg.EXT_I2C_SDA_PIN );
  if ( rStat != CDC::NO_ERR ) {

    printf( "Configuring EXT I2C, error: %d\n", rStat );
  }

  int scanCount = 0;  
  
  while( true ) {

    printf( "Scanning (%d) ... \n", scanCount );

    if ( cfg.NVM_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) {

      printf( "Scanning NVM I2C Bus\n" );
      scanI2CBus(  cfg.NVM_I2C_SCL_PIN, cfg.NVM_I2C_SDA_PIN );
      printf( "\n" );
    }

    if ( cfg.EXT_I2C_SCL_PIN != CDC::UNDEFINED_PIN ) {

      printf( "Scanning EXT I2C Bus\n" );
      scanI2CBus(  cfg.EXT_I2C_SCL_PIN, cfg.EXT_I2C_SDA_PIN );
      printf( "\n" );
    }

    scanCount++;
    CDC::sleepMillis( 5000 );
  }
  
  return( 0 );
}