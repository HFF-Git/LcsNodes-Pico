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

using namespace CDC;

//----------------------------------------------------------------------------------------------------------
//
// Quick hack for configuring the two I2C channels...
//----------------------------------------------------------------------------------------------------------
const uint8_t RES_ID_LED        = 1;
const uint8_t RES_ID_NVM_I2C    = 2;
const uint8_t RES_ID_EXT_I2C    = 3;

const uint8_t LED_PIN           = 15;  

const uint8_t NVM_I2C_SCL_PIN   = 3;
const uint8_t NVM_I2C_SDA_PIN   = 2;

const uint8_t EXT_I2C_SCL_PIN   = 17;
const uint8_t EXT_I2C_SDA_PIN   = 16;

//----------------------------------------------------------------------------------------------------------
// Init the CDC library.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initCdcLib( ) {

    cdcInit( );
    sleepMillis( 2000 );
    configureConsoleIO( );

    configureController(    CDC_CF_RP_PICO, 
                            CDC_CF_C_RP_2040, 
                            260 * 1024,
                            0, 
                            2000, 
                            3300, 
                            1024, 
                            LED_PIN, 
                            CDC::UNDEFINED_PIN );

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------------------------
// Set up Active LED pin and set it to on.
//
//----------------------------------------------------------------------------------------------------------
uint8_t configureLed( ) {

    uint8_t rStat = configureDio( RES_ID_LED, LED_PIN, 0, CDC_DIO_OUT );
    if ( rStat == NO_ERR ) writeDio( RES_ID_LED, true );

    return( NO_ERR );
}

//----------------------------------------------------------------------------------------------------------
// Configure the two I2C channels.
//
//----------------------------------------------------------------------------------------------------------
uint8_t configureI2cChannels(  ) {

    uint8_t rStat = configureI2C( RES_ID_NVM_I2C, NVM_I2C_SCL_PIN, NVM_I2C_SDA_PIN );
    if ( rStat != NO_ERR ) {

        printf( "Error configuring NVM I2C channel: %d\n", rStat );
    }

    rStat = CDC::configureI2C( RES_ID_EXT_I2C, EXT_I2C_SCL_PIN, EXT_I2C_SDA_PIN );
    if ( rStat != NO_ERR ) {

        printf( "Error configuring EXT I2C channel: %d\n", rStat );
    }

    return( rStat );
}

//----------------------------------------------------------------------------------------------------------
// "scanI2CBus" is the loop through all possible I2C addresses in an I2C bus. If a valid one is found, the 
// address is printed.
//
//----------------------------------------------------------------------------------------------------------
void scanI2cBus( uint8_t resId ) {

  uint8_t rStat     = 0;
  uint8_t i2cAdr    = 0;
  uint8_t nDevices  = 0;
  uint8_t buf       = 0;

  printf( "Scanning for I2C Bus, resId: %d\n", resId );

  for ( i2cAdr = 1; i2cAdr < 127; i2cAdr++ ) {
    
    rStat = CDC::i2cRead( resId, i2cAdr, &buf, 1 );

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

    rStat = configureI2cChannels( );
    rStat = configureLed( );
 
    int scanCount = 0;  
  
    while( true ) {

        printf( "Scanning (%d) ... \n", scanCount );

        printf( "Scanning NVM I2C Bus\n" );
        scanI2cBus( RES_ID_NVM_I2C );
        printf( "\n" );

        printf( "Scanning EXT I2C Bus\n" );
        scanI2cBus( RES_ID_EXT_I2C );
        printf( "\n" );

        scanCount++;
        sleepMillis( 5000 );
    }
  
    return( 0 );
}