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

CdcResourceMap cMap = {

    .cFamily                = CDC_CF_RP_PICO,
    .cType                  = CDC_CF_C_RP_2040,
    .memorySize             = 260 * 1024,

    .adcRefVoltageMillis    = 3300,
    .adcDigitRange          = 1024,

    .ledPin                 = 15,
    .pFailPin               = 7,
    
    .adcPin_0               = 26,
    .adcPin_1               = 27,

    .dioPin_0               = 8,
    .dioPin_1               = 9,
    .dioPin_2               = 10,
    .dioPin_3               = 11,
    .dioPin_4               = 21,
    .dioPin_5               = 20,
    .dioPin_6               = 19,
    .dioPin_7               = 18,

    .pwmPin_0               = 20,
    .pwmPin_1               = 21,
  
    .i2cSclPin_0            = 3,
    .i2cSdaPin_0            = 2,

    .i2cSclPin_1            = 17,
    .i2cSdaPin_1            = 16,
};

//----------------------------------------------------------------------------------------------------------
// Init the CDC library.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initCdcLib( ) {

    cdcInit( &cMap );
    sleepMillis( 2000 );
    configureConsoleIO( );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------------------------
// Set up Active LED pin and set it to on.
//
//----------------------------------------------------------------------------------------------------------
uint8_t configureLed( ) {

    uint8_t rStat = configureDio( cMap.ledPin, CDC_DIO_OUT );
    if ( rStat == NO_ERR ) writeDio( cMap.ledPin, true );

    return( NO_ERR );
}

//----------------------------------------------------------------------------------------------------------
// Configure the two I2C channels.
//
//----------------------------------------------------------------------------------------------------------
uint8_t configureI2cChannels(  ) {

    uint8_t rStat = configureI2C( cMap.i2cSclPin_0, cMap.i2cSdaPin_0 );
    if ( rStat != NO_ERR ) {

        printf( "Error configuring NVM I2C channel: %d\n", rStat );
    }

    rStat = configureI2C( cMap.i2cSclPin_1, cMap.i2cSdaPin_1 );
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
void scanI2cBus( uint8_t sclPin ) {

  uint8_t rStat     = 0;
  uint8_t i2cAdr    = 0;
  uint8_t nDevices  = 0;
  uint8_t buf       = 0;

  printf( "Scanning for I2C Bus, pin: %d\n", sclPin );

  for ( i2cAdr = 1; i2cAdr < 127; i2cAdr++ ) {
    
    rStat = i2cRead( sclPin, i2cAdr, &buf, 1 );

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
        scanI2cBus( cMap.i2cSclPin_0 );
        printf( "\n" );

        printf( "Scanning EXT I2C Bus\n" );
        scanI2cBus( cMap.i2cSclPin_1 );
        printf( "\n" );

        scanCount++;
        sleepMillis( 5000 );
    }
  
    return( NO_ERR );
}