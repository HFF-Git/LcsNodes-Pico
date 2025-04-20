//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED - Test Program
//
//------------------------------------------------------------------------------------------------------------
// A simple OLED test program for the SDD1306 library.
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
#include "LcsUIElements.h"

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
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
UIDisplay           *oled   = nullptr;

//----------------------------------------------------------------------------------------------------------
// Set up the CDC hardware data. Since we handle any board, all that needs to be configured are the two
// LEDs, the PFAIL pin, the two I2C channels and the CAN  Bus.
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
//----------------------------------------------------------------------------------------------------------
uint8_t initCdcLib( ) {

    cdcInit( &cMap );
    sleepMillis( 2000 );
    configureConsoleIO( );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = initCdcLib( );
    if ( rStat != CDC::NO_ERR ) {

        printf( "Error in CDC init: %d\n", rStat );
        CDC::sleepMillis( 5000 );
        return( -1 );
    }

    oled = new UIDisplayOled( DT_OLED_DISPLAY_128x64, cMap.i2cSclPin_1, cMap.i2cSdaPin_1, 0x3C );

    while ( true ) {

        oled -> setCursor( 1,1 );
        oled -> print( "FH" );

        CDC::sleepMillis( 1000 );

        oled -> setCursor( 1,1 );
        oled -> print( "HF" );

        CDC::sleepMillis( 1000 );

        oled -> setCursor( 2,1 );
        oled -> print( 'F' );

        CDC::sleepMillis( 1000 );

        oled -> setCursor( 2,1 );
        oled -> print( 'H' );

        CDC::sleepMillis( 1000 );
    }

    return( 0 );
}