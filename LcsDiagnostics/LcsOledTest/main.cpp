//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED - Test Program
//
//------------------------------------------------------------------------------------------------------------
// 
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

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig   cfg     = CDC::getConfigDefault( );
UIDisplay           *oled   = nullptr;

//----------------------------------------------------------------------------------------------------------
// Set up the CDC hardware data. Since we handle any board, all that needs to be configured are the two
// LEDs, the PFAIL pin, the two I2C channels and the CAN  Bus.
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
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
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = CDC::ALL_OK;

    rStat = initCdcLib( );
    if ( rStat != CDC::ALL_OK ) {

        printf( "Error in CDC init: %d\n", rStat );
        return( -1 );
    }

    oled = new UIDisplayOled( DT_OLED_DISPLAY_128x64_16_4, 17, 16, 0x3C );

    while ( true ) {

        oled -> setCursor( 1,1 );
        oled -> print( 'F' );

        CDC::sleepMillis( 1000 );

        oled -> setCursor( 1,1 );
        oled -> print( 'H' );

         CDC::sleepMillis( 1000 );
    }

    return( 0 );
}