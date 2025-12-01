//----------------------------------------------------------------------------------------
//
// LCS - OLED - Test Program
//
//----------------------------------------------------------------------------------------
// A simple OLED test program for the SDD1306 library.
// 
//----------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2024 Helmut Fieres
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsMainControllerBoardDesc.h"
#include "LcsCdcLib.h"
#include "LcsUIElements.h"

using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap  dMap    = LCS_MAIN_CONTROLLER_BOARD_DESC_B_02_00;
UIDisplay           *oled   = nullptr;

//----------------------------------------------------------------------------------------
// Set up the CDC hardware data. Since we handle any board, all that needs to be 
// configured are the two LEDs, the PFAIL pin, the two I2C channels and the CAN  Bus.
//
//----------------------------------------------------------------------------------------
uint8_t initCdcLib( ) {

    cdcInit( &dMap );
    sleepMillis( 2000 );
    configureUsbIO( );

    printResourceDescMap( &dMap );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = initCdcLib( );
    if ( rStat != NO_ERR ) {

        printf( "Error in CDC init: %d\n", rStat );
        CDC::sleepMillis( 5000 );
        return( -1 );
    }

    oled = new UIDisplayOledI2C( DT_OLED_DISPLAY_128x64, CDC_RN_EXT_NVM, 0x3C );

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