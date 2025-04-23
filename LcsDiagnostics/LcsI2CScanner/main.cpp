//------------------------------------------------------------------------------------------------------------
//
// LCS - I2C Bus Scanner
//
//------------------------------------------------------------------------------------------------------------
// We need a program to see what is on the I2C bus. The same functionality is also provided i the RtLLb.
// Perhaps retire this program...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2025 Helmut Fieres
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
#include "LcsCdcDescMapDefaults.h"

using namespace CDC;

CdcResourceDescMap dMap = RES_MAP_RP_2040;

int main( ) {

    uint8_t rStat = cdcInit( &dMap );

    sleepMillis( 2000 );
    configureConsoleIO( );

    if ( rStat == NO_ERR ) rStat = configureDio( CDC_RN_ACTIVITY_LED );
    if ( rStat == NO_ERR ) rStat = writeDio( CDC_RN_ACTIVITY_LED, true );

    if ( rStat == NO_ERR ) rStat = configureI2C( CDC_RN_NVM );
    if ( rStat != NO_ERR ) printf( "Error configuring NVM I2C channel: %d\n", rStat );

    if ( rStat == NO_ERR ) rStat = configureI2C( CDC_RN_EXT_NVM );
    if ( rStat != NO_ERR ) printf( "Error configuring EXT I2C channel: %d\n", rStat );

    if ( rStat != NO_ERR ) return( 0 );

    int scanCount = 0;  
  
    while( true ) {

        printf( "Scanning (%d) ... \n", scanCount );

        printf( "Scanning NVM I2C Bus\n" );
        scanI2CBus( CDC_RN_NVM );
        printf( "\n" );

        printf( "Scanning EXT I2C Bus\n" );
        scanI2CBus( CDC_RN_EXT_NVM );
        printf( "\n" );

        scanCount++;
        sleepMillis( 5000 );
    }
  
    return( NO_ERR );
}