//------------------------------------------------------------------------------------------------------------
//
// LCS - Diesel Throttle
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
//
// ??? both throttle should share a throttle lib with common code...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Code - Raspberry PI Pico Implementation
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
#include "LcsCdcDescMapDefaults.h"
#include "LcsRuntimeLib.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------------------------
CdcResourceDescMap dMap;

//----------------------------------------------------------------------------------------------------------
// "printStatus" is a little helper function for the initialization routines protocol printing. 
//
//----------------------------------------------------------------------------------------------------------
uint8_t printStatus( uint8_t status ) {

    if ( status == ALL_OK ) printf( "-> OK" );
    else                    printf( "-> FAILED: %d\n", status );
       
    return ( status );
}


//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initThrottle( ) {

    uint8_t rStat = ALL_OK;

    printf( "LCS Diesel Cab Throttle\n" );
    printf( "Under construction ... stay tuned\n\n");
    sleepMillis( 2000 );

    return( 99 );
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

    uint8_t rStat = ALL_OK;
    if ( rStat == ALL_OK ) rStat = initThrottle( );
    if ( rStat == ALL_OK ) rStat = startLcsRuntime( );
    return( ALL_OK );
}