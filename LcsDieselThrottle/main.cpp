//----------------------------------------------------------------------------------------
//
// LCS - Diesel Throttle
//
//----------------------------------------------------------------------------------------
// This source file contains ...
//
//
// ??? both throttle should share a throttle lib with common code...
//
//----------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Code - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
#include "LcsDieselThrottleBoardDesc.h"
#include "LcsCdcLib.h"
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

    if ( status == NO_ERR ) printf( "-> OK" );
    else                    printf( "-> FAILED: %d\n", status );
       
    return ( status );
}


//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initBlockNode( ) {

    uint8_t rStat = NO_ERR;

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


    return( NO_ERR );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = NO_ERR;
    if ( rStat == NO_ERR ) rStat = initBlockNode( );
    if ( rStat == NO_ERR ) rStat = startLcsRuntime( );
    return( NO_ERR );
}