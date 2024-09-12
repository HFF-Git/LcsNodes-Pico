//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect extension boards
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the ...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect extension boards
// Copyright (C) 2022 - 2024  Helmut Fieres
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

#include "LcsDrvOccDetectLib.h"

//------------------------------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space with all internal
// variables and routines local to the file. The second part contains the exported routines to be called by
// the core library and the firmware designers.
//
//------------------------------------------------------------------------------------------------------------
namespace {

//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
uint8_t   sclPin    = 0;
uint8_t   i2cAdr    = 0x20;
uint16_t  ioData    = 0;


//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
uint8_t initDrv( ) {

    // CDC::configureI2C( 0, 0 );

    return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
//
// ??? use CDC....
//------------------------------------------------------------------------------------------------------------
bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

uint8_t readReg( uint8_t reg ) {

    uint8_t buf[ 2 ];
    uint8_t rStat = CDC::ALL_OK;
    
    rStat = CDC::i2cWrite( sclPin, i2cAdr, &reg, 1 );
    if ( rStat == CDC::ALL_OK ) {

        rStat = CDC::i2cRead( sclPin, i2cAdr, buf, 1 );
        return( buf[ 0 ] );
    }
    else return( 0 );
}

uint8_t writeReg( uint8_t reg, uint8_t val ) {

    uint8_t buf[ 2 ];
    buf[ 0 ] = reg;
    buf[ 1 ] = val;

    return( CDC::i2cWrite( sclPin, i2cAdr, buf, 2 ));
}

bool chipReady( uint8_t sclPin, uint8_t i2cAdr ) {

    uint8_t ret = 1;
    uint8_t tmp = 0;

    while ( ret != CDC::ALL_OK ) {

        ret = CDC::i2cWrite( sclPin, i2cAdr, &tmp, 1 );
    }

    return ( true );
}

} // namespace



namespace LCS {

//------------------------------------------------------------------------------------------------------------
// Each driver is just a function to handle the request.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsDrvOccDetect( uint8_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    switch( item ) {

        case ITEM_ID_RESET: {

            // ??? set inversion bit to one. Explain ...
            writeReg( 4, 0xFF );
            writeReg( 5, 0xFF );

            // ??? set pins as input
            writeReg( 6, 0xFF );
            writeReg( 7, 0xFF );

            // ??? e.g. setting the IO direction...
            //
            // arg1 -> pin, or a port or a two ports bit mask
            // arg2 -> 0 = output ( PCA9555 expects a zero for output )  & ~ ( 1 << pin )
            // arg2 -> 1 = input  ( PCA9555 expects a one for input )    | ( 1 << pin )
            //
            // have a 16 bit mask for the bits, write both a when we set something ?
            // writeI2C ( config reg 1, 2 )

            return( ALL_OK );

        } break;

        case DRV_OCC_READ_MASK: {

            uint8_t tmp1 = readReg( 0 );
            uint8_t tmp2 = readReg( 1 );

            *arg1 = ( tmp1 << 8 ) | tmp2; 

            return( ALL_OK );

        } break;

        default: return( ERR_INVALID_DRV_ITEM );
    }
}

} // namespace