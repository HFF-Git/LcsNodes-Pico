///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect extension boards
//
///---------------------------------------------------------------------------------------
// This source file contains the occupancy detector driver routine. It is a fairly simple driver that just 
// reads in the track section state for the track detector circuit. The data is returned for the user defined
// DRV_OCC_READ_MASK. The driver date area is not used for now.
//
///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect extension boards
// Copyright (C) 2022 - 2024  Helmut Fieres
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
//----------------------------------------------------------------------------------------

#include "LcsDrvOccDetectLib.h"


// ???? need to have a better way to get to the CDC stuff....
// ??? or library stuff....

///---------------------------------------------------------------------------------------
// External declaration to global structures defined in "LcsRtSetup".
//
///---------------------------------------------------------------------------------------
namespace LCS {

    using namespace CDC;

    extern uint16_t debugMask;
};

///---------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space with all internal
// variables and routines local to the file. The second part contains the exported routines to be called by
// the core library and the firmware designers.
//
///---------------------------------------------------------------------------------------
namespace {

using namespace LCS;
using namespace CDC;

///---------------------------------------------------------------------------------------
// 
//
///---------------------------------------------------------------------------------------
uint8_t rNumI2C             = CDC_RN_EXT_NVM;
uint8_t PCA9555I2cAdrRoot   = 0x20;

///---------------------------------------------------------------------------------------
// The PCA9555 chip features a set of eight registers.
//
// Reg 0 - Input port 0
// Reg 1 - Input port 1
// Reg 2 - Output port 0
// Reg 3 - Output port 1
// Reg 4 - Polarity Inversion port 0
// Reg 5 - Polarity Inversion port 1
// Reg 6 - Configuration port 0
// Reg 7 - Configuration port 1
//
///---------------------------------------------------------------------------------------

///---------------------------------------------------------------------------------------
// The final I2C address consist of the chip fixed bits and the A0,2,3 section lines. A0 is zero hardwired,
// A2 and A1 represent the board Id.
//
///---------------------------------------------------------------------------------------
uint8_t mapI2CAdr( uint8_t boardId ) {

    return( PCA9555I2cAdrRoot | (( boardId % MAX_EXT_BOARD_MAP_ENTRIES ) << 1 ));
}

///---------------------------------------------------------------------------------------
// The occupancy detect board has the PCA9555 chip as an I2C to 16-bit port input/output chip. The "readReg"
// and "writeReg" routines allow to access the chip internal register.
// 
///---------------------------------------------------------------------------------------
uint8_t readReg( uint8_t i2cAdr, uint8_t reg ) {

    uint8_t rStat = NO_ERR;
    uint8_t buf[ 2 ];
    
    rStat = i2cWrite( rNumI2C, i2cAdr, &reg, 1, true );
    if ( rStat == CDC::NO_ERR ) {

        rStat = i2cRead( rNumI2C, i2cAdr, buf, 1 );
        return( buf[ 0 ] );
    }
    else return( ALL_OK );
}

uint8_t writeReg( uint8_t i2cAdr, uint8_t reg, uint8_t val ) {

    uint8_t buf[ 2 ];
    buf[ 0 ] = reg;
    buf[ 1 ] = val;

    return( i2cWrite( rNumI2C, i2cAdr, buf, 2 ));
}

} // namespace


namespace LCS {

///---------------------------------------------------------------------------------------
// Each driver is just a function to handle the request.
//
///---------------------------------------------------------------------------------------
uint8_t lcsDrvOccDetect( uint16_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    switch( item ) {

        //----------------------------------------------------------------------------------------------------
        // The driver reset function. All drivers must support a reset item. For the PCA9555 we need to set 
        // the IO direction an data inversion registers. These are the registers found on the chip:
        //
        // Reg 0 - Input port 0
        // Reg 1 - Input port 1
        // Reg 2 - Output port 0
        // Reg 3 - Output port 1
        // Reg 4 - Polarity Inversion port 0
        // Reg 5 - Polarity Inversion port 1
        // Reg 6 - Configuration port 0
        // Reg 7 - Configuration port 1
        //
        //----------------------------------------------------------------------------------------------------
        case ITEM_ID_RESET: {

            uint8_t rStat =         writeReg( mapI2CAdr( boardId ), 4, 0xFF );
            if ( rStat == ALL_OK )  writeReg( mapI2CAdr( boardId ), 5, 0xFF );
            if ( rStat == ALL_OK )  writeReg( mapI2CAdr( boardId ), 6, 0xFF );
            if ( rStat == ALL_OK )  writeReg( mapI2CAdr( boardId ), 7, 0xFF );

            printf( "Occ Detect RESET: ret: %d\n", rStat );
            return( rStat );

        } break;

        //----------------------------------------------------------------------------------------------------
        // Read mask. All 16 occupancy detect inputs are stored in a 16-bit word. 
        //
        //----------------------------------------------------------------------------------------------------
        case DRV_OCC_READ_MASK: {

            uint8_t tmp1 = readReg( mapI2CAdr( boardId ), 0 );
            uint8_t tmp2 = readReg( mapI2CAdr( boardId ), 1 );

            *arg1 = ( tmp1 << 8 ) | tmp2; 
            return( ALL_OK );

        } break;

        default: return( ERR_INVALID_DRV_ITEM );
    }
}

} // namespace