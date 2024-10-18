//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - node access routines.
//
//------------------------------------------------------------------------------------------------------------
// The file contains the part of the LCS Runtime that implements the GET, SET and REQ access for the driver
// that manages an extension board.
//
// ??? what else to explain ?
//------------------------------------------------------------------------------------------------------------
//
// LCS - Core Library
// Copyright (C) 2021 - 2024  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation, either version 3 of the License,
// or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details. You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.
//
//------------------------------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//------------------------------------------------------------------------------------------------------------
// External declaration to global structures defined in "LcsRtSetup".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t             debugMask;
    extern LCS::LcsCdcDesc      cdcMap;
    extern LCS::LcsNodeMap      nodeMap;
    extern LCS::LcsDrvMap       drvMap;
};

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Utility routines.
//
//------------------------------------------------------------------------------------------------------------
bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

uint8_t lowByte( uint16_t arg ) { 
    
    return( arg & 0xFF ); 
}

uint8_t highByte( uint16_t arg ) { 
    
    return( arg >> 8 ); 
}

} // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "drvGet" returns a value from the driver data array. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t drvGet( uint8_t boardId, uint8_t item, uint16_t *arg ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES ) return ( ERR_INVALID_BOARD_ID );

    if ( isInRangeU( item, IR_ATTR_MEM_RANGE_START, IR_ATTR_MEM_RANGE_END )) {

        *arg = drvMap.map -> extBoard -> driverData[ item ];
        return( ALL_OK );
    }
    else if ( isInRangeU( item, IR_ATTR_NVM_RANGE_START, IR_ATTR_NVM_RANGE_END )) {

       if ( extNvmGetWord( boardId, 0, arg ) != ALL_OK ) return( 0 ); // ??? fix
       return( ALL_OK );
    }
    else return( ERR_INVALID_ITEM_ID ); 
}

//------------------------------------------------------------------------------------------------------------
// "drvPut" sets a value in the driver data array. Note that this is only the MEM portion. The NVM portion
// is write disabled after initial configuration.
//
//------------------------------------------------------------------------------------------------------------
uint8_t drvPut(uint8_t boardId, uint8_t item, uint16_t arg ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES ) return ( ERR_INVALID_BOARD_ID );

     if ( isInRangeU( item, IR_ATTR_MEM_RANGE_START, IR_ATTR_MEM_RANGE_END )) {

        drvMap.map -> extBoard-> driverData[ item ] = arg;
        return( ALL_OK );
    }
    else if ( isInRangeU( item, IR_ATTR_NVM_RANGE_START, IR_ATTR_NVM_RANGE_END )) {

       if ( extNvmPutWord( boardId, 0, arg ) != ALL_OK ) return( 0 ); // ??? fix
       return( ALL_OK );
    }
    else return( ERR_INVALID_ITEM_ID ); 
}

//------------------------------------------------------------------------------------------------------------
// "drvReq" is the entry point to an extension board. For each extension board type there is driver function.
// This function is called when we access that extension board.
//
//------------------------------------------------------------------------------------------------------------
uint8_t drvReq( uint8_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES ) return ( ERR_INVALID_BOARD_ID );

    if ( drvMap.map[ boardId - 1 ].drvFunc != nullptr ) {

        return( drvMap.map[ boardId - 1 ].drvFunc( boardId - 1, item, arg1, arg2 ));
    }
    else return( ERR_EXT_BOARD_NOT_VALID );
}

} // namespace LCS

