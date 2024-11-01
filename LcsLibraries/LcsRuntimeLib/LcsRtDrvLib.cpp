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

//------------------------------------------------------------------------------------------------------------
// "buildDrvBoardDescArea" will create a default data area for the extension board NVM.
//
//------------------------------------------------------------------------------------------------------------
uint8_t buildDrvBoardDescArea( uint8_t boardId ) {

    uint8_t         rStat;
    LcsDrvBoardDesc tmp;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "buildDrvBoardDescArea, boardId: %d\n", boardId );

    rStat = extNvmPutBytes( boardId, 0, (uint8_t *) &tmp, sizeof( tmp ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_SETUP )) 
        printf( "buildDrvBoardDescArea, stat: %d\n", rStat  );

    return( rStat );
}

} // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {
    
//------------------------------------------------------------------------------------------------------------
//
// 
//------------------------------------------------------------------------------------------------------------
uint8_t drvInit( ) {

    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "drvGet" returns a value from the driver data array. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t drvGet( uint8_t boardId, uint8_t item, uint16_t *arg ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES )                 return ( ERR_INVALID_BOARD_ID );
    if ( ! drvMap.map[ boardId ].flags & BF_EXT_BOARD_PRESENT ) return( ERR_INVALID_BOARD_ID );

    if ( isInRangeU( item, IR_ATTR_MEM_RANGE_START, IR_ATTR_MEM_RANGE_END )) {

        *arg = drvMap.map[ boardId ].extBoard.driverData[ item - IR_ATTR_MEM_RANGE_START ];
        return( ALL_OK );
    }
    else if ( isInRangeU( item, IR_ATTR_NVM_RANGE_START, IR_ATTR_NVM_RANGE_END )) {

        uint32_t ofs =  offsetof( LcsDrvBoardDesc, driverData ) + 
                        (( item - IR_ATTR_NVM_RANGE_START ) * sizeof( uint16_t ));

       if ( extNvmGetWord( boardId, ofs, arg ) != ALL_OK ) return( ERR_DRV_GET_ERR );
       return( ALL_OK );
    }
    else if ( item == ITEM_ID_BOARD_VERSION ) {

        *arg = drvMap.map[ boardId ].extBoard.head.boardVersion;
        return( ALL_OK );
    }
    else if ( item == ITEM_ID_TYPE ) {

        *arg = drvMap.map[ boardId ].extBoard.head.boardType;
        return( ALL_OK );
    }
    else return( ERR_INVALID_ITEM_ID ); 
}

//------------------------------------------------------------------------------------------------------------
// "drvPut" sets a value in the driver data array. Note that this is during normal operations only the MEM 
// portion. The NVM chip on the extension board is write disabled after initial configuration. When the 
// jumper on the board is taken out, writing to the NVM is enabled. The PUT and GET routines can be called
// for a board that has no driver associated yet. This way, for example, the driver type and other initial
// data can be set. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t drvPut(uint8_t boardId, uint8_t item, uint16_t arg ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES )                 return( ERR_INVALID_BOARD_ID );
    if ( ! drvMap.map[ boardId ].flags & BF_EXT_BOARD_PRESENT ) return( ERR_INVALID_BOARD_ID );

     if ( isInRangeU( item, IR_ATTR_MEM_RANGE_START, IR_ATTR_MEM_RANGE_END )) {

        drvMap.map[ boardId ].extBoard.driverData[ item - IR_ATTR_MEM_RANGE_START ] = arg;
        return( ALL_OK );
    }
    else if ( isInRangeU( item, IR_ATTR_NVM_RANGE_START, IR_ATTR_NVM_RANGE_END )) {

        uint32_t ofs =  offsetof( LcsDrvBoardDesc, driverData ) + 
                        (( item - IR_ATTR_NVM_RANGE_START ) * sizeof( uint16_t ));

       if ( extNvmPutWord( boardId, ofs, arg ) != ALL_OK ) return( ERR_DRV_PUT_ERR );
       return( ALL_OK );
    }
    else if ( item == ITEM_ID_BOARD_VERSION ) {

        uint8_t rStat = ALL_OK;

        rStat = extNvmPutWord( boardId, offsetof( LcsDrvBoardDesc, head.boardVersion ), arg );
        if ( rStat == ALL_OK ) drvMap.map[ boardId ].extBoard.head.boardVersion = arg;
        return( rStat );
    }
    else if ( item == ITEM_ID_TYPE ) {

        uint8_t rStat = ALL_OK;

        rStat = extNvmPutWord( boardId, offsetof( LcsDrvBoardDesc, head.boardType ), arg );
        if ( rStat == ALL_OK ) drvMap.map[ boardId ].extBoard.head.boardType = arg;
        return( rStat );
    }
    else return( ERR_INVALID_ITEM_ID ); 
}

//------------------------------------------------------------------------------------------------------------
// "drvReq" is the entry point to an extension board. For each extension board type there is driver function.
// This function is called when we access that extension board. Note that the REQ call will only work when
// there is a board with a driver associated. There is however the case that the header area is a new area
// or an invalid area. We have a board detected bit could not setup the driver for it. The "ITEM_ID_FORMAT" 
// item is used to setup the extension board NVM. It works without checking for a valid driver.
// 
// The PUT and GET routines can be called for a board that has no driver associated yet. This way, for 
// example, the driver type and other initial data can be set. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t drvReq( uint8_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    if ( boardId >= MAX_EXT_BOARD_MAP_ENTRIES )                 return ( ERR_INVALID_BOARD_ID );
    if ( ! drvMap.map[ boardId ].flags & BF_EXT_BOARD_PRESENT ) return( ERR_INVALID_BOARD_ID );

    if ( item == ITEM_ID_FORMAT ) {

            uint8_t rStat = ALL_OK;

            return( buildDrvBoardDescArea( boardId ));
    }
    else {

        if ( drvMap.map[ boardId ].drvFunc != nullptr ) {

            return( drvMap.map[ boardId ].drvFunc( boardId - 1, item, arg1, arg2 ));
        }
        else return( ERR_EXT_BOARD_NOT_VALID );
    }
}

} // namespace LCS

