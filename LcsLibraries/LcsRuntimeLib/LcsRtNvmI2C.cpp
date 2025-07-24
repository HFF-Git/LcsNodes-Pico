//----------------------------------------------------------------------------------------
//
// LCS Runtime library - Non volatile storage I2C interface
//
//----------------------------------------------------------------------------------------
// This file implements the LCS runtime library non-volatile memory. The hardware is the AA24xxx chip family,
// which offers an I2C protocol based chip with various capacities. They all share the same pin layout and
// command structure.
//
// In addition we also support the M24C04 chip, which is used on the extension boards as a configuration
// storage. This chip will however be replaced by 24AA32, a 4K chip of the same chip family as the other
// chips on the controller board.
//
//----------------------------------------------------------------------------------------
//
// LCS Runtime library - Non volatile storage I2C interface
// Copyright (C) 2022 - 2025 Helmut Fieres
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

//----------------------------------------------------------------------------------------
// Include files.
//
//----------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//----------------------------------------------------------------------------------------
// Externals.
// 
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t     debugMask;
    extern LcsNodeMap   nodeMap;
};

//----------------------------------------------------------------------------------------
// Local file declarations.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Definitions for the M24LCxxx chips page size and total size. The chips have a pageSize which is the unit
// updated in case of a write. A write cannot across a page boundary and must be split into several writes
// if necessary. Reads do not have this problem. All chips have the same I2C address root which is "1010".
// There are three address lines A2, A1 and A0, which are used to select a chips. Up to eight chips can be
// addressed on a single I2C bus.
//
// The pageSizes on the chip are a multiple of 32bytes. For now, we use this size as the common denominator.
// Block handling and chipSize page handling are nicely taken care of this way. The downside is however that
// a write will update the chip page up to four times for a pageSize of 128. However, since the chips have
// more than a million write cycles and we rarely write large chunks of data, this will hopefully not be an
// issue in the near future.
//
// ??? the M24C04 is to be phased out ... we do not use that chip anymore...
//----------------------------------------------------------------------------------------
const uint16_t      BUFFER_BLOCK_SIZE           = 16; // ??? until we remove the M24C04 chip, then 32...

const uint16_t      M24LC32_PAGE_SIZE           = 32;
const uint32_t      M24LC32_MAX_SIZE            = 4096;

const uint16_t      M24LC64_PAGE_SIZE           = 32;
const uint32_t      M24LC64_MAX_SIZE            = 8192;

const uint16_t      M24LC128_PAGE_SIZE          = 64;
const uint32_t      M24LC128_MAX_SIZE           = 16384;

const uint16_t      M24LC256_PAGE_SIZE          = 64;
const uint32_t      M24LC256_MAX_SIZE           = 32768;

const uint16_t      M24LC512_PAGE_SIZE          = 128;
const uint32_t      M24LC512_MAX_SIZE           = 65536;

const uint16_t      M24C04_PAGE_SIZE            = 16;
const uint32_t      M24C04_MAX_SIZE             = 512;

const uint8_t       NVM_I2C_ADR_ROOT            = 0b1010000;
const uint32_t      NVM_I2C_BAUDRATE            = 100 * 1000;

const uint8_t       EXT_I2C_ADR_ROOT            = 0b1010000;
const uint32_t      EXT_I2C_BAUDRATE            = 50 * 1000;

const uint8_t       NVM_WRITE_DELAY             = 0x05;

//----------------------------------------------------------------------------------------
// Runtime NVM sizes. The maximum size of a NVM chip is 64Kb. The maximum size for an extension board NVM 
// chip is 4Kb. 
//
//----------------------------------------------------------------------------------------
const uint32_t      NVM_MAX_NVM_SIZE            = 0x10000;
const uint32_t      NVM_MAX_EXT_SIZE            = 0x1000;

//----------------------------------------------------------------------------------------
// Module global data. A LCS node board has two NVM channels. The "NVM" channel refers to the NVM chip on
// main controller board. The "EXT" channel is the I2C bus that reaches out the the extension boards. On
// each extension board there is again a small NVM chip with configuration data. 
//
// There is no easy way to determine the size of the actual chip. By convention, the extension board NVM chip
// has a fixed 4 Kbytes. The NVM chip on the main controller board is at least 16Kbyte. The maximum size is 
// 64 Kbytes. All the chips are from a hardware perspective identical. The difference between the runtime map
// size and the particular NVM chip maximum is considered "user NVM space" which the firmware can use as 
// needed.
//
//----------------------------------------------------------------------------------------
uint32_t    nodeNvmSize     = 0;
uint32_t    extNvmSize      = 0;

uint8_t     rNumNvm         = UNDEFINED_RES_ID;
uint8_t     rNumExtNvm      = UNDEFINED_RES_ID;

//----------------------------------------------------------------------------------------
// A little helper function to report any errors.
//
//----------------------------------------------------------------------------------------
uint8_t errStat( uint8_t errId ) {

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_NVM_ACCESS )) printf( "Ret: %d\n", errId );
    return ( errId );
}

//----------------------------------------------------------------------------------------
// "testNvmChipMemorySize" will check the NVM chip for its size. Since the chip itself has no way of telling
// its memory capacity, we need to go a rather cumbersome way. For each possible size, read the last byte,
// store a new value there, read it again. If the values match, it is a valid memory location. Don't forget
// to restore the previous value. If we are not successful, try the next smaller size. Currently, the LCS 
// hardware uses The chip family M24LCxxx with sizes of 4, 8, 16, 32 and 64Kbytes.
//
// ??? not tested yet ...
// ??? will not work for the smaller then 4K chips. Wait until we only have 4K and higher.
//----------------------------------------------------------------------------------------
uint32_t testNvmChipMemorySize( uint8_t rNum, uint8_t i2cAdr ) {

    uint32_t    nvmSize         = M24LC512_MAX_SIZE;
    uint32_t    testAdr         = nvmSize - 1;
    uint8_t     originalValue   = 0;
    uint8_t     testValue       = 0xab;
    uint8_t     tmpValue        = 0;
    uint8_t     tmpBuf[ 3 ]     = { 0 };
    uint8_t     rStat           = ALL_OK;
    
    while ( nvmSize >= M24LC32_MAX_SIZE ) {

        tmpBuf[ 0 ] = testAdr >> 8 & 0xFF;
        tmpBuf[ 1 ] = testAdr &0xFF;
        
        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 2, true );
        if ( rStat == ALL_OK ) rStat = CDC::i2cRead( rNum, i2cAdr, &originalValue, 1 );
        if ( rStat != ALL_OK ) return ( ERR_NVM_CHIP_SIZE_DETECT );

        tmpBuf[ 2 ] = testValue;
        
        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, sizeof( tmpBuf ));
        if ( rStat == ALL_OK ) {

            sleepMillis( NVM_WRITE_DELAY );

            rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 2, true );
            if ( rStat == ALL_OK ) rStat = i2cRead( rNum, i2cAdr, &tmpValue, 1 );
            if ( rStat == ALL_OK ) {

                if ( tmpValue == testValue ) {

                    rStat = i2cWrite( rNum, i2cAdr, tmpBuf, sizeof( tmpBuf ));
                    sleepMillis( NVM_WRITE_DELAY );
                    return ( nvmSize );
                }
                else {
                    
                    nvmSize = nvmSize / 2;
                    testAdr = nvmSize - 1;
                }
            }
        }   
        else return ( ERR_NVM_CHIP_SIZE_DETECT ); 
    }

    return ( nvmSize );
}

//----------------------------------------------------------------------------------------
// Each NVM chip has certain size. This function will round the size to the next lower chip memory size.
// We expect however that the programmer used the correct size, so this is done just in case. The lowest
// value is the 4Kb chip.
//
//----------------------------------------------------------------------------------------
uint32_t roundNvmMaxSize( uint16_t chipSize ) {

    if      ( chipSize <= M24C04_MAX_SIZE )   return ( M24C04_MAX_SIZE );
    else if ( chipSize <= M24LC32_MAX_SIZE )  return ( M24LC32_MAX_SIZE );
    else if ( chipSize <= M24LC64_MAX_SIZE )  return ( M24LC64_MAX_SIZE );
    else if ( chipSize <= M24LC128_MAX_SIZE ) return ( M24LC128_MAX_SIZE );
    else if ( chipSize <= M24LC256_MAX_SIZE ) return ( M24LC256_MAX_SIZE );
    else if ( chipSize <= M24LC512_MAX_SIZE ) return ( M24LC512_MAX_SIZE );
    else                                      return ( M24LC32_MAX_SIZE );
}

//----------------------------------------------------------------------------------------
// "nvmGetBytesFromPage" transmits a set of data bytes only within the page boundary. Although a read can 
// cross a page boundary, we follow the same principle as we do for writes when it comes to page boundaries.
// The read is send ing the address with retaining the bus. The PICO library will then use the restart
// condition. Just like we did in the write buffer counterpart, we need to send the address as one buffer.
//
// ??? one day we take out the M24C04
//----------------------------------------------------------------------------------------
uint8_t nvmGetBytesFromPage( uint8_t rNum, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_NVM_ACCESS )) {

        printf( "nvmGetBytesFromPage: rNum: %d, i2cAdr: 0x%x, ofs: 0x%x, buf: %p, len: %d\n", 
                rNum, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );

    if ( nvmSize == M24C04_MAX_SIZE ) {

        uint8_t tmpAdr  = i2cAdr | (( ofs >> 8 ) & 0x01 );
        uint8_t tmpData = ofs & 0xFF;

        rStat = i2cWrite( rNum, tmpAdr, &tmpData, sizeof( tmpData ), true );
        if ( rStat == ALL_OK ) rStat = i2cRead( rNum, tmpAdr, buf, len );
    }
    else {

        uint8_t adr[ 2 ];

        adr[ 0 ] =  ( ofs  >> 8 ) & 0xFF;
        adr[ 1 ] =  ofs & 0xFF;

        rStat = i2cWrite( rNum, i2cAdr, adr, 2, true );
        if ( rStat == ALL_OK ) rStat = i2cRead( rNum, i2cAdr, buf, len );
    }

    return ( errStat( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmPutBytesInPage" transmits a set of data bytes only within the page boundary. In general, a write 
// cannot cross a chip internal page boundary. The Chip expects a write to be one sequence with the address
// bytes first followed by the data bytes with no stop or restart condition in between. This costed my
// quite some debugging to figure this out. We will have a local buffer where we combine the address and
// data and then send it.
//
// ??? one day we take out the M24C04
//----------------------------------------------------------------------------------------
uint8_t nvmPutBytesInPage( uint8_t rNum, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;
    uint8_t dataBuf[ BUFFER_BLOCK_SIZE + 2 ];

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_NVM_ACCESS )) {

        printf( "nvmPutBytesInPage: rNum: %d, i2cAdr: 0x%x, ofs: 0x%x, bufAdr: %p, len: %d\n", 
        rNum, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );

    if ( nvmSize == M24C04_MAX_SIZE ) {

        dataBuf[ 0 ] = ofs & 0xFF;
        for ( int i = 0; i < len; i++ ) dataBuf[ i + 1 ] = buf[ i ];

        uint8_t tmpAdr = i2cAdr | (( ofs >> 8 ) & 0x01 );
        rStat = i2cWrite( rNum, tmpAdr, dataBuf, len + 1 );
    }
    else {

        dataBuf[ 0 ]    = ( ofs  >> 8 ) & 0xFF;
        dataBuf[ 1 ]    = ofs & 0xFF;

        for ( int i = 0; i < len; i++ ) dataBuf[ i + 2 ] = buf[ i ];

        rStat = i2cWrite( rNum, i2cAdr, dataBuf, len + 2 );
    }

    return ( errStat( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmGetBytes" reads a set of data bytes from the memory. Although read operations do not have a page
// boundary issue, we stick to the concept to read within page boundaries as we may one day use more than
// chip to build NVMs and then we have no problems with crossing chip boundaries.
//
//----------------------------------------------------------------------------------------
uint8_t nvmGetBytes( uint8_t rNum, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_NVM_ACCESS )) {

        printf( "nvmGetBytes: rNum: %d, i2c: 0x%x, ofs: 0x%x, bufAdr: %p, len: %d\n", 
                rNum, i2cAdr, ofs, (uint32_t) buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );
    if ( ofs + len > nvmSize ) return ( errStat( ERR_NVM_SIZE_EXCEEDED ));

    uint32_t  bytesLeft     = len;
    uint32_t  pageBytesLeft = BUFFER_BLOCK_SIZE - ofs % BUFFER_BLOCK_SIZE;

    while ( bytesLeft > pageBytesLeft ) {

        rStat = nvmGetBytesFromPage( rNum, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, pageBytesLeft );
        if ( rStat != ALL_OK ) break;

        bytesLeft       -= pageBytesLeft;
        pageBytesLeft   = BUFFER_BLOCK_SIZE;
    }

    if (( rStat == ALL_OK ) && ( bytesLeft > 0 )) {

        rStat = nvmGetBytesFromPage( rNum, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, bytesLeft );
    }

    return ( errStat( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmPutBytes" transmits a set of data bytes to the memory. We cannot write across the internal NVM page
// boundary and also across a chip boundary. This routine will split the data to write only within one page
// in a given write cycle.
//
// There is a quirk with figuring out that a chip is ready for the next write instruction. The data sheet 
// suggest a writing of one byte to see of the chip acknowledges. If not it is still in a write operation. 
// This approach does not seem to work with the PICO i2c libraries. So, we will go the "slow" way of giving 
// the chip the time to complete the write cycle before issuing another one. Since we do not often write 
// to the NVM, the slow mode is perhaps acceptable for now.
//
//----------------------------------------------------------------------------------------
uint8_t nvmPutBytes( uint8_t rNum, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_NVM_ACCESS )) {

        printf( "nvmPutBytes: rNum: %d, i2c: 0x%x, ofs: 0x%x, buf: %p, len: %d\n", rNum, i2cAdr, ofs, buf, len );
     }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );
    if ( ofs + len > nvmSize ) return ( errStat( ERR_NVM_SIZE_EXCEEDED ));

    uint32_t  bytesLeft     = len;
    uint32_t  pageBytesLeft = BUFFER_BLOCK_SIZE - ofs % BUFFER_BLOCK_SIZE;

    while ( bytesLeft > pageBytesLeft ) {

        rStat = nvmPutBytesInPage( rNum, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, pageBytesLeft );
        if ( rStat != ALL_OK ) break;

        bytesLeft       -= pageBytesLeft;
        pageBytesLeft   = BUFFER_BLOCK_SIZE;

        CDC::sleepMillis( NVM_WRITE_DELAY );
    }

    if (( rStat == ALL_OK ) && ( bytesLeft > 0 )) {

       rStat = nvmPutBytesInPage( rNum, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, bytesLeft );
       CDC::sleepMillis( NVM_WRITE_DELAY );
    }

    return ( errStat( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmClearArea" wipes out an area of the NVM chip. To speed up the writing, we fill a local buffer with 
// the value and then write blocks at a time.
//
//----------------------------------------------------------------------------------------
uint8_t nvmClearArea( uint8_t rNum, uint8_t i2cAdr, uint32_t ofs, uint32_t len, uint8_t val ) {

    if (( debugMask & LCS_DBG_CONFIG ) && ( debugMask & LCS_DBG_NVM_ACCESS )) {

        printf( "nvmClearArea: rNum: %d, i2c: 0x%x, ofs: 0x%x, len: %d, val: %d\n", 
                rNum, i2cAdr, ofs, len, val );
    }

    uint8_t     tmpBuf[ BUFFER_BLOCK_SIZE ];
    uint8_t     rStat   = ALL_OK;
    uint32_t    nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );
    uint32_t    limit   = ofs + len;

    if ( ofs + len > nvmSize ) return ( errStat( ERR_NVM_SIZE_EXCEEDED ));

    for ( int i = 0; i < BUFFER_BLOCK_SIZE; i ++ ) tmpBuf[ i ] = val;

    while ( len > BUFFER_BLOCK_SIZE ) {

        rStat = nvmPutBytes( rNum, i2cAdr, ofs, tmpBuf, sizeof( tmpBuf ));
        if ( rStat != ALL_OK ) break;
        
        ofs += BUFFER_BLOCK_SIZE;
        len -= BUFFER_BLOCK_SIZE;
    }

    if (( rStat == ALL_OK ) && ( len > 0 )) {

        rStat = nvmPutBytes( rNum, i2cAdr, ofs, tmpBuf, len );
    }

    return ( errStat( rStat ));
}

}; // namespace


//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// "configNvm" will setup the module local variables. 
//
//----------------------------------------------------------------------------------------
uint8_t configNvm(  uint8_t     rIdNvm, 
                    uint32_t    nvmSize, 
                    uint8_t     rIdExtNvm,
                    uint32_t    extNvmSize ) {

    rNumNvm     = rIdNvm;
    rNumExtNvm  = rIdExtNvm;
    nodeNvmSize = nvmSize;
    extNvmSize  = extNvmSize;

    if ( nodeNvmSize > NVM_MAX_NVM_SIZE )   nodeNvmSize = NVM_MAX_NVM_SIZE;
    if ( extNvmSize > NVM_MAX_EXT_SIZE )    extNvmSize  = NVM_MAX_EXT_SIZE;

    return ( ALL_OK );
}

//----------------------------------------------------------------------------------------
// Controller Board Runtime Map access routines. The runtime map occupies the first 8 Kbytes of the main 
// controller NVM chip. There are routines for getting and setting a word as well as routines to read and 
// write a buffer. All access routines are prefixed with "rt".
//
//----------------------------------------------------------------------------------------
uint8_t rtNvmPutWord( uint32_t ofs, uint16_t word ) {

    return ( nvmPutBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) &word, sizeof( uint16_t )));
}

uint8_t rtNvmGetWord( uint32_t ofs, uint16_t *word ) {

    return ( nvmGetBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) word, sizeof( uint16_t )));
}

uint8_t rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    return ( nvmPutBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t rtNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    return ( nvmGetBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t rtNvmClearArea( uint32_t ofs, uint32_t len, uint8_t val ) {

    return ( nvmClearArea( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, len, val ));
}

uint32_t rtNvmGetSize( ) { 

    return ( nodeNvmSize );
}

//----------------------------------------------------------------------------------------
// Extension Board Map access routines. These routines access the NVM on the extension board. The I2C address
// is formed by the chip common I2C address plus the address bits of the chip to select the chip on the 
// particular extension board. Similar to the runtime NVM access routines, there are routines for getting 
// and setting a word as well as routines to read and  write a buffer. All access routines are prefixed with
// "ext".
//
//----------------------------------------------------------------------------------------
uint8_t extNvmPutWord( uint8_t boardId, uint32_t ofs, uint16_t word ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + (( boardId % MAX_EXT_BOARD_MAP_ENTRIES ) << 1 );
    return ( nvmPutBytes( rNumExtNvm, i2cAdr, ofs, (uint8_t *) &word, sizeof( uint16_t )));
}

uint8_t extNvmGetWord( uint8_t boardId, uint32_t ofs, uint16_t *word ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + (( boardId % MAX_EXT_BOARD_MAP_ENTRIES ) << 1 );
    return ( nvmGetBytes( rNumExtNvm, i2cAdr, ofs, (uint8_t *) word, sizeof( uint16_t )));
}

uint8_t extNvmPutBytes( uint8_t boardId, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + (( boardId % MAX_EXT_BOARD_MAP_ENTRIES ) << 1 );
    return ( nvmPutBytes( rNumExtNvm, i2cAdr, ofs, buf, len ));
}

uint8_t extNvmGetBytes( uint8_t boardId, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + (( boardId % MAX_EXT_BOARD_MAP_ENTRIES ) << 1 );
    return ( nvmGetBytes( rNumExtNvm, i2cAdr, ofs, buf, len ));
}

uint8_t extNvmClearArea( uint8_t boardId, uint32_t ofs, uint32_t len, uint8_t val ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + (( boardId % MAX_EXT_BOARD_MAP_ENTRIES ) << 1 );
    return ( nvmClearArea( rNumExtNvm, i2cAdr, ofs, len, val ));
}

uint32_t extNvmGetSize( ) {

    return ( extNvmSize );
}

//----------------------------------------------------------------------------------------
// Controller Board User Map access routines. The area between the main controller NVM chip runtime area and
// the chips hardware maximum size is the memory area available for the firmware programmer. Again, there 
// are routines for getting and setting a word as well as routines to read and  write a buffer. All access 
// routines are prefixed with "usr".
//
//----------------------------------------------------------------------------------------
uint8_t usrNvmPutWord( uint32_t ofs, uint16_t word ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmPutBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) &word, sizeof( uint16_t )));
}

uint8_t usrNvmGetWord( uint32_t ofs, uint16_t *word ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmGetBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) word, sizeof( uint16_t )));
}

uint8_t usrNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmPutBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t usrNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmGetBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint32_t usrNvmGetSize( ) {

    if (( nodeMap.nodeState != NS_OPERATE ) && ( nodeMap.nodeState != NS_CONFIG )) return ( ERR_LIB_NOT_READY );
    return ( nodeNvmSize - NVM_RUNTIME_MAPS_SIZE );
}

}; // namespace LCS
