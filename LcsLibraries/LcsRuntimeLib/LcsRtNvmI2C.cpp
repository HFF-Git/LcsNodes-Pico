//------------------------------------------------------------------------------------------------------------
//
// LCS Runtime library - Non volatile storage based on the M24LCxxx chip family
//
//------------------------------------------------------------------------------------------------------------
// This file implements the LCS runtime library non-volatile memory. The hardware is the AA24xxx chip family,
// which offers an I2C protocol based chip with various capacities. They all share the same pin layout and
// command structure.
//
// In addition we also support the M24C04 chip, which is used on the extension boards as a configuration
// storage. This chip will however be replaced by 24AA32, a 4K chip of the same chip family as the other
// chips on the controller board.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS Core library - Non volatile storage based on the M24LCxxx chip family
// Copyright (C) 2021 - 2024  Helmut Fieres
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

//------------------------------------------------------------------------------------------------------------
// Include files.
//
//------------------------------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//------------------------------------------------------------------------------------------------------------
// Externals.
// 
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t debugMask;
};

//------------------------------------------------------------------------------------------------------------
// Local file declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Definitions for the M24LCxxx chips page size and total size. The chips have a pageSize which is the unit
// updated in case of a write. A write cannot across a page boundary and must be split into several writes
// if necessary. Reads do not have this problem. Al chips have the same I2C address root which is "1010".
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
//------------------------------------------------------------------------------------------------------------
const uint16_t  BUFFER_BLOCK_SIZE           = 32;

const uint16_t  M24LC32_PAGE_SIZE           = 32;
const uint32_t  M24LC32_MAX_SIZE            = 4096;

const uint16_t  M24LC64_PAGE_SIZE           = 32;
const uint32_t  M24LC64_MAX_SIZE            = 8192;

const uint16_t  M24LC128_PAGE_SIZE          = 64;
const uint32_t  M24LC128_MAX_SIZE           = 16384;

const uint16_t  M24LC256_PAGE_SIZE          = 64;
const uint32_t  M24LC256_MAX_SIZE           = 32768;

const uint16_t  M24LC512_PAGE_SIZE          = 128;
const uint32_t  M24LC512_MAX_SIZE           = 65536;

const uint16_t  M24C04_PAGE_SIZE            = 8;
const uint32_t  M24C04_MAX_SIZE             = 512;

const uint8_t   NVM_I2C_ADR_ROOT            = 0b1010000;
const uint8_t   EXT_I2C_ADR_ROOT            = 0b1010000;

//------------------------------------------------------------------------------------------------------------
// Runtime NVM sizes. The runtime map has a maximum of 8Kb. The maximum size of a NVM chip is 64Kb.  The 
// maximum size for an extension board NVM chip is 4Kb. 
//
//------------------------------------------------------------------------------------------------------------
const uint32_t  NVM_RUNTIME_MAP_SIZE        = 0x2000;
const uint32_t  NVM_MAX_NVM_SIZE            = 0x10000;
const uint32_t  NVM_MAX_EXT_SIZE            = 0x1000;

//------------------------------------------------------------------------------------------------------------
// Module global data. A LCS node board has two NVM channels. The "NVM" channel refers to the NVM chip on
// main controller board. The "EXT" channel is the I2C bus that reaches out the the extension boards. On
// each extension board there is again a small NVM chip with configuration data. Besides the hardware pins
// there are the sizes of the chips. 
//
// There is no easy way to determine the size of the actual chip. By convention, the extension board NVM 
// chip has a fixed 4 Kbytes. The NVM chip on the main controller board is at least 8Kbytes, which is the 
// architected runtime data structures size. The maximum size is 64 Kbytes. All the chips are from a hardware
// perspective identical. When we start a node, the nodeMap structure, i.e. the first few hundred bytes, 
// contains a field that holds the actual size configured for the chip on the particular board. The difference
// between the runtime map size and the particular NVM chip maximum is considered "user NVM space" which the
// firmware can use as needed.
//
//------------------------------------------------------------------------------------------------------------
uint32_t    nodeNvmSize                     = 0;
uint32_t    extNvmSize                      = 0;

uint8_t     nvmSclPin                       = CDC::UNDEFINED_PIN;
uint8_t     nvmSdaPin                       = CDC::UNDEFINED_PIN;

uint8_t     extSclPin                       = CDC::UNDEFINED_PIN;
uint8_t     extSdaPin                       = CDC::UNDEFINED_PIN;

//------------------------------------------------------------------------------------------------------------
// A little helper function to test whether the chip is read for the next operation. The test consist of 
// writing to the chip and see if this works. There is the case that it just takes a little time or there
// is just no chip at that address. We will use a retry count so we will not try forever.
//
//------------------------------------------------------------------------------------------------------------
uint8_t chipReady( uint8_t sclPin, uint8_t i2cAdr, uint16_t retryCnt = 100 ) {

    uint8_t ret = 1;
    uint8_t tmp = 0;

    while ( ret != ALL_OK ) {

        ret = CDC::i2cWrite( sclPin, i2cAdr, &tmp, 1 );
        
        retryCnt --;
        if ( retryCnt == 0 ) break;
    }

    return ( ret );
}

//------------------------------------------------------------------------------------------------------------
// Each NVM chip has certain size. This function will round the size to the next lower chip memory size.
// We expect however that the programmer used the correct size, so this is done just in case. The lowest
// value is the 4Kb chip.
//
//------------------------------------------------------------------------------------------------------------
uint32_t roundNvmMaxSize( uint16_t chipSize ) {

    if      ( chipSize <= M24C04_MAX_SIZE )   return ( M24C04_MAX_SIZE );
    else if ( chipSize <= M24LC32_MAX_SIZE )  return ( M24LC32_MAX_SIZE );
    else if ( chipSize <= M24LC64_MAX_SIZE )  return ( M24LC64_MAX_SIZE );
    else if ( chipSize <= M24LC128_MAX_SIZE ) return ( M24LC128_MAX_SIZE );
    else if ( chipSize <= M24LC256_MAX_SIZE ) return ( M24LC256_MAX_SIZE );
    else if ( chipSize <= M24LC512_MAX_SIZE ) return ( M24LC512_MAX_SIZE );
    else                                      return ( M24LC32_MAX_SIZE );
}

//------------------------------------------------------------------------------------------------------------
// The nvmSize in buffer size blocks.
//
//------------------------------------------------------------------------------------------------------------
uint16_t nvmSizeInBlocks( uint32_t nvmSize ) {

    return( nvmSize / BUFFER_BLOCK_SIZE );
}

//------------------------------------------------------------------------------------------------------------
// "nvmGetBytesFromPage" transmits a set of data bytes only within the page boundary. Although a read can 
// cross a page boundary, we follow the same principle as we do for writes when it comes to page boundaries.
// The read is send ing the address with retaining the bus. The PICO library will then use the restart
// condition. Just like we did in the write buffer counterpart, we need to send the address as one buffer.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmGetBytesFromPage( uint8_t sclPin, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmGetBytesFromPage: sclPin: %d, i2cAdr: 0x%x, ofs: 0x%x, buf: %p, len: %d\n", 
                sclPin, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( sclPin == nvmSclPin ) ? nodeNvmSize : extNvmSize );

    if ( nvmSize == M24C04_MAX_SIZE ) {

        uint8_t tmpAdr  = i2cAdr | (( ofs >> 8 ) & 0x01 );
        uint8_t tmpData = ofs & 0xFF;

        rStat = chipReady( sclPin, i2cAdr );
        if ( rStat == ALL_OK ) rStat = CDC::i2cWrite( sclPin, tmpAdr, &tmpData, sizeof( tmpData ), true );
        if ( rStat == ALL_OK ) rStat = CDC::i2cRead( sclPin, tmpAdr, buf, len );
    }
    else {

        uint8_t adr[ 2 ];

        adr[ 0 ] =  ( ofs  >> 8 ) & 0xFF;
        adr[ 1 ] =  ofs & 0xFF;

        rStat = chipReady( sclPin, i2cAdr );
        if ( rStat == ALL_OK ) rStat = CDC::i2cWrite( sclPin, i2cAdr, adr, 2, true );
        if ( rStat == ALL_OK ) rStat = CDC::i2cRead( sclPin, i2cAdr, buf, len );
    }

   if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmGetBytesFromPage: %d\n", rStat );
    }

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "nvmPutBytesInPage" transmits a set of data bytes only within the page boundary. In general, a write 
// cannot cross a chip internal page boundary. The Chip expects a write to be one sequence with the address
// bytes first followed by the data bytes with no stop or restart condition in between. This costed my
// quite some debugging to figure this out. We will have a local buffer where we combine the address and
// data and then send it.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmPutBytesInPage( uint8_t sclPin, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;
    uint8_t dataBuf[ BUFFER_BLOCK_SIZE + 2 ];

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmPutBytesInPage: sclPin: %d, i2cAdr: 0x%x, ofs: 0x%x, bufAdr: %p, len: %d\n", 
        sclPin, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( sclPin == nvmSclPin ) ? nodeNvmSize : extNvmSize );

    if ( nvmSize == M24C04_MAX_SIZE ) {

        uint8_t tmpAdr = i2cAdr | (( ofs >> 8 ) & 0x01 );
        uint8_t tmpOfs = ( ofs ) & 0xFF;

        rStat = chipReady( sclPin, tmpAdr );
        if ( rStat == ALL_OK ) CDC::i2cWrite( sclPin, tmpAdr, &tmpOfs, 1, true );
        if ( rStat == ALL_OK ) CDC::i2cWrite( sclPin, tmpAdr, buf, len );
    }
    else {

        dataBuf[ 0 ]    = ( ofs  >> 8 ) & 0xFF;
        dataBuf[ 1 ]    = ofs & 0xFF;

        for ( int i = 0; i < len; i++ ) dataBuf[ i + 2 ] = buf[ i ];

        rStat = chipReady( sclPin, i2cAdr );
        if ( rStat == ALL_OK ) rStat = CDC::i2cWrite( sclPin, i2cAdr, dataBuf, len + 2 );
    }

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmPutBytesInPage: ret: %d\n", rStat );
    }

    return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "nvmGetBytes" reads a set of data bytes from the memory. Although read operations do not have a page
// boundary issue, we stick to the concept to read within page boundaries as we may one day use more than
// chip to build NVMs and then we have no problems with crossing chip boundaries.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmGetBytes( uint8_t sclPin, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmGetBytes: scl: %d, i2c: 0x%x, ofs: 0x%x, bufAdr: %p, len: %d\n", 
                sclPin, i2cAdr, ofs, (uint32_t) buf, len );
    }

    uint32_t nvmSize = (( sclPin == nvmSclPin ) ? nodeNvmSize : extNvmSize );
    if ( ofs + len > nvmSize ) return ( ERR_NVM_SIZE_EXCEEDED );

    uint32_t  bytesLeft     = len;
    uint32_t  pageBytesLeft = BUFFER_BLOCK_SIZE - ofs % BUFFER_BLOCK_SIZE;

    while ( bytesLeft > pageBytesLeft ) {

        rStat = nvmGetBytesFromPage( sclPin, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, pageBytesLeft );
        if ( rStat != ALL_OK ) break;

        bytesLeft       -= pageBytesLeft;
        pageBytesLeft   = BUFFER_BLOCK_SIZE;
    }

    if ( rStat == ALL_OK ) {

        return ( nvmGetBytesFromPage( sclPin, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, bytesLeft ));
    }
    else return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "nvmPutBytes" transmits a set of data bytes to the memory. We cannot write across the internal NVM page
// boundary and also across a chip boundary. This routine will split the data to write only within one page
// in a given write cycle.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmPutBytes( uint8_t sclPin, uint8_t i2cAdr, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t rStat = ALL_OK;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmPutBytes: scl: %d, i2c: 0x%x, ofs: 0x%x, buf: %p, len: %d\n", sclPin, i2cAdr, ofs, buf, len );
     }

    uint32_t nvmSize = (( sclPin == nvmSclPin ) ? nodeNvmSize : extNvmSize );
    if ( ofs + len > nvmSize ) return ( ERR_NVM_SIZE_EXCEEDED );

    uint32_t  bytesLeft     = len;
    uint32_t  pageBytesLeft = BUFFER_BLOCK_SIZE - ofs % BUFFER_BLOCK_SIZE;

    while ( bytesLeft > pageBytesLeft ) {

        rStat = nvmPutBytesInPage( sclPin, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, pageBytesLeft );
        if ( rStat != ALL_OK ) break;

        bytesLeft       -= pageBytesLeft;
        pageBytesLeft   = BUFFER_BLOCK_SIZE;
    }

    if ( rStat == ALL_OK ) {

        return ( nvmPutBytesInPage( sclPin, i2cAdr, ofs + len - bytesLeft, buf + len - bytesLeft, bytesLeft ));
    }
    else return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// "nvmClearArea" wipes out an area of the NVM chip. To speed up the writing, we fill a local buffer with 
// the value and then write blocks at a time.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmClearArea( uint8_t sclPin, uint8_t i2cAdr, uint32_t ofs, uint32_t len, uint8_t val ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_NVM_ACCESS )) {

        printf( "nvmClearArea: scl: %d, i2c: 0x%x, ofs: 0x%x, len: %d, val: %d\n", sclPin, i2cAdr, ofs, len, val );
    }

    uint8_t     tmpBuf[ BUFFER_BLOCK_SIZE ];
    uint8_t     rStat   = ALL_OK;
    uint32_t    nvmSize = (( sclPin == nvmSclPin ) ? nodeNvmSize : extNvmSize );
    uint32_t    limit   = ofs + len;

    if ( ofs + len > nvmSize ) return ( ERR_NVM_SIZE_EXCEEDED );

    for ( int i = 0; i < BUFFER_BLOCK_SIZE; i ++ ) tmpBuf[ i ] = val;

    while ( len > BUFFER_BLOCK_SIZE ) {

        rStat = nvmPutBytes( sclPin, i2cAdr, ofs, tmpBuf, sizeof( tmpBuf ));
        if ( rStat != ALL_OK ) break;
        
        ofs += sizeof( val );
        len -= BUFFER_BLOCK_SIZE;
    }

    if ( rStat == ALL_OK ) {

        return( nvmPutBytes( sclPin, i2cAdr, ofs, tmpBuf, len ));
    }
    else return( rStat );
}

}; // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// "configNvm" will setup the module local variables. We copy the I2C hardware pins and the NVM related data
// from the CDC descriptors. The CDC descriptor also contains the configured sizes for the NVM chips. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t configNvm( CDC::CdcPinConfig *ci ) {

    nvmSclPin     = ci -> NVM_I2C_SCL_PIN;
    nvmSdaPin     = ci -> NVM_I2C_SDA_PIN;
    extSclPin     = ci -> EXT_I2C_SCL_PIN;
    extSdaPin     = ci -> EXT_I2C_SDA_PIN;
    nodeNvmSize   = ci -> NODE_NVM_SIZE;
    extNvmSize    = ci -> EXT_NVM_SIZE;

    if ( nodeNvmSize > NVM_MAX_NVM_SIZE )   nodeNvmSize = NVM_MAX_NVM_SIZE;
    if ( extNvmSize > NVM_MAX_EXT_SIZE )    extNvmSize  = NVM_MAX_EXT_SIZE;

    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// Controller Board Runtime Map access routines. The runtime map occupies the first 8 Kbytes of the main 
// controller NVM chip. There are routines for getting and setting a word as well as routines to read and 
// write a buffer. All access routines are prefixed with "rt".
//
//------------------------------------------------------------------------------------------------------------
uint8_t rtNvmPutWord( uint32_t ofs, uint16_t word ) {

    return( nvmPutBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) &word, sizeof( uint16_t )));
}

uint8_t rtNvmGetWord( uint32_t ofs, uint16_t *word ) {

    return( nvmGetBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) word, sizeof( uint16_t )));
}

uint8_t rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    return( nvmPutBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t rtNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    return( nvmGetBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t rtNvmClearArea( uint32_t ofs, uint32_t len, uint8_t val ) {

    return( nvmClearArea( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, len, val ));
}

uint32_t rtNvmGetSize( ) { 

    return( NVM_RUNTIME_MAP_SIZE );
}

//------------------------------------------------------------------------------------------------------------
// Extension Board Map access routines. These routines access the NVM on the extension board. The I2C address
// is formed by the chip common I2C address plus the address bits of the chip to select the chip on the 
// particular extension board. Similar to the runtime NVM access routines, there are routines for getting 
// and setting a word as well as routines to read and  write a buffer. All access routines are prefixed with
// "ext".
//
//------------------------------------------------------------------------------------------------------------
uint8_t extNvmPutWord( uint8_t boardId, uint32_t ofs, uint16_t word ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + ( boardId % MAX_EXT_BOARD_MAP_ENTRIES );
    return( nvmPutBytes( extSclPin, i2cAdr, ofs, (uint8_t *) &word, sizeof( uint16_t )));
}

uint8_t extNvmGetWord( uint8_t boardId, uint32_t ofs, uint16_t *word ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + ( boardId % MAX_EXT_BOARD_MAP_ENTRIES );
    return( nvmGetBytes( extSclPin, i2cAdr, ofs, (uint8_t *) word, sizeof( uint16_t )));
}

uint8_t extNvmPutBytes( uint8_t boardId, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + ( boardId % MAX_EXT_BOARD_MAP_ENTRIES );
    return( nvmPutBytes( extSclPin, i2cAdr, ofs, buf, len ));
}

uint8_t extNvmGetBytes( uint8_t boardId, uint32_t ofs, uint8_t *buf, uint32_t len ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + ( boardId % MAX_EXT_BOARD_MAP_ENTRIES );
    return( nvmGetBytes( extSclPin, i2cAdr, ofs, buf, len ));
}

uint8_t extNvmClearArea( uint8_t boardId, uint32_t ofs, uint32_t len, uint8_t val ) {

    uint8_t i2cAdr = EXT_I2C_ADR_ROOT + ( boardId % MAX_EXT_BOARD_MAP_ENTRIES );
    return( nvmClearArea( extSclPin, i2cAdr, ofs, len, val ));
}

uint32_t extNvmGetSize( ) {

    return( extNvmSize );
}

//------------------------------------------------------------------------------------------------------------
// Controller Board User Map access routines. The area between the main controller NVM chip runtime area and
// the chips hardware maximum size is the memory area available for the firmware programmer. Again, there 
// are routines for getting and setting a word as well as routines to read and  write a buffer. All access 
// routines are prefixed with "nvm".
//
//------------------------------------------------------------------------------------------------------------
uint8_t usrNvmPutWord( uint32_t ofs, uint16_t word ) {

    ofs = ofs + NVM_USER_MAP_START;
    return( nvmPutBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) &word, sizeof( uint16_t )));
}

uint8_t usrNvmGetWord( uint32_t ofs, uint16_t *word ) {

    ofs = ofs + NVM_USER_MAP_START;
    return( nvmGetBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, (uint8_t *) word, sizeof( uint16_t )));
}

uint8_t usrNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    ofs = ofs + NVM_USER_MAP_START;
    return( nvmPutBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t usrNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    ofs = ofs + NVM_USER_MAP_START;
    return( nvmGetBytes( nvmSclPin, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint32_t usrNvmGetSize( ) {

    return ( nodeNvmSize - NVM_USER_MAP_START );
}

}; // namespace LCS
