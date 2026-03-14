//----------------------------------------------------------------------------------------
//
// LCS Runtime library - Non volatile storage I2C interface
//
//----------------------------------------------------------------------------------------
// This file implements the LCS runtime library non-volatile memory. The hardware 
// is the AA24xxx chip family, which offers an I2C protocol based chip with various 
// capacities. They all share the same pin layout and command structure.
//
// In addition we also support the M24C04 chip, which is used on the extension 
// boards as a configuration storage. This chip will however be replaced by 24AA32,
// a 4K chip of the same chip family as the other chips on the controller board.
//
//----------------------------------------------------------------------------------------
//
// LCS Runtime library - Non volatile storage I2C interface
// Copyright (C) 2020 - 2026 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------


// ??? this file will only support the runtime NVM, all else is handled by I2C channel
// stuff.... take out extension board stuff.
//
// ??? take out the LC04 chip type...

// ??? compute the available user map size during configuration...



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
// Definitions for the M24LCxxx chips page size and total size. The chips have a 
// pageSize which is the unit updated in case of a write. A write cannot across a
// page boundary and must be split into several writes if necessary. Reads do not 
// have this problem. All chips have the same I2C address root which is "1010". 
// There are three address lines A2, A1 and A0, which are used to select a chips.
// Up to eight chips can be addressed on a single I2C bus.
//
// The pageSizes on the chip are a multiple of 32bytes. For now, we use this size 
// as the common denominator. Block handling and chipSize page handling are nicely
// taken care of this way. The downside is however that a write will update the chip
// page up to four times for a pageSize of 128. However, since the chips have more
// than a million write cycles and we rarely write large chunks of data, this will
// hopefully not be an issue in the near future.
//
// ??? the M24C04 is to be phased out ... we do not use that chip anymore...
//----------------------------------------------------------------------------------------
const uint16_t      MAX_BUFFER_BLOCK_SIZE       = 128;
const uint16_t      DEF_BUFFER_BLOCK_SIZE       = 16;

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

const uint8_t       NVM_I2C_ADR_ROOT            = 0b1010000;
const uint32_t      NVM_I2C_BAUDRATE            = 100 * 1000;

const uint8_t       EXT_I2C_ADR_ROOT            = 0b1010000;
const uint32_t      EXT_I2C_BAUDRATE            = 50 * 1000;

const uint8_t       NVM_WRITE_DELAY             = 0x05;

//----------------------------------------------------------------------------------------
// Runtime NVM sizes. The maximum size of a NVM chip is 64Kb. The maximum size for
// an extension board NVM chip is 4Kb. 
//
//----------------------------------------------------------------------------------------
const uint32_t      NVM_MAX_NVM_SIZE            = 0x10000;
const uint32_t      NVM_MAX_EXT_SIZE            = 0x1000;

//----------------------------------------------------------------------------------------
// Module global data. A LCS node board has two NVM channels. The "NVM" channel 
// refers to the NVM chip on main controller board. The "EXT" channel is the I2C 
// bus that reaches out the the extension boards. On each extension board there is
// again a small NVM chip with configuration data. 
//
// There is no easy way to determine the size of the actual chip. By convention, the
// extension board NVM chip has a fixed 4 Kbytes. The NVM chip on the main controller 
// board is at least 16Kbyte. The maximum size is 64 Kbytes. All the chips are from
// a hardware perspective identical. The difference between the runtime map size and 
// the particular NVM chip maximum is considered "user NVM space" which the firmware
// can use as needed.
//
//----------------------------------------------------------------------------------------
uint32_t    nodeNvmSize         = 0;
uint32_t    nodeNvmBlockSize    = DEF_BUFFER_BLOCK_SIZE;

uint32_t    extNvmSize          = 0;
uint32_t    extNvmBlockSize     = DEF_BUFFER_BLOCK_SIZE;

uint8_t     rNumNvm             = UNDEFINED_RES_ID;
uint8_t     rNumExtNvm          = UNDEFINED_RES_ID;

//----------------------------------------------------------------------------------------
// "nvmDebugEnabled" and "retStat" are the debug support routines. We can easily 
// check whether debug is enabled at all. The return status routine will print 
// out a return status message when debugging is enabled. The macro "RET_STAT" 
// is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool nvmDebugEnabled( ) {

    return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_NVM_ACCESS )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( nvmDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// Each NVM chip has certain size. This function will round the size to the next 
// lower chip memory size. We expect however that the programmer used the correct 
// size, so this is done just in case. The lowest value is the 4Kb chip.
//
//----------------------------------------------------------------------------------------
uint32_t roundNvmMaxSize( uint16_t chipSize ) {

    if      ( chipSize <= M24LC32_MAX_SIZE )  return ( M24LC32_MAX_SIZE );
    else if ( chipSize <= M24LC64_MAX_SIZE )  return ( M24LC64_MAX_SIZE );
    else if ( chipSize <= M24LC128_MAX_SIZE ) return ( M24LC128_MAX_SIZE );
    else if ( chipSize <= M24LC256_MAX_SIZE ) return ( M24LC256_MAX_SIZE );
    else if ( chipSize <= M24LC512_MAX_SIZE ) return ( M24LC512_MAX_SIZE );
    else                                      return ( M24LC32_MAX_SIZE );
}

//----------------------------------------------------------------------------------------
// "determineNvmChipMemorySize" detects the size of an I2C NVM (M24LCxxx family).
// These chips have no internal register to report their capacity, so the only 
// way to determine the size is by probing memory locations and observing whether
// higher addresses are actually addressable or just mirrored (aliased) copies
// of lower ones.
//
// Algorithm:
//
//   1. Start with the largest supported size (e.g. 64 Kbytes for M24LC512).
//   2. For this assumed size "N":
//        - Let A  = N - 1  (the last byte of this address range)
//        - Let A2 = A - N/2 (the midpoint address)
//   3. Read and store the original values at A and A2.
//   4. Write a known test value (e.g. 0xAB) to address A.
//   5. Read back both A and A2.
//        • If *both* locations changed to the test value, the EEPROM has aliased
//          the top half of the address space onto the bottom half — the assumed
//          size N is too large.  Halve N and repeat the test.
//        • If only A changed (A2 kept its original content), then A and A2 are
//          distinct physical addresses, so N is the actual device size.
//   6. Restore all modified bytes to their original contents before returning.
//
// This approach works because all M24LCxx devices have power-of-two sizes and,
// when addressed beyond their actual capacity, their internal address counter
// simply wraps around, creating mirrored address regions.
//
// Notes:
//   • Two-byte addressing is used (valid for 24LC32 and larger).
//   • The function currently tests 32-, 64-, 128-, 256-, and 512-Kbit parts,
//     corresponding to 4, 8, 16, 32, and 64 Kbytes of memory.
//   • Devices smaller than 32-Kbit (24LC16/08/04) use 1-byte addressing and
//     separate I²C sub-addresses; support for those can be added separately.
//
//----------------------------------------------------------------------------------------
uint32_t determineNvmChipMemorySize( uint8_t rNum, uint8_t i2cAdr ) {

    uint32_t nvmSize    = M24LC512_MAX_SIZE;
    uint32_t testAdr    = nvmSize - 1;
    uint32_t mirrorAdr  = 0;
    uint8_t originalA   = 0x00;
    uint8_t originalA2  = 0x00;
    uint8_t testValue   = 0xAB;
    uint8_t readA       = 0;
    uint8_t readA2      = 0;
    uint8_t tmpBuf[3]   = {0};
    uint8_t rStat       = NO_ERR;

    while ( nvmSize >= M24LC32_MAX_SIZE ) {

        testAdr     = nvmSize - 1;
        mirrorAdr   = testAdr - ( nvmSize / 2 );

       if ( nvmDebugEnabled( )) {

            printf( "Testing Size=%lu, testAdr=%04lX, mirrorAdr=%04lX\n", 
                    (unsigned long) nvmSize,
                    (unsigned long) testAdr, 
                    (unsigned long) mirrorAdr );
        }

        // Read original at A
        tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF );
        tmpBuf[1] = (uint8_t)( testAdr & 0xFF );

        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 2, true );
        if ( rStat != NO_ERR ) return ( 0 );

        rStat = i2cRead(rNum, i2cAdr, &originalA, 1);
        if ( rStat != NO_ERR ) return ( 0 );

        // Read original at A2
        tmpBuf[0] = (uint8_t)(( mirrorAdr >> 8 ) & 0xFF);
        tmpBuf[1] = (uint8_t)( mirrorAdr & 0xFF );
        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 2, true );
        if ( rStat != NO_ERR ) return ( 0 );

        rStat = i2cRead(rNum, i2cAdr, &originalA2, 1);
        if ( rStat != NO_ERR ) return ( 0 );

        // Write testValue to A
        tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF);
        tmpBuf[1] = (uint8_t)( testAdr & 0xFF );
        tmpBuf[2] = testValue;

        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
        if (rStat != NO_ERR) return ( 0 );

        sleepMillis(NVM_WRITE_DELAY);

        // Read back A
        tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF );
        tmpBuf[1] = (uint8_t)( testAdr & 0xFF );

        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 2, true );
        if ( rStat != NO_ERR ) return ( 0 );

        rStat = i2cRead( rNum, i2cAdr, &readA, 1) ;
        if (rStat != NO_ERR) {

            // attempt to restore originals before returning error
            tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF);
            tmpBuf[1] = (uint8_t)( testAdr & 0xFF );
            tmpBuf[2] = originalA;
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
            sleepMillis(NVM_WRITE_DELAY);
            return ( 0 );
        }

        // Read back A2
        tmpBuf[0] = (uint8_t)(( mirrorAdr >> 8 ) & 0xFF);
        tmpBuf[1] = (uint8_t)( mirrorAdr & 0xFF );

        rStat = i2cWrite( rNum, i2cAdr, tmpBuf, 2, true );
        if ( rStat != NO_ERR ) return ( 0 );

        rStat = i2cRead( rNum, i2cAdr, &readA2, 1 );
        if (rStat != NO_ERR) {

            // restore A and A2 originals where possible, then return
            tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF );
            tmpBuf[1] = (uint8_t)( testAdr & 0xFF );
            tmpBuf[2] = originalA;
            i2cWrite(rNum, i2cAdr, tmpBuf, 3, false );

            sleepMillis(NVM_WRITE_DELAY);
            
            tmpBuf[0] = (uint8_t)(( mirrorAdr >> 8) & 0xFF );
            tmpBuf[1] = (uint8_t)( mirrorAdr & 0xFF );
            tmpBuf[2] = originalA2;
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
            
            sleepMillis( NVM_WRITE_DELAY );
            return ( 0 );
        }

        if ( nvmDebugEnabled( )) {

            printf( "Read back A=%02X A2=%02X\n", readA, readA2 );
        }

        if (( readA == testValue ) && ( readA2 == testValue )) {

            // aliasing detected -> N too big
            // restore originals and continue with next smaller size

            tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF );
            tmpBuf[1] = (uint8_t)( testAdr & 0xFF );
            tmpBuf[2] = originalA;
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false) ;
            sleepMillis(NVM_WRITE_DELAY);

            tmpBuf[0] = (uint8_t)(( mirrorAdr >> 8) & 0xFF );
            tmpBuf[1] = (uint8_t)( mirrorAdr & 0xFF );
            tmpBuf[2] = originalA2;
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
            sleepMillis(NVM_WRITE_DELAY);

            nvmSize /= 2;
            continue;

        } else if ( readA == testValue ) {

            // write stuck only at A -> addressable, so N is actual size
            // restore original at A and return N

            tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF );
            tmpBuf[1] = (uint8_t)( testAdr & 0xFF );
            tmpBuf[2] = originalA;
            
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
            sleepMillis( NVM_WRITE_DELAY );

            if ( nvmDebugEnabled( )) {

                printf( "Size computed: %d\n", nvmSize );
            }
            
            return ( nvmSize );

        } else {

            // write didn't stick at A (unexpected), treat as error or reduce size
            // restore originals and reduce size

            tmpBuf[0] = (uint8_t)(( testAdr >> 8 ) & 0xFF );
            tmpBuf[1] = (uint8_t)( testAdr & 0xFF );
            tmpBuf[2] = originalA;
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
            sleepMillis( NVM_WRITE_DELAY );

            tmpBuf[0] = (uint8_t)(( mirrorAdr >> 8 ) & 0xFF );
            tmpBuf[1] = (uint8_t)( mirrorAdr & 0xFF );
            tmpBuf[2] = originalA2;
            i2cWrite( rNum, i2cAdr, tmpBuf, 3, false );
            sleepMillis( NVM_WRITE_DELAY );

            nvmSize /= 2;
        }
    }

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_SETUP )) {

        printf( "testNvmChipMemorySize: Size computed: %d\n", nvmSize );
    }

    return ( nvmSize );
}

//----------------------------------------------------------------------------------------
// The NVJ chips have an internal page size for writing. Depending on the chip 
// memory size, the buffer is different.
// 
//----------------------------------------------------------------------------------------
uint32_t determineBufferBlockSize( uint32_t size ) {

    uint32_t bufSize = 0;

    switch ( size ) {

        case 4 * 1024:   
        case 8 * 1024:   bufSize = 32; break;

        case 16 * 1024: 
        case 32 * 1024:  bufSize = 64; break;

        case 64 * 1024:  bufSize = 128; break;

        default:         bufSize = 16; 
    }

    if ( nvmDebugEnabled( )) {

        printf( "determineBufferBlockSize: Size computed: %d\n", bufSize );
    }

    return ( bufSize );
}

//----------------------------------------------------------------------------------------
// "nvmGetBytesFromPage" transmits a set of data bytes only within the page boundary. 
// Although a read can cross a page boundary, we follow the same principle as we 
// do for writes when it comes to page boundaries. The read is sending the address
// with retaining the bus. The PICO library will then use the restart condition. 
// Just like we did in the write buffer counterpart, we need to send the address 
// as one buffer.
//
//----------------------------------------------------------------------------------------
uint8_t nvmGetBytesFromPage( uint8_t  rNum, 
                             uint8_t  i2cAdr, 
                             uint32_t ofs, 
                             uint8_t  *buf, 
                             uint32_t len ) {

    uint8_t rStat = NO_ERR;

    if ( nvmDebugEnabled( )) {

        printf( "nvmGetBytesFromPage: rNum: %d, i2cAdr: 0x%x," 
                " ofs: 0x%x, buf: %p, len: %d\n", 
                rNum, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );

    uint8_t adr[ 2 ];

    adr[ 0 ] =  ( ofs >> 8 ) & 0xFF;
    adr[ 1 ] =  ofs & 0xFF;

    rStat = i2cWrite( rNum, i2cAdr, adr, 2, true );
    if ( rStat == NO_ERR ) rStat = i2cRead( rNum, i2cAdr, buf, len, false );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmPutBytesInPage" transmits a set of data bytes only within the page boundary.
// In general, a write cannot cross a chip internal page boundary. The Chip expects 
// a write to be one sequence with the address bytes first followed by the data bytes
// with no stop or restart condition in between. This took me quite some debugging 
// to figure  this out. We will have a local buffer where we combine the address and
// data and then send it.
//
//----------------------------------------------------------------------------------------
uint8_t nvmPutBytesInPage( uint8_t  rNum, 
                           uint8_t  i2cAdr, 
                           uint32_t ofs, 
                           uint8_t  *buf, 
                           uint32_t len ) {

    uint8_t rStat = NO_ERR;
    uint8_t dataBuf[ MAX_BUFFER_BLOCK_SIZE + 2 ];

    if ( nvmDebugEnabled( )) {

        printf( "nvmPutBytesInPage: rNum: %d, i2cAdr: 0x%x, ofs: 0x%x, "
                "bufAdr: %p, len: %d\n", rNum, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );

    dataBuf[ 0 ] = ( ofs  >> 8 ) & 0xFF;
    dataBuf[ 1 ] = ofs & 0xFF;

    for ( int i = 0; i < len; i++ ) dataBuf[ i + 2 ] = buf[ i ];

    rStat = i2cWrite( rNum, i2cAdr, dataBuf, len + 2, false );

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmGetBytes" reads a set of data bytes from the memory. Although read operations
// do not have a page boundary issue, we stick to the concept to read within page 
// boundaries as we may one day use more than chip to build NVMs and then we have 
// no problems with crossing chip boundaries.
//
//----------------------------------------------------------------------------------------
uint8_t nvmGetBytes( uint8_t rNum, 
                     uint8_t i2cAdr, 
                     uint32_t ofs, 
                     uint8_t *buf, 
                     uint32_t len ) {

    uint8_t rStat = NO_ERR;

    if ( nvmDebugEnabled( )) {

        printf( "nvmGetBytes: rNum: %d, i2cAdr: 0x%x, ofs: 0x%x, "
                "bufAdr: %p, len: %d\n", rNum, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );
    if ( ofs + len > nvmSize ) return ( RET_STAT( ERR_NVM_SIZE_EXCEEDED ));

    uint32_t bufSize = (( rNum == rNumNvm ) ? nodeNvmBlockSize : extNvmBlockSize );

    uint32_t  bytesLeft     = len;
    uint32_t  pageBytesLeft = bufSize - ofs % bufSize;

    while ( bytesLeft > pageBytesLeft ) {

        rStat = nvmGetBytesFromPage( rNum, 
                                     i2cAdr, 
                                     ofs + len - bytesLeft, 
                                     buf + len - bytesLeft, 
                                     pageBytesLeft );
        if ( rStat != NO_ERR ) break;

        bytesLeft       -= pageBytesLeft;
        pageBytesLeft   = bufSize;
    }

    if (( rStat == NO_ERR ) && ( bytesLeft > 0 )) {

        rStat = nvmGetBytesFromPage( rNum, 
                                     i2cAdr, 
                                     ofs + len - bytesLeft, 
                                     buf + len - bytesLeft, 
                                     bytesLeft );
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmPutBytes" transmits a set of data bytes to the memory. We cannot write across
// the internal NVM page boundary and also across a chip boundary. This routine will
// split  the data to write only within one page in a given write cycle.
//
// There is a quirk with figuring out that a chip is ready for the next write. The 
// data sheet suggest a writing of one byte to see of the chip acknowledges. If not 
// it is still in a write operation. This approach does not seem to work with the 
// PICO i2c libraries. So, we will go the "slow" way of giving the chip the time to
// complete the write cycle before issuing another one. Since we do not often write
// to the NVM, the slow mode is perhaps acceptable for now.
//
//----------------------------------------------------------------------------------------
uint8_t nvmPutBytes( uint8_t rNum, 
                     uint8_t i2cAdr, 
                     uint32_t ofs, 
                     uint8_t *buf, 
                     uint32_t len ) {

    uint8_t rStat = NO_ERR;

    if ( nvmDebugEnabled( )) {

        printf( "nvmPutBytes: rNum: %d, i2cAdr: 0x%x, ofs: 0x%x,"
                " buf: %p, len: %d\n", rNum, i2cAdr, ofs, buf, len );
    }

    uint32_t nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );
    if ( ofs + len > nvmSize ) return ( RET_STAT( ERR_NVM_SIZE_EXCEEDED ));

    uint32_t bufSize = (( rNum == rNumNvm ) ? nodeNvmBlockSize : extNvmBlockSize );

    uint32_t  bytesLeft     = len;
    uint32_t  pageBytesLeft = bufSize - ofs % bufSize;

    while ( bytesLeft > pageBytesLeft ) {

        rStat = nvmPutBytesInPage( rNum, 
                                   i2cAdr, 
                                   ofs + len - bytesLeft, 
                                   buf + len - bytesLeft, 
                                   pageBytesLeft );
        if ( rStat != NO_ERR ) break;

        bytesLeft       -= pageBytesLeft;
        pageBytesLeft   = bufSize;

        sleepMillis( NVM_WRITE_DELAY );
    }

    if (( rStat == NO_ERR ) && ( bytesLeft > 0 )) {

       rStat = nvmPutBytesInPage( rNum, 
                                  i2cAdr, 
                                  ofs + len - bytesLeft, 
                                  buf + len - bytesLeft,
                                  bytesLeft );
       sleepMillis( NVM_WRITE_DELAY );
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "nvmClearArea" wipes out an area of the NVM chip. To speed up the writing, we 
// fill a local buffer with the value and then write blocks at a time.
//
//----------------------------------------------------------------------------------------
uint8_t nvmClearArea( uint8_t rNum, 
                      uint8_t i2cAdr, 
                      uint32_t ofs, 
                      uint32_t len, 
                      uint8_t val ) {

    if ( nvmDebugEnabled( )) {

        printf( "nvmClearArea: rNum: %d, i2c: 0x%x, ofs: 0x%x, len: %d, val: %d\n", 
                rNum, i2cAdr, ofs, len, val );
    }

    uint8_t     tmpBuf[ DEF_BUFFER_BLOCK_SIZE ];
    uint8_t     rStat   = NO_ERR;
    uint32_t    nvmSize = (( rNum == rNumNvm ) ? nodeNvmSize : extNvmSize );
    uint32_t    limit   = ofs + len;

    if ( ofs + len > nvmSize ) return ( RET_STAT( ERR_NVM_SIZE_EXCEEDED ));

    for ( int i = 0; i < DEF_BUFFER_BLOCK_SIZE; i ++ ) tmpBuf[ i ] = val;

    while ( len > DEF_BUFFER_BLOCK_SIZE ) {

        rStat = nvmPutBytes( rNum, i2cAdr, ofs, tmpBuf, sizeof( tmpBuf ));
        if ( rStat != NO_ERR ) break;
        
        ofs += DEF_BUFFER_BLOCK_SIZE;
        len -= DEF_BUFFER_BLOCK_SIZE;
    }

    if (( rStat == NO_ERR ) && ( len > 0 )) {

        rStat = nvmPutBytes( rNum, i2cAdr, ofs, tmpBuf, len );
    }

    return ( RET_STAT( rStat ));
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
uint8_t configNvm(  uint8_t rIdNvm, uint32_t nvmSize ) {

    rNumNvm     = rIdNvm;
    nodeNvmSize = nvmSize;
    extNvmSize  = extNvmSize;

    if ( nodeNvmSize > NVM_MAX_NVM_SIZE )   nodeNvmSize = NVM_MAX_NVM_SIZE;

    uint32_t testSize = determineNvmChipMemorySize( rIdNvm, NVM_I2C_ADR_ROOT );
    if ( testSize < nodeNvmSize ) nodeNvmSize = testSize;

    nodeNvmBlockSize = determineBufferBlockSize( nodeNvmSize );
    extNvmBlockSize  = DEF_BUFFER_BLOCK_SIZE;

    if ( nvmDebugEnabled( )) {

        printf( "configNvm: rIdNvm: %d, size: %d, blockSize: %d\n",
                rIdNvm, nodeNvmSize, nodeNvmBlockSize ); 
    }

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// Controller Board Runtime Map access routines. The runtime map occupies the first 
// 8 Kbytes of the main controller NVM chip. There are routines for getting and 
// setting a word as well as routines to read and write a buffer. All access 
// routines are prefixed with "rt".
//
//----------------------------------------------------------------------------------------
uint8_t rtNvmPutWord( uint32_t ofs, uint16_t word ) {

    return ( nvmPutBytes( rNumNvm, 
                          NVM_I2C_ADR_ROOT + 0, 
                          ofs, 
                          (uint8_t *) &word, 
                          sizeof( uint16_t )));
}

uint8_t rtNvmGetWord( uint32_t ofs, uint16_t *word ) {

    return ( nvmGetBytes( rNumNvm, 
                          NVM_I2C_ADR_ROOT + 0, 
                          ofs, 
                          (uint8_t *) word, 
                          sizeof( uint16_t )));
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

#if 0
// ??? they will go away ... ?
//----------------------------------------------------------------------------------------
// Extension Board Map access routines. These routines access the NVM on the 
// extension board. The I2C address is formed by the chip common I2C address plus
// the address bits of the chip to select the chip on the particular extension 
// board. Similar to the runtime NVM access routines, there are routines for 
// getting and setting a word as well as routines to read and  write a buffer. 
// All access routines are prefixed with "ext".
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


// ??? will go away ? we compute the offset and use the standard routines rtNvmXXX
//----------------------------------------------------------------------------------------
// Controller Board User Map access routines. The area between the main controller 
// NVM chip runtime area and the chips hardware maximum size is the memory area 
// available for the firmware programmer. Again, there are routines for getting and
// setting a word as well as routines to read and  write a buffer. All access routines
// are  prefixed with "usr".
//
// ??? how do we best offers the user space ? just a set of attributes ?
// ??? or do we model his just like the runtime area and let the "item" code
// figure out what to access ?
//----------------------------------------------------------------------------------------
uint8_t usrNvmPutWord( uint32_t ofs, uint16_t word ) {

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmPutBytes( rNumNvm, 
                          NVM_I2C_ADR_ROOT + 0, 
                          ofs, 
                          (uint8_t *) &word, 
                          sizeof( uint16_t )));
}

uint8_t usrNvmGetWord( uint32_t ofs, uint16_t *word ) {

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmGetBytes( rNumNvm, 
                          NVM_I2C_ADR_ROOT + 0, 
                          ofs, 
                          (uint8_t *) word, 
                          sizeof( uint16_t )));
}

uint8_t usrNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmPutBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint8_t usrNvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len ) {

    ofs = ofs + NVM_USER_MAP_OFS;
    return ( nvmGetBytes( rNumNvm, NVM_I2C_ADR_ROOT + 0, ofs, buf, len ));
}

uint32_t usrNvmGetSize( ) {
       
    return ( nodeNvmSize - NVM_RUNTIME_MAPS_SIZE );
}

#endif

}; // namespace LCS
