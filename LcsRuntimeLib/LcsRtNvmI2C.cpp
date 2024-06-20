//------------------------------------------------------------------------------------------------------------
//
// LCS Core library - Non volatile storage based on the M24LCxxx chip family
//
//------------------------------------------------------------------------------------------------------------
// This file implements the LCS core library non-volatile memory. The hardware is the AA24xxx chip family,
// which offers an I2C protocol based chip with various capacities. They all share the same pin layout and
// command structure.
//
// The chip family allows up to eight chips on the I2Caddress root. We will support a model if up to four
// chips on the particular I2C bus. Each chip is an object, the class has static functions to transparently
// select the correct chip for the I/O reques.
//
//
// In addition we also support the M24C04 chip, which is used on the extension boards as a configuration
// storage.
//
// ??? the M24C04 chip will however go away... replaced by 24AA32.
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
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"


//------------------------------------------------------------------------------------------------------------
// Local file declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  //----------------------------------------------------------------------------------------------------------
  // Definitions for the M24LCxxx chips page size and total size. The chips have a pageSize which is the unit
  // updated in case of a write. A write cannot across a page boundary and must be split into several writes
  // if necessay. Reads do not have this problem. Al chips have the same I2C address root which is "1010".
  // There are three address lines A2, A1 and A0, which are all used on all chips. Up to eight chips can
  // this be addressed on a single I2C bus.
  //
  // The pageSizes on the chip are a multiple of 32bytes. For now, we use this size as the common denominator.
  // Block handling and chipSize page handling are nicely taken care of this way. The downside is however that
  // a write will update the chip page up to four times for a pageSize of 128. However, since the chips have
  // more than a million write cycles and we rately write large chunks of data, this will hopefully not be an
  // issue in the near future.
  //
  //----------------------------------------------------------------------------------------------------------
  const uint16_t  MAX_NVM_CHIPS           = 4;
  const uint16_t  BUFFER_BLOCK_SIZE       = 32;

  const uint16_t  M24LC32_PAGE_SIZE       = 32;
  const uint32_t  M24LC32_MAX_SIZE        = 4096;

  const uint16_t  M24LC64_PAGE_SIZE       = 32;
  const uint32_t  M24LC64_MAX_SIZE        = 8192;

  const uint16_t  M24LC128_PAGE_SIZE      = 64;
  const uint32_t  M24LC128_MAX_SIZE       = 16384;

  const uint16_t  M24LC256_PAGE_SIZE      = 64;
  const uint32_t  M24LC256_MAX_SIZE       = 32768;

  const uint16_t  M24LC512_PAGE_SIZE      = 128;
  const uint32_t  M24LC512_MAX_SIZE       = 65536;

  const uint16_t  M24C04_PAGE_SIZE        = 8;
  const uint32_t  M24C04_MAX_SIZE         = 512;

  const uint8_t   FIRST_CHIP_I2C_ADR      = 0b1010000;

  const uint32_t NVM_SYS_MAP_SIZE         = 0x2000;
  const uint32_t NVM_MAX_MAP_SIZE         = 0x40000;

  //----------------------------------------------------------------------------------------------------------
  // We maintain a table of NVM chip descriptors. The table entry contains the NVM object allocated and the
  // absolute address range that the chip represents. The i2cAdr is the final address of the chip, which is
  // the root "1010" and the chip address bits A1 and A0. The maximum size is the size of the chip in
  // Kbytes. The page size defines the chippage soze for writes. Writes csannot cross a page boundary. The
  // start and end address is assigend when the chip is added to the list. We always start with the first
  // entry and work our way up.
  //
  //----------------------------------------------------------------------------------------------------------
  struct NvmTabEntry {

    uint8_t   i2cAdr;
    uint32_t  size;
    uint32_t  startAdr;
    uint32_t  endAdr;
  };

  //----------------------------------------------------------------------------------------------------------
  // Module global data.
  //
  //----------------------------------------------------------------------------------------------------------
  uint32_t    nvmMaxSize                      = 0;
  uint32_t    nvmUserMapSize                  = 0;
  uint8_t     nvmSclPin                       = CDC::UNDEFINED_PIN;
  uint8_t     nvmSdaPin                       = CDC::UNDEFINED_PIN;
  NvmTabEntry nvmTab[ MAX_NVM_CHIPS ];

  //----------------------------------------------------------------------------------------------------------
  // A little help function to test whether the chip is read for the next operation. The test is to write to
  // the chip and see if this works.
  //
  //----------------------------------------------------------------------------------------------------------
  bool chipReady( uint8_t i2cAdr ) {

    uint8_t ret = 1;
    uint8_t tmp = 0;

    while ( ret != ALL_OK ) {

      ret = CDC::i2cWrite( nvmSclPin, i2cAdr, &tmp, 1 );
    }

    return ( true );
  }

  //----------------------------------------------------------------------------------------------------------
  // Each NVM chip has certain size. This function will round the size to the next lower chip memory size.
  // We expect however that the programmer used the correct size, so this is done just in case. The lowest
  // value is the 4Kb chip.
  //
  //----------------------------------------------------------------------------------------------------------
  uint32_t roundNvmMaxSize( uint16_t chipSize ) {

    if      ( chipSize <= M24C04_MAX_SIZE )   return ( M24C04_MAX_SIZE );
    else if ( chipSize <= M24LC32_MAX_SIZE )  return ( M24LC32_MAX_SIZE );
    else if ( chipSize <= M24LC64_MAX_SIZE )  return ( M24LC64_MAX_SIZE );
    else if ( chipSize <= M24LC128_MAX_SIZE ) return ( M24LC128_MAX_SIZE );
    else if ( chipSize <= M24LC256_MAX_SIZE ) return ( M24LC256_MAX_SIZE );
    else if ( chipSize <= M24LC512_MAX_SIZE ) return ( M24LC512_MAX_SIZE );
    else                                      return ( M24LC32_MAX_SIZE );
  }

  //----------------------------------------------------------------------------------------------------------
  // Among other data, the node map contains the data for the NVM subsystem. This is primaily the number and
  // size of the chips on the NVM I2C bus. We just read in the node map and do basic consistency checking.
  // Note that we do not know any thing about the actual NVM chip installation. But in any case each chip
  // would have a node map, so we just read.
  //
  // The first chip adress is "1010" and the adress bits "000".
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t readInitialNodeMap( LcsNodeMap *nodeMap ) {

    if ( chipReady( FIRST_CHIP_I2C_ADR )) {

      uint8_t tmp = 0;

      CDC::i2cWrite( nvmSclPin, FIRST_CHIP_I2C_ADR, &tmp, 1 );
      CDC::i2cWrite( nvmSclPin, FIRST_CHIP_I2C_ADR, &tmp, 1 );

      CDC::i2cRead( nvmSclPin, FIRST_CHIP_I2C_ADR, (uint8_t *) nodeMap, sizeof( LcsNodeMap ));
    }

    // sanity check what we have ....


    return ( ALL_OK );
  }

  //----------------------------------------------------------------------------------------------------------
  // When we have a valid nodeMap descriptor, we need to set up the local NVM chip table. For each chip in
  // the nodeMap, an entry is properly initialized. Each chip then has a size and the start and end address
  // which are absolute offsets over the entire set of chips.
  //
  // ??? we insist on at least 8K NVM.
  //----------------------------------------------------------------------------------------------------------
  uint8_t buildNvmTab( LcsNodeMap *nodeMap ) {

    #if DEBUG_NVM == 1
    printf( "Build NVM Chip Table\n" );
    #endif

    uint32_t tmpOfs = 0;

    if ( nodeMap -> nvmMemSize0 >= NVM_SYS_MAP_SIZE ) {

      nvmTab[ 0 ].i2cAdr    = nodeMap -> nvmChipI2CAdrRoot + 0;
      nvmTab[ 0 ].size      = roundNvmMaxSize( nodeMap -> nvmMemSize0 );
      nvmTab[ 0 ].startAdr  = tmpOfs;
      nvmTab[ 0 ].endAdr    = tmpOfs + nvmTab[ 0 ].size - 1;

      tmpOfs += nvmTab[ 0 ].size;
    }
    else CDC::fatalError( 1 );

    if ( nodeMap -> nvmMemSize1 > 0 ) {

      nvmTab[ 1 ].i2cAdr    = nodeMap -> nvmChipI2CAdrRoot + 1;
      nvmTab[ 1 ].size      = roundNvmMaxSize( nodeMap -> nvmMemSize1 );
      nvmTab[ 1 ].startAdr  = tmpOfs;
      nvmTab[ 1 ].endAdr    = tmpOfs + nvmTab[ 1 ].size - 1;

      tmpOfs += nvmTab[ 1 ].size;
    }

    if ( nodeMap -> nvmMemSize2 > 0 ) {

      nvmTab[ 2 ].i2cAdr    = nodeMap -> nvmChipI2CAdrRoot + 2;
      nvmTab[ 2 ].size      = roundNvmMaxSize( nodeMap -> nvmMemSize2 );
      nvmTab[ 2 ].startAdr  = tmpOfs;
      nvmTab[ 2 ].endAdr    = tmpOfs + nvmTab[ 2 ].size - 1;

      tmpOfs += nvmTab[ 2 ].size;
    }

    if ( nodeMap -> nvmMemSize3 > 0 ) {

      nvmTab[ 3 ].i2cAdr    = nodeMap -> nvmChipI2CAdrRoot + 2;
      nvmTab[ 3 ].size      = roundNvmMaxSize( nodeMap -> nvmMemSize3 );
      nvmTab[ 3 ].startAdr  = tmpOfs;
      nvmTab[ 3 ].endAdr    = tmpOfs + nvmTab[ 3 ].size - 1;

      tmpOfs += nvmTab[ 3 ].size;
    }

    nvmMaxSize      = tmpOfs;
    nvmUserMapSize  = nvmMaxSize - NVM_SYS_MAP_SIZE;

    #if DEBUG_NVM == 1
    printf( "NVM System Map Size: 0x%x, User Map Size: 0x%x\n", nvmMaxSize, nvmUserMapSize );

    printf( "NVM Chip Table: \n" );

    for ( int i = 0; i < MAX_NVM_CHIPS; i++ ) {

      NvmTabEntry *ptr = &nvmTab[ i ];

      if ( ptr != nullptr ) {

        printf( "[%d]: I2CAdr: %d, size: 0x%x, start: 0x%x, end: 0x%x\n",
                i, ptr -> i2cAdr, ptr -> size, ptr -> startAdr, ptr -> endAdr );
      }
      else printf( "[%d]: not set\n", i );
    }
    #endif

    return ( ALL_OK );
  }

  //----------------------------------------------------------------------------------------------------------
  // Select the chip instance. We need to find the chip for the requested address. This is a simple search
  // of the NVM table for a chip that would contain the adress asked for. The address is the NVM absolute
  // address.
  //
  //----------------------------------------------------------------------------------------------------------
  NvmTabEntry *selectNvmChip( uint32_t adr ) {

    for ( int i = 0; i < MAX_NVM_CHIPS; i++ ) {

      NvmTabEntry *ptr = &nvmTab[ i ];
      if (( ptr != nullptr ) && ( ptr -> startAdr <= adr ) && ( ptr -> endAdr <= adr )) return ( ptr );
    }

    return ( nullptr );
  }


  //----------------------------------------------------------------------------------------------------------
  // ??? have a routine to check and compute the offset ... ?

  bool buildOffset( uint32_t *ofs, uint32_t len, bool userMap ) {

    if ( userMap ) *ofs = *ofs + NVM_SYS_MAP_SIZE;
    if ( *ofs + len >= nvmMaxSize - 1 ) return ( false );

    return ( true );
  }

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Initialize NVM. The very first thing to do when starting the node is to find the NVM data and set up the
// NVM storage. The NVM data is part of the nodeMap, which is the first area in the storage. The setup is
// done in a few steps. We first configure the I2C channel and then try read in the nodeMap. Also, a few
// basic consistency checks of the nodeMap fata are performed at this stage. If OK, then all chips set
// will be added to the NVM table.
//
// There is of course the case that the nodeMap is not valid. The most likely reason is that there is a new
// board, not configured yet. In this case we just make up a minimum nodeMap which a small NVM chip. The node
// is placed in configuration mode, the nodeMap indicates this state. It can be accessed for configuration
// but not for gneral operations.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmInitSubSys( uint8_t sclPin, uint8_t sdaPin ) {

  uint8_t       rStat = ALL_OK;
  LcsNodeMap    nodeMap;

  #if DEBUG_NVM == 1
  printf( "nvmInitSubSys: SCL: %d, SDA: %d\n", sclPin, sdaPin );
  #endif

  rStat = CDC::configureI2C( sclPin, sdaPin );
  if ( rStat == ALL_OK ) {

    nvmSclPin = sclPin;
    nvmSdaPin = sdaPin;
  }
  else CDC::fatalError( 1 );

  rStat = readInitialNodeMap(  &nodeMap );
  if ( rStat != ALL_OK ) {

    // ??? if not a valid nodeMap, make a minimum one with an (k bytes area...
  }

  rStat = buildNvmTab( &nodeMap );

  #if DEBUG_NVM == 1
  printf( "nvmInitSubSys: % d\n", rStat );
  #endif

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "nvmGetBytesFromPage" transmits a set of data bytes only within the page boundary.
//
//------------------------------------------------------------------------------------------------------------
bool nvmGetBytesFromPage( uint32_t ofs, uint8_t *buf, uint32_t len ) {

  if ((uint32_t)( ofs + len ) > nvmMaxSize - 1 ) return ( false );

  NvmTabEntry *nvm = selectNvmChip( ofs );

  if ( nvm == nullptr ) return ( 99 );

  // ??? compute the chip relative offset ...

  uint8_t i2cAdr = 0;
  uint32_t chipOfs = nvm -> startAdr - ofs;

  if ( nvmMaxSize == M24C04_MAX_SIZE ) {

    uint8_t tmpAdr  = i2cAdr | (( chipOfs >> 8 ) & 0x01 );
    uint8_t tmpData = chipOfs & 0xFF;

    if ( chipReady( tmpAdr )) {

      CDC::i2cWrite( nvmSclPin, tmpAdr, &tmpData, sizeof( tmpData ), true );
      CDC::i2cRead( nvmSclPin, tmpAdr, buf, BUFFER_BLOCK_SIZE );
    }
  }
  else {

    if ( chipReady( i2cAdr )) {

      uint8_t tmp = 0;

      tmp = (( chipOfs >> 8 ) & 0xFF );
      CDC::i2cWrite( nvmSclPin, i2cAdr, &tmp, 1 );

      tmp = chipOfs & 0xFF;
      CDC::i2cWrite( nvmSclPin, i2cAdr, &tmp, 1 );

      CDC::i2cRead( nvmSclPin, i2cAdr, buf, BUFFER_BLOCK_SIZE );
    }

  }

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//  "nvmPutBytesInPage" transmits a set of data bytes only within the page boundary.
//
//------------------------------------------------------------------------------------------------------------
bool nvmPutBytesInPage( uint32_t ofs, uint8_t *buf, uint32_t len ) {

  if ((uint32_t)( ofs + len ) >= nvmMaxSize ) return ( false );

  NvmTabEntry *nvm = selectNvmChip( ofs );

  if ( nvm == nullptr ) return ( 99 );

  // ??? compute the chip reölative offset ...

  uint8_t i2cAdr = 0;
  uint32_t chipOfs = nvm -> startAdr - ofs;

  if ( nvmMaxSize == M24C04_MAX_SIZE ) {

    uint8_t tmpAdr = i2cAdr | (( ofs >> 8 ) & 0x01 );
    uint8_t tmpOfs = ( chipOfs ) & 0xFF;

    if ( chipReady( tmpAdr )) {

      CDC::i2cWrite( nvmSclPin, tmpAdr, &tmpOfs, 1 );
      CDC::i2cWrite( nvmSclPin, tmpAdr, buf, len );
    }
  }
  else {

    if ( chipReady( i2cAdr )) {

      uint8_t tmp = 0;

      tmp = (( chipOfs >> 8 ) & 0xFF );
      CDC::i2cWrite( nvmSclPin, i2cAdr, &tmp, 1 );

      tmp = chipOfs & 0xFF;
      CDC::i2cWrite( nvmSclPin, i2cAdr, &tmp, 1 );

      CDC::i2cWrite( nvmSclPin, i2cAdr, buf, len );
    }
  }

  return ( true );
}

//------------------------------------------------------------------------------------------------------------
// "nvmGetBytes" reads a set of data bytes from the memory. Alhough read operations do not have a page
// boundary issue, we stick to the concept to read within page boundaries as we may cross a chip boudary.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmGetBytes( uint32_t ofs, uint8_t *buf, uint32_t len, bool userMap ) {

  #if DEBUG_NVM == 1
  printf( "nvmGetBytes: ofs: 0x%x, bufAdr: %ptr, len: %d\n", ofs, (uint32_t) buf, len );
  #endif

  if ( userMap ) ofs = ofs + NVM_SYS_MAP_SIZE;
  if ( ofs + len >= nvmMaxSize - 1 ) return ( 99 );

  uint32_t  bytesLeft     = len;
  uint32_t  pageBytesLeft = BUFFER_BLOCK_SIZE - ofs % BUFFER_BLOCK_SIZE;

  while ( bytesLeft > pageBytesLeft ) {

    nvmGetBytesFromPage( ofs + len - bytesLeft, buf + len - bytesLeft, pageBytesLeft );
    bytesLeft       -= pageBytesLeft;
    pageBytesLeft   = BUFFER_BLOCK_SIZE;
  }

  return ( nvmGetBytesFromPage( ofs + len - bytesLeft, buf + len - bytesLeft, bytesLeft ));
}

//------------------------------------------------------------------------------------------------------------
// "nvmPutBytes" transmits a set of data bytes to the memory. We cannot write across the internal NVM page
// boundary and also across a chip boundary. This routine will split the bytes to write only to one page in
// a given write cycle.
//
// ??? check what we could do for wraparound of unsigned int ...
//------------------------------------------------------------------------------------------------------------
uint8_t nvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len, bool userMap ) {

  #if DEBUG_NVM == 1
  printf( "nvmPutBytes: ofs: 0x%x, bufAdr: %ptr, len: %d, uMap: %d\n", ofs, buf, len, userMap );
  #endif

  if ( userMap ) ofs = ofs + NVM_SYS_MAP_SIZE;
  if ( ofs + len >= nvmMaxSize - 1 ) return ( 99 );

  uint32_t  bytesLeft     = len;
  uint32_t  pageBytesLeft = BUFFER_BLOCK_SIZE - ofs % BUFFER_BLOCK_SIZE;

  while ( bytesLeft > pageBytesLeft ) {

    nvmPutBytesInPage( ofs + len - bytesLeft, buf + len - bytesLeft, pageBytesLeft );
    bytesLeft       -= pageBytesLeft;
    pageBytesLeft   = BUFFER_BLOCK_SIZE;
  }

  return ( nvmPutBytesInPage( ofs + len - bytesLeft, buf + len - bytesLeft, bytesLeft ));
}

//------------------------------------------------------------------------------------------------------------
// "nvmGetWord" and "nvmPutWord" are convenience functions to read a 16-bit word, which we do a lot.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmGetWord( uint32_t ofs, uint16_t *word, bool userMap ) {

  return ( nvmGetBytes( ofs, (uint8_t *) word, sizeof( uint16_t ), userMap ));
}

uint8_t nvmPutWord( uint32_t ofs, uint16_t word, bool userMap ) {

  return ( nvmPutBytes( ofs, (uint8_t *) &word, sizeof( uint16_t ), userMap ));
}

//------------------------------------------------------------------------------------------------------------
// Fill an NVM area with an initial value.
//
//------------------------------------------------------------------------------------------------------------
uint8_t nvmInitArea( uint32_t ofs, uint32_t len, uint8_t val, bool userMap ) {

  for ( uint32_t i = 0; i < len; i++ ) {

    uint8_t rStat = nvmPutBytes( ofs + i, (uint8_t *) &val, 1U, userMap );
    if ( rStat != ALL_OK ) return ( rStat );
  }

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// Getter functions.
//
//------------------------------------------------------------------------------------------------------------
uint32_t nvmGetSize( bool userMap ) {

  return ( nvmMaxSize );
}


// it would perhaps be better to have a dedicated set of user routines, so that we do not write into
// the system area...

// we could also just have a optional paramater, which is false by default and all internal write set it to true...
