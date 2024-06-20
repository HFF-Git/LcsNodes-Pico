//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime setup file.
//
//------------------------------------------------------------------------------------------------------------
// The file implements a part of the LcsRuntimeLib that deals with the startup and storage aspects of a node.
//
//
//
//
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
// Externals.
//
//------------------------------------------------------------------------------------------------------------
extern LcsRuntimeMap             runtimeMap;
extern LcsCallbackMap            callbackMap;


//------------------------------------------------------------------------------------------------------------
// The LcsCoreLibConfig implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  //----------------------------------------------------------------------------------------------------------
  // The NVM node map area has a "magic" word as the first bytes. The idea is simple. If this word is read,
  // the area was initialized before and we continue to check the rest of the header.
  //
  //----------------------------------------------------------------------------------------------------------
  const uint32_t NVM_MAGIC_WORD  = (uint32_t) 0x0050102b;

  //----------------------------------------------------------------------------------------------------------
  // Utility routines for number range check.
  //
  //----------------------------------------------------------------------------------------------------------
  static inline bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
  }

  //----------------------------------------------------------------------------------------------------------
  // "roundup" rounds up a value to the next highest multiple of the block size.
  //
  //----------------------------------------------------------------------------------------------------------
  static uint16_t roundup( uint16_t elements, uint16_t alignSize ) {

    return ((( elements + alignSize - 1 ) / alignSize ) * alignSize );
  }

}; // namespace


//============================================================================================================
// Object part.
//============================================================================================================


// The startup code needs to completely reworked...



//------------------------------------------------------------------------------------------------------------
// The very first thing to do is to setup the CDC library and setup the "active" and "ready" LED pins used by
// the board. The pins need to to be configured. We also make a call to to initialize the CDC. Note that this
// may have been done before, when for example the firmware programmer wants to use the HW before calling any
// library setup code.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCdcLayer( ) {

  #if DEBUG_CONFIG == 1
  #endif

  CDC::CdcPinConfig *ci = nullptr;

  CDC::init( ci );

  if ( ci -> READY_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> READY_LED_PIN, CDC::OUT );
  if ( ci -> ACTIVE_LED_PIN != CDC::UNDEFINED_PIN ) CDC::configureDio( ci -> ACTIVE_LED_PIN, CDC::OUT );

  #if DEBUG_CONFIG == 1
  #endif

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
// Befoee we can do anything, we need access to the NVM data.
//
//------------------------------------------------------------------------------------------------------------
uint8_t initNvmHw( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? setup NVM object and put it into "nvm" variable...

  /*
    if ( cLibDesc.options & NOPT_USE_EXT_NVM ) {

      if (( cdcDesc.NVM_CHIP_TYPE == M24LC128 ) ||
          ( cdcDesc.NVM_CHIP_TYPE == M24LC256 ) ||
          ( cdcDesc.NVM_CHIP_TYPE == M24LC1024 )) {

        nvmMap = new LcsNvmI2C( cdcDesc.I2C_SCL_PIN_1,
                                cdcDesc.I2C_SDA_PIN_1,
                                cdcDesc.I2C_ADR_DEF_1,
                                cdcDesc.NVM_CHIP_TYPE );
      }
      else {

    #if DEBUG_CONFIG == 1
        INTERFACE.println( F( "setupNvm: Invalid NVM chip type" ));
    #endif
      }
    }

    #if DEBUG_CONFIG == 1
    INTERFACE.print( "Allocate NVM Window (max:size:ofs): " );
    INTERFACE.print( nvmMap -> getMaxNvmSize( ));
    INTERFACE.print( ":" );
    INTERFACE.print( nvmMap -> getNvmSize( ));
    INTERFACE.print( ":" );
    INTERFACE.println( nvmMap -> getNvmOfs( ));
    #endif


    return (( nvmMap -> getNvmSize( ) > 0 ) ? ALL_OK : ERR_NVM_SETUP );


  */

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t initCanBus( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? setup CAN object and put it into "msgBus" variable...

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupRootMap( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? create the memory space, and read in the root data descriptor.
  // ??? validate it, and if valid, create the MEM space for the entire data we need.

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupNodeMap( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? read in the NVM data at the right place in the MEM structure.
  // ??? anything to validate ?

  // nvm -> getNvmBytes( ... );

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupPortMap( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? read in the NVM data at the right place in the MEM structure.
  // ??? anything to validate ?

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupEventMap( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? read in the NVM data at the right place in the MEM structure.
  // ??? anything to validate ?
  
  // ??? the event map is expected to be sorted as we only add / remove sorted.
  // ??? only read up to the high water mark.

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupUserMap( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? create the memory space, and read in the NVM data...

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t detectExtensionBoards( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? try to find extension boards ( 1 to 4 )

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupDriverEnv( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? init each driver

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupExtensionBoards( ) {

  #if DEBUG_CONFIG == 1
  #endif

  uint8_t rStat = detectExtensionBoards( );

  if ( rStat == ALL_OK ) {

    rStat = setupDriverEnv( );
  }

  return ( ALL_OK );
}


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t invokeInitCallbacks( ) {

  #if DEBUG_CONFIG == 1
  #endif

  // ??? invoke registered user callbacks

  return ( ALL_OK );
}




//------------------------------------------------------------------------------------------------------------
//
//
//
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t initRuntime( ) {

  #if DEBUG_CONFIG == 1
  #endif

  uint8_t rStat = ALL_OK;

  if ( rStat == ALL_OK ) rStat = initCdcLayer( );
  if ( rStat == ALL_OK ) rStat = initNvmHw( );
  if ( rStat == ALL_OK ) rStat = initCanBus( );
  if ( rStat == ALL_OK ) rStat = setupNodeMap( );
  if ( rStat == ALL_OK ) rStat = setupPortMap( );
  if ( rStat == ALL_OK ) rStat = setupEventMap( );
  if ( rStat == ALL_OK ) rStat = setupUserMap( );
  if ( rStat == ALL_OK ) rStat = setupExtensionBoards( );

  // ??? anything else ?

  return ( ALL_OK );
}
