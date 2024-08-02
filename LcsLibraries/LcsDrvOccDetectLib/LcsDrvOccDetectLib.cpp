//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occuopancy Detect extension boards
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the ...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occuopancy Detect extension boards
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

  //----------------------------------------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t   i2cAdr    = 0x20;
  uint16_t  ioData    = 0;


  //----------------------------------------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t initDrv( ) {

    // CDC::configureI2C( 0, 0 );


    return ( 0 );
  }

  //----------------------------------------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------------------------------------
  void reset( ) {

    // writeI2C( adr, val );
  }



  //----------------------------------------------------------------------------------------------------------
  //
  // ??? use CDC....
  //----------------------------------------------------------------------------------------------------------
  uint8_t readReg( uint8_t reg ) {

    // WIRE.beginTransmission( i2cAdr );
    // WIRE.write( reg );
    // WIRE.endTransmission( );

    // WIRE.requestFrom(( uint8_t) i2cAdr, (uint8_t) 1 );
    // return ( WIRE.read( ));

    return( 0 ); // for now ...
  }

  void writeReg( uint8_t reg, uint8_t val ) {

    //  WIRE.beginTransmission( i2cAdr );
    //  WIRE.write( reg );
    //  WIRE.write( val );
    // WIRE.endTransmission( );
  }

  bool testI2C ( uint8_t i2cAdr ) {

    // WIRE.beginTransmission( i2cAdr );
    // WIRE.write( 0x02 );
    // return ( WIRE.endTransmission( ) == 0 );

    return( 0 ); // for now ...
  }

  // ??? perhaps a nice function for the I2C layer
  // ??? we could also think about a routine that returns an array of 128 bits, one for each device that answered...

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Each drivr is just a function to handle the request.
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect( uint8_t boardId, uint8_t item, uint16_t arg1, uint16_t *arg2 ) {

  switch( item ) {


    default: ;
  }


  return( 0 ); // ??? for now ...
}

#if 0
//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect::init( uint16_t flags ) {

  initDrv( );

  // ??? set inversion bit to one. Explain ...
  writeReg( 4, 0xFF );
  writeReg( 5, 0xFF );

  // ??? set pins as input
  writeReg( 6, 0xFF );
  writeReg( 7, 0xFF );

  return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect::reset( uint16_t flags ) {

  return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect::config( uint8_t padId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

 

  // ??? e.g. setting the IO direction...
  //
  // arg1 -> pin, or a port or a two ports bit mask
  // arg2 -> 0 = output ( PCA9555 expects a zero for output )  & ~ ( 1 << pin )
  // arg2 -> 1 = input  ( PCA9555 expects a one for input )    | ( 1 << pin )
  //
  // have a 16 bit mask for the bits, write both a when we set something ?
  // writeI2C ( config reg 1, 2 )

  return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
//
// ??? set IO direction flags
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect::control( uint8_t padId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

  // ??? e.g. setting the IO direction...
  //
  // arg1 -> pin, or a port or a two ports bit mask
  // arg2 -> 0 = output ( PCA9555 expects a zero for output )  & ~ ( 1 << pin )
  // arg2 -> 1 = input  ( PCA9555 expects a one for input )    | ( 1 << pin )
  //
  // have a 16 bit mask for the bits, write both a when we set something ?
  // writeI2C ( config reg 1, 2 )



  return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
//
// ??? get the config data
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect::info( uint8_t padId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

  // ??? return the config data
  // ?? return the state of the pin register

  return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
//
// ??? read the two ports
// ??? read a port
// ??? read a bit
//
// ??? quick hack for now ...
//------------------------------------------------------------------------------------------------------------
uint8_t LcsDrvOccDetect::read( uint8_t padId, uint16_t *arg ) {

  // ??? read the INPUT register...
  // ??? read both registers and return the bit, port or all readI2C( )
  
    uint8_t tmp1 = readReg( 0 );
    uint8_t tmp2 = readReg( 1 );

    *arg = ( tmp1 << 8 ) | tmp2; 

    return ( 0 );
}


  #endif


  
