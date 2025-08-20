///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for SERVO boards
//
///---------------------------------------------------------------------------------------
// This source file contains the lower level library for all the servo extension board.
// We also call this library a "driver". The driver provides a set of defined interfaces
// to the upper level extension library. Being a driver, it truly knows the hardware
// underneath and maps the upper level calls to the lower level hardware calls to make.
//
///---------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2023  Helmut Fieres
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

#include "LcsDrvServoLib.h"

///---------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space with
// all internal variables and routines local to the file. The second part contains the 
// exported routines to be called by the core library and the firmware designers.
//
///---------------------------------------------------------------------------------------
namespace {

///---------------------------------------------------------------------------------------
//
///---------------------------------------------------------------------------------------

const int PCA9685_SUB_ADR_1 = 2;
const int PCA9685_SUB_ADR_2 = 3;
const int PCA9685_SUB_ADR_3 = 4;

const int PCA9685_MODE_1    = 0;

const int PCA9685_PRESCALE = 0xFE;

// ??? start for the LED registers. Find the others by adding to these values. ( n * 4 )
const int PCA9685_LED0_ON_L_ADR = 0x6;
const int PCA9685_LED0_ON_H_ADR = 0x7;
const int PCA9685_LED0_OFF_L_ADR = 0x8;
const int PCA9685_LED0_OFF_H_ADR = 0x9;

const int PCA9685_LED_ALL_ON_L_ADR = 0xFA;
const int PCA9685_LED_ALL_ON_H_ADR = 0xFB;
const int PCA9685_LED_ALL_OFF_L_ADR = 0xFC;
const int PCA9685_LED_ALL_OFF_H_ADR = 0xFD;


uint8_t i2cAdr;
uint8_t sclPin;


  bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

  return (( val >= lower ) && ( val <= upper ));
}




  uint8_t readReg( uint8_t reg ) {

    uint8_t buf[ 2 ];
    uint8_t rStat = CDC::NO_ERR;
    
    rStat = CDC::i2cWrite( sclPin, i2cAdr, &reg, 1 );
    if ( rStat == CDC::NO_ERR ) {

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

    while ( ret != CDC::NO_ERR ) {

        ret = CDC::i2cWrite( sclPin, i2cAdr, &tmp, 1 );
    }

    return ( true );
}

  //--------------------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------------------
  void init( ) {

    // WIRE.begin( );
    // reset( );
  }

  //--------------------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------------------
  void reset( ) {

    // writeI2C( PCA9685_MODE_1, 0 );
  }

  //--------------------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------------------
  void setPwmFreq( float freq ) {


    // some computation to end up with the preScale value derived from "freq"
    /*
      freq *= 0.9; // overshoot correction ?
      float pp = 25000000; // 25Mhz
      pp /= 4096;
      pp /= freq;
      pp -= 1;
      preScaleVal = flor/ pp + 0.5 );
    */

    uint8_t preScaleVal = 0;

    // uint8_t oldMode = readI2C( PCA9685_MODE_1 );
    // uint8_t newMode = ( oldMode & 0x7F ) | 0x10; // sleep

    // writeI2C( PCA9685_MODE_1, newMode );
    // writeI2C( PCA9685_MODE_1, preScaleVal );
    // writeI2C( PCA9685_MODE_1, oldMode );
    // delay( 5 );
    // writeI2C( PCA9685_MODE_1, oldMode | 0xA1; // turn auto increment

  }

  //--------------------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------------------
  // ??? idea: set the LED duty cycle staggered in the 4096 bit window, such that not
  //  all LEDs will draw current at the same time.

  void setPwm( uint8_t ledNum, uint16_t on, uint16_t off ) {

    // ??? need a write16...

  /*
    WIRE.beginTransmission( i2cAdr );
    WIRE.write( PCA9685_LED0_ON_L_ADR + ( ledNum * 4 ));
    WRITE.write( on );
    WIRE.write( on >> 8 );
    WIRE.write( off );
    WIRE.write( off >> 8 );
    WIRE.endTransmission( );
  */
  }

  //--------------------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------------------
  void setLed( uint8_t ledNum, uint16_t val, bool invert ) {

  /*
    val = min( val, 4095 );
    if ( invert ) {

      if      ( val == 0 )    setPwm( ledNum, 4096, 0 ); // fully on
      else if ( val == 4095 ) setPwm( ledNum, 0, 4096 ); // fully off
      else                    setPwm( ledNum, 0, 4095 - val );
    }
    else {

      if      ( val == 4095 ) setPwm( ledNum, 4096, 0 ); // fully on
      else if ( val == 0 )    setPwm( ledNum, 0, 4096 ); // fully off
      else                    setPwm( ledNum, 0, val );
    }
  */
  }


  //--------------------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------------------
  // ??? will go into CDC....

  uint8_t readI2C( uint8_t adr ) {

  /*
    WIRE.beginTransmission( i2cAdr );
    WIRE.write( adr );
    WIRE.endTransmission( );

    WIRE.requestFrom(( uint8_t) i2cAdr, (uint8_t) 1 );
    return ( WIRE.read( )); 
  */

  return( 0 );

  }

  void writeI2C( uint8_t adr, uint8_t val ) {

  /*
    WIRE.beginTransmission( i2cAdr );
    WIRE.write( adr );
    WIRE.write( val );
    WIRE.endTransmission( );
  */
  }

}; // namespace



namespace LCS {

///---------------------------------------------------------------------------------------
// Each driver is just a function to handle the request.
//
///---------------------------------------------------------------------------------------
uint8_t lcsDrvServo( uint8_t boardId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

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

        default: return( ERR_INVALID_DRV_ITEM );
    }
}

} // namespace
