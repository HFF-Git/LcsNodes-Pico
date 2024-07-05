//------------------------------------------------------------------------------------------------------------
//
// LCS - Basic Throttle
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
// ??? both throttle could share a throttle lib with common code...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Basic Throttle Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2024 Helmut Fieres
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

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg;


//----------------------------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

  cfg = CDC::getConfigDefault( );

  //--------------------------------------------------------------------------------------------------------
  // Current mapping: Main Controller Board B.01.00 - PICO - newest version.
  //
  //--------------------------------------------------------------------------------------------------------
  cfg.ADC_PIN_0             = 26;
  cfg.ADC_PIN_1             = 27;

  cfg.PWM_PIN_0             = 20;
  cfg.PWM_PIN_1             = 21;

  cfg.PFAIL_PIN             = 7;
  cfg.EXT_INT_PIN           = 22;
  cfg.READY_LED_PIN         = 14;
  cfg.ACTIVE_LED_PIN        = 15;

  cfg.DIO_PIN_0             = 9;
  cfg.DIO_PIN_1             = 8;
  cfg.DIO_PIN_2             = 10;
  cfg.DIO_PIN_3             = 11;
  cfg.DIO_PIN_4             = 21;
  cfg.DIO_PIN_5             = 20;
  cfg.DIO_PIN_6             = 19;
  cfg.DIO_PIN_7             = 18;

  cfg.NVM_I2C_SCL_PIN       = 17;
  cfg.NVM_I2C_SDA_PIN       = 16;
  cfg.NVM_I2C_ADR_ROOT      = 0x50;

  cfg.EXT_I2C_SCL_PIN       = 3;
  cfg. EXT_I2C_SDA_PIN      = 2;
  cfg.EXT_I2C_ADR_ROOT      = 0x50;

  CDC::printConfigInfo( &cfg );
}

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

  setupConfigInfo( );
 
  uint8_t rStat = CDC::init( &cfg );

  if ( rStat != ALL_OK ) {


  }


  printf( "LCS Basic Throttle\n" );

  
  if ( rStat != 0 )  printf( "Err code: %d\n", rStat );
  else printf( "OK\n" );

  return( 0 );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {


  return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t startLcsRuntime( ) {


  return( ALL_OK );
}



//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    initLcsRuntime( );
    registerCallbacks( );
    startLcsRuntime( );
    return( 0 );
}

#if 0 

// rework ...

//------------------------------------------------------------------------------------------------------------
//
// LCS - Cab Handheld
//
//------------------------------------------------------------------------------------------------------------
//
//
//  Key modules...
//
//  - CabLcsBus
//  - CabScreens
//  - CabStack
//  - CabUIElements
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Cab Handheld
// Copyright (C) 2019 - 2023  Helmut Fieres
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
#include "CabHandheld.h"

//------------------------------------------------------------------------------------------------------------
// Cab Handheld global objects, we just create them. The setup will be done once all are in place.
//
//------------------------------------------------------------------------------------------------------------
CDC::CdcConfigInfo  cfg = CDC::getConfigDefault( );

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values. Note that the ATmega version uses an I2C expander. The RPico version has all the UI
// elements directly connected. We will for now just have defines ( sigh ) to separate them throuout the
// code.
//
//----------------------------------------------------------------------------------------------------------
uint8_t setupConfigInfo( CDC::CdcConfigInfo *cfg ) {

  #if defined  ( __AVR_ATmega1284P__ )

  cfg -> READY_LED_PIN        = 19;
  cfg -> ACTIVE_LED_PIN       = 18;
  cfg -> BUTTON_PIN           = 31;

  cfg -> CAN_BUS_SELECT_PIN   = 0;
  cfg -> CAN_BUS_RX_PIN       = 0; // ??? must be zero, we use this argument for select Pin ... to be changed...
  cfg -> CAN_BUS_TX_PIN       = CDC::UNDEFINED_PIN;
  cfg -> CAN_BUS_CTRL_MODE    = CAN_BUS_LIB_MCP2515_500K_16MHZ;

  cfg -> NVM_SELECT_PIN       = 1;
  cfg -> NVM_CHIP_TYPE        = M25LC256;

  #elif defined ( ARDUINO_ARCH_RP2040 )

  cfg -> READY_LED_PIN        = 4;

  cfg -> I2C_SCL_PIN_0        = 17;
  cfg -> I2C_SDA_PIN_0        = 16;
  cfg -> I2C_ADR_0            = 0x3C;

  cfg -> I2C_SCL_PIN_1        = 3;
  cfg -> I2C_SDA_PIN_1        = 2;
  cfg -> I2C_ADR_1            = 0x50;

  cfg -> CAN_BUS_SELECT_PIN   = CDC::UNDEFINED_PIN;
  cfg -> CAN_BUS_RX_PIN       = 0;
  cfg -> CAN_BUS_TX_PIN       = 1;
  cfg -> CAN_BUS_CTRL_MODE    = CAN_BUS_LIB_PICO_PIO_500K_M_CORE;

  cfg -> NVM_CHIP_TYPE        = M24LC128;

  #else
#error CANNOT COMPILE - specify the Atmega1284 or PICO LCS main controller board pins
  #endif

  return ( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// "printStatus" is a little helper function for the initialization routines protocol printing. If there is
// a serial IO, these routines will list the success of the particular setup operation.
//
// ??? may be a bit of an overkill ?
//----------------------------------------------------------------------------------------------------------
uint8_t printStatus( uint8_t status ) {

  INTERFACE.print( F( "-> " ));
  if ( status == ALL_OK ) INTERFACE.println( F( "OK" ));
  else {

    INTERFACE.print ( F( "FAILED: " ));
    INTERFACE.println( status );
  }

  return ( status );
}

//------------------------------------------------------------------------------------------------------------
// Main program setup. We setup all elements and the just enter the LCS run loop. Note that the UI SCreens
// will have picked up the first screen and this is what is shown before we enter the LCS core library run
// loop. We are in business now.
//
//------------------------------------------------------------------------------------------------------------
void setup( ) {

  uint8_t rStat = ALL_OK;

  delay( 2000 );
  INTERFACE.begin( 115200 );
  delay( 100 );

  if ( rStat == ALL_OK ) {

    Serial.print( "Setup Config Info " );
    rStat = printStatus( setupConfigInfo( &cfg ));

    Serial.println( "Actual CDC Config " );
    CDC::printCfgInfo( &cfg );
  }

  if ( rStat == ALL_OK ) {

    INTERFACE.print( "Setup LCS Library " );
    rStat = printStatus( setupLcsLib( ));
  }

  if ( rStat == ALL_OK ) {

    INTERFACE.print( "Setup Msg Bus " );
    rStat = printStatus( setupMsgBus( ));
  }

  if ( rStat == ALL_OK ) {

    INTERFACE.print( "Setup UI Elements " );
    rStat = printStatus( setupUIElements( ));
  }

  if ( rStat == ALL_OK ) {

    INTERFACE.print( "Setup Screens " );
    rStat = printStatus( setupScreens( ));
  }

  if ( rStat == ALL_OK ) {

    INTERFACE.print( "Setup Cab Stack " );
    rStat = printStatus( setupCabStack ( ));
  }

  if ( rStat == ALL_OK ) {

    INTERFACE.println( "Ready..." );
    UIScreen::setup( );
    lcsLib -> run( );
  }
}

//------------------------------------------------------------------------------------------------------------
// The Arduino loop routine. We never come here.
//
//------------------------------------------------------------------------------------------------------------
void loop( ) {
  
}
#endif