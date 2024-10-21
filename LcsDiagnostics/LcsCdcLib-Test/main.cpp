//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Test Program
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the the RP2040 controller family hardware library code. The idea of this library
// is to shield the actual hardware of processor and board implementation from the upper layers but still keep
// the flexibility and performance of the underlying hardware. 
//
// This is a little test program for the individual functions of the CDC layer. It is a rather crude program
// and you need to recompile it for each test of a portion of the library.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
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

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg;

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
// Init the library...
//
//----------------------------------------------------------------------------------------------------------
void initCdcLib( ) {

  CDC::sleepMillis( 2000 );

  printf( "Test LCS Controller dependent code library\n" );

  setupConfigInfo( );
 
  int rStat = CDC::init( &cfg );
  
  CDC::printConfigInfo( &cfg );

  if ( rStat != 0 ) printf( "Err code: %d\n", rStat );
  else printf( "OK\n" );
}

//----------------------------------------------------------------------------------------------------------
// test the console IO.
//
//----------------------------------------------------------------------------------------------------------
void testConsoleIO ( ) {

  CDC::configureDio( 25, CDC::OUT );
  CDC::writeDio( 25, true );
  CDC::sleepMillis( 1000 );

  printf( "Test Console IO..\n" );
  printf( "USB is connected: %d\n", CDC::isConsoleConnected( ));

  printf( "->" );

  while ( true ) {

    char c = CDC::getConsoleChar( );
    if ( c != 0 ) {

      if ( c == 'q' ) break;
      
      printf( "%c\n", c );
      
       printf( "USB is connected: %d\n", CDC::isConsoleConnected( ));
       printf( "->" );
    }
  }

  printf( "terminated ... \n" );
}

//----------------------------------------------------------------------------------------------------------
// test the power failure option.
//
//----------------------------------------------------------------------------------------------------------
void pfailCallback( uint8_t pin, uint8_t event ) {

  printf( "PFAIL..\n" );
  while ( true )  printf( "%d\n", CDC::getMillis( ));
}

void testPfail( ) {

  CDC::configureDio( cfg.PFAIL_PIN, CDC::IN );
  CDC::registerDioCallback( cfg.PFAIL_PIN, CDC::EVT_LOW, pfailCallback );
  
  CDC::configureDio( cfg.READY_LED_PIN, CDC::OUT );
  CDC::writeDio( cfg.READY_LED_PIN, true );
  
  printf( "testPfail -> unplug the power cord \n" );
}

//----------------------------------------------------------------------------------------------------------
// Test the onboard LEDs.
//
//----------------------------------------------------------------------------------------------------------
void testLeds( ) {

  printf( "testLeds\n" );

  CDC::configureDio( cfg.READY_LED_PIN, CDC::OUT );
  CDC::configureDio( cfg.ACTIVE_LED_PIN, CDC::OUT );

  while ( true ) {

    CDC::writeDio( cfg.READY_LED_PIN, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.ACTIVE_LED_PIN, true );
    CDC::sleepMillis( 500 );

    CDC::writeDio( cfg.READY_LED_PIN, false );
    CDC::writeDio( cfg.ACTIVE_LED_PIN, false );
    CDC::sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test Fatal Error LED. Note, we will not come back from this call.
//
//----------------------------------------------------------------------------------------------------------
void testFatalErr( ) {

  printf( "Fatal Error Test\n" );

  CDC::fatalError( 4 );
}

//----------------------------------------------------------------------------------------------------------
// Set the DIO pins to input, pull-up and read the values. Use a little cable to set the voltage on the
// extension connector.
//
//----------------------------------------------------------------------------------------------------------
void testDioInput( ) {

  printf( "testDioInput\n" );

  CDC::configureDio( cfg.ACTIVE_LED_PIN, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_0, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_1, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_2, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_3, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_4, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_5, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_6, CDC::IN_PULLUP );
  CDC::configureDio( cfg.DIO_PIN_7, CDC::IN_PULLUP );

  while ( true ) {

    CDC::sleepMillis( 1000 );
    CDC::toggleDio( cfg.ACTIVE_LED_PIN );

    printf( "Econ Dio In 0: %d\n", CDC::readDio( cfg.DIO_PIN_0 ));
    printf( "Econ Dio In 1: %d\n", CDC::readDio( cfg.DIO_PIN_1 ));
    printf( "Econ Dio In 2: %d\n", CDC::readDio( cfg.DIO_PIN_2 ));
    printf( "Econ Dio In 3: %d\n", CDC::readDio( cfg.DIO_PIN_3 ));
    printf( "Econ Dio In 4: %d\n", CDC::readDio( cfg.DIO_PIN_4 ));
    printf( "Econ Dio In 5: %d\n", CDC::readDio( cfg.DIO_PIN_5 ));
    printf( "Econ Dio In 6: %d\n", CDC::readDio( cfg.DIO_PIN_6 ));
    printf( "Econ Dio In 7: %d\n", CDC::readDio( cfg.DIO_PIN_7 ));
  }
}

//----------------------------------------------------------------------------------------------------------
// Set the DIO pins to output and periodically toggle the values. Use a an LED array and connect the pins of
// the extension connector to it. The toggle Led just indicates that the board is basically working.
//
//----------------------------------------------------------------------------------------------------------
void testDioOutput( ) {

  printf( "testDioOutput\n" );

  CDC::configureDio( cfg.ACTIVE_LED_PIN, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_0, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_1, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_2, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_3, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_4, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_5, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_6, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_7, CDC::OUT );

  while ( true ) {

    CDC::toggleDio( cfg.ACTIVE_LED_PIN );
    CDC::writeDio( cfg.DIO_PIN_0, false );
    CDC::writeDio( cfg.DIO_PIN_1, false );
    CDC::writeDio( cfg.DIO_PIN_2, false );
    CDC::writeDio( cfg.DIO_PIN_3, false );
    CDC::writeDio( cfg.DIO_PIN_4, false );
    CDC::writeDio( cfg.DIO_PIN_5, false );
    CDC::writeDio( cfg.DIO_PIN_6, false );
    CDC::writeDio( cfg.DIO_PIN_7, false );
    CDC::sleepMillis( 1000 );

    CDC::writeDio( cfg.DIO_PIN_0, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_1, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_2, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_3, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_4, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_5, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_6, true );
    CDC::sleepMillis( 500 );
    CDC::writeDio( cfg.DIO_PIN_7, true );
    CDC::sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the DIO pin pairs. Use a LED array and connect the pins of the extension connector to it. You
// should see a binary counting up. The toggle Led just indicates that the board is basically working.
//
//----------------------------------------------------------------------------------------------------------
void testDioOutputPair( ) {

  printf( "testDioOutputPair\n" );

  CDC::configureDio( cfg.ACTIVE_LED_PIN, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_0, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_1, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_2, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_3, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_4, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_5, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_6, CDC::OUT );
  CDC::configureDio( cfg.DIO_PIN_7, CDC::OUT );

  while ( true ) {

    CDC::toggleDio( cfg.ACTIVE_LED_PIN );

    CDC::writeDio( cfg.DIO_PIN_0, false );
    CDC::writeDio( cfg.DIO_PIN_1, false );
    CDC::writeDio( cfg.DIO_PIN_2, false );
    CDC::writeDio( cfg.DIO_PIN_3, false );
    CDC::writeDio( cfg.DIO_PIN_4, false );
    CDC::writeDio( cfg.DIO_PIN_5, false );
    CDC::writeDio( cfg.DIO_PIN_6, false );
    CDC::writeDio( cfg.DIO_PIN_7, false );
    CDC::sleepMillis( 1000 );

    CDC::writeDioPair( cfg.DIO_PIN_0, true, cfg.DIO_PIN_1, false );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_0, false, cfg.DIO_PIN_1, true );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_0, true, cfg.DIO_PIN_1, true );
    CDC::sleepMillis( 500 );

    CDC::writeDioPair( cfg.DIO_PIN_2, true, cfg.DIO_PIN_3, false );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_2, false, cfg.DIO_PIN_3, true );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_2, true, cfg.DIO_PIN_3, true );
    CDC::sleepMillis( 500 );

    CDC::writeDioPair( cfg.DIO_PIN_4, true, cfg.DIO_PIN_5, false );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_4, false, cfg.DIO_PIN_5, true );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_4, true, cfg.DIO_PIN_5, true );
    CDC::sleepMillis( 500 );

    CDC::writeDioPair( cfg.DIO_PIN_6, true, cfg.DIO_PIN_7, false );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_6, false, cfg.DIO_PIN_7, true );
    CDC::sleepMillis( 500 );
    CDC::writeDioPair( cfg.DIO_PIN_6, true, cfg.DIO_PIN_7, true );
    CDC::sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the analog blocking read.
//
//----------------------------------------------------------------------------------------------------------
void testAdcBlockingRead( ) {

  float digitToVolt = (float) CDC::getAdcRefVoltage( ) / (float) CDC::getAdcDigitRange( ) / 1000;

  printf( "adcBlockingRead\n" );

  CDC::configureAdc( cfg.ADC_PIN_0 );
  CDC::configureAdc( cfg.ADC_PIN_1 );

  uint16_t val;

  while ( true ) {

    val = CDC::readAdc( cfg.ADC_PIN_0 );
    printf( "ADC -> (%d:%d:%d)\n", cfg.ADC_PIN_0, val, val * digitToVolt );
    CDC::sleepMillis( 1000 );

    val = CDC::readAdc( cfg.ADC_PIN_1 );
    printf( "ADC -> (%d:%d:%d)\n", cfg.ADC_PIN_1, val, val * digitToVolt );
    CDC::sleepMillis( 1000 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the timer interrupt. The callback function is invoked an we display the that the timer fired. The
// toggle Led just indicates that the board is basically working.
//
//----------------------------------------------------------------------------------------------------------
void timerCallback( uint32_t timerVal) {

  printf( "Timer fired: %d\n", CDC::getMillis( ));
}

void testTimer( ) {

  printf( "testTimer\n" );
  CDC::onTimerEvent( timerCallback );
  CDC::startRepeatingTimer( 500000 );

  while ( true ) { }
}

//----------------------------------------------------------------------------------------------------------
// "testI2C" uses the I2C bus. Currently, there is no real test. The I2C routines are used by the runtime
// library. SInce we have a HW setup for NVM read and write, might as well debug I2C it there.
//
//----------------------------------------------------------------------------------------------------------
void testI2C( ) {

}

//----------------------------------------------------------------------------------------------------------
// "testSPI" uses the SPI bus. Currently, there is no project that uses the SPI code. To be debugged when 
// we have such a case.
//
//----------------------------------------------------------------------------------------------------------
void testSPI( ) {

}

//----------------------------------------------------------------------------------------------------------
// "testPWMFixed" tests the PWM functionality of the DIO pins 6 and 7. We will just configure the two ports,
// set the frequency and three values to see of the duty cycle changes. Best to see on an Oscilloscope.
//
//----------------------------------------------------------------------------------------------------------
void testPWMFixed( ) {

  printf( "testPWMFixed\n" );

  uint32_t fPWM = 100;

  CDC::configurePwm( cfg.DIO_PIN_6, fPWM, true, false );
  CDC::configurePwm( cfg.DIO_PIN_7, fPWM, true, false );

  while ( true ) {

    CDC::writePwm( cfg.DIO_PIN_6, 127 );
    CDC::writePwm( cfg.DIO_PIN_7, 63 );
    CDC::sleepMillis( 2000 );

    CDC::writePwm( cfg.DIO_PIN_6, 192 );
    CDC::writePwm( cfg.DIO_PIN_7, 127 );
    CDC::sleepMillis( 2000 );

    CDC::writePwm( cfg.DIO_PIN_6, 63 );
    CDC::writePwm( cfg.DIO_PIN_7, 192 );
    CDC::sleepMillis( 2000 );
  }
}

//----------------------------------------------------------------------------------------------------------
// "testPWMWithAnalogInput" will read in an analog value and use it as a dutyCycle for the PWM outputs. We
// use a frequency of 100Hz, which is nicely to see on an Oscilloscope with a period length of 10ms. The
// analog input is a bit noisy, so we ignore anything very small values.
//
//----------------------------------------------------------------------------------------------------------
void testPWMWithAnalogInput( ) {

  printf( "testPWMWithAnalogInput\n" );

  uint32_t fPWM             = 100;
  uint16_t dutyCycle        = 0;
  uint16_t minimalThreshold = 6;

  CDC::configureAdc( cfg.ADC_PIN_0 );
  CDC::configurePwm( cfg.DIO_PIN_6, fPWM, true, false );
  CDC::configurePwm( cfg.DIO_PIN_7, fPWM, true, false );

  while ( true ) {

    dutyCycle = CDC::readAdc( cfg.ADC_PIN_0 );

    if ( dutyCycle < minimalThreshold ) dutyCycle = 0;
    if ( dutyCycle > 255 )              dutyCycle = 255;

    CDC::writePwm( cfg.DIO_PIN_6, dutyCycle );
    CDC::writePwm( cfg.DIO_PIN_7, dutyCycle );
    CDC::sleepMillis( 100 );
  }
}

//----------------------------------------------------------------------------------------------------------
// "testUIDGen" test the UID generation code.
//
//----------------------------------------------------------------------------------------------------------
void testUIDGen( ) {

  printf( "UID generation test\n" );
  CDC::sleepMillis( 1000 );

  for ( int i = 0; i < 20; i++ ) {

    printf( "UID -> %d\n ", CDC::createUid( ));
    CDC::sleepMillis( 100 );
  }
}

//----------------------------------------------------------------------------------------------------------
// "testRepeatingTimerWithDio" test the repeating timer ands emits an alternating signal to DIO_PIN_0.
//
//----------------------------------------------------------------------------------------------------------
void timerRepeatingCallback( uint32_t timerVal ) {

  CDC::setRepeatingTimerLimit(( CDC::getRepeatingTimerLimit( ) == 58 ) ? 116 : 58 );
  CDC::toggleDio( cfg.DIO_PIN_0 );
}

void testRepeatingTimerWithDio( ) {

  printf( "testRepeatingTimerWithDio\n" );

  CDC::onTimerEvent( timerRepeatingCallback );
  CDC::configureDio( cfg.DIO_PIN_0, CDC::OUT );
  CDC::startRepeatingTimer( 58 );

  while ( true ) { }
}

//----------------------------------------------------------------------------------------------------------
// The crude test setup. The idea is to enable the respective test, compile, load and debug and go on to
//  the next test.
//
//----------------------------------------------------------------------------------------------------------
void testCases( ) {

  initCdcLib( );

  // testFatalErr( );
  testConsoleIO( );
  // testPfail( );
  // testLeds( );
  // testDioInput( );
  // testDioOutput( );
  // testDioOutputPair( );
  // testAdcBlockingRead( );
  // testTimer( );
  // testI2C( );
  // testSPI( );
  // testPWMFixed( );
  // testPWMWithAnalogInput( );
  // testUIDGen( );
  // testRepeatingTimerWithDio( );
}

//----------------------------------------------------------------------------------------------------------
// Here we go ...
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

  CDC::configureConsoleIO( );

  fprintf( stdout, "LCS CDC Library Test Program...\n\n" );      
   
  testCases( );
  return( 0 );
}