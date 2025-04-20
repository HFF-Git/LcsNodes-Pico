//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Test Program
//
//------------------------------------------------------------------------------------------------------------
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

using namespace CDC;

CdcResourceMap cMap = {

    .cFamily                = CDC_CF_RP_PICO,
    .cType                  = CDC_CF_C_RP_2040,
    .memorySize             = 260 * 1024,

    .adcRefVoltageMillis    = 3300,
    .adcDigitRange          = 1024,

    .ledPin                 = 15,
    .pFailPin               = 7,
    
    .adcPin_0               = 26,
    .adcPin_1               = 27,

    .dioPin_0               = 8,
    .dioPin_1               = 9,
    .dioPin_2               = 10,
    .dioPin_3               = 11,
    .dioPin_4               = 21,
    .dioPin_5               = 20,
    .dioPin_6               = 19,
    .dioPin_7               = 18,

    .pwmPin_0               = 20,
    .pwmPin_1               = 21,
  
    .i2cSclPin_0            = 3,
    .i2cSdaPin_0            = 2,

    .i2cSclPin_1            = 17,
    .i2cSdaPin_1            = 16,
};

//----------------------------------------------------------------------------------------------------------
// Init the library...
//
//----------------------------------------------------------------------------------------------------------
void initCdcLib( ) {

    cdcInit( &cMap );
    configureConsoleIO( );
    sleepMillis( 2000 );
    printf( "Test LCS Controller dependent code library\n" );
}

//----------------------------------------------------------------------------------------------------------
// test the console IO.
//
// PIN 25 is the Led on the PICO board itself.
//----------------------------------------------------------------------------------------------------------
void testConsoleIO ( ) {

  configureDio( cMap.ledPin, CDC_DIO_OUT );
  writeDio( cMap.ledPin, true );
  sleepMillis( 1000 );

  printf( "Test Console IO..\n" );
  printf( "USB is connected: %d\n", CDC::isConsoleConnected( ));

  printf( "->" );

  while ( true ) {

    char c = getConsoleChar( );
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

  configureDio( cMap.pFailPin, CDC_DIO_IN );
  registerDioCallback( cMap.pFailPin, CDC_EVT_LOW, pfailCallback );
  
  configureDio( cMap.ledPin, CDC_DIO_OUT );
  writeDio( cMap.ledPin, true );
  
  printf( "testPfail -> unplug the power cord \n" );
}

//----------------------------------------------------------------------------------------------------------
// Test the onboard LEDs.
//
//----------------------------------------------------------------------------------------------------------
void testLeds( ) {

  printf( "Active Led Test\n" );
  configureDio( cMap.ledPin, CDC_DIO_OUT );
  
  while ( true ) {

    writeDio( cMap.ledPin, true );
    sleepMillis( 500 );
    
    writeDio( cMap.ledPin, false );
    sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test Fatal Error LED. Note, we will not come back from this call.
//
//----------------------------------------------------------------------------------------------------------
void testFatalErr( ) {

  printf( "Fatal Error Test\n" );
  fatalError( 4 );
}

//----------------------------------------------------------------------------------------------------------
// Set the DIO pins to input, pull-up and read the values. Use a little cable to set the voltage on the
// extension connector.
//
//----------------------------------------------------------------------------------------------------------
void testDioInput( ) {

    printf( "DIO input test\n" );

    configureDio( cMap.ledPin, CDC_DIO_OUT );
    configureDio( cMap.dioPin_0, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_1, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_2, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_3, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_4, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_5, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_6, CDC_DIO_IN_PULLUP );
    configureDio( cMap.dioPin_7, CDC_DIO_IN_PULLUP );
   
    while ( true ) {

        sleepMillis( 1000 );
        toggleDio( cMap.ledPin );

        bool val;
        uint8_t rStat;

        rStat = readDio( cMap.dioPin_0, &val );
        printf( "Econ Dio In 0: %d\n", val );

        rStat = readDio( cMap.dioPin_1, &val );
        printf( "Econ Dio In 1: %d\n", val );

        rStat = readDio( cMap.dioPin_2, &val );
        printf( "Econ Dio In 2: %d\n", val );

        rStat = readDio( cMap.dioPin_3, &val );
        printf( "Econ Dio In 3: %d\n", val );

        rStat = readDio( cMap.dioPin_4, &val );
        printf( "Econ Dio In 4: %d\n", val );

        rStat = readDio( cMap.dioPin_5, &val );
        printf( "Econ Dio In 5: %d\n", val );

        rStat = readDio( cMap.dioPin_6, &val );
        printf( "Econ Dio In 6: %d\n", val );

        rStat = readDio( cMap.dioPin_7, &val );
        printf( "Econ Dio In 7: %d\n", val );
    }
}

//----------------------------------------------------------------------------------------------------------
// Set the DIO pins to output and periodically toggle the values. Use a an LED array and connect the pins of
// the extension connector to it. The toggle Led just indicates that the board is basically working.
//
//----------------------------------------------------------------------------------------------------------
void testDioOutput( ) {

    printf( "DIO output test\n" );

    configureDio( cMap.ledPin, CDC_DIO_OUT );
    configureDio( cMap.dioPin_0, CDC_DIO_OUT );
    configureDio( cMap.dioPin_1, CDC_DIO_OUT );
    configureDio( cMap.dioPin_2, CDC_DIO_OUT );
    configureDio( cMap.dioPin_3, CDC_DIO_OUT );
    configureDio( cMap.dioPin_4, CDC_DIO_OUT );
    configureDio( cMap.dioPin_5, CDC_DIO_OUT );
    configureDio( cMap.dioPin_6, CDC_DIO_OUT );
    configureDio( cMap.dioPin_7, CDC_DIO_OUT );

    while ( true ) {

        toggleDio( cMap.ledPin);
        writeDio( cMap.dioPin_0, false );
        writeDio( cMap.dioPin_1, false );
        writeDio( cMap.dioPin_2, false );
        writeDio( cMap.dioPin_3, false );
        writeDio( cMap.dioPin_4, false );
        writeDio( cMap.dioPin_5, false );
        writeDio( cMap.dioPin_6, false );
        writeDio( cMap.dioPin_7, false );
        sleepMillis( 1000 );

        writeDio( cMap.dioPin_0, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_1, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_2, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_3, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_4, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_5, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_6, true );
        sleepMillis( 500 );
        writeDio( cMap.dioPin_7, true );
        sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the DIO pin pairs. We use the first four resource IDs and pass two pins at configuration time.
//
//----------------------------------------------------------------------------------------------------------
void testDioOutputPair( ) {

  printf( "DIO output pair test\n" );

  configureDio( cMap.ledPin, CDC_DIO_OUT );
  configureDio( cMap.dioPin_0, CDC_DIO_OUT );
  configureDio( cMap.dioPin_1, CDC_DIO_OUT );
  configureDio( cMap.dioPin_2, CDC_DIO_OUT );
  configureDio( cMap.dioPin_3, CDC_DIO_OUT );
  configureDio( cMap.dioPin_4, CDC_DIO_OUT );
  configureDio( cMap.dioPin_5, CDC_DIO_OUT );
  configureDio( cMap.dioPin_6, CDC_DIO_OUT );
  configureDio( cMap.dioPin_7, CDC_DIO_OUT );
  
  while ( true ) {

    toggleDio( cMap.ledPin );

    writeDioPair( cMap.dioPin_0, false, cMap.dioPin_1, false );
    writeDioPair( cMap.dioPin_2, false, cMap.dioPin_3, false );
    writeDioPair( cMap.dioPin_4, false, cMap.dioPin_5, false );   
    writeDioPair( cMap.dioPin_6, false, cMap.dioPin_7, false );
    sleepMillis( 1000 );

    writeDioPair( cMap.dioPin_0, true, cMap.dioPin_1, false );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_0, false, cMap.dioPin_1, true );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_0, true, cMap.dioPin_1, true );
    sleepMillis( 500 );

    writeDioPair( cMap.dioPin_2, true, cMap.dioPin_3, false );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_2, false, cMap.dioPin_3, true );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_2, true, cMap.dioPin_3, true );
    sleepMillis( 500 );

    writeDioPair( cMap.dioPin_4, true, cMap.dioPin_5, false );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_4, false, cMap.dioPin_5, true );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_4, true, cMap.dioPin_5, true );
    sleepMillis( 500 );

    writeDioPair( cMap.dioPin_6, true, cMap.dioPin_7, false );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_6, false, cMap.dioPin_7, true );
    sleepMillis( 500 );
    writeDioPair( cMap.dioPin_6, true, cMap.dioPin_7, true );
    sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the analog blocking read.
//
//----------------------------------------------------------------------------------------------------------
void testAdcBlockingRead( ) {

  float digitToVolt = (float) 3300 / 1024 / 1000; // quick hack ...

  printf( "ADC read test\n" );

  configureDio( cMap.ledPin, CDC_DIO_OUT );
  configureAdc( cMap.adcPin_0 );
  configureAdc( cMap.adcPin_1 );
  
  while ( true ) {

    uint16_t    val;
    uint8_t     rStat = readAdc( cMap.adcPin_0, &val );

    printf( "ADC -> ( pin: %d, val: %d, Volt: %d )\n", cMap.adcPin_0, val, val * digitToVolt );
    sleepMillis( 1000 );

    rStat = readAdc( cMap.adcPin_1, &val );

    printf( "ADC -> ( pin: %d, val: %d, Volt: %d )\n", cMap.adcPin_1, val, val * digitToVolt );
    sleepMillis( 1000 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the timer interrupt. The callback functions are invoked an we display the that the timer fired. 
//
//----------------------------------------------------------------------------------------------------------
void timerCallback0( uint32_t timerVal ) {

    printf( "Timer 0 fired: %d\n", getMillis( ));
}

void timerCallback1( uint32_t timerVal ) {

    printf( "Timer 1 fired: %d\n", getMillis( ));
}

void testTimer( ) {

    printf( "Timer test\n" );

    configureDio( cMap.ledPin, CDC_DIO_OUT );

    uint8_t rStat = NO_ERR;

    rStat = configureTimer( 100, timerCallback0 );
    rStat = configureTimer( 200, timerCallback1 );

    writeDio( cMap.ledPin, true );

    startRepeatingTimer( 100, 500000 );
    startRepeatingTimer( 100, 250000 );

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
// "testPWMFixed" tests the PWM functionality of the DIO pins 6 and 7. We will just configure the two ports,
// set the frequency and three values to see of the duty cycle changes. Best to see on an Oscilloscope.
//
//----------------------------------------------------------------------------------------------------------
void testPWMFixed( ) {

  printf( "PWM fixed frequency test\n" );

  uint32_t fPWM  = 100;
  configureDio( cMap.ledPin, CDC_DIO_OUT );

  configurePwm( cMap.pwmPin_0, fPWM );
  configurePwm( cMap.pwmPin_1, fPWM );

    while ( true ) {

        toggleDio( cMap.ledPin );
    
        writePwmPair( cMap.pwmPin_0, 127, 63 );
        sleepMillis( 2000 );

        writePwmPair( cMap.pwmPin_0, 192, 127 );
        sleepMillis( 2000 );

        writePwmPair( cMap.pwmPin_0, 63, 192 );
        sleepMillis( 2000 );
    }
}

//----------------------------------------------------------------------------------------------------------
// "testPWMWithAnalogInput" will read in an analog value and use it as a dutyCycle for the PWM outputs. We
// use a frequency of 100Hz, which is nicely to see on an Oscilloscope with a period length of 10ms. The
// analog input is a bit noisy, so we ignore anything very small values.
//
//----------------------------------------------------------------------------------------------------------
void testPWMWithAnalogInput( ) {

    printf( "PWM with analog input test\n" );

    configureDio( cMap.ledPin, CDC_DIO_OUT );

    uint32_t fPWM               = 100;
    uint16_t dutyCycle          = 0;
    uint16_t minimalThreshold   = 6;
    uint8_t  rStat              = NO_ERR;
   
    rStat = configureAdc( cMap.adcPin_0 );
    rStat = configurePwm( cMap.pwmPin_0, fPWM );
  
    while ( true ) {

        toggleDio( cMap.ledPin );

        rStat = readAdc( cMap.adcPin_0, &dutyCycle );

        if ( dutyCycle < minimalThreshold ) dutyCycle = 0;
        if ( dutyCycle > 255 )              dutyCycle = 255;

        writePwm( cMap.pwmPin_0, dutyCycle );
        sleepMillis( 100 );
  }
}

//----------------------------------------------------------------------------------------------------------
// "testUIDGen" test the UID generation code.
//
//----------------------------------------------------------------------------------------------------------
void testUIDGen( ) {

    printf( "UID generation test\n" );
    sleepMillis( 1000 );

    for ( int i = 0; i < 20; i++ ) {

        printf( "UID -> %d\n ", createUid( ));
        sleepMillis( 100 );
    }
}

//----------------------------------------------------------------------------------------------------------
// Main. A bit crude. Just enable what you want to test ...
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

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

    return( 0 );
}