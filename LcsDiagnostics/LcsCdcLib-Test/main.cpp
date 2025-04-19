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
#include "LcsBoardGenericCdcLib.h"

using namespace CDC;

//------------------------------------------------------------------------------------------------------------
//
// ??? quick hack to get the new CDC lib going ...
//------------------------------------------------------------------------------------------------------------
enum CdcResIdList : uint8_t {

    CDC_RES_ID_PFAIL        = 1,
    CDC_RES_ID_LED          = 2,
    CDC_RES_ID_EXT_INT      = 3,
    CDC_RES_ID_ADC_0        = 4,
    CDC_RES_ID_ADC_1        = 5,
    CDC_RES_ID_PWM_0        = 6,
    CDC_RES_ID_PWM_1        = 7,
    
    CDC_RES_ID_DIO_0        = 10,
    CDC_RES_ID_DIO_1        = 11,
    CDC_RES_ID_DIO_2        = 12,
    CDC_RES_ID_DIO_3        = 13,
    CDC_RES_ID_DIO_4        = 14,
    CDC_RES_ID_DIO_5        = 15,
    CDC_RES_ID_DIO_6        = 16,
    CDC_RES_ID_DIO_7        = 17,
    
    CDC_RES_ID_NVM_I2C      = 20,
    CDC_RES_ID_EXT_I2C      = 21,
    CDC_RES_ID_PICO_LED     = 22,
    CDC_RES_ID_TIMER_0      = 23,
    CDC_RES_ID_TIMER_1      = 24
};

enum CdcPins : uint8_t {

    CDC_PIN_PICO_LED        = 25,

    CDC_PIN_PFAIL           = 7,
    CDC_PIN_LED             = 15,
    CDC_PIN_EXT_INT         = 22,
   
    CDC_PIN_ADC_0           = 26,
    CDC_PIN_ADC_1           = 27,

    CDC_PIN_PWM_0           = 20,
    CDC_PIN_PWM_1           = 21,
    
    CDC_PIN_DIO_0           = 8,
    CDC_PIN_DIO_1           = 9,
    CDC_PIN_DIO_2           = 10,
    CDC_PIN_DIO_3           = 11,
    CDC_PIN_DIO_4           = 21,
    CDC_PIN_DIO_5           = 20,
    CDC_PIN_DIO_6           = 19,
    CDC_PIN_DIO_7           = 18,
    
    CDC_PIN_NVM_I2C_SCL     = 3,
    CDC_PIN_NVM_I2C_SDA     = 2,

    CDC_PIN_EXT_I2C_SCL     = 17,
    CDC_PIN_EXT_I2C_SDA     = 16
};

//----------------------------------------------------------------------------------------------------------
// Init the library...
//
//----------------------------------------------------------------------------------------------------------
void initCdcLib( ) {

    cdcInit( );
    configureConsoleIO( );

    configureController(    CDC_CF_RP_PICO, 
                            CDC_CF_C_RP_2040, 
                            260 * 1024,
                            0, 
                            2000, 
                            3300, 
                            1024, 
                            CDC_PIN_LED, 
                            CDC_PIN_PFAIL );

    sleepMillis( 2000 );
    printf( "Test LCS Controller dependent code library\n" );
}

//----------------------------------------------------------------------------------------------------------
// test the console IO.
//
// PIN 25 is the Led on the PICO board itself.
//----------------------------------------------------------------------------------------------------------
void testConsoleIO ( ) {

  configureDio( CDC_RES_ID_PICO_LED, CDC_PIN_PICO_LED, UNDEFINED_PIN, CDC_DIO_OUT );
  writeDio( CDC_RES_ID_PICO_LED, true );
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

  configureDio( CDC_RES_ID_PFAIL,  CDC_PIN_PFAIL, UNDEFINED_PIN, CDC_DIO_IN );
  registerDioCallback( CDC_RES_ID_PFAIL, CDC_EVT_LOW, pfailCallback );
  
  configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );
  writeDio( CDC_RES_ID_LED, true );
  
  printf( "testPfail -> unplug the power cord \n" );
}

//----------------------------------------------------------------------------------------------------------
// Test the onboard LEDs.
//
//----------------------------------------------------------------------------------------------------------
void testLeds( ) {

  printf( "Active Led Test\n" );
  configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );
  
  while ( true ) {

    writeDio( CDC_RES_ID_LED, true );
    sleepMillis( 500 );
    
    writeDio( CDC_RES_ID_LED, false );
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

    configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_0, CDC_PIN_DIO_0, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_1, CDC_PIN_DIO_1, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_2, CDC_PIN_DIO_2, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_3, CDC_PIN_DIO_3, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_4, CDC_PIN_DIO_4, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_5, CDC_PIN_DIO_5, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_6, CDC_PIN_DIO_6, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
    configureDio( CDC_RES_ID_DIO_7, CDC_PIN_DIO_7, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );

    while ( true ) {

        sleepMillis( 1000 );
        toggleDio( CDC_RES_ID_LED );

        bool val;
        uint8_t rStat;

        rStat = readDio( CDC_RES_ID_DIO_0, &val );
        printf( "Econ Dio In 0: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_1, &val );
        printf( "Econ Dio In 1: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_2, &val );
        printf( "Econ Dio In 2: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_3, &val );
        printf( "Econ Dio In 3: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_4, &val );
        printf( "Econ Dio In 4: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_5, &val );
        printf( "Econ Dio In 5: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_6, &val );
        printf( "Econ Dio In 6: %d\n", val );

        rStat = readDio( CDC_RES_ID_DIO_7, &val );
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

    configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_0, CDC_PIN_DIO_0, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_1, CDC_PIN_DIO_1, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_2, CDC_PIN_DIO_2, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_3, CDC_PIN_DIO_3, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_4, CDC_PIN_DIO_4, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_5, CDC_PIN_DIO_5, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_6, CDC_PIN_DIO_6, UNDEFINED_PIN, CDC_DIO_OUT );
    configureDio( CDC_RES_ID_DIO_7, CDC_PIN_DIO_7, UNDEFINED_PIN, CDC_DIO_OUT );

    while ( true ) {

        toggleDio( CDC_RES_ID_LED );
        writeDio( CDC_RES_ID_DIO_0, false );
        writeDio( CDC_RES_ID_DIO_1, false );
        writeDio( CDC_RES_ID_DIO_2, false );
        writeDio( CDC_RES_ID_DIO_3, false );
        writeDio( CDC_RES_ID_DIO_4, false );
        writeDio( CDC_RES_ID_DIO_5, false );
        writeDio( CDC_RES_ID_DIO_6, false );
        writeDio( CDC_RES_ID_DIO_7, false );
        sleepMillis( 1000 );

        writeDio( CDC_RES_ID_DIO_0, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_1, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_2, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_3, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_4, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_5, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_6, true );
        sleepMillis( 500 );
        writeDio( CDC_RES_ID_DIO_7, true );
        sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the DIO pin pairs. We use the first four resource IDs and pass two pins at configuration time.
//
//----------------------------------------------------------------------------------------------------------
void testDioOutputPair( ) {

  printf( "DIO output pair test\n" );

  configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );
  configureDio( CDC_RES_ID_DIO_0, CDC_PIN_DIO_0, CDC_PIN_DIO_1, CDC_DIO_OUT );
  configureDio( CDC_RES_ID_DIO_1, CDC_PIN_DIO_2, CDC_PIN_DIO_3, CDC_DIO_OUT );
  configureDio( CDC_RES_ID_DIO_2, CDC_PIN_DIO_4, CDC_PIN_DIO_5, CDC_DIO_OUT );
  configureDio( CDC_RES_ID_DIO_3, CDC_PIN_DIO_6, CDC_PIN_DIO_7, CDC_DIO_OUT );

  while ( true ) {

    toggleDio( CDC_RES_ID_LED );

    writeDio( CDC_RES_ID_DIO_0, false, false );
    writeDio( CDC_RES_ID_DIO_1, false, false );
    writeDio( CDC_RES_ID_DIO_2, false, false );
    writeDio( CDC_RES_ID_DIO_3, false, false );
    sleepMillis( 1000 );

    writeDio( CDC_RES_ID_DIO_0, true, false );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_0, false, true );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_0, true, true );
    sleepMillis( 500 );

    writeDio( CDC_RES_ID_DIO_1, true, false );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_1, false, true );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_1, true, true );
    sleepMillis( 500 );

    writeDio( CDC_RES_ID_DIO_2, true, false );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_2, false, true );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_2, true, true );
    sleepMillis( 500 );

    writeDio( CDC_RES_ID_DIO_3, true, false );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_3, false, true );
    sleepMillis( 500 );
    writeDio( CDC_RES_ID_DIO_3, true, true );
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

  configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );
  configureAdc( CDC_RES_ID_ADC_0, CDC_PIN_ADC_0 );
  configureAdc( CDC_RES_ID_ADC_1, CDC_PIN_ADC_1 );
  
  while ( true ) {

    uint16_t    val;
    uint8_t     rStat = readAdc( CDC_RES_ID_ADC_0, &val );

    printf( "ADC -> ( resId: %d, val: %d, Volt: %d )\n", CDC_RES_ID_ADC_0, val, val * digitToVolt );
    sleepMillis( 1000 );

    rStat = readAdc( CDC_RES_ID_ADC_1, &val );

    printf( "ADC -> ( resId: %d, val: %d, Volt: %d )\n", CDC_RES_ID_ADC_1, val, val * digitToVolt );
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

    uint8_t rStat = configureDio( CDC_RES_ID_LED, CDC_PIN_LED, UNDEFINED_PIN, CDC_DIO_OUT );

    rStat = configureTimer( CDC_RES_ID_TIMER_0, timerCallback0 );
    rStat = configureTimer( CDC_RES_ID_TIMER_1, timerCallback1 );

    writeDio( CDC_RES_ID_LED, true );

    startRepeatingTimer( CDC_RES_ID_TIMER_0, 500000 );
    startRepeatingTimer( CDC_RES_ID_TIMER_1, 250000 );

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
  uint8_t  rStat = configureDio( CDC_RES_ID_LED, CDC_PIN_LED, 0, CDC_DIO_OUT );

  configurePwm( CDC_RES_ID_PWM_0, CDC_PIN_PWM_0, UNDEFINED_PIN, fPWM );

    while ( true ) {

        toggleDio( CDC_RES_ID_LED );
    
        writePwm( CDC_RES_ID_PWM_0, 127, 63 );
        sleepMillis( 2000 );

        writePwm( CDC_RES_ID_PWM_0, 192, 127 );
        sleepMillis( 2000 );

        writePwm( CDC_RES_ID_PWM_0, 63, 192 );
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

    uint8_t  rStat = configureDio( CDC_RES_ID_LED, CDC_PIN_LED, 0, CDC_DIO_OUT );

    uint32_t fPWM               = 100;
    uint16_t dutyCycle          = 0;
    uint16_t minimalThreshold   = 6;
   
    rStat = configureAdc( CDC_RES_ID_ADC_0, CDC_PIN_ADC_0 );
    rStat = configurePwm( CDC_RES_ID_PWM_0, CDC_PIN_PWM_0, CDC_PIN_PWM_1, fPWM );
  
    while ( true ) {

        toggleDio( CDC_RES_ID_LED );

        rStat = readAdc( CDC_RES_ID_ADC_0, &dutyCycle );

        if ( dutyCycle < minimalThreshold ) dutyCycle = 0;
        if ( dutyCycle > 255 )              dutyCycle = 255;

        writePwm( CDC_RES_ID_PWM_0, dutyCycle, 0 );
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