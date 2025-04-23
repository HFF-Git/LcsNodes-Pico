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
#include "LcsCdcDescMapDefaults.h"

using namespace CDC;

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
CdcResourceDescMap dMap = RES_MAP_RP_2040;

const uint8_t ADC_0 = CDC_RN_FIRST_USER_RN + 0;
const uint8_t ADC_1 = CDC_RN_FIRST_USER_RN + 1;
const uint8_t DIO_0 = CDC_RN_FIRST_USER_RN + 2;
const uint8_t DIO_1 = CDC_RN_FIRST_USER_RN + 3;
const uint8_t DIO_2 = CDC_RN_FIRST_USER_RN + 4;
const uint8_t DIO_3 = CDC_RN_FIRST_USER_RN + 5;
const uint8_t DIO_4 = CDC_RN_FIRST_USER_RN + 6;
const uint8_t DIO_5 = CDC_RN_FIRST_USER_RN + 7;
const uint8_t DIO_6 = CDC_RN_FIRST_USER_RN + 8;
const uint8_t DIO_7 = CDC_RN_FIRST_USER_RN + 9;

const uint8_t PWM_0 = CDC_RN_FIRST_USER_RN + 10;
const uint8_t PWM_1 = CDC_RN_FIRST_USER_RN + 11;

const uint8_t DIO_P_0 = CDC_RN_FIRST_USER_RN + 12;
const uint8_t DIO_P_1 = CDC_RN_FIRST_USER_RN + 13;
const uint8_t DIO_P_2 = CDC_RN_FIRST_USER_RN + 14;
const uint8_t DIO_P_3 = CDC_RN_FIRST_USER_RN + 15;

const uint8_t PWM_P_0 = CDC_RN_FIRST_USER_RN + 16;


//----------------------------------------------------------------------------------------------------------
// Init the library...
//
//----------------------------------------------------------------------------------------------------------
void initCdcLib( ) {

    dMap.map[ ADC_0 ].type          = CDC_RT_ADC;
    dMap.map[ ADC_0 ].adc.pin       = 26;
    dMap.map[ ADC_0 ].adc.adcNum    = 0;

    dMap.map[ ADC_1 ].type          = CDC_RT_ADC;
    dMap.map[ ADC_1 ].adc.pin       = 27;
    dMap.map[ ADC_1 ].adc.adcNum    = 1;

    dMap.map[ DIO_0 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_0 ].gpio.pinA     = 8;
    dMap.map[ DIO_0 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_0 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_1 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_1 ].gpio.pinA     = 9;
    dMap.map[ DIO_1 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_1 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_2 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_2 ].gpio.pinA     = 10;
    dMap.map[ DIO_2 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_2 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_3 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_3 ].gpio.pinA     = 11;
    dMap.map[ DIO_3 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_3 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_4 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_4 ].gpio.pinA     = 21;
    dMap.map[ DIO_4 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_4 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_5 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_5 ].gpio.pinA     = 20;
    dMap.map[ DIO_5 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_5 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_6 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_6 ].gpio.pinA     = 19;
    dMap.map[ DIO_6 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_6 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_7 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_7 ].gpio.pinA     = 18;
    dMap.map[ DIO_7 ].gpio.pinB     = UNDEFINED_PIN;
    dMap.map[ DIO_7 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_0 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_0 ].gpio.pinA     = 8;
    dMap.map[ DIO_P_0 ].gpio.pinB     = 9;
    dMap.map[ DIO_P_0 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_1 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_1 ].gpio.pinA     = 10;
    dMap.map[ DIO_P_1 ].gpio.pinB     = 11;
    dMap.map[ DIO_P_1 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_2 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_2 ].gpio.pinA     = 21;
    dMap.map[ DIO_P_2 ].gpio.pinB     = 20;
    dMap.map[ DIO_P_2 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ DIO_P_3 ].type          = CDC_RT_GPIO;
    dMap.map[ DIO_P_3 ].gpio.pinA     = 19;
    dMap.map[ DIO_P_3 ].gpio.pinB     = 18;
    dMap.map[ DIO_P_3 ].gpio.pinMode  = CDC_DIO_IN;

    dMap.map[ PWM_0 ].type          = CDC_RT_PWM;
    dMap.map[ PWM_0 ].pwm.pinA      = 20;
    dMap.map[ PWM_0 ].pwm.pinB      = UNDEFINED_PIN;
    dMap.map[ PWM_0 ].pwm.frequency = 100;

    dMap.map[ PWM_1 ].type          = CDC_RT_PWM;
    dMap.map[ PWM_1 ].pwm.pinA      = 21;
    dMap.map[ PWM_1 ].pwm.pinB      = UNDEFINED_PIN;
    dMap.map[ PWM_1 ].pwm.frequency = 100;

    dMap.map[ PWM_P_0 ].type          = CDC_RT_PWM;
    dMap.map[ PWM_P_0 ].pwm.pinA      = 20;
    dMap.map[ PWM_P_0 ].pwm.pinB      = 21;
    dMap.map[ PWM_P_0 ].pwm.frequency = 100;

    cdcInit( &dMap );
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

  configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );
  writeDio( CDC_RN_ACTIVITY_LED, true );
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

  configureDio( CDC_RN_PFAIL );
  registerDioCallback( CDC_RN_PFAIL, CDC_EVT_LOW, pfailCallback );
  
  configureDio( CDC_RN_ACTIVITY_LED );
  writeDio( CDC_RN_ACTIVITY_LED, true );
  
  printf( "testPfail -> unplug the power cord \n" );
}

//----------------------------------------------------------------------------------------------------------
// Test the onboard LEDs.
//
//----------------------------------------------------------------------------------------------------------
void testLeds( ) {

  printf( "Active Led Test\n" );
  configureDio( CDC_RN_ACTIVITY_LED );
  
  while ( true ) {

    writeDio( CDC_RN_ACTIVITY_LED, true );
    sleepMillis( 500 );
    
    writeDio( CDC_RN_ACTIVITY_LED, false );
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

    configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );
    configureDio( DIO_0, CDC_DIO_IN_PULLUP );
    configureDio( DIO_1, CDC_DIO_IN_PULLUP );
    configureDio( DIO_2, CDC_DIO_IN_PULLUP );
    configureDio( DIO_3, CDC_DIO_IN_PULLUP );
    configureDio( DIO_4, CDC_DIO_IN_PULLUP );
    configureDio( DIO_5, CDC_DIO_IN_PULLUP );
    configureDio( DIO_6, CDC_DIO_IN_PULLUP );
    configureDio( DIO_7, CDC_DIO_IN_PULLUP );
   
    while ( true ) {

        sleepMillis( 1000 );
        toggleDio( CDC_RN_ACTIVITY_LED );

        bool val;
        uint8_t rStat;

        rStat = readDio( DIO_0, &val );
        printf( "Econ Dio In 0: %d\n", val );

        rStat = readDio( DIO_1, &val );
        printf( "Econ Dio In 1: %d\n", val );

        rStat = readDio( DIO_2, &val );
        printf( "Econ Dio In 2: %d\n", val );

        rStat = readDio( DIO_3, &val );
        printf( "Econ Dio In 3: %d\n", val );

        rStat = readDio( DIO_4, &val );
        printf( "Econ Dio In 4: %d\n", val );

        rStat = readDio( DIO_5, &val );
        printf( "Econ Dio In 5: %d\n", val );

        rStat = readDio( DIO_6, &val );
        printf( "Econ Dio In 6: %d\n", val );

        rStat = readDio( DIO_7, &val );
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

    configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );
    configureDio( DIO_0, CDC_DIO_OUT );
    configureDio( DIO_1, CDC_DIO_OUT );
    configureDio( DIO_2, CDC_DIO_OUT );
    configureDio( DIO_3, CDC_DIO_OUT );
    configureDio( DIO_4, CDC_DIO_OUT );
    configureDio( DIO_5, CDC_DIO_OUT );
    configureDio( DIO_6, CDC_DIO_OUT );
    configureDio( DIO_7, CDC_DIO_OUT );

    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );
        writeDio( DIO_0, false );
        writeDio( DIO_1, false );
        writeDio( DIO_2, false );
        writeDio( DIO_3, false );
        writeDio( DIO_4, false );
        writeDio( DIO_5, false );
        writeDio( DIO_6, false );
        writeDio( DIO_7, false );
        sleepMillis( 1000 );

        writeDio( DIO_0, true );
        sleepMillis( 500 );
        writeDio( DIO_1, true );
        sleepMillis( 500 );
        writeDio( DIO_2, true );
        sleepMillis( 500 );
        writeDio( DIO_3, true );
        sleepMillis( 500 );
        writeDio( DIO_4, true );
        sleepMillis( 500 );
        writeDio( DIO_5, true );
        sleepMillis( 500 );
        writeDio( DIO_6, true );
        sleepMillis( 500 );
        writeDio( DIO_7, true );
        sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------------------------
// Test the DIO pin pairs. We use the first four resource IDs and pass two pins at configuration time.
//
//----------------------------------------------------------------------------------------------------------
void testDioOutputPair( ) {

  printf( "DIO output pair test\n" );

  configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );
  configureDio( DIO_0, CDC_DIO_OUT );
  configureDio( DIO_1, CDC_DIO_OUT );
  configureDio( DIO_2, CDC_DIO_OUT );
  configureDio( DIO_3, CDC_DIO_OUT );
  configureDio( DIO_4, CDC_DIO_OUT );
  configureDio( DIO_5, CDC_DIO_OUT );
  configureDio( DIO_6, CDC_DIO_OUT );
  configureDio( DIO_7, CDC_DIO_OUT );
  
  while ( true ) {

    toggleDio( CDC_RN_ACTIVITY_LED );

    writeDio( DIO_P_0, false, false );
    writeDio( DIO_P_1, false, false );
    writeDio( DIO_P_2, false, false );   
    writeDio( DIO_P_3, false, false );
    sleepMillis( 1000 );

    writeDio( DIO_P_0, true, false );
    sleepMillis( 500 );
    writeDio( DIO_P_0, false, true );
    sleepMillis( 500 );
    writeDio( DIO_P_0, true, true );
    sleepMillis( 500 );

    writeDio( DIO_P_1, true, false );
    sleepMillis( 500 );
    writeDio( DIO_P_1, false, true );
    sleepMillis( 500 );
    writeDio( DIO_P_1, true, true );
    sleepMillis( 500 );

    writeDio( DIO_P_2, true, false );
    sleepMillis( 500 );
    writeDio( DIO_P_2, false, true );
    sleepMillis( 500 );
    writeDio( DIO_P_2, true, true );
    sleepMillis( 500 );

    writeDio( DIO_P_3, true, false );
    sleepMillis( 500 );
    writeDio( DIO_P_3, false, true );
    sleepMillis( 500 );
    writeDio( DIO_P_3, true, true );
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

  configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );
  configureAdc( ADC_0 );
  configureAdc( ADC_1 );
  
  while ( true ) {

    uint16_t    val;
    uint8_t     rStat = readAdc( ADC_0, &val );

    printf( "ADC -> ( rNum: %d, val: %d, Volt: %d )\n", ADC_0, val, val * digitToVolt );
    sleepMillis( 1000 );

    rStat = readAdc( ADC_1, &val );

    printf( "ADC -> ( rNum: %d, val: %d, Volt: %d )\n", ADC_1, val, val * digitToVolt );
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

    configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );

    uint8_t rStat = NO_ERR;

    rStat = configureTimer( 100, timerCallback0 );
    rStat = configureTimer( 200, timerCallback1 );

    writeDio( CDC_RN_ACTIVITY_LED, true );

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

  uint32_t fPwm  = 100;
  configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );

  configurePwm( PWM_0, fPwm );
  configurePwm( PWM_1, fPwm );

    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );
    
        writePwm( PWM_P_0, 127, 63 );
        sleepMillis( 2000 );

        writePwm( PWM_P_0, 192, 127 );
        sleepMillis( 2000 );

        writePwm( PWM_P_0, 63, 192 );
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

    configureDio( CDC_RN_ACTIVITY_LED, CDC_DIO_OUT );

    uint32_t fPWM               = 100;
    uint16_t dutyCycle          = 0;
    uint16_t minimalThreshold   = 6;
    uint8_t  rStat              = NO_ERR;
   
    rStat = configureAdc( ADC_0 );
    rStat = configurePwm( PWM_0, fPWM );
  
    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );

        rStat = readAdc( ADC_0, &dutyCycle );

        if ( dutyCycle < minimalThreshold ) dutyCycle = 0;
        if ( dutyCycle > 255 )              dutyCycle = 255;

        writePwm( PWM_0, dutyCycle, 0 );
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
    // testConsoleIO( );
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