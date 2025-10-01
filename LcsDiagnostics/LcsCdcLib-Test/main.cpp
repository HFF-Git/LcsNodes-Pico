//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Test Program
//
//----------------------------------------------------------------------------------------
// This is a little test program for the individual functions of the CDC layer.
// It is a rather crude program and you need to recompile it for each test of a
// portion of the library.
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Test Program
// Copyright (C) 2025 - 2025 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details. You should have received a copy of the GNU General Public
// License along with this program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsCdcLibTestBoardDesc.h"
#include "LcsCdcLib.h"

using namespace CDC;

//----------------------------------------------------------------------------------------
// This is the board configuration we use. See the include file for the details.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap dMap = LCS_MAIN_CONTROLLER_BOARD_DESC_B_01_00;

//----------------------------------------------------------------------------------------
// Init the library. 
//
//----------------------------------------------------------------------------------------
void initCdcLib( ) {

    dMap.options    = 0;
    dMap.debugMask  = CDC_DBG_CONFIG | CDC_DBG_SETUP;

    cdcInit( &dMap );
    configureConsoleIO( );
    sleepMillis( 2000 );
    printf( "Test LCS Controller dependent code library\n" );
    printResourceDescMap( &dMap );
    sleepMillis( 2000 );
}

//----------------------------------------------------------------------------------------
// Test the console IO.
//
//----------------------------------------------------------------------------------------
void testConsoleIO ( ) {

    configureDio( CDC_RN_ACTIVITY_LED );
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

//----------------------------------------------------------------------------------------
// "testEcho" simply echoes what is sent to the PICO via the USB console. We need 
// it for RocRail testing of RocNet Messages...
//
//----------------------------------------------------------------------------------------
void echoConsole( ) {

    configureDio( CDC_RN_ACTIVITY_LED );
    writeDio( CDC_RN_ACTIVITY_LED, true );
    sleepMillis( 1000 );

    printf( "Show Console IO input\n" );
    printf( "USB is connected: %d\n", CDC::isConsoleConnected( ));

    while ( true ) {

        char c = getConsoleChar( );
        printf( "%c\n", c );
    }
}

//----------------------------------------------------------------------------------------
// test the power failure option.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Test the onboard LEDs.
//
//----------------------------------------------------------------------------------------
void testOnboardLeds( ) {

  printf( "Active Led Test\n" );
  uint8_t rStat = configureDio( CDC_RN_ACTIVITY_LED );
  if ( rStat != NO_ERR ) fatalError( 4 );
  
  while ( true ) {

    writeDio( CDC_RN_ACTIVITY_LED, true );
    sleepMillis( 500 );
    
    writeDio( CDC_RN_ACTIVITY_LED, false );
    sleepMillis( 500 );
  }
}

//----------------------------------------------------------------------------------------
// Test Fatal Error LED. Note, we will not come back from this call.
//
//----------------------------------------------------------------------------------------
void testFatalErr( ) {

  printf( "Fatal Error Test\n" );
  fatalError( 4 );
}

//----------------------------------------------------------------------------------------
// Set the DIO pins to input, pull-up and read the values. Use a little cable 
// to set the voltage on the extension connector. Note that we overwrite the 
// board descriptor data, since we need another pinMode.
//
//----------------------------------------------------------------------------------------
void configureDioInPinA( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_GPIO ); 
    if ( dPtr == nullptr ) fatalError( 4 );

    configureDio( rNum, dPtr -> gpio.pinA, UNDEFINED_PIN, CDC_DIO_IN_PULLUP );
}

void testDioInput( ) {

    printf( "DIO input test\n" );

    configureDio( CDC_RN_ACTIVITY_LED );
    configureDioInPinA( RNUM_DIO_0 );
    configureDioInPinA( RNUM_DIO_1 );
    configureDioInPinA( RNUM_DIO_2 );
    configureDioInPinA( RNUM_DIO_3 );
    configureDioInPinA( RNUM_DIO_4 );
    configureDioInPinA( RNUM_DIO_5 );
    configureDioInPinA( RNUM_DIO_6 );
    configureDioInPinA( RNUM_DIO_7 );

    while ( true ) {

        sleepMillis( 1000 );
        toggleDio( CDC_RN_ACTIVITY_LED );

        bool val;
        uint8_t rStat;

        rStat = readDio( RNUM_DIO_0, &val );
        printf( "Econ Dio In 0: %d\n", val );

        rStat = readDio( RNUM_DIO_1, &val );
        printf( "Econ Dio In 1: %d\n", val );

        rStat = readDio( RNUM_DIO_2, &val );
        printf( "Econ Dio In 2: %d\n", val );

        rStat = readDio( RNUM_DIO_3, &val );
        printf( "Econ Dio In 3: %d\n", val );

        rStat = readDio( RNUM_DIO_4, &val );
        printf( "Econ Dio In 4: %d\n", val );

        rStat = readDio( RNUM_DIO_5, &val );
        printf( "Econ Dio In 5: %d\n", val );

        rStat = readDio( RNUM_DIO_6, &val );
        printf( "Econ Dio In 6: %d\n", val );

        rStat = readDio( RNUM_DIO_7, &val );
        printf( "Econ Dio In 7: %d\n", val );
    }
}

//----------------------------------------------------------------------------------------
// Set the DIO pins to output and periodically toggle the values. Use a an LED 
// array and connect the pins of the extension connector to it. The toggle Led 
// just indicates that the board is basically working. Note that we overwrite 
// the Board descriptor data, since we need another pinMode.
//
//----------------------------------------------------------------------------------------
void configureDioOutPinA( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_GPIO ); 
    if ( dPtr == nullptr ) fatalError( 4 );

    configureDio( rNum, dPtr -> gpio.pinA, UNDEFINED_PIN, CDC_DIO_OUT );
}

void testDioOutput( ) {

    printf( "DIO output test\n" );

    configureDio( CDC_RN_ACTIVITY_LED );
    configureDioOutPinA( RNUM_DIO_0 );
    configureDioOutPinA( RNUM_DIO_1 );
    configureDioOutPinA( RNUM_DIO_2 );
    configureDioOutPinA( RNUM_DIO_3 );
    configureDioOutPinA( RNUM_DIO_4 );
    configureDioOutPinA( RNUM_DIO_5 );
    configureDioOutPinA( RNUM_DIO_6 );
    configureDioOutPinA( RNUM_DIO_7 );

    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );
        writeDio( RNUM_DIO_0, false );
        writeDio( RNUM_DIO_1, false );
        writeDio( RNUM_DIO_2, false );
        writeDio( RNUM_DIO_3, false );
        writeDio( RNUM_DIO_4, false );
        writeDio( RNUM_DIO_5, false );
        writeDio( RNUM_DIO_6, false );
        writeDio( RNUM_DIO_7, false );
        sleepMillis( 1000 );

        writeDio( RNUM_DIO_0, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_1, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_2, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_3, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_4, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_5, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_6, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_7, true );
        sleepMillis( 500 );
    }
}

//----------------------------------------------------------------------------------------
// Test the DIO pin pairs. We use the first four resource IDs and pass two pins
// at configuration time. Note that we overwrite the board descriptor data, 
// since we need another pinMode.
//
//----------------------------------------------------------------------------------------
void configureDioOutPinPair( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_GPIO ); 
    if ( dPtr == nullptr ) fatalError( 4 );

    configureDio( rNum, dPtr -> gpio.pinA, dPtr -> gpio.pinB, CDC_DIO_OUT );
}

void testDioOutputPair( ) {

    printf( "DIO output pair test\n" );

    configureDio( CDC_RN_ACTIVITY_LED );
    configureDioOutPinPair( RNUM_DIO_P_0 );
    configureDioOutPinPair( RNUM_DIO_P_1 );
    configureDioOutPinPair( RNUM_DIO_P_2 );
    configureDioOutPinPair( RNUM_DIO_P_3 );
     
    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );

        writeDio( RNUM_DIO_P_0, false, false );
        writeDio( RNUM_DIO_P_1, false, false );
        writeDio( RNUM_DIO_P_2, false, false );   
        writeDio( RNUM_DIO_P_3, false, false );
        sleepMillis( 1000 );

        writeDio( RNUM_DIO_P_0, true, false );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_0, false, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_0, true, true );
        sleepMillis( 500 );

        writeDio( RNUM_DIO_P_1, true, false );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_1, false, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_1, true, true );
        sleepMillis( 500 );

        writeDio( RNUM_DIO_P_2, true, false );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_2, false, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_2, true, true );
        sleepMillis( 500 );

        writeDio( RNUM_DIO_P_3, true, false );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_3, false, true );
        sleepMillis( 500 );
        writeDio( RNUM_DIO_P_3, true, true );
        sleepMillis( 500 );
    }
}

//----------------------------------------------------------------------------------------
// Test the analog blocking read.
//
//----------------------------------------------------------------------------------------
void testAdcBlockingRead( ) {

  float digitToVolt = (float) 3300 / 1024 / 1000; // quick hack ...

  printf( "ADC read test\n" );

  configureDio( CDC_RN_ACTIVITY_LED );
  configureAdc( RNUM_ADC_0 );
  configureAdc( RNUM_ADC_1 );
  
  while ( true ) {

    uint16_t    val;
    uint8_t     rStat = readAdc( RNUM_ADC_0, &val );

    printf( "ADC -> ( rNum: %d, val: %d, Volt: %d )\n", 
            RNUM_ADC_0, val, val * digitToVolt );
    sleepMillis( 1000 );

    rStat = readAdc( RNUM_ADC_1, &val );

    printf( "ADC -> ( rNum: %d, val: %d, Volt: %d )\n", 
            RNUM_ADC_1, val, val * digitToVolt );
    sleepMillis( 1000 );
  }
}

//----------------------------------------------------------------------------------------
// Test the timer interrupt. The callback functions are invoked an we display
// the that the timer fired. 
//
//----------------------------------------------------------------------------------------
void timerCallback0( uint32_t timerVal ) {

    printf( "Timer 0 fired: %d\n", getMillis( ));
}

void timerCallback1( uint32_t timerVal ) {

    printf( "Timer 1 fired: %d\n", getMillis( ));
}

void testTimer( ) {

    printf( "Timer test\n" );

    configureDio( CDC_RN_ACTIVITY_LED );
    writeDio( CDC_RN_ACTIVITY_LED, true );

    uint8_t rStat = NO_ERR;

    while ( true ) {
    
        printf( "Configure Timers \n");
        rStat = configureTimer( RNUM_TIMER_0, timerCallback0 );
        rStat = configureTimer( RNUM_TIMER_1, timerCallback1 );

        printf( "Start Timers \n");
        startRepeatingTimer( RNUM_TIMER_0, 500000 );
        startRepeatingTimer( RNUM_TIMER_1, 250000 );

        sleepMillis( 10000 );

        printf( "Stop Timers \n");
        stopRepeatingTimer( RNUM_TIMER_0 );
        stopRepeatingTimer( RNUM_TIMER_1 );
    }
}

//----------------------------------------------------------------------------------------
// "testI2C" uses the I2C bus. Currently, there is no real test. The I2C 
// routines are used by the runtime library. SInce we have a HW setup for NVM
// read and write, might as well debug I2C it there.
//
//----------------------------------------------------------------------------------------
void testI2C( ) {

    printf( "List I2C channels test\n" );

    uint8_t rStat = NO_ERR;

    configureDio( CDC_RN_ACTIVITY_LED );
    writeDio( CDC_RN_ACTIVITY_LED, true );

    if ( rStat == NO_ERR ) rStat = configureI2C( CDC_RN_NVM );
    if ( rStat != NO_ERR ) printf( "Error configuring NVM I2C channel: %d\n", rStat );

    if ( rStat == NO_ERR ) rStat = configureI2C( CDC_RN_EXT_NVM );
    if ( rStat != NO_ERR ) printf( "Error configuring EXT I2C channel: %d\n", rStat );

    if ( rStat != NO_ERR );

    int scanCount = 0;  
  
    while( true ) {

        printf( "Scanning (%d) ... \n", scanCount );

        printf( "Scanning NVM I2C Bus\n" );
        scanI2CBus( CDC_RN_NVM );
        printf( "\n" );

        printf( "Scanning EXT I2C Bus\n" );
        scanI2CBus( CDC_RN_EXT_NVM );
        printf( "\n" );

        scanCount++;
        sleepMillis( 5000 );
    }
}

//----------------------------------------------------------------------------------------
// "testPWMFixed" tests the PWM functionality of the DIO pins 6 and 7. We will 
// just configure the two ports, set the frequency and three values to see of
// the duty cycle changes. Best to see on an Oscilloscope.
//
//----------------------------------------------------------------------------------------
void configurePwmPins( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_PWM ); 
    if ( dPtr == nullptr ) fatalError( 4 );

    configurePwm( rNum, 
                  dPtr -> pwm.pinA, 
                  dPtr -> pwm.pinB, 
                  dPtr -> pwm.frequency );     
}

void testPWMFixed( ) {

    printf( "PWM fixed frequency test\n" );

    configureDio( CDC_RN_ACTIVITY_LED );
    configurePwmPins( RNUM_PWM_P_0 );
  
    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );
    
        writePwm( RNUM_PWM_P_0, 127, 63 );
        sleepMillis( 5000 );

        writePwm( RNUM_PWM_P_0, 192, 127 );
        sleepMillis( 5000 );

        writePwm( RNUM_PWM_P_0, 63, 192 );
        sleepMillis( 5000 );
    }
}

//----------------------------------------------------------------------------------------
// "testPWMWithAnalogInput" will read in an analog value and use it as a 
// dutyCycle for the PWM outputs. We use a frequency of 100Hz, which is nicely
// to see on an Oscilloscope with a period length of 10ms. 
//
//----------------------------------------------------------------------------------------
void testPWMWithAnalogInput( ) {

    printf( "PWM with analog input test\n" );

    configureDio( CDC_RN_ACTIVITY_LED );

    uint16_t dutyCycle          = 0;
    uint16_t minimalThreshold   = 6;
   
    configureAdc( RNUM_ADC_0 );
    configurePwmPins( RNUM_PWM_0 );
  
    while ( true ) {

        toggleDio( CDC_RN_ACTIVITY_LED );

        readAdc( RNUM_ADC_0, &dutyCycle );

        if ( dutyCycle < minimalThreshold ) dutyCycle = 0;
        if ( dutyCycle > 255 )              dutyCycle = 255;

        writePwm( RNUM_PWM_0, dutyCycle, 0 );
        sleepMillis( 100 );
  }
}

//----------------------------------------------------------------------------------------
// "testUIDGen" test the UID generation code.
//
//----------------------------------------------------------------------------------------
void testUIDGen( ) {

    printf( "UID generation test\n" );
    sleepMillis( 1000 );

    for ( int i = 0; i < 20; i++ ) {

        printf( "UID -> %d\n ", createUid( ));
        sleepMillis( 100 );
    }
}

//----------------------------------------------------------------------------------------
// Main. A bit crude. Just enable what you want to test ...
//
//----------------------------------------------------------------------------------------
int main( ) {

    initCdcLib( );

    // testFatalErr( );
    // testConsoleIO( );
    // testPfail( );
    // testOnboardLeds( );
    // testDioInput( );
    // testDioOutput( );
    testDioOutputPair( );
    // testAdcBlockingRead( );
    // testTimer( );
    // testI2C( );
    // testPWMFixed( );
    // testPWMWithAnalogInput( );
    // testUIDGen( );
    
    // echoConsole( );

    return( 0 );
}