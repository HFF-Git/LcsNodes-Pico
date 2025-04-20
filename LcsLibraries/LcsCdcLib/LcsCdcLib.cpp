//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the the Raspberry Pi controller family hardware library code. The idea of this
// library is to shield the actual hardware of processor and board implementation from the upper layers but 
// still keep the flexibility and performance of the underlying hardware. We also need to provide a handle 
// from the defined hardware elements to the particular board. 
//
// The library works with a concept of "resources". During startup, the resources are configured based on 
// data from the board descriptor. They can locate a resources by a unique resource ID name, which is used
// to lookup the resource handle. From thereon the handle is used to access that particular resource. An 
// application is thus written assigning the predefined resource names to the function it needs. 
//
// Boards are free to use whatever names, there is however a convention to name the controller resource 
// descriptor "Controller". 
//
// A historic note. The original LCS code was written for Atmega and Pico. With the complete shift to PICO,
// the CDC library just serves as a simple interface to the PICO functions. One day, we may see more different
// controllers and controller families. 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2025 Helmut Fieres
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
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "tusb_config.h"
#include "hardware/regs/usb.h"
#include "hardware/regs/rosc.h"
#include "hardware/regs/addressmap.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "LcsCdcLib.h"

//------------------------------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space with all internal
// variables and routines local to the file. The second part contains the exported routines to be called by
// the core library and the firmware designers that need access to the underlying HW portion managed by this
// lowest layer.
//
//------------------------------------------------------------------------------------------------------------
namespace {

using namespace CDC;

//------------------------------------------------------------------------------------------------------------  
// Debug and Trace support. Instead of conditional compilation, we will print debug messages based on the
// setting of the debug mask.
//
//------------------------------------------------------------------------------------------------------------ 
uint16_t debugMask = CDC_DBG_CONFIG | CDC_DBG_PWM;

//------------------------------------------------------------------------------------------------------------  
// The CDC Library version data.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t CDC_LIB_MAJOR_VERSION = 1;
const uint8_t CDC_LIB_MINOR_VERSION = 0;

//------------------------------------------------------------------------------------------------------------
// Valid pin mappings for the Raspberry PI Pico board. We construct a set of bitmask for the pin numbers.
// Pin Numbers range from 0 to 28. The bitmasks specify wether a pin can be assigned to the hardware type
// purpose. During configuration of a CDC function, the pins are checked against these bitmasks. All pins
// can be used as GPIO pins or PWM pins. All other hardware functions are bound to dedicated pins. Note
// that we do not check for assigning a pin to several different hardware functions. All we check is that
// the pin can be used for the desired purpose. A check performed by the CDC library routines is simply
// done through:
//
//    if (( 1 <<  pin ) & VALID_xxx )
//
//------------------------------------------------------------------------------------------------------------
const uint8_t  MAX_PIN_NUM          = 28;

const uint32_t VALID_GPIO_PINS      =   0x1FFFFFFF;
const uint32_t VALID_PWM_PINS       =   0x1FFFFFFF;
const uint32_t VALID_ADC_PINS       =   ( 1 << 26 ) | ( 1 << 27 ) | ( 1 << 28 );

const uint32_t VALID_I2C_0_SDA_PINS =   ( 1 << 0  ) | ( 1 << 4  ) | ( 1 << 8  ) |
                                        ( 1 << 12 ) | ( 1 << 16 ) | ( 1 << 20 );
const uint32_t VALID_I2C_0_SCL_PINS =   ( 1 << 1  ) | ( 1 << 5  ) | ( 1 << 9  ) |
                                        ( 1 << 13 ) | ( 1 << 17 ) | ( 1 << 21 );

const uint32_t VALID_I2C_1_SDA_PINS =   ( 1 << 2  ) | ( 1 << 6  ) | ( 1 << 10 ) |
                                        ( 1 << 14 ) | ( 1 << 18 ) | ( 1 << 26 );
const uint32_t VALID_I2C_1_SCL_PINS =   ( 1 << 3  ) | ( 1 << 7  ) | ( 1 << 11 ) |
                                        ( 1 << 15 ) | ( 1 << 19 ) | ( 1 << 27 );

const uint32_t VALID_UART_0_TX_PINS =   ( 1 << 0  ) | ( 1 << 12 ) | ( 1 << 16 );
const uint32_t VALID_UART_0_RX_PINS =   ( 1 << 1  ) | ( 1 << 13 ) | ( 1 << 17 );

const uint32_t VALID_UART_1_TX_PINS =   ( 1 << 4  ) | ( 1 << 8  );
const uint32_t VALID_UART_1_RX_PINS =   ( 1 << 5  ) | ( 1 << 9  );

const uint32_t VALID_SPI_0_SCK_PINS =   ( 1 << 2  ) | ( 1 << 6  ) | ( 1 << 18 );
const uint32_t VALID_SPI_0_TX_PINS  =   ( 1 << 3  ) | ( 1 << 7  ) | ( 1 << 19 );
const uint32_t VALID_SPI_0_RX_PINS  =   ( 1 << 0  ) | ( 1 << 4  ) | ( 1 << 16 );

const uint32_t VALID_SPI_1_SCK_PINS =   ( 1 << 10 ) | ( 1 << 14 );
const uint32_t VALID_SPI_1_TX_PINS  =   ( 1 << 11 ) | ( 1 << 15 );
const uint32_t VALID_SPI_1_RX_PINS  =   ( 1 << 8  ) | ( 1 << 12 );

const uint32_t VALID_I2C_0_PINS     =   VALID_I2C_0_SDA_PINS | VALID_I2C_0_SCL_PINS;
const uint32_t VALID_I2C_1_PINS     =   VALID_I2C_1_SDA_PINS | VALID_I2C_1_SCL_PINS;

const uint32_t VALID_UART_0_PINS    =   VALID_UART_0_TX_PINS | VALID_UART_0_RX_PINS;
const uint32_t VALID_UART_1_PINS    =   VALID_UART_1_TX_PINS | VALID_UART_1_RX_PINS;

const uint32_t VALID_SPI_0_PINS     =   VALID_SPI_0_SCK_PINS | VALID_SPI_0_TX_PINS | VALID_SPI_0_RX_PINS;
const uint32_t VALID_SPI_1_PINS     =   VALID_SPI_1_SCK_PINS | VALID_SPI_1_TX_PINS | VALID_SPI_1_RX_PINS;

//----------------------------------------------------------------------------------------------------------
// Characteristics of the Raspberry Pi Pico and some key constants for the CDC library.
// 
//----------------------------------------------------------------------------------------------------------
const uint32_t  CHIP_MEM_SIZE                = 264 * 1024;         // ??? not true for RP2350
const uint32_t  CHIP_NVM_SIZE                = 0;

const uint16_t  ADC_DIGIT_RANGE              = 1024;
const uint16_t  ADC_REF_VOLTAGE_MILLI_VOLT   = 3300;

const uint8_t   MAX_UART_BUF_SIZE            = 8;

const uint32_t  I2C_FREQUENCY                = 100 * 1000;
const uint32_t  I2C_TIME_OUT_IN_MS           = 250;

const uint16_t  MAX_CPU_CORE                = 2;
const uint16_t  MAX_INT_PIN                 = 24;

const uint16_t  MAX_RESOURCE_ENTRIES        = 64;

//------------------------------------------------------------------------------------------------------------
// Controller dependent code uses a set of hardware resource structures to control the controller hardware. 
// When a particular resource, e.g. an I2C channel, is configured all further access will use the resource 
// data for its operation. 
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// A timer instance. We need to keep the local timer instance data for the PICO.
//
//------------------------------------------------------------------------------------------------------------
struct TimerResource {

    bool                configured = false;
    uint8_t             timerResId;
    TimerCallback       timerCallback;
    repeating_timer_t   timerData;
};

//------------------------------------------------------------------------------------------------------------
// An ADC instance. The PICO supports up to three ADC inputs. When we use such an input, the corresponding
// instance data is kept in this structure. We also keep the PICO ADC number, so we can select the correct
// instance.
//
//------------------------------------------------------------------------------------------------------------
struct AdcResource {

    bool      configured  = false;
    uint8_t   adcPin      = UNDEFINED_PIN;
    uint8_t   adcNum      = 0;
};

//------------------------------------------------------------------------------------------------------------
//
// ??? well what to here if at all...
//------------------------------------------------------------------------------------------------------------
struct GpioResource {

    uint8_t         dioPin  = UNDEFINED_PIN;
    uint8_t         pinMode = CDC_DIO_IN_PULLUP;
    GpioCallback    handler = nullptr;
};

//------------------------------------------------------------------------------------------------------------
// A PWM output resource. GPIO pins can also be used as PWM output pins. The PWM output related data is
// kept in this instance. We keep track of one or two pins, which must be on the same slice. The idea is 
// that we use for H-Bridge control two output signals, which act as a pair. Note that the PICO can set
// some attributes, such as "wrap", only for the slice, not an individual channel. 
//
//------------------------------------------------------------------------------------------------------------
struct PwmResource {

    bool        configured      = false;
    uint8_t     pwmPin          = UNDEFINED_PIN;
    uint32_t    frequency       = 0;
    uint        wrap            = 0;
    uint        channel         = 0;
    uint        sliceNum        = 0;
    bool        inverted        = false;
    bool        phaseCorrect    = true;
};

//------------------------------------------------------------------------------------------------------------
// UARTS are used to read in a serial stream from the RailCom detectors. There can be two hardware based UART
// instances. The instance also keeps a small buffer where the data is read into. We also keep the PICO UART
// HW instance used.
//
//------------------------------------------------------------------------------------------------------------
struct UartResource {

    bool              configured    = false;
    uint8_t           rxPin         = UNDEFINED_PIN;
    uint8_t           txPin         = UNDEFINED_PIN;
    uint16_t          baudSetting   = 0;
    uint8_t           dataBits      = 8;
    uart_parity_t     parityMode    = UART_PARITY_NONE;
    uint8_t           stopBits      = 1;
    int               uartIrq       = 0;

    volatile uint8_t  rxBufIndex    = 0;
    volatile uint8_t  rxDataBuf[ MAX_UART_BUF_SIZE ] = { 0 };

    uart_inst_t       *uartHw       = nullptr;
};

//------------------------------------------------------------------------------------------------------------
// The PICO features two I2C HW instances. The instance data contains the assigned GPIO pins, the baud rate 
// and a timeout.
//
//------------------------------------------------------------------------------------------------------------
struct I2CResource {

    bool        configured    = false;
    uint8_t     sclPin        = UNDEFINED_PIN;
    uint8_t     sdaPin        = UNDEFINED_PIN;
    uint32_t    baudRate      = I2C_FREQUENCY;
    uint32_t    timeoutValMs  = I2C_TIME_OUT_IN_MS;

    i2c_inst_t  *i2cHw        = nullptr;
};

//------------------------------------------------------------------------------------------------------------
// The CAN bus resource. Although our current controller does not feature a CAN bus, the resource described 
// the hardware elements needed. Currently, we use a software version based on a PIO program to implement the
// CAN bus. 
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusResource {

    uint8_t         canPinRx;
    uint8_t         canPinTx;
    uint32_t        baudRate;
    bool            twoCores;
};

//------------------------------------------------------------------------------------------------------------
// The interrupt table for the GPIO pin interrupts. The PICO can have only one interrupt handler. We will
// allocate a table where an interrupt handler can be set for each HW pin. When an interrupt comes in we 
// look up the corresponding resource handle and when there is a handler configured, it will be called. 
//
//------------------------------------------------------------------------------------------------------------
struct GpioIsrTable {

    uint16_t        numOfHandlers = 0;
    GpioCallback    gpioIsrTable[ MAX_CPU_CORE ][ MAX_INT_PIN + 1 ];
};

//------------------------------------------------------------------------------------------------------------
// Local variables. We maintain an instance variable for each of the possible HW entities, such as an I2C
// interface or a UART. Note that not all are used at the same time. The instance variables map from the
// simple pin numbers to the PICO structures and whatever else we need to remember for this entity.
//
//------------------------------------------------------------------------------------------------------------
bool                    initialized = false; // move into cMap ?
CdcResourceMap          cMap;

TimerResource           timer_0;
TimerResource           timer_1;
TimerResource           timer_2;
TimerResource           timer_3;

GpioIsrTable            dioIntHandlers;

AdcResource             adc_0;
AdcResource             adc_1;
AdcResource             adc_2;
AdcResource             adc_3;

PwmResource             pwm_0;
PwmResource             pwm_1;
PwmResource             pwm_2;
PwmResource             pwm_3;
PwmResource             pwm_4;
PwmResource             pwm_5;
PwmResource             pwm_6;
PwmResource             pwm_7;

UartResource            uart_0;
UartResource            uart_1;

I2CResource             i2c_0;
I2CResource             i2c_1;

//------------------------------------------------------------------------------------------------------------
// "validPin" is called to check that a pin is in the correct number range, defined and matches the bitmask
// for the desired purpose. For example, configuring an I2C port will check that the two GPIO pins are
// indeed routable to the I2C HW block in the PICO.
//
//------------------------------------------------------------------------------------------------------------
bool validPin( uint8_t pin, uint32_t mask ) {

    if ( pin == UNDEFINED_PIN )     return ( true );
    if ( pin > MAX_PIN_NUM )        return ( false );
    return (( 1 << pin ) & mask );
}

//------------------------------------------------------------------------------------------------------------
// When no interrupt is configured for a GPIO pin, we set the table entry to a dummy handler. This way
// we do not have to check for a valid procedure label when we handle an interrupt.
//
//------------------------------------------------------------------------------------------------------------
void dummyIsrHandler ( uint8_t pin, uint8_t event ) { }

//------------------------------------------------------------------------------------------------------------
// Setup the ISR table. The PICO can have only one interrupt handler. When you want a handler per GPIO pin,
// the solution is to have a table when you keep the handler on a per pin base.
//
//------------------------------------------------------------------------------------------------------------
void initIsrTable( ) {

    dioIntHandlers.numOfHandlers = 0;

    for ( uint16_t i = 0; i < MAX_CPU_CORE; i++ ) {

        for ( uint16_t j = 0; j < MAX_INT_PIN; j++ ) {

            dioIntHandlers.gpioIsrTable[ i ][ j ] = dummyIsrHandler;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// The PICO uses a set of constants to describe the interrupt type. We map our interrupt types to the PICO
// GPIO_IRQ_xxx types.
//
//------------------------------------------------------------------------------------------------------------
uint32_t mapGpioIntEvent( uint8_t event ) {

    switch ( event ) {

        case CDC_EVT_LOW:      return ( GPIO_IRQ_LEVEL_LOW );
        case CDC_EVT_HIGH:     return ( GPIO_IRQ_LEVEL_HIGH );
        case CDC_EVT_FALL:     return ( GPIO_IRQ_EDGE_FALL );
        case CDC_EVT_RISE:     return ( GPIO_IRQ_EDGE_RISE );
        case CDC_EVT_CHANGE:   return ( GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL );
        default:               return ( 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// The PICO uses a set of constants to describe the interrupt type. We map them to our types. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t mapPicoGpioEvent( uint32_t event ) {

    switch ( event ) {

        case GPIO_IRQ_LEVEL_LOW:  return ( CDC_EVT_LOW );
        case GPIO_IRQ_LEVEL_HIGH: return ( CDC_EVT_HIGH );
        case GPIO_IRQ_EDGE_FALL:  return ( CDC_EVT_FALL );
        case GPIO_IRQ_EDGE_RISE:  return ( CDC_EVT_RISE );
        default:                  return ( 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// Global Interrupt handlers. The hardware and low level library will call these handlers, which in turn 
// will invoke the respective callback function if configured. 
//
// The repeating timer alarm will handle timer interrupts. We stored the respective timer resource in the 
// "user_data" field, so that we can get to the interrupt handler configured.
//
// The GPIO interrupt handler manages the handler for all possible IO pins. The PICO can only have one 
// interrupt routine, so we feature an array of handlers where a handler for a GPIO pin can be registered. If
//  there is a handler set, we just invoke it. 
// 
// The UART handlers will handle receive character interrupts of the UART hardware blocks. There is no easy
// way to get to the resource structure where the input buffer is. We maintain two global variables in this 
// file to store the configured resource for each UART HW block.
// 
//------------------------------------------------------------------------------------------------------------
bool repeatingTimerAlarm( repeating_timer_t *rt ) {

    TimerResource *ptr = (TimerResource *) rt -> user_data;

    if ( ptr -> timerCallback != nullptr ) ptr -> timerCallback((uint32_t) ( - ptr -> timerData.delay_us ));
    return ( true );
}

void gpioCallback( uint gpioPin, uint32_t event ) {

    dioIntHandlers.gpioIsrTable[ get_core_num( )][ gpioPin ] ( gpioPin, mapPicoGpioEvent( event ));
}

void uartRxCallback0( ) {

    while ( uart_is_readable( uart0 )) {

        uint8_t ch = uart_getc( uart0 );
        if ( uart_0.rxBufIndex < MAX_UART_BUF_SIZE ) uart_0.rxDataBuf[ uart_0.rxBufIndex++ ] = ch;
    }
}

void uartRxCallback1( ) {

    while ( uart_is_readable( uart1 )) {

        uint8_t ch = uart_getc( uart1 );
        if ( uart_1.rxBufIndex < MAX_UART_BUF_SIZE ) uart_1.rxDataBuf[ uart_1.rxBufIndex++ ] = ch;
    }
}

//------------------------------------------------------------------------------------------------------------
// A little helper function to set the GPIO mode for input and output.
//
//------------------------------------------------------------------------------------------------------------
void setGpioMode( uint8_t pin, uint8_t mode ) {

    switch ( mode ) {

        case CDC_DIO_IN:  {
            
            gpio_set_dir( mode, false ); 
        
        } break;

        case CDC_DIO_OUT: {

            gpio_set_dir( pin, true );
            gpio_set_drive_strength ( pin, GPIO_DRIVE_STRENGTH_12MA );

        }  break;

        case CDC_DIO_IN_PULLUP: {

            gpio_set_dir( pin, false ); 
            gpio_pull_up( pin ); 

        } break;
    }
}

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Name space CDC. All routines and definitions exported are in this name space.
//
//------------------------------------------------------------------------------------------------------------
namespace CDC {

//------------------------------------------------------------------------------------------------------------
// For debugging purposes. Instead of conditional compilations, the debug mask will enable the printing of
// debug and trace data.
//
//------------------------------------------------------------------------------------------------------------
void setDebugMask( uint16_t mask ) {

    debugMask = mask;
}

uint16_t getDebugMask( ) {

    return ( debugMask );
}

//------------------------------------------------------------------------------------------------------------
// CDC library setup. The "init" routine will ready the CDC library. The main task is to validate the pins and
// values for the particular controller capabilities. The init routine can be called more than once without a
// problem.
//
//------------------------------------------------------------------------------------------------------------
uint8_t cdcInit( CdcResourceMap *cMapPtr ) {

    cMap = *cMapPtr;

    if ( ! initialized ) {

        initIsrTable( );
    }

    // ??? validate data ?

    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// "getDefaultResourceMap" initializes a configuration structure and sets the pre-assigned values. A typical
// sequence for an application start sequence would be to create an initial structure this way and then set
// the relevant pins and values according to the actual hardware configuration.
//
//------------------------------------------------------------------------------------------------------------
CdcResourceMap  getDefaultResourceMap( ) {

    CdcResourceMap map;
    return ( map );
 }
 
 //------------------------------------------------------------------------------------------------------------
 // "getResourceMap" will return a pointer to the copy we kept when calling the init routine with the config
 // structure to use. There is no need for the upper layers to keep the structure used at initialization time.
 //
 //------------------------------------------------------------------------------------------------------------
 CdcResourceMap  *getResourceMap( ) {

    return ( &cMap );
 }

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t getVersion( uint32_t *version ) {

    *version = ( CDC_LIB_MAJOR_VERSION << 8 | CDC_LIB_MINOR_VERSION );
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// "fatalError" is the error communication method when we cannot get anything to work, except the onboard
// LED. The Raspberry Pi PICO has a small Led on the board. We will use this LED to "blink" an error code.
// There are up to eight codes. The sequence is as follows:
//
//    repeat forever:
//
//    - 1s ON, 0.5s 0FF
//    - for ( int i = 0; i < n; i++ ) { 0.5s ON; 0.5s OFF; }
//
// The only way to get out of this loop is then to reset the board. Fatal errors are hopefully not many. One
// obvious one is when we cannot detect the NVM and thus know nothing about the board.
//
// If we have a console, we attempt to first write an error message to the console. 
//------------------------------------------------------------------------------------------------------------
void fatalError( uint8_t n, char *str, uint8_t rStat ) {

    if ( str != nullptr ) {

        if ( isConsoleConnected( )) printf( "Fatal Error: %d: %s, rStat: %d\n", n, str, rStat );
    }

    const uint8_t   ledPin      = 25;
    const uint32_t  longPulse   = 1000;
    const uint32_t  shortPulse  = 250;

    n = n % 8;

    gpio_init( ledPin );
    gpio_set_dir( ledPin, GPIO_OUT );

    while ( true ) {

        sleep_ms( longPulse );
       
        for ( int i = 0; i < n; i++ ) {

            gpio_put( ledPin, true );
            sleep_ms( shortPulse );
            gpio_put( ledPin, false );
            sleep_ms( shortPulse );
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// Simple timestamp functions.
// 
//------------------------------------------------------------------------------------------------------------
uint32_t getMillis( ) {

    return ( to_ms_since_boot( get_absolute_time( )));
}

uint32_t getMicros( ) {

    return ( to_us_since_boot( get_absolute_time( )));
}

void sleepMillis( uint32_t val ) {

    sleep_ms( val );
}

void sleepMicros( uint32_t val ) {

    sleep_us( val );
}

//------------------------------------------------------------------------------------------------------------
// "createUid" is the routine that produces a unique ID for the node. The scheme is based on a random number. 
// Alternatively we could use the unique flash chip ID on the board. 
//
//------------------------------------------------------------------------------------------------------------
uint32_t createUid( ) {

    uint32_t rVal = 0;

    volatile uint32_t *rnd_reg = (uint32_t *) ( ROSC_BASE + ROSC_RANDOMBIT_OFFSET );

    for ( int k = 0; k < 32; k++ ) {

        rVal = rVal << 1;
        rVal = rVal + ( 0x00000001 & ( *rnd_reg ));
    }

    return ( rVal );
}

//------------------------------------------------------------------------------------------------------------
// Console IO section. We set up the stdio via the USB connector. As part of the CDC init call, the configure
// call should be done rather early, so that we can print out debug messages. In normal LCS node operation
// there is no USB connected. Detecting a connection helps to decide whether we can report an error or need
// to resort to a fatal error call at startup. 
//
// There are two basic ways to detect an USB connection. The first is to simply check if there is power on 
// the USB port. The PICO features an internal GPIO pin for this purpose. Using this method still does not
// mean that we have someone connected to the USB, but just that there is a cable with power. Well, good
// enough for us. The second method truly detects that there is a USB host connected. This check is provided
// via the PICO libraries which in turn use the tinyUSB library. However, there could be a timing problem
// where the USB stack is not ready and we conclude wrongly that there is no USB connection. For now, let's
// rather go with the approach to check if there is power on the VBUS pin, at the risk that there is just 
// power on the USB connector and no data.
//
// Finally, there is a routine to get a character for the command interfaces. Since the function just reads
// in a character, optionally with a timeout how long to wait for any inout.
//
// PS: The USB check way would be "return ( stdio_usb_connected( ));" instead of the GPIO check.
//------------------------------------------------------------------------------------------------------------
uint8_t configureConsoleIO( ) {

    stdio_init_all( );
    return ( NO_ERR );
}

bool isConsoleConnected( ) {

    gpio_init( PICO_VBUS_PIN );
    gpio_set_dir( PICO_VBUS_PIN, GPIO_IN );

    return ( gpio_get( PICO_VBUS_PIN ));
}
  
char getConsoleChar( uint32_t timeoutVal ) {

    int ch = getchar_timeout_us( timeoutVal );
    return (( ch == PICO_ERROR_TIMEOUT ) ? 0 : ch );
}

//------------------------------------------------------------------------------------------------------------
// Processor general values required by the low level LCS core library functions. A controller is configured
// just like any other resource. Although most of the controller parameters are fixed we take the values from
// the descriptor array, since some of them are not that easy to get. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t getFamily( uint16_t *family ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *family =cMap.cFamily;
    return ( NO_ERR );
}

uint8_t getChipMemSize( uint32_t *size ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *size = cMap.memorySize;
    return ( NO_ERR );
}

uint8_t getChipNvmSize( uint32_t *size ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *size = CHIP_NVM_SIZE;
    return ( NO_ERR );
}

uint8_t getCpuFrequency( uint32_t *frequency ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *frequency = clock_get_hz( clk_sys );
    return ( NO_ERR );
}

uint8_t watchDogEnable( bool enable ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    watchdog_enable( cMap.watchDogIntervallMillis, 1 );
    return ( NO_ERR );
}

uint8_t watchDogUpdate( ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    watchdog_update( );
    return ( NO_ERR );
}

uint8_t watchDogCausedReboot( bool *reboot ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    return ( watchdog_caused_reboot( ));
}

// ??? where do we set the pfailPin ?

uint8_t setPfailHandler( PfailCallback functionId ) {

    // ??? to do ...
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// Timer section. The CDC library features a repeating timer with a microsecond resolution. Up to four timers
// can be configured, identified via a chosen resource Id. There are routines start and stop the timer and 
// allow to set a new limit. The PICO offers a high level function that schedules a repeating timer with the
// property of measuring the interval also from the start of the callback invocation. 
//
// ??? restrictions on resId ?
//------------------------------------------------------------------------------------------------------------
TimerResource *getTimerResource( uint8_t resId ) {

    if      ( timer_0.timerResId == resId ) return( &timer_0 );
    else if ( timer_1.timerResId == resId ) return( &timer_1 );
    else if ( timer_2.timerResId == resId ) return( &timer_2 );
    else if ( timer_3.timerResId == resId ) return( &timer_3 );
    else                                    return( nullptr );
}

uint8_t configureTimer( uint8_t resId, TimerCallback functionId ) {

    TimerResource *ptr = getTimerResource( resId );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    ptr -> timerResId    = resId;
    ptr -> timerCallback = functionId;
    return( NO_ERR );
}

uint8_t startRepeatingTimer( uint8_t resId, uint32_t val ) {

    TimerResource *ptr = getTimerResource( resId );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    add_repeating_timer_us( - limit, repeatingTimerAlarm, ptr, &ptr -> timerData );
    return( NO_ERR );
}

uint8_t stopRepeatingTimer( uint8_t resId ) {

    TimerResource *ptr = getTimerResource( resId );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    cancel_repeating_timer( &ptr -> timerData );
    return( NO_ERR );
}

uint8_t getRepeatingTimerLimit( uint8_t resId, uint32_t *val ) {

    TimerResource *ptr = getTimerResource( resId );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    *val = (uint32_t) ( - ptr -> timerData.delay_us );
    return ( NO_ERR );
}

uint8_t setRepeatingTimerLimit( uint8_t resId, uint32_t val ) {

    TimerResource *ptr = getTimerResource( resId );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    ptr -> timerData.delay_us = ((int64_t) - limit );
    return( NO_ERR );
}

uint8_t onTimerEvent( uint8_t resId, TimerCallback functionId ) {

    TimerResource *ptr = getTimerResource( resId );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    ptr -> timerCallback        = functionId;
    ptr -> timerData.user_data  = (void *) ptr;
    return( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// ADC section. The analog input channel represented by the pin is configured. At initialization, the ADC pin
// number is validated and the ADC subsystem is initialized. The PICO does an analog read in about 2us. This 
// is so fast, it does for our purpose make not much sense to implement an asynchronous option. Furthermore, 
// the ADC value scaled down to a 10-bit resolution.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t adcPin ) {

    AdcResource *ptr;

    if ( adcPin == cMap.adcPin_0 ) {

        ptr = &adc_0;
        ptr -> adcPin = adcPin;
        ptr -> adcNum = 0;
    }
     else if ( adcPin == cMap.adcPin_1 ) {

        ptr = &adc_1;
        ptr -> adcPin = adcPin;
        ptr -> adcNum = 1;
    }
    else if ( adcPin == cMap.adcPin_2 ) {

        ptr = &adc_2;
        ptr -> adcPin = adcPin;
        ptr -> adcNum = 2;
    }
    else return ( ADC_PIN_ERR );

    adc_init( );
    adc_gpio_init( adcPin );
    return ( NO_ERR );
}

uint8_t readAdc( uint8_t adcPin, uint16_t *val ) {

    AdcResource *ptr = nullptr;

    if      ( adc_0.adcPin == adcPin )  ptr = &adc_0;
    else if ( adc_1.adcPin == adcPin )  ptr = &adc_1;
    else if ( adc_2.adcPin == adcPin )  ptr = &adc_2;
    else                                return ( ADC_PIN_ERR );

    adc_select_input( ptr -> adcNum );
    *val = ( adc_read( ) >> 2 );
    return ( NO_ERR );
}

uint16_t getAdcRefVoltage( ) {

    return ( cMap.adcRefVoltageMillis );
}

uint16_t getAdcDigitRange( ) {

    return ( cMap.adcDigitRange );
}
 
//------------------------------------------------------------------------------------------------------------
// DIO section. A digital pin is the bread and butter hardware resource and can be an input or output pin. For
// inputs, an internal pull-up resistor can be set.There are a couple of interfaces. First the single pin
// read, write and toggle. Next are read and write mask routines which work on all IO pins at once. Note that
// no cross checking is done if the pins are used by other CDC functions. Finally there is a convenience 
// routine which write a pair of data. This is typically used for the H-Bridge control pins, which are set at
// the same time. 
//
// A GPIO pin can also have an attached interrupt handler. When we register a handler for a pin, there are 
// two different PICO lib routines to use. When there is no handler registered so far, we register the 
// common callback and store the particular GPIO handler in the handler table. Otherwise, we just store the
// handler and enable the GPIO pin for interrupts.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureDio( uint8_t dioPin, uint8_t pinMode ) {

    if ( ! validPin( dioPin, VALID_GPIO_PINS )) return ( DIO_PIN_ERR );

    gpio_init( dioPin );
    setGpioMode( dioPin, pinMode );
    return ( NO_ERR );
}

uint8_t registerDioCallback( uint8_t dioPin, uint8_t event, GpioCallback func ) {

    if ( dioPin <= MAX_INT_PIN ) {

        if ( dioIntHandlers.numOfHandlers == 0 ) 
            gpio_set_irq_enabled_with_callback( dioPin, mapGpioIntEvent( event ), true, gpioCallback );
        else
            gpio_set_irq_enabled( dioPin, mapGpioIntEvent( event ), true);
    
        dioIntHandlers.gpioIsrTable[ get_core_num( ) ][ dioPin ] = func;
        dioIntHandlers.numOfHandlers ++;
    }

    return( NO_ERR );
}

uint8_t unregisterDioCallback( uint8_t dioPin ) {

    if ( dioPin <= MAX_INT_PIN ) {

        if ( dioIntHandlers.gpioIsrTable[ get_core_num( ) ][ dioPin ] != nullptr ) {

            gpio_set_irq_enabled( dioPin, 0, false );
            dioIntHandlers.gpioIsrTable[ get_core_num( ) ][ dioPin ] = dummyIsrHandler;
            dioIntHandlers.numOfHandlers --;
        }
    }

    return( NO_ERR );
}

uint8_t readDio( uint8_t dioPin, bool *val ) {

    *val = gpio_get( dioPin );
    return ( NO_ERR );
}

uint8_t writeDio( uint8_t dioPin, bool val ) {

    gpio_put( dioPin, val );
    return ( NO_ERR );
}

uint8_t toggleDio( uint8_t dioPin ) {

    // ??? check that the pin is in output mode ?

    gpio_put( dioPin, ! gpio_get( dioPin ));
    return ( NO_ERR );
}

uint8_t writeDioPair( uint8_t dioPin1, bool val1, uint8_t dioPin2, bool val2 ) {

    uint32_t maskData = ( 1UL << dioPin1 ) | ( 1UL << dioPin2 );
    uint32_t valData  = (( val1 ) ? ( 1 << dioPin1 ) : 0 ) | (( val2 ) ? ( 1 << dioPin2 ) : 0 );

    gpio_put_masked( maskData, valData );
    return ( NO_ERR );
}

// ??? necessary ?

uint8_t readDioMask( uint32_t dioMask, uint32_t *val ) {

    *val = gpio_get_all( ) & dioMask;
    return ( NO_ERR );
}

uint8_t writeDioMask( uint32_t dioMask, uint32_t val ) {

    gpio_put_masked( dioMask, val );
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// PWM section. The PICO is quite flexible when it comes to PWM signals. We implement a simple PWM capability.
// There is the frequency which set during configuration and there is the write operation which set the duty
// cycle. The calculations are best described in the PICO C++ SDK. Note that although the PICO is quite 
// flexible, the wrap and phase parameters are set for the slice and not a single channel. The same is true
// for the signal inverter. This is normally not an issue unless you want to have separate values for PWM
// pins on the same slice. 
//
// The "writePwm" function will just manipulate the duty cycle. When we need to change the frequency we need
// to configure again. 
// 
//------------------------------------------------------------------------------------------------------------
PwmResource *getPwmResource( uint8_t pwmPin ) {

    if      ( pwm_0.pwmPin == pwmPin ) return( &pwm_0 );
    else if ( pwm_1.pwmPin == pwmPin ) return( &pwm_1 );
    else if ( pwm_2.pwmPin == pwmPin ) return( &pwm_2 );
    else if ( pwm_3.pwmPin == pwmPin ) return( &pwm_3 );
    else if ( pwm_4.pwmPin == pwmPin ) return( &pwm_4 );
    else if ( pwm_5.pwmPin == pwmPin ) return( &pwm_5 );
    else if ( pwm_6.pwmPin == pwmPin ) return( &pwm_6 );
    else if ( pwm_7.pwmPin == pwmPin ) return( &pwm_7 );
    else                               return( nullptr );
}

uint8_t configurePwm(   uint8_t     pwmPin, 
                        uint32_t    frequency, 
                        bool        phaseCorrect,
                        bool        inverted ) {

    PwmResource *pwm = nullptr;

    if      ( pwmPin == cMap.pwmPin_0 ) pwm = &pwm_0;
    else if ( pwmPin == cMap.pwmPin_1 ) pwm = &pwm_1;
    else if ( pwmPin == cMap.pwmPin_2 ) pwm = &pwm_2;
    else if ( pwmPin == cMap.pwmPin_3 ) pwm = &pwm_3;
    else if ( pwmPin == cMap.pwmPin_4 ) pwm = &pwm_4;
    else if ( pwmPin == cMap.pwmPin_5 ) pwm = &pwm_5;
    else if ( pwmPin == cMap.pwmPin_6 ) pwm = &pwm_6;
    else if ( pwmPin == cMap.pwmPin_7 ) pwm = &pwm_7;
    else                                return ( PWM_PIN_ERR );

    if ( phaseCorrect ) frequency = frequency * 2;

    uint32_t sysClock = clock_get_hz( clk_sys );
    uint32_t clkDiv   = sysClock / frequency / 4096 + ( sysClock % ( frequency * 4096 ) != 0 );
    if ( clkDiv / 16 == 0 ) clkDiv = 16;

    pwm -> pwmPin       = pwmPin;
    pwm -> frequency    = frequency;
    pwm -> wrap         = sysClock * 16 / clkDiv / frequency - 1;
    pwm -> inverted     = inverted;
    pwm -> phaseCorrect = phaseCorrect;
    pwm -> sliceNum     = pwm_gpio_to_slice_num( pwmPin );
    pwm -> channel      = pwm_gpio_to_channel( pwmPin );
   
    pwm_config pwmConfig = pwm_get_default_config( );
    gpio_set_function( pwm -> pwmPin, GPIO_FUNC_PWM );
    pwm_config_set_wrap( &pwmConfig, pwm -> wrap );
    pwm_config_set_phase_correct( &pwmConfig, phaseCorrect );
    pwm_config_set_output_polarity( &pwmConfig, pwm -> inverted, pwm -> inverted );

    pwm_init( pwm -> sliceNum, &pwmConfig, false );
    pwm_set_clkdiv_int_frac( pwm -> sliceNum, clkDiv / 16, clkDiv & 0xF );
    pwm_set_enabled( pwm -> sliceNum, true );

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {
   
        printf( "pin: % d, fPwm: % d, phase: % d, inverted: % d, " 
                "clkDiv: % d, wrap: %d, sliceNum: %d, channel: %d\n",
                pwmPin, frequency, phaseCorrect, inverted, 
                clkDiv, pwm -> wrap, pwm -> sliceNum, pwm -> channel );
    }

    return ( NO_ERR );
}

uint8_t writePwm( uint8_t pwmPin, uint8_t dutyCycle ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {
        
        printf( "Write PWM: pin: %d, duty: %d\n", pwmPin, dutyCycle );
    }

    PwmResource *pwm = getPwmResource( pwmPin );
    if ( pwm == nullptr ) return ( PWM_PIN_ERR );

#if 1
    pwm_set_gpio_level( pwm -> pwmPin, dutyCycle );
#else

    // will go away. -- not changed ....
    if ( dutyCycle == 0 ) {

        gpio_set_function( pwm -> pwmPin, GPIO_FUNC_SIO );
        gpio_set_dir( pwm -> pwmPin, GPIO_OUT );
        gpio_put( pwm -> pwmPin, 0 );
    }
    else if ( dutyCycle == 255 ) {

        gpio_set_function( pwm -> pwmPin, GPIO_FUNC_SIO );
        gpio_set_dir( pwm -> pwmPin, GPIO_OUT);
        gpio_put( pwm -> pwmPin, 1 );
    }
    else {

        gpio_set_function( pwm -> pwmPin, GPIO_FUNC_PWM );
        pwm_set_chan_level( pwm -> sliceNum, pwm -> channel, ( pwm -> wrap * dutyCycle / 255 ));
        pwm_set_enabled( pwm -> sliceNum, true );
    }
#endif

    return ( NO_ERR );
}

uint8_t writePwmPair(uint8_t pwmPin, uint8_t dutyCycleA, uint8_t dutyCycleB ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {
        
        printf( "Write PWM Pair: pin: %d, dutyA: %d, dutyB: %d\n", pwmPin, dutyCycleA, dutyCycleB );
    }

    PwmResource *pwm = getPwmResource( pwmPin );
    if ( pwm == nullptr ) return ( PWM_PIN_ERR );

    pwm_set_both_levels( pwm -> sliceNum, dutyCycleA, dutyCycleB );
    return ( NO_ERR );
}

uint8_t syncPwm( uint8_t pwmPin ) {

    PwmResource *pwm = getPwmResource( pwmPin );
    if ( pwm == nullptr ) return ( PWM_PIN_ERR );

    pwm_set_counter( pwm -> sliceNum, 0 );
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// UART section. The UART interface is primarily used for the RailCom Detector that sends a serial signal.
// So far, only the receiver portion is implemented because that is all what is needed for RailCom messages.
// There are two general categories. The first uses the PICO built-in UART hardware blocks. The second
// implements a software UART based on the PICO PIO blocks.
//
// There are three routines. The "startUartRead" will enable the UART and start reading bytes into the local
// buffer. The "stopUartRead" will then finish the byte collection and disable the UART again. Finally, the
// "getUartBuffer" routine will return the bytes received. Again, note that this is not a generic UART read
// interface.
//
// The work on the PIO based UART version has not started yet ... it will be needed for the quad block
// controller. Looking forward to it ...:-)
//
//------------------------------------------------------------------------------------------------------------
UartResource *getUartResource( uint8_t rxPin ) {

    if      ( uart_0.rxPin == rxPin ) return( &uart_0 );
    else if ( uart_1.rxPin == rxPin ) return( &uart_1 );
    else                              return( nullptr );
}

uint8_t configureUart( uint8_t rxPin, uint8_t txPin, uint32_t baudRate ) {

    UartResource *uart;

    if (( validPin( rxPin, VALID_UART_0_RX_PINS )) && ( validPin( txPin, VALID_UART_0_TX_PINS ))) {

        uart_0.rxPin        = rxPin;
        uart_0.txPin        = txPin;
        uart_0.dataBits     = 8;
        uart_0.stopBits     = 1;
        uart_0.parityMode   = UART_PARITY_NONE;
        uart_0.uartHw       = uart0;
        uart_0.uartIrq      = UART0_IRQ;
        uart                = &uart_0;
    }
    else if (( validPin( rxPin, VALID_UART_1_RX_PINS )) && ( validPin( txPin, VALID_UART_1_TX_PINS ))) {

        uart_1.rxPin        = rxPin;
        uart_1.txPin        = txPin;
        uart_1.dataBits     = 8;
        uart_1.stopBits     = 1;
        uart_1.parityMode   = UART_PARITY_NONE;
        uart_1.uartHw       = uart1;
        uart_1.uartIrq      = UART1_IRQ;
        uart                = &uart_0;
    }
    else return ( UART_PORT_ERR );

    uart_init( uart -> uartHw, baudRate );
    gpio_set_function( uart -> rxPin, GPIO_FUNC_UART );
    gpio_set_function( uart -> txPin, GPIO_FUNC_UART );
    uart_set_hw_flow( uart -> uartHw, false, false );
    uart_set_format(    uart -> uartHw, 
                        uart -> dataBits, 
                        uart -> stopBits, 
                        uart -> parityMode );
    uart_set_fifo_enabled( uart -> uartHw, false );

    if      ( uart -> uartIrq == UART0_IRQ ) irq_set_exclusive_handler( uart -> uartIrq, uartRxCallback0 );
    else if ( uart -> uartIrq == UART1_IRQ ) irq_set_exclusive_handler( uart -> uartIrq, uartRxCallback1 );

    irq_set_enabled( uart -> uartIrq, true );
    return ( NO_ERR );
}

uint8_t startUartRead( uint8_t rxPin ) {

    UartResource *uart = getUartResource( rxPin );
    if ( uart == nullptr ) return ( UART_PIN_ERR );

    uart_set_irq_enables( uart -> uartHw, true, false );
    uart -> rxBufIndex = 0;
    return ( NO_ERR );
}

uint8_t stopUartRead( uint8_t rxPin ) {

    UartResource *uart = getUartResource( rxPin );
    if ( uart == nullptr ) return ( UART_PIN_ERR );
    
    uart_set_irq_enables( uart -> uartHw, false, false );
    return ( NO_ERR );
}

uint8_t getUartBuffer( uint8_t rxPin, uint8_t *buf, uint8_t bufLen ) {

    UartResource *uart = getUartResource( rxPin );
    if ( uart == nullptr ) return ( UART_PIN_ERR );

    if (( uart -> rxBufIndex > 0 ) && ( bufLen > 0 )) {

        uint8_t i = 0;
        while (( i < uart -> rxBufIndex ) && ( i < bufLen )) {

            buf[ i ] = uart -> rxDataBuf[ i ];
            i++;
        }
    
        return ( i );
    }
    else return ( 0 );
}

//------------------------------------------------------------------------------------------------------------
// I2C Section. The PICO has two HW blocks for I2C interfaces. The interface implements a simple read and
// write access to an I2C element. There is a timeout to avoid waiting forever on an operation.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureI2C( uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate ) {

    I2CResource *i2c = nullptr;

    if ((( 1 << sclPin ) & VALID_I2C_0_SCL_PINS ) && (( 1 << sdaPin ) & VALID_I2C_0_SDA_PINS )) {

        i2c = &i2c_0;
        i2c -> i2cHw = i2c0;
    }
    else if ((( 1 << sclPin ) & VALID_I2C_1_SCL_PINS ) && (( 1 << sdaPin ) & VALID_I2C_1_SDA_PINS )) {

        i2c = &i2c_1;
        i2c -> i2cHw = i2c1;
    }
    else return ( I2C_PORT_ERR );

    i2c -> sclPin       = sclPin;
    i2c -> sdaPin       = sdaPin;
    i2c -> baudRate     = baudRate;
    i2c -> timeoutValMs = I2C_TIME_OUT_IN_MS;
   
    i2c_init( i2c -> i2cHw, i2c -> baudRate );
    i2c_set_slave_mode( i2c -> i2cHw, false, 0 );
    
    gpio_set_function( i2c -> sclPin, GPIO_FUNC_I2C );
    gpio_set_function( i2c -> sdaPin, GPIO_FUNC_I2C);
    gpio_pull_up( i2c -> sclPin );
    gpio_pull_up( i2c -> sdaPin );
    return ( NO_ERR );
}

uint8_t i2cRead( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    I2CResource *i2c = nullptr;

    if      (( i2c_0.sclPin == sclPin ) && ( i2c_0.configured )) i2c = &i2c_0;
    else if (( i2c_1.sclPin == sclPin ) && ( i2c_1.configured )) i2c = &i2c_1;
    else return ( I2C_PORT_ERR );

    auto ret = i2c_read_blocking_until( i2c -> i2cHw,
                                        i2cAdr,
                                        buf,
                                        len,
                                        stopBit,
                                        make_timeout_time_ms( i2c -> timeoutValMs ));

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "i2cRead: sclPin: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                sclPin, i2cAdr, buf, buf[0], buf[1], len, stopBit );

        if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {
            
            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C read, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C read, PICO timeout error\n" );
        }
    }
   
    if (( ret == PICO_ERROR_GENERIC ) || ( ret == PICO_ERROR_TIMEOUT )) return ( I2C_READ_ERR );
    return ( NO_ERR );
}

uint8_t i2cWrite( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {
        
        printf( "i2cWrite: sclPin: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                sclPin, i2cAdr, buf, buf[0], buf[1], len, stopBit );
    }

    I2CResource *i2c = nullptr;

    if      (( i2c_0.sclPin == sclPin ) && ( i2c_0.configured )) i2c = &i2c_0;
    else if (( i2c_1.sclPin == sclPin ) && ( i2c_1.configured )) i2c = &i2c_1;
    else return ( I2C_PORT_ERR );

    auto ret = i2c_write_blocking_until( i2c -> i2cHw,
                                         i2cAdr,
                                         buf,
                                         len,
                                         stopBit,
                                         make_timeout_time_ms( i2c -> timeoutValMs ));

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {

        if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {

            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C write, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C write, PICO timeout error\n" );
        }
    }
    
    if (( ret == PICO_ERROR_TIMEOUT) || ( ret == PICO_ERROR_GENERIC ) || ( ret != len )) return ( I2C_WRITE_ERR );
    return ( NO_ERR );
}

uint8_t i2cBusreset( uint8_t sclPin ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "I2C Bus reset, resId: %d\n", sclPin );
    }

    I2CResource *i2c = nullptr;

    if      (( i2c_0.sclPin == sclPin ) && ( i2c_0.configured )) i2c = &i2c_0;
    else if (( i2c_1.sclPin == sclPin ) && ( i2c_1.configured )) i2c = &i2c_1;
    else return ( I2C_PORT_ERR );

    uint8_t reset_cmd = 0x06;
    i2c_write_blocking( i2c -> i2cHw, 0x00, &reset_cmd, 1, false); 
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// CAN bus Section. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t pinRx, uint8_t pinTx, uint32_t baudRate, bool twoCores ) {

   // ??? what do we do with this call ... ?

    return( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// Print out the Config Structure.
//
// ??? this will for sure change ... we will print out the array of resources....
//------------------------------------------------------------------------------------------------------------
void printResourceMap( CdcResourceMap *cMap ) {

    printf( "CDC Resource Map\n" );





    printf( "DIO pins ( 0 .. 7 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
        cMap -> dioPin_0, cMap -> dioPin_1, cMap -> dioPin_2, cMap -> dioPin_3,
        cMap -> dioPin_4, cMap -> dioPin_5, cMap -> dioPin_6, cMap -> dioPin_7 );

    printf( "DIO pins ( 8 .. 15 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
        cMap -> dioPin_8, cMap -> dioPin_9, cMap -> dioPin_10, cMap -> dioPin_11,
        cMap -> dioPin_12, cMap -> dioPin_13, cMap -> dioPin_14, cMap -> dioPin_15 );

    printf( "ADC pins ( 0 .. 3 ): %2d %2d %2d %2d\n",
        cMap -> adcPin_0, cMap -> adcPin_1, cMap -> adcPin_2, cMap -> adcPin_3 );

    printf( "PWM pins ( 0 .. 7 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
        cMap -> pwmPin_0, cMap -> pwmPin_1, cMap -> pwmPin_2, cMap -> pwmPin_3,
        cMap -> pwmPin_4, cMap -> pwmPin_5, cMap -> pwmPin_6, cMap -> pwmPin_7 );

    printf( "UART RX pins ( 0 .. 3 ): %2d %2d %2d %2d\n",
        cMap -> uartRxPin_0, cMap -> uartRxPin_1 );

    printf( "UART TX pins ( 0 .. 3 ): %2d %2d %2d %2d\n",
        cMap -> uartTxPin_0, cMap -> uartTxPin_1 );

#if 0

    printf( "Pfail pin: %2d, ExtInt pin: %2d \n", ci -> PFAIL_PIN, ci -> EXT_INT_PIN );

    printf( "ActiveLed pin: %2d \n", ci -> ACTIVE_LED_PIN );


    printf( "NVM I2C Pins: SCL: %2d, SDA: %2d, I2C Root: 0x%x \n",
            ci -> NVM_I2C_SCL_PIN, ci -> NVM_I2C_SDA_PIN, ci -> NVM_I2C_ADR_ROOT );

    printf( "EXT I2C Pins: SCL: %2d, SDA: %2d, I2C Root: 0x%x \n",
            ci -> EXT_I2C_SCL_PIN, ci -> EXT_I2C_SDA_PIN, ci -> EXT_I2C_ADR_ROOT );   
#endif

    printf( "\n" );

}

}; // namespace CDC
