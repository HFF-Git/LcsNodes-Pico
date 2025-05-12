//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the the Raspberry Pi controller family hardware library code. The idea of this
// library is to shield the actual hardware of processor and board implementation from the upper layers but 
// still keep the flexibility and performance of the underlying hardware. 
//
// The library works with a concept of "resources". During startup, the resources are configured based on 
// data from the board resource descriptor. Most routines in the CDC layer use the resource id to access the
// particular hardware function.
//
// A historic note. The original LCS code was written for Atmega and Pico. With the complete shift to PICO,
// the CDC library just serves as an interface to the PICO functions. One day, we may see more different
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

#include "LcsCdcLibVersion.h"
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
uint16_t debugMask = CDC_DBG_CONFIG | CDC_DBG_SETUP;

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
const uint16_t  MAX_CPU_CORE                = 2;
const uint16_t  MAX_INT_PIN                 = 24;

const uint16_t  MAX_RESOURCE_ENTRIES        = 32;
const uint16_t  MAX_RES_NAME                = 64;
const uint8_t   MAX_UART_BUF_SIZE           = 8;

//------------------------------------------------------------------------------------------------------------
// Controller dependent code uses a set of hardware resource structures to control the controller hardware. 
// When a particular resource, e.g. an I2C channel, is configured all further access will use the resource 
// data for its operation. 
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// A timer resource. We need to keep the local timer instance data for the PICO.
//
//------------------------------------------------------------------------------------------------------------
struct TimerResource {

    uint32_t            timerVal;
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

    uint8_t   adcPin;
    uint8_t   adcNum;
};

//------------------------------------------------------------------------------------------------------------
// The GPIO resource is perhaps the most fundamental resource. It manages a HW pin. Optional, we can have
// two pins which then act as pair and are read from or written to simultaneously.
// 
//------------------------------------------------------------------------------------------------------------
struct GpioResource {

    uint8_t         dioPinA;
    uint8_t         dioPinB;
    uint8_t         pinMode;
    GpioCallback    handler;
};

//------------------------------------------------------------------------------------------------------------
// The PWM output resource manages a PWM configured output pin. We keep track of one or two pins, which must
// be on the same PWM slice. The idea is  that we use for H-Bridge control two output signals, which act as 
// a pair. 
//
//------------------------------------------------------------------------------------------------------------
struct PwmResource {

    uint8_t     pwmPinA;
    uint8_t     pwmPinB;
    uint32_t    frequency;
    uint        wrap;
    uint        channel;
    uint        sliceNum;
    bool        inverted;
    bool        phaseCorrect;
};

//------------------------------------------------------------------------------------------------------------
// UARTS are used to read in a serial stream from the RailCom detectors. There can be two hardware based UART
// resources. The resource also keeps a small buffer where the data is read into.
//
//------------------------------------------------------------------------------------------------------------
struct UartResource {

    uint8_t           rxPin;
    uint8_t           txPin;
    uint16_t          baudSetting;
    uint8_t           dataBits;
    uart_parity_t     parityMode;
    uint8_t           stopBits;
    int               uartIrq;

    volatile uint8_t  rxBufIndex;
    volatile uint8_t  rxDataBuf[ MAX_UART_BUF_SIZE ];

    uart_inst_t       *uartHw;
};

//------------------------------------------------------------------------------------------------------------
// The PICO features two I2C HW channels. The resource data contains the assigned GPIO pins, the baud rate 
// and a timeout. We also keep an I2C address root, which comes in handy for addressing chips with the same
// root address.
//
//------------------------------------------------------------------------------------------------------------
struct I2cResource {

    uint8_t     sclPin;
    uint8_t     sdaPin;
    uint8_t     i2cAdrRoot;
    uint32_t    baudRate;
    uint32_t    timeoutValMs;

    i2c_inst_t  *i2cHw;
};

//------------------------------------------------------------------------------------------------------------
// The CAN bus resource. Although our current controller does not feature a CAN bus hardware, the resource 
// describes the hardware elements needed. Currently, we use a software version based on a PIO program to 
// implement the CAN bus. 
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusResource {

    uint8_t         canPinRx;
    uint8_t         canPinTx;
    uint32_t        baudRate;
    uint32_t        canId;
    bool            twoCores;
};

//------------------------------------------------------------------------------------------------------------
// The resource map has an array of the resources. 
//
//------------------------------------------------------------------------------------------------------------
struct CdcResource {

    uint8_t type;
    uint8_t resId;

    union {

        TimerResource   timer;
        GpioResource    gpio;
        AdcResource     adc;
        PwmResource     pwm;
        UartResource    uart;
        I2cResource     i2c;
        CanBusResource  can;
    };
};

//------------------------------------------------------------------------------------------------------------
// The resource map is the central data structure to talk to the hardware. It is built at runtime startup
// using the resource descriptor map. Essentially it contain all the data from the resource descriptors and
// depending on the descriptor type the PICO data structures necessary.
//
//------------------------------------------------------------------------------------------------------------
struct CdcResourceMap {

    uint16_t            options;
    uint16_t            debugMask;
    uint16_t            boardId;     
    uint16_t            cFamily;
    uint16_t            cType;
    uint16_t            cpuCores;
    uint32_t            memorySize;
    uint32_t            eepromSize;
    uint32_t            watchDogIntervallMillis;
    uint16_t            adcRefVoltageMillis;
    uint16_t            adcDigitRange;
    char                name[ MAX_RES_NAME ];
    CdcResource         map[ MAX_RESOURCE_ENTRIES ];
};

//------------------------------------------------------------------------------------------------------------
// The interrupt table for the GPIO pin interrupts. The PICO has only one interrupt handler. We will allocate
// a table where an interrupt handler can be set for each HW pin. 
//
//------------------------------------------------------------------------------------------------------------
struct GpioIsrTable {

    uint16_t        numOfHandlers = 0;
    GpioCallback    gpioIsrTable[ MAX_CPU_CORE ][ MAX_INT_PIN + 1 ];
};

//------------------------------------------------------------------------------------------------------------
// File local variables. 
//
//------------------------------------------------------------------------------------------------------------
bool                    initialized = false;

GpioIsrTable            dioIntHandlers;
CdcResourceDescMap      dMap;
CdcResourceMap          rMap;

UartResource            *uartRes0;
UartResource            *uartRes1;

//------------------------------------------------------------------------------------------------------------
// "validPin" is called to check that a pin is in the correct number range, defined and matches the bitmask
// for the desired purpose. For example, configuring an I2C port will check that the two GPIO pins are
// indeed routable to an I2C HW block in the PICO.
//
//------------------------------------------------------------------------------------------------------------
bool validPin( uint8_t pin, uint32_t mask ) {

    if ( pin == UNDEFINED_PIN )     return ( true );
    if ( pin > MAX_PIN_NUM )        return ( false );
    return (( 1 << pin ) & mask );
}

//------------------------------------------------------------------------------------------------------------
// When no interrupt is configured for a GPIO pin, we set the table entry to a dummy handler. This way
// we do not have to check every time for a valid procedure label when we handle an interrupt.
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
// Set up the CDC resource map with default values.
//
//------------------------------------------------------------------------------------------------------------
void initResourceMap( CdcResourceMap *rMap ) {

    rMap -> options                     = 0;
    rMap -> debugMask                   = 0;
    rMap -> boardId                     = 0;
    rMap -> cFamily                     = CDC_CF_C_UNDEFINED;
    rMap -> cType                       = CDC_CF_C_UNDEFINED;
    rMap -> cpuCores                    = 1;
    rMap -> memorySize                  = 0;
    rMap -> eepromSize                  = 0;
    rMap -> watchDogIntervallMillis     = 2000;
    rMap -> adcRefVoltageMillis         = 3300;
    rMap -> adcDigitRange               = 1024; 
    rMap -> name[0 ]                    = 0;

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) rMap -> map[ i ].type = CDC_RT_UNDEFINED;
} 

//------------------------------------------------------------------------------------------------------------
// A resource descriptor is found by indexing into the resource descriptor map with index and resource type.
//
//------------------------------------------------------------------------------------------------------------
CdcResourceDesc *lookupResourceDesc( uint8_t rNum, uint8_t type ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );
    if ( dMap.map[ rNum ].type == type ) return( &dMap.map[ rNum ] );
    else return( nullptr );
}

//------------------------------------------------------------------------------------------------------------
// A resource is found by indexing into the resource map with index and resource type.
//
//------------------------------------------------------------------------------------------------------------
CdcResource *lookupResource( uint8_t rNum, uint8_t type ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( nullptr );
    if ( rMap.map[ rNum ].type = type ) return( nullptr );
    return( &rMap.map[ rNum ] );
}

//------------------------------------------------------------------------------------------------------------
// The configuration routines will allocate the corresponding entry in the resource map. When the entry is 
// found but of a different type, it is an error. When there is no entry yet, the entry is initialized with
// the type and can be used for configuration.
//
//------------------------------------------------------------------------------------------------------------
CdcResource *allocateResourceType( uint8_t rNum, uint8_t type ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );
    if ( rMap.map[ rNum ].type == CDC_RT_UNDEFINED ) {

        rMap.map[ rNum ].type = type;
        return ( &rMap.map[ rNum ] );
    }
    else if ( rMap.map[ rNum ].type == type ) {

        return ( &rMap.map[ rNum ] );
    }
   else return ( nullptr );
}

//------------------------------------------------------------------------------------------------------------
// The PICO uses a set of constants to describe the GPIO pin interrupt type. We map our CDC interrupt types to 
// the PICO GPIO_IRQ_xxx types.
//
//------------------------------------------------------------------------------------------------------------
uint32_t mapCdcIntEvent( uint8_t event ) {

    switch ( event ) {

        case CDC_EVT_LOW:      return ( GPIO_IRQ_LEVEL_LOW );
        case CDC_EVT_HIGH:     return ( GPIO_IRQ_LEVEL_HIGH );
        case CDC_EVT_FALL:     return ( GPIO_IRQ_EDGE_FALL );
        case CDC_EVT_RISE:     return ( GPIO_IRQ_EDGE_RISE );
        case CDC_EVT_CHANGE:   return ( GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL );
        default:               return ( 0 );
    }
}

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
// interrupt routine, so we feature an array of handlers where a handler for a GPIO pin can be registered. 
// 
// The UART handlers will handle receive interrupts of the UART hardware blocks. There is no easy way to get
// to the resource structure where the input buffer is. We therefore maintain two global variables in this 
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
        if ( uartRes0 -> rxBufIndex < MAX_UART_BUF_SIZE ) uartRes0 -> rxDataBuf[ uartRes0 -> rxBufIndex++ ] = ch;
    }
}

void uartRxCallback1( ) {

    while ( uart_is_readable( uart1 )) {

        uint8_t ch = uart_getc( uart1 );
        if ( uartRes1 -> rxBufIndex < MAX_UART_BUF_SIZE ) uartRes1 -> rxDataBuf[ uartRes1 -> rxBufIndex++ ] = ch;
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
// Version Info.
//
//------------------------------------------------------------------------------------------------------------
uint32_t getVersion( ) {

    return ( CDC_LIB_VERSION );
}

uint32_t getPatchLevel( ) {
    
    return ( CDC_LIB_PATCH_LEVEL );
}

//------------------------------------------------------------------------------------------------------------
// CDC library setup. The "init" routine will ready the CDC library and keep a copy of the descriptor map
// which will be used for the setup. The init routine can be called more than once without a problem.
//
//------------------------------------------------------------------------------------------------------------
uint8_t cdcInit( CdcResourceDescMap *dMapPtr ) {

    dMap = *dMapPtr;
    if ( ! initialized ) {

        initIsrTable( );
    }

    return( NO_ERR );
}
 
 //------------------------------------------------------------------------------------------------------------
 // "getResourceMap" will return a pointer to the configured resource map. This is typically the map that was
 // created with the data from the resource descriptor map.
 //
 //------------------------------------------------------------------------------------------------------------
 CdcResourceMap  *getResourceMap( ) {

    return ( &rMap );
 }

//------------------------------------------------------------------------------------------------------------
// "fatalError" is the error communication method when we cannot get anything to work. The Raspberry Pi PICO
// has a small Led on the board. We will use this LED to "blink" an error code. There are up to eight codes. 
// The sequence is as follows:
//
//    repeat forever:
//
//    - 1s ON, 0.5s 0FF
//    - for ( int i = 0; i < n; i++ ) { 0.5s ON; 0.5s OFF; }
//
// The only way to get out of this loop is then to reset the board. Fatal errors are hopefully not many. One
// obvious one is when we cannot detect the NVM and thus know nothing about the board.
//
// If we have a console, we attempt to first write an error message to the console before looping.
//
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
// Simple timestamp and sleep functions.
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
// Console IO section. We set up the stdio via the USB connector. As part of the cdcInit call, the console 
// configure call should be done rather early, so that we can print out debug messages. In normal LCS node
// operation there is no USB connected. Detecting a connection helps to decide whether we can report an error
// or need to resort to a fatal error call at startup. 
//
// There are two basic ways to detect an USB connection. The first is to simply check if there is power on 
// the USB port. The PICO features an internal GPIO pin for this purpose. Using this method still does not
// mean that we have a computer connected to the USB, but just that there is a cable with power. Well, good
// enough for us. The second method truly detects that there is a USB host connected. This check is provided
// via the PICO libraries which in turn use the tinyUSB library. However, there could be a timing problem
// where the USB stack is not ready yet and we conclude wrongly that there is no USB connection. For now, 
// let's rather go with the crude approach to check if there is power on the VBUS pin, at the risk that there
// is just power on the USB connector and no data.
//
// Finally, there is a routine to get a character for the command interfaces. Since the function just reads
// in a character, optionally with a timeout how long to wait for any input.
//
// PS: The USB check way would be "return ( stdio_usb_connected( ));" instead of the GPIO check.
//
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
// just like any other resource but does not have a resource ID number. Although most of the controller 
// parameters are fixed we take the values from the descriptor array, since some of them are not that easy 
// to get. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t getControllerFamily( uint16_t *family ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *family = rMap.cFamily;
    return ( NO_ERR );
}

uint8_t getControllerChip( uint16_t *ctl ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *ctl = rMap.cType;
    return ( NO_ERR );
}

uint8_t getBoardId( uint16_t *bId ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *bId = rMap.boardId;
    return ( NO_ERR );
}

uint8_t getChipMemSize( uint32_t *size ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *size = rMap.memorySize;
    return ( NO_ERR );
}

uint8_t getChipNvmSize( uint32_t *size ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *size = rMap.eepromSize;
    return ( NO_ERR );
}

uint8_t getChipCpuFrequency( uint32_t *frequency ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    *frequency = clock_get_hz( clk_sys );
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// Watchdog facility.
//
//------------------------------------------------------------------------------------------------------------
uint8_t watchDogEnable( bool enable ) {

    if ( ! initialized ) return( NOT_INITIALZED_ERR );
    watchdog_enable( rMap.watchDogIntervallMillis, 1 );
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

//------------------------------------------------------------------------------------------------------------
// Timer section. The CDC library features a repeating timer with a microsecond resolution. Up to four timers
// can be configured, identified via their resource Id. There are routines to start and stop the timer as well 
// as to allow to set a new limit. The PICO offers a high level function that schedules a repeating timer with
// the property of measuring the interval also from the start of the callback invocation. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureTimer( uint8_t rNum, TimerCallback functionId ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    if ( rNum >= CDC_RN_FIRST_USER_RN ) return( RES_NUM_ERR );

    CdcResourceDescTimer *dPtr = (CdcResourceDescTimer *) lookupResourceDesc( rNum, CDC_RT_TIMER );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    TimerResource *ptr = (TimerResource *) allocateResourceType( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    ptr -> timerVal         = dPtr -> timerVal;
    ptr -> timerCallback    = functionId;
    return( NO_ERR );
}

uint8_t startRepeatingTimer( uint8_t rNum, uint32_t val ) {

    TimerResource *ptr = (TimerResource *) lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    add_repeating_timer_us( - limit, repeatingTimerAlarm, ptr, &ptr -> timerData );
    return( NO_ERR );
}

uint8_t stopRepeatingTimer( uint8_t rNum ) {

    TimerResource *ptr = (TimerResource *) lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    cancel_repeating_timer( &ptr -> timerData );
    return( NO_ERR );
}

uint8_t getRepeatingTimerLimit( uint8_t rNum, uint32_t *val ) {

    TimerResource *ptr = (TimerResource *) lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    *val = (uint32_t) ( - ptr -> timerData.delay_us );
    return ( NO_ERR );
}

uint8_t setRepeatingTimerLimit( uint8_t rNum, uint32_t val ) {

    TimerResource *ptr = (TimerResource *) lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    ptr -> timerData.delay_us = ((int64_t) - limit );
    return( NO_ERR );
}

// ??? needed, look at configure timer...
uint8_t onTimerEvent( uint8_t rNum, TimerCallback functionId ) {

    TimerResource *ptr = (TimerResource *) lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    ptr -> timerCallback        = functionId;
    ptr -> timerData.user_data  = (void *) ptr;
    return( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// ADC section. The analog input channel represented by the pin is configured. At initialization, the ADC pin
// number is validated and the ADC subsystem is initialized. The PICO does an analog read in about 2us. This 
// is so fast, it does for our purpose make not much sense to implement an asynchronous option. Furthermore, 
// the ADC value scaled down to a 10-bit resolution. The PICO support up to three ADC pins at the dedicated
// HW pins numbers 26, 27 and 28. They also need to be mapped an ADC select number for selecting the ADC
// hardware.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    if ( rNum < CDC_RN_FIRST_USER_RN  ) return( RES_NUM_ERR );

    CdcResourceDescAdc *dPtr = (CdcResourceDescAdc *) lookupResourceDesc( rNum, CDC_RT_ADC );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    return( configureAdc( rNum, dPtr -> adcPin, dPtr -> adcNum ));
}

uint8_t configureAdc( uint8_t rNum, uint8_t adcPin, uint8_t adcNum ) {

    if ( rNum < CDC_RN_FIRST_USER_RN  ) return( RES_NUM_ERR );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    
    AdcResource *rPtr = (AdcResource *) allocateResourceType( rNum, CDC_RT_ADC );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( adcPin == 26 ) {

        rPtr -> adcPin = 26;
        rPtr -> adcNum = 0;
    }
    else  if ( adcPin == 27 ) {

        rPtr -> adcPin = 27;
        rPtr -> adcNum = 1;
    }
    else  if ( adcPin == 28 ) {

        rPtr -> adcPin = 28;
        rPtr -> adcNum = 2;
    }
    else return ( ADC_PIN_ERR );

    adc_init( );
    adc_gpio_init( rPtr -> adcPin );
    return ( NO_ERR );
}

uint8_t readAdc( uint8_t rNum, uint16_t *val ) {

    AdcResource *rPtr = (AdcResource *) lookupResource( rNum, CDC_RT_ADC );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    adc_select_input( rPtr -> adcNum );
    *val = ( adc_read( ) >> 2 );
    return ( NO_ERR );
}

uint16_t getAdcRefVoltage( ) {

    return ( dMap.adcRefVoltageMillis );
}

uint16_t getAdcDigitRange( ) {

    return ( dMap.adcDigitRange );
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
// ??? we support only setting a handler for pinA ?
//------------------------------------------------------------------------------------------------------------
uint8_t configureDio( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
   
    CdcResourceDescGpio *dPtr = (CdcResourceDescGpio *) lookupResourceDesc( rNum, CDC_RT_GPIO );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    return( configureDio( rNum, dPtr -> pinA, dPtr -> pinB, dPtr -> pinMode ));
}

uint8_t configureDio( uint8_t rNum, uint8_t pinA, uint8_t pinB, uint8_t mode ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    
    GpioResource *rPtr = (GpioResource *) allocateResourceType( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    rPtr -> dioPinA = pinA;
    rPtr -> dioPinB = pinB;
    rPtr -> handler = nullptr;
 
    if ( mode == CDC_DIO_DEFAULT )  rPtr -> pinMode = mode % 4;
    else                            rPtr -> pinMode = mode;

    if ( ! validPin( rPtr -> dioPinA, VALID_GPIO_PINS )) return ( DIO_PIN_ERR );
    if ( ! validPin( rPtr -> dioPinB, VALID_GPIO_PINS )) return ( DIO_PIN_ERR );

    gpio_init( rPtr -> dioPinA );
    setGpioMode( rPtr -> dioPinA, rPtr -> pinMode );

    if ( rPtr -> dioPinB != UNDEFINED_PIN ) gpio_init( rPtr -> dioPinB );
    if ( rPtr -> dioPinB != UNDEFINED_PIN ) setGpioMode( rPtr -> dioPinB, rPtr -> pinMode );
    return ( NO_ERR );
}

uint8_t registerDioCallback( uint8_t rNum, uint8_t event, GpioCallback func ) {

    GpioResource *rPtr = (GpioResource *) lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( rPtr -> dioPinA <= MAX_INT_PIN ) {

        if ( dioIntHandlers.numOfHandlers == 0 ) 
            gpio_set_irq_enabled_with_callback( rPtr -> dioPinA, mapCdcIntEvent( event ), true, gpioCallback );
        else
            gpio_set_irq_enabled( rPtr -> dioPinA, mapCdcIntEvent( event ), true);
    
        dioIntHandlers.gpioIsrTable[ get_core_num( ) ][ rPtr -> dioPinA ] = func;
        dioIntHandlers.numOfHandlers ++;
    }

    return( NO_ERR );
}

uint8_t unregisterDioCallback( uint8_t rNum ) {

    GpioResource *rPtr = (GpioResource *) lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( rPtr -> dioPinA <= MAX_INT_PIN ) {

        if ( dioIntHandlers.gpioIsrTable[ get_core_num( ) ][ rPtr -> dioPinA ] != nullptr ) {

            gpio_set_irq_enabled(  rPtr -> dioPinA, 0, false );
            dioIntHandlers.gpioIsrTable[ get_core_num( ) ][ rPtr -> dioPinA ] = dummyIsrHandler;
            dioIntHandlers.numOfHandlers --;
        }
    }

    return( NO_ERR );
}

uint8_t readDio( uint8_t rNum, bool *valA, bool *valB ) {

    GpioResource *rPtr = (GpioResource *) lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    *valA = gpio_get( rPtr -> dioPinA );
    if (( valB != nullptr ) && ( rPtr -> dioPinB != UNDEFINED_PIN )) *valB = gpio_get( rPtr -> dioPinB );
    return ( NO_ERR );
}

uint8_t writeDio( uint8_t rNum, bool valA, bool valB ) {

    GpioResource *rPtr = (GpioResource *) lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( rPtr -> dioPinB == UNDEFINED_PIN ) {

        gpio_put( rPtr -> dioPinA, valA );
    }
    else {

        uint32_t maskData = ( 1UL << rPtr -> dioPinA ) | ( 1UL << rPtr -> dioPinB );
        uint32_t valData  = (( valA ) ? ( 1 << rPtr -> dioPinA ) : 0 ) | 
                            (( valB ) ? ( 1 << rPtr -> dioPinB ) : 0 );
        gpio_put_masked( maskData, valData );
    }

    return ( NO_ERR );
}

uint8_t toggleDio( uint8_t rNum ) {

    GpioResource *rPtr = (GpioResource *) lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( rPtr -> pinMode == CDC_DIO_OUT ) {

        gpio_put( rPtr -> dioPinA, ! gpio_get( rPtr -> dioPinA ));
        return ( NO_ERR );
    }
    else return( DIO_MODE_ERR );
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
uint8_t configurePwm( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
   
    CdcResourceDescPwm *dPtr = (CdcResourceDescPwm *) lookupResourceDesc( rNum, CDC_RT_PWM );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    return( configurePwm( rNum, dPtr -> pinA, dPtr -> pinB, dPtr -> frequency ));
}

uint8_t configurePwm( uint8_t rNum, uint8_t pinA, uint8_t pinB, uint32_t frequency ) {

    if ( rNum < CDC_RN_FIRST_USER_RN  ) return( RES_NUM_ERR );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    
    PwmResource *rPtr = (PwmResource *) allocateResourceType( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    rPtr -> pwmPinA         = pinA;
    rPtr -> pwmPinB         = pinB;
    rPtr -> phaseCorrect    = true;
    rPtr -> inverted        = false;     
    rPtr -> sliceNum        = pwm_gpio_to_slice_num( rPtr -> pwmPinA );
    rPtr -> frequency       = frequency;              

    if ( rPtr -> pwmPinB != UNDEFINED_PIN ) {

        if ( pwm_gpio_to_slice_num( rPtr -> pwmPinA ) != pwm_gpio_to_slice_num( rPtr -> pwmPinB ))
        return( PWM_PIN_ERR );
    }

    if ( rPtr -> phaseCorrect ) rPtr -> frequency = rPtr -> frequency * 2;

    uint32_t sysClock = clock_get_hz( clk_sys );
    uint32_t clkDiv   = sysClock / rPtr -> frequency / 4096 + ( sysClock % ( rPtr -> frequency * 4096 ) != 0 );
    if ( clkDiv / 16 == 0 ) clkDiv = 16;

    rPtr -> wrap        = sysClock * 16 / clkDiv / rPtr -> frequency - 1;
   
    pwm_config pwmConfig = pwm_get_default_config( );
    gpio_set_function( rPtr -> pwmPinA, GPIO_FUNC_PWM );
    if ( rPtr -> pwmPinB != UNDEFINED_PIN )  gpio_set_function( rPtr -> pwmPinB, GPIO_FUNC_PWM );
    pwm_config_set_wrap( &pwmConfig, rPtr -> wrap );
    pwm_config_set_phase_correct( &pwmConfig, rPtr -> phaseCorrect );
    pwm_config_set_output_polarity( &pwmConfig, rPtr -> inverted, rPtr -> inverted );

    pwm_init( rPtr -> sliceNum, &pwmConfig, false );
    pwm_set_clkdiv_int_frac( rPtr -> sliceNum, clkDiv / 16, clkDiv & 0xF );
    pwm_set_enabled( rPtr -> sliceNum, true );

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {
   
        printf( "pinA: % d, pinB: %d, fPwm: % d, phase: % d, inverted: % d, " 
                "clkDiv: % d, wrap: %d, sliceNum: %d\n",
                rPtr -> pwmPinA, rPtr -> pwmPinB, rPtr -> frequency, rPtr -> phaseCorrect, 
                rPtr -> inverted, clkDiv, rPtr -> wrap, rPtr -> sliceNum );
    }

    return ( NO_ERR );
}

uint8_t setPwmFrequency( uint8_t rNum, uint32_t frequency ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {
        
        printf( "Set PWMFrequency: rNum: %d, f: %d\n", rNum, frequency );
    }

    PwmResource *rPtr = (PwmResource *) lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );


    return( configurePwm( rNum, rPtr -> pwmPinA, rPtr -> pwmPinB, frequency ));
}

uint8_t writePwm( uint8_t rNum, uint8_t dutyCycleA, uint8_t dutyCycleB ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {
        
        printf( "Write PWM: rNum: %d, dutyA: %d, dutyB: %d\n", rNum, dutyCycleA, dutyCycleB );
    }

    PwmResource *rPtr = (PwmResource *) lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( rPtr -> pwmPinB != UNDEFINED_PIN ) {

        pwm_set_gpio_level( rPtr -> pwmPinA, dutyCycleA );
    }
    else {

        pwm_set_both_levels( rPtr -> sliceNum, dutyCycleA, dutyCycleB );
    }

    return ( NO_ERR );
}

uint8_t syncPwm( uint8_t rNum ) {

    PwmResource *rPtr = (PwmResource *) lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    pwm_set_counter( rPtr -> sliceNum, 0 );
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
//------------------------------------------------------------------------------------------------------------
uint8_t configureUart( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
   
    CdcResourceDescUart *dPtr = (CdcResourceDescUart *) lookupResourceDesc( rNum, CDC_RT_UART );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    return( configureUart( rNum, dPtr -> rxPin, dPtr -> txPin, dPtr -> baudRate ));
}

uint8_t configureUart( uint8_t rNum, uint8_t rxPin, uint8_t txPin, uint32_t baudRate ) {

    if ( rNum < CDC_RN_FIRST_USER_RN  ) return( RES_NUM_ERR );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    
    UartResource *rPtr = (UartResource *) allocateResourceType( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    rPtr -> rxPin           = rxPin;
    rPtr -> txPin           = txPin;
    rPtr -> baudSetting     = baudRate;
    rPtr -> dataBits        = 8;
    rPtr -> parityMode      = UART_PARITY_NONE;
    rPtr -> stopBits        = 1;

    if (( validPin( rPtr -> rxPin, VALID_UART_0_RX_PINS )) && 
        ( validPin( rPtr -> txPin, VALID_UART_0_TX_PINS ))) {

        rPtr -> uartHw  = uart0;
        rPtr -> uartIrq = UART0_IRQ;
        uartRes0        = rPtr;
    }
    else if (( validPin( rPtr -> rxPin, VALID_UART_1_RX_PINS )) && 
             ( validPin( rPtr -> txPin, VALID_UART_1_TX_PINS ))) {

        rPtr -> uartHw  = uart1;
        rPtr -> uartIrq = UART1_IRQ;
        uartRes1        = rPtr;
    }
    else return ( UART_PORT_ERR );

    uart_init( rPtr -> uartHw, rPtr -> baudSetting );
    gpio_set_function( rPtr -> rxPin, GPIO_FUNC_UART );
    if ( rPtr -> txPin != UNDEFINED_PIN ) gpio_set_function( rPtr -> txPin, GPIO_FUNC_UART );
    uart_set_hw_flow( rPtr -> uartHw, false, false );
    uart_set_format(    rPtr -> uartHw, 
                        rPtr -> dataBits, 
                        rPtr -> stopBits, 
                        rPtr -> parityMode );
    uart_set_fifo_enabled( rPtr -> uartHw, false );

    if      ( rPtr -> uartIrq == UART0_IRQ ) irq_set_exclusive_handler( rPtr -> uartIrq, uartRxCallback0 );
    else if ( rPtr -> uartIrq == UART1_IRQ ) irq_set_exclusive_handler( rPtr -> uartIrq, uartRxCallback1 );

    irq_set_enabled( rPtr -> uartIrq, true );
    return ( NO_ERR );
}

uint8_t startUartRead( uint8_t rNum ) {

    UartResource *rPtr = (UartResource *) lookupResource( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    uart_set_irq_enables( rPtr -> uartHw, true, false );
    rPtr -> rxBufIndex = 0;
    return ( NO_ERR );
}

uint8_t stopUartRead( uint8_t rNum ) {

    UartResource *rPtr = (UartResource *) lookupResource( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );
    
    uart_set_irq_enables( rPtr -> uartHw, false, false );
    return ( NO_ERR );
}

uint8_t getUartBuffer( uint8_t rNum, uint8_t *buf, uint8_t bufLen ) {

    UartResource *rPtr = (UartResource *) lookupResource( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if (( rPtr -> rxBufIndex > 0 ) && ( bufLen > 0 )) {

        uint8_t i = 0;
        while (( i < rPtr -> rxBufIndex ) && ( i < bufLen )) {

            buf[ i ] = rPtr -> rxDataBuf[ i ];
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
uint8_t configureI2C( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
   
    CdcResourceDescI2c *dPtr = (CdcResourceDescI2c *) lookupResourceDesc( rNum, CDC_RT_I2C );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    return( configureI2C(   rNum, dPtr -> sclPin, dPtr -> sdaPin, 
                            dPtr -> baudRate, dPtr -> i2cTimeoutMs ));
}

uint8_t configureI2C( uint8_t rNum, uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate, uint32_t timeoutVal ) {

    if ( rNum < CDC_RN_FIRST_USER_RN  ) return( RES_NUM_ERR );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    
    I2cResource *rPtr = (I2cResource *) allocateResourceType( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    rPtr -> sclPin          = sclPin;
    rPtr -> sdaPin          = sdaPin;
    rPtr -> baudRate        = baudRate;
    rPtr -> timeoutValMs    = timeoutVal;


    if ((( 1 << rPtr -> sclPin ) & VALID_I2C_0_SCL_PINS ) && (( 1 << rPtr -> sdaPin ) & VALID_I2C_0_SDA_PINS )) {

        rPtr -> i2cHw = i2c0;
    }
    else if ((( 1 << rPtr -> sclPin ) & VALID_I2C_1_SCL_PINS ) && (( 1 << rPtr -> sdaPin ) & VALID_I2C_1_SDA_PINS )) {

        rPtr -> i2cHw = i2c1;
    }
    else return ( I2C_PORT_ERR );

    i2c_init( rPtr -> i2cHw, rPtr -> baudRate );
    i2c_set_slave_mode( rPtr -> i2cHw, false, 0 );
    
    gpio_set_function( rPtr -> sclPin, GPIO_FUNC_I2C );
    gpio_set_function( rPtr -> sdaPin, GPIO_FUNC_I2C);
    gpio_pull_up( rPtr -> sclPin );
    gpio_pull_up( rPtr -> sdaPin );
    return ( NO_ERR );
}

uint8_t i2cRead( uint8_t rNum, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    auto ret = i2c_read_blocking_until( rPtr -> i2cHw,
                                        i2cAdr,
                                        buf,
                                        len,
                                        stopBit,
                                        make_timeout_time_ms( rPtr -> timeoutValMs ));

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "i2cRead: rNum: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                rNum, i2cAdr, buf, buf[0], buf[1], len, stopBit );

        if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {
            
            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C read, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C read, PICO timeout error\n" );
        }
    }
   
    if (( ret == PICO_ERROR_GENERIC ) || ( ret == PICO_ERROR_TIMEOUT )) return ( I2C_READ_ERR );
    return ( NO_ERR );
}

uint8_t i2cWrite( uint8_t rNum, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {
        
        printf( "i2cWrite: rNum: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                rNum, i2cAdr, buf, buf[0], buf[1], len, stopBit );
    }

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    auto ret = i2c_write_blocking_until( rPtr -> i2cHw,
                                         i2cAdr,
                                         buf,
                                         len,
                                         stopBit,
                                         make_timeout_time_ms( rPtr -> timeoutValMs ));

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_PWM )) {

        if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {

            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C write, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C write, PICO timeout error\n" );
        }
    }
    
    if (( ret == PICO_ERROR_TIMEOUT) || ( ret == PICO_ERROR_GENERIC ) || ( ret != len )) return ( I2C_WRITE_ERR );
    return ( NO_ERR );
}

uint8_t i2cBusreset( uint8_t rNum ) {

    if (( debugMask & CDC_DBG_CONFIG ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "I2C Bus reset, rNum: %d\n", rNum );
    }

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    uint8_t reset_cmd = 0x06;
    i2c_write_blocking( rPtr -> i2cHw, 0x00, &reset_cmd, 1, false); 
    return ( NO_ERR );
}


uint8_t i2cGetSclPin( uint8_t rNum ) {

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( UNDEFINED_PIN );

    return( rPtr -> sclPin );
}

uint8_t i2cGetSdaPin( uint8_t rNum ) {

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( UNDEFINED_PIN );

    return( rPtr -> sdaPin );
}

uint8_t i2cGetBaudrate( uint8_t rNum ) {

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( 0 );

    return( rPtr -> baudRate );
}

//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t scanI2CBus( uint8_t rNum ) {

    I2cResource *rPtr = (I2cResource *) lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    if ( rPtr -> sclPin == UNDEFINED_PIN ) {

        printf( "I2C bus for rNum: %d not configured\n", rNum );
        return ( NO_ERR );
    }
    else {

        uint8_t rStat     = 0;
        uint8_t i2cAdr    = 0;
        uint8_t nDevices  = 0;
        uint8_t buf       = 0;
    
        for ( i2cAdr = 1; i2cAdr < 127; i2cAdr++ ) {
    
            rStat = i2cRead( rNum, i2cAdr, &buf, 1 );
          
            if ( rStat == 0 ) {
    
                printf( "I2C device found at i2cAdr 0x%x\n", i2cAdr );
                nDevices ++;
            }
        }
    
        if ( nDevices == 0 )  printf( "No I2C devices found\n" );
        else                  printf( "Scan done\n" );
    }

    return( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// CAN bus Section. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
   
    CdcResourceDescCanBus *dPtr = (CdcResourceDescCanBus *) lookupResourceDesc( rNum, CDC_RT_CAN_BUS );
    if ( dPtr == nullptr ) return( RES_NUM_ERR );
   
    return( configureCanBus( rNum, dPtr -> rxPin, dPtr -> txPin, dPtr -> baudRate, dPtr -> twoCores ));
}

uint8_t configureCanBus( uint8_t rNum, uint8_t rxPin, uint8_t txPin, uint32_t baudRate, bool twoCores ) {

    if ( rNum < CDC_RN_FIRST_USER_RN  ) return( RES_NUM_ERR );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return( RES_NUM_ERR );
    
    CanBusResource *rPtr = (CanBusResource *) allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return( RES_NUM_ERR );

    rPtr -> canPinRx = rxPin;
    rPtr -> canPinTx = txPin;
    rPtr -> baudRate = baudRate;
    rPtr -> twoCores = twoCores;

    return( NO_ERR );
}

uint8_t canGetRxPin( uint8_t rNum ) {

    CanBusResource *rPtr = (CanBusResource *) allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return( UNDEFINED_PIN );

    return( rPtr -> canPinRx );
}

uint8_t canGetTxPin( uint8_t rNum ) {

    CanBusResource *rPtr = (CanBusResource *) allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return( UNDEFINED_PIN );

    return( rPtr -> canPinTx );
}

uint32_t canGetBaudrate( uint8_t rNum ) {

    CanBusResource *rPtr = (CanBusResource *) allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return( 0 );

    return( rPtr -> baudRate );
}

bool canGetTwoCores( uint8_t rNum ) {

    CanBusResource *rPtr = (CanBusResource *) allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return( false );

    return( rPtr -> twoCores );
}

//------------------------------------------------------------------------------------------------------------
// Print out the Resource Descriptor Map.
//
//------------------------------------------------------------------------------------------------------------
void printResourceDescMap( CdcResourceDescMap *dMap ) {

    printf( "CDC Resource Descriptor Map for:\n" );
    printf( "%s\n\n", dMap -> name );

    printf( "Options: 0x%4x\n", dMap -> options );
    printf( "Debug Mask: 0x%4x\n", dMap -> debugMask );
    printf( "Controller Family: %d, Chip: %d\n", dMap -> cFamily, dMap -> cType );
    printf( "Controller Cores: %d, Mem: %d, EEPROM: %d\n", dMap -> cpuCores, dMap -> memorySize, dMap -> eepromSize );
    printf( "WatchDog Interval (MS): %d\n", dMap -> watchDogIntervallMillis );
    printf( "ADC Ref Voltage: %d, Digit range: %d\n", dMap -> adcRefVoltageMillis, dMap -> adcDigitRange ); 

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResourceDesc *dPtr = &dMap -> map[ i ];

        printf( "(%2d): ", i );

        switch ( dPtr ->type ) {

            case CDC_RT_ADC: {

                printf( "ADC: pin: %d, select: %d\n", 
                        dPtr -> adc.adcPin, dPtr -> adc.adcNum );

            } break;

            case CDC_RT_GPIO: {

                printf( "GPIO: pinA: %d, pinB: %d, mode: %d\n", 
                        dPtr -> gpio.pinA, dPtr -> gpio.pinB, dPtr -> gpio.pinMode );

            } break;

            case CDC_RT_PWM: {

                printf( "PWM: pinA: %d, pinB: %d, fPwm: %d\n",
                        dPtr ->pwm.pinA,  dPtr ->pwm.pinB,  dPtr ->pwm.frequency );

            } break;

            case CDC_RT_UART: {

                printf( "UART: rxPin: %d, txPin: %d, baudRate: %d\n",
                    dPtr -> uart.rxPin,  dPtr -> uart.txPin,  dPtr -> uart.baudRate );

            } break;

            case CDC_RT_I2C: {

                printf( "I2C: sclPin: %d, sdaPin: %d, baudRate: %d, i2cRoot: 0x2x, timeout(MS): %d\n",
                        dPtr -> i2c.sclPin, dPtr -> i2c.sdaPin, dPtr -> i2c.baudRate, dPtr -> i2c.i2cTimeoutMs );

            } break;

            case CDC_RT_CAN_BUS: {

                printf( "CAN: rxPin: %d, txPin: %d, baudRate: %d, twoCores: %d\n",
                        dPtr -> can.rxPin, dPtr -> can.txPin, dPtr -> can.baudRate, dPtr -> can.twoCores );
            }

            case CDC_RT_UNDEFINED: break;

            default: printf( "Unknown type: %d\n", i );
        }
    }

    printf( "\n" );
} 

//------------------------------------------------------------------------------------------------------------
// Print out the Resource Map.
//
//------------------------------------------------------------------------------------------------------------
void printResourceMap( ) {

    printf( "CDC Resource Map for:\n" );
    printf( "%s\n\n", rMap.name );

    printf( "Options: 0x%4x\n", rMap.options );
    printf( "Debug Mask: 0x%4x\n", rMap.debugMask );
    printf( "Controller Family: %d, Chip: %d\n", rMap.cFamily, rMap.cType );
    printf( "Controller Cores: %d, Mem: %d, EEPROM: %d\n", rMap.cpuCores, rMap.memorySize, rMap.eepromSize );
    printf( "WatchDog Interval (MS): %d\n", rMap.watchDogIntervallMillis );
    printf( "ADC Ref Voltage: %d, Digit range: %d\n", rMap.adcRefVoltageMillis, rMap.adcDigitRange ); 

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResource *rPtr = &rMap.map[ i ];

        printf( "(%2d): ", i );

        switch ( rPtr ->type ) {

            case CDC_RT_ADC: {

                printf( "ADC: pin: %d, select: %d\n", 
                        rPtr -> adc.adcPin, rPtr -> adc.adcNum );

            } break;

            case CDC_RT_GPIO: {

                printf( "GPIO: pinA: %d, pinB: %d, mode: %d\n", 
                        rPtr -> gpio.dioPinA, rPtr -> gpio.dioPinB, rPtr -> gpio.pinMode );

            } break;

            case CDC_RT_PWM: {

                uint8_t     pwmPinA;
                uint8_t     pwmPinB;
                uint32_t    frequency;
                uint        wrap;
                uint        sliceNum;
                bool        inverted;
                bool        phaseCorrect;

                printf( "PWM: pinA: %d, pinB: %d, fPwm: %d, wrap: %d, slice: %d, invert: %d, phase: %d\n",
                        rPtr ->pwm.pwmPinA,  rPtr ->pwm.pwmPinB,  rPtr ->pwm.frequency,
                        rPtr -> pwm.sliceNum, rPtr -> pwm.inverted, rPtr -> pwm.phaseCorrect );

            } break;

            case CDC_RT_UART: {

                printf( "UART: rxPin: %d, txPin: %d, baudRate: %d, dataBits: %d, parity: %d, stopBits: %d\n",
                        rPtr -> uart.rxPin,  rPtr -> uart.txPin,  rPtr -> uart.baudSetting,
                        rPtr -> uart.dataBits, rPtr -> uart.parityMode, rPtr -> uart.stopBits );

            } break;

            case CDC_RT_I2C: {

                printf( "I2C: sclPin: %d, sdaPin: %d, baudRate: %d, i2cRoot: 0x2x, timeout(MS): %d\n",
                        rPtr -> i2c.sclPin, rPtr -> i2c.sdaPin, rPtr -> i2c.baudRate, 
                        rPtr -> i2c.i2cAdrRoot, rPtr -> i2c.timeoutValMs );

            } break;

            case CDC_RT_CAN_BUS: {

                uint8_t         canPinRx;
                uint8_t         canPinTx;
                uint32_t        baudRate;
                uint32_t        canId;
                bool            twoCores;

                printf( "CAN: rxPin: %d, txPin: %d, baudRate: %d, canId: 0x4x, twoCores: %d\n",
                        rPtr -> can.canPinRx, rPtr -> can.canPinTx, rPtr -> can.baudRate, 
                        rPtr -> can.canId, rPtr -> can.twoCores );
            }

            case CDC_RT_UNDEFINED: break;

            default: printf( "Unknown type: %d\n", i );
        }
    }

    printf( "\n" );
}

}; // namespace CDC
