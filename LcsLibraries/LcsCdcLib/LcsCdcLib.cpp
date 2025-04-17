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

#include "LcsBoardDescriptors.h"
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
uint16_t debugMask = DBG_CONFIG | DBG_PWM;

//------------------------------------------------------------------------------------------------------------  
// The CDC Library version data.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t CDC_LIB_MAJOR_VERSION = 1;
const uint8_t CDC_LIB_MINOR_VERSION = 0;

//------------------------------------------------------------------------------------------------------------
// Valid pin mapping for the Raspberry PI Pico board. We construct a set of bitmask for the pin numbers.
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
// ??? these elements should go where ?
//----------------------------------------------------------------------------------------------------------
const uint32_t CHIP_MEM_SIZE                = 264 * 1024;         // ??? not true for RP2350
const uint32_t CHIP_NVM_SIZE                = 0;

const uint16_t ADC_DIGIT_RANGE              = 1024;
const uint16_t ADC_REF_VOLTAGE_MILLI_VOLT   = 3300;

const uint8_t  MAX_UART_BUF_SIZE            = 8;

const uint32_t I2C_FREQUENCY                = 100 * 1000;
const uint32_t I2C_TIME_OUT_IN_MS           = 250;

const uint32_t SPI_FREQUENCY                = 10000000L;

const uint16_t  MAX_CPU_CORE                = 2;
const uint16_t  MAX_INT_PIN                 = 24;

const uint16_t  MAX_RESOURCE_ENTRIES        = 64;

//------------------------------------------------------------------------------------------------------------
// Controller dependent code uses a set of hardware resource structures to control the controller hardware. 
// When a particular resource, e.g. an I2C channel, is configured all further access is done with a handle
// to it. The resource structure takes care of isolating the controller hardware from the runtime library 
// and user firmware. There is a counter part to the resource structure, which contains the configured
// parameters values for that resource. These values are used in the initial configuration process. At any 
// time later, the configure routine for a given resource can be called to change these values, but not the
// type.
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// The controller specific attributes.
//
//------------------------------------------------------------------------------------------------------------
struct ControllerResource {

    ControllerFamily        controllerFamily;
    ControllerChip          controllerChip;
    uint32_t                memorySize;
    uint32_t                internalNvmSize;
    uint16_t                cpuCores;
    uint32_t                watchDogIntervallMillis;
    uint16_t                adcRefVoltage;
    uint16_t                adcDigitRange;
    uint8_t                 ledPin;
    uint8_t                 pFailPin;
};

//------------------------------------------------------------------------------------------------------------
// Timer specific attributes.
//
//------------------------------------------------------------------------------------------------------------
struct TimerResource {

    uint32_t            timerIntervalMicros;
    TimerCallback       timerCallback;
    repeating_timer_t   timerData;
};

//------------------------------------------------------------------------------------------------------------
// ADC specific fields. The PICO has 3 externally available adc inputs. We describe for each ADC input the
// defined digit range and the reference voltage.
//
//------------------------------------------------------------------------------------------------------------
struct AdcResource {

    uint8_t     adcPin;
    uint8_t     adcNum;
};

//------------------------------------------------------------------------------------------------------------
// GPIO specific attributes. Almost all IO pins can be used as general purposes IO pins. In addition to
// a single DIO pin, there is also the option to write a pair. This is primarily used for H-Bridge control
// signals to set them simultaneously. Single DIO pins also feature the option to configure an interrupt
// handler.
//
//------------------------------------------------------------------------------------------------------------
struct GpioResource {

    uint8_t         pinA;
    uint8_t         pinB;
    uint8_t         pinMode;
    GpioCallback    handler;
};

//------------------------------------------------------------------------------------------------------------
// A PWM output resource. GPIO pins can also be used as PWM output pins. The PWM output related data is kept
// in the PWM structures. The PICO features a set of PWM slices, each of which has two channels. We have 
// a resource defined for a single PWM output and also a PWM pair where the two channels can be set 
// simultaneously. The pair feature is used for our H-Bridge PWM signal, where one pin is the PWM signal 
// while the other pin is held low. 
//
//------------------------------------------------------------------------------------------------------------
struct PwmResource {

    uint8_t     pinA;
    uint8_t     pinB;
    uint        wrap;
    bool        inverted;
    uint        channelA;
    uint        channelB;
    uint        sliceNum;
};

//------------------------------------------------------------------------------------------------------------
// A UART resource. UARTS are used to read in a serial stream from the RailCom detectors. The PICO features 
// two hardware based UART blocks. The resource also keeps a small buffer where the data is read into. 
//
//------------------------------------------------------------------------------------------------------------
struct UartResource {

    uint8_t             rxPin;
    uint8_t             txPin;
    uint16_t            baudSetting;
    uint8_t             dataBits;
    uart_parity_t       parityMode;
    uint8_t             stopBits;
    int                 uartIrq;
    uint8_t             uartMode;
    volatile uint8_t    rxBufIndex;
    volatile uint8_t    rxDataBuf[ MAX_UART_BUF_SIZE ];
    uart_inst_t         *uartHw;
};

//------------------------------------------------------------------------------------------------------------
// The I2C specific data. The PICO features two HW resources of an I2C port. 
//
//------------------------------------------------------------------------------------------------------------
struct I2CResource {

    uint8_t         sclPin;
    uint8_t         sdaPin;
    uint32_t        baudRate;
    uint32_t        timeoutValMs;
    i2c_inst_t      *i2cHw;
};

//------------------------------------------------------------------------------------------------------------
// The CAN bus resource. Although our current controller does not feature a CAN bus, the resource described 
// the hardware elements needed. Currently, we use a software version based on a PIO program to implement the
// CAN bus. 
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusResource {

    uint8_t         canPinH;
    uint8_t         canPinL;
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
    uint8_t         gpioHandleTab[ MAX_CPU_CORE ][ MAX_INT_PIN + 1 ]; 
    GpioCallback    gpioIsrTable[ MAX_CPU_CORE ][ MAX_INT_PIN + 1 ];
};

//------------------------------------------------------------------------------------------------------------
// Controller hardware is configured via "Resources". An resource groups a hardware peripheral function with 
// all settings and hardware pins it may need. These resources are then referred by the CDC routines with an
// index into the resource array, called a handle. This array is populated during initial configuration. An
// application can  locate this index by using the assigned name.
//
//------------------------------------------------------------------------------------------------------------
struct CdcResource {

    CdcResourceType type;
   
    union {

        ControllerResource  ctl;
        TimerResource       timer;
        GpioResource        gpio;
        PwmResource         pwm;
        UartResource        uart;
        AdcResource         adc;
        I2CResource         i2c;
        CanBusResource      can;
    };
};

//------------------------------------------------------------------------------------------------------------
// The resource map. The map is allocated at startup. Each configured resource will have an entry in this map.
//
//------------------------------------------------------------------------------------------------------------
struct CdcResourceMap {
    
    CdcResource map[ MAX_RESOURCE_ENTRIES ];
};

//------------------------------------------------------------------------------------------------------------
// Local variables. 
//
//------------------------------------------------------------------------------------------------------------
bool                initialized     = false;
UartResource        *uartResource0  = nullptr;
UartResource        *uartResource1  = nullptr;

CdcResource         resMap[ MAX_RESOURCE_ENTRIES ];
GpioIsrTable        cdcIntHandlers;

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

    for ( uint16_t i = 0; i < MAX_CPU_CORE; i++ ) {

        for ( uint16_t j = 0; j < MAX_INT_PIN; j++ ) {

            cdcIntHandlers.gpioIsrTable[ i ][ j ] = dummyIsrHandler;
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

    // ??? add the handle somehow ... this is what we are interested in ...
    // ??? our interrupt handler routine could accept the handle vs. the pin. 
    // ??? we map the the HW pin set to the handle.

    cdcIntHandlers.gpioIsrTable[ get_core_num( )][ gpioPin ] ( gpioPin, mapPicoGpioEvent( event ));
}

void uartRxCallback0( ) {

    while ( uart_is_readable( uart0 )) {

        uint8_t ch = uart_getc( uart0 );

        if ( uartResource0 -> rxBufIndex < MAX_UART_BUF_SIZE ) 
            uartResource0 -> rxDataBuf[ uartResource0 -> rxBufIndex++ ] = ch;
    }
}

void uartRxCallback1( ) {

    while ( uart_is_readable( uart1 )) {

        uint8_t ch = uart_getc( uart1 );

        if ( uartResource1 -> rxBufIndex < MAX_UART_BUF_SIZE ) 
            uartResource1 -> rxDataBuf[ uartResource1 -> rxBufIndex++ ] = ch;
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

//------------------------------------------------------------------------------------------------------------
// The resource map is the dynamic structure that contains all the configured resources. It is just an array
// of resource entries. All CDC routines pass a resId to index into this array. Each access will check 
// whether CDC was initialized and whether the resource type matches. If the type is undefined, the entry
// is not used, if not the type must match. Allocation first checks if the entry is already used in which 
// case the type must match.
//
//------------------------------------------------------------------------------------------------------------
void initResourceMap( ) {

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        resMap[ i ].type = CDC_IT_UNDEFINED;
    }
}

CdcResource *allocateResourceById( uint8_t resId, CdcResourceType rTyp ) {

    if ( ! initialized ) return( nullptr );

    CdcResource *entry = &resMap[ resId % MAX_RESOURCE_ENTRIES ];

    if ( entry -> type == CDC_IT_UNDEFINED ) {

        entry -> type = rType;
        return ( entry );
    }
    else return (( entry -> type == rTyp ) ? entry : nullptr );
}

CdcResource *getResourceById( uint8_t resId, CdcResourceType rTyp ) {

    if ( ! initialized ) return( nullptr );

    CdcResource *entry = &resMap[ resId % MAX_RESOURCE_ENTRIES ];
    return (( entry -> type == rTyp ) ? entry : nullptr );
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
uint8_t cdcInit(  ) {

    if ( ! initialized ) {

        initResourceMap( );
        initIsrTable( );
    }

    return ( NO_ERR );
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
//------------------------------------------------------------------------------------------------------------
void fatalError( uint8_t n ) {

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
// "fatalErrorMsg" will result in a fatal error, but we attempt to first write an error message to the 
// console.
//
//------------------------------------------------------------------------------------------------------------
void fatalErrorMsg( char *str, uint8_t n, uint8_t rStat ) {

    if ( isConsoleConnected( )) printf( "Fatal Error: %d: %s, rStat: %d\n", n, str, rStat );
    fatalError( n );
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
uint8_t configureController(    ControllerFamily    family, 
                                ControllerChip      processor,
                                uint32_t            memorySize,
                                uint32_t            internalNvmSize,
                                uint32_t            watchDogMillis,
                                uint16_t            adcRefVoltage, 
                                uint16_t            adcDigitRange,
                                uint8_t             ledPin,
                                uint8_t             pFailPin ) {

    CdcResource *entry = allocateResourceById( 0, CDC_IT_CONTROLLER );
    if ( entry == nullptr ) return( INVALID_RES_ID_ERR );
   
    entry -> ctl.controllerFamily           = family;
    entry -> ctl.controllerChip             = processor;
    entry -> ctl.memorySize                 = memorySize;
    entry -> ctl.internalNvmSize            = internalNvmSize;
    entry -> ctl.watchDogIntervallMillis    = watchDogMillis;
    entry -> ctl.adcRefVoltage              = adcRefVoltage;
    entry -> ctl.adcDigitRange              = adcDigitRange;
    entry -> ctl.ledPin                     = ledPin;
    entry -> ctl.pFailPin                   = pFailPin;   
    return ( NO_ERR );
}

uint8_t getFamily( ControllerFamily *family ) {

    *family =resMap[ 0 ].ctl.controllerFamily;
    return ( NO_ERR );
}

uint8_t getChipMemSize( uint32_t *size ) {

    *size = resMap[ 0 ].ctl.memorySize;
    return ( NO_ERR );
}

uint8_t getChipNvmSize( uint32_t *size ) {

    *size = CHIP_NVM_SIZE;
    return ( NO_ERR );
}

uint8_t getCpuFrequency( uint32_t *frequency ) {

    *frequency = clock_get_hz( clk_sys );
    return ( NO_ERR );
}

uint8_t watchDogEnable( bool enable ) {

    watchdog_enable( resMap[ 0 ].ctl.watchDogIntervallMillis, 1 );
    return ( NO_ERR );
}

uint8_t watchDogUpdate( ) {

    watchdog_update( );
    return ( NO_ERR );
}

uint8_t watchDogCausedReboot( bool *reboot ) {

    return ( watchdog_caused_reboot( ));
}

//------------------------------------------------------------------------------------------------------------
// Timer section. The CDC library features one generic repeating timer with a microsecond resolution. The
// routines start and stop the timer and allow to set a new limit. The PICO offers a high level function that
// schedules a repeating timer with the property of measuring the interval also from the start of the
// callback invocation. This is exactly what we need to implement the tick interrupt for the DCC signal state
// machine. The "setRepeatingTimerLimit" function will adjust the timer limit counter while the timer already
// is counting toward a limit. Note that the timer option that already start the next round while the timer
// interrupt handler executes is specified by using negative limit values. The timer functionality also
// offers two timestamp routines to get the number of milliseconds and number of microseconds since system
// start.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureTimer( uint8_t resId, TimerCallback functionId ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return( INVALID_RES_ID_ERR );

    entry -> timer.timerIntervalMicros  = 0;
    entry -> timer.timerCallback        = functionId;
    return( NO_ERR );
}

uint8_t startRepeatingTimer( uint8_t resId, uint32_t val ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    int64_t limit = val;
    add_repeating_timer_us( - limit, repeatingTimerAlarm, nullptr, &entry -> timer.timerData );
    return( NO_ERR );
}

uint8_t stopRepeatingTimer( uint8_t resId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    cancel_repeating_timer( &entry -> timer.timerData );
    return( NO_ERR );
}

uint8_t getRepeatingTimerLimit( uint8_t resId, uint32_t *val ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    *val = (uint32_t) ( - entry -> timer.timerData.delay_us );
    return ( NO_ERR );
}

uint8_t setRepeatingTimerLimit( uint8_t resId, uint32_t val ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    int64_t limit = val;
    entry -> timer.timerData.delay_us = ((int64_t) - limit );
    return( NO_ERR );
}

uint8_t onTimerEvent( uint8_t resId, TimerCallback functionId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    entry -> timer.timerCallback          = functionId;
    entry -> timer.timerData.user_data    = (void *) entry;
    return( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// ADC section. The analog input channel represented by the pin is configured. At initialization, the ADC pin
// number is validated and the ADC subsystem initialized. The PICO does an analog read in about 2us. This is
// so fast, it does for our purpose make not much sense to implement an asynchronous option. Furthermore, the
// ADC value scaled down to a 10-bit resolution.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t resId, uint8_t adcPin ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_TIMER );
    if ( entry == nullptr ) return( INVALID_RES_ID_ERR );

    // ??? is there a PICO lib function to map pin to num ?
    entry -> adc.adcPin = adcPin;

    if      ( adcPin == 26 ) entry -> adc.adcNum = 0;
    else if ( adcPin == 27 ) entry -> adc.adcNum = 1;
    else if ( adcPin == 28 ) entry -> adc.adcNum = 2;

    adc_init( );
    adc_gpio_init( adcPin );
    return ( NO_ERR );
}

uint8_t readAdc( uint8_t resId, uint16_t *val ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_ADC );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    adc_select_input( entry -> adc.adcNum );
    *val = ( adc_read( ) >> 2 );
    
    return ( NO_ERR );
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
uint8_t configureDio( uint8_t resId, uint8_t pinA, uint8_t pinB, uint8_t pinMode ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_ADC );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    entry -> gpio.pinA         = pinA;
    entry -> gpio.pinB         = pinB;
    entry -> gpio.pinMode      = pinMode;
    
    gpio_init( pinA );
    setGpioMode( pinA, pinMode );
    if ( pinB != UNDEFINED_PIN )  setGpioMode( pinA, pinMode );
    if ( pinB != UNDEFINED_PIN )  setGpioMode( pinB, pinMode );
    return ( NO_ERR );
}

uint8_t registerDioCallback( uint8_t resId, uint8_t event, GpioCallback func ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_GPIO );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    entry -> gpio.handler = func;

    if ( entry -> gpio.pinA <= MAX_INT_PIN ) {

        if ( cdcIntHandlers.numOfHandlers == 0 ) {

            gpio_set_irq_enabled_with_callback( entry -> gpio.pinA, mapGpioIntEvent( event ), true, gpioCallback );

        }
        else { 

            gpio_set_irq_enabled( entry -> gpio.pinA, mapGpioIntEvent( event ), true );

        }
    
        cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ entry -> gpio.pinA ] = func;
        cdcIntHandlers.numOfHandlers ++;
    }
    else return ( DIO_INT_HANDLER_ERR );

    if ( entry -> gpio.pinB <= MAX_INT_PIN ) {

        if ( cdcIntHandlers.numOfHandlers == 0 ) 
            gpio_set_irq_enabled_with_callback( entry -> gpio.pinB, mapGpioIntEvent( event ), true, gpioCallback );
        else
            gpio_set_irq_enabled( entry -> gpio.pinB, mapGpioIntEvent( event ), true );
    
        cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ entry -> gpio.pinB ] = func;
        cdcIntHandlers.numOfHandlers ++;
    }
    else return ( DIO_INT_HANDLER_ERR );

    return( NO_ERR );
}

uint8_t unregisterDioCallback( uint8_t resId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_GPIO );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    entry -> gpio.handler = nullptr;

    if (( entry -> gpio.pinA != UNDEFINED_PIN ) && ( entry -> gpio.pinA <= MAX_INT_PIN )) {

        if ( cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ entry -> gpio.pinA ] != nullptr ) {

            gpio_set_irq_enabled( entry -> gpio.pinA, 0, false );
            cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ entry -> gpio.pinA ] = dummyIsrHandler;
            cdcIntHandlers.numOfHandlers --;
        }
    }

    if (( entry -> gpio.pinB != UNDEFINED_PIN ) && ( entry -> gpio.pinB <= MAX_INT_PIN )) {

        if ( cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ entry -> gpio.pinB ] != nullptr ) {

            gpio_set_irq_enabled( entry -> gpio.pinB, 0, false );
            cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ entry -> gpio.pinB ] = dummyIsrHandler;
            cdcIntHandlers.numOfHandlers --;
        }
    }

    return( NO_ERR );
}

uint8_t readDio( uint8_t resId, bool *val ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_GPIO );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    *val = gpio_get( entry -> gpio.pinA );
    return ( NO_ERR );
}

uint8_t writeDio( uint8_t resId, bool val ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_GPIO );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    gpio_put( entry -> gpio.pinA, val );
    return ( NO_ERR );
}

uint8_t toggleDio( uint8_t resId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_GPIO );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    if ( entry -> gpio.pinMode != CDC_DIO_OUT ) return( DIO_MODE_ERR );

    gpio_put( entry -> gpio.pinA, ! gpio_get( entry -> gpio.pinA ));
    if ( entry -> gpio.pinB != UNDEFINED_PIN ) gpio_put( entry -> gpio.pinB, ! gpio_get( entry -> gpio.pinB ));
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// PWM section. The PICO is quite flexible when it comes to PWM signals. We implement a simple PWM capability.
// There is the frequency which set during configuration and there is the write operation which set the duty
// cycle. The calculations are best described in the PICO C++ SDK. Our PWM configuration supports the 
// simultaneous level setting of the two associated pins. This way we can model a two pin PWM signal, where 
// either pin contains the PWM signal and  the other is set to low. Note that although the PICO is quite 
// flexible, the wrap and phase parameters are set for the slice and not a single channel. The same is true
// for the signal inverter. This is normally not an issue unless you want to have separate frequencies, phase
// mode and so on. 
//
// The "writePwm" function will just manipulate the duty cycle. When we need to change the frequency we need
// to configure again. 
// 
//------------------------------------------------------------------------------------------------------------
uint8_t configurePwm(   uint8_t     resId, 
                        uint8_t     pinA, 
                        uint8_t     pinB, 
                        uint32_t    frequency, 
                        bool        phaseCorrect,
                        bool        inverted ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_PWM );
    if ( entry == nullptr ) return( RES_ID_ALLOCATE_ERR );

    if ( pinB != UNDEFINED_PIN ) {

        if ( pwm_gpio_to_slice_num( pinA ) != ( pwm_gpio_to_slice_num( pinB ))) return ( PWM_PIN_ERR );
    }

    if ( phaseCorrect ) frequency = frequency * 2;

    uint32_t sysClock = clock_get_hz( clk_sys );
    uint32_t clkDiv   = sysClock / frequency / 4096 + ( sysClock % ( frequency * 4096 ) != 0 );
    if ( clkDiv / 16 == 0 ) clkDiv = 16;

    entry -> pwm.pinA         = pinA;
    entry -> pwm.pinB         = pinB;
    entry -> pwm.wrap         = sysClock * 16 / clkDiv / frequency - 1;
    entry -> pwm.inverted     = inverted;

    entry -> pwm.sliceNum     = pwm_gpio_to_slice_num( pinA );
    entry -> pwm.channelA     = pwm_gpio_to_channel( pinA );
    if ( pinB != UNDEFINED_PIN ) entry -> pwm.channelB = pwm_gpio_to_channel( pinB );

    pwm_config pwmConfig = pwm_get_default_config( );
    gpio_set_function( entry -> pwm.pinA, GPIO_FUNC_PWM );
    if ( entry -> pwm.pinB != UNDEFINED_PIN ) gpio_set_function( entry -> pwm.pinB, GPIO_FUNC_PWM );
    pwm_config_set_wrap( &pwmConfig, entry -> pwm.wrap );
    pwm_config_set_phase_correct( &pwmConfig, phaseCorrect );
    pwm_config_set_output_polarity( &pwmConfig, entry -> pwm.inverted, entry -> pwm.inverted );
    pwm_init( entry -> pwm.sliceNum, &pwmConfig, false );
    pwm_set_clkdiv_int_frac( entry -> pwm.sliceNum, clkDiv / 16, clkDiv & 0xF );
    pwm_set_enabled( entry -> pwm.sliceNum, true );

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_PWM )) {
   
        printf( "pinA: % d, pinB: %d, fPwm: % d, phase: % d, inverted: % d, " 
                "clkDiv: % d, wrap: %d, sliceNum: %d, channelA: %d, channelB: %d\n",
                entry -> pwm.pinA, entry -> pwm.pinB, frequency, phaseCorrect, inverted, 
                clkDiv, entry -> pwm.wrap, entry -> pwm.sliceNum, entry -> pwm.channelA, entry -> pwm.channelB );
    }

    return ( NO_ERR );
}

uint8_t writePwm( uint8_t resId, uint8_t dutyCycleA, uint8_t dutyCycleB ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_PWM )) {
        
        printf( "Write PWM: resId: %d, dutyA: %d, dutyB: %d\n", resId, dutyCycleA, dutyCycleB );
    }

    CdcResource *entry = getResourceById( resId, CDC_IT_PWM );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

  
#if 1
    if ( entry -> pwm.pinB != UNDEFINED_PIN ) 

        pwm_set_both_levels( entry -> pwm.sliceNum, dutyCycleA, dutyCycleB );
    else 
        pwm_set_gpio_level( entry -> pwm.pinA, dutyCycleA );
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

uint8_t syncPwm( uint8_t resId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_PWM );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    pwm_set_counter( entry -> pwm.sliceNum, 0 );
    
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
uint8_t configureUart( uint8_t resId, uint8_t rxPin, uint8_t txPin, uint32_t baudRate ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_UART );
    if ( entry == nullptr ) return( RES_ID_ALLOCATE_ERR );

    if (( validPin( rxPin, VALID_UART_0_RX_PINS )) && ( validPin( txPin, VALID_UART_0_TX_PINS ))) {

        entry -> uart.rxPin         = rxPin;
        entry -> uart.txPin         = txPin;
        entry -> uart.dataBits      = 8;
        entry -> uart.stopBits      = 1;
        entry -> uart.parityMode    = UART_PARITY_NONE;
        entry -> uart.uartHw        = uart0;
        entry -> uart.uartIrq       = UART0_IRQ;

        uartResource0               = &entry -> uart;
    }
    else if (( validPin( rxPin, VALID_UART_1_RX_PINS )) && ( validPin( txPin, VALID_UART_1_TX_PINS ))) {
       
        entry -> uart.rxPin         = rxPin;
        entry -> uart.txPin         = txPin;
        entry -> uart.dataBits      = 8;
        entry -> uart.stopBits      = 1;
        entry -> uart.parityMode    = UART_PARITY_NONE;
        entry -> uart.uartHw        = uart1;
        entry -> uart.uartIrq       = UART1_IRQ;

        uartResource1               =  &entry -> uart;
    }
    else return ( UART_PORT_ERR );

    uart_init( entry -> uart.uartHw, baudRate );
    gpio_set_function( entry -> uart.rxPin, GPIO_FUNC_UART );
    gpio_set_function( entry -> uart.txPin, GPIO_FUNC_UART );
    uart_set_hw_flow( entry -> uart.uartHw, false, false );
    uart_set_format( entry -> uart.uartHw, 
                     entry -> uart.dataBits, 
                     entry -> uart.stopBits, 
                     entry -> uart.parityMode );
    uart_set_fifo_enabled( entry -> uart.uartHw, false );

    if      ( entry -> uart.uartIrq == UART0_IRQ ) irq_set_exclusive_handler( entry -> uart.uartIrq, uartRxCallback0 );
    else if ( entry -> uart.uartIrq == UART1_IRQ ) irq_set_exclusive_handler( entry -> uart.uartIrq, uartRxCallback1 );

    irq_set_enabled( entry -> uart.uartIrq, true );
    return ( NO_ERR );
}

uint8_t startUartRead( uint8_t resId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_UART );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    uart_set_irq_enables( entry -> uart.uartHw, true, false );
    entry -> uart.rxBufIndex = 0;
    return ( NO_ERR );
}

uint8_t stopUartRead( uint8_t resId ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_UART );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );
    
    uart_set_irq_enables( entry -> uart.uartHw, false, false );
    return ( NO_ERR );
}

uint8_t getUartBuffer( uint8_t resId, uint8_t *buf, uint8_t bufLen ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_UART );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    if (( entry -> uart.rxBufIndex > 0 ) && ( bufLen > 0 )) {

        uint8_t i = 0;
        while (( i < entry -> uart.rxBufIndex ) && ( i < bufLen )) {

            buf[ i ] = entry -> uart.rxDataBuf[ i ];
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
uint8_t configureI2C( uint8_t resId, uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_I2C );
    if ( entry == nullptr ) return( RES_ID_ALLOCATE_ERR );

    if ((( 1 << sclPin ) & VALID_I2C_0_SCL_PINS ) && (( 1 << sdaPin ) & VALID_I2C_0_SDA_PINS )) {

        entry -> i2c.i2cHw = i2c0;
    }
    else if ((( 1 << sclPin ) & VALID_I2C_1_SCL_PINS ) && (( 1 << sdaPin ) & VALID_I2C_1_SDA_PINS )) {

        entry -> i2c.i2cHw = i2c1;
    }
    else return ( I2C_PORT_ERR );

    entry -> i2c.sclPin       = sclPin;
    entry -> i2c.sdaPin       = sdaPin;
    entry -> i2c.baudRate     = baudRate;
    entry -> i2c.timeoutValMs = I2C_TIME_OUT_IN_MS;
   
    i2c_init( entry -> i2c.i2cHw, entry -> i2c.baudRate );
    i2c_set_slave_mode( entry -> i2c.i2cHw, false, 0 );
    
    gpio_set_function( entry -> i2c.sclPin, GPIO_FUNC_I2C );
    gpio_set_function( entry -> i2c.sdaPin, GPIO_FUNC_I2C);
    gpio_pull_up( entry -> i2c.sclPin );
    gpio_pull_up( entry -> i2c.sdaPin );
    return ( NO_ERR );
}

uint8_t i2cRead( uint8_t resId, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    CdcResource *entry = getResourceById( resId, CDC_IT_I2C );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    auto ret = i2c_read_blocking_until( entry -> i2c.i2cHw,
                                        i2cAdr,
                                        buf,
                                        len,
                                        stopBit,
                                        make_timeout_time_ms( entry -> i2c.timeoutValMs ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {

        printf( "i2cRead: resId: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                resId, i2cAdr, buf, buf[0], buf[1], len, stopBit );

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {
            
            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C read, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C read, PICO timeout error\n" );
        }
    }
   
    if (( ret == PICO_ERROR_GENERIC ) || ( ret == PICO_ERROR_TIMEOUT )) return ( I2C_READ_ERR );
    return ( NO_ERR );
}

uint8_t i2cWrite( uint8_t resId, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {
        
        printf( "i2cWrite: resId: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                resId, i2cAdr, buf, buf[0], buf[1], len, stopBit );
    }

    CdcResource *entry = getResourceById( resId, CDC_IT_I2C );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    auto ret = i2c_write_blocking_until( entry -> i2c.i2cHw,
                                         i2cAdr,
                                         buf,
                                         len,
                                         stopBit,
                                         make_timeout_time_ms( entry -> i2c.timeoutValMs ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_PWM )) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {

            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C write, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C write, PICO timeout error\n" );
        }
    }
    
    if (( ret == PICO_ERROR_TIMEOUT) || ( ret == PICO_ERROR_GENERIC ) || ( ret != len )) return ( I2C_WRITE_ERR );
    return ( NO_ERR );
}

uint8_t i2cBusreset( uint8_t resId ) {

     if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {

        printf( "I2C Bus reset, resId: %d\n", resId );
    }

    CdcResource *entry = getResourceById( resId, CDC_IT_I2C );
    if ( entry == nullptr ) return ( INVALID_RES_ID_ERR );

    uint8_t reset_cmd = 0x06;
    i2c_write_blocking( entry -> i2c.i2cHw, 0x00, &reset_cmd, 1, false); 
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// CAN bus Section. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t resId, uint8_t pinH, uint8_t pinL, uint32_t baudRate ) {

    CdcResource *entry = allocateResourceById( resId, CDC_IT_CAN_BUS );
    if ( entry == nullptr ) return( RES_ID_ALLOCATE_ERR );

    entry -> can.canPinH    = pinH;
    entry -> can.canPinL    = pinL;
    entry -> can.baudRate   = baudRate;
    
    // ??? just set values... ?

    return( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// Print out the Config Structure.
//
// ??? this will for sure change ... we will print out the array of resources....
//------------------------------------------------------------------------------------------------------------
void printCdcSubSystemInfo( CdcResourceMap *cMap ) {

    CdcResource *entry  = &resMap[ 0 ];
    int         index   = 0;

    printf( "CDC Resource Map\n" );

    while ( entry -> type != CDC_IT_UNDEFINED ) {

        printf( "(%2d), Type: %d\n", index, entry -> type  );

        switch ( entry -> type ) {

            case CDC_IT_CONTROLLER: {
                
            } break;
            
            case CDC_IT_TIMER: {
                
            } break;

            case CDC_IT_ADC: {
                
            } break;

            case CDC_IT_GPIO: { 
                
            } break;

            case CDC_IT_PWM: { 
                
            } break;

            case CDC_IT_I2C: {
                
            } break;

            case CDC_IT_CAN_BUS: {
                
            } break;

            case CDC_IT_UART: {
                
            } break;

            default: {
                
            };
        }

        entry ++;
    }
}

#if 0
void printConfigInfo( CdcConfigDesc *ci ) {

    printf( "CDC Pin Configuration Info ( status %d ): \n", ci -> CFG_STATUS );

    printf( "Pfail pin: %2d, ExtInt pin: %2d \n", ci -> PFAIL_PIN, ci -> EXT_INT_PIN );

    printf( "ActiveLed pin: %2d \n", ci -> ACTIVE_LED_PIN );

    printf( "DIO pins ( 0 .. 7 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
            ci -> DIO_PIN_0, ci -> DIO_PIN_1, ci -> DIO_PIN_2, ci -> DIO_PIN_3,
            ci -> DIO_PIN_4, ci -> DIO_PIN_5, ci -> DIO_PIN_6, ci -> DIO_PIN_7 );

    printf( "DIO pins ( 8 .. 15 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
            ci -> DIO_PIN_8, ci -> DIO_PIN_9, ci -> DIO_PIN_10, ci -> DIO_PIN_11,
            ci -> DIO_PIN_12, ci -> DIO_PIN_13, ci -> DIO_PIN_14, ci -> DIO_PIN_15 );

    printf( "ADC pins ( 0 .. 3 ): %2d %2d %2d %2d\n",
            ci -> ADC_PIN_0, ci -> ADC_PIN_1, ci -> ADC_PIN_2, ci -> ADC_PIN_3 );

    printf( "PWM pins ( 0 .. 7 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
            ci -> PWM_PIN_0, ci -> PWM_PIN_1, ci -> PWM_PIN_2, ci -> PWM_PIN_3,
            ci -> PWM_PIN_4, ci -> PWM_PIN_5, ci -> PWM_PIN_6, ci -> PWM_PIN_7 );

    printf( "PWM pins ( 8 .. 15 ): %2d %2d %2d %2d %2d %2d %2d %2d\n",
            ci -> PWM_PIN_8, ci -> PWM_PIN_9, ci -> PWM_PIN_10, ci -> PWM_PIN_11,
            ci -> PWM_PIN_12, ci -> PWM_PIN_13, ci -> PWM_PIN_14, ci -> PWM_PIN_15 );

    printf( "UART RX pins ( 0 .. 3 ): %2d %2d %2d %2d\n",
            ci -> UART_RX_PIN_0, ci -> UART_RX_PIN_1, ci -> UART_RX_PIN_2, ci -> UART_RX_PIN_3 );

    printf( "UART TX pins ( 0 .. 3 ): %2d %2d %2d %2d\n",
            ci -> UART_TX_PIN_0, ci -> UART_TX_PIN_1, ci -> UART_TX_PIN_2, ci -> UART_TX_PIN_3 );

    printf( "SPI0 Pins: MOSI: %2d, MISO: %2d, SCLK: %2d \n",
            ci -> SPI_MOSI_PIN_0, ci -> SPI_MISO_PIN_0, ci -> SPI_SCLK_PIN_0 );

    printf( "SPI1 Pins: MOSI: %2d, MISO: %2d, SCLK: %2d \n",
            ci -> SPI_MOSI_PIN_1, ci -> SPI_MISO_PIN_1, ci -> SPI_SCLK_PIN_1 );

    printf( "NVM I2C Pins: SCL: %2d, SDA: %2d, I2C Root: 0x%x \n",
            ci -> NVM_I2C_SCL_PIN, ci -> NVM_I2C_SDA_PIN, ci -> NVM_I2C_ADR_ROOT );

    printf( "EXT I2C Pins: SCL: %2d, SDA: %2d, I2C Root: 0x%x \n",
            ci -> EXT_I2C_SCL_PIN, ci -> EXT_I2C_SDA_PIN, ci -> EXT_I2C_ADR_ROOT );

    printf( "\n" );

}
#endif



//------------------------------------------------------------------------------------------------------------
// "configureCdcResource" configures one resource using the data from the resource descriptor. 
// 
//------------------------------------------------------------------------------------------------------------
uint8_t configureCdcResource( CdcResourceDesc *desc ) {

    switch ( desc -> type ) {

        case CDC_IT_CONTROLLER: {


        } break;

        case CDC_IT_ADC: {


        } break;

        // ??? and so on ...

        default: ;

    }

    return( NO_ERR ); // for now ...
}

//------------------------------------------------------------------------------------------------------------
// At startup the configuration data is taken from the board specific resource map. We simply run through
// the array of resource descriptors. 
// 
//------------------------------------------------------------------------------------------------------------
uint8_t configureCdcSubSytem( CdcResourceDesc *descMap ) {

    CdcResourceDesc *ptr = descMap;

    while ( ptr -> type != CDC_IT_UNDEFINED ) {

        uint8_t rStat = configureCdcResource( ptr );
        if ( rStat != NO_ERR ) break;

        ptr ++;
    } 

    return( NO_ERR ); 
}


}; // namespace CDC
