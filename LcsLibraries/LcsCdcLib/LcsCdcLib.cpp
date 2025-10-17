//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// This source file contains the the Raspberry Pi controller family hardware 
// library code. The idea of this library is to shield the actual hardware of 
// processor and board implementation from the upper layers but still keep the
// flexibility and performance of the underlying hardware. 
//
// The library works with a concept of "resources". During startup, resources 
// are configured based on data from the board resource descriptor. Most 
// routines in the CDC layer use the resource id to access the particular 
// hardware function.
//
// A historic note. The original LCS code was written for Atmega and Pico. With
// the complete shift to PICO, the CDC library just serves as an interface to 
// the PICO functions. One day, we may see more different controllers and 
// controller families. 
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2025 Helmut Fieres
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

//----------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space
// with all internal variables and routines local to the file. The second part contains
// the exported routines to be called by the core library and the firmware designers 
// that need access to the underlying HW portion managed by this lowest layer.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;

//----------------------------------------------------------------------------------------
// Valid pin mappings for the Raspberry PI Pico board. We construct a set of bitmask
// for the pin numbers. Pin Numbers range from 0 to 28. The bitmasks specify wether a
// pin can be assigned to the hardware type purpose. During configuration of a CDC 
// function, the pins are checked against these bitmasks. All pins can be used as GPIO
// pins or PWM pins. All other hardware functions are bound to dedicated pins. Note 
// that we do not check for assigning a pin to several different hardware functions. 
// All we check is that the pin can be used for the desired purpose. A check performed
// by the CDC library routines is simply done through:
//
//    if (( 1 <<  pin ) & VALID_xxx )
//
//----------------------------------------------------------------------------------------
const uint8_t  MAX_PIN_NUM          = 28;

const uint32_t VALID_GPIO_PINS      = 0x1FFFFFFF;
const uint32_t VALID_PWM_PINS       = 0x1FFFFFFF;
const uint32_t VALID_ADC_PINS       = ( 1 << 26 ) | ( 1 << 27 ) | ( 1 << 28 );

const uint32_t VALID_I2C_0_SDA_PINS = ( 1 << 0  ) | ( 1 << 4  ) | ( 1 << 8  ) |
                                      ( 1 << 12 ) | ( 1 << 16 ) | ( 1 << 20 );
const uint32_t VALID_I2C_0_SCL_PINS = ( 1 << 1  ) | ( 1 << 5  ) | ( 1 << 9  ) |
                                      ( 1 << 13 ) | ( 1 << 17 ) | ( 1 << 21 );

const uint32_t VALID_I2C_1_SDA_PINS = ( 1 << 2  ) | ( 1 << 6  ) | ( 1 << 10 ) |
                                      ( 1 << 14 ) | ( 1 << 18 ) | ( 1 << 26 );
const uint32_t VALID_I2C_1_SCL_PINS = ( 1 << 3  ) | ( 1 << 7  ) | ( 1 << 11 ) |
                                      ( 1 << 15 ) | ( 1 << 19 ) | ( 1 << 27 );

const uint32_t VALID_UART_0_TX_PINS = ( 1 << 0  ) | ( 1 << 12 ) | ( 1 << 16 );
const uint32_t VALID_UART_0_RX_PINS = ( 1 << 1  ) | ( 1 << 13 ) | ( 1 << 17 );

const uint32_t VALID_UART_1_TX_PINS = ( 1 << 4  ) | ( 1 << 8  );
const uint32_t VALID_UART_1_RX_PINS = ( 1 << 5  ) | ( 1 << 9  );

const uint32_t VALID_SPI_0_SCK_PINS = ( 1 << 2  ) | ( 1 << 6  ) | ( 1 << 18 );
const uint32_t VALID_SPI_0_TX_PINS  = ( 1 << 3  ) | ( 1 << 7  ) | ( 1 << 19 );
const uint32_t VALID_SPI_0_RX_PINS  = ( 1 << 0  ) | ( 1 << 4  ) | ( 1 << 16 );

const uint32_t VALID_SPI_1_SCK_PINS = ( 1 << 10 ) | ( 1 << 14 );
const uint32_t VALID_SPI_1_TX_PINS  = ( 1 << 11 ) | ( 1 << 15 );
const uint32_t VALID_SPI_1_RX_PINS  = ( 1 << 8  ) | ( 1 << 12 );

const uint32_t VALID_I2C_0_PINS  = VALID_I2C_0_SDA_PINS | VALID_I2C_0_SCL_PINS;
const uint32_t VALID_I2C_1_PINS  = VALID_I2C_1_SDA_PINS | VALID_I2C_1_SCL_PINS;

const uint32_t VALID_UART_0_PINS = VALID_UART_0_TX_PINS | VALID_UART_0_RX_PINS;
const uint32_t VALID_UART_1_PINS = VALID_UART_1_TX_PINS | VALID_UART_1_RX_PINS;

//----------------------------------------------------------------------------------------
// Characteristics of the Raspberry Pi Pico and some key constants for the CDC library.
// 
//----------------------------------------------------------------------------------------
const uint16_t  MAX_CPU_CORE                = 2;
const uint16_t  MAX_INT_PIN                 = 24;

const uint16_t  MAX_RESOURCE_ENTRIES        = MAX_RES_DESC_ENTRIES;
const uint16_t  MAX_RES_NAME                = 64;
const uint8_t   MAX_UART_BUF_SIZE           = 8;

const uint32_t  WATCHDOG_TIMER_MILLIS       = 2000;
const uint32_t  ADC_REF_VOLTAGE_MILLIS      = 3300;
const uint16_t  ADC_DIGIT_RANGE             = 1024;

//----------------------------------------------------------------------------------------
// Controller dependent code uses a set of hardware resource structures to control the
// controller hardware. When a particular resource, e.g. an I2C channel, is configured
// all further access will use the resource data for its operation. 
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// The resource map has an array of the resources. 
//
//----------------------------------------------------------------------------------------
struct CdcResource {

    uint8_t type;
    uint8_t resId;

    union {

        //--------------------------------------------------------------------------------
        // A timer resource. We need to keep the local timer instance data for the PICO.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint32_t            timerVal;
            TimerCallback       timerCallback;
            repeating_timer_t   timerData;

        } timer;

        //--------------------------------------------------------------------------------
        // The GPIO resource is perhaps the most fundamental resource. It manages a HW
        // pin. Optional, we can have two pins which then act as pair and are read from
        // or written to simultaneously.
        // 
        //--------------------------------------------------------------------------------
        struct {

            uint8_t         pinA;
            uint8_t         pinB;
            uint8_t         pinMode;
            GpioCallback    handler;

        } gpio;

        //--------------------------------------------------------------------------------
        // An ADC instance. The PICO supports up to three ADC inputs. When we use such
        // an input, the corresponding instance data is kept in this structure. We also
        // keep the PICO ADC number, so we can select the correct HW instance.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t   adcPin;
            uint8_t   adcNum;

        } adc;

        //--------------------------------------------------------------------------------
        // The PWM output resource manages a PWM configured output pin. We keep track 
        // of one or two pins, which must be on the same PWM slice. The idea is  that 
        // we use for H-Bridge control two output signals, which act as a pair. 
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t     pinA;
            uint8_t     pinB;
            uint32_t    frequency;
            uint        wrap;
            uint        channel;
            uint        sliceNum;
            bool        inverted;
            bool        phaseCorrect;

        } pwm;

        //--------------------------------------------------------------------------------
        // UARTS are used to read in a serial stream from the RailCom detectors. There
        // can be two hardware based UART resources. The resource also keeps a small
        // buffer where the data is read into.
        //
        //--------------------------------------------------------------------------------
        struct {

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

        } uart;
    
        //--------------------------------------------------------------------------------
        // The PICO features two I2C HW channels. The resource data contains the GPIO
        // pins assigned, the baud rate and a timeout. We also keep an I2C address root,
        // which comes in handy for addressing chips with the same root address.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t     sclPin;
            uint8_t     sdaPin;
            uint8_t     i2cAdrRoot;
            uint32_t    baudRate;
            uint32_t    timeoutValMs;

            i2c_inst_t  *i2cHw;

        } i2c;

        //--------------------------------------------------------------------------------
        // The CAN bus resource. Although our current controller does not feature a
        // CAN bus hardware, the resource describes the hardware elements needed. 
        // We currently use a software version based on a PIO program to implement the
        // CAN bus layer. 
        // 
        //--------------------------------------------------------------------------------
        struct {

            uint8_t         canPinRx;
            uint8_t         canPinTx;
            uint32_t        baudRate;
            uint32_t        canId;
            bool            twoCores;

        } can;
    };
};

//----------------------------------------------------------------------------------------
// The resource map is the central data structure to talk to the hardware. It is built
// at runtime startup using the resource descriptor map. Essentially it contain all the
// data from the resource descriptors and depending on the descriptor type the PICO
// data structures necessary.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// The interrupt table for the GPIO pin interrupts. The PICO has only one interrupt 
// handler. We will allocate a table where an interrupt handler can be set for each 
// HW pin. 
//
//----------------------------------------------------------------------------------------
struct GpioIsrTable {

    uint16_t        numOfHandlers = 0;
    GpioCallback    gpioIsrTable[ MAX_CPU_CORE ][ MAX_INT_PIN + 1 ];
};

//----------------------------------------------------------------------------------------
// File local variables. We need to remember whether we initialized already. We also
// store the descriptor and resource map. Finally, we need to have a table for DIO 
// interrupt handlers, and the instances of the HW UART instances.
//
//----------------------------------------------------------------------------------------
bool                    initialized = false;
CdcResourceDescMap      dMap;
CdcResourceMap          rMap;
GpioIsrTable            dioIntHandlers;
CdcResource             *uartRes0;
CdcResource             *uartRes1;

//----------------------------------------------------------------------------------------
// "validPin" is called to check that a pin is in the correct number range, defined and
// matches the bitmask for the desired purpose. For example, configuring an I2C port 
// will check that the two GPIO pins are indeed routable to an I2C HW block in the PICO.
//
//----------------------------------------------------------------------------------------
bool validPin( uint8_t pin, uint32_t mask ) {

    if ( pin == UNDEFINED_PIN )     return ( true );
    if ( pin > MAX_PIN_NUM )        return ( false );
    return (( 1 << pin ) & mask );
}

//----------------------------------------------------------------------------------------
// When no interrupt is configured for a GPIO pin, we set the table entry to a dummy
// handler. This way we do not have to check every time for a valid procedure label 
// when we handle an interrupt.
//
//----------------------------------------------------------------------------------------
void dummyIsrHandler ( uint8_t pin, uint8_t event ) { }

//----------------------------------------------------------------------------------------
// Setup the ISR table. The PICO can have only one interrupt handler. When you want a 
// handler per GPIO pin, the solution is to have a table when you keep the handler on
// a per pin base.
//
//----------------------------------------------------------------------------------------
void initIsrTable( ) {

    dioIntHandlers.numOfHandlers = 0;

    for ( uint16_t i = 0; i < MAX_CPU_CORE; i++ ) {

        for ( uint16_t j = 0; j < MAX_INT_PIN; j++ ) {

            dioIntHandlers.gpioIsrTable[ i ][ j ] = dummyIsrHandler;
        }
    }
}

//----------------------------------------------------------------------------------------
// Set up the CDC resource map with default values.
//
//----------------------------------------------------------------------------------------
void initResourceMap( CdcResourceMap *rMap ) {

    rMap -> options                     = 0;
    rMap -> debugMask                   = CDC_DBG_ALL;
    rMap -> boardId                     = 0;
    rMap -> cFamily                     = CDC_CF_UNDEFINED;
    rMap -> cType                       = CDC_CF_UNDEFINED;
    rMap -> cpuCores                    = 1;
    rMap -> memorySize                  = 0;
    rMap -> eepromSize                  = 0;
    rMap -> watchDogIntervallMillis     = 2000;
    rMap -> adcRefVoltageMillis         = 3300;
    rMap -> adcDigitRange               = 1024; 
    rMap -> name[0 ]                    = 0;

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {
        
        rMap -> map[ i ].type = CDC_RT_UNDEFINED;
    }
} 

//----------------------------------------------------------------------------------------
// A resource is found by indexing into the resource map with index and resource type.
//
//----------------------------------------------------------------------------------------
CdcResource *lookupResource( uint8_t rNum, uint8_t type ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );
    if ( rMap.map[ rNum ].type != type ) return ( nullptr );
    return ( &rMap.map[ rNum ] );
}

//----------------------------------------------------------------------------------------
// The configuration routines will allocate the corresponding entry in the resource
// map. When the entry is found but of a different type, it is an error. When there is 
// no entry yet, the entry is initialized with the type and can be used for the further
// configuration.
//
//----------------------------------------------------------------------------------------
CdcResource *allocateResourceType( uint8_t rNum, uint8_t type ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );
    
    if ( rMap.map[ rNum ].type == CDC_RT_UNDEFINED ) {

        rMap.map[ rNum ].type   = type;
        rMap.map[ rNum ].resId  = rNum;
        return ( &rMap.map[ rNum ] );
    }
    else if ( rMap.map[ rNum ].type == type ) {

        return ( &rMap.map[ rNum ] );
    }
   else return ( nullptr );
}

//----------------------------------------------------------------------------------------
// The PICO uses a set of constants to describe the GPIO pin interrupt type. We map 
// our CDC interrupt types to the PICO GPIO_IRQ_xxx types.
//
//----------------------------------------------------------------------------------------
uint32_t mapCdcIntEvent( uint8_t event ) {

    switch ( event ) {

        case CDC_EVT_LOW:    return ( GPIO_IRQ_LEVEL_LOW );
        case CDC_EVT_HIGH:   return ( GPIO_IRQ_LEVEL_HIGH );
        case CDC_EVT_FALL:   return ( GPIO_IRQ_EDGE_FALL );
        case CDC_EVT_RISE:   return ( GPIO_IRQ_EDGE_RISE );
        case CDC_EVT_CHANGE: return ( GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL );
        default:             return ( GPIO_IRQ_EDGE_RISE );
    }
}

uint8_t mapPicoGpioEvent( uint32_t event ) {

    switch ( event ) {

        case GPIO_IRQ_LEVEL_LOW:  return ( CDC_EVT_LOW );
        case GPIO_IRQ_LEVEL_HIGH: return ( CDC_EVT_HIGH );
        case GPIO_IRQ_EDGE_FALL:  return ( CDC_EVT_FALL );
        case GPIO_IRQ_EDGE_RISE:  return ( CDC_EVT_RISE );
        default:                  return ( CDC_EVT_RISE );
    }
}

//----------------------------------------------------------------------------------------
// Global Interrupt handlers. The hardware and low level library will call these 
// handlers, which in turn will invoke the respective callback function if configured. 
//
// The repeating timer alarm will handle timer interrupts. We stored the respective 
// timer resource in the "user_data" field, so that we can get to the interrupt handler
// configured.
//
// The GPIO interrupt handler manages the handler for all possible IO pins. The PICO 
// can only have one interrupt routine, so we feature an array of handlers where a 
// handler for a GPIO pin can be registered. 
// 
// The UART handlers will handle receive interrupts of the UART hardware blocks. There
// is no easy way to get to the resource structure where the input buffer is. We 
// therefore maintain two global variables in this file to store the configured resource
// for each UART HW block.
// 
//----------------------------------------------------------------------------------------
bool repeatingTimerAlarm( repeating_timer_t *rt ) {

    CdcResource *ptr = (CdcResource *) rt -> user_data;

    if ( ptr -> timer.timerCallback != nullptr ) {

        ptr -> timer.timerCallback((uint32_t)
                                    ( - ptr -> timer.timerData.delay_us ));       
    }
    
    return ( true );
}

void gpioCallback( uint gpioPin, uint32_t event ) {

    dioIntHandlers.gpioIsrTable[ get_core_num( )][ gpioPin ] 
                                     ( gpioPin, mapPicoGpioEvent( event ));
}

void uartRxCallback0( ) {

    while ( uart_is_readable( uart0 )) {

        uint8_t ch = uart_getc( uart0 );
        if ( uartRes0 -> uart.rxBufIndex < MAX_UART_BUF_SIZE ) { 
            
            uartRes0 -> uart.rxDataBuf[ uartRes0 -> uart.rxBufIndex++ ] = ch;
        }
    }
}

void uartRxCallback1( ) {

    while ( uart_is_readable( uart1 )) {

        uint8_t ch = uart_getc( uart1 );
        if ( uartRes1 -> uart.rxBufIndex < MAX_UART_BUF_SIZE ) {
            
            uartRes1 -> uart.rxDataBuf[ uartRes1 -> uart.rxBufIndex++ ] = ch;
        }
    }
}

//----------------------------------------------------------------------------------------
// A little helper function to set the GPIO mode for input and output.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Name space CDC. All routines and definitions exported are in this name space.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// For debugging purposes. Instead of conditional compilations, the debug mask will
// enable the printing of debug and trace data.
//
//----------------------------------------------------------------------------------------
void setDebugMask( uint16_t mask ) {

    rMap.debugMask = mask;
}

uint16_t getDebugMask( ) {

    return ( rMap.debugMask );
}

//----------------------------------------------------------------------------------------
// Version Info and patch level.
//
//----------------------------------------------------------------------------------------
uint32_t getVersion( ) {

    return ( CDC_LIB_VERSION );
}

uint32_t getPatchLevel( ) {
    
    return ( CDC_LIB_PATCH_LEVEL );
}

//----------------------------------------------------------------------------------------
// CDC library setup. The "init" routine will ready the CDC library and keep a copy of
// the descriptor map which will be used for the setup. The init routine can be called
// more than once without a problem.
//
//----------------------------------------------------------------------------------------
uint8_t cdcInit( CdcResourceDescMap *dMapPtr ) {

    dMap = *dMapPtr;
 
    if ( ! initialized ) {

        initResourceMap( &rMap );
        initIsrTable( );
        configureConsoleIO( );
    }

    return ( NO_ERR );
}
 
//----------------------------------------------------------------------------------------
// "getResourceMap" will return a pointer to the configured resource map. This is 
// typically the map that was created with the data from the resource descriptor map.
//
//----------------------------------------------------------------------------------------
CdcResourceMap  *getResourceMap( ) {

    return ( &rMap );
}

//----------------------------------------------------------------------------------------
// A resource descriptor is found by searching the resource Id in the entries. This 
// gives us also the flexibility of arranging the resource descriptor entries in the
//  board descriptor.
//
//----------------------------------------------------------------------------------------
CdcResourceDesc *lookupResourceDesc( uint8_t rNum, uint8_t type ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResourceDesc *ptr = &dMap.map[ i ];
        if (( ptr -> resId == rNum ) && ( ptr -> type == type )) return( ptr );
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// "fatalError" is the error communication method when we cannot get anything to work.
// The Raspberry Pi PICO has a small Led on the board. We will use this LED to "blink"
// an error code. There are up to eight codes. The sequence is as follows:
//
//    repeat forever:
//
//    - 1s ON, 0.5s 0FF
//    - for ( int i = 0; i < n; i++ ) { 0.5s ON; 0.5s OFF; }
//
// The only way to get out of this loop is then to reset the board. Fatal errors are
// hopefully not many. One obvious one is when we cannot detect the NVM and thus know
// nothing about the board.
//
// If we have a console, we attempt to first write an error message to the console 
// before looping.
//
//----------------------------------------------------------------------------------------
void fatalError( uint8_t n, char *str, uint8_t rStat ) {

    if ( str != nullptr ) {

        if ( isConsoleConnected( )) { 
            
            printf( "Fatal Error: %d: %s, rStat: %d\n", n, str, rStat );
        }
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

//----------------------------------------------------------------------------------------
// Simple timestamp and sleep functions.
// 
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// "createUid" is the routine that produces a unique ID for the node. The scheme is 
// based on a random number. Alternatively we could use the unique flash chip ID on
// the board. 
//
//----------------------------------------------------------------------------------------
uint32_t createUid( ) {

    uint32_t rVal = 0;

    volatile 
    uint32_t *rnd_reg = (uint32_t *) ( ROSC_BASE + ROSC_RANDOMBIT_OFFSET );

    for ( int k = 0; k < 32; k++ ) {

        rVal = rVal << 1;
        rVal = rVal + ( 0x00000001 & ( *rnd_reg ));
    }

    return ( rVal );
}

//----------------------------------------------------------------------------------------
// Console IO section. We set up the stdio via the USB connector. As part of the
// cdcInit call, the console configure call should be done rather early, so that we  
// can print out debug messages. In normal LCS node operation there is no USB device
// connected. Detecting a connection helps to decide whether we can report an error 
// or need to resort to a fatal error call at startup. 
//
// There are two basic ways to detect an USB connection. The first is to simply check
// if there is power on the USB port. The PICO features an internal GPIO pin for this
// purpose. Using this method still does not mean that we have a computer connected to
// the USB, but just that there is a cable with power. Well, good enough for us. The
// second method truly detects that there is a USB host connected. This check is
// provided via the PICO libraries which in turn use the tinyUSB library. However, 
// there could be a timing problem where the USB stack is not ready yet and we conclude
// wrongly that there is no USB connection. For now, let's rather go with the crude 
// approach to check if there is power on the VBUS pin, at the risk that there is just
// power on the USB connector and no data.
//
// Finally, there is a routine to get a character for the command interfaces. Since 
// the function just reads in a character, optionally with a timeout how long to wait 
// for any input.
//
// PS: The USB check way would be "return ( stdio_usb_connected( ));" instead of the
// GPIO check.
//
//----------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------
// Processor general attributes. 
//
//----------------------------------------------------------------------------------------
uint8_t getControllerFamily( uint16_t *family ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *family = rMap.cFamily;
    return ( NO_ERR );
}

uint8_t getControllerChip( uint16_t *ctl ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *ctl = rMap.cType;
    return ( NO_ERR );
}

uint8_t getBoardId( uint16_t *bId ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *bId = rMap.boardId;
    return ( NO_ERR );
}

uint8_t getChipMemSize( uint32_t *size ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *size = rMap.memorySize;
    return ( NO_ERR );
}

uint8_t getChipNvmSize( uint32_t *size ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *size = rMap.eepromSize;
    return ( NO_ERR );
}

uint8_t getChipCpuFrequency( uint32_t *frequency ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *frequency = clock_get_hz( clk_sys );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Watchdog facility.
//
//----------------------------------------------------------------------------------------
uint8_t watchDogEnable( bool enable ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    watchdog_enable( rMap.watchDogIntervallMillis, 1 );
    return ( NO_ERR );
}

uint8_t watchDogUpdate( ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    watchdog_update( );
    return ( NO_ERR );
}

uint8_t watchDogCausedReboot( bool *reboot ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    return ( watchdog_caused_reboot( ));
}

//----------------------------------------------------------------------------------------
// Timer section. The CDC library features a repeating timer with a microsecond 
// resolution. There are routines to start and stop the timer as well as to allow to
// set a new limit. The PICO offers a high level function that schedules a repeating
// timer with the property of measuring the interval also from the start of the 
// callback invocation. 
//
//----------------------------------------------------------------------------------------
uint8_t configureTimer( uint8_t rNum, TimerCallback functionId ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    if ( rNum < CDC_RN_FIRST_USER_RN ) return ( RES_NUM_ERR );

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_TIMER );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    CdcResource *ptr = allocateResourceType( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    ptr -> timer.timerVal         = dPtr -> timer.timerVal;
    ptr -> timer.timerCallback    = functionId;
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Start a timer.
//
//----------------------------------------------------------------------------------------
uint8_t startRepeatingTimer( uint8_t rNum, uint32_t val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    add_repeating_timer_us( - limit, 
                            repeatingTimerAlarm, 
                            ptr, 
                            &ptr -> timer.timerData );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Stop a timer.
//
//----------------------------------------------------------------------------------------
uint8_t stopRepeatingTimer( uint8_t rNum ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    cancel_repeating_timer( &ptr -> timer.timerData );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Return the upper limit value for the timer.
//
//----------------------------------------------------------------------------------------
uint8_t getRepeatingTimerLimit( uint8_t rNum, uint32_t *val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    *val = (uint32_t) ( - ptr -> timer.timerData.delay_us );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Set a new timer limit.
//
//----------------------------------------------------------------------------------------
uint8_t setRepeatingTimerLimit( uint8_t rNum, uint32_t val ) {

    CdcResource *ptr = lookupResource( rNum, CDC_RT_TIMER );
    if ( ptr == nullptr ) return ( TIMER_RES_ERR );

    int64_t limit = val;
    ptr -> timer.timerData.delay_us = ((int64_t) - limit );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// ADC section. The analog input channel represented by the pin is configured. At 
// initialization, the ADC pin number is validated and the ADC subsystem is initialized.
// The PICO does an analog read in about 2us. This is so fast, it is sufficient for our
// purpose, so it does not make much sense to implement an asynchronous option. The 
// PICO support up to three ADC pins at the dedicated HW pins numbers 26, 27 and 28. 
// They also need to be mapped an ADC select number for selecting the ADC hardware.
//
//----------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    if ( rNum < CDC_RN_FIRST_USER_RN  ) return ( RES_NUM_ERR );

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_ADC );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureAdc( rNum, dPtr -> adc.adcPin, dPtr -> adc.adcNum ));
}

//----------------------------------------------------------------------------------------
// Configure the ADC channel.
//
//----------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t rNum, uint8_t adcPin, uint8_t adcNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    if ( rNum < CDC_RN_FIRST_USER_RN  ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_ADC );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( adcPin == 26 ) {

        rPtr -> adc.adcPin = 26;
        rPtr -> adc.adcNum = 0;
    }
    else  if ( adcPin == 27 ) {

        rPtr -> adc.adcPin = 27;
        rPtr -> adc.adcNum = 1;
    }
    else  if ( adcPin == 28 ) {

        rPtr -> adc.adcPin = 28;
        rPtr -> adc.adcNum = 2;
    }
    else return ( ADC_PIN_ERR );

    adc_init( );
    adc_gpio_init( rPtr -> adc.adcPin );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Read the ADC value. The resolution is 12-bits.
//
//----------------------------------------------------------------------------------------
uint8_t readAdc( uint8_t rNum, uint16_t *val ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_ADC );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    adc_select_input( rPtr -> adc.adcNum );
    *val = adc_read( );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
uint16_t getAdcRefVoltage( ) {

    return ( ADC_REF_VOLTAGE_MILLIS );
}

uint16_t getAdcDigitRange( ) {

    return ( ADC_DIGIT_RANGE );
}
 
//----------------------------------------------------------------------------------------
// DIO section. A digital pin is the bread and butter hardware resource and can be an
// input or output pin. For inputs, an internal pull-up resistor can be set.There are
// a couple of interfaces. First the single pin read, write and toggle. Note that no
// cross checking is done if the pins are used by other CDC functions. The DIO routines
// allow to pass two pins and their values. We often use DIO pins as pairs. This is
// typically used for the H-Bridge control pins, which are set at the same time. 
//
// A GPIO pin can also have an attached interrupt handler. When we register a handler
// for a pin, there are two different PICO lib routines to use. When there is no handler
// registered so far, we register the common callback and store the particular GPIO 
// handler in our ISR handler table. Otherwise, we just store the handler in the table
// and enable the GPIO pin for interrupts. If the resource configured two pins, the 
// handler is set for both pins.
//
//----------------------------------------------------------------------------------------
uint8_t configureDio( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
   
    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_GPIO );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );

    return ( configureDio( rNum, dPtr-> gpio.pinA, 
                           dPtr -> gpio.pinB, 
                           dPtr -> gpio.pinMode ));
}

uint8_t configureDio( uint8_t rNum, uint8_t pinA, uint8_t pinB, uint8_t mode ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_GPIO )) {

        printf( "configureDio: rNum: %d, pinA: %d, pinB: %d, mode: %d\n", 
                rNum, pinA, pinB, mode );
    }

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );
    
    rPtr -> gpio.pinA    = pinA;
    rPtr -> gpio.pinB    = pinB;
    rPtr -> gpio.handler = nullptr;
    rPtr -> gpio.pinMode = mode % 4;
 
    if (( ! validPin( rPtr -> gpio.pinA, VALID_GPIO_PINS )) ||
        ( ! validPin( rPtr -> gpio.pinB, VALID_GPIO_PINS ))) {

        return ( DIO_PIN_ERR );
    }
    
    gpio_init( rPtr -> gpio.pinA );
    setGpioMode( rPtr -> gpio.pinA, rPtr -> gpio.pinMode );

    if ( rPtr -> gpio.pinB != UNDEFINED_PIN ) {
        
        gpio_init( rPtr -> gpio.pinB );
        setGpioMode( rPtr -> gpio.pinB, rPtr -> gpio.pinMode );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Register a GPIO callback.
//
//----------------------------------------------------------------------------------------
uint8_t registerDioCallback( uint8_t rNum, uint8_t event, GpioCallback func ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> gpio.pinA <= MAX_INT_PIN ) {

        if ( dioIntHandlers.numOfHandlers == 0 ) {

            gpio_set_irq_enabled_with_callback( 
                                            rPtr -> gpio.pinA, 
                                            mapCdcIntEvent( event ), 
                                            true, 
                                            gpioCallback );
        }
        else {

            gpio_set_irq_enabled( rPtr -> gpio.pinA, 
                                  mapCdcIntEvent( event ), 
                                  true);
        }

        int     core    = get_core_num( );
        uint8_t pin     = rPtr -> gpio.pinA;
    
        dioIntHandlers.gpioIsrTable[ core ][ pin ] = func;
        dioIntHandlers.numOfHandlers ++;
    }

    if (( rPtr -> gpio.pinB != UNDEFINED_PIN ) && 
        ( rPtr -> gpio.pinB <= MAX_INT_PIN )) {

        if ( dioIntHandlers.numOfHandlers == 0 ) {

            gpio_set_irq_enabled_with_callback( rPtr -> gpio.pinB, 
                                                mapCdcIntEvent( event ), 
                                                true, 
                                                gpioCallback );
        }
        else {
            
            gpio_set_irq_enabled( rPtr -> gpio.pinB, 
                                  mapCdcIntEvent( event ), 
                                  true);
        }

        int     core    = get_core_num( );
        uint8_t pin     = rPtr -> gpio.pinA;
    
        dioIntHandlers.gpioIsrTable[ core ][ pin ] = func;
        dioIntHandlers.numOfHandlers ++;
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Remove a GPIO callback.
//
//----------------------------------------------------------------------------------------
uint8_t unregisterDioCallback( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> gpio.pinA <= MAX_INT_PIN ) {

        int     core    = get_core_num( );
        uint8_t pin     = rPtr -> gpio.pinA;

        if ( dioIntHandlers.gpioIsrTable[ core ][ pin ] != nullptr ) {

            gpio_set_irq_enabled( pin, 0, false );
            dioIntHandlers.gpioIsrTable[ core ][ pin ] = dummyIsrHandler;
            dioIntHandlers.numOfHandlers --;
        }
    }

    if (( rPtr -> gpio.pinB != UNDEFINED_PIN ) && 
        ( rPtr -> gpio.pinB <= MAX_INT_PIN )) {

        int     core    = get_core_num( );
        uint8_t pin     = rPtr -> gpio.pinB;

        if ( dioIntHandlers.gpioIsrTable[ core ][ pin ] != nullptr ) {

            gpio_set_irq_enabled( pin, 0, false );
            dioIntHandlers.gpioIsrTable[ core ][ pin ] = dummyIsrHandler;
            dioIntHandlers.numOfHandlers --;
        }
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Read DIO data.
//
//----------------------------------------------------------------------------------------
uint8_t readDio( uint8_t rNum, bool *valA, bool *valB ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> gpio.pinB == UNDEFINED_PIN ) {

        *valA = gpio_get( rPtr -> gpio.pinA );
    }
    else {

        uint32_t maskData = ( 1UL << rPtr -> gpio.pinA ) | 
                            ( 1UL << rPtr -> gpio.pinB );

        *valA = gpio_get( rPtr -> gpio.pinA );
        *valB = gpio_get( rPtr -> gpio.pinB );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Write DIO data.
//
//----------------------------------------------------------------------------------------
uint8_t writeDio( uint8_t rNum, bool valA, bool valB ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> gpio.pinB == UNDEFINED_PIN ) {

        gpio_put( rPtr -> gpio.pinA, valA );
    }
    else {

        uint32_t maskData = ( 1UL << rPtr -> gpio.pinA ) | 
                            ( 1UL << rPtr -> gpio.pinB );

        uint32_t valData  = (( valA ) ? ( 1 << rPtr -> gpio.pinA ) : 0 ) | 
                            (( valB ) ? ( 1 << rPtr -> gpio.pinB ) : 0 );

        gpio_put_masked( maskData, valData );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Toggle GPIO pin.
//
//----------------------------------------------------------------------------------------
uint8_t toggleDio( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> gpio.pinMode != CDC_DIO_OUT ) return ( DIO_MODE_ERR );

    if ( rPtr -> gpio.pinB == UNDEFINED_PIN ) {

        gpio_put( rPtr -> gpio.pinA, ! gpio_get( rPtr -> gpio.pinA ));
    }
    else {

        gpio_put( rPtr -> gpio.pinA, ! gpio_get( rPtr -> gpio.pinA ));
        gpio_put( rPtr -> gpio.pinB, ! gpio_get( rPtr -> gpio.pinB ));
    }
        
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// PWM section. The PICO is quite flexible when it comes to PWM signals. We implement
// a simple PWM capability. There is the frequency which set during configuration and
// there is the write operation which set the duty cycle. The calculations are best 
// described in the PICO C++ SDK. Note that although the PICO is quite flexible, the 
// wrap and phase parameters are set for the slice and not a single channel. The same 
// is true for the signal inverter. This is normally not an issue unless you want to
// have separate values for PWM pins on the same slice. 
//
// The "writePwm" function will just manipulate the duty cycle. When we need to change
// the frequency we need to configure again. The "syncPwm" function will reset the wrap
// count, which is used to implement the sync function for H-Bridges emitting a PWM
// signal.
// 
//----------------------------------------------------------------------------------------
uint8_t configurePwm( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
   
    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_PWM );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configurePwm( rNum, 
                           dPtr -> pwm.pinA, 
                           dPtr -> pwm.pinB, 
                           dPtr -> pwm.frequency ));
}

//----------------------------------------------------------------------------------------
// Configure the PWM channel.
//
//----------------------------------------------------------------------------------------
uint8_t configurePwm( uint8_t rNum, 
                      uint8_t pinA, 
                      uint8_t pinB, 
                      uint32_t frequency ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_PWM )) {

        printf( "Configure Pwm: rNum: %d, pinA: %d, pinB: %d, f: %d\n",
                rNum, pinA, pinB, frequency );
    }

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    if ( rNum < CDC_RN_FIRST_USER_RN  ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> pwm.pinA            = pinA;
    rPtr -> pwm.pinB            = pinB;
    rPtr -> pwm.phaseCorrect    = true;
    rPtr -> pwm.inverted        = false;     
    rPtr -> pwm.sliceNum        = pwm_gpio_to_slice_num( rPtr -> pwm.pinA );
    rPtr -> pwm.frequency       = frequency;              

    if ( rPtr -> pwm.pinB != UNDEFINED_PIN ) {

        if ( pwm_gpio_to_slice_num( rPtr -> pwm.pinA ) != 
             pwm_gpio_to_slice_num( rPtr -> pwm.pinB )) {

            return ( PWM_PIN_ERR );
        }
    }

    if ( rPtr -> pwm.phaseCorrect ) {
        
        rPtr -> pwm.frequency = rPtr -> pwm.frequency * 2;
    }

    uint32_t sysClock = clock_get_hz( clk_sys );
    uint32_t clkDiv   = sysClock / rPtr -> pwm.frequency / 4096 + 
                        ( sysClock % ( rPtr -> pwm.frequency * 4096 ) != 0 );
    if ( clkDiv / 16 == 0 ) clkDiv = 16;

    rPtr -> pwm.wrap = sysClock * 16 / clkDiv / rPtr -> pwm.frequency - 1;
   
    pwm_config pwmConfig = pwm_get_default_config( );
    gpio_set_function( rPtr -> pwm.pinA, GPIO_FUNC_PWM );
    
    if ( rPtr -> pwm.pinB != UNDEFINED_PIN )  
        gpio_set_function( rPtr -> pwm.pinB, GPIO_FUNC_PWM );
   
    pwm_config_set_wrap( &pwmConfig, rPtr -> pwm.wrap );
    pwm_config_set_phase_correct( &pwmConfig, rPtr -> pwm.phaseCorrect );

    pwm_config_set_output_polarity( &pwmConfig, 
                                    rPtr -> pwm.inverted, 
                                    rPtr -> pwm.inverted );

    pwm_init( rPtr -> pwm.sliceNum, &pwmConfig, false );
    pwm_set_clkdiv_int_frac( rPtr -> pwm.sliceNum, clkDiv / 16, clkDiv & 0xF );
    pwm_set_enabled( rPtr -> pwm.sliceNum, true );

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_PWM )) {
   
        printf( "pinA: % d, pinB: %d, fPwm: % d, phase: % d, inverted: % d, " 
                "clkDiv: % d, wrap: %d, sliceNum: %d\n",
                rPtr -> pwm.pinA, 
                rPtr -> pwm.pinB, 
                rPtr -> pwm.frequency, 
                rPtr -> pwm.phaseCorrect, 
                rPtr -> pwm.inverted,
                clkDiv, 
                rPtr -> pwm.wrap, 
                rPtr -> pwm.sliceNum );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Set the PWM frequency.
//
//----------------------------------------------------------------------------------------
uint8_t setPwmFrequency( uint8_t rNum, uint32_t frequency ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_PWM )) {
        
        printf( "Set PWMFrequency: rNum: %d, f: %d\n", rNum, frequency );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    return ( configurePwm( rNum, rPtr -> pwm.pinA, rPtr -> pwm.pinB, frequency ));
}

//----------------------------------------------------------------------------------------
// Write the PWM duty cycles.
//
//----------------------------------------------------------------------------------------
uint8_t writePwm( uint8_t rNum, uint8_t dutyCycleA, uint8_t dutyCycleB ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_PWM )) {
        
        printf( "Write PWM: rNum: %d, dutyA: %d, dutyB: %d\n", 
                rNum, dutyCycleA, dutyCycleB );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> pwm.pinB == UNDEFINED_PIN ) {

        pwm_set_gpio_level( rPtr -> pwm.pinA, dutyCycleA );
    }
    else {
                            
        printf( "Write Pwm: Slice: %d\n", rPtr -> pwm.sliceNum );
        pwm_set_both_levels( rPtr -> pwm.sliceNum, dutyCycleA, dutyCycleB );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Sync the PWM channel, i.e. reset the PWM slice counter.
//
//----------------------------------------------------------------------------------------
uint8_t syncPwm( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_PWM );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    pwm_set_counter( rPtr -> pwm.sliceNum, 0 );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// UART section. The UART interface is primarily used for the RailCom Detector that 
// sends a serial signal. So far, only the receiver portion is implemented because that
// is all what is needed for RailCom messages. There are two general categories. The 
// first uses the PICO built-in UART hardware blocks. The second implements a software
// UART based on the PICO PIO blocks.
//
// There are three routines. The "startUartRead" will enable the UART and start reading
// bytes into the local buffer. The "stopUartRead" will then finish the byte collection
// and disable the UART again. Finally, the "getUartBuffer" routine will return the
// bytes received. Again, note that this is not a generic UART read interface.
//
//----------------------------------------------------------------------------------------
uint8_t configureUart( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
   
    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_UART );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureUart( rNum, 
                            dPtr -> uart.rxPin, 
                            dPtr -> uart.txPin, 
                            dPtr -> uart.baudRate ));
}

//----------------------------------------------------------------------------------------
// Configure the UART channel.
//
//----------------------------------------------------------------------------------------
uint8_t configureUart( uint8_t rNum, 
                       uint8_t rxPin, 
                       uint8_t txPin, 
                       uint32_t baudRate ) {

    if ( rNum < CDC_RN_FIRST_USER_RN  ) return ( RES_NUM_ERR );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> uart.rxPin           = rxPin;
    rPtr -> uart.txPin           = txPin;
    rPtr -> uart.baudSetting     = baudRate;
    rPtr -> uart.dataBits        = 8;
    rPtr -> uart.parityMode      = UART_PARITY_NONE;
    rPtr -> uart.stopBits        = 1;

    if (( validPin( rPtr -> uart.rxPin, VALID_UART_0_RX_PINS )) && 
        ( validPin( rPtr -> uart.txPin, VALID_UART_0_TX_PINS ))) {

        rPtr -> uart.uartHw     = uart0;
        rPtr -> uart.uartIrq    = UART0_IRQ;
        uartRes0                = rPtr;
    }
    else if (( validPin( rPtr -> uart.rxPin, VALID_UART_1_RX_PINS )) && 
             ( validPin( rPtr -> uart.txPin, VALID_UART_1_TX_PINS ))) {

        rPtr -> uart.uartHw     = uart1;
        rPtr -> uart.uartIrq    = UART1_IRQ;
        uartRes1                = rPtr;
    }
    else return ( UART_PORT_ERR );

    uart_init( rPtr -> uart.uartHw, rPtr -> uart.baudSetting );
    gpio_set_function( rPtr -> uart.rxPin, GPIO_FUNC_UART );
    if ( rPtr -> uart.txPin != UNDEFINED_PIN ) {
        
        gpio_set_function( rPtr -> uart.txPin, GPIO_FUNC_UART );
    }

    uart_set_hw_flow( rPtr -> uart.uartHw, false, false );
    uart_set_format( rPtr -> uart.uartHw, 
                     rPtr -> uart.dataBits, 
                     rPtr -> uart.stopBits, 
                     rPtr -> uart.parityMode );

    uart_set_fifo_enabled( rPtr -> uart.uartHw, false );

    if ( rPtr -> uart.uartIrq == UART0_IRQ ) 
        irq_set_exclusive_handler( rPtr -> uart.uartIrq, uartRxCallback0 );
    else if ( rPtr -> uart.uartIrq == UART1_IRQ ) 
        irq_set_exclusive_handler( rPtr -> uart.uartIrq, uartRxCallback1 );

    irq_set_enabled( rPtr -> uart.uartIrq, true );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Start a UART read, i.e. enable the interrupt to receive any input.
//
//----------------------------------------------------------------------------------------
uint8_t startUartRead( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    uart_set_irq_enables( rPtr -> uart.uartHw, true, false );
    rPtr -> uart.rxBufIndex = 0;
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Stop the UART reading, i.e. disable the interrupt handling.
//
//----------------------------------------------------------------------------------------
uint8_t stopUartRead( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );
    
    uart_set_irq_enables( rPtr -> uart.uartHw, false, false );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Read the buffer of received data.
//
//----------------------------------------------------------------------------------------
uint8_t getUartBuffer( uint8_t rNum, uint8_t *buf, uint8_t bufLen ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_UART );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if (( rPtr -> uart.rxBufIndex > 0 ) && ( bufLen > 0 )) {

        uint8_t i = 0;
        while (( i < rPtr -> uart.rxBufIndex ) && ( i < bufLen )) {

            buf[ i ] = rPtr -> uart.rxDataBuf[ i ];
            i++;
        }
    
        return ( i );
    }
    else return ( 0 );
}

//----------------------------------------------------------------------------------------
// I2C Section. The PICO has two HW blocks for I2C interfaces. The interface implements
// a simple read and write access to an I2C element. There is a timeout to avoid waiting
// forever on an operation. Finally,we have routines to get the pins and baud rate.
//
//----------------------------------------------------------------------------------------
uint8_t configureI2C( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
   
    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_I2C );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureI2C(  rNum, 
                            dPtr -> i2c.sclPin, 
                            dPtr -> i2c.sdaPin, 
                            dPtr -> i2c.baudRate, 
                            dPtr -> i2c.i2cTimeoutMs ));
}

//----------------------------------------------------------------------------------------
// Configure the I2C channel.
//
//----------------------------------------------------------------------------------------
uint8_t configureI2C( uint8_t  rNum, 
                      uint8_t  sclPin, 
                      uint8_t  sdaPin, 
                      uint32_t baudRate, 
                      uint32_t timeoutVal ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_I2C )) {

        printf( "configureI2C: rNum: %d, slc: %d, sda: %d, baud: %d, tVal: %d\n",
                rNum, sclPin, sdaPin, baudRate, timeoutVal  );
    }

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> i2c.sclPin          = sclPin;
    rPtr -> i2c.sdaPin          = sdaPin;
    rPtr -> i2c.baudRate        = baudRate;
    rPtr -> i2c.timeoutValMs    = timeoutVal;

    if ((( 1 << rPtr -> i2c.sclPin ) & VALID_I2C_0_SCL_PINS ) && 
        (( 1 << rPtr -> i2c.sdaPin ) & VALID_I2C_0_SDA_PINS )) {

        rPtr -> i2c.i2cHw = i2c0;
    }
    else if ((( 1 << rPtr -> i2c.sclPin ) & VALID_I2C_1_SCL_PINS ) && 
             (( 1 << rPtr -> i2c.sdaPin ) & VALID_I2C_1_SDA_PINS )) {
                
        rPtr -> i2c.i2cHw = i2c1;
    }
    else return ( I2C_PORT_ERR );

    i2c_init( rPtr -> i2c.i2cHw, rPtr -> i2c.baudRate );
    i2c_set_slave_mode( rPtr -> i2c.i2cHw, false, 0 );
    
    gpio_set_function( rPtr -> i2c.sclPin, GPIO_FUNC_I2C );
    gpio_set_function( rPtr -> i2c.sdaPin, GPIO_FUNC_I2C);
    gpio_pull_up( rPtr -> i2c.sclPin );
    gpio_pull_up( rPtr -> i2c.sdaPin );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Read from the I2C channel.
//
//----------------------------------------------------------------------------------------
uint8_t i2cRead( uint8_t  rNum, 
                 uint8_t  i2cAdr, 
                 uint8_t  *buf, 
                 uint16_t len, 
                 bool     stopBit ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    auto ret = i2c_read_blocking_until( 
                            rPtr -> i2c.i2cHw,
                            i2cAdr,
                            buf,
                            len,
                            stopBit,
                            make_timeout_time_ms( rPtr -> i2c.timeoutValMs ));

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_I2C )) {

        printf( "i2cRead: rNum: %d, i2c: 0x%x, buf: %p, "
                "buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                rNum, i2cAdr, buf, buf[0], buf[1], len, stopBit );

        if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_I2C )) {
            
            if ( ret == PICO_ERROR_GENERIC ) { 
                
                printf( "I2C read, PICO generic error\n" );
            }

            if ( ret == PICO_ERROR_TIMEOUT ) { 
                
                printf( "I2C read, PICO timeout error\n" );
            }
        }
    }
   
    if (( ret == PICO_ERROR_GENERIC ) || ( ret == PICO_ERROR_TIMEOUT )) {
        
        return ( I2C_READ_ERR );
    }

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Write to the I2C channel.
//
//----------------------------------------------------------------------------------------
uint8_t i2cWrite( uint8_t  rNum, 
                  uint8_t  i2cAdr,
                  uint8_t  *buf, 
                  uint16_t len, 
                  bool     stopBit ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_I2C )) {
        
        printf( "i2cWrite: rNum: %d, i2cAdr: 0x%x, buf: %p, "
                "buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                rNum, i2cAdr, buf, buf[0], buf[1], len, stopBit );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );
 
    printf( "About to call PICO write: i2cAdr: %x, slc: %d, sda: %d, time: %d, stop: %d \n", 
            i2cAdr, rPtr -> i2c.sclPin, rPtr -> i2c.sdaPin, rPtr -> i2c.timeoutValMs, stopBit );

    int ret = 0;

    for (int i = 0; i < 20; ++i) {

        ret = i2c_write_blocking_until( rPtr->i2c.i2cHw, 
                        i2cAdr, 
                        buf, 
                        len, 
                        stopBit, 
                        make_timeout_time_ms( rPtr -> i2c.timeoutValMs ));
        printf("write ret=%d\n", ret);
        if (ret >= 0) break;

        sleep_ms(1);
    }

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_PWM )) {

        if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_I2C )) {

            if ( ret == PICO_ERROR_GENERIC ){
                
                printf( "I2C write, PICO generic error\n" );
            }

            if ( ret == PICO_ERROR_TIMEOUT ) { 
                
                printf( "I2C write, PICO timeout error\n" );
            }
        }
    }
    
    if (( ret == PICO_ERROR_TIMEOUT) || 
        ( ret == PICO_ERROR_GENERIC ) || 
        ( ret != len )) return ( I2C_WRITE_ERR );

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Reset the I2C channel.
// 
//----------------------------------------------------------------------------------------
uint8_t i2cBusreset( uint8_t rNum ) {

    if (( rMap.debugMask & CDC_DBG_CONFIG ) && ( rMap.debugMask & CDC_DBG_I2C )) {

        printf( "I2C Bus reset, rNum: %d\n", rNum );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    uint8_t reset_cmd = 0x06;
    i2c_write_blocking( rPtr -> i2c.i2cHw, 0x00, &reset_cmd, 1, false); 
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// I2C helper routines.
//
//----------------------------------------------------------------------------------------
uint8_t i2cGetSclPin( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> i2c.sclPin );
}

uint8_t i2cGetSdaPin( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> i2c.sdaPin );
}

uint8_t i2cGetBaudrate( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( 0 );

    return ( rPtr -> i2c.baudRate );
}

//----------------------------------------------------------------------------------------
// "scanI2CBus" is a utility routine that displays all devices found on an I2C channel.
//
//----------------------------------------------------------------------------------------
uint8_t scanI2CBus( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    if ( rPtr -> i2c.sclPin == UNDEFINED_PIN ) {

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

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// CAN bus Section. The CAN bus is the message bus used for LCS. For the PICO there 
// is a library "can2040" which implements the CAN protocol using the PIO state machine.
// This saves us hardware. The resource is the structure where we just keep the HW pins,
// the baud rate, and whether we run on one or two CPUs. In other words, we do not 
// describe a PICO hardware block.
//
//----------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_CAN_BUS );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureCanBus( rNum, 
                              dPtr -> can.rxPin, 
                              dPtr -> can.txPin, 
                              dPtr -> can.baudRate, 
                              dPtr -> can.twoCores ));
}

//----------------------------------------------------------------------------------------
// Configure the CAN bus.
//
//----------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t  rNum, 
                         uint8_t  rxPin, 
                         uint8_t  txPin, 
                         uint32_t baudRate, 
                         bool     twoCores ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );
    
    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> can.canPinRx = rxPin;
    rPtr -> can.canPinTx = txPin;
    rPtr -> can.baudRate = baudRate;
    rPtr -> can.twoCores = twoCores;

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Helper functions.
//
//----------------------------------------------------------------------------------------
uint8_t canGetRxPin( uint8_t rNum ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> can.canPinRx );
}

uint8_t canGetTxPin( uint8_t rNum ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> can.canPinTx );
}

uint32_t canGetBaudrate( uint8_t rNum ) {

   CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( 0 );

    return ( rPtr -> can.baudRate );
}

bool canGetTwoCores( uint8_t rNum ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( false );

    return ( rPtr -> can.twoCores );
}

//----------------------------------------------------------------------------------------
// Print out the Resource Descriptor Map. 
//
//----------------------------------------------------------------------------------------
void printResourceDescMap( CdcResourceDescMap *dMap ) {

    printf( "CDC Resource Descriptor Map for:\n" );
    printf( "%s\n", dMap -> name );

    printf( "Options: 0x%4x\n", dMap -> options );
    printf( "Debug Mask: 0x%4x\n", dMap -> debugMask );
    printf( "Board MWord: 0x%4x\n", dMap ->boardMword );
    printf( "Board Info: 0x%4x\n", dMap -> boardInfo );
    printf( "Board Version: 0x%4x\n", dMap -> boardVersion );
    printf( "Board Controller: 0x%4x\n", dMap -> boardCtrlInfo );

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResourceDesc *dPtr = &dMap -> map[ i ];

        printf( "(%2d): rNum: %d, ", i, dPtr -> resId  );

        switch ( dPtr ->type ) {

            case CDC_RT_TIMER: {

                printf( "Timer: val: %d\n", dPtr -> timer.timerVal );

            } break;

            case CDC_RT_ADC: {

                printf( "ADC: pin: %d, select: %d\n", 
                        dPtr -> adc.adcPin, dPtr -> adc.adcNum );

            } break;

            case CDC_RT_GPIO: {

                printf( "GPIO: pinA: %d, pinB: %d, mode: %d\n", 
                        dPtr -> gpio.pinA, 
                        dPtr -> gpio.pinB, 
                        dPtr -> gpio.pinMode );

            } break;

            case CDC_RT_PWM: {

                printf( "PWM: pinA: %d, pinB: %d, fPwm: %d\n",
                        dPtr ->pwm.pinA, 
                        dPtr ->pwm.pinB,  
                        dPtr ->pwm.frequency );

            } break;

            case CDC_RT_UART: {

                printf( "UART: rxPin: %d, txPin: %d, baudRate: %d\n",
                    dPtr -> uart.rxPin,  
                    dPtr -> uart.txPin,  
                    dPtr -> uart.baudRate );

            } break;

            case CDC_RT_I2C: {

                printf( "I2C: sclPin: %d, sdaPin: %d, baudRate: %d, "
                        "i2cRoot: 0x2x, timeout(MS): %d\n",
                        dPtr -> i2c.sclPin, 
                        dPtr -> i2c.sdaPin, 
                        dPtr -> i2c.baudRate, 
                        dPtr -> i2c.i2cTimeoutMs );

            } break;

            case CDC_RT_CAN_BUS: {

                printf( "CAN: rxPin: %d, txPin: %d, "
                        "baudRate: %d, twoCores: %d\n",
                        dPtr -> can.rxPin, 
                        dPtr -> can.txPin, 
                        dPtr -> can.baudRate, 
                        dPtr -> can.twoCores );
            } break;

            case CDC_RT_UNDEFINED: {
                
                printf( "Undefined\n" );
            
            } break;

            default: printf( "Unknown type: %d\n", i );
        }
    }

    printf( "\n" );
} 

//----------------------------------------------------------------------------------------
// Print out the Resource Map.
//
//----------------------------------------------------------------------------------------
void printResourceMap( ) {

    printf( "CDC Resource Map for:" );
    printf( "%s\n", rMap.name );

    printf( "Options: 0x%4x\n", rMap.options );
    printf( "Debug Mask: 0x%4x\n", rMap.debugMask );
    printf( "Controller Family: %d, Chip: %d\n", rMap.cFamily, rMap.cType );
    printf( "Controller Cores: %d, Mem: %d, EEPROM: %d\n", 
            rMap.cpuCores, rMap.memorySize, rMap.eepromSize );
    printf( "WatchDog Interval (MS): %d\n", rMap.watchDogIntervallMillis );
    printf( "ADC Ref Voltage: %d, Digit range: %d\n", 
            rMap.adcRefVoltageMillis, rMap.adcDigitRange ); 

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResource *rPtr = &rMap.map[ i ];

         printf( "(%2d): rNum: %d, ", i, rPtr -> resId  );

        switch ( rPtr ->type ) {

            case CDC_RT_TIMER: {

                printf( "Timer: val: %d\n", rPtr -> timer.timerVal );

            } break;

            case CDC_RT_ADC: {

                printf( "ADC: pin: %d, select: %d\n", 
                        rPtr -> adc.adcPin, rPtr -> adc.adcNum );

            } break;

            case CDC_RT_GPIO: {

                printf( "GPIO: pinA: %d, pinB: %d, mode: %d\n", 
                        rPtr -> gpio.pinA, 
                        rPtr -> gpio.pinB, 
                        rPtr -> gpio.pinMode );

            } break;

            case CDC_RT_PWM: {

                uint8_t     pwmPinA;
                uint8_t     pwmPinB;
                uint32_t    frequency;
                uint        wrap;
                uint        sliceNum;
                bool        inverted;
                bool        phaseCorrect;

                printf( "PWM: pinA: %d, pinB: %d, fPwm: %d, wrap: %d, "
                        "slice: %d, invert: %d, phase: %d\n",
                        rPtr ->pwm.pinA,  
                        rPtr ->pwm.pinB,  
                        rPtr ->pwm.frequency,
                        rPtr -> pwm.sliceNum, 
                        rPtr -> pwm.inverted, 
                        rPtr -> pwm.phaseCorrect );

            } break;

            case CDC_RT_UART: {

                printf( "UART: rxPin: %d, txPin: %d, baudRate: %d, "
                        "dataBits: %d, parity: %d, stopBits: %d\n",
                        rPtr -> uart.rxPin,  
                        rPtr -> uart.txPin, 
                         rPtr -> uart.baudSetting,
                        rPtr -> uart.dataBits, 
                        rPtr -> uart.parityMode, 
                        rPtr -> uart.stopBits );

            } break;

            case CDC_RT_I2C: {

                printf( "I2C: sclPin: %d, sdaPin: %d, baudRate: %d, "
                        "i2cRoot: 0x2x, timeout(MS): %d\n",
                        rPtr -> i2c.sclPin, 
                        rPtr -> i2c.sdaPin, 
                        rPtr -> i2c.baudRate, 
                        rPtr -> i2c.i2cAdrRoot, 
                        rPtr -> i2c.timeoutValMs );

            } break;

            case CDC_RT_CAN_BUS: {

                printf( "CAN: rxPin: %d, txPin: %d, baudRate: %d, "
                        "canId: 0x4x, twoCores: %d\n",
                        rPtr -> can.canPinRx, 
                        rPtr -> can.canPinTx, 
                        rPtr -> can.baudRate, 
                        rPtr -> can.canId, 
                        rPtr -> can.twoCores );
            } break;

            case CDC_RT_UNDEFINED: {
               
                printf( "Undefined\n" );
            } break;

            default: printf( "Unknown type: %d\n", i );
        }
    }

    printf( "\n" );
}

}; // namespace CDC
