//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the the RP2040 controller family hardware library code. The idea of this library
// is to shield the actual hardware of processor and board implementation from the upper layers but still keep
// the flexibility and performance of the underlying hardware. We also need to provide a handle from the 
// defined hardware elements to the particular board. 


// ??? rework text ....
// The library works with the concept of HW pins, which are identifiers for an HW entity. This is easy for a 
// GPIO pin, where the mapping is directly one to one. For more complex HW entries such as the I2C or UART 
// hardware, one pin is selected as the identifier to that entity. For each complex entity an instance variable
// is maintained where all the relevant data is kept.
//


// A historic note. The original LCS code was written for Atmega and Pico. With the complete shift to PICO,
// the CDC library just serves as a simple interface to the PICO functions. One day, we may see more different
// controllers and controller families. The idea is that the LCS runtime is shielded from them.
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

const uint16_t  MAX_INST_ENTRIES            = 64;

//------------------------------------------------------------------------------------------------------------
// Controller dependent code uses a set of hardware instances structures to control the controller hardware. 
// When a particular instance, e.g. an I2C channel, is configured all further access is done with a handle
// to this instance. The instance structure takes care of isolating the controller hardware from the runtime
// library and user firmware. There is a counter part to the instance structure, which contains the defined
// parameters for that instance. The parameters are used in the configuration process. The key difference
// between the two structure is that the instance structure contains controller hardware specific data 
// fields not exported to the user of the CDC library.
//
// We should be able to define all the hardware instances that a particular board offers in an array of 
// instance descriptors. Perhaps we could one day have an include file which contains for each board the
// instance descriptors array. 
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// The controller instance describes the actual controller.
//
//------------------------------------------------------------------------------------------------------------
struct ControllerInst {

    ControllerFamily    family;
    uint32_t            memorySize;
    uint32_t            internalNvmSize;
    uint16_t            cores;
};

//------------------------------------------------------------------------------------------------------------
// Watchdog Instance.
//
//------------------------------------------------------------------------------------------------------------
struct WatchDogInst {

    bool        configured;
    uint32_t    intervalMillis;
};

//------------------------------------------------------------------------------------------------------------
// A timer instance. We currently support inly one HW timer and Configuration Descriptor.
//
//------------------------------------------------------------------------------------------------------------
struct TimerInst {

    bool                configured;
    repeating_timer_t   timerData;
};

//------------------------------------------------------------------------------------------------------------
// A GPIO instance is the most basic digital IO channel.
//
//------------------------------------------------------------------------------------------------------------
struct GpioInst {

    bool    configured;
    uint8_t pin;
};

//------------------------------------------------------------------------------------------------------------
// An ADC instance. The PICO supports up to three ADC inputs. When we use such an input, the corresponding 
// instance data is kept in this structure. We also keep the PICO ADC number, so we can select the correct
//  instance.
//
//------------------------------------------------------------------------------------------------------------
struct AdcInst {

    bool      configured;
    uint8_t   adcPin;
    uint8_t   adcNum;
};

//------------------------------------------------------------------------------------------------------------
// A PWM output instance. GPIO pins can also be used as PWM output pins. The PWM output related data is kept
// in this instance. The PICO features a set of PWM slices, each of which has two channels. We will use the 
// two channels for our PWM signal, where one pin is the PWM signal while the other is held low.
//
//------------------------------------------------------------------------------------------------------------
struct PwmInst {

    bool        configured;
    uint8_t     pwmPinA;
    uint8_t     pwmPinB;
    uint        wrap;
    bool        inverted;
    uint        channelA;
    uint        channelB;
    uint        sliceNum;
};

//------------------------------------------------------------------------------------------------------------
// A UART instance. UARTS are used to read in a serial stream from the RailCom detectors. There can be two 
// hardware based UART instances, or up to four software defined instances. The instance also keeps a small 
// buffer where the data is read into. We also keep the PICO UART HW instance used.
//
//------------------------------------------------------------------------------------------------------------
struct UartInst {

    bool              configured;
    uint8_t           rxPin;
    uint8_t           txPin;
    uint16_t          baudSetting;
    uint8_t           dataBits;
    uart_parity_t     parityMode;
    uint8_t           stopBits;
    int               uartIrq;
    uint8_t           uartMode;

    volatile uint8_t  rxBufIndex;
    volatile uint8_t  rxDataBuf[ MAX_UART_BUF_SIZE ];

    uart_inst_t       *uartHw;
};

//------------------------------------------------------------------------------------------------------------
// The I2C instance. The PICO features two HW instances of an I2C port. The instance data contains the 
// assigned GPIO pins, the baud rate and a timeout. We also keep the I2C HW instance used.
//
//------------------------------------------------------------------------------------------------------------
struct I2CInst {

    bool        configured;
    uint8_t     sclPin;
    uint8_t     sdaPin;
    uint32_t    baudRate;
    uint32_t    timeoutValMs;

    i2c_inst_t  *i2cHw;
};

//------------------------------------------------------------------------------------------------------------
// The SPI instance. The PICO features two SPI HW instances. We keep the assigned GPIO pins for the SPI 
// interface as well as the PICO HW instance. Since the SPI protocol explicitly sets the selected HW select 
// pin, we remember that we are in a transaction with perhaps more than one call to the SPI routines.
//
//------------------------------------------------------------------------------------------------------------
struct SPIInst {

    bool        configured;
    bool        active;
    uint8_t     selectPin;
    uint8_t     mosiPin;
    uint8_t     misoPin;
    uint8_t     sclkPin;
    uint32_t    frequency;

    spi_inst_t  *spiHw;
};

//------------------------------------------------------------------------------------------------------------
// The CAN bus instance. Although our current controller do not feature a CAN bus, the instance described 
// the hardware elements needed from the controller. Currently, we use a software version based on a PIO
// program to implement the CAN bus. 
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusInst {

    uint8_t     canPin1;
    uint8_t     canPin2;
    uint32_t    baudRate;
};

//------------------------------------------------------------------------------------------------------------
// The interrupt table for the GPIO pin interrupts. The PICO can have only one interrupt handler. We will
// allocate a table where a handler can be set for each pin. When an interrupt comes in and there is a 
// handler configured, it will be called.
//
//------------------------------------------------------------------------------------------------------------
struct GpioIsrTable {

    uint16_t                numOfHandlers = 0;
    GpioCallback            gpioIsrTable[ MAX_CPU_CORE ][ MAX_INT_PIN + 1 ];
};

//------------------------------------------------------------------------------------------------------------
// Controller hardware is configured via Instances. An instance groups a hardware peripheral function with 
// all settings and hardware pins it may need. These instances are then referred by the CDC routines with an
// index into the instance array. This array is populated during initial configuration. An application can 
// locate this index by using the lookup function which accepts an identifier name. For example, PWM_INST_0,
// is the resId for a PWM channel that was configured using this name.
//
//------------------------------------------------------------------------------------------------------------
struct CdcInstance {

    bool            configured;
    uint16_t        resId;
    CdcInstanceType type;

    union {

        ControllerInst  ctl;
        WatchDogInst    wd;
        TimerInst       timer;
        GpioInst        gpio;
        PwmInst         pwm;
        UartInst        uart;
        AdcInst         adc;
        I2CInst         i2c;
        SPIInst         spi;
        CanBusInst      can;
    };
};

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
struct CdcInstanceMap {

    uint16_t    size;
    CdcInstance map[ MAX_INST_ENTRIES ];
};



// ??? we need a configure routine for each instance type
// ??? we need a lookup function to fund the instance by "instance name"
// ??? we need a function to take an array of instance descriptors and execute the config routines...

// ??? the instance descriptors can be grouped based on the board/version as a constant data structure.
// ??? we could store a board type in the board NVM to fund the board descriptor. 
// ??? there is a default descriptor ( minimum data, such as NVM I2C, etc. ) to prepare a board and 
// let it react to config commands.




//------------------------------------------------------------------------------------------------------------
// Local variables. We maintain an instance variable for each of the possible HW entities, such as an I2C
// interface or a UART. Note that not all are used at the same time. The instance variables map from the
// simple pin numbers to the PICO structures and whatever else we need to remember for this entity.
//
// ??? this will be reworked and mostly disappear...
//------------------------------------------------------------------------------------------------------------
CdcInstanceMap instanceMap;




CDC::CdcConfigDesc          cfg;
CDC::TimerCallback          timerCallback = nullptr;
GpioIsrTable                cdcIntHandlers;
repeating_timer_t           timerData;
WatchDogInst                cdcWatchDog;
AdcInst                     cdcAdc0;
AdcInst                     cdcAdc1;
AdcInst                     cdcAdc2;
AdcInst                     cdcAdc3;
I2CInst                     cdcI2C0;
I2CInst                     cdcI2C1;
SPIInst                     cdcSPI0;
SPIInst                     cdcSPI1;
UartInst                    cdcUart0;
UartInst                    cdcUart1;
UartInst                    cdcUart2;
UartInst                    cdcUart3;
PwmInst                     cdcPwm0;
PwmInst                     cdcPwm1;
PwmInst                     cdcPwm2;
PwmInst                     cdcPwm3;
PwmInst                     cdcPwm4;
PwmInst                     cdcPwm5;
PwmInst                     cdcPwm6;
PwmInst                     cdcPwm7;

//------------------------------------------------------------------------------------------------------------
// "validPin" is called to check that a pin is in the correct number range, defined and matches the bitmask
// for the desired purpose. For example, configuring an I2C port will check that the two GPIO pins are
// indeed routable to the I2C HW block in the PICO.
//
//------------------------------------------------------------------------------------------------------------
bool validPin( uint8_t pin, uint32_t mask ) {

    if ( pin == CDC::UNDEFINED_PIN )  return ( true );
    if ( pin > MAX_PIN_NUM )          return ( false );
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

        case CDC::EVT_LOW:     return ( GPIO_IRQ_LEVEL_LOW );
        case CDC::EVT_HIGH:    return ( GPIO_IRQ_LEVEL_HIGH );
        case CDC::EVT_FALL:    return ( GPIO_IRQ_EDGE_FALL );
        case CDC::EVT_RISE:    return ( GPIO_IRQ_EDGE_RISE );
        case CDC::EVT_CHANGE:  return ( GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL );
        default:      return ( 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// The PICO uses a set of constants to describe the interrupt type. We map them to our types. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t mapPicoGpioEvent( uint32_t event ) {

    switch ( event ) {

        case GPIO_IRQ_LEVEL_LOW:  return ( CDC::EVT_LOW );
        case GPIO_IRQ_LEVEL_HIGH: return ( CDC::EVT_HIGH );
        case GPIO_IRQ_EDGE_FALL:  return ( CDC::EVT_FALL );
        case GPIO_IRQ_EDGE_RISE:  return ( CDC::EVT_RISE );
        default:                  return ( 0 );
    }
}

//------------------------------------------------------------------------------------------------------------
// Global Interrupt handlers. The hardware and low level library will call these handlers, which in turn 
// will invoke the respective callback function if configured. The GPIO interrupt handler manages the 
// handler for all possible IO pins. The PICO can only have one interrupt routine, so we feature an array 
// of handlers where a handler for a GPIO pin can be registered. If there is a handler set, we just invoke 
// it. The other handlers are for the timer and the UART hardware.
//
//------------------------------------------------------------------------------------------------------------
void gpioCallback( uint gpioPin, uint32_t event ) {

    cdcIntHandlers.gpioIsrTable[ get_core_num( )][ gpioPin] ( gpioPin, mapPicoGpioEvent( event ));
}

bool repeatingTimerAlarm( repeating_timer_t *rt ) {

    if ( timerCallback != nullptr ) timerCallback((uint32_t) ( - timerData.delay_us ));
    return ( true );
}

void uartRxCallback0( ) {

    while ( uart_is_readable( uart0 )) {

        uint8_t ch = uart_getc( uart0 );
        if ( cdcUart0.rxBufIndex < MAX_UART_BUF_SIZE ) cdcUart0.rxDataBuf[cdcUart0.rxBufIndex++ ] = ch;
    }
}

void uartRxCallback1( ) {

    while ( uart_is_readable( uart1 )) {

        uint8_t ch = uart_getc( uart1 );
        if ( cdcUart1.rxBufIndex < MAX_UART_BUF_SIZE ) cdcUart1.rxDataBuf[ cdcUart1.rxBufIndex++ ] = ch;
    }
}








//------------------------------------------------------------------------------------------------------------
// The default configuration descriptor. The Application program fills in such a structure, which can be
// seen as the HW pin assignments for the PICO controllers and the particular board on which the application
// will be deployed. The application will simply use the field names to address the particular PICO HW
// function. For example, a configuration has mapped DIO_PIN_5 to GPIO pin 12, because that is where the 
// particular board has mapped DIO_PIN_5 to the hardware line. The application will just use the DIO_PIN_5 
// field when talking to that GPIO pin. Whenever the board layout changes, there could be another PICO GPIO 
// pin, but the name "DIO_PIN_5" for the application upper layers does not change.
//
// Note that there is a great flexibility what a PICO HW  pin can do and hence a lot of our fields are just
// "UNDEFINED" with no constraints. Nevertheless, there is a function which will do some plausibility checks
// for such a structure. Also, each configuration routine will do again a check that the GPIO pins used do
// indeed map to a PICO HW block for the desired purpose.
//
// The configuration structure does not replace the actual configuration calls to make to the CDC library.
// It is just a mapping of reserved names to actual GPIO pins.
//
// ??? this will change ....
//------------------------------------------------------------------------------------------------------------
CDC::CdcConfigDesc getConfigDefaultRP2040( ) {

    CDC::CdcConfigDesc tmp;

    tmp.CFG_STATUS          = CDC::INIT_PENDING;

    // ??? controller family ?
    // ??? what other characteristics ? ( e.g. mem size ? )

    tmp.ACTIVE_LED_PIN      = CDC::UNDEFINED_PIN;

    tmp.EXT_INT_PIN         = CDC::UNDEFINED_PIN;
    tmp.PFAIL_PIN           = CDC::UNDEFINED_PIN;

    tmp.DIO_PIN_0           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_1           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_2           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_3           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_4           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_5           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_6           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_7           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_8           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_9           = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_10          = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_11          = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_12          = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_13          = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_14          = CDC::UNDEFINED_PIN;
    tmp.DIO_PIN_15          = CDC::UNDEFINED_PIN;

    tmp.ADC_PIN_0           = CDC::UNDEFINED_PIN;
    tmp.ADC_PIN_1           = CDC::UNDEFINED_PIN;
    tmp.ADC_PIN_2           = CDC::UNDEFINED_PIN;
    tmp.ADC_PIN_3           = CDC::ILLEGAL_PIN;

    tmp.PWM_PIN_0           = CDC::UNDEFINED_PIN;
    tmp.PWM_PIN_1           = CDC::UNDEFINED_PIN;
    tmp.PWM_PIN_2           = CDC::UNDEFINED_PIN;
    tmp.PWM_PIN_3           = CDC::UNDEFINED_PIN;

    tmp.UART_RX_PIN_0       = CDC::UNDEFINED_PIN;
    tmp.UART_TX_PIN_0       = CDC::UNDEFINED_PIN;

    tmp.UART_RX_PIN_1       = CDC::UNDEFINED_PIN;
    tmp.UART_TX_PIN_1       = CDC::UNDEFINED_PIN;

    tmp.UART_RX_PIN_2       = CDC::UNDEFINED_PIN;
    tmp.UART_TX_PIN_2       = CDC::UNDEFINED_PIN;

    tmp.UART_RX_PIN_3       = CDC::UNDEFINED_PIN;
    tmp.UART_TX_PIN_3       = CDC::UNDEFINED_PIN;

    tmp.SPI_MOSI_PIN_0      = CDC::UNDEFINED_PIN;
    tmp.SPI_MISO_PIN_0      = CDC::UNDEFINED_PIN;
    tmp.SPI_SCLK_PIN_0      = CDC::UNDEFINED_PIN;

    tmp.SPI_MOSI_PIN_1      = CDC::UNDEFINED_PIN;
    tmp.SPI_MISO_PIN_1      = CDC::UNDEFINED_PIN;
    tmp.SPI_SCLK_PIN_1      = CDC::UNDEFINED_PIN;

    tmp.NVM_I2C_SCL_PIN     = CDC::UNDEFINED_PIN;
    tmp.NVM_I2C_SDA_PIN     = CDC::UNDEFINED_PIN;

    tmp.EXT_I2C_SCL_PIN     = 17;
    tmp.EXT_I2C_SDA_PIN     = 16;

    tmp.CAN_BUS_RX_PIN      = CDC::UNDEFINED_PIN;
    tmp.CAN_BUS_TX_PIN      = CDC::UNDEFINED_PIN;

    return ( tmp );
}

//------------------------------------------------------------------------------------------------------------
// Validate a configuration structure. This routine will do basic checking of the pin configuration passed.
// The PICO is very flexible when it comes to what a pin can do. However, there are still some rules to 
// follow. Also, we have dedicated settings for at least the I2C channels and the CAN bus IO pins.
//
// ??? still needed, but done internally...
//------------------------------------------------------------------------------------------------------------
uint8_t validateConfigRP20040( CDC::CdcConfigDesc *ci ) {

    // ??? a ton of "validXXX" ?

    return ( NO_ERR ); // for now....
}


//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
void initInstanceMap( ) {

    instanceMap.size = MAX_INST_ENTRIES;
   
    for ( int i = 0; i < MAX_INST_ENTRIES; i++ ) {

        CdcInstance *entry = &instanceMap.map[ i ];

        entry -> configured = false;
        entry -> type       = CDC_IT_UNDEFINED;
        entry -> resId      = 0;
    }
}

CdcInstance *getFreeInstance( ) {

    for ( uint16_t i = 0; i < instanceMap.size; i++ ) {

        if ( instanceMap.map[ i ].type == CDC_IT_UNDEFINED ) return( &instanceMap.map[ i ] );
    } 

    return( nullptr );
}

CdcInstance *getInstanceByResId( uint16_t resId ) {

    for ( uint16_t i = 0; i < instanceMap.size; i++ ) {

        if ( instanceMap.map[ i ].resId == resId ) return( &instanceMap.map[ i ] );
    } 

    return( nullptr );
}

CdcInstance *getInstanceByHandle( uint8_t handle, CdcInstanceType type ) {

    CdcInstance *entry = &instanceMap.map[ handle ];
    return((( entry -> configured ) && ( entry -> type == type )) ? entry : nullptr );
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

// ??? goes away...
//------------------------------------------------------------------------------------------------------------
// "getConfigDefault" initializes a configuration structure and sets the pre-assigned values. A typical
// sequence for an application start sequence would be to create an initial structure this way and then set
// the relevant pins and values according to the actual hardware configuration.
//
//------------------------------------------------------------------------------------------------------------
CdcConfigDesc getConfigDefault( ) {

   return ( getConfigDefaultRP2040( ));
}

// ??? goes away...
//------------------------------------------------------------------------------------------------------------
// "getConfigActual" will return a pointer to the copy we kept when calling the init routine with the config
// structure to use. There is no need for the upper layers to keep the structure used at initialization time.
//
//------------------------------------------------------------------------------------------------------------
CdcConfigDesc *getConfigActual( ) {

   return ( &cfg );
}

//------------------------------------------------------------------------------------------------------------
// CDC library setup. The "init" routine will ready the CDC library. The main task is to validate the pins and
// values for the particular controller capabilities. The init routine can be called more than once without a
// problem.
//
//------------------------------------------------------------------------------------------------------------
uint8_t init( CdcConfigDesc *ci ) {

    cfg = *ci;

    initIsrTable( );
    configureConsoleIO( );

    // ??? set up instance table ...

    return ( validateConfigRP20040( ci ));
}

//------------------------------------------------------------------------------------------------------------
// - a routine to take the descriptor array and just configure all the instances...
//
// 
//------------------------------------------------------------------------------------------------------------
uint8_t configureCdcSubSytem( CdcInstanceDescMap *map ) {


    return( 0 ); // for now ...
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
// WatchDog function. We need a way to restart a failed or looping node without going there hitting the 
// reset button. The idea is that there is a watch dog timer. If the watchDog is not updated every N milli
// seconds it will reboot the node. 
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureWatchDog( uint8_t *handle, uint32_t millis ) {

    CdcInstance *ptr = getInstanceByResId( CDC_RID_WATCHDOG );

    if ( ptr == nullptr ) {

        ptr = getFreeInstance( );
        if ( ptr == nullptr ) return( MAX_RES_ID_ERR );
    }

    ptr -> configured = true;
    ptr -> wd.intervalMillis = millis;

    *handle = ( ptr - instanceMap.map ) + 1;
    return ( NO_ERR );
}

uint8_t watchDogEnable( uint8_t handle, bool enable ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_WATCHDOG );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    watchdog_enable( ptr -> wd.intervalMillis, 1 );
    return ( NO_ERR );
}

uint8_t watchDogUpdate( uint8_t handle ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_WATCHDOG );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    watchdog_update( );
    return ( NO_ERR );
}

uint8_t watchDogCausedReboot( uint8_t handle, bool *reboot ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_WATCHDOG );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    return ( watchdog_caused_reboot( ));
}

//------------------------------------------------------------------------------------------------------------
// Processor general values required by the low level LCS core library functions. A controller is configured
// just like any other resource, although we cannot set most of the values. For example, main memory size
// is a given. Since these values are still not that easy to get from the controller chip directly, we
// provide a little help through "configuring" these values.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureController( uint8_t *handle ) {  

    // ??? what parameters can we actually configure ?
    // ??? to do ...

    return( NO_ERR );
}

uint8_t getFamily( uint8_t handle, ControllerFamily *family ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_CONTROLLER );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    *family = CDC_CF_RP_PICO_2040;
    return ( NO_ERR );
}

uint8_t getVersion( uint8_t handle, uint32_t *version ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_CONTROLLER );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    *version = ( CDC_LIB_MAJOR_VERSION << 8 | CDC_LIB_MINOR_VERSION );
    return ( NO_ERR );
}

uint8_t getChipMemSize( uint8_t handle, uint32_t *size ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_CONTROLLER );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    *size = CHIP_MEM_SIZE;
    return ( NO_ERR );
}

uint8_t getChipNvmSize( uint8_t handle, uint32_t *size ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_CONTROLLER );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    *size = CHIP_NVM_SIZE;
    return ( NO_ERR );
}

uint8_t getCpuFrequency( uint8_t handle, uint32_t *frequency ) {

    CdcInstance *ptr = getInstanceByHandle( handle, CDC_IT_CONTROLLER );
    if ( ptr == nullptr ) return ( INVALID_HANDLE_ERR );

    *frequency = clock_get_hz( clk_sys );
    return ( NO_ERR );
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
// rather go with the risk that there is just power on the USB connector.
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
// ??? would we one day need more than one timer instance ?
//------------------------------------------------------------------------------------------------------------
void startRepeatingTimer( uint32_t val ) {

    int64_t limit = val;
    add_repeating_timer_us( - limit, repeatingTimerAlarm, nullptr, &timerData );
}

void stopRepeatingTimer( ) {

    cancel_repeating_timer( &timerData );
}

uint32_t getRepeatingTimerLimit( ) {

    return ((uint32_t) ( - timerData.delay_us ));
}

void setRepeatingTimerLimit( uint32_t val ) {

    int64_t limit = val;
    timerData.delay_us = ((int64_t) - limit );
}

void onTimerEvent( CDC::TimerCallback functionId ) {

    timerCallback = functionId;
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
uint8_t configureDio( uint8_t dioPin, uint8_t mode ) {

    if ( ! validPin( dioPin, VALID_GPIO_PINS )) return ( DIO_PIN_ERR );

    gpio_init( dioPin );

    switch ( mode ) {

        case DIO_IN:  gpio_set_dir( dioPin, false );  break;
        case DIO_OUT: {

            gpio_set_dir( dioPin, true );
            gpio_set_drive_strength ( dioPin, GPIO_DRIVE_STRENGTH_12MA );

        }  break;

        case DIO_IN_PULLUP: {

            gpio_set_dir( dioPin, false );
            gpio_pull_up( dioPin );

        } break;

        default: gpio_set_dir( dioPin, false );
    }

  return ( NO_ERR );
}

void registerDioCallback( uint8_t dioPin, uint8_t event, CDC::GpioCallback func ) {

    if ( dioPin <= MAX_INT_PIN ) {

        if ( cdcIntHandlers.numOfHandlers == 0 ) 
            gpio_set_irq_enabled_with_callback( dioPin, mapGpioIntEvent( event ), true, gpioCallback );
        else
            gpio_set_irq_enabled( dioPin, mapGpioIntEvent( event ), true);
    
        cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ dioPin ] = func;
        cdcIntHandlers.numOfHandlers ++;
    }
}

void unregisterDioCallback( uint8_t dioPin ) {

    if ( dioPin <= MAX_INT_PIN ) {

        if ( cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ dioPin ] != nullptr ) {

            gpio_set_irq_enabled( dioPin, 0, false );
            cdcIntHandlers.gpioIsrTable[ get_core_num( ) ][ dioPin ] = dummyIsrHandler;
            cdcIntHandlers.numOfHandlers --;
        }
    }
}

bool readDio( uint8_t dioPin ) {

    return ( gpio_get( dioPin ));
}

uint8_t writeDio( uint8_t dioPin, bool val ) {

    gpio_put( dioPin, val );
    return ( NO_ERR );
}

uint8_t toggleDio( uint8_t dioPin ) {

    writeDio( dioPin, ! readDio( dioPin ));
    return ( NO_ERR );
}

uint8_t writeDioPair( uint8_t dioPin1, bool val1, uint8_t dioPin2, bool val2 ) {

    uint32_t maskData = ( 1UL << dioPin1 ) | ( 1UL << dioPin2 );
    uint32_t valData  = (( val1 ) ? ( 1 << dioPin1 ) : 0 ) | (( val2 ) ? ( 1 << dioPin2 ) : 0 );

    gpio_put_masked( maskData, valData );
    return ( NO_ERR );
}

uint32_t readDioMask( uint32_t dioMask ) {

    return ( gpio_get_all( ) & dioMask );
}

uint8_t writeDioMask( uint32_t dioMask, uint32_t dioVal ) {

    gpio_put_masked( dioMask, dioVal );
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// ADC section. The analog input channel represented by the pin is configured. At initialization, the ADC pin
// number is validated and the ADC subsystem initialized. The PICO does an analog read in about 2us. This is
// so fast, it does for our purpose make not much sense to implement an asynchronous option. Furthermore, the
// ADC value scaled down to a 10-bit resolution.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureAdc( uint8_t adcPin ) {

    if ( ! validPin( adcPin, VALID_ADC_PINS ))  return ( ADC_PIN_ERR );

    AdcInst *tmp = nullptr;

    if ( adcPin == cfg.ADC_PIN_0 ) {

        tmp = &cdcAdc0;
        tmp -> adcPin = adcPin;
        tmp -> adcNum = 0;
        
    }
     else if ( adcPin == cfg.ADC_PIN_1 ) {

        tmp = &cdcAdc1;
        tmp -> adcPin = adcPin;
        tmp -> adcNum = 1;
    }
    else if ( adcPin == cfg.ADC_PIN_2 ) {

        tmp = &cdcAdc2;
        tmp -> adcPin = adcPin;
        tmp -> adcNum = 2;
    }
    else return ( ADC_PIN_ERR );

    adc_init( );
    adc_gpio_init( tmp -> adcPin );
    tmp -> configured = true;

    return ( NO_ERR );
}

uint16_t readAdc( uint8_t adcPin ) {

    AdcInst *tmp = nullptr;

    if      ( adcPin == cdcAdc0.adcPin ) tmp = &cdcAdc0;
    else if ( adcPin == cdcAdc1.adcPin ) tmp = &cdcAdc1;
    else if ( adcPin == cdcAdc2.adcPin ) tmp = &cdcAdc2;
    else return ( ADC_PIN_ERR );

    if ( ! tmp -> configured ) return ( ADC_PIN_ERR );

    adc_select_input( tmp -> adcNum );
    return ( adc_read( ) >> 2 );
}

uint16_t getAdcRefVoltage( ) {

    return ( ADC_REF_VOLTAGE_MILLI_VOLT );
}

uint16_t getAdcDigitRange( ) {

    return ( ADC_DIGIT_RANGE );
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
uint8_t configureUart( uint8_t rxPin, uint8_t txPin, uint32_t baudRate, UartMode mode ) {

    UartInst *uart = nullptr;

    if ( mode == UART_MODE_8N1 ) {

        if (( validPin( rxPin, VALID_UART_0_RX_PINS )) && ( validPin( txPin, VALID_UART_0_TX_PINS ))) {

            uart                = &cdcUart0;
            uart -> uartMode    = mode;
            uart -> rxPin       = rxPin;
            uart -> txPin       = txPin;
            uart -> dataBits    = 8;
            uart -> stopBits    = 1;
            uart -> parityMode  = UART_PARITY_NONE;
            uart -> uartHw      = uart0;
            uart -> uartIrq     = UART0_IRQ;
        }
        else if (( validPin( rxPin, VALID_UART_1_RX_PINS )) && ( validPin( txPin, VALID_UART_1_TX_PINS ))) {

            uart                = &cdcUart1;
            uart -> uartMode    = mode;
            uart -> rxPin       = rxPin;
            uart -> txPin       = txPin;
            uart -> dataBits    = 8;
            uart -> stopBits    = 1;
            uart -> parityMode  = UART_PARITY_NONE;
            uart -> uartHw      = uart1;
            uart -> uartIrq     = UART1_IRQ;
        }
        else return ( UART_PORT_ERR );

        uart_init( uart -> uartHw, baudRate );
        gpio_set_function( rxPin, GPIO_FUNC_UART );
        gpio_set_function( txPin, GPIO_FUNC_UART );
        uart_set_hw_flow( uart -> uartHw, false, false );
        uart_set_format( uart -> uartHw, uart -> dataBits, uart -> stopBits, uart -> parityMode );
        uart_set_fifo_enabled( uart -> uartHw, false );

        if      ( uart -> uartIrq == UART0_IRQ ) irq_set_exclusive_handler( uart -> uartIrq, uartRxCallback0 );
        else if ( uart -> uartIrq == UART1_IRQ ) irq_set_exclusive_handler( uart -> uartIrq, uartRxCallback1 );

        irq_set_enabled( uart -> uartIrq, true );
        return ( NO_ERR );
    }
    else if ( mode == UART_MODE_8N1_PIO ) {

        // ??? this is the place where we deal with a PIO version of the UART.

        return ( NOT_SUPPORTED );
    }
    else return ( NOT_SUPPORTED );
}

uint8_t startUartRead( uint8_t rxPin ) {

    UartInst *uart = nullptr;

    if      ( rxPin == cdcUart0.rxPin ) uart = &cdcUart0;
    else if ( rxPin == cdcUart1.rxPin ) uart = &cdcUart1;
    else if ( rxPin == cdcUart2.rxPin ) uart = &cdcUart2;
    else if ( rxPin == cdcUart3.rxPin ) uart = &cdcUart3;
    else                                return ( UART_PORT_ERR );

    if ( ! uart -> configured ) return ( UART_PORT_ERR );

    if (( uart != nullptr ) && ( uart -> uartMode == UART_MODE_8N1 )) {

        uart_set_irq_enables( uart -> uartHw, true, false );
        uart -> rxBufIndex = 0;
        return ( NO_ERR );
    }
    else if (( uart != nullptr ) && ( uart -> uartMode == UART_MODE_8N1_PIO )) {

        return ( NOT_SUPPORTED );
    }
    else return ( UART_PORT_ERR );
}

uint8_t stopUartRead( uint8_t rxPin ) {

    UartInst *uart = nullptr;

    if      ( rxPin == cdcUart0.rxPin ) uart = &cdcUart0;
    else if ( rxPin == cdcUart1.rxPin ) uart = &cdcUart1;
    else if ( rxPin == cdcUart2.rxPin ) uart = &cdcUart2;
    else if ( rxPin == cdcUart3.rxPin ) uart = &cdcUart3;
    else                                return ( UART_PORT_ERR );

    if ( ! uart -> configured ) return ( UART_PORT_ERR );

    if ( uart ->uartMode == UART_MODE_8N1 ) {

        uart_set_irq_enables( uart -> uartHw, false, false );
        return ( NO_ERR );
    }
    else if ( uart -> uartMode == UART_MODE_8N1_PIO ) {

        return ( NOT_SUPPORTED );
    }
    else return ( UART_PORT_ERR );
}

uint8_t getUartBuffer( uint8_t rxPin, uint8_t *buf, uint8_t bufLen ) {

    UartInst *uart = nullptr;

    if      ( rxPin == cdcUart0.rxPin ) uart = &cdcUart0;
    else if ( rxPin == cdcUart1.rxPin ) uart = &cdcUart1;
    else if ( rxPin == cdcUart2.rxPin ) uart = &cdcUart2;
    else if ( rxPin == cdcUart3.rxPin ) uart = &cdcUart3;
    else                                return ( UART_PORT_ERR );

    if ( ! uart -> configured ) return ( UART_PORT_ERR );

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
// PWM section. The PICO is quite flexible when it comes to PWM signals. We implement a simple PWM capability.
// There is the frequency which set during configuration and there is the write operation which set the duty
// cycle. The calculations are best described in the PICO C++ SDK. We do the setting of phase, wrap count,
// etc. once when we configure the PWM channels. Our PWM configuration supports the simultaneous level setting
// of the two associated pins. This way we can model a two pin PWM signal, where either pin contains the 
// PWM signal and  the other is set to low. Note that although the PICO is quite flexible, the wrap and phase
// parameters are set for the slice and not a single channel. The same is true for the signal inverter. This
// is normally not an issue unless you want to have separate frequencies, phase mode and so on. 
//
// The "writePwm" function will just manipulate the duty cycle. When we need to change the frequency we need
// to configure again. 
// 
//------------------------------------------------------------------------------------------------------------
PwmInst *lookupPwmInst( uint8_t pwmPin ) {

    if      ( pwmPin == cfg.PWM_PIN_0 ) return ( &cdcPwm0 );
    else if ( pwmPin == cfg.PWM_PIN_1 ) return ( &cdcPwm1 );
    else if ( pwmPin == cfg.PWM_PIN_2 ) return ( &cdcPwm2 );
    else if ( pwmPin == cfg.PWM_PIN_3 ) return ( &cdcPwm3 );
    else if ( pwmPin == cfg.PWM_PIN_4 ) return ( &cdcPwm4 );
    else if ( pwmPin == cfg.PWM_PIN_5 ) return ( &cdcPwm5 );
    else if ( pwmPin == cfg.PWM_PIN_6 ) return ( &cdcPwm6 );
    else if ( pwmPin == cfg.PWM_PIN_7 ) return ( &cdcPwm7 );
    else                                return ( nullptr );
}

uint8_t configurePwm(   uint8_t     pwmPinA, 
                        uint8_t     pwmPinB, 
                        uint32_t    pwmFreqency, 
                        bool        phaseCorrect,
                        bool        inverted ) {

    PwmInst *pwm = lookupPwmInst( pwmPinA );
    if ( pwm == nullptr ) return ( PWM_PIN_ERR );

    if ( pwmPinB != UNDEFINED_PIN ) {

        if ( pwm_gpio_to_slice_num( pwmPinA ) != ( pwm_gpio_to_slice_num( pwmPinB ))) return ( PWM_PIN_ERR );
    }

    if ( phaseCorrect ) pwmFreqency = pwmFreqency * 2;

    uint32_t sysClock = clock_get_hz( clk_sys );
    uint32_t clkDiv   = sysClock / pwmFreqency / 4096 + ( sysClock % ( pwmFreqency * 4096 ) != 0 );
    if ( clkDiv / 16 == 0 ) clkDiv = 16;

    pwm -> pwmPinA      = pwmPinA;
    pwm -> pwmPinB      = pwmPinB;
    pwm -> wrap         = sysClock * 16 / clkDiv / pwmFreqency - 1;
    pwm -> inverted     = inverted;

    pwm -> sliceNum     = pwm_gpio_to_slice_num( pwmPinA );
    pwm -> channelA     = pwm_gpio_to_channel( pwmPinA );
    if ( pwmPinB != UNDEFINED_PIN ) pwm -> channelB = pwm_gpio_to_channel( pwmPinB );

    pwm_config pwmConfig = pwm_get_default_config( );
    gpio_set_function( pwm -> pwmPinA, GPIO_FUNC_PWM );
    if ( pwmPinB != UNDEFINED_PIN ) gpio_set_function( pwm -> pwmPinB, GPIO_FUNC_PWM );
    pwm_config_set_wrap( &pwmConfig, pwm -> wrap );
    pwm_config_set_phase_correct( &pwmConfig, phaseCorrect );
    pwm_config_set_output_polarity( &pwmConfig, pwm -> inverted, pwm -> inverted );
    pwm_init( pwm -> sliceNum, &pwmConfig, false );
    pwm_set_clkdiv_int_frac( pwm -> sliceNum, clkDiv / 16, clkDiv & 0xF );
    pwm_set_enabled( pwm -> sliceNum, true );

    pwm -> configured = true;

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_PWM )) {
   
        printf( "pinA: % d, pinB: %d, fPwm: % d, phase: % d, inverted: % d, " 
                "clkDiv: % d, wrap: %d, sliceNum: %d, channelA: %d, channelB: %d\n",
                pwm -> pwmPinA, pwm -> pwmPinB, pwmFreqency, phaseCorrect, inverted, 
                clkDiv, pwm -> wrap, pwm -> sliceNum, pwm -> channelA, pwm -> channelB );
    }

    return ( NO_ERR );
}

uint8_t writePwm( uint8_t pwmPin, uint8_t dutyCycleA, uint8_t dutyCycleB ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_PWM )) {
        
        printf( "Write PWM: Pin: %d, dutyA: %d, dutyB: %d\n", pwmPin, dutyCycleA, dutyCycleB );
    }
   
    PwmInst *pwm = lookupPwmInst( pwmPin );
    if (( pwm == nullptr ) && ( ! pwm -> configured )) return ( PWM_PIN_ERR );

#if 1

        pwm_set_both_levels( pwm -> sliceNum, dutyCycleA, dutyCycleB );
#else

    // will go away. 
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

uint8_t syncPwm( uint8_t pwmPin ) {

    PwmInst *pwm = lookupPwmInst( pwmPin );
    if (( pwm == nullptr ) && ( ! pwm -> configured )) return ( PWM_PIN_ERR );

    pwm_set_counter( pwm ->sliceNum, 0 );
    
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// I2C Section. The PICO has two HW blocks for I2C interfaces. The interface implements a simple read and
// write access to an I2C element. There is a timeout to avoid waiting forever on an operation.
//
//------------------------------------------------------------------------------------------------------------
uint8_t configureI2C( uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate ) {

    I2CInst *i2c = nullptr;

    if ((( 1 << sclPin ) & VALID_I2C_0_SCL_PINS ) && (( 1 << sdaPin ) & VALID_I2C_0_SDA_PINS )) {

        i2c = &cdcI2C0;
        i2c -> i2cHw = i2c0;
    }
    else if ((( 1 << sclPin ) & VALID_I2C_1_SCL_PINS ) && (( 1 << sdaPin ) & VALID_I2C_1_SDA_PINS )) {

        i2c = &cdcI2C1;
        i2c -> i2cHw = i2c1;
    }
    else return ( CDC::I2C_PORT_ERR );

    i2c -> sclPin       = sclPin;
    i2c -> sdaPin       = sdaPin;
    i2c -> baudRate     = baudRate;
    i2c -> timeoutValMs = I2C_TIME_OUT_IN_MS;
    i2c -> configured   = true;

    i2c_init( i2c -> i2cHw, i2c -> baudRate );
    i2c_set_slave_mode( i2c -> i2cHw, false, 0 );
    
    gpio_set_function( i2c -> sclPin, GPIO_FUNC_I2C );
    gpio_set_function( i2c -> sdaPin, GPIO_FUNC_I2C);
    gpio_pull_up( i2c -> sclPin );
    gpio_pull_up( i2c -> sdaPin );

    return ( NO_ERR );
}

uint8_t i2cRead( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    I2CInst *i2c = nullptr;

    if      (( cdcI2C0.sclPin == sclPin ) && ( cdcI2C0.configured )) i2c = &cdcI2C0;
    else if (( cdcI2C1.sclPin == sclPin ) && ( cdcI2C1.configured )) i2c = &cdcI2C1;
    else return ( I2C_PORT_ERR );

    if ( ! i2c -> configured ) return ( I2C_PORT_ERR );

    auto ret = i2c_read_blocking_until( i2c -> i2cHw,
                                        i2cAdr,
                                        buf,
                                        len,
                                        stopBit,
                                        make_timeout_time_ms( i2c -> timeoutValMs ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {

        printf( "i2cRead: scl: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                sclPin, i2cAdr, buf, buf[0], buf[1], len, stopBit );

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {
            
            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C read, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C read, PICO timeout error\n" );
        }
    }
   
    if (( ret == PICO_ERROR_GENERIC ) || ( ret == PICO_ERROR_TIMEOUT )) return ( I2C_READ_ERR );
    return ( NO_ERR );
}

uint8_t i2cWrite( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit ) {

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {
        
        printf( "i2cWrite: scl: %d, i2c: 0x%x, buf: %p, buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                sclPin, i2cAdr, buf, buf[0], buf[1], len, stopBit );
    }

    I2CInst *i2c = nullptr;

    if      (( cdcI2C0.sclPin == sclPin ) && ( cdcI2C0.configured )) i2c = &cdcI2C0;
    else if (( cdcI2C1.sclPin == sclPin ) && ( cdcI2C1.configured )) i2c = &cdcI2C1;
    else return ( I2C_PORT_ERR );

    if ( ! i2c -> configured ) return ( I2C_PORT_ERR );

    auto ret = i2c_write_blocking_until( i2c -> i2cHw,
                                         i2cAdr,
                                         buf,
                                         len,
                                         stopBit,
                                         make_timeout_time_ms( i2c -> timeoutValMs ));

    if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_PWM )) {

        if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {

            if ( ret == PICO_ERROR_GENERIC ) printf( "I2C write, PICO generic error\n" );
            if ( ret == PICO_ERROR_TIMEOUT ) printf( "I2C write, PICO timeout error\n" );
        }
    }
    
    if (( ret == PICO_ERROR_TIMEOUT) || ( ret == PICO_ERROR_GENERIC ) || ( ret != len )) return ( I2C_WRITE_ERR );
    return ( NO_ERR );
}

uint8_t i2cBusreset( uint8_t sclPin ) {

     if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_I2C )) {

        printf( "I2C Bus reset, slc: %d\n",sclPin );
    }

    I2CInst *i2c = nullptr;

    if      (( cdcI2C0.sclPin == sclPin ) && ( cdcI2C0.configured )) i2c = &cdcI2C0;
    else if (( cdcI2C1.sclPin == sclPin ) && ( cdcI2C1.configured )) i2c = &cdcI2C1;
    else return ( I2C_PORT_ERR );

    if ( ! i2c -> configured ) return ( I2C_PORT_ERR );

    uint8_t reset_cmd = 0x06;
    i2c_write_blocking( i2c -> i2cHw, 0x00, &reset_cmd, 1, false); 
    return ( NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// SPI interface section. The PICO features two SPI HW blocks. We implement a simple SPI interface with a
// a fixed set of SPI options for frequency, bit order and mode. One day this may change. We do not take 
// care of the chip select stuff and expect that the caller manages the select pin.
//
// ??? untested !!!!
//------------------------------------------------------------------------------------------------------------
uint8_t configureSPI( uint8_t sclkPin, uint8_t mosiPin, uint8_t misoPin, uint32_t baudRate ) {

    SPIInst *spi = nullptr;

     if ((( 1 << sclkPin ) & VALID_SPI_0_SCK_PINS ) && 
         (( 1 << mosiPin ) & VALID_SPI_0_TX_PINS  ) &&
         (( 1 << misoPin ) & VALID_SPI_0_RX_PINS  )) {

        spi = &cdcSPI0;
        spi -> spiHw = spi0;
    }
    else if ((( 1 << sclkPin ) & VALID_SPI_1_SCK_PINS ) && 
             (( 1 << mosiPin ) & VALID_SPI_1_TX_PINS  ) &&
             (( 1 << misoPin ) & VALID_SPI_1_RX_PINS  )) {

        spi = &cdcSPI1;
        spi -> spiHw = spi1;
    }  
    else return ( SPI_PORT_ERR );

    spi -> mosiPin     = mosiPin;
    spi -> misoPin     = misoPin;
    spi -> sclkPin     = sclkPin;
    spi -> frequency   = SPI_FREQUENCY;
    spi -> configured  = true;
    spi -> active      = false;

    spi_init( spi -> spiHw, SPI_FREQUENCY );

    spi_set_format( spi -> spiHw,     // SPI instance
                    8,                // Number of bits per transfer
                    SPI_CPOL_1,       // Polarity (CPOL)
                    SPI_CPHA_1,       // Phase (CPHA)
                    SPI_MSB_FIRST );

    gpio_set_function( sclkPin, GPIO_FUNC_SPI );
    gpio_set_function( mosiPin, GPIO_FUNC_SPI );
    gpio_set_function( misoPin, GPIO_FUNC_SPI );

    return ( NO_ERR );
}

uint8_t  spiBeginTransaction( uint8_t sclkPin, uint8_t csPin ) {

    SPIInst *spi = nullptr;

    if      (( cdcSPI0.sclkPin == sclkPin ) && ( cdcSPI0.configured )) spi = &cdcSPI0;
    else if (( cdcSPI1.sclkPin == sclkPin ) && ( cdcSPI1.configured )) spi = &cdcSPI1;
    else return ( SPI_PORT_ERR );

    if ( ! spi -> configured ) return ( SPI_PORT_ERR );

    if ( spi -> active ) {

        // ??? should we check who is active and just ignore when the same ? else "error " ?

        return ( NO_ERR );

    } else {

        spi -> active     = true;
        spi -> selectPin  = csPin;

        CDC::writeDio( csPin, false );
        return ( NO_ERR );
    }
}

uint8_t spiEndTransaction( uint8_t sclkPin, uint8_t csPin ) {

    SPIInst *spi = nullptr;

    if      (( cdcSPI0.sclkPin == sclkPin ) && ( cdcSPI0.configured )) spi = &cdcSPI0;
    else if (( cdcSPI1.sclkPin == sclkPin ) && ( cdcSPI1.configured )) spi = &cdcSPI1;
    else return ( SPI_PORT_ERR );

    if ( ! spi -> configured ) return ( SPI_PORT_ERR );

    if ( spi -> active ) {

        CDC::writeDio( csPin, true );
    
        spi -> active     = false;
        spi -> selectPin  = UNDEFINED_PIN;
        return ( NO_ERR );
    }
    else return ( SPI_NOT_ACTIVE_ERR );
}

uint8_t spiRead( uint8_t sclkPin, uint8_t *buf, uint32_t len ) {

    SPIInst *spi = nullptr;

    if      (( cdcSPI0.sclkPin == sclkPin ) && ( cdcSPI0.configured )) spi = &cdcSPI0;
    else if (( cdcSPI1.sclkPin == sclkPin ) && ( cdcSPI1.configured )) spi = &cdcSPI1;
    else return ( SPI_PORT_ERR );

    if ( ! spi -> configured ) return ( SPI_PORT_ERR );

    if ( spi -> active ) {

        int bytesRead = spi_read_blocking( spi -> spiHw, 0, buf, len );
        return ( NO_ERR );

    } else return ( SPI_NOT_ACTIVE_ERR );
}

uint8_t spiWrite( uint8_t sclkPin, uint8_t *buf, uint32_t len ) {

    SPIInst *spi = nullptr;

    if      (( cdcSPI0.sclkPin == sclkPin ) && ( cdcSPI0.configured )) spi = &cdcSPI0;
    else if (( cdcSPI1.sclkPin == sclkPin ) && ( cdcSPI1.configured )) spi = &cdcSPI1;
    else return ( SPI_PORT_ERR );

    if ( ! spi -> configured ) return ( SPI_PORT_ERR );

    if ( spi -> active ) {

        spi_write_blocking( spi -> spiHw, buf, len );
        return ( NO_ERR );

    } else return ( SPI_NOT_ACTIVE_ERR );
}

//------------------------------------------------------------------------------------------------------------
// Print out the Config Structure.
//
// ??? this will for sure change ... we will print out the array of instances....
//------------------------------------------------------------------------------------------------------------
void printCdcSubSystemInfo( CdcInstanceDescMap *map ) {


}


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

}; // namespace CDC
