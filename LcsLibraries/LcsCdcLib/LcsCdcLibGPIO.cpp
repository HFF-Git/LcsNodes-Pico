//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2025 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <cstring>

#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;

//----------------------------------------------------------------------------------------
// Global Interrupt handlers. The hardware and low level library will call these 
// handlers, which in turn will invoke the respective callback function if configured. 
//
// The GPIO interrupt handler manages the handler for all possible IO pins. The PICO 
// can only have one interrupt routine, so we feature an array of handlers where a 
// handler for a GPIO pin can be registered. 
// 
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
//
//
//----------------------------------------------------------------------------------------
GpioIsrTable dioIntHandlers;

//----------------------------------------------------------------------------------------
// When no interrupt is configured for a GPIO pin, we set the table entry to a dummy
// handler. This way we do not have to check every time for a valid procedure label 
// when we handle an interrupt.
//
//----------------------------------------------------------------------------------------
void dummyIsrHandler ( uint8_t pin, uint8_t event ) { }

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

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void gpioCallback( uint gpioPin, uint32_t event ) {

    dioIntHandlers.gpioIsrTable[ get_core_num( )][ gpioPin ] 
                                     ( gpioPin, mapPicoGpioEvent( event ));
}

} // namespace

//----------------------------------------------------------------------------------------
// Global variables for the CDC lib. Declared in "LcsCdcLib.cpp".
//
//----------------------------------------------------------------------------------------
namespace CDC {

    extern uint16_t                debugMask;
    extern uint16_t                options;

    extern CdcResourceDescMap      dMap;
    extern CdcResourceMap          rMap;

    extern CdcResource *lookupResource( uint8_t rNum, uint8_t type );
    extern CdcResource *allocateResourceType( uint8_t rNum, uint8_t type );

    extern bool validPin( uint8_t pin, uint32_t mask );
}

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

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
// DIO section. A digital pin is the bread and butter hardware resource and can be 
// an input or output pin. For inputs, an internal pull-up resistor can be set.There
// are a couple of interfaces. First the single pin read, write and toggle. Note that
// no cross checking is done if the pins are used by other CDC functions. The DIO 
// routines allow to pass two pins and their values. We often use DIO pins as pairs.
// This is typically used for the H-Bridge control pins, which are set at the same 
// time. 
//
// A GPIO pin can also have an attached interrupt handler. When we register a handler
// for a pin, there are two different PICO lib routines to use. When there is no 
// handler registered so far, we register the common callback and store the particular
// GPIO handler in our ISR handler table. Otherwise, we just store the handler in the
// table and enable the GPIO pin for interrupts. If the resource configured two pins,
// the handler is set for both pins.
//
//----------------------------------------------------------------------------------------
uint8_t configureDio( uint8_t rNum ) {

    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( RES_NUM_ERR );

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_GPIO );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );

    return ( configureDio( rNum, 
                           dPtr -> gpio.pinA, 
                           dPtr -> gpio.pinB, 
                           dPtr -> gpio.pinMode ));
}

uint8_t configureDio( uint8_t rNum, uint8_t pinA, uint8_t pinB, uint8_t mode ) {

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_GPIO )) {

        printf( "configureDio-detail: rNum: %d, pinA: %d, pinB: %d, mode: %d\n", 
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
// Get the HW pin numbers. Used by external non-LCS libraries.
//
//----------------------------------------------------------------------------------------
uint8_t getGpioPinA( uint8_t rNum) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return( rPtr -> gpio.pinA );
}

uint8_t getGpioPinB( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_GPIO );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return( rPtr -> gpio.pinB );
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

} // namespace CDC
