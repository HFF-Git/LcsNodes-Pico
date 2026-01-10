//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// The UART interface is primarily used for the RailCom Detector that sends a 
// serial signal. So far, only the receiver portion is implemented because that
// is all what is needed for RailCom messages. There are two general categories. 
// The first uses the PICO built-in UART hardware blocks. The second implements a
// software UART based on the PICO PIO blocks.
//
// There are three routines. The "startUartRead" will enable the UART and start 
// reading bytes into the local buffer. The "stopUartRead" will then finish the 
// byte collection and disable the UART again. Finally, the "getUartBuffer" 
// routine will return the bytes received. Again, note that this is not a generic
// UART read interface.
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
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
// handlers, which in turn will invoke the respective callback function. 
//
// The UART handlers will handle receive interrupts of the UART hardware blocks. 
// There is no easy way to get to the resource structure where the input buffer is.
// We therefore maintain two global variables in this file to store the configured 
// resource for each UART HW block.
// 
//----------------------------------------------------------------------------------------
CdcResource *uartRes0 = nullptr;
CdcResource *uartRes1 = nullptr;

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

    extern bool validPin( uint8_t pin, uint32_t mask );

    extern CdcResource *lookupResource( uint8_t rNum, uint8_t type );
    extern CdcResource *allocateResourceType( uint8_t rNum, uint8_t type );
}

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// Configure a UART resource. 
//
//----------------------------------------------------------------------------------------
uint8_t configureUart( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_UART );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureUart( rNum, 
                            dPtr -> uart.rxPin, 
                            dPtr -> uart.txPin, 
                            dPtr -> uart.baudRate ));
}

//----------------------------------------------------------------------------------------
// Configure the UART resource.
//
//----------------------------------------------------------------------------------------
uint8_t configureUart( uint8_t rNum, 
                       uint8_t rxPin, 
                       uint8_t txPin, 
                       uint32_t baudRate ) {
   
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

} // namespace CDC
