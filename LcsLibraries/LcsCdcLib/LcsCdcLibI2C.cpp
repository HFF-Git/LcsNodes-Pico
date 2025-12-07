//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// The PICO has two HW blocks for I2C interfaces. The CDC interface implements a 
// simple read and write access to an I2C element. There is a timeout to avoid 
// waiting forever on an operation. Finally,we have routines to get the pins and
// baud rate. This is primarily used by external libraries.
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


}

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
}

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// Configure the I2C channel.
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

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "configureI2C: rNum: %d, scl: %d, sda: %d, baud: %d, tVal: %d\n",
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

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "i2cRead: rNum: %d, i2c: 0x%x, buf: %p, "
                "buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                rNum, i2cAdr, buf, buf[0], buf[1], len, stopBit );

        if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_I2C )) {
            
            if ( ret == PICO_ERROR_GENERIC ) { 
                
                printf( "I2C read, PICO generic error\n" );
            }
            else if ( ret == PICO_ERROR_TIMEOUT ) { 
                
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

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_I2C )) {
        
        printf( "i2cWrite: rNum: %d, i2cAdr: 0x%x, buf: %p, "
                "buf[0] %x, buf[1] %x, len: %d, stop: %d\n", 
                rNum, i2cAdr, buf, buf[0], buf[1], len, stopBit );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );
 
    int ret = 0;
    for ( int i = 0; i < 10; ++i ) {

        ret = i2c_write_blocking_until( rPtr->i2c.i2cHw, 
                        i2cAdr, 
                        buf, 
                        len, 
                        stopBit, 
                        make_timeout_time_ms( rPtr -> i2c.timeoutValMs ));
        if ( ret >= 0 ) break;

        sleep_ms(1);
    }

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_PWM )) {

        if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_I2C )) {

            if ( ret == PICO_ERROR_GENERIC ){
                
                printf( "I2C write, PICO generic error\n" );
            }
            else if ( ret == PICO_ERROR_TIMEOUT ) { 
                
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

    if (( debugMask & CDC_DBG_ENABLE ) && ( debugMask & CDC_DBG_I2C )) {

        printf( "I2C Bus reset, rNum: %d\n", rNum );
    }

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    uint8_t reset_cmd = 0x06;
    i2c_write_blocking( rPtr -> i2c.i2cHw, 0x00, &reset_cmd, 1, false); 
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// I2C helper routines. Although we work with resource numbers, sometimes we need 
// to pass the physical pin numbers and the baud rate to an external library.
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

} // namespace CDC
