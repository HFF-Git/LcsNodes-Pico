//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file
//
//------------------------------------------------------------------------------------------------------------
//
// ??? explain the interlocking between board descriptors and this lib.
//
// The controller dependent code layer concentrates all processor dependent code into one library. The idea
// is twofold. First, there needs to be a way to isolate the controller specific hardware from the LCS runtime
// Library as well as the extension module firmware. The Raspberry PI Pico offers a C++ SDK with a set of
// libraries to invoke the desired function rather than access to registers. The Pico also offers a great
// flexibility of pin assignment for the hardware IO functions. Second, within the hardware IO boundaries of
// the controller family the individual hardware pin assignment used may vary from board to board design.
// This include file and the board descriptor include file implement the CDC layer from a hardware function 
// and board configuration perspective. Note that the CDC layer is not a generic HW abstraction. The layer
// is very specific to the LCS controller boards described in the book. 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file
// Copyright (C) 2022 - 2025  Helmut Fieres
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
#ifndef LcsCdcLib_h
#define LcsCdcLib_h

//------------------------------------------------------------------------------------------------------------
// Include files.
//
//------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include "LcsBoardDescriptors.h"
#include "LcsCdcLibVersion.h"

//------------------------------------------------------------------------------------------------------------
// All definitions and functions are in the CDC name space.
//
//------------------------------------------------------------------------------------------------------------
namespace CDC {

//------------------------------------------------------------------------------------------------------------
// The debug mask. The library has a debug mask where each major part of the library has a flag. There could 
// also be flags reserved for the firmware. There is an ITEM to read and set this mask. Wherever debugging is
// needed, the bit mask will be used to determine whether to print debugging data or not. From a performance 
// perspective, the test will take just a few instructions. In other words we do not take out debugging code 
// when going into production. Never liked this approach of conditional debug anyway.
//
// The usage of the debug mask is generally: 
//
//      if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_xxx )) ....
// 
// The DBG_CONFIG bit allows for the entire debugging messages to be enabled or disabled. This feature will 
// also be used when we test whether we even have a console or not. If there is no console, all the prints
// will not be executed.
//
//------------------------------------------------------------------------------------------------------------
enum DebugOtions : uint16_t {

    DBG_CONFIG      = ( 1U << 15 ),
    DBG_SETUP       = ( 1U << 0 ),
    DBG_I2C         = ( 1U << 1 ),
    DBG_SPI         = ( 1U << 2 ),
    DBG_PWM         = ( 1U << 3 ),
    DBG_UART        = ( 1U << 4 ),
    DBG_GPIO        = ( 1U << 5 )
};

//------------------------------------------------------------------------------------------------------------
// Error status codes. The errors are used when setting up the Hal library. During operation, all routines
// validate the input for correctness. If they are not correct, the call is simply not performed and an
// error is returned.
//
//------------------------------------------------------------------------------------------------------------
enum CdcStatus : uint8_t {

    NO_ERR              = 0,

    NOT_SUPPORTED_ERR   = 1,
    NOT_IMPLEMENTED_ERR = 2,
    RES_ID_ALLOCATE_ERR = 3,
    INVALID_RES_ID_ERR  = 4,

    DIO_PIN_ERR         = 10,
    ADC_PIN_ERR         = 11,
    PWM_PIN_ERR         = 12,
    UART_PIN_ERR        = 13,
    I2C_PIN_ERR         = 14,

    DIO_MODE_ERR        = 20,
    DIO_INT_HANDLER_ERR = 21,

    UART_PORT_ERR       = 30,
    UART_CONFIG_ERR     = 31,
    UART_WRITE_ERR      = 32,
    UAT_READ_ERR        = 33,

    I2C_PORT_ERR        = 40,
    I2C_CONFIG_ERR      = 41,
    I2C_WRITE_ERR       = 42,
    I2C_READ_ERR        = 43
};

//------------------------------------------------------------------------------------------------------------
// Callback functions signatures.
//
//------------------------------------------------------------------------------------------------------------
extern "C" {

    typedef void ( *TimerCallback ) ( uint32_t timerVal );
    typedef void ( *GpioCallback ) ( uint8_t pin, uint8_t event );
}

//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
enum CdcResIdNumbers : uint8_t {

    // ??? what can we predefine as resource numbers ? Should we even do this ?
};

//------------------------------------------------------------------------------------------------------------
// The routines that make up the hardware abstraction layer. In general, there are routines that are just 
// basic utility routines common to all controller implementations. The majority of routines provide the 
// interface to the controller resources. Each resource type has a name and a set of routines for accessing
// it. The idea is that at startup, the resources are configured and a handle is provided to the upper layer.
// 
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// Basic init and error handling.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     cdcInit( );
uint8_t     getVersion( uint32_t *version );
void        fatalError( uint8_t errNum );
void        fatalErrorMsg( char *str, uint8_t errNum, uint8_t rStat );
void        setDebugLevel( uint8_t level = 0 );

//------------------------------------------------------------------------------------------------------------
// General utility routines.
//
//------------------------------------------------------------------------------------------------------------
uint32_t    getMillis( );
uint32_t    getMicros( );
void        sleepMillis( uint32_t val );
void        sleepMicros( uint32_t val );
uint32_t    createUid( );

//------------------------------------------------------------------------------------------------------------
// The console IO functions. We will provide a serial IO via the USB connector of the PICO. The files 
// need to be linked with the "tinyUSB" library and the cmake file needs to set the option. Then we can
// use scanf and printf and so on. In addition, we need  function  that just attempts to read a character
// and returns immediately when there is none.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureConsoleIO( );
bool        isConsoleConnected( );
char        getConsoleChar( uint32_t timeoutVal = 0 );

//------------------------------------------------------------------------------------------------------------
// CDC high level setup and configuration routines. In addition to setting up the individual resources, the
// particular mappings for a LCS Nodes board need to be provided. This is where the "CdcInstanceDescMap"
// will be used. It is basically an array of resource descriptors and each board has its unique descriptor.
// There is an include file which contains the descriptor for each board type and board version designed.
// Nevertheless, an given resource configuration routine can be called any time, to either overwrite or 
// replace the use of the descriptor information in the board descriptor array.
// 
//------------------------------------------------------------------------------------------------------------
uint8_t     configureCdcSubSytem( CdcResourceDesc *map );
void        printCdcSubSystemInfo( CdcResourceDesc *map );

//------------------------------------------------------------------------------------------------------------
// General controller info routines.
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureController(    ControllerFamily    family, 
                                    ControllerChip      processor,
                                    uint32_t            memorySize,
                                    uint32_t            internalNvmSize,
                                    uint32_t            watchDogMillis,
                                    uint16_t            adcRefVoltage, 
                                    uint16_t            adcDigitRange, 
                                    uint8_t             ledPin,
                                    uint8_t             pFailPin );

uint8_t     getFamily( ControllerFamily *family );
uint8_t     getControllerChip( ControllerChip *chip );
uint8_t     getChipMemSize( uint32_t *size );
uint8_t     getChipNvmSize( uint32_t *size );
uint8_t     getCpuFrequency( uint32_t *frequency );

uint8_t     watchDogEnable( bool enable );
uint8_t     watchDogUpdate( );
uint8_t     watchDogCausedReboot( bool *reboot );


//------------------------------------------------------------------------------------------------------------
// Timer management routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureTimer( uint8_t resId, TimerCallback functionId );
uint8_t     startRepeatingTimer( uint8_t resId, uint32_t val );
uint8_t     setRepeatingTimerLimit( uint8_t resId, uint32_t val );
uint8_t     getRepeatingTimerLimit( uint8_t resId, uint32_t *val );
uint8_t     stopRepeatingTimer( uint8_t resId );

//------------------------------------------------------------------------------------------------------------
// Analog input routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureAdc( uint8_t resId, uint8_t adcPin );
uint8_t     readAdc( uint8_t resId, uint16_t *val );

//------------------------------------------------------------------------------------------------------------
// Digital Input/Output routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureDio(   uint8_t     resId,
                            uint8_t     pinA, 
                            uint8_t     pinB, 
                            uint8_t     pinMode );

uint8_t     registerDioCallback( uint8_t resId, uint8_t event, GpioCallback func );
uint8_t     unregisterDioCallback( uint8_t resId );
uint8_t     readDio( uint8_t resId, bool *val );
uint8_t     writeDio( uint8_t resId, bool val );
uint8_t     writeDio( uint8_t resId, bool valA, bool valB );
uint8_t     toggleDio( uint8_t resId );

//------------------------------------------------------------------------------------------------------------
// PWM output routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configurePwm(   uint8_t     resId,
                            uint8_t     pinA,
                            uint8_t     pinB,
                            uint32_t    freqency,
                            bool        phaseCorrect  = true,
                            bool        inverted      = false
                        );

uint8_t     writePwm( uint8_t resId, uint8_t dutyCycleA, uint8_t dutyCycleB );
uint8_t     syncPwm( uint8_t resId );

//------------------------------------------------------------------------------------------------------------
// Serial IO routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureUart( uint8_t resId, uint8_t rxPin, uint8_t txPin, uint32_t baudRate );
uint8_t     startUartRead( uint8_t resId );
uint8_t     stopUartRead( uint8_t resId );
uint8_t     getUartBuffer( uint8_t resId, uint8_t *buf, uint8_t bufLen );

//------------------------------------------------------------------------------------------------------------
// I2C management routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureI2C( uint8_t resId, uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate = 100 * 1000 );
uint8_t     i2cBusreset( uint8_t resId );
uint8_t     i2cWrite( uint8_t resId, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit = false );
uint8_t     i2cRead( uint8_t resId, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit = false );

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t     configureCanBus( uint8_t resId, uint8_t pinH, uint8_t pinL, uint32_t baudRate );

};

#endif
