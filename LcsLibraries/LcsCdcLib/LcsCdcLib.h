//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file
//
//------------------------------------------------------------------------------------------------------------
// The controller dependent code layer concentrates all processor dependent code into one library. The idea
// is twofold. First, there needs to be a way to isolate the controller specific hardware from the LCS runtime
// Library as well as the extension module firmware. The Raspberry PI Pico offers a C++ SDK with a set of
// libraries to invoke the desired function rather than access to registers.The Pico also offers a great
// flexibility of pin assignment for the hardware IO functions. Second, within the hardware IO boundaries of
// the controller family the individual hardware pin assignment used may vary from board to board design.
// Nevertheless, the Extension Connector layout and basic functions available should be the same for all
// controllers used. For the upper software layers, the CDC library offers a structured way to describe
// the possible pins assignments.
//
// Note that this layer is not a generic HW abstraction. The layer is very specific to the LCS controller
// boards described in the book. Nevertheless, some pins can vary, depending on the board version. Currently,
// only the Raspberry PI Pico Board is supported.
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
// ??? clean up a little ... what is really needed ?
//------------------------------------------------------------------------------------------------------------
enum CdcStatus : uint8_t {

    NO_ERR              = 0,
    INIT_PENDING        = 1,
    NOT_SUPPORTED       = 2,
    NOT_IMPLEMENTED     = 3,

    MEM_SIZE_ERR        = 10,

    INVALID_HANDLE_ERR  = 11,
    MAX_RES_ID_ERR      = 12,


    ACTIVE_LED_PIN_ERR  = 13,
    BUTTON_PIN_ERR      = 14,
    PFAIL_PIN_ERR       = 15,
    EXT_INT_PIN_ERR     = 16,
    DIO_PIN_ERR         = 17,
    ADC_PIN_ERR         = 18,
    PWM_PIN_ERR         = 19,

    UART_PORT_ERR       = 20,
    UART_CONFIG_ERR     = 21,
    UART_WRITE_ERR      = 22,
    UAT_READ_ERR        = 23,

    SPI_PORT_ERR        = 25,
    SPI_CONFIG_ERR      = 26,
    SPI_WRITE_ERR       = 27,
    SPI_READ_ERR        = 28,
    SPI_NOT_ACTIVE_ERR  = 29,

    I2C_PORT_ERR        = 30,
    I2C_CONFIG_ERR      = 31,
    I2C_WRITE_ERR       = 32,
    I2C_READ_ERR        = 33

};

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
const uint16_t  MAX_INST_DESC_ENTRIES = 64;

//------------------------------------------------------------------------------------------------------------
// Controller pin related definitions. A pin can be valid, undefined or illegal. An undefined pin for a pin
// field in the configuration structure indicates that  the pin has not been used by the firmware
// implementation but is a pin that the particular controller would support. An illegal pin means that the
// pin is not offered by this controller and cannot be assigned at all.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t UNDEFINED_PIN     = 255;
const uint8_t ILLEGAL_PIN       = 254;

//------------------------------------------------------------------------------------------------------------
// The CDC instances have a type which tells us what the particular instance is.
//
//------------------------------------------------------------------------------------------------------------
enum CdcInstanceType : uint16_t {

    CDC_IT_UNDEFINED    = 0,
    CDC_IT_CONTROLLER   = 1,
    CDC_IT_WATCHDOG     = 2,
    CDC_IT_TIMER        = 3,
    CDC_IT_GPIO         = 4,
    CDC_IT_ADC          = 5,
    CDC_IT_PWM          = 6,
    CDC_IT_UART         = 7,
    CDC_IT_I2C          = 8,
    CDC_IT_SPI          = 9,
    CDC_IT_CAN_BUS      = 10
};

//------------------------------------------------------------------------------------------------------------
// The controller families. Currently, there is only the Raspberry PI Pico models.
//
//------------------------------------------------------------------------------------------------------------
enum ControllerFamily : uint8_t {

    CDC_CF_UNDEFINED    = 0,
    CDC_CF_RP_PICO_2040 = 1,
    CDC_CF_RP_PICO_2350 = 2
};

//------------------------------------------------------------------------------------------------------------
// DIO pin related definitions. A digital pin can be an input pin, with or without pull-up, or an output 
// pin. DIO pins can also be associated with an interrupt handler. The handler itself is mapped to an edge
// or level event.
//
//------------------------------------------------------------------------------------------------------------
enum dioMode : uint8_t {

    DIO_IN              = 0,
    DIO_OUT             = 1,
    DIO_IN_PULLUP       = 2
};

//------------------------------------------------------------------------------------------------------------
// GPIO interrupts are detected as level change or edge changes.
//
//------------------------------------------------------------------------------------------------------------
enum intEventTyp : uint8_t {

    EVT_NONE            = 0,
    EVT_LOW             = 1,
    EVT_HIGH            = 2,
    EVT_FALL            = 3,
    EVT_RISE            = 4,
    EVT_CHANGE          = 5
};

//------------------------------------------------------------------------------------------------------------
// The UART modes. There are two implementations. The PICO offers two hardware UARTS. We use them with 8 
// bits with a parity bit. The second type UART is a software implementation based on the PICO PIO blocks.
//
//------------------------------------------------------------------------------------------------------------
enum UartMode : uint8_t {

    UART_MODE_UNDEFINED = 0,
    UART_MODE_8N1       = 1,
    UART_MODE_8N1_PIO   = 2
};

//------------------------------------------------------------------------------------------------------------
// PWM duty cycle.
//
//------------------------------------------------------------------------------------------------------------
enum PwmDutyCycle : uint8_t {

    MIN_DUTY_CCYCLE     = 0,
    MAX_DUTY_CYCLE      = 255
};

//------------------------------------------------------------------------------------------------------------
// The controller instance type. The controller itself has parameters we can set.
//
// ??? letÄs see where this takes us...
//------------------------------------------------------------------------------------------------------------
struct ControllerDesc {

    ControllerFamily    cFamily         = CDC_CF_UNDEFINED;
    uint32_t            memorySize      = 0;
    uint32_t            internalNvmSize = 0;
};

//------------------------------------------------------------------------------------------------------------
// The controller features a watchdog facility. The idea is that when the controller software hangs, it will
// be restarted. To avoid the automatic restarting, the software periodically needs to reset the watchdog 
// timer. The interval timer specifies the maximum time without resetting the watchdog timer.
//
//------------------------------------------------------------------------------------------------------------
struct WatchDogInstDesc {

    uint32_t    intervalMillis = 0;
};

//------------------------------------------------------------------------------------------------------------
// Timer instances descriptor.
// 
// ??? option to specify whether the timer should restart while the interrupt is served or after.
//------------------------------------------------------------------------------------------------------------
struct TimerInstDesc {

    uint32_t    intervalMillis = 0;
};

//------------------------------------------------------------------------------------------------------------
// A GPIO instance descriptor declares the pin(s) and their mode. Note that the mode can be changed later via
// the config routine.
//
// ??? should we also have a concept to have a pin mask for multiple pins ?
//------------------------------------------------------------------------------------------------------------
struct GpioInstDesc {

    uint8_t     pinA            = UNDEFINED_PIN;
    uint8_t     pinB            = UNDEFINED_PIN;
    dioMode     pinAMode        = DIO_IN;
    dioMode     pinBMode        = DIO_IN;
};

//------------------------------------------------------------------------------------------------------------
// The ADC instance descriptor declares analog input.
//
//------------------------------------------------------------------------------------------------------------
struct AdcInstDesc {

    uint8_t     adcPin          = CDC::UNDEFINED_PIN;
};

//------------------------------------------------------------------------------------------------------------
// The PWM instance declares the pins(s) and the PWM options, such as frequency.
//
// ??? range to specify ? or always 0 .. 255 ?
//------------------------------------------------------------------------------------------------------------
struct PwmInstDesc {

    uint8_t     pwmPinA         = CDC::UNDEFINED_PIN;
    uint8_t     pwmPinB         = CDC::UNDEFINED_PIN;
    uint32_t    pwmFreqency     = 0;
    uint32_t    wrap            = 0;
    bool        inverted        = false;
};

//------------------------------------------------------------------------------------------------------------
// The UART instance descriptor declares a serial IO interface. We need the Rx and Tx pins and the UART
// options.
//
//------------------------------------------------------------------------------------------------------------
struct UartInstDesc {

    uint8_t     rxPin           = CDC::UNDEFINED_PIN;
    uint8_t     txPin           = CDC::UNDEFINED_PIN;
    uint32_t    baudRate        = 0;
    uint8_t     dataBits        = 8;
    uint8_t     parityMode      = 0;
    uint8_t     stopBits        = 1;
};

//------------------------------------------------------------------------------------------------------------
// The I2C instance descriptor declares an I2C channel.
// 
//------------------------------------------------------------------------------------------------------------
struct I2CInstDesc {

    uint8_t     sclPin          = CDC::UNDEFINED_PIN;
    uint8_t     sdaPin          = CDC::UNDEFINED_PIN;
    uint32_t    baudRate        = 0;
    uint32_t    timeoutValMs    = 0;
};

//------------------------------------------------------------------------------------------------------------
// The SPI instance descriptor declares an SPI channel.
//
//------------------------------------------------------------------------------------------------------------
struct SPIInstDesc {

    uint8_t     selectPin       = CDC::UNDEFINED_PIN;
    uint8_t     mosiPin         = CDC::UNDEFINED_PIN;
    uint8_t     misoPin         = CDC::UNDEFINED_PIN;
    uint8_t     sclkPin         = CDC::UNDEFINED_PIN;
    uint32_t    frequency       = 0;
};

//------------------------------------------------------------------------------------------------------------
// The CAN Bus instance descriptor declares an the necessary CAN bus data.
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusInstDesc {

    uint8_t     canPin1         = CDC::UNDEFINED_PIN;
    uint8_t     canPin2         = CDC::UNDEFINED_PIN;
    uint32_t    baudRate        = 0;
    
    // ??? options for can2040 specifics ?
    
};

//------------------------------------------------------------------------------------------------------------
// A CDC configuration descriptor is the counterpart to the instances. A descriptor for a given instance type
// will contain all the information to configure that instance. An instance can be configured with the 
// configure routine and its parameter list or based on the data in this descriptor. It should be possible to
// configure the entire board based on an array of such descriptors. The idea is that each board can be 
// uniquely described with such an array.
//
//------------------------------------------------------------------------------------------------------------
struct CdcInstanceConfigDesc {

    CdcInstanceType type;

    union {

        ControllerDesc      ctl;
        WatchDogInstDesc    wd;
        TimerInstDesc       timer;
        GpioInstDesc        gpio;
        PwmInstDesc         pwm;
        UartInstDesc        uart;
        AdcInstDesc         adc;
        I2CInstDesc         i2c;
        SPIInstDesc         spi;
        CanBusInstDesc      can;
    };
};

//------------------------------------------------------------------------------------------------------------
// The CDC instance map is the data structure that has an entry for each declared instance.
//
//------------------------------------------------------------------------------------------------------------
struct CdcInstanceDescMap {

    uint16_t flags;
    uint16_t size;
    
    // ??? boardId ?

    CdcInstanceConfigDesc map[ MAX_INST_DESC_ENTRIES ];
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
// Each resource has a unique ID. The ID is used in the configuration routines to create and locate the 
// particular entry. The IDs are used by the upper layers to obtain the handle that was created when the 
// resource was configured. From thereon, the handle is the key argument to pass for a configured resource.
//
//------------------------------------------------------------------------------------------------------------
enum CdcResIdNames {

    CDC_RID_UNDEFINED   = 0,
    CDC_RID_WATCHDOG    = 1,
    CDC_RID_CONTROLLER  = 2,
    CDC_RID_TIMER_0     = 3,

    CDC_RID_DIO_0       = 10,
    CDC_RID_DIO_1       = 11,
    CDC_RID_DIO_2       = 12,
    CDC_RID_DIO_3       = 13,
    CDC_RID_DIO_4       = 14,
    CDC_RID_DIO_5       = 15,
    CDC_RID_DIO_6       = 16,
    CDC_RID_DIO_7       = 17,
    CDC_RID_DIO_8       = 18,
    CDC_RID_DIO_9       = 19,
    CDC_RID_DIO_10      = 20,
    CDC_RID_DIO_11      = 21,
    CDC_RID_DIO_12      = 22,
    CDC_RID_DIO_13      = 23,
    CDC_RID_DIO_14      = 24,
    CDC_RID_DIO_16      = 25,

    CDC_RID_ADC_0       = 30,
    CDC_RID_ADC_1       = 31,
    CDC_RID_ADC_2       = 32,
    CDC_RID_ADC_3       = 33,

    CDC_RID_PWM_0       = 40,
    CDC_RID_PWM_1       = 41,
    CDC_RID_PWM_2       = 42,
    CDC_RID_PWM_3       = 43,
    CDC_RID_PWM_4       = 44,
    CDC_RID_PWM_5       = 45,
    CDC_RID_PWM_6       = 46,
    CDC_RID_PWM_7       = 47,
    CDC_RID_PWM_8       = 48,
    CDC_RID_PWM_9       = 49,
    CDC_RID_PWM_10      = 50,
    CDC_RID_PWM_11      = 51,
    CDC_RID_PWM_12      = 52,
    CDC_RID_PWM_13      = 53,
    CDC_RID_PWM_14      = 54,
    CDC_RID_PWM_16      = 55,

    CDC_RID_UART_0      = 60,
    CDC_RID_UART_1      = 61,
    CDC_RID_UART_2      = 62,
    CDC_RID_UART_3      = 63,
   
    CDC_RID_SPI_0       = 65, 
    CDC_RID_SPI_1       = 66,

    CDC_RID_I2C_NVM     = 70,
    CDC_RID_I2C_EXT     = 71,

    CDC_RID_CAN_BUS     = 75,

};

// ??? this will go away...
//------------------------------------------------------------------------------------------------------------
// CDC features a data structure that records all HW specific pins and flags. The values are set by the
// initialization code in a project and are validated. All modules in a project will then just use the
// data structure fields using the data for calls to the Hal layer. For example, an application that
// uses DIO_PIN_0 and DIO_PIN_1 will set the HW pin numbers of the controller / board combination used
// in a config data structure "cfg". A call to write a value to the DIO pin, will then just use
// "cfg.DIO_PIN_1" as argument in the "writeDio" call. The "writeDio" call itself will not check the
// value of the configured DIO pin, all it will do is to ensure that it is not UNDEFINED. Note that the
// structure has more pins defined that a potential controller may have. If so, these fields are set to
// UNDEFINED. The structure is the superset of all possible HW items to configure.
//
// In a later runtime version, we may put this structure as constant data into the non-volatile chip on
// the board. After all the HW pin assignments is linked to the particular board. It will then just be read 
// from the board NVM.
//
// ??? think about a way to just have an array of pin numbers with a pointer to the instance. A constant
// labels the pin or resource behind it. The array would also need to have a pointer to the instance it
// belongs to. When an instance needs two pins, like i2c, the array fields point to the same entry.
//------------------------------------------------------------------------------------------------------------
struct CdcConfigDesc {

    uint8_t   CFG_STATUS;

    uint8_t   PFAIL_PIN;
    uint8_t   EXT_INT_PIN;
    uint8_t   ACTIVE_LED_PIN;

    uint8_t   DIO_PIN_0;
    uint8_t   DIO_PIN_1;
    uint8_t   DIO_PIN_2;
    uint8_t   DIO_PIN_3;
    uint8_t   DIO_PIN_4;
    uint8_t   DIO_PIN_5;
    uint8_t   DIO_PIN_6;
    uint8_t   DIO_PIN_7;
    uint8_t   DIO_PIN_8;
    uint8_t   DIO_PIN_9;
    uint8_t   DIO_PIN_10;
    uint8_t   DIO_PIN_11;
    uint8_t   DIO_PIN_12;
    uint8_t   DIO_PIN_13;
    uint8_t   DIO_PIN_14;
    uint8_t   DIO_PIN_15;

    uint8_t   ADC_PIN_0;
    uint8_t   ADC_PIN_1;
    uint8_t   ADC_PIN_2;
    uint8_t   ADC_PIN_3;

    uint8_t   PWM_PIN_0;
    uint8_t   PWM_PIN_1;
    uint8_t   PWM_PIN_2;
    uint8_t   PWM_PIN_3;
    uint8_t   PWM_PIN_4;
    uint8_t   PWM_PIN_5;
    uint8_t   PWM_PIN_6;
    uint8_t   PWM_PIN_7;
    uint8_t   PWM_PIN_8;
    uint8_t   PWM_PIN_9;
    uint8_t   PWM_PIN_10;
    uint8_t   PWM_PIN_11;
    uint8_t   PWM_PIN_12;
    uint8_t   PWM_PIN_13;
    uint8_t   PWM_PIN_14;
    uint8_t   PWM_PIN_15;

    uint8_t   UART_RX_PIN_0;
    uint8_t   UART_TX_PIN_0;

    uint8_t   UART_RX_PIN_1;
    uint8_t   UART_TX_PIN_1;

    uint8_t   UART_RX_PIN_2;
    uint8_t   UART_TX_PIN_2;

    uint8_t   UART_RX_PIN_3;
    uint8_t   UART_TX_PIN_3;

    uint8_t   SPI_MOSI_PIN_0;
    uint8_t   SPI_MISO_PIN_0;
    uint8_t   SPI_SCLK_PIN_0;

    uint8_t   SPI_MOSI_PIN_1;
    uint8_t   SPI_MISO_PIN_1;
    uint8_t   SPI_SCLK_PIN_1;

    uint8_t   NVM_I2C_SCL_PIN;
    uint8_t   NVM_I2C_SDA_PIN;
    uint8_t   NVM_I2C_ADR_ROOT;

    uint8_t   EXT_I2C_SCL_PIN;
    uint8_t   EXT_I2C_SDA_PIN;
    uint8_t   EXT_I2C_ADR_ROOT;

    // ??? still uneasy whether NVM sizes should be here...
    uint32_t  NODE_NVM_SIZE;
    uint32_t  EXT_NVM_SIZE;

    // ??? should control mode and default ID be here ? Pins we need of course...
    uint8_t   CAN_BUS_CTRL_MODE;
    uint32_t  CAN_BUS_DEF_ID;
    uint8_t   CAN_BUS_RX_PIN;
    uint8_t   CAN_BUS_TX_PIN;
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
uint8_t         init( CdcConfigDesc *ci );
void            fatalError( uint8_t n );
void            fatalErrorMsg( char *str, uint8_t n, uint8_t rStat );
void            setDebugLevel( uint8_t level = 0 );

//------------------------------------------------------------------------------------------------------------
// General utility routines.
//
//------------------------------------------------------------------------------------------------------------
uint32_t        getMillis( );
uint32_t        getMicros( );
void            sleepMillis( uint32_t val );
void            sleepMicros( uint32_t val );
uint32_t        createUid( );

//------------------------------------------------------------------------------------------------------------
// The console IO functions. We will provide a serial IO via the USB connector of the PICO. The files 
// need to be linked with the "tinyUSB" library and the cmake file needs to set the option. Then we can
// use scanf and printf and so on. In addition, we need  function  that just attempts to read a character
// and returns immediately when there is none.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureConsoleIO( );
bool            isConsoleConnected( );
char            getConsoleChar( uint32_t timeoutVal = 0 );

//------------------------------------------------------------------------------------------------------------
// CDC high level setup and configuration routines. In addition to setting up the individual resources, the
// particular mappings for a LCS Nodes board need to be provided. This is where the "CdcInstanceDescMap"
// will be used. It is basically an array of resource descriptors and each board has its unique descriptor.
// There is an include file which contains the descriptor for each board type and board version designed.
// Nevertheless, an given resource configuration routine can be called any time, to either overwrite or 
// replace the use of the descriptor information in the board descriptor array.
// 
//------------------------------------------------------------------------------------------------------------
uint8_t         configureCdcSubSytem( CdcInstanceDescMap *map );
void            printCdcSubSystemInfo( CdcInstanceDescMap *map );

// ??? individual print routines for each type ?

//------------------------------------------------------------------------------------------------------------
// General controller info routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureController( uint8_t *handle );
uint8_t         getFamily( uint8_t handle, ControllerFamily *family );
uint8_t         getVersion( uint8_t handle, uint32_t *version );
uint8_t         getChipMemSize( uint8_t handle, uint32_t *size );
uint8_t         getChipNvmSize( uint8_t handle, uint32_t *size );
uint8_t         getCpuFrequency( uint8_t handle, uint32_t *frequency );

//------------------------------------------------------------------------------------------------------------
// The watchdog facility. There are routines to configure the time interval, enabling and disabling the 
// watchdog timer, updating it periodically, as well as detecting that we came from a watchdog restart 
// when restarting.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureWatchDog( uint8_t *handle, uint32_t millis );
uint8_t         watchDogEnable( uint8_t handle, bool enable );
uint8_t         watchDogUpdate( uint8_t handle );
uint8_t         watchDogCausedReboot( uint8_t handle, bool *reboot );

//------------------------------------------------------------------------------------------------------------
// Timer management routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureTimer( uint8_t *handle, TimerCallback functionId );
uint8_t         startRepeatingTimer( uint8_t handle, uint32_t val );
uint8_t         setRepeatingTimerLimit( uint8_t handle, uint32_t val );
uint32_t        getRepeatingTimerLimit( uint8_t handle );
void            stopRepeatingTimer( uint8_t handle );

// phase out...
void            onTimerEvent( TimerCallback functionId );
void            startRepeatingTimer( uint32_t val );
void            setRepeatingTimerLimit( uint32_t val );
uint32_t        getRepeatingTimerLimit( );
void            stopRepeatingTimer( );

//------------------------------------------------------------------------------------------------------------
// Analog input routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureAdc( uint8_t adcPin );
uint16_t        getAdcRefVoltage( );
uint16_t        getAdcDigitRange( );
uint16_t        readAdc( uint8_t adcPin );

//------------------------------------------------------------------------------------------------------------
// Digital Input/Output routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureDio( uint8_t dioPin, uint8_t Mode = DIO_IN );
void            registerDioCallback( uint8_t dioPin, uint8_t event, CDC::GpioCallback func );
void            unregisterDioCallback( uint8_t dioPin );
bool            readDio( uint8_t dioPin );
uint8_t         writeDio( uint8_t dioPin, bool val );
uint8_t         toggleDio( uint8_t dioPin );
uint32_t        readDioMask( uint32_t dioMask );
uint8_t         writeDioMask( uint32_t dioMask, uint32_t dioVal );
uint8_t         writeDioPair( uint8_t dioPin1, bool val1, uint8_t dioPin2, bool val2 );

//------------------------------------------------------------------------------------------------------------
// PWM output routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configurePwm(   uint8_t   pwmPinA,
                                uint8_t   pwmPinB,
                                uint32_t  pwmFreqency,
                                bool      phaseCorrect  = true,
                                bool      inverted      = false
                            );

uint8_t         writePwm( uint8_t pwmPin, uint8_t dutyCycleA, uint8_t dutyCycleB );

//------------------------------------------------------------------------------------------------------------
// Serial IO routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureUart( uint8_t rxPin, uint8_t txPin, uint32_t baudRate, UartMode mode );
uint8_t         startUartRead( uint8_t rxPin );
uint8_t         stopUartRead( uint8_t rxPin );
uint8_t         getUartBuffer( uint8_t rxPin, uint8_t *buf, uint8_t bufLen );

//------------------------------------------------------------------------------------------------------------
// I2C management routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureI2C( uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate = 100 * 1000 );
uint8_t         i2cBusreset( uint8_t sclPin );
uint8_t         i2cWrite( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit = false );
uint8_t         i2cRead( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint16_t len, bool stopBit = false );

//------------------------------------------------------------------------------------------------------------
// SPI management routines.
//
//------------------------------------------------------------------------------------------------------------
uint8_t         configureSPI( uint8_t sclkPin, uint8_t mosiPin, uint8_t misoPin, uint32_t baudRate = 10 * 1000 * 1000 );
uint8_t         spiBeginTransaction( uint8_t sclkPin, uint8_t csPin );
uint8_t         spiEndTransaction( uint8_t sclkPin, uint8_t csPin );
uint8_t         spiRead( uint8_t sclkPin, uint8_t *buf, uint32_t len );
uint8_t         spiWrite( uint8_t sclkPin, uint8_t *buf, uint32_t len );




// ??? to phase out ....

void            printConfigInfo( CdcConfigDesc *ci );               // goes away...
CdcConfigDesc   getConfigDefault( );                                // goes away
CdcConfigDesc   *getConfigActual( );                                // goes away




};

#endif
