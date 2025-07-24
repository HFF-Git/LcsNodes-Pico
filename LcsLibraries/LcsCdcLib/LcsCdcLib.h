//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// The controller dependent code layer concentrates all processor dependent code
// into one library. The idea is twofold. First, there needs to be a way to
// isolate the controller specific hardware from the LCS library as well as the
// extension module firmware. The Raspberry PI Pico offers a C++ SDK with a set
// of libraries to invoke the desired hardware function rather than access to 
// registers. The Pico also offers a great flexibility of pin assignment for the
// hardware IO functions. Mapping the pins to functions is the first key goal. 
// Second, within the hardware IO boundaries of the controller family the 
// individual hardware pin assignment used may vary from board to board design.
// The goal is also to describe a board by type and version.
//
// The second idea is to provide the firmware designer a set of resources based
// on the board capabilities. A CDC resource encapsulates a certain HW function. 
// Resources are described via a resource descriptor. A particular board is 
// described through a data structure which contains all the resources the 
// Board offers.
// 
// This include file implements the CDC layer from a hardware function and board
// configuration perspective.  Note that the CDC layer is not a generic HW 
// abstraction. The layer is very specific to the LCS controller board 
// requirements described in the book. 
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
#ifndef LcsCdcLib_h
#define LcsCdcLib_h

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <cstring>

//----------------------------------------------------------------------------------------
// All definitions and functions are in the CDC name space.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// The debug mask. The library has a debug mask where each major part of the 
// library has a flag. Wherever debugging is needed, the bit mask will be used
// to determine whether to print debugging data or not. From a performance 
// perspective, the test will take just a few instructions. In other words we 
// do not take out debugging code when going into production. Never liked this
// kind of conditional debug anyway.
//
// The usage of the debug mask is generally: 
//
//      if (( debugMask & DBG_CONFIG ) && ( debugMask & DBG_xxx )) ....
// 
// The DBG_CONFIG bit allows for the entire debugging messages to be enabled or
// disabled. This feature will also be used when we test whether we even have a
// console to print to or not. If there is no console, all the print functions
// will not be executed.
//
//----------------------------------------------------------------------------------------
enum DebugOptions : uint16_t {

    CDC_DBG_CONFIG      = ( 1U << 15 ),
    CDC_DBG_SETUP       = ( 1U << 0 ),
    CDC_DBG_I2C         = ( 1U << 1 ),
    CDC_DBG_SPI         = ( 1U << 2 ),
    CDC_DBG_PWM         = ( 1U << 3 ),
    CDC_DBG_UART        = ( 1U << 4 ),
    CDC_DBG_GPIO        = ( 1U << 5 )
};

//----------------------------------------------------------------------------------------
// Error status codes. The errors are used when setting up the Hal library. 
// During operation, all routines validate the input for correctness. If they
// are not correct, the call is simply not performed and an error is returned.
//
//----------------------------------------------------------------------------------------
enum CdcStatus : uint8_t {

    NO_ERR              = 0,

    NOT_SUPPORTED_ERR   = 1,
    NOT_IMPLEMENTED_ERR = 2,
    NOT_INITIALZED_ERR  = 3,

    RES_NUM_ERR         = 4,
   
    TIMER_RES_ERR       = 10,
    DIO_PIN_ERR         = 11,
    ADC_PIN_ERR         = 12,
    PWM_PIN_ERR         = 13,
    UART_PIN_ERR        = 14,
    I2C_PIN_ERR         = 15,

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

//----------------------------------------------------------------------------------------
// Callback functions signatures. So far, there are the timer callbacks and the
// GPIO pin callback.
//
//----------------------------------------------------------------------------------------
extern "C" {

    typedef void ( *TimerCallback ) ( uint32_t timerVal );
    typedef void ( *GpioCallback ) ( uint8_t pin, uint8_t event );
}

//----------------------------------------------------------------------------------------
// Common constants.
// 
//----------------------------------------------------------------------------------------
const int       MAX_RES_DESC_ENTRIES    = 64;
const int       MAX_RES_NAME_SIZE       = 64;
const uint8_t   UNDEFINED_RES_ID        = 255;
const uint8_t   UNDEFINED_PIN           = 255;
const uint8_t   ILLEGAL_PIN             = 254;

//----------------------------------------------------------------------------------------
// The defined board types. When the runtime is initialized, the firmware will 
// pass the type to specify what board it expects. This value is compared to 
// what is actually stored in the NVM of the main controller board. If they 
// don't match, it is considered an error and the NVM needs to be configured to
//  support the firmware. 
//
//----------------------------------------------------------------------------------------
enum CdcBoardInfo : uint16_t {

    CDC_BT_NIL                  = 0,
    CDC_BT_MAIN                 = 1,
    CDC_BT_EXT                  = 2,

    CDC_BT_MAIN_CONTROLLER      = 10,
    CDC_BT_BASE_STATION         = 11,
    CDC_BT_BLOCK_CONTROLLER     = 12,
    BT_CAB_HANDHELD             = 13,

    CDC_BT_EXT_OCC_DETECT       = 80,
    CDC_BT_EXT_SV_32_IO_16      = 81,
    
};

//----------------------------------------------------------------------------------------
// The controller families. Currently, there is only the Raspberry PI Pico 
// family models.
//
//----------------------------------------------------------------------------------------
enum ControllerInfo : uint8_t {

    CDC_CF_UNDEFINED            = 0,
    CDC_CF_RP_PICO              = 1,

    CDC_CF_C_RP_2040            = 10,
    CDC_CF_C_RP_2350            = 11
};

//----------------------------------------------------------------------------------------
// The CDC resources have a type which tells us what the particular resource is.
// Note that the are "real"  hardware resources such as a GPIO pin, but also 
// logical resources such as a software timer. The value of 255 is used as the
// invalid resource Id.
//
//----------------------------------------------------------------------------------------
enum CdcResourceType : uint8_t {

    CDC_RT_UNDEFINED    = 0,
    CDC_RT_TIMER        = 1,
    CDC_RT_GPIO         = 2,
    CDC_RT_ADC          = 4,
    CDC_RT_PWM          = 5,
    CDC_RT_UART         = 7,
    CDC_RT_I2C          = 8,
    CDC_RT_CAN_BUS      = 9,

    CDC_RT_INVALID      = 255
};

//----------------------------------------------------------------------------------------
// There are predefined resource channels common to all boards. They are for 
// example the activity LED and the NVM I2C channel. These resources channels 
// numbers are consequently reserved and cannot be used by the firmware 
// programmer. The programmer identifies its resources relative to the start of
// user definable resource numbers.
//
//----------------------------------------------------------------------------------------
enum CdcResourceIdNum : uint8_t {

    CDC_RN_ACTIVITY_LED     = 0,
    CDC_RN_PFAIL            = 1,
    CDC_RN_CAN_BUS          = 2,
    CDC_RN_NVM              = 3,
    CDC_RN_EXT_NVM          = 4,
    
    CDC_RN_FIRST_USER_RN    = 8, 
    CDC_RN_UNDEFINED        = 255
};

//----------------------------------------------------------------------------------------
// DIO pin related definitions. A digital pin can be an input pin, with or 
// without pull-up, or an output pin. DIO pins can also be associated with an 
// interrupt handler. The handler itself is mapped to an edge or level event.
//
//----------------------------------------------------------------------------------------
enum dioMode : uint8_t {

    CDC_DIO_IN              = 0,
    CDC_DIO_OUT             = 1,
    CDC_DIO_IN_PULLUP       = 2,
    CDC_DIO_DEFAULT         = 3
};

//----------------------------------------------------------------------------------------
// GPIO interrupts are detected as level change or edge changes.
//
//----------------------------------------------------------------------------------------
enum intEventTyp : uint8_t {

    CDC_EVT_NONE            = 0,
    CDC_EVT_LOW             = 1,
    CDC_EVT_HIGH            = 2,
    CDC_EVT_FALL            = 3,
    CDC_EVT_RISE            = 4,
    CDC_EVT_CHANGE          = 5
};

//----------------------------------------------------------------------------------------
// PWM duty cycle. We LCS library specifies a range of 0 to 255 as the duty 
// cycle value.
//
//----------------------------------------------------------------------------------------
enum PwmDutyCycle : uint8_t {

    CDC_MIN_DUTY_CCYCLE     = 0,
    CDC_MAX_DUTY_CYCLE      = 255
};

//----------------------------------------------------------------------------------------
// The CDC resource descriptor describes a CDC resource channel. A channel is a
// hardware entity that the CDC layer offers to the LCS library and the firmware
// programmer. Primarily it contains the actual pin settings but also any other
// relevant data for the particular channel. A channel entry also contains a 
// type and the resource ID number. The resource ID specifies the index in the 
// resource array where the resource can be found.
//
//----------------------------------------------------------------------------------------
struct CdcResourceDesc {

    uint8_t type;
    uint8_t resId;

    union {

        struct {

            uint32_t    timerVal;

        } timer;

        struct {

            uint8_t     pinA;
            uint8_t     pinB;
            uint8_t     pinMode;
        } gpio;

        struct  {

            uint8_t     adcPin;
            uint8_t     adcNum;

        } adc;

        struct {
            
            uint8_t     pinA;
            uint8_t     pinB;
            uint32_t    frequency;

        } pwm;

        struct {

            uint8_t     rxPin;
            uint8_t     txPin;
            uint32_t    baudRate;

        } uart;

        struct {

            uint8_t     sclPin;
            uint8_t     sdaPin;
            uint32_t    baudRate;
            uint32_t    i2cTimeoutMs; 

        } i2c;

        struct {

            uint8_t     rxPin;
            uint8_t     txPin;
            uint32_t    baudRate;
            bool        twoCores;

        } can;
    };
};

//----------------------------------------------------------------------------------------
// The resource descriptor map is the data structure passed to the runtime 
// library initialization routine. The data is used in the configuration process
// of the particular hardware board. We will over time have several boards and 
// consequently a map for each board version. 
//
//----------------------------------------------------------------------------------------
struct CdcResourceDescMap {

    uint32_t            boardMword;
    uint16_t            boardInfo;                      // type/subtype
    uint16_t            boardVersion;                   // major / sub version
    uint16_t            boardCtrlInfo;                  // family / cType
    
    // ??? really have them here ?
    uint16_t            options;
    uint16_t            debugMask;  

    char                name[ MAX_RES_NAME_SIZE ];  
    CdcResourceDesc     map[ MAX_RES_DESC_ENTRIES ];
};

//----------------------------------------------------------------------------------------
// The CDC library routines that make up the hardware abstraction layer. Most 
// routines have a return code, representing the return status of the routine.
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// Basic init and error handling.
//
//----------------------------------------------------------------------------------------
uint8_t         cdcInit( CdcResourceDescMap *dMap );

CdcResourceDesc *lookupResourceDesc( uint8_t rNum, uint8_t type );
void            printResourceDescMap( CdcResourceDescMap *dMap );
void            printResourceMap( );

uint32_t        getVersion( );
uint32_t        getPatchLevel( );

void            fatalError( uint8_t errNum, 
                            char *str = nullptr,  
                            uint8_t rStat = NO_ERR );

uint16_t        getDebugMask( );
void            setDebugMask( uint16_t mask = 0 );

//----------------------------------------------------------------------------------------
// General utility routines.
//
//----------------------------------------------------------------------------------------
uint32_t        getMillis( );
uint32_t        getMicros( );
void            sleepMillis( uint32_t val );
void            sleepMicros( uint32_t val );
uint32_t        createUid( );

//----------------------------------------------------------------------------------------
// The console IO functions. We will provide a serial IO via the USB connector
// of the PICO. The files need to be linked with the "tinyUSB" library and the 
// cmake file needs to set the option. Then we can use scanf and printf and so 
// on. In addition, we need  function  that just attempts to read a character
// and returns immediately when there is none.
//
//----------------------------------------------------------------------------------------
uint8_t         configureConsoleIO( );
bool            isConsoleConnected( );
char            getConsoleChar( uint32_t timeoutVal = 0 );

//----------------------------------------------------------------------------------------
// General controller and board info routines.
//
//----------------------------------------------------------------------------------------
uint8_t         getBoardId( uint16_t *bId );
uint8_t         getControllerFamily( uint16_t *family );
uint8_t         getControllerChip( uint16_t *chip );
uint8_t         getChipMemSize( uint32_t *size );
uint8_t         getChipNvmSize( uint32_t *size );
uint8_t         getChipCpuFrequency( uint32_t *frequency );

uint8_t         watchDogEnable( bool enable );
uint8_t         watchDogUpdate( );
uint8_t         watchDogCausedReboot( bool *reboot );

//----------------------------------------------------------------------------------------
// Timer management routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configureTimer( uint8_t rNum, TimerCallback functionId );
uint8_t         startRepeatingTimer( uint8_t rNum, uint32_t val );
uint8_t         stopRepeatingTimer( uint8_t rNum );
uint8_t         setRepeatingTimerLimit( uint8_t rNum, uint32_t val );
uint8_t         getRepeatingTimerLimit( uint8_t rNum, uint32_t *val );

//----------------------------------------------------------------------------------------
// Analog input routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configureAdc( uint8_t rNum );
uint8_t         configureAdc( uint8_t rNum, uint8_t adcPin, uint8_t adcNum );
uint8_t         readAdc( uint8_t rNum, uint16_t *val );

//----------------------------------------------------------------------------------------
// Digital Input/Output routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configureDio( uint8_t rNum );
uint8_t         configureDio(   uint8_t rNum, 
                                uint8_t pinA, 
                                uint8_t pinB = UNDEFINED_PIN, 
                                uint8_t pinMode = CDC_DIO_DEFAULT );
uint8_t         readDio( uint8_t rNum, bool *valA, bool *valB = nullptr );
uint8_t         writeDio( uint8_t rNum, bool valA, bool valB = false );
uint8_t         toggleDio( uint8_t rNum );
uint8_t         registerDioCallback( uint8_t rNum, 
                                     uint8_t event, 
                                     GpioCallback func );
uint8_t         unregisterDioCallback( uint8_t rNum );

//----------------------------------------------------------------------------------------
// PWM output routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configurePwm( uint8_t rNum );
uint8_t         configurePwm(   uint8_t rNum, 
                                uint8_t pinA, 
                                uint8_t pinB, 
                                uint32_t frequency );
                                
uint8_t         setPwmFrequency( uint8_t rNum, uint32_t frequency );

uint8_t         writePwm( uint8_t rNum,
                          uint8_t dutyCycleA, 
                          uint8_t dutyCycleB = 0 ); 

uint8_t         syncPwm( uint8_t rNum );

//----------------------------------------------------------------------------------------
// Serial IO routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configureUart( uint8_t rNum );
uint8_t         configureUart(  uint8_t rNum, 
                                uint8_t rxPin, 
                                uint8_t txPin, 
                                uint32_t baudRate );
uint8_t         startUartRead( uint8_t rNum );
uint8_t         stopUartRead( uint8_t rNum );
uint8_t         getUartBuffer( uint8_t rNum, uint8_t *buf, uint8_t bufLen );

//----------------------------------------------------------------------------------------
// I2C management routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configureI2C( uint8_t rNum );
uint8_t         configureI2C( uint8_t rNum, 
                              uint8_t sclPin, 
                              uint8_t sdaPin, 
                              uint32_t baudRate, 
                              uint32_t timeOut );
uint8_t         i2cBusreset( uint8_t rNum );
uint8_t         i2cWrite( uint8_t rNum, 
                          uint8_t i2cAdr, 
                          uint8_t *buf, 
                          uint16_t len, 
                          bool stopBit = false );
uint8_t         i2cRead( uint8_t rNum,
                         uint8_t i2cAdr, 
                         uint8_t *buf, 
                         uint16_t len, 
                         bool stopBit = false );

uint8_t         i2cGetSclPin( uint8_t rNum );
uint8_t         i2cGetSdaPin( uint8_t rNum );
uint8_t         i2cGetBaudrate( uint8_t rNUm );
uint8_t         scanI2CBus( uint8_t rNum );

//----------------------------------------------------------------------------------------
// CAN Bus hardware routines.
//
//----------------------------------------------------------------------------------------
uint8_t         configureCanBus( uint8_t rNum );
uint8_t         configureCanBus( uint8_t rNum, 
                                 uint8_t rxPin, 
                                 uint8_t xPin, 
                                 uint32_t baudRate, 
                                 bool twoCores = false );
uint8_t         canGetRxPin( uint8_t rNum );
uint8_t         canGetTxPin( uint8_t rNum );
uint32_t        canGetBaudrate( uint8_t rNum );
bool            canGetTwoCores( uint8_t rNum );

};

#endif
