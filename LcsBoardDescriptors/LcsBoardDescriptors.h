//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Include file
//
//------------------------------------------------------------------------------------------------------------
// Board descriptors define the controller / board pin and function mapping. While the CDC layer abstracts
// the various hardware functions, the board descriptor table defines the pin mapping and a few other values
// for the particular board. The hardware functions are called resources and define the pins and other 
// attributes used. For example, a UART resource needs the receive and transmit pins, as well as what data
// length, stop bits, and so on are set. There are also software resources such as a repeating timer. Each
// board has a type and a unique ID by which the correct descriptor map can be located. Upon library start,
// each resource is configured from this descriptor data.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Include file
// Copyright (C) 2025 - 2025  Helmut Fieres
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
#ifndef Lcs_BoardDescriptors_h
#define Lcs_BoardDescriptors_h

namespace CDC {

//------------------------------------------------------------------------------------------------------------
// Common constants.
// 
//------------------------------------------------------------------------------------------------------------
const int       MAX_INST_DESC_ENTRIES   = 64;
const int       MAX_DESC_NAME           = 64;
const int       MAX_RES_ID_NAME         = 16;
const uint8_t   UNDEFINED_PIN           = 255;
const uint8_t   ILLEGAL_PIN             = 254;

//------------------------------------------------------------------------------------------------------------
// The CDC resources have a type which tells us what the particular resource is. Note that the are "real"
// hardware resources such as a GPIO pin, but also logical resources such as a software timer.
//
//------------------------------------------------------------------------------------------------------------
enum CdcResourceType : uint16_t {

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
    CDC_IT_CAN_BUS      = 10,

    CDC_IT_GPIO_PAIR    = 100,
    CDC_IT_PWM_PAIR     = 101,
};

//------------------------------------------------------------------------------------------------------------
// The controller families. Currently, there is only the Raspberry PI Pico family models.
//
//------------------------------------------------------------------------------------------------------------
enum ControllerFamily : uint16_t {

    CDC_CF_UNDEFINED    = 0,
    CDC_CF_RP_PICO      = 1,
};

//------------------------------------------------------------------------------------------------------------
// The controller chips in a family. Currently, there is only the Raspberry PI Pico models RP2040 and RP2350.
//
//------------------------------------------------------------------------------------------------------------
enum ControllerChip : uint16_t {

    CDC_CF_C_UNDEFINED  = 0,
    CDC_CF_C_RP_2040    = 1,
    CDC_CF_C_RP_2350    = 2,
};

//------------------------------------------------------------------------------------------------------------
// DIO pin related definitions. A digital pin can be an input pin, with or without pull-up, or an output 
// pin. DIO pins can also be associated with an interrupt handler. The handler itself is mapped to an edge
// or level event.
//
//------------------------------------------------------------------------------------------------------------
enum dioMode : uint8_t {

    CDC_DIO_IN              = 0,
    CDC_DIO_OUT             = 1,
    CDC_DIO_IN_PULLUP       = 2
};

//------------------------------------------------------------------------------------------------------------
// GPIO interrupts are detected as level change or edge changes.
//
//------------------------------------------------------------------------------------------------------------
enum intEventTyp : uint8_t {

    CDC_EVT_NONE            = 0,
    CDC_EVT_LOW             = 1,
    CDC_EVT_HIGH            = 2,
    CDC_EVT_FALL            = 3,
    CDC_EVT_RISE            = 4,
    CDC_EVT_CHANGE          = 5
};

// ??? to think about, just keep it for HW Uarts... ?
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

    CDC_MIN_DUTY_CCYCLE     = 0,
    CDC_MAX_DUTY_CYCLE      = 255
};


//------------------------------------------------------------------------------------------------------------
// Each resource has a unique ID. The ID is used in the configuration routines to create and locate the 
// particular entry. The IDs are used by the upper layers to obtain the handle that was created when the 
// resource was configured. From thereon, the handle is the key argument to pass for a configured resource.
//
//------------------------------------------------------------------------------------------------------------
enum CdcResIdNames {

    CDC_RID_UNDEFINED   = 0,
    CDC_RID_CONTROLLER  = 2,

    CDC_RID_TIMER_0     = 3,
    CDC_RID_TIMER_1     = 4,
    CDC_RID_TIMER_2     = 5,
    CDC_RID_TIMER_3     = 6,

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
    CDC_RID_DIO_15      = 25,

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

//------------------------------------------------------------------------------------------------------------
// The controller resource type. The controller itself has parameters we can set. We also have parameters
// such as the ADC voltage, that applies to the controller chip ADC subsystem.
//
//------------------------------------------------------------------------------------------------------------
struct ControllerDesc {

    uint16_t    controllerFamily;
    uint16_t    controllerChip;
    uint16_t    cpuCores;
    uint32_t    memorySize;
    uint32_t    internalNvmSize;
    uint32_t    watchDogIntervallMillis;
    uint16_t    adcRefVoltage;
    uint16_t    adcDigitRange;

    // ??? perhaps more to come what is the same across all resources...
};

//------------------------------------------------------------------------------------------------------------
// Timer resources descriptor.
// 
// ??? option to specify whether the timer should restart while the interrupt is served or after.
//------------------------------------------------------------------------------------------------------------
struct TimerResourceDesc {

    uint32_t intervalMillis;
};

//------------------------------------------------------------------------------------------------------------
// A GPIO resource descriptor declares the pin(s) and their mode. Note that the mode can be changed later via
// the config routine.
//
// ??? should we also have a concept to have a pin mask for multiple pins ?
//------------------------------------------------------------------------------------------------------------
struct GpioResourceDesc {

    uint8_t     pin;
    uint8_t     pinMode;
};

struct GpioPairResourceDesc {

    uint8_t     pinA;
    uint8_t     pinB;
    uint8_t     pinMode;
};

//------------------------------------------------------------------------------------------------------------
// The ADC resource descriptor declares analog input.
//
//------------------------------------------------------------------------------------------------------------
struct AdcResourceDesc {

    uint8_t     adcPin;
    uint16_t    adcDigitRange;
    uint16_t    adcRefVoltageMilliVolt;
};

//------------------------------------------------------------------------------------------------------------
// The PWM resource declares the pins(s) and the PWM options, such as frequency.
//
// ??? range to specify ? or always 0 .. 255 ?
//------------------------------------------------------------------------------------------------------------
struct PwmResourceDesc {

    // ??? pin and frequency would be enough ...

    uint8_t     pwmPinA;
    uint8_t     pwmPinB;
    uint32_t    pwmFreqency;
    uint32_t    wrap;
    bool        inverted;
};

struct PwmPairResource {

    uint8_t     pwmPinA;
    uint8_t     pwmPinB;
    uint32_t    pwmFreqency;
};

//------------------------------------------------------------------------------------------------------------
// The UART resource descriptor declares a serial IO interface. We need the Rx and Tx pins and the UART
// options.
//
//------------------------------------------------------------------------------------------------------------
struct UartResourceDesc {

    uint8_t     rxPin;
    uint8_t     txPin;
    uint32_t    baudRate;
    uint8_t     dataBits;
    uint8_t     parityMode;
    uint8_t     stopBits;
};

//------------------------------------------------------------------------------------------------------------
// The I2C resource descriptor declares an I2C channel.
// 
//------------------------------------------------------------------------------------------------------------
struct I2CResourceDesc {

    uint8_t     sclPin;
    uint8_t     sdaPin;
    uint32_t    baudRate;
    uint32_t    timeoutValMs;
};

//------------------------------------------------------------------------------------------------------------
// The SPI resource descriptor declares an SPI channel.
//
//------------------------------------------------------------------------------------------------------------
struct SPIResourceDesc {

    uint8_t     selectPin;
    uint8_t     mosiPin;
    uint8_t     misoPin;
    uint8_t     sclkPin;
    uint32_t    frequency;
};

//------------------------------------------------------------------------------------------------------------
// The CAN Bus resource descriptor declares an the necessary CAN bus data.
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusResourceDesc {

    uint8_t     canPinH;
    uint8_t     canPinL;
    uint32_t    baudRate;
    
    // ??? options for can2040 specifics ?
    
};

//------------------------------------------------------------------------------------------------------------
// A CDC configuration descriptor is the counterpart to the resources. A descriptor for a given resource type
// will contain all the information to configure that resource. An resource can be configured with the 
// configure routine and its parameter list or based on the data in this descriptor. It should be possible to
// configure the entire board based on an array of such descriptors. The idea is that each board can be 
// uniquely described with such an array.
//
//------------------------------------------------------------------------------------------------------------
struct CdcResourceConfigDesc {

    char        name[ MAX_RES_ID_NAME ];
    bool        configured;
    uint16_t    type;
  
    union {

        ControllerDesc      ctl;
        TimerResourceDesc       timer;
        GpioResourceDesc        gpio;
        GpioPairResourceDesc    gpioP;
        PwmResourceDesc         pwm;
        PwmPairResource         pwmP;
        UartResourceDesc        uart;
        AdcResourceDesc         adc;
        I2CResourceDesc         i2c;
        SPIResourceDesc         spi;
        CanBusResourceDesc      can;
    };
};

//------------------------------------------------------------------------------------------------------------
// The CDC resource map is the data structure that has an entry for each declared resource. Each map is 
// uniquely associated with a board.
//
//------------------------------------------------------------------------------------------------------------
struct CdcResourceDescMap {

    char name[ MAX_DESC_NAME ];
   
    // ??? boardId ? version ?

    CdcResourceConfigDesc map[ MAX_INST_DESC_ENTRIES ];
};


//------------------------------------------------------------------------------------------------------------
// This may be a better approach ?
//
//------------------------------------------------------------------------------------------------------------
struct CdcResourceDescMapNew {

    char name[ MAX_DESC_NAME ];
    // ??? boardId ? version ?  

    ControllerDesc      ctl;

    TimerResourceDesc   timer0;
    TimerResourceDesc   timer1;
    TimerResourceDesc   timer2;
    TimerResourceDesc   timer3;

    GpioResourceDesc    dio0;
    GpioResourceDesc    dio1;
    GpioResourceDesc    dio2;
    GpioResourceDesc    dio3;
    GpioResourceDesc    dio4;
    GpioResourceDesc    dio5;
    GpioResourceDesc    dio6;
    GpioResourceDesc    dio7;
    GpioResourceDesc    dio8;
    GpioResourceDesc    dio9;
    GpioResourceDesc    dio10;
    GpioResourceDesc    dio11;
    GpioResourceDesc    dio12;
    GpioResourceDesc    dio13;
    GpioResourceDesc    dio14;
    GpioResourceDesc    dio15;

    PwmResourceDesc     pwm0;
    PwmResourceDesc     pwm1;
    PwmResourceDesc     pwm2;
    PwmResourceDesc     pwm3;
    PwmResourceDesc     pwm4;
    PwmResourceDesc     pwm5;
    PwmResourceDesc     pwm6;
    PwmResourceDesc     pwm7;

    AdcResourceDesc     adc0;
    AdcResourceDesc     adc1;
    AdcResourceDesc     adc2;
    AdcResourceDesc     adc3;

    UartResourceDesc    uart0;
    UartResourceDesc    uart1;
    UartResourceDesc    uart2;
    UartResourceDesc    uart3;

    I2CResourceDesc     i2cNVM;
    I2CResourceDesc     i2cEXT;

    SPIResourceDesc     spi0;
    SPIResourceDesc     spi1;

    CanBusResourceDesc  can;



};


#include "LcsBoardTest.h"


}; // namespace CDC


#endif