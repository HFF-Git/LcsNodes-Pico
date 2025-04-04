//------------------------------------------------------------------------------------------------------------
//
// LCS - Board Descriptors - Include file
//
//------------------------------------------------------------------------------------------------------------
// 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file
// Copyright (C) 2024 - 2025  Helmut Fieres
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


const int MAX_INST_DESC_ENTRIES = 64;

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
enum CdcInstanceType : uint8_t {

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


//------------------------------------------------------------------------------------------------------------
// The controller instance type. The controller itself has parameters we can set.
//
// ??? letÄs see where this takes us...
//------------------------------------------------------------------------------------------------------------
struct ControllerDesc {

    uint16_t    cFamily;
    uint32_t    memorySize;
    uint32_t    internalNvmSize;
};

//------------------------------------------------------------------------------------------------------------
// The controller features a watchdog facility. The idea is that when the controller software hangs, it will
// be restarted. To avoid the automatic restarting, the software periodically needs to reset the watchdog 
// timer. The interval timer specifies the maximum time without resetting the watchdog timer.
//
//------------------------------------------------------------------------------------------------------------
struct WatchDogInstDesc {

    uint32_t intervalMillis;
};

//------------------------------------------------------------------------------------------------------------
// Timer instances descriptor.
// 
// ??? option to specify whether the timer should restart while the interrupt is served or after.
//------------------------------------------------------------------------------------------------------------
struct TimerInstDesc {

    uint32_t intervalMillis;
};

//------------------------------------------------------------------------------------------------------------
// A GPIO instance descriptor declares the pin(s) and their mode. Note that the mode can be changed later via
// the config routine.
//
// ??? should we also have a concept to have a pin mask for multiple pins ?
//------------------------------------------------------------------------------------------------------------
struct GpioInstDesc {

    uint8_t     pinA;
    uint8_t     pinB;
    uint8_t     pinAMode;
    uint8_t     pinBMode;
};

//------------------------------------------------------------------------------------------------------------
// The ADC instance descriptor declares analog input.
//
//------------------------------------------------------------------------------------------------------------
struct AdcInstDesc {

    uint8_t adcPin;
};

//------------------------------------------------------------------------------------------------------------
// The PWM instance declares the pins(s) and the PWM options, such as frequency.
//
// ??? range to specify ? or always 0 .. 255 ?
//------------------------------------------------------------------------------------------------------------
struct PwmInstDesc {

    uint8_t     pwmPinA;
    uint8_t     pwmPinB;
    uint32_t    pwmFreqency;
    uint32_t    wrap;
    bool        inverted;
};

//------------------------------------------------------------------------------------------------------------
// The UART instance descriptor declares a serial IO interface. We need the Rx and Tx pins and the UART
// options.
//
//------------------------------------------------------------------------------------------------------------
struct UartInstDesc {

    uint8_t     rxPin;
    uint8_t     txPin;
    uint32_t    baudRate;
    uint8_t     dataBits;
    uint8_t     parityMode;
    uint8_t     stopBits;
};

//------------------------------------------------------------------------------------------------------------
// The I2C instance descriptor declares an I2C channel.
// 
//------------------------------------------------------------------------------------------------------------
struct I2CInstDesc {

    uint8_t     sclPin;
    uint8_t     sdaPin;
    uint32_t    baudRate;
    uint32_t    timeoutValMs;
};

//------------------------------------------------------------------------------------------------------------
// The SPI instance descriptor declares an SPI channel.
//
//------------------------------------------------------------------------------------------------------------
struct SPIInstDesc {

    uint8_t     selectPin;
    uint8_t     mosiPin;
    uint8_t     misoPin;
    uint8_t     sclkPin;
    uint32_t    frequency;
};

//------------------------------------------------------------------------------------------------------------
// The CAN Bus instance descriptor declares an the necessary CAN bus data.
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusInstDesc {

    uint8_t     canPinH;
    uint8_t     canPinL;
    uint32_t    baudRate;
    
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

    uint8_t     type;
    uint8_t     resId;

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
//
//
//
// a little test ...
// ??? perhaps a separate file for each board and just include it here.... 
//
//------------------------------------------------------------------------------------------------------------
const struct CdcInstanceDescMap test = {

    .flags = 0,
    .size  = 0,
    
    .map = {

        {   .type   = CDC_IT_ADC,
            .resId  = CDC_RID_ADC_0,

            .adc    =  {   
            
                .adcPin     = 0 
            }   
        },

        {   .type   = CDC_IT_ADC,
            .resId  = CDC_RID_ADC_1,

            .adc    =  {   
            
                .adcPin     = 0 
            }   
        }

    }
};

}; // namespace CDC


#endif