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
    CDC_IT_TIMER        = 2,
    CDC_IT_GPIO         = 3,
    CDC_IT_ADC          = 4,
    CDC_IT_PWM          = 5,
    CDC_IT_UART         = 6,
    CDC_IT_I2C          = 7,
    CDC_IT_CAN_BUS      = 8
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

//------------------------------------------------------------------------------------------------------------
// PWM duty cycle.
//
//------------------------------------------------------------------------------------------------------------
enum PwmDutyCycle : uint8_t {

    CDC_MIN_DUTY_CCYCLE     = 0,
    CDC_MAX_DUTY_CYCLE      = 255
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
    uint16_t    adcRefVoltageMillis;
    uint16_t    adcDigitRange;
    uint8_t     ledPin;
    uint8_t     pFailPin;

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
// the config routine. Optionally, two pins can be set simultaneously.
//
//------------------------------------------------------------------------------------------------------------
struct GpioResourceDesc {

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
};

//------------------------------------------------------------------------------------------------------------
// The PWM resource declares the pins(s) and the PWM frequency.
//
//------------------------------------------------------------------------------------------------------------
struct PwmResourceDesc {

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
// The CAN Bus resource descriptor declares an the necessary CAN bus data.
// 
//------------------------------------------------------------------------------------------------------------
struct CanBusResourceDesc {

    uint8_t     canPinRx;
    uint8_t     canPinTx;
    uint32_t    baudRate;
    bool        twoCores;
};

//------------------------------------------------------------------------------------------------------------
// A CDC configuration descriptor contains all the information to configure a resource. An resource will be 
// configured with the resource configure routine using the data in this descriptor. It should be possible to
// configure the entire board based on an array of such descriptors. The idea is that each board can be 
// uniquely described with such an array.
//
//------------------------------------------------------------------------------------------------------------
struct CdcResourceDesc {

    uint8_t     resId;
    uint16_t    type;
  
    union {

        ControllerDesc          ctl;
        TimerResourceDesc       timer;
        GpioResourceDesc        gpio;
        PwmResourceDesc         pwm;
        UartResourceDesc        uart;
        AdcResourceDesc         adc;
        I2CResourceDesc         i2c;
        CanBusResourceDesc      can;
    };
};

struct CdcResourceDescMap {

    uint16_t        options         = 0;
    uint16_t        entries         = 0;
    char            name[ 64 ]      = { 0 };

    CdcResourceDesc map[ 64 ];
};

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
#include "LcsBoardGenericRtLib.h"


}; // namespace CDC


#endif