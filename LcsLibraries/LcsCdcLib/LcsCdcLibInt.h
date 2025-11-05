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
#ifndef CDC_LIB_INT_h
#define CDC_LIB_INT_h

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <cstring>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/time.h"
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

#include "LcsUtilLib.h"
#include "LcsCdcLib.h"

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
using namespace CDC;

//----------------------------------------------------------------------------------------
// Valid pin mappings for the Raspberry PI Pico board. We construct a set of bitmask
// for the pin numbers. Pin Numbers range from 0 to 28. The bitmasks specify wether a
// pin can be assigned to the hardware type purpose. During configuration of a CDC 
// function, the pins are checked against these bitmasks. All pins can be used as GPIO
// pins or PWM pins. All other hardware functions are bound to dedicated pins. Note 
// that we do not check for assigning a pin to several different hardware functions. 
// All we check is that the pin can be used for the desired purpose. A check performed
// by the CDC library routines is simply done through:
//
//    if (( 1 <<  pin ) & VALID_xxx )
//
//----------------------------------------------------------------------------------------
const uint8_t  MAX_PIN_NUM          = 28;

const uint32_t VALID_GPIO_PINS      = 0x1FFFFFFF;
const uint32_t VALID_PWM_PINS       = 0x1FFFFFFF;
const uint32_t VALID_ADC_PINS       = ( 1 << 26 ) | ( 1 << 27 ) | ( 1 << 28 );

const uint32_t VALID_I2C_0_SDA_PINS = ( 1 << 0  ) | ( 1 << 4  ) | ( 1 << 8  ) |
                                      ( 1 << 12 ) | ( 1 << 16 ) | ( 1 << 20 );
const uint32_t VALID_I2C_0_SCL_PINS = ( 1 << 1  ) | ( 1 << 5  ) | ( 1 << 9  ) |
                                      ( 1 << 13 ) | ( 1 << 17 ) | ( 1 << 21 );

const uint32_t VALID_I2C_1_SDA_PINS = ( 1 << 2  ) | ( 1 << 6  ) | ( 1 << 10 ) |
                                      ( 1 << 14 ) | ( 1 << 18 ) | ( 1 << 26 );
const uint32_t VALID_I2C_1_SCL_PINS = ( 1 << 3  ) | ( 1 << 7  ) | ( 1 << 11 ) |
                                      ( 1 << 15 ) | ( 1 << 19 ) | ( 1 << 27 );

const uint32_t VALID_UART_0_TX_PINS = ( 1 << 0  ) | ( 1 << 12 ) | ( 1 << 16 );
const uint32_t VALID_UART_0_RX_PINS = ( 1 << 1  ) | ( 1 << 13 ) | ( 1 << 17 );

const uint32_t VALID_UART_1_TX_PINS = ( 1 << 4  ) | ( 1 << 8  );
const uint32_t VALID_UART_1_RX_PINS = ( 1 << 5  ) | ( 1 << 9  );

const uint32_t VALID_SPI_0_SCK_PINS = ( 1 << 2  ) | ( 1 << 6  ) | ( 1 << 18 );
const uint32_t VALID_SPI_0_TX_PINS  = ( 1 << 3  ) | ( 1 << 7  ) | ( 1 << 19 );
const uint32_t VALID_SPI_0_RX_PINS  = ( 1 << 0  ) | ( 1 << 4  ) | ( 1 << 16 );

const uint32_t VALID_SPI_1_SCK_PINS = ( 1 << 10 ) | ( 1 << 14 );
const uint32_t VALID_SPI_1_TX_PINS  = ( 1 << 11 ) | ( 1 << 15 );
const uint32_t VALID_SPI_1_RX_PINS  = ( 1 << 8  ) | ( 1 << 12 );

const uint32_t VALID_I2C_0_PINS  = VALID_I2C_0_SDA_PINS | VALID_I2C_0_SCL_PINS;
const uint32_t VALID_I2C_1_PINS  = VALID_I2C_1_SDA_PINS | VALID_I2C_1_SCL_PINS;

const uint32_t VALID_UART_0_PINS = VALID_UART_0_TX_PINS | VALID_UART_0_RX_PINS;
const uint32_t VALID_UART_1_PINS = VALID_UART_1_TX_PINS | VALID_UART_1_RX_PINS;

//----------------------------------------------------------------------------------------
// Characteristics of the Raspberry Pi Pico and some key constants for the CDC library.
// 
//----------------------------------------------------------------------------------------
const uint16_t  MAX_CPU_CORE                = 2;
const uint16_t  MAX_INT_PIN                 = 24;

const uint16_t  MAX_RESOURCE_ENTRIES        = MAX_RES_DESC_ENTRIES;
const uint16_t  MAX_RES_NAME                = 64;
const uint8_t   MAX_UART_BUF_SIZE           = 8;

const uint32_t  WATCHDOG_TIMER_MILLIS       = 2000;
const uint32_t  ADC_REF_VOLTAGE_MILLIS      = 3300;
const uint16_t  ADC_DIGIT_RANGE             = 1024;

//----------------------------------------------------------------------------------------
// The default time intervals. The debounce value will determine when we consider
// a button pushed. The click value defines how ling we press a button for a click
// and the press value defines what is considered a long push. There is also the
// option to detect a double click. Care needs to be taken that the click interval
// is not too long, which would result in a long press, and not too short, which
// would lead to not even consider a double click. The default values are the result
// of testing some common tactical switch buttons.
//
//----------------------------------------------------------------------------------------
const uint16_t  DEFAULT_DEBOUNCE_MILLIS   = 40;
const uint16_t  DEFAULT_CLICK_MILLIS      = 40;
const uint16_t  DEFAULT_PRESS_MILLIS      = 500;

//----------------------------------------------------------------------------------------
// Controller dependent code uses a set of hardware resource structures to control the
// controller hardware. When a particular resource, e.g. an I2C channel, is configured
// all further access will use the resource data for its operation. 
//
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// The resource map has an array of the resources. 
//
//----------------------------------------------------------------------------------------
struct CdcResource {

    uint8_t type;
    uint8_t resId;

    union {

        //--------------------------------------------------------------------------------
        // A timer resource. We need to keep the local timer instance data for the PICO.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint32_t            timerVal;
            TimerCallback       timerCallback;
            repeating_timer_t   timerData;
            bool                timerHighPri;

        } timer;

        //--------------------------------------------------------------------------------
        // The GPIO resource is perhaps the most fundamental resource. It manages a HW
        // pin. Optional, we can have two pins which then act as pair and are read from
        // or written to simultaneously.
        // 
        //--------------------------------------------------------------------------------
        struct {

            uint8_t         pinA;
            uint8_t         pinB;
            uint8_t         pinMode;
            GpioCallback    handler;

        } gpio;

        //--------------------------------------------------------------------------------
        // An ADC instance. The PICO supports up to three ADC inputs. When we use such
        // an input, the corresponding instance data is kept in this structure. We also
        // keep the PICO ADC number, so we can select the correct HW instance.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t   adcPin;
            uint8_t   adcNum;

        } adc;

        //--------------------------------------------------------------------------------
        // The PWM output resource manages a PWM configured output pin. We keep track 
        // of one or two pins, which must be on the same PWM slice. The idea is  that 
        // we use for H-Bridge control two output signals, which act as a pair. 
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t     pinA;
            uint8_t     pinB;
            uint32_t    frequency;
            uint        wrap;
            uint        channel;
            uint        sliceNum;
            bool        inverted;
            bool        phaseCorrect;

        } pwm;

        //--------------------------------------------------------------------------------
        // UARTS are used to read in a serial stream from the RailCom detectors. There
        // can be two hardware based UART resources. The resource also keeps a small
        // buffer where the data is read into.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t           rxPin;
            uint8_t           txPin;
            uint16_t          baudSetting;
            uint8_t           dataBits;
            uart_parity_t     parityMode;
            uint8_t           stopBits;
            int               uartIrq;

            volatile uint8_t  rxBufIndex;
            volatile uint8_t  rxDataBuf[ MAX_UART_BUF_SIZE ];

            uart_inst_t       *uartHw;

        } uart;
    
        //--------------------------------------------------------------------------------
        // The PICO features two I2C HW channels. The resource data contains the GPIO
        // pins assigned, the baud rate and a timeout. We also keep an I2C address root,
        // which comes in handy for addressing chips with the same root address.
        //
        //--------------------------------------------------------------------------------
        struct {

            uint8_t     sclPin;
            uint8_t     sdaPin;
            uint8_t     i2cAdrRoot;
            uint32_t    baudRate;
            uint32_t    timeoutValMs;

            i2c_inst_t  *i2cHw;

        } i2c;

        //--------------------------------------------------------------------------------
        // The CAN bus resource. Although our current controller does not feature a
        // CAN bus hardware, the resource describes the hardware elements needed. 
        // We currently use a software version based on a PIO program to implement the
        // CAN bus layer. 
        // 
        //--------------------------------------------------------------------------------
        struct {

            uint8_t         canPinRx;
            uint8_t         canPinTx;
            uint32_t        baudRate;
            uint32_t        canId;
            bool            twoCores;

        } can;
    };
};

//----------------------------------------------------------------------------------------
// The resource map is the central data structure to talk to the hardware. It is built
// at runtime startup using the resource descriptor map. Essentially it contain all the
// data from the resource descriptors and depending on the descriptor type the PICO
// data structures necessary.
//
//----------------------------------------------------------------------------------------
struct CdcResourceMap {

    uint16_t            boardId;     
    uint16_t            cFamily;
    uint16_t            cType;
    uint16_t            cpuCores;
    uint32_t            memorySize;
    uint32_t            eepromSize;
    uint32_t            watchDogIntervallMillis;
    uint16_t            adcRefVoltageMillis;
    uint16_t            adcDigitRange;

    char                name[ MAX_RES_NAME ];
    CdcResource         map[ MAX_RESOURCE_ENTRIES ];
};

#endif