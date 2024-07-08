//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Include file
//
//------------------------------------------------------------------------------------------------------------
// The controller dependent code layer concentrates all processor dependent code into one library. The idea
// is twofold. First, there needs to be a way to isolate the controller specific hardware from the LCS runtime
// Library as well as the extension module firmware. The Rasberry PI Pico offers a C++ SDK with a set of
// libraries to invoke the desired function rather than access to registers.The Pico also offers a great
// flexibilty of pin assignment for the hardware IO functions. Second, within the hardware IO boundaries of
// the controller family the individual hardware pin assignmnt used may vary from board to board design.
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
// Copyright (C) 2022 - 2024  Helmut Fieres
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
// #include <inttypes.h>

//------------------------------------------------------------------------------------------------------------
// All definitions and functions are in the CDC name space.
//
//------------------------------------------------------------------------------------------------------------
namespace CDC {

  //----------------------------------------------------------------------------------------------------------
  // Error status codes. The erros are used when setting up the Hal library. During operation, all routines
  // validate the input for correctness. If they are not correct, the call is simply not performed.
  //
  // ??? clean up a little ... what is really needed ?
  //----------------------------------------------------------------------------------------------------------
  enum CdcStatus : uint8_t {

    ALL_OK              = 0,
    INIT_PENDING        = 1,
    NOT_SUPPORTED       = 2,
    NOT_IMPLEMENTED     = 3,

    MEM_SIZE_ERR        = 10,

    READY_LED_PIN_ERR   = 12,
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

    I2C_PORT_ERR        = 30,
    I2C_CONFIG_ERR      = 31,
    I2C_WRITE_ERR       = 32,
    I2C_READ_ERR        = 33

  };

  //----------------------------------------------------------------------------------------------------------
  // Controller pin related definitions. A pin can be valid, undefined or illegal. An undefined pin for a pin
  // field in the configuration structure indicates that  the pin has not been used by the firmware
  // implementation but is a pin that the particular controller would support. An illegal pin means that the
  // pin is not offerered by this controller and cannot be assigned at all.
  //
  //----------------------------------------------------------------------------------------------------------
  const uint8_t UNDEFINED_PIN   = 255;
  const uint8_t ILLEGAL_PIN     = 254;

  //----------------------------------------------------------------------------------------------------------
  // The controller families. Currently, there is only the Raspberry PI Pico.
  //
  //----------------------------------------------------------------------------------------------------------
  enum ControllerFamily : uint8_t {

    CF_UNDEFINED    = 0,
    CF_RP_PICO      = 1
  };

  //----------------------------------------------------------------------------------------------------------
  // DIO pin related definitions. A digital pin can be an input pin, with or without pullup, or an ouput pin.
  // DIO pinns can also be assiciated with an interrupt handler. The handler itself is mapped to an edge or
  // level event.
  //
  //----------------------------------------------------------------------------------------------------------
  enum dioMode : uint8_t {

    IN            = 0,
    OUT           = 1,
    IN_PULLUP     = 2
  };


  //----------------------------------------------------------------------------------------------------------
  // GPIO interruots are detected as level change or edge changes.
  //
  //----------------------------------------------------------------------------------------------------------
  enum intEventTyp : uint8_t {

    EVT_NONE    = 0,
    EVT_LOW     = 1,
    EVT_HIGH    = 2,
    EVT_FALL    = 3,
    EVT_RISE    = 4,
    EVT_CHANGE  = 5
  };

  //----------------------------------------------------------------------------------------------------------
  // The UART modes. There are two implementions. The PICO offers two hardware UARTS. We use them with 8 bits
  // with a parity bit. The second type UART is a software implementation based on the PICO PIO blocks.
  //
  //----------------------------------------------------------------------------------------------------------
  enum UartMode : uint8_t {

    UART_MODE_UNDEFINED = 0,
    UART_MODE_8N1       = 1,
    UART_MODE_8N1_PIO   = 2
  };

  //----------------------------------------------------------------------------------------------------------
  // Callback functions signatures.
  //
  //----------------------------------------------------------------------------------------------------------
  extern "C" {

    typedef void ( *TimerCallback ) ( uint32_t timerVal );
    typedef void ( *GpioCallback ) ( uint8_t pin, uint8_t event );
  }

  //----------------------------------------------------------------------------------------------------------
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
  // the board. It will then just be read from there.
  //
  //----------------------------------------------------------------------------------------------------------
  struct CdcPinConfig {

    uint8_t   CFG_STATUS;

    uint8_t   PFAIL_PIN;
    uint8_t   EXT_INT_PIN;
    uint8_t   READY_LED_PIN;
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

    uint8_t   CAN_BUS_CTRL_MODE;
    uint8_t   CAN_BUS_RX_PIN;
    uint8_t   CAN_BUS_TX_PIN;
    uint32_t  CAN_BUS_DEF_ID;
  };

  //----------------------------------------------------------------------------------------------------------
  // The routines that make up the hardware abstraction layer. The routines expect hardware pin numbers.
  // To recap, the CDC layer offers a set of reserved resource names, such as "DIO_PIN_0", which describes
  // the resource containing the hardware pin and some flags. The configuration routines in this layer will use
  // these pins and other data stored to configure the hardware. Under the defined resource name name all
  // upper layers refer to the hardware using the to the configured IO capabilities.
  //
  // Complex resources, such as the UART or SPI interface, have more than one HW pin they will use. In this
  // case one of the HW pins, see the function documention, will serve as the handle to the resource.
  //
  //----------------------------------------------------------------------------------------------------------

  //----------------------------------------------------------------------------------------------------------
  // The console IO functions. We will provide a serial IO via the USB connector of the PICO. The files 
  // need to be linked with the "tinyUSB" library and the cmake file needs to set the option. Then we can
  // use scanf and printf and so on. In addition, we need  function  that just attemps to read a charaxcter
  // and returns immediately when there is none.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configureConsoleIO( );
  char          getConsoleChar( );
  bool          isConsoleConnected( );

  //----------------------------------------------------------------------------------------------------------
  // CDC setup and configuration routines. The idea is to help the library write with a default configuration
  // structure. All pins HW that are fixed in their location will be set. A library programmer will just get
  // that default structure and set the values necessary for the particular case.
  //
  //----------------------------------------------------------------------------------------------------------
  CdcPinConfig  getConfigDefault( );
  CdcPinConfig  *getConfigActual( );
  void          printConfigInfo( CdcPinConfig *ci );

  uint8_t       init( CdcPinConfig *ci );
  void          fatalError( uint8_t n );

  //----------------------------------------------------------------------------------------------------------
  // General controller routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint16_t      getFamily( );
  uint32_t      getVersion( );
  uint32_t      getChipMemSize( );
  uint32_t      getChipNvmSize( );
  uint32_t      getCpuFrequency( );
  uint32_t      getMillis( );
  uint32_t      getMicros( );
  void          sleepMillis( uint32_t val );
  void          sleepMicros( uint32_t val );

  //----------------------------------------------------------------------------------------------------------
  // The LCS runtime needs to buid a unique ID for the node.
  //
  //----------------------------------------------------------------------------------------------------------
  uint32_t      createUid( );

  //----------------------------------------------------------------------------------------------------------
  // Timer management routines.
  //
  //----------------------------------------------------------------------------------------------------------
  void          onTimerEvent( TimerCallback functionId );
  void          startRepeatingTimer( uint32_t val );
  void          setRepeatingTimerLimit( uint32_t val );
  uint32_t      getRepeatingTimerLimit( );
  void          stopRepeatingTimer( );

  //----------------------------------------------------------------------------------------------------------
  // Ananlog input routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configureAdc( uint8_t adcPin );
  uint16_t      getAdcRefVoltage( );
  uint16_t      getAdcDigitRange( );
  uint16_t      readAdc( uint8_t adcPin );

  //----------------------------------------------------------------------------------------------------------
  // Digital Input/Ouput routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configureDio( uint8_t dioPin, uint8_t Mode = IN );
  void          registerDioCallback( uint8_t dioPin, uint8_t event, CDC::GpioCallback func );
  void          unregisterDioCallback( uint8_t dioPin );
  bool          readDio( uint8_t dioPin );
  uint8_t       writeDio( uint8_t dioPin, bool val );
  uint8_t       toggleDio( uint8_t dioPin );
  uint32_t      readDioMask( uint32_t dioMask );
  uint8_t       writeDioMask( uint32_t dioMask, uint32_t dioVal );
  uint8_t       writeDioPair( uint8_t dioPin1, bool val1, uint8_t dioPin2, bool val2 );

  //----------------------------------------------------------------------------------------------------------
  // PWM output routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configurePwm( uint8_t   pwmPin,
                              uint32_t  pwmFreqency,
                              bool      phaseCorrect  = true,
                              bool      inverted      = false
                            );

  uint8_t       writePwm( uint8_t pwmPin, uint8_t dutyCycle );

  //----------------------------------------------------------------------------------------------------------
  // Serial IO routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configureUart( uint8_t rxPin, uint8_t txPin, uint32_t baudRate, UartMode mode );
  uint8_t       startUartRead( uint8_t rxPin );
  uint8_t       stopUartRead( uint8_t rxPin );
  uint8_t       getUartBuffer( uint8_t rxPin, uint8_t *buf, uint8_t bufLen );

  //----------------------------------------------------------------------------------------------------------
  // I2C management routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configureI2C( uint8_t sclPin, uint8_t sdaPin, uint32_t baudRate = 100 * 1000 );
  uint8_t       i2cWrite( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint8_t len, bool stopBit = false );
  uint8_t       i2cRead( uint8_t sclPin, uint8_t i2cAdr, uint8_t *buf, uint8_t len, bool stopBit = false );

  //----------------------------------------------------------------------------------------------------------
  // SPI management routines.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t       configureSPI( uint8_t sclkPin, uint8_t mosiPin, uint8_t misoPin, uint32_t baudRate = 10 * 1000 * 1000 );
  uint8_t       spiBeginTransaction( uint8_t sclkPin, uint8_t csPin );
  uint8_t       spiEndTransaction( uint8_t sclkPin, uint8_t csPin );
  uint8_t       spiRead( uint8_t sclkPin, uint8_t *buf, uint32_t len );
  uint8_t       spiWrite( uint8_t sclkPin, uint8_t *buf, uint32_t len );

};

#endif
