//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// This source file contains the the Raspberry Pi controller family hardware 
// library code. The idea of this library is to shield the actual hardware of 
// processor and board implementation from the upper layers but still keep the
// flexibility and performance of the underlying hardware. 
//
// The library works with a concept of "resources". During startup, resources 
// are configured based on data from the board resource descriptor. Most 
// routines in the CDC layer use the resource id to access the particular 
// hardware function.
//
// A historic note. The original LCS code was written for Atmega and Pico. With
// the complete shift to PICO, the CDC library just serves as an interface to 
// the PICO functions. One day, we may see more different controllers and 
// controller families. 
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
#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

//----------------------------------------------------------------------------------------
// Name space CDC. All routines and definitions exported are in this name space.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// External functions.
//
//----------------------------------------------------------------------------------------
extern void initIsrTable( );
extern void setupAlarmPools( );

//----------------------------------------------------------------------------------------
// Global variables. We need to remember whether we initialized already. We also
// store the descriptor and resource map. Finally, we need to have a table for DIO 
// interrupt handlers, and the instances of the HW UART instances.
//
//----------------------------------------------------------------------------------------
bool                    initialized = false;

uint16_t                debugMask;
uint16_t                options;

CdcResourceDescMap      dMap;
CdcResourceMap          rMap;

//----------------------------------------------------------------------------------------
// "validPin" is called to check that a pin is in the correct number range, defined
// and matches the bitmask for the desired purpose. For example, configuring an I2C
// port will check that the two GPIO pins are indeed routable to an I2C HW block in 
// the PICO.
//
//----------------------------------------------------------------------------------------
bool validPin( uint8_t pin, uint32_t mask ) {

    if ( pin == UNDEFINED_PIN )     return ( true );
    if ( pin > MAX_PIN_NUM )        return ( false );
    return (( 1 << pin ) & mask );
}

//----------------------------------------------------------------------------------------
// Set up the CDC resource map with default values.
//
//----------------------------------------------------------------------------------------
void initResourceMap( CdcResourceMap *rMap ) {

    rMap -> boardId                     = 0;
    rMap -> cFamily                     = CDC_CF_UNDEFINED;
    rMap -> cType                       = CDC_CF_UNDEFINED;
    rMap -> cpuCores                    = 2;
    rMap -> memorySize                  = 264 * 1024;
    rMap -> eepromSize                  = 0;
    rMap -> watchDogIntervallMillis     = 2000;
    rMap -> adcRefVoltageMillis         = 3300;
    rMap -> adcDigitRange               = 1024; 
    rMap -> name[0 ]                    = 0;

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {
        
        rMap -> map[ i ].type = CDC_RT_UNDEFINED;
    }
} 

//----------------------------------------------------------------------------------------
// A resource is found by indexing into the resource map with index and resource 
// type. This is a critical routine and needs to be fast. Nevertheless, we will 
// check for a valid resource number and type.
//
//----------------------------------------------------------------------------------------
CdcResource *lookupResource( uint8_t rNum, uint8_t type ) {

    if ( ! initialized ) return( nullptr );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );
    if ( rMap.map[ rNum ].type != type ) return ( nullptr );
    return ( &rMap.map[ rNum ] );
}

//----------------------------------------------------------------------------------------
// The configuration routines will allocate the corresponding entry in the resource
// map. When the entry is found but of a different type, it is an error. When there
// is no entry yet, the entry is initialized with the type and can be used for the
// further configuration.
//
//----------------------------------------------------------------------------------------
CdcResource *allocateResourceType( uint8_t rNum, uint8_t type ) {

    if ( ! initialized ) return( nullptr );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );
    
    if ( rMap.map[ rNum ].type == CDC_RT_UNDEFINED ) {

        rMap.map[ rNum ].type   = type;
        rMap.map[ rNum ].resId  = rNum;
        return ( &rMap.map[ rNum ] );
    }
    else if ( rMap.map[ rNum ].type == type ) {

        return ( &rMap.map[ rNum ] );
    }
   else return ( nullptr );
}

//----------------------------------------------------------------------------------------
// For debugging purposes. Instead of conditional compilations, the debug mask will
// enable the printing of debug and trace data.
//
//----------------------------------------------------------------------------------------
void setDebugMask( uint16_t mask ) {

    debugMask = mask;
}

uint16_t getDebugMask( ) {

    return ( debugMask );
}

//----------------------------------------------------------------------------------------
// Version Info and patch level.
//
//----------------------------------------------------------------------------------------
uint32_t getVersion( ) {

    return ( CDC_LIB_VERSION );
}

uint32_t getPatchLevel( ) {
    
    return ( CDC_LIB_PATCH_LEVEL );
}

//----------------------------------------------------------------------------------------
// CDC library setup. The "init" routine will ready the CDC library and keep a copy
// of the descriptor map which will be used for the setup. The init routine can be
// called more than once without a problem.
//
//----------------------------------------------------------------------------------------
uint8_t cdcInit( CdcResourceDescMap *dMapPtr, uint16_t options, uint16_t debugMask ) {

    dMap = *dMapPtr;
 
    if ( ! initialized ) {

        initResourceMap( &rMap );
        debugMask = debugMask;
        options   = options;

        initIsrTable( );
        configureUsbIO( );
        setupAlarmPools( );

        initialized = true;
    }

    return ( NO_ERR );
}
 
//----------------------------------------------------------------------------------------
// "getResourceMap" will return a pointer to the configured resource map. This is 
// typically the map that was created with the data from the resource descriptor map.
//
//----------------------------------------------------------------------------------------
CdcResourceMap *getResourceMap( ) {

    return ( &rMap );
}

//----------------------------------------------------------------------------------------
// A resource descriptor is found by searching the resource Id in the map. This 
// gives us also the flexibility of arranging the resource descriptor entries in
// the board descriptor.
//
//----------------------------------------------------------------------------------------
CdcResourceDesc *lookupResourceDesc( uint8_t rNum, uint8_t type ) {

    if ( ! initialized ) return( nullptr );
    if ( rNum >= MAX_RESOURCE_ENTRIES ) return ( nullptr );

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResourceDesc *ptr = &dMap.map[ i ];
        if (( ptr -> resId == rNum ) && ( ptr -> type == type )) return( ptr );
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// "fatalError" is the error communication method when we cannot get anything to 
// work. The Raspberry Pi PICO has a small Led on the board. We will use this LED
// to "blink" an error code. There are up to eight codes. The sequence is as follows:
//
//    repeat forever:
//
//    - 1s ON, 0.5s 0FF
//    - for ( int i = 0; i < n; i++ ) { 0.5s ON; 0.5s OFF; }
//
// The only way to get out of this loop is then to reset the board. Fatal errors 
// are hopefully not many. One obvious one is when we cannot detect the NVM and thus
// know nothing about the board.
//
// If we have a console, we attempt to first write an error message to the console 
// before looping.
//
//----------------------------------------------------------------------------------------
void fatalError( uint8_t n, char *str, uint8_t rStat ) {

    if ( str != nullptr ) {

        if ( usbIsConnected( )) { 
            
            printf( "Fatal Error: %d: %s, rStat: %d\n", n, str, rStat );
        }
    }

    const uint8_t   ledPin      = 25;
    const uint32_t  longPulse   = 1000;
    const uint32_t  shortPulse  = 250;

    n = n % 8;

    gpio_init( ledPin );
    gpio_set_dir( ledPin, GPIO_OUT );

    while ( true ) {

        sleep_ms( longPulse );
       
        for ( int i = 0; i < n; i++ ) {

            gpio_put( ledPin, true );
            sleep_ms( shortPulse );
            gpio_put( ledPin, false );
            sleep_ms( shortPulse );
        }
    }
}

//----------------------------------------------------------------------------------------
// Simple timestamp and sleep functions.
// 
//----------------------------------------------------------------------------------------
uint32_t getMillis( ) {

    return ( to_ms_since_boot( get_absolute_time( )));
}

uint32_t getMicros( ) {

    return ( to_us_since_boot( get_absolute_time( )));
}

void sleepMillis( uint32_t val ) {

    sleep_ms( val );
}

void sleepMicros( uint32_t val ) {

    sleep_us( val );
}

//----------------------------------------------------------------------------------------
// "createUid" is the routine that produces a unique ID for the node. The scheme is 
// based on a random number. Alternatively we could use the unique flash chip ID on
// the board. 
//
//----------------------------------------------------------------------------------------
uint32_t createUid( ) {

    uint32_t rVal = 0;

    volatile 
    uint32_t *rnd_reg = (uint32_t *) ( ROSC_BASE + ROSC_RANDOMBIT_OFFSET );

    for ( int k = 0; k < 32; k++ ) {

        rVal = rVal << 1;
        rVal = rVal + ( 0x00000001 & ( *rnd_reg ));
    }

    return ( rVal );
}

//----------------------------------------------------------------------------------------
// Processor general attributes. 
//
//----------------------------------------------------------------------------------------
uint8_t getControllerFamily( uint16_t *family ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *family = rMap.cFamily;
    return ( NO_ERR );
}

uint8_t getControllerChip( uint16_t *ctl ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *ctl = rMap.cType;
    return ( NO_ERR );
}

uint8_t getBoardId( uint16_t *bId ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *bId = rMap.boardId;
    return ( NO_ERR );
}

uint8_t getChipMemSize( uint32_t *size ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *size = rMap.memorySize;
    return ( NO_ERR );
}

uint8_t getChipNvmSize( uint32_t *size ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *size = rMap.eepromSize;
    return ( NO_ERR );
}

uint8_t getChipCpuFrequency( uint32_t *frequency ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    *frequency = clock_get_hz( clk_sys );
    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Watchdog facility. The watchdog system, once set up, will cause a reboot if the
// watchdog timer expires. It cannot be disabled easily. A simple kludge is to 
// set a huge timer value. A value of "0xffffffff" is about 49 days.
//
//----------------------------------------------------------------------------------------
uint8_t watchDogEnable( bool enable ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    if ( enable ) watchdog_enable( rMap.watchDogIntervallMillis, 1 );
    else          watchdog_enable( 0xFFFFFFFF, 1 );
    return ( NO_ERR );
}

uint8_t watchDogUpdate( ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    watchdog_update( );
    return ( NO_ERR );
}

uint8_t watchDogCausedReboot( bool *reboot ) {

    if ( ! initialized ) return ( NOT_INITIALZED_ERR );
    return ( watchdog_caused_reboot( ));
}

//----------------------------------------------------------------------------------------
// "scanI2CBus" is a utility routine that displays all devices found on an I2C 
// channel.
//
//----------------------------------------------------------------------------------------
uint8_t scanI2CBus( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_I2C );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    uint8_t rStat     = 0;
    uint8_t i2cAdr    = 0;
    uint8_t nDevices  = 0;
    uint8_t buf       = 0;

    for ( i2cAdr = 1; i2cAdr < 127; i2cAdr++ ) {

        rStat = i2cRead( rNum, i2cAdr, &buf, 1 );
        
        if ( rStat == 0 ) {

            printf( "I2C device found at i2cAdr 0x%x\n", i2cAdr );
            nDevices ++;
        }
    }

    if ( nDevices == 0 )  printf( "No I2C devices found\n" );
    else                  printf( "Scan done\n" );

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Print out the Resource Descriptor Map. 
//
//----------------------------------------------------------------------------------------
void printResourceDescMap( CdcResourceDescMap *dMap ) {

    printf( "CDC Resource Descriptor Map for: " );
    printf( "%s\n", dMap -> boardName );

    printf( "Board Type: %d\n", dMap -> boardInfo );
    printf( "Board Controller: %d\n", dMap -> boardCtrlInfo );
    printf( "Board Version: %d.%d\n", ( dMap -> boardVersion >> 8 ) & 0xff,
                                      ( dMap -> boardVersion & 0xFF ));
    
    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResourceDesc *dPtr = &dMap -> map[ i ];

        switch ( dPtr ->type ) {

            case CDC_RT_TIMER: {

                printf( "rNum: %2d, Timer: val: %d, pri: %d\n", 
                        dPtr -> resId, 
                        dPtr -> timer.timerVal,
                        dPtr -> timer.highPri );

            } break;

            case CDC_RT_ADC: {

                printf( "rNum: %2d, ADC: pin: %d, select: %d\n", 
                        dPtr -> resId, dPtr -> adc.adcPin, dPtr -> adc.adcNum );

            } break;

            case CDC_RT_GPIO: {

                printf( "rNum: %2d, GPIO: pinA: %d, pinB: %d, mode: %d\n", 
                        dPtr -> resId,
                        dPtr -> gpio.pinA, 
                        dPtr -> gpio.pinB, 
                        dPtr -> gpio.pinMode );

            } break;

            case CDC_RT_PWM: {

                printf( "rNum: %2d, PWM: pinA: %d, pinB: %d, fPwm: %d\n",
                        dPtr -> resId,
                        dPtr ->pwm.pinA, 
                        dPtr ->pwm.pinB,  
                        dPtr ->pwm.frequency );

            } break;

            case CDC_RT_UART: {

                printf( "rNum: %2d, UART: rxPin: %d, txPin: %d, baudRate: %d\n",
                    dPtr -> resId,
                    dPtr -> uart.rxPin,  
                    dPtr -> uart.txPin,  
                    dPtr -> uart.baudRate );

            } break;

            case CDC_RT_I2C: {

                printf( "rNum: %2d, I2C: sclPin: %d, sdaPin: %d, baudRate: %d, "
                        "i2cRoot: 0x2x, timeout(MS): %d\n",
                        dPtr -> resId,
                        dPtr -> i2c.sclPin, 
                        dPtr -> i2c.sdaPin, 
                        dPtr -> i2c.baudRate, 
                        dPtr -> i2c.i2cTimeoutMs );

            } break;

            case CDC_RT_CAN_BUS: {

                printf( "rNum: %2d, CAN: rxPin: %d, txPin: %d, "
                        "baudRate: %d, twoCores: %d\n",
                        dPtr -> resId,
                        dPtr -> can.rxPin, 
                        dPtr -> can.txPin, 
                        dPtr -> can.baudRate, 
                        dPtr -> can.twoCores );
            } break;

            case CDC_RT_UNDEFINED: {
                
            } break;

            default: printf( "Unknown type: %d\n", i );
        }
    }

    printf( "\n" );
} 

//----------------------------------------------------------------------------------------
// Print out the Resource Map.
//
//----------------------------------------------------------------------------------------
void printResourceMap( ) {

    printf( "CDC Resource Map for:" );
    printf( "%s\n", rMap.name );

    printf( "Options: 0x%04x\n", options );
    printf( "Debug Mask: 0x%04x\n", debugMask );

    printf( "Controller Family: %d\n", dMap.boardCtrlInfo );
    printf( "Controller Cores: %d, Mem: %d, EEPROM: %d\n", 
            rMap.cpuCores, rMap.memorySize, rMap.eepromSize );
            
    printf( "WatchDog Interval (MS): %d\n", rMap.watchDogIntervallMillis );
    printf( "ADC Ref Voltage: %d, Digit range: %d\n", 
            rMap.adcRefVoltageMillis, rMap.adcDigitRange ); 

    for ( int i = 0; i < MAX_RESOURCE_ENTRIES; i++ ) {

        CdcResource *rPtr = &rMap.map[ i ];

        switch ( rPtr ->type ) {

            case CDC_RT_TIMER: {

                printf( "rNum: %d, Timer: val: %d, pri: %d\n", 
                         rPtr -> resId, 
                         rPtr -> timer.timerVal, 
                         rPtr -> timer.timerHighPri );

            } break;

            case CDC_RT_ADC: {

                printf( "rNum: %d, ADC: pin: %d, select: %d\n", 
                        rPtr -> resId, 
                        rPtr -> adc.adcPin, 
                        rPtr -> adc.adcNum );

            } break;

            case CDC_RT_GPIO: {

                printf( "rNum: %d, GPIO: pinA: %d, pinB: %d, mode: %d\n", 
                        rPtr -> resId,
                        rPtr -> gpio.pinA, 
                        rPtr -> gpio.pinB, 
                        rPtr -> gpio.pinMode );

            } break;

            case CDC_RT_PWM: {

                uint8_t     pwmPinA;
                uint8_t     pwmPinB;
                uint32_t    frequency;
                uint        wrap;
                uint        sliceNum;
                bool        inverted;
                bool        phaseCorrect;

                printf( "rNum: %d, PWM: pinA: %d, pinB: %d, fPwm: %d, wrap: %d, "
                        "slice: %d, invert: %d, phase: %d\n",
                        rPtr -> resId,
                        rPtr ->pwm.pinA,  
                        rPtr ->pwm.pinB,  
                        rPtr ->pwm.frequency,
                        rPtr -> pwm.sliceNum, 
                        rPtr -> pwm.inverted, 
                        rPtr -> pwm.phaseCorrect );

            } break;

            case CDC_RT_UART: {

                printf( "rNum: %d, UART: rxPin: %d, txPin: %d, baudRate: %d, "
                        "dataBits: %d, parity: %d, stopBits: %d\n",
                        rPtr -> resId,
                        rPtr -> uart.rxPin,  
                        rPtr -> uart.txPin, 
                        rPtr -> uart.baudSetting,
                        rPtr -> uart.dataBits, 
                        rPtr -> uart.parityMode, 
                        rPtr -> uart.stopBits );

            } break;

            case CDC_RT_I2C: {

                printf( "rNum: %d, I2C: sclPin: %d, sdaPin: %d, baudRate: %d, "
                        "i2cRoot: 0x2x, timeout(MS): %d\n",
                        rPtr -> resId,
                        rPtr -> i2c.sclPin, 
                        rPtr -> i2c.sdaPin, 
                        rPtr -> i2c.baudRate, 
                        rPtr -> i2c.i2cAdrRoot, 
                        rPtr -> i2c.timeoutValMs );

            } break;

            case CDC_RT_CAN_BUS: {

                printf( "rNum: %d, CAN: rxPin: %d, txPin: %d, baudRate: %d, "
                        "canId: 0x4x, twoCores: %d\n",
                        rPtr -> resId,
                        rPtr -> can.canPinRx, 
                        rPtr -> can.canPinTx, 
                        rPtr -> can.baudRate, 
                        rPtr -> can.canId, 
                        rPtr -> can.twoCores );
            
            } break;

            case CDC_RT_UNDEFINED: {
            
            } break;

            default: printf( "Unknown type: %d\n", i );
        }
    }

    printf( "\n" );
}

}; // namespace CDC
