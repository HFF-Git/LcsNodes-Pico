//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Block Track
//
//----------------------------------------------------------------------------------------
// The Block Controller track power module manages the track of the block. Each 
// block is associated with a port on the node and the this object essentially 
// controls the H-Bridge. At the heart is a state machine manages the power state. 
// The power consumption is measured on a periodic base and an overload leads to
// switching the block track off.
// 
// The block can operate in two basic modes. The first mode is the DCC mode. The 
// control select pins are set to route the DCC signal from the LCS bus to the 
// H-Bridge. The second mode is the analog mode. There are two sub modes, which 
// are forward and reverse PWM setting. 
//
//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Block Track
// Copyright (C) 2020 - 2026  Helmut Fieres
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
//----------------------------------------------------------------------------------------
#include "LcsBlockController.h"
#include <math.h>

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// The Block Track Object local definitions. 
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Block controller global limits. Perhaps to move to a configurable place...
//
//----------------------------------------------------------------------------------------
const uint16_t MILLI_VOLT_PER_DIGIT                 = 5; // ??? correct value ... 
                                                         // rather a float ?
const uint16_t MILLI_VOLT_PER_AMP                   = 1500;

//----------------------------------------------------------------------------------------
// Block track power management is a state machine managing the setting of the power
// track. Maximum values for the track power start and stop sequence as well as
// limits for power overload events are defined. We also define reasonable default
// values.
//
//----------------------------------------------------------------------------------------
const uint16_t MAX_START_TIME_THRESHOLD_MILLIS      = 2000;
const uint16_t MAX_STOP_TIME_THRESHOLD_MILLIS       = 1000;
const uint16_t MAX_OVERLOAD_TIME_THRESHOLD_MILLIS   = 500;
const uint16_t MAX_OVERLOAD_EVENT_COUNT             = 10;
const uint16_t MAX_OVERLOAD_RESTART_COUNT           = 10;

const uint16_t DEF_START_TIME_THRESHOLD_MILLIS      = 1000;
const uint16_t DEF_STOP_TIME_THRESHOLD_MILLIS       = 500;
const uint16_t DEF_OVERLOAD_TIME_THRESHOLD_MILLIS   = 300;
const uint16_t DEF_OVERLOAD_EVENT_COUNT             = 10;
const uint16_t DEF_OVERLOAD_RESTART_COUNT           = 10;

//----------------------------------------------------------------------------------------
// Track state machine state definitions. See the track state machine routine for
// an explanation of the individual states.
//
//----------------------------------------------------------------------------------------
enum DccTrackState : uint8_t {

    TRACK_POWER_OFF       = 0,
    TRACK_POWER_ON        = 1,
    TRACK_POWER_OVERLOAD  = 2,
    TRACK_POWER_START1    = 3,
    TRACK_POWER_START2    = 4,
    TRACK_POWER_STOP1     = 5,
    TRACK_POWER_STOP2     = 6
};

//----------------------------------------------------------------------------------------
// Utility routine for number range checks.
//
//----------------------------------------------------------------------------------------
bool isInRangeU( uint8_t val, uint8_t lower, uint8_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

//----------------------------------------------------------------------------------------
// Conversion functions between milliAmps and digit values as report4de by the 
// analog to digital converter hardware. For a better precision, the formula uses
// 32 bit computation and stores the result back in a 16 bit quantity. 
//
//----------------------------------------------------------------------------------------
uint16_t milliAmpToDigitValue( uint16_t milliAmp, uint16_t digitsPerAmp ) {

    #if 0
    uint32_t mA = milliAmp;
    uint32_t dPA = digitsPerAmp;
    return (( uint16_t ) ( mA * dPA / 1000 ));
    #endif

    return ((uint16_t) ((((uint32_t) milliAmp ) * ((uint32_t) digitsPerAmp )) / 1000 ));
}

uint16_t digitValueToMilliAmp( uint16_t digitValue, uint16_t digitsPerAmp ) {

    #if 0
    uint32_t dV = digitValue;
    uint32_t dPA = digitsPerAmp;
    return ((uint16_t)( dV * 1000 / dPA ));
    #endif

    return ((uint16_t) ((((uint32_t) digitValue ) * 1000 ) / ((uint32_t) digitsPerAmp )));
}

}; // namespace


//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
//========================================================================================
using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Object instance section. The DccTrack constructor. Nothing to do so far.
//
//----------------------------------------------------------------------------------------
LcsBlockTrack::LcsBlockTrack( ) { }

//----------------------------------------------------------------------------------------
// "setupDccTrack" performs the setup tasks for the DCC track.  We will configure
// the hardware, the initial state machine state. There is quite a list of 
// parameters and options that can be set. 
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockTrack::setupBlockTrack( LcsBlockTrackDesc* trackDesc ) {

    if (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_SETUP )) {

        printf( "setupBlockTrack\n" );

        printf( "rNumControl: %d, rNumSense: %d, rNumUartRx: %d\n", 
        trackDesc -> rNumControl, trackDesc -> rNumSense, 0 );
    }

    if ((  trackDesc -> rNumControl == 0 ) || ( trackDesc -> rNumSense == 0 )) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_PIN_CONFIG );
    }

    if (( trackDesc -> initCurrentMilliAmp  > trackDesc -> limitCurrentMilliAmp )                   ||
        ( trackDesc -> limitCurrentMilliAmp > trackDesc -> maxCurrentMilliAmp )                     ||
        ( trackDesc -> startTimeThresholdMillis > MAX_START_TIME_THRESHOLD_MILLIS )                 ||
        ( trackDesc -> stopTimeThresholdMillis > MAX_STOP_TIME_THRESHOLD_MILLIS )                   ||
        ( trackDesc -> overloadTimeThresholdMillis > MAX_OVERLOAD_TIME_THRESHOLD_MILLIS )           ||
        ( trackDesc -> overloadEventThreshold > MAX_OVERLOAD_EVENT_COUNT )                          ||
        ( trackDesc -> overloadRestartThreshold > MAX_OVERLOAD_RESTART_COUNT )
        ) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_TRACK_CONFIG );
    }

    trackState                  = TRACK_POWER_OFF;
    flags                       = BT_F_DEFAULT_SETTING;
    options                     = trackDesc -> options;
    rNumControl                 = trackDesc -> rNumControl;
    rNumSense                   = trackDesc -> rNumSense;
    pwmFrequency                = trackDesc -> pwmFrequency;
    initialTrackMode            = trackDesc -> initialTrackMode;
    initialTrackSpeed           = trackDesc -> initialTrackSpeed;
    initCurrentMilliAmp         = trackDesc -> initCurrentMilliAmp;
    limitCurrentMilliAmp        = trackDesc -> limitCurrentMilliAmp;
    maxCurrentMilliAmp          = trackDesc -> maxCurrentMilliAmp;
    startTimeThreshold          = trackDesc -> startTimeThresholdMillis;
    stopTimeThreshold           = trackDesc -> stopTimeThresholdMillis;
    overloadTimeThreshold       = trackDesc -> overloadTimeThresholdMillis;
    overloadEventThreshold      = trackDesc -> overloadEventThreshold;
    overloadRestartThreshold    = trackDesc -> overloadRestartThreshold;

    // ??? MILLI_VOLT_PER_DIGIT is actually 4,72V / 1024 = 4,6 mV. 
    // How to make this more precise ?

    milliVoltPerAmp             = trackDesc -> milliVoltPerAmp;
    digitsPerAmp                = milliVoltPerAmp / MILLI_VOLT_PER_DIGIT;

    limitCurrentDigitValue      = milliAmpToDigitValue( initCurrentMilliAmp, 
                                                        digitsPerAmp );
    actualCurrentDigitValue     = 0;
    totalPwrSamplesTaken        = 0;
    lastPwrSamplePerSecTaken    = 0;
    pwrSamplesPerSec            = 0;

    uint8_t rStat = configurePwm( rNumControl );
    if ( rStat == LCS_OK ) rStat = configureAdc( rNumSense );
    if ( rStat == LCS_OK ) rStat = setTrackMode( initialTrackMode, initialTrackSpeed );

    if ( rStat != LCS_OK ) flags |= BT_F_CONFIG_ERROR;

    if (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_SETUP )) {

        printf( "setupBlockTrack, ret: %d\n", rStat );
    }

    return ( rStat );
}

//----------------------------------------------------------------------------------------
// "setTrackMode" sets the control output pins for the block controller H-Bridge.
// The H-Bridge has two half bridge control in puts and an enable input. The 
// setting of these three inputs are encoded into a pair of select pins with the
// following settings:
// 
//      BT_MODE_OFF         -   both select pins are set to zero. This leads to 
//                              putting the H-Bridge into a high impedance state.
//
//      BT_MODE_PWM_FWD     -   select pin 1 is set to the PWM signal, select 
//                              pin 2 is set to zero. The speed parameter specifies
//                              the duty cycle on a range from 0 to 255.
//  
//      BT_MODE_PWM_REV     -   select pin 1 is set to zero, select pin 2 is set
//                              to the PWM signal. The speed parameter specifies 
//                              the duty cycle on a range from 0 to 255.
//
//      BT_MODE_DCC         -   both select pins are set to one.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockTrack::setTrackMode( uint16_t mode, uint8_t speed ) {

    if (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_TRACK )) {

        printf( "setTrackMode: mode: %d, speed: %d\n", mode, speed );
    }

    uint8_t rStat;

    switch( mode ) {

        case BT_MODE_PWM_FWD: {

            rStat = writePwm( rNumControl, speed, 0 );
            return( rStat );

        } break;

        case BT_MODE_PWM_REV: {

            rStat = writePwm( rNumControl, 0, speed );
            return( rStat );

        } break;

        case BT_MODE_DCC: {

            rStat = writePwm( rNumControl, 255, 255 );
            return( rStat );

            } break;

        case BT_MODE_OFF: {

            rStat = writePwm( rNumControl, 0, 0 );
            return( rStat );

        } break;

        default: {

            if (( debugMask & DBG_BC_CONFIG ) && 
                ( debugMask & DBG_BC_TRACK )) {

                printf( "setTrackMode: mode: %d invalid\n", mode );
                // ??? for now ...
            }

            return( 255 ); 
        }
    }
}

//----------------------------------------------------------------------------------------
//
//
// ??? quick hack for debugging analog ...
//----------------------------------------------------------------------------------------
uint8_t LcsBlockTrack::setPwmFrequency( uint32_t frequency ) {

    if (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_TRACK )) {

        printf( "setPwmFrequency: frequency(Hz): %d\n", frequency );
    }

    if (( frequency >= 50 ) && ( frequency < 30000U )) {

        uint8_t rStat = configurePwm( rNumControl );
        if ( rStat == LCS_OK ) CDC::setPwmFrequency( rNumControl, frequency );
        return( rStat );
    }
    else return ( 255 );
} 

//----------------------------------------------------------------------------------------
// Track power is not just a matter of turning power on or off. To address all the
// requirements of the DCC standard, the track is managed by a state machine that 
// implements the start and stop sequences. They will also be executed in analog 
// mode. It is important that we do not really block the progress of the entire
// block controller, so any timing calls are handled by timestamp comparison in 
// state machine WAIT states. The track state machine routine is expected to be 
// called very often.
//
//  TRACK_POWER_START1    - this is the first state of a start sequence. When the
//                          track should be powered on, the first activity is to
//                          set the status flags and enable the power module. We 
//                          set the power module current consumption to the initial
//                          limit configured. The next state is TRACK_POWER_START2.
//
//  TRACK_POWER_START2    - we stay in this state until the threshold time has 
//                          passed. Once the threshold is reached, the current 
//                          consumption limit is set to the configured limit. 
//                          Then we move on to TRACK_POWER_ON.
//
//  TRACK_POWER_ON        - this is the state when power is on and things are 
//                          running  normal. An overload situation is set by the
//                          current  measurement routines through setting the 
//                          overload measurement routines through setting the 
//                          overload status flag. We make sure that we have seen 
//                          a couple of overloads in a row before taking action 
//                          which is to turn power off and set the state to 
//                          TRACK_POWER_OVERLOAD. Otherwise we stay in this state.
//
//  TRACK_POWER_OVERLOAD  - with power turned off, we stay in this state until 
//                          the threshold time has passed. If passed, the overload
//                          restart count is incremented and checked for its 
//                          threshold. If reached, we have tried to restart several
//                          times and failed. The track state becomes state 
//                          TRACK_POWER_STOP1, something is wrong on the track. 
//                          If not, we move on to TRACK_POWER_START1.
//
//  TRACK_POWER_STOP1     - this state initiates a shutdown sequence. We disable 
//                          the power module, set status flags and advance to the 
//                          TRACK_POWER_STOP2 state.
//
//  TRACK_POWER_STOP2     - we stay in this state until the configured threshold 
//                          has passed. Then we move on to TRACK_POWER_OFF. The 
//                          key reason for this time delay is to implement the 
//                          requirement that track turned off and perhaps switched
//                          to another mode, should be powerless for one second. 
//                          Switch track modes becomes simply a matter of stopping
//                          and then starting again.
//
//  TRACK_POWER_OFF       - the track is disabled. We just stay in this state until
//                          the state is set to a different state from outside.
//
// During the power on state, we append the actual current measurement value to a 
// circular buffer when the time interval for this kind of measurement has passed. 
// The idea is to measure the samples at a more or less constant interval rate and 
// compute the power consumption RMS value from the data in the buffer when 
// requested. In the interest of minimizing the controller load, the calculation 
// is done in digit values the result is presented in then in milliAmps.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::runTrackStateMachine( ) {

    switch ( trackState ) {

        case TRACK_POWER_START1: {

            // ??? do we need a way to check for overload during this 
            // initial phase, just like we do when ON ?

            trackTimeStamp          = CDC::getMillis( );
            flags                   |= BT_F_POWER_ON;
            flags                   &= ~BT_F_POWER_OVERLOAD;
            flags                   &= ~BT_F_MEASUREMENT_ON;
            limitCurrentDigitValue  = milliAmpToDigitValue( initCurrentMilliAmp, 
                                                            digitsPerAmp );

            setTrackMode( initialTrackMode, 0 );
            trackState = TRACK_POWER_START2;

        }  break;

        case TRACK_POWER_START2: {

            if (( CDC::getMillis( ) - trackTimeStamp ) > startTimeThreshold ) {

                highWaterMarkDigitValue = 0;
                actualCurrentDigitValue = 0;
                overloadRestartCount    = 0;
                overloadEventCount      = 0;
                flags                   |= BT_F_POWER_ON | BT_F_MEASUREMENT_ON;
                limitCurrentDigitValue  = milliAmpToDigitValue( limitCurrentMilliAmp, 
                                                                digitsPerAmp );

                trackState = TRACK_POWER_ON;
            }

        } break;

        case TRACK_POWER_ON: {

            if (( getMillis( ) - lastPwrSampleTimeStamp ) > 
                    PWR_SAMPLE_TIME_INTERVAL_MILLIS ) {

                pwrSampleBuf[ pwrSampleBufIndex % TRACK_POWER_ON ] = 
                        actualCurrentDigitValue;
                pwrSampleBufIndex ++;
                lastPwrSampleTimeStamp = CDC::getMillis( );
            }

            if (( CDC::getMillis( ) - lastPwrSamplePerSecTimeStamp ) > 1000 ) {

                pwrSamplesPerSec = 
                    totalPwrSamplesTaken - lastPwrSamplePerSecTaken;

                lastPwrSamplePerSecTaken  = totalPwrSamplesTaken;
                lastPwrSamplePerSecTimeStamp = CDC::getMillis( );
            }

            if ( flags & BT_F_POWER_OVERLOAD ) {

                overloadEventCount  ++;

                if ( overloadEventCount > overloadEventThreshold ) {

                    if (( debugMask & DBG_BC_CONFIG ) && 
                        ( debugMask &   DBG_BC_TRACK )) {

                        printf( "Overload detected: " );

                        #if 0
                        printf( "(hwm(mA): %d : limit(mA): %d )\n", 
                                digitValueToMilliAmp( highWaterMarkDigitValue, digitsPerAmp ),
                                digitValueToMilliAmp( limitCurrentDigitValue, digitsPerAmp ));
                        #else
                        printf( "(hwm(dVal): %d  : limit(dVal): %d )\n", 
                                highWaterMarkDigitValue, limitCurrentDigitValue );
                        #endif
                    }

                    trackTimeStamp  = CDC::getMillis( );
                    flags           |= BT_F_POWER_OVERLOAD;
                    flags           &= ~BT_F_POWER_ON;
                    flags           &= ~BT_F_MEASUREMENT_ON;

                    setTrackMode( BT_MODE_OFF );
                    trackState = TRACK_POWER_OVERLOAD;
                }
            }

        }  break;

        case TRACK_POWER_OVERLOAD: {

            if ( CDC::getMillis( ) - trackTimeStamp > overloadTimeThreshold ) {

                overloadRestartCount ++;

                if ( overloadRestartCount > overloadRestartThreshold ) {

                    if (( debugMask & DBG_BC_CONFIG ) && 
                        ( debugMask & DBG_BC_TRACK )) {

                        printf( "Overload restart failed, Cnt:%d\n", 
                                overloadRestartCount );
                    }

                    trackState = TRACK_POWER_STOP1;
                }
                else trackState = TRACK_POWER_START1;
            }

        }  break;

        case TRACK_POWER_STOP1: {

            trackTimeStamp  = CDC::getMillis( );
            flags           &= ~BT_F_POWER_ON;
            flags           &= ~BT_F_POWER_OVERLOAD;
            flags           &= ~BT_F_MEASUREMENT_ON;

            setTrackMode( BT_MODE_OFF );
            trackState = TRACK_POWER_STOP2;

        }  break;

        case TRACK_POWER_STOP2: {

            if ( getMillis( ) - trackTimeStamp > stopTimeThreshold ) 
                trackState = TRACK_POWER_OFF;

        } break;

        case TRACK_POWER_OFF: {

        } break;
    }
}

//----------------------------------------------------------------------------------------
// Some getter functions. Straightforward.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBlockTrack::getFlags( ) {

    return ( flags );
}

uint16_t LcsBlockTrack::getOptions( ) {

    return ( options );
}

uint32_t LcsBlockTrack::getPwrSamplesTaken( ) {

    return ( totalPwrSamplesTaken );
}

uint16_t LcsBlockTrack::getPwrSamplesPerSec( ) {

    return ( pwrSamplesPerSec );
}

bool LcsBlockTrack::isPowerOn( ) {

    return ( flags & BT_F_POWER_ON );
}

bool LcsBlockTrack::isPowerOverload( ) {

    return ( flags & BT_F_POWER_OVERLOAD );
}

//----------------------------------------------------------------------------------------
// Track power management functions. The actual state of track power is kept in
// the track status field and can be queried or set by setting the respective flag.
// Starting and stopping track power is done by setting the respective START or 
// STOP state.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::powerStart( ) {

    trackState = TRACK_POWER_START1;
}

void LcsBlockTrack::powerStop( ) {
    
    trackState = TRACK_POWER_STOP1;
}

//----------------------------------------------------------------------------------------
// Power Consumption Management. There are two key values. The first is the actual
// current consumption as measured by the ADC hardware on each ZERO DCC bit. This
// value is used to do the power overload checking. The second value is the high 
// water mark built from these measurements. This values is used for the DCC
// decoder programming logic. The high water mark will be set to zero before 
// collecting measurements. All measurement values are actually ADC digit values
// for performance reason. Only on limit setting and external data access are 
// these values converted from and to milliAmps.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBlockTrack::getLimitCurrent( ) {

    return ( limitCurrentMilliAmp );
}

uint16_t LcsBlockTrack::getActualCurrent( ) {

    return ( digitValueToMilliAmp( actualCurrentDigitValue, digitsPerAmp ));
}

uint16_t LcsBlockTrack::getInitCurrent( ) {

    return ( initCurrentMilliAmp );
}

uint16_t LcsBlockTrack::getMaxCurrent( ) {

    return ( maxCurrentMilliAmp );
}

void LcsBlockTrack::setLimitCurrent( uint16_t val ) {

    if      ( val < initCurrentMilliAmp )  val = initCurrentMilliAmp;
    else if ( val > maxCurrentMilliAmp  )  val = maxCurrentMilliAmp;

    limitCurrentMilliAmp    = val;
    limitCurrentDigitValue  = milliAmpToDigitValue( val, digitsPerAmp );
}

//----------------------------------------------------------------------------------------
// The "getRMSCurrent" function returns the power consumption based on the samples
// taken and stored in the sample buffer. The function computes the square root of
// the sum of the squares of the array elements. The result is returned in milliAmps.
// Note that our measurement is based on unsigned 16-bit quantities that come from
// the controller ADC hardware. We compute the RMS based on 16-bit unsigned integers,
// which compared to floating point computation is not really precise. However, for
// our purpose to just show a rough power consumption, the error should be not a big
// issue. We will not use RMS values for power overload detection or decoder ACK 
// detection.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBlockTrack::getRMSCurrent( ) {

    uint32_t res = 0;

    for ( uint8_t i = 0; i < PWR_SAMPLE_BUF_SIZE; i++ ) 
        res += pwrSampleBuf[ i ] * pwrSampleBuf[ i ];

    return ( digitValueToMilliAmp( sqrt( res / PWR_SAMPLE_BUF_SIZE ), digitsPerAmp ));
}

//----------------------------------------------------------------------------------------
// This function is called whenever we want to measure the power consumption. 
// Typically this routine will be called from an timer or interrupt handler.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::powerMeasurement( ) {

    if ( flags & BT_F_MEASUREMENT_ON ) {

        uint16_t adcVal;

        readAdc( rNumSense, &adcVal );

        actualCurrentDigitValue = adcVal;

        totalPwrSamplesTaken ++;

        if ( actualCurrentDigitValue > highWaterMarkDigitValue ) 
            highWaterMarkDigitValue = actualCurrentDigitValue;

        if ( actualCurrentDigitValue > limitCurrentDigitValue ) 
            flags |= BT_F_POWER_OVERLOAD;
    }
}

//----------------------------------------------------------------------------------------
// Print out the DCC Track configuration data. For debugging purposes.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::printTrackConfig( ) {

    printf( "Track Config: \n" );

    printf( "Config options: ( 0x%x ) -> ", flags );
    if ( options & BT_OPT_RAILCOM ) printf( "Railcom " );
    printf( "\n" );

    printf( "rNumControl: %d, SrNumSensor: %d\n", rNumControl, rNumSense );

    printf( "Initial Block State: %d, speed: %d\n", 
            initialTrackMode, initialTrackSpeed );

    printf( "Current Initial(mA): %d Current Limit(mA): %d Current Max(mA): %d\n",
            getInitCurrent( ), getLimitCurrent( ), getMaxCurrent( ));

    printf( "milliVoltPerAmp: %d\n", milliVoltPerAmp ); 
    printf( "digitsPerAmp: %d\n", digitsPerAmp );
    printf( "Limit Digit Value: %d\n", limitCurrentDigitValue );
}

//----------------------------------------------------------------------------------------
// Print out the DCC Track status.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::printTrackStatus( ) {

    printf( ", Track Status: ( 0x%x ) -> ", flags );
    
    if ( flags & BT_F_POWER_ON         ) printf( "PowerOn " );
    if ( flags & BT_F_POWER_OVERLOAD   ) printf( "PowerOverload " );
    if ( flags & BT_F_MEASUREMENT_ON   ) printf( "PowerMeasOn " );
    if ( flags & BT_F_CONFIG_ERROR     ) printf( "ConfigError " );
    printf( "\n" );

    printf( "Total Power Samples: %d\n", totalPwrSamplesTaken );
    printf( "Power Samples per Sec: %d\n", pwrSamplesPerSec );
    printf( "Power consumption (RMS): %d\n", getRMSCurrent( ));
    printf( "\n" );
}
