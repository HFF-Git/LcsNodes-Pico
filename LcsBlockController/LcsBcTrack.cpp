//----------------------------------------------------------------------------------------
//
// LCS Block Controller - Block Track
//
//----------------------------------------------------------------------------------------
// The Block Controller track module manages the track power of the block. At the
// heart is a state machine manages the power state. The power consumption is 
// measured on a periodic base and an overload leads to switching the block track off.
// 
// The block can operate in two basic modes. The first mode is the DCC mode. The 
// control select pins are set to route the DCC signal from the LCS bus to the 
// H-Bridge. The second mode is the analog mode, where the track module generates
// a PWM signal to control the track voltage. In both modes, the power consumption
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
// "debugEnabled" and "retStat" are the debug support routines. We can easily 
// check whether debug is enabled at all. The return status routine will print 
// out a return status message when debugging is enabled. The macro "RET_STAT" 
// is a nice helper that adds the function name to the message.
// 
//------------------------------------------------------------------------------------
inline bool trackDebugEnabled(  ) {

    return (( debugMask & DBG_BC_CONFIG ) && ( debugMask & DBG_BC_TRACK )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( trackDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define RET_STAT(x) retStat((char *) __func__, ( x ))

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
// Conversion functions between milliAmps and digit values as reported by the 
// analog to digital converter hardware. For a better precision, the formula uses
// 32 bit computation and stores the result back in a 16 bit quantity. 
//
//----------------------------------------------------------------------------------------
inline uint16_t milliAmpToDigitValue(uint16_t milliAmp, uint16_t digitsPerAmp) {

    if (digitsPerAmp == 0) return 0;
    return (uint16_t)((((uint32_t)milliAmp * digitsPerAmp) + 500) / 1000);
}

inline uint16_t digitValueToMilliAmp(uint16_t digitValue, uint16_t digitsPerAmp) {

    if (digitsPerAmp == 0) return 0; 
    return (uint16_t)((((uint32_t)digitValue * 1000) + (digitsPerAmp / 2)) / digitsPerAmp);
}


}; // namespace


// ??? need an interrupt handler for the PIO machines.... we have up to four machines 
// running and actually only one needs to do the job for all ... 

// ??? we need to set the relevant config data in the attributes. The ones from 
// the track HW descriptor...


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
// Object constructor.
//
//----------------------------------------------------------------------------------------
LcsBlockTrack::LcsBlockTrack( ) { 

    trackState  = TRACK_POWER_OFF;
    flags       = BT_F_DEFAULT_SETTING;

    // ??? set all fields, just to be sure ?
}

//----------------------------------------------------------------------------------------
// "getDefaultTrackDesc" returns a with reasonable defaults initialized config
// descriptor. The "setXXX" routines can modify individual fields as needed.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::getDefaultTrackDesc( LcsBlockTrackDesc *tDesc ) {

    tDesc -> options                         = BT_OPT_DEFAULT_SETTING;

    tDesc -> rNumControl                     = 0;
    tDesc -> rNumSense                       = 0;
    
    tDesc -> pwmFrequency                    = 70;

    tDesc -> initCurrentMilliAmp             = 0;
    tDesc -> limitCurrentMilliAmp            = 0;
    tDesc -> maxCurrentMilliAmp              = 0;
    tDesc -> milliVoltPerAmp                 = 0;

    tDesc -> startTimeThresholdMillis        = DEF_START_TIME_THRESHOLD_MILLIS;
    tDesc -> stopTimeThresholdMillis         = DEF_STOP_TIME_THRESHOLD_MILLIS;
    tDesc -> overloadTimeThresholdMillis     = DEF_OVERLOAD_TIME_THRESHOLD_MILLIS;
    tDesc -> overloadEventThreshold          = DEF_OVERLOAD_EVENT_COUNT;
    tDesc -> overloadRestartThreshold        = DEF_OVERLOAD_RESTART_COUNT;
}

//----------------------------------------------------------------------------------------
// Configuration setting routines.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::setStartTimeThresholdMillis( LcsBlockTrackDesc *tDesc, 
                                                    uint16_t val ) {

    tDesc -> startTimeThresholdMillis = 
        clampU16( val, 0, MAX_START_TIME_THRESHOLD_MILLIS );
}

void LcsBlockTrack::setStopTimeThresholdMillis( LcsBlockTrackDesc *tDesc, uint16_t val ) {

    tDesc -> stopTimeThresholdMillis = 
        clampU16( val, 0, MAX_STOP_TIME_THRESHOLD_MILLIS );
}

void LcsBlockTrack::setOverloadTimeThresholdMillis( LcsBlockTrackDesc *tDesc, 
                                                                uint16_t val ) {

    tDesc -> overloadTimeThresholdMillis = 
        clampU16( val, 0, MAX_OVERLOAD_TIME_THRESHOLD_MILLIS  );
}

void LcsBlockTrack::setOverloadEventThreshold( LcsBlockTrackDesc *tDesc, uint16_t val ) {

    tDesc -> overloadEventThreshold  = 
        clampU16( val, 0, MAX_OVERLOAD_EVENT_COUNT );
}

void LcsBlockTrack::setOverloadRestartThreshold( LcsBlockTrackDesc *tDesc, uint16_t val ) {

    tDesc -> overloadRestartThreshold = 
        clampU16( val, 0, MAX_OVERLOAD_RESTART_COUNT );
}

void LcsBlockTrack::setInitCurrentMilliAmp( LcsBlockTrackDesc *tDesc, uint16_t val ) {

    initCurrentMilliAmp = clampU16( val, 0, 100 ); // ??? fix ...
}

void LcsBlockTrack::setLimitCurrentMilliAmp( LcsBlockTrackDesc *tDesc, uint16_t val ) {

    limitCurrentMilliAmp = clampU16( val, 0, 100 ); // ??? fix ...
}
    
void LcsBlockTrack::setMaxCurrentMilliAmp( LcsBlockTrackDesc *tDesc, uint16_t val ) {

    maxCurrentMilliAmp = clampU16( val, 0, 100 ); // ??? fix ...
}

//----------------------------------------------------------------------------------------
// "setupBlockTrack" performs the setup tasks for the track power module.  We will
// configure the hardware, the initial state machine state. There is quite a list 
// of parameters and options. The settings passed in "tDesc" will be cross checked
// before we start the show.
//
// ??? what to store in the attributes .... !!!!!
//
// ??? the setup needs to factor in the attributes stored. The descriptor is 
// the default setting for certain values.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockTrack::setupBlockTrack( LcsBlockTrackDesc* tDesc ) {

    if ( trackDebugEnabled( )) printf( "setupBlockTrack\n" );

    if ((  tDesc -> rNumControl == 0 ) || ( tDesc -> rNumSense == 0 )) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_RNUM_CONFIG );
    }

    if (( tDesc -> initCurrentMilliAmp  > tDesc -> limitCurrentMilliAmp ) ||
        ( tDesc -> limitCurrentMilliAmp > tDesc -> maxCurrentMilliAmp )) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_TRACK_CONFIG );
    }

    if (( tDesc -> startTimeThresholdMillis > MAX_START_TIME_THRESHOLD_MILLIS )       ||
        ( tDesc -> stopTimeThresholdMillis > MAX_STOP_TIME_THRESHOLD_MILLIS )         ||
        ( tDesc -> overloadTimeThresholdMillis > MAX_OVERLOAD_TIME_THRESHOLD_MILLIS ) ||
        ( tDesc -> overloadEventThreshold > MAX_OVERLOAD_EVENT_COUNT )                ||
        ( tDesc -> overloadRestartThreshold > MAX_OVERLOAD_RESTART_COUNT )
        ) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_TRACK_CONFIG );
    }

    trackState                      = TRACK_POWER_OFF;
    flags                           = BT_F_DEFAULT_SETTING;

    options                         = tDesc -> options;
    rNumControl                     = tDesc -> rNumControl;
    rNumSense                       = tDesc -> rNumSense;
    pwmFrequency                    = tDesc -> pwmFrequency;

    initCurrentMilliAmp             = tDesc -> initCurrentMilliAmp;
    limitCurrentMilliAmp            = tDesc -> limitCurrentMilliAmp;
    maxCurrentMilliAmp              = tDesc -> maxCurrentMilliAmp;
   
    startTimeThreshold              = tDesc -> startTimeThresholdMillis;
    stopTimeThreshold               = tDesc -> stopTimeThresholdMillis;
    overloadTimeThreshold           = tDesc -> overloadTimeThresholdMillis;
    overloadEventThreshold          = tDesc -> overloadEventThreshold;
    overloadRestartThreshold        = tDesc -> overloadRestartThreshold;

    // ??? MILLI_VOLT_PER_DIGIT is actually 4,72V / 1024 = 4,6 mV. 
    // How to make this more precise ?

    milliVoltPerAmp                 = tDesc -> milliVoltPerAmp;
    digitsPerAmp                    = milliVoltPerAmp / MILLI_VOLT_PER_DIGIT;

    limitCurrentDigitValue          = milliAmpToDigitValue( initCurrentMilliAmp, 
                                                            digitsPerAmp );
    actualCurrentDigitValue         = 0;
    totalPowerSamplesTaken          = 0;
    lastPowerSamplePerSecTaken      = 0;
    powerSamplesPerSec              = 0;

    errId = configurePwm( rNumControl );

    if ( errId != LCS_OK ) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_RNUM_CONFIG );
    }

    if ( options & BT_OPT_ADC_MUX ) {

        // ??? configure Mux GPIO pins...
        // ??? for now ...
        flags = BT_F_CONFIG_ERROR;
        return ( ERR_RNUM_CONFIG );
    }

    errId = configureAdc( rNumSense );
    if ( errId != LCS_OK ) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_RNUM_CONFIG );
    }

    errId = setTrackModeSpeed( BT_MODE_OFF, 0 );
    if ( errId != LCS_OK ) {

        flags = BT_F_CONFIG_ERROR;
        return ( ERR_RNUM_CONFIG );
    }

    if ( trackDebugEnabled( )) printTrackConfig( );
    return( RET_STAT( errId ));
}

//----------------------------------------------------------------------------------------
// Getter functions. Straightforward.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBlockTrack::getFlags( ) {

    return ( flags );
}

uint8_t LcsBlockTrack::getErrId( ) {

    return ( errId );
}

uint16_t LcsBlockTrack::getOptions( ) {

    return ( options );
}

uint16_t LcsBlockTrack::getLimitCurrentMilliAmp( ) {

    return ( limitCurrentMilliAmp );
}

uint16_t LcsBlockTrack::getActualCurrentMilliAmp( ) {

    return ( digitValueToMilliAmp( actualCurrentDigitValue, digitsPerAmp ));
}

uint16_t LcsBlockTrack::getInitCurrentMilliAmp( ) {

    return ( initCurrentMilliAmp );
}

uint16_t LcsBlockTrack::getMaxCurrentMilliAmp( ) {

    return ( maxCurrentMilliAmp );
}

uint32_t LcsBlockTrack::getPowerSamplesTaken( ) {

    return ( totalPowerSamplesTaken );
}

uint16_t LcsBlockTrack::getPowerSamplesPerSec( ) {

    return ( powerSamplesPerSec );
}

bool LcsBlockTrack::isPowerOn( ) {

    return ( flags & BT_F_POWER_ON );
}

bool LcsBlockTrack::isPowerOverload( ) {

    return ( flags & BT_F_POWER_OVERLOAD );
}

uint8_t LcsBlockTrack::getTrackMode( ) {

    return( trackMode );
}

uint8_t LcsBlockTrack::getTrackSpeed( ) {

    return( trackSpeed );
}

//----------------------------------------------------------------------------------------
// This function is called whenever we want to measure the power consumption. We
// the actual value and update a high water mark if the value is larger than what
// we have seen before. When the actual value is exceeding the current limit set,
// the OVERLOAD flag is set.
//
// There are two different HW options. The first of for the dual controller, where
// we have enough ADC pins. On the quad controller hardware, we use a 4 channel
// multiplexer and always the ADC0 pin. 
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::powerMeasurement( ) {

    if ( flags & BT_F_MEASUREMENT_ON ) {

        if ( options & BT_OPT_ADC_MUX ) {

           // ??? select adc input first, need our block index...
        }
        
        uint16_t  tmp;
        readAdc( rNumSense, &tmp );
        actualCurrentDigitValue = tmp;

        totalPowerSamplesTaken ++;
        if ( actualCurrentDigitValue > limitCurrentDigitValue ) 
            flags |= BT_F_POWER_OVERLOAD;

        if ( actualCurrentDigitValue > highWaterMarkDigitValue ) 
            highWaterMarkDigitValue = actualCurrentDigitValue; 
    }
}

//----------------------------------------------------------------------------------------
// "samplePowerMeasurement" is called by the state machine to add a sample to the 
// circular sample buffer and to update the samples per second value.
// 
//----------------------------------------------------------------------------------------
void LcsBlockTrack::samplePowerMeasurement( ) {

    if (( getMillis( ) - lastPowerSampleTimeStamp ) > 
                    PWR_SAMPLE_TIME_INTERVAL_MILLIS ) {

        powerSampleBuf[ powerSampleBufIndex %  PWR_SAMPLE_BUF_SIZE ] =  
            actualCurrentDigitValue;
        powerSampleBufIndex ++;
        lastPowerSampleTimeStamp = getMillis( );
    }

    if (( getMillis( ) - lastPowerSamplePerSecTimeStamp ) > 1000 ) {

        powerSamplesPerSec = totalPowerSamplesTaken - lastPowerSamplePerSecTaken;
        lastPowerSamplePerSecTaken  = totalPowerSamplesTaken;
        lastPowerSamplePerSecTimeStamp = getMillis( );
    }
}

//----------------------------------------------------------------------------------------
// The "getRMSCurrentMilliAmp" function returns the power consumption based on the 
// samples taken and stored in the sample buffer. The function computes the square 
// root of the sum of the squares of the array elements. The result is returned in
// milliAmps.
// 
// Note that our measurement is based on unsigned 16-bit quantities that come from
// the controller ADC hardware. We compute the RMS based on 16-bit unsigned integers,
// which compared to floating point computation is not really precise. However, for
// our purpose to just show a rough power consumption, the error should be not a big
// issue. We will not use RMS values for power overload detection.
//
//----------------------------------------------------------------------------------------
uint16_t LcsBlockTrack::getRMSCurrentMilliAmp( ) {

    uint32_t res = 0;

    for ( int i = 0; i < PWR_SAMPLE_BUF_SIZE; i++ ) 
        res += powerSampleBuf[ i ] * powerSampleBuf[ i ];

    return ( digitValueToMilliAmp( sqrt( res / PWR_SAMPLE_BUF_SIZE ), digitsPerAmp ));
}

//----------------------------------------------------------------------------------------
// "syncPwm" is called by the interrupt routine to synchronize the PWM signal of
// our blocks with the CUTOUT signal, which we use as the sync point to align all
// PWM signals. The CDC layer will handle the sync operation.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::syncPwmSignals( ) {

    syncPwm( rNumControl );
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
//
//  ??? idea: the POWER_ON state can be enhanced to detect reverse loops. Also,
// we need to figure out how we detect a short circuit. The H-Bridge is very fast
// and will shut down the chip. Fine. But all we see then is a zero current 
// consumption. So, we need something like "if i have a speed set and the current
// consumption is zero after a while, perhaps it is a short circuit..." 
//
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
//                          to another mode, should be powerless for at least one
//                          second. Switch track modes becomes simply a matter of 
//                          stopping and then starting again.
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

            trackTimeStamp          = getMillis( );
            flags                   |= BT_F_POWER_ON;
            flags                   &= ~BT_F_POWER_OVERLOAD;
            flags                   &= ~BT_F_MEASUREMENT_ON;
            limitCurrentDigitValue  = 
                milliAmpToDigitValue( initCurrentMilliAmp, digitsPerAmp );

            setTrackModeSpeed( trackMode, 0 );
            trackState = TRACK_POWER_START2;

            // ??? need to enable power measurement ?

        }  break;

        case TRACK_POWER_START2: {

            if (( getMillis( ) - trackTimeStamp ) > startTimeThreshold ) {

                highWaterMarkDigitValue = 0;
                actualCurrentDigitValue = 0;
                overloadRestartCount    = 0;
                overloadEventCount      = 0;
                flags                   |= BT_F_POWER_ON | BT_F_MEASUREMENT_ON;
                limitCurrentDigitValue  = 
                    milliAmpToDigitValue( limitCurrentMilliAmp, digitsPerAmp );

                trackState = TRACK_POWER_ON;
            }

        } break;

         case TRACK_POWER_STOP1: {

            trackTimeStamp  = getMillis( );
            flags           &= ~BT_F_POWER_ON;
            flags           &= ~BT_F_POWER_OVERLOAD;
            flags           &= ~BT_F_MEASUREMENT_ON;

            setTrackModeSpeed( BT_MODE_OFF );
            trackState = TRACK_POWER_STOP2;

        }  break;

        case TRACK_POWER_STOP2: {

            if ( getMillis( ) - trackTimeStamp > stopTimeThreshold ) 
                trackState = TRACK_POWER_OFF;

        } break;

        case TRACK_POWER_ON: {

            samplePowerMeasurement( );

            if ( flags & BT_F_POWER_OVERLOAD ) {

                overloadEventCount  ++;

                if ( overloadEventCount > overloadEventThreshold ) {

                    trackTimeStamp  = getMillis( );
                    flags           |= BT_F_POWER_OVERLOAD;
                    flags           &= ~BT_F_POWER_ON;
                    flags           &= ~BT_F_MEASUREMENT_ON;

                    setTrackModeSpeed( BT_MODE_OFF );
                    trackState = TRACK_POWER_OVERLOAD;
                }
            }

        }  break;

        case TRACK_POWER_OFF: {

        } break;

        case TRACK_POWER_OVERLOAD: {

            if ( trackDebugEnabled( )) {

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

            if ( getMillis( ) - trackTimeStamp > overloadTimeThreshold ) {

                overloadRestartCount ++;

                if ( overloadRestartCount > overloadRestartThreshold ) {

                    if ( trackDebugEnabled( )) {

                        printf( "Overload restart failed, Cnt:%d\n", 
                                overloadRestartCount );
                    }

                    trackState = TRACK_POWER_STOP1;
                }
                else trackState = TRACK_POWER_START1;
            }

        }  break;
    }
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
//
// ??? will change when we have PIO...
// ??? should we sync in any case when we switch to PWM ?
//----------------------------------------------------------------------------------------
uint8_t LcsBlockTrack::setTrackModeSpeed( uint16_t mode, uint8_t speed ) {

    if ( trackDebugEnabled( )) {

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

            rStat = 255; // ??? fix ...
        }
    }

    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "setPwmFrequency" allows to set the frequency of the PWM signal. We may need
// it for different engines.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBlockTrack::setPwmFrequency( uint32_t frequency ) {

    if ( trackDebugEnabled( )) {

        printf( "setPwmFrequency: frequency(Hz): %d\n", frequency );
    }

    frequency = clampU32( frequency, 50, 30000 );

    uint8_t rStat = configurePwm( rNumControl );
    if ( rStat == LCS_OK ) CDC::setPwmFrequency( rNumControl, frequency );

    return( RET_STAT( rStat ));
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
// Print out the DCC Track configuration data. For debugging purposes.
//
//----------------------------------------------------------------------------------------
void LcsBlockTrack::printTrackConfig( ) {

    printf( "Track Config: \n" );

    printf( "Options: ( 0x%x ) -> ", flags );
    if ( options & BT_OPT_ADC_MUX ) printf( "AdcMux " );
    if ( options & BT_OPT_RAILCOM ) printf( "Railcom " );
    printf( "\n" );

    printf( "rNumControl: %d, rNumAdcMux: %d, rNumSensor: %d\n", 
            rNumControl, rNumAdcMux, rNumSense );

    printf( "Current: Initial(mA): %d, Limit(mA): %d, Max(mA): %d\n",
            getInitCurrentMilliAmp( ), 
            getLimitCurrentMilliAmp( ), 
            getMaxCurrentMilliAmp( ));

    printf( "Threshold: Start: %d, Stop: %d, ovlTime: %d, ovlEvent: %d, restart: %d\n", 
            startTimeThreshold, stopTimeThreshold,       
            overloadTimeThreshold, overloadEventThreshold, overloadRestartThreshold  );

    printf( "PWM frequency: %d\n", pwmFrequency );
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

    printf( "Track Mode: %d\n", trackMode );
    printf( "Track Speed: %d\n", trackSpeed );
    printf( "Total Power Samples: %d\n", totalPowerSamplesTaken );
    printf( "Power Samples per Sec: %d\n", powerSamplesPerSec );
    printf( "Power consumption (RMS): %d\n", getRMSCurrentMilliAmp( ));
    printf( "\n" );
}
