//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller - Bridge Test program for PIO driven Bridge.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller
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
// The LCS block controller manage a a couple of H-Bridges. In order to save elaborate external control logic
// hardware, let the PICO PIO state machine do the work. A bridge is controlled by 4 command codes.
//
//   0 - put the bridge into a disconnected state. ( Out: 0b11 )
//   1 - put the bridge in PWM forward mode. ( Out: 0b01, first pin controlled by PWM ) 
//   2 - put the bridge in PWM forward mode. ( Out: 0b10, second pin controlled by PWM )  
//   3 - put the bridge in DCC tracking mode. ( Out: 0b00 )
//
// Changes: bridge is controlled via two bits:
//
//      - 00 - short circuit outputs to ground
//      - 01 - Bridge "+"
//      - 10 - Bridge "-"
//      - 11 - Bridge "Z"
// 
// There is a NAND Gate to detect "11" and set the enable Pin to low. This way the L6205 goes high impedance.
//
//------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "H-Bridge-Pio.pio.h"

//------------------------------------------------------------------------------------------------------------
// Globals. 
//
//------------------------------------------------------------------------------------------------------------
const int   MAX_BRIDGE_INSTANCES        = 4;
PIO         pio                         = pio0;
uint        sm[ MAX_BRIDGE_INSTANCES ]  = { 0 };

const uint OUTPUT_PINS[ MAX_BRIDGE_INSTANCES ][ 2 ] = {

    {6, 7}, {8, 9}, {18,19}, {20, 21}
};

const uint INPUT_PINS[ MAX_BRIDGE_INSTANCES ][ 2 ] = {
    
    {4, 5}, {4, 5}, {4, 5}, {4, 5}  
};

//------------------------------------------------------------------------------------------------------------
// Setup a PWM channel. ( 8-bit resolution, 50% duty cycle )
//
//------------------------------------------------------------------------------------------------------------
void setupPwm( uint gpio, uint slice, uint channel ) {

    gpio_set_function( gpio, GPIO_FUNC_PWM );
    pwm_set_wrap(slice, 255);                   
    pwm_set_chan_level(slice, channel, 128 );
    pwm_set_enabled(slice, true);
}

//------------------------------------------------------------------------------------------------------------
// Switching routines when going from PIO control to PWM control of a pin and back.
//
//------------------------------------------------------------------------------------------------------------
void switchToPwm( uint gpio ) {

    printf( "switchToPwm: %d/n", gpio );
    gpio_set_function( gpio, GPIO_FUNC_PWM );
}

void switchToPio( uint gpio ) {

    printf( "switchToPio: %d/n", gpio );
    gpio_set_function( gpio, GPIO_FUNC_PIO0 );
}

//------------------------------------------------------------------------------------------------------------
// Configure a Bridge PIO instance.
//
//------------------------------------------------------------------------------------------------------------
void setupPioProgramInstance ( int index ) {

    printf( "setupPioProgramInstance: index: %d\n", index );

    uint offset = pio_add_program( pio, & dcc_h_bridge_control_program );
    sm[ index ] = pio_claim_unused_sm( pio, true );

    dcc_h_bridge_control_program_init(  pio, 
                                    sm[ index ], 
                                    offset, 
                                    OUTPUT_PINS[ index ][ 0 ], 
                                    OUTPUT_PINS[ index ][ 1], 
                                    INPUT_PINS[ index ][ 0 ], 
                                    INPUT_PINS[ index ][ 1] 
                                );
}

//------------------------------------------------------------------------------------------------------------
// Configure a Bridge PWM instance.
//
//------------------------------------------------------------------------------------------------------------
void setupPwmChannelInstance( int index ) {

    printf( "setupPwm: index: %d, channel: 0\n", index );

    uint slice      = pwm_gpio_to_slice_num( OUTPUT_PINS[ index ][ 0 ]);  
    uint channel    = pwm_gpio_to_channel( OUTPUT_PINS[ index ][ 0 ]);
    setupPwm( OUTPUT_PINS[ index ][ 0 ], slice, channel);

    printf( "setupPwm: index: %d, channel: 1\n", index );

    slice      = pwm_gpio_to_slice_num( OUTPUT_PINS[ index ][ 1 ]);  
    channel    = pwm_gpio_to_channel( OUTPUT_PINS[ index ][ 1 ]);
    setupPwm( OUTPUT_PINS[ index ][ 1 ], slice, channel);
}

//------------------------------------------------------------------------------------------------------------
// Set up all PIO state machines.
//
//------------------------------------------------------------------------------------------------------------
void setupPio( ) {

    printf( "setupPio\n");

    for ( int i = 0; i < MAX_BRIDGE_INSTANCES; i++ ) {

        setupPioProgramInstance( i );
    }
}

//------------------------------------------------------------------------------------------------------------
// Setup all PWM instances.
//
//------------------------------------------------------------------------------------------------------------
void setupPwm( ) {

    printf( "setupPwm\n");

    for ( int i = 0; i < MAX_BRIDGE_INSTANCES; i++ ) {

        setupPwmChannelInstance( i );
    }
}

//------------------------------------------------------------------------------------------------------------
// Control the Bridge. We have to claim the PWM if needed and release it later on.
//
//------------------------------------------------------------------------------------------------------------
void setPioSelect( int index, int sel ) {

    printf( "setPioSelect: index: %d, sel: %d\n", index, sel );

    if      ( sel == 1 ) switchToPwm( OUTPUT_PINS[ index % 2 ] [ 0 ] );
    else if ( sel == 2 ) switchToPwm( OUTPUT_PINS[ index % 2 ] [ 1 ] );
       
    pio_sm_put( pio, sm[ index % 2 ], sel % 2 ); 

    if      ( sel == 1 ) switchToPio( OUTPUT_PINS[ index % 2 ] [ 0 ] );
    else if ( sel == 2 ) switchToPio( OUTPUT_PINS[ index % 2 ] [ 1 ] );
}

//------------------------------------------------------------------------------------------------------------
// Here we go...
//
//------------------------------------------------------------------------------------------------------------
int main() {

    printf( "Bridge Test Program for Bridge-via-PIO\n" );
    
    stdio_init_all( );
    setupPio( );
    setupPwm( );
   
    while ( 1 ) {

        sleep_ms(2000);

        for ( int sel = 0; sel < 3; sel++ ) { 

            for ( int i = 0; i < MAX_BRIDGE_INSTANCES; i++ ) {

                setPioSelect( i, sel );
                sleep_ms(2000);
            }
            sleep_ms( 2000 ); 
        }
    }

    return 0;
}