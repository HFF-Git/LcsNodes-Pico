// Main Idea: move bridge control logic into thePIO state machine.

// Changes: bridge is controlled via two bits:
//
//      - 00 - short circuit outputs to ground
//      - 01 - Bridge "+"
//      - 10 - Bridge "-"
//      - 11 - Bridge "Z"
// 
// there is a NAND Gate to detect "11" and set the enable Pin to low. This way the L6205 goes high impedance.
//
// The software control of the bridge stays as is:
//
//      - 00 - bridge off ( "Z" )
//      - 01 - bridge "+"
//      - 10 - bridge "-"
//      - 11  bridge tracks DCC
//
// The short "Z" gap on entering "00" in DCC mode, will be handled by the state machine code.
//


#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "H-Bridge-Pio.pio.h"

#define NUM_INSTANCES 4  // 4 instances of the logic

const uint OUTPUT_PINS[NUM_INSTANCES][2] = {
    {2, 3}, {4, 5}, {6, 7}, {8, 9}  // Adjust as needed
};
const uint INPUT_PINS[NUM_INSTANCES][2] = {
    {10, 11}, {12, 13}, {14, 15}, {16, 17}  // Adjust as needed
};

void setup_pwm(uint gpio, uint slice, uint channel) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_set_wrap(slice, 255);  // 8-bit resolution
    pwm_set_chan_level(slice, channel, 128);  // 50% duty cycle
    pwm_set_enabled(slice, true);
}

int main() {
    stdio_init_all();

    PIO pio = pio0;
    uint sm[NUM_INSTANCES];

    for (int i = 0; i < NUM_INSTANCES; i++) {
        uint offset = pio_add_program(pio, & h_bridge_control_program);
        sm[i] = pio_claim_unused_sm(pio, true);
        
        h_bridge_control_program_init(pio, sm[i], offset, OUTPUT_PINS[i][0], OUTPUT_PINS[i][1], INPUT_PINS[i][0], INPUT_PINS[i][1]);

        uint slice = pwm_gpio_to_slice_num(OUTPUT_PINS[i][1]);  
        uint channel = pwm_gpio_to_channel(OUTPUT_PINS[i][1]);
        setup_pwm(OUTPUT_PINS[i][1], slice, channel);
    }

    // Set initial select value for each instance
    for (int i = 0; i < NUM_INSTANCES; i++) {
        pio_sm_put(pio, sm[i], 0);  // Start in Select 0
    }

    sleep_ms(2000);

    // Now cycle through select values
    while (1) {
        for (int sel = 1; sel <= 3; sel++) {
            for (int i = 0; i < NUM_INSTANCES; i++) {
                pio_sm_put(pio, sm[i], sel);  // Send new select value
            }
            sleep_ms(2000);  // Stay in each mode for 2 seconds
        }
    }

    return 0;
}