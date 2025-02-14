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

#if 0
// PIO and PWM need to share the same pin. For PWM modes, the PWM has control, else PIO

// switching can be done in PIO
// set pindirs 1   ; PIO takes control of the pin
// set pindirs 0   ; PWM takes control of the pin

// Example:

// .program my_pio_program
//.side_set 2 opt

// loop:
//    set pindirs 1 side 0b01  ; PIO takes control
//    set pins, 1
//    nop [10]
//    set pins, 0
//    nop [10]

//    set pindirs 0 side 0b00  ; PWM takes control
//    jmp loop

// or in C;
//

void switch_to_pwm() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);  // Give control to PWM
}

void switch_to_pio() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PIO0); // Give control to PIO
}



void setup_pio() {
    PIO pio = pio0;
    uint sm = 0;  // State machine 0

    uint offset = pio_add_program(pio, &my_pio_program);
    pio_sm_config c = my_pio_program_get_default_config(offset);
    
    sm_config_set_out_pins(&c, PWM_PIN, 1);  // Assign the same pin to PIO
    sm_config_set_set_pins(&c, PWM_PIN, 1);
    pio_gpio_init(pio, PWM_PIN);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif




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