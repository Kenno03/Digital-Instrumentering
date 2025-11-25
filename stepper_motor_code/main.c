#include <stepper_motor.h>
#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course
#include "stepper_motor.h"

// This function assumes these requries: void shift595Write(uint8_t value); void delay_ms(uint32_t ms);
#include <stdint.h>

#define STEPS_PER_REV  200      // Geuss, needs change
#define STEP_DELAY_MS  5        // delay between steps (Needs tuning)



void moveStepperToAngle(float target_deg)
{
    // Position of motor
    int32_t current_step = 0;

    // Lasted used 4-bit thing
    uint8_t seq_index = 0;

    // Step sequence for CLOCKWISE rotation
    const uint8_t seq[4] = {
        0x18,
        0x0A,
        0x06,
        0x14
    };

    // Normalize target angle to [0, 360)
    while (target_deg < 0.0f)   target_deg += 360.0f;
    while (target_deg >= 360.0f) target_deg -= 360.0f;

    // Convert angle to target step index
    float steps_f = (target_deg / 360.0f) * (float)STEPS_PER_REV;
    int32_t target_step = (int32_t)(steps_f + 0.5f);  // round to nearest int

    // Normalize current_step to [0, STEPS_PER_REV)
    int32_t current_norm = current_step % STEPS_PER_REV;
    if (current_norm < 0) {
        current_norm += STEPS_PER_REV;
    }

    // Shortest step difference
    int32_t delta = target_step - current_norm;
    //Step logic
    if (delta > (STEPS_PER_REV / 2)) {
        delta -= STEPS_PER_REV;
    } else if (delta < -(STEPS_PER_REV / 2)) {
        delta += STEPS_PER_REV;
    }

    int32_t steps_to_move = (delta >= 0) ? delta : -delta;

    // Step the motor CW or CCW
    for (int32_t i = 0; i < steps_to_move; i++) {

        if (delta >= 0) {
            // CLOCKWISE: go forward
            seq_index = (seq_index + 1) & 0x03;   // mod 4 cause 4 stages.
            current_step++;
        } else {
            // COUNTER-CLOCKWISE: go backward
            seq_index = (seq_index + 3) & 0x03;   // (index-1) mod 4
            current_step--;
        }

        // Output to shift register to motor driver
        shift595Write(seq[seq_index]);

        // Small delay
        delay_ms(STEP_DELAY_MS);
    }

}


int main(void) {
    //uart_init(9600);

    // Initialize 74HC595 GPIO pins
    gpio_init_74hc595();

    // Small startup delay
    delay_ms(200);

    while (1) {
    // Motor go woop woop
    	moveStepperToAngle(-90);
    	delay_ms(100);

    }

    return 0;
}
