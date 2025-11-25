#ifndef STEPPER_MOTOR_ANGLE_H
#define STEPPER_MOTOR_ANGLE_H

#include <stdint.h>

void moveStepperToAngle(float target_deg);

// These must be defined elsewhere in your project:
void shift595Write(uint8_t value);
void delay_ms(uint32_t ms);
void gpio_init_74hc595(void);

#endif
