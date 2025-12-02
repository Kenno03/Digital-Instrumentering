#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course

void init_servo_PA12(void);
void update_servo(uint8_t degrees, uint32_t *lastPulse);
void find_light(uint8_t *tilt_degrees, uint16_t *pan_degrees);

#endif /* SERVO_H_ */
