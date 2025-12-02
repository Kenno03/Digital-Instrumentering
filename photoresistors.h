#ifndef PHOTORESISTORS_H_
#define PHOTORESISTORS_H_

#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course

void ADC_setup_5_photo();
uint16_t ADC1_measure_channel(uint8_t ch);

#endif /* PHOTORESISTORS_H_ */
