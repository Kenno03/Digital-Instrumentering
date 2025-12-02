#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course
#include "servo.h"

void init_servo_PA12(void){
	// Initialize timer 16 with settings for a PWM signal
	// Enable timer 16 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

	// Set the clock in upcounting mode and to trigger an interrupt with 50Hz
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 639; // 64 MHz / (639+1) = 100 kHz tick
	TIM_InitStructure.TIM_Period = 1999; // 2000 ticks * 10 µs = 20 ms
	TIM_TimeBaseInit(TIM16,&TIM_InitStructure); // Run the initialization of the timer with the given settings


	// Initialize the PWM settings of the timer, by initializing the output compare for the given channel
	// Enable Output compare in PMW mode
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM16, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

	// Enable the PWM output
	TIM_CtrlPWMOutputs(TIM16, ENABLE);
	TIM_Cmd(TIM16,ENABLE);

	// Set the initial TIM16 pulse, given as an input to the initialization function
	TIM_SetCompare1(TIM16, 150);

	// Initialize the PA12 GPIO pin to output a PWM signal, by using the alternate function mode
	// Enable the GPIO port clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A

	// Initialize the GPIO pin
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pin
	GPIO_StructInit(&GPIO_InitStructAll);
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF; // Alternate function
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_12; // Pin PA12
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz; // Set speed of the pin to 50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructAll);

	// Select the pin for alternate function 1:
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12,GPIO_AF_1); // TIM16_CH1
}

void update_servo(uint8_t degrees, uint32_t *lastPulse){
	uint32_t newPulse;

	// Compute a value between 100 and 200 for the pulse
	newPulse = (uint32_t)((degrees / 180.0f) * 100.0f + 100.0f);

	if (newPulse != *lastPulse){
		TIM_SetCompare1(TIM16, newPulse);
		*lastPulse = newPulse;
	}

}

void find_light(uint8_t *tilt_degrees, uint16_t *pan_degrees){

	// Measure photoresistor 1->5:
	uint16_t photo_val1 = ADC1_measure_channel(1);
	uint16_t photo_val2 = ADC1_measure_channel(2);
	uint16_t photo_val3 = ADC1_measure_channel(3);
	uint16_t photo_val4 = ADC1_measure_channel(4);
	uint16_t photo_val5 = ADC1_measure_channel(5);

	// Update tilt angle, depending on photoresistor values:
	if (photo_val1 > photo_val5 && photo_val1 > photo_val3 && *tilt_degrees < 179){
		*tilt_degrees = *tilt_degrees+1;
	}
	if (photo_val5 > photo_val1 && photo_val5 > photo_val3 && *tilt_degrees > 0){
		*tilt_degrees = *tilt_degrees-1;
	}

	// Update pan angle, depending on photoresistor values:
	if (photo_val2 > photo_val3 && photo_val2 > photo_val4 && *pan_degrees < 359){
		*pan_degrees = *pan_degrees+1;
		// Stepper motor +1
	}
	if (photo_val4 > photo_val3 && photo_val4 > photo_val2 && *pan_degrees > 0){
		*pan_degrees = *pan_degrees-1;
		// Stepper motor -1
	}
}

