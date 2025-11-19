#include <stepper_motor.h>
#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course
/*
#include "GPIO.h"
#include "interrupt.h"
#include "timer.h"
#include "lcd.h"
#include "flash.h"
#include "ADC.h"
#include "PWM.h"
*/


int main(void) {
    //uart_init(9600);

    // Initialize 74HC595 GPIO pins
    gpio_init_74hc595();

    // Small startup delay
    delay_ms(200);

    while (1) {

    	// Define delay between stepper motor state changes, to control speed (1ms):
    	int delay_time = 1;

    	// The bits going into the shift595Write(0xXX) function are setting the outputs as:
    	// M4B, M3B, M4A, M2B, M1B, M1A, M2A, M3A

    	// Counter clock wise should shift bits like:
    	// 0001 0100
    	// 0000 0110
    	// 0000 1010
    	// 0001 1000


    	// Tested function calls that give counter-clockwise movement of the motor:
    	/*
        shift595Write(0x14);
        delay_ms(delay_time);
        shift595Write(0x06);
        delay_ms(delay_time);
        shift595Write(0x0A);
        delay_ms(delay_time);
        shift595Write(0x18);
        delay_ms(delay_time);
        */

        // Tested for Clockwise (Reversed compared to CCW):
        shift595Write(0x18);
        delay_ms(delay_time);
        shift595Write(0x0A);
        delay_ms(delay_time);
        shift595Write(0x06);
        delay_ms(delay_time);
        shift595Write(0x14);
        delay_ms(delay_time);
    }

    return 0;
}

