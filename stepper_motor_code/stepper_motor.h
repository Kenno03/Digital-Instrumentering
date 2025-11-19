#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include "stm32f30x_conf.h"   // STM32 SPL config
#include "30010_io.h"         // UART / printf for this course
#include "ADC.h"              // Your existing ADC helpers
#include <stdint.h>
#include <stdio.h>

/*------------------ Pin assignments (STM32 → 74HC595) -----------------------
 * Match Arduino wiring on the Nucleo headers:
 *   SER  -> D8  -> PA9
 *   RCLK -> D12 -> PA6
 *   SRCLK-> D4  -> PB5
 *   /OE  -> D7  -> PA8 (active LOW)
 */
#define PIN_SER_PORT   GPIOA
#define PIN_SER_PIN    GPIO_Pin_9   // 74HC595 SER (pin 14)  - D8

#define PIN_RCLK_PORT  GPIOB
#define PIN_RCLK_PIN   GPIO_Pin_14   // 74HC595 RCLK / LATCH  - D12

#define PIN_SRCLK_PORT GPIOB
#define PIN_SRCLK_PIN  GPIO_Pin_5   // 74HC595 SRCLK / SHIFT - D4

#define PIN_OE_PORT    GPIOA
#define PIN_OE_PIN     GPIO_Pin_8   // 74HC595 /OE (active LOW) - D7

/*------------------ ADC channels (STM32) ------------------------------------
 * Reuse your existing ADC_setup_PA() and ADC_measure_PA(ch).
 *  - A0 -> PA0 -> ADC1_IN1  (OUT1)
 *  - A1 -> PA1 -> ADC1_IN2  (OUT2)
 */
#define ADC_CH_OUT1    1   // PA0 = A0
#define ADC_CH_OUT2    2   // PA1 = A1

/*------------------ Misc constants -----------------------------------------*/
#define ADC_AVG_N      8          // samples to average (same as Arduino code)
#define SETTLE_MS      80         // settle time after changing 74HC595 outputs
#define PAUSE_MS       1000       // pause between states

/*------------------ Bit positions in the 74HC595 byte ----------------------*/
#define BIT_IN1        0   // Q0
#define BIT_IN2        1   // Q1

/*------------------ Prebuilt patterns for clarity --------------------------*/
#define PAT_COAST  ((0 << BIT_IN1) | (0 << BIT_IN2))   // 0b00000000
#define PAT_FWD    ((1 << BIT_IN1) | (0 << BIT_IN2))   // 0b00000001
#define PAT_REV    ((0 << BIT_IN1) | (1 << BIT_IN2))   // 0b00000010
#define PAT_BRAKE  ((1 << BIT_IN1) | (1 << BIT_IN2))   // 0b00000011

void delay_ms(uint32_t ms);
static inline void gpio_set(GPIO_TypeDef *port, uint16_t pin);

static inline void gpio_clr(GPIO_TypeDef *port, uint16_t pin);
void gpio_init_74hc595(void);
void shift595Write(uint8_t value);
uint16_t analogReadStable(uint8_t ch, uint8_t samples);
void reportState(const char *label);


#endif /* STEPPER_MOTOR_H_ */
