/*
 *  74HC595 -> L293D analog monitor (STM32F302R8, Nucleo)
 *
 *  Wiring (Nucleo Arduino headers, same as on Arduino UNO):
 *
 *  74HC595:
 *    SER   (pin 14) -> D8  (PA9)
 *    RCLK  (pin 12) -> D12 (PB14)
 *    SRCLK (pin 11) -> D4  (PB5)
 *    /OE   (pin 13) -> D7  (PA8, active LOW)
 *    VCC            -> 5V
 *    GND            -> GND
 *
 *  L293D (channel 1):
 *    OUT1 (pin 3)   -> A0 (PA0, +10k -> GND)
 *    OUT2 (pin 6)   -> A1 (PA1, +10k -> GND)
 *
 *  IMPORTANT:
 *    Ensure L293D outputs are limited so the STM32 pins never see > 3.3 V.
 *    Use a resistor divider if your motor supply / VCC2 is 5 V.
 */

#include "stm32f30x_conf.h"   // STM32 SPL config
#include "30010_io.h"         // UART / printf for this course
#include "ADC.h"              // Your existing ADC helpers
#include <stdint.h>
#include <stdio.h>
#include <stepper_motor.h>

/*------------------ Pin assignments (STM32 → 74HC595) -----------------------
 * Match Arduino wiring on the Nucleo headers:
 *   SER  -> D8  -> PA9
 *   RCLK -> D12 -> PB14
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

/*----------------------------------------------------------------------------
 * Simple blocking delay (approximate, for lab use only).
 * If you already have a timer-based delay function, feel free to swap this.
 *----------------------------------------------------------------------------*/
void delay_ms(uint32_t ms) {
    volatile uint32_t count;
    while (ms--) {
        // Inner loop count is approximate; tuned for "rough" delays only.
        count = 7000;
        while (count--) {
            __NOP();
        }
    }
}

/*----------------------------------------------------------------------------
 * Helper to set a GPIO pin HIGH or LOW (for readability).
 *----------------------------------------------------------------------------*/
static inline void gpio_set(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_SetBits(port, pin);
}

static inline void gpio_clr(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_ResetBits(port, pin);
}

/*----------------------------------------------------------------------------
 * GPIO init for 74HC595 pins.
 * All 4 pins are configured as push-pull outputs.
 *----------------------------------------------------------------------------*/
void gpio_init_74hc595(void) {
    // Enable clocks for GPIOA and GPIOB
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;

    // Init SER, RCLK, /OE on GPIOA
    GPIO_InitStruct.GPIO_Pin = PIN_SER_PIN | PIN_OE_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Init SRCLK on GPIOB
    GPIO_InitStruct.GPIO_Pin = PIN_SRCLK_PIN | PIN_RCLK_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Default levels: everything low, outputs disabled state = /OE low (enabled)
    gpio_clr(PIN_SER_PORT,  PIN_SER_PIN);
    gpio_clr(PIN_RCLK_PORT, PIN_RCLK_PIN);
    gpio_clr(PIN_SRCLK_PORT, PIN_SRCLK_PIN);

    // /OE must be LOW to enable outputs
    gpio_clr(PIN_OE_PORT, PIN_OE_PIN);
}

/*----------------------------------------------------------------------------
 * Bit-banged shiftOut equivalent: MSB-first.
 * Mirrors Arduino's:
 *   digitalWrite(RCLK, LOW);
 *   shiftOut(SER, SRCLK, MSBFIRST, value);
 *   digitalWrite(RCLK, HIGH);
 *----------------------------------------------------------------------------*/
void shift595Write(uint8_t value) {
    // Latch low while shifting
    gpio_clr(PIN_RCLK_PORT, PIN_RCLK_PIN);

    // Shift out 8 bits, MSB first
    for (int8_t i = 7; i >= 0; i--) {
        // Set SER line
        if (value & (1 << i)) {
            gpio_set(PIN_SER_PORT, PIN_SER_PIN);
        } else {
            gpio_clr(PIN_SER_PORT, PIN_SER_PIN);
        }

        // Toggle SRCLK
        gpio_set(PIN_SRCLK_PORT, PIN_SRCLK_PIN);
        __NOP(); __NOP(); __NOP();    // very short hold
        gpio_clr(PIN_SRCLK_PORT, PIN_SRCLK_PIN);
    }

    // Latch updated value to outputs
    gpio_set(PIN_RCLK_PORT, PIN_RCLK_PIN);
}





/*----------------------------------------------------------------------------
 * Stable ADC read (averaged).
 * Reuses your ADC_measure_PA(ch) helper.
 * ch = 1 → PA0 / OUT1
 * ch = 2 → PA1 / OUT2
 *----------------------------------------------------------------------------*/
uint16_t analogReadStable(uint8_t ch, uint8_t samples) {
    uint32_t acc = 0;

    // Throw away first sample to settle mux / sample cap
    (void)ADC_measure_PA(ch);

    for (uint8_t i = 0; i < samples; i++) {
        acc += ADC_measure_PA(ch);
        // You can tighten this to a µs delay if you like; ms is just simple.
        delay_ms(1);
    }

    return (uint16_t)(acc / samples);
}

/*----------------------------------------------------------------------------
 * Report current state to UART (PuTTY / Serial Monitor).
 * label: "COAST ", "FWD   ", "REV   ", "BRAKE "
 *----------------------------------------------------------------------------*/
void reportState(const char *label) {
    // Read raw ADC values
    uint16_t r0 = analogReadStable(ADC_CH_OUT1, ADC_AVG_N);
    uint16_t r1 = analogReadStable(ADC_CH_OUT2, ADC_AVG_N);

    // Convert to voltages using your helper
    float v0 = ADC_absolute_voltage(r0);
    float v1 = ADC_absolute_voltage(r1);

    // Digital interpretation as a sanity check (simple mid-scale threshold)
    int d0 = (r0 > 2048) ? 1 : 0;
    int d1 = (r1 > 2048) ? 1 : 0;

    printf("%s | A0=%.2f V (D=%d)  A1=%.2f V (D=%d)\r\n",
           label, v0, d0, v1, d1);
}

