/*
  74HC595 -> L293D analog monitor (readability version)
  Wiring:
    - 74HC595 Q0 -> L293D IN1 (pin 2)
    - 74HC595 Q1 -> L293D IN2 (pin 7)
    - L293D EN1,2 (pin 1) = 5V (or PWM later)
    - L293D VCC2 (pin 8) = 5V (bench test)
    - L293D VCC1 (pin 16) = 5V
    - GND common: Arduino, 74HC595, L293D (pins 4,5,12,13)

  Analog taps:
    - L293D OUT1 (pin 3) -> A0  + 10kΩ to GND
    - L293D OUT2 (pin 6) -> A1  + 10kΩ to GND

  Serial Monitor: 115200 baud
*/

#include <Arduino.h>

// ---------------- Pin assignments (Arduino -> 74HC595) ----------------
const uint8_t PIN_SER   = 8;   // 74HC595 SER (pin 14)
const uint8_t PIN_RCLK  = 12;  // 74HC595 RCLK / LATCH (pin 12)
const uint8_t PIN_SRCLK = 4;   // 74HC595 SRCLK / SHIFT (pin 11)
const uint8_t PIN_OE    = 7;   // 74HC595 /OE  (pin 13, active LOW)

// ---------------- Analog inputs (Arduino) ----------------
const uint8_t PIN_OUT1_AIN = A0;  // from L293D OUT1 (pin 3)
const uint8_t PIN_OUT2_AIN = A1;  // from L293D OUT2 (pin 6)

// ---------------- Misc constants ----------------
const float   VREF          = 5.0;      // ADC reference (Uno default)
const uint8_t ADC_AVG_N     = 8;        // how many samples to average
const uint16_t SETTLE_MS    = 80;       // settle time after shifting
const uint16_t PAUSE_MS     = 1000;     // pause between states

// Bit positions in the 74HC595 byte (matching Q0->IN1, Q1->IN2)
const uint8_t BIT_IN1 = 0;  // Q0
const uint8_t BIT_IN2 = 1;  // Q1

// Prebuilt patterns for clarity
const uint8_t PAT_COAST  = (0 << BIT_IN1) | (0 << BIT_IN2); // 0b00000000
const uint8_t PAT_FWD    = (1 << BIT_IN1) | (0 << BIT_IN2); // 0b00000001
const uint8_t PAT_REV    = (0 << BIT_IN1) | (1 << BIT_IN2); // 0b00000010
const uint8_t PAT_BRAKE  = (1 << BIT_IN1) | (1 << BIT_IN2); // 0b00000011

// ---------------- Helpers ----------------
void shift595Write(uint8_t value) {
  digitalWrite(PIN_RCLK, LOW);
  shiftOut(PIN_SER, PIN_SRCLK, MSBFIRST, value);
  digitalWrite(PIN_RCLK, HIGH);
}

int analogReadStable(uint8_t pin, uint8_t samples = ADC_AVG_N) {
  // Throw away first sample to settle mux/sample-cap
  analogRead(pin);
  long acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += analogRead(pin);
    delayMicroseconds(150);
  }
  return (int)(acc / samples);
}

void reportState(const __FlashStringHelper* label) {
  const int r0 = analogReadStable(PIN_OUT1_AIN);
  const int r1 = analogReadStable(PIN_OUT2_AIN);

  const float v0 = r0 * (VREF / 1023.0f);
  const float v1 = r1 * (VREF / 1023.0f);

  // Digital interpretation as sanity check
  pinMode(PIN_OUT1_AIN, INPUT);
  pinMode(PIN_OUT2_AIN, INPUT);
  const int d0 = digitalRead(PIN_OUT1_AIN);
  const int d1 = digitalRead(PIN_OUT2_AIN);

  Serial.print(label);
  Serial.print(F(" | A0=")); Serial.print(v0, 2); Serial.print(F("V (D=")); Serial.print(d0); Serial.print(F(")"));
  Serial.print(F("  A1=")); Serial.print(v1, 2); Serial.print(F("V (D=")); Serial.print(d1); Serial.println(F(")"));
}

// ---------------- Arduino lifecycle ----------------
void setup() {
  pinMode(PIN_SER,   OUTPUT);
  pinMode(PIN_RCLK,  OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_OE,    OUTPUT);
  digitalWrite(PIN_OE, LOW); // enable 74HC595 outputs

  Serial.begin(115200);
  delay(200);
  Serial.println(F("74HC595 -> L293D analog monitor"));
}

void loop() {
  // COAST: IN1=0, IN2=0  -> both outputs ~0V
  shift595Write(PAT_COAST);
  delay(SETTLE_MS);
  reportState(F("COAST "));
  delay(PAUSE_MS);

  // FORWARD: IN1=1, IN2=0 -> OUT1 ~ (VCC2 - ~1.2..1.8V), OUT2 ~ 0V
  shift595Write(PAT_FWD);
  delay(SETTLE_MS);
  reportState(F("FWD   "));
  delay(PAUSE_MS);

  // REVERSE: IN1=0, IN2=1 -> OUT1 ~ 0V, OUT2 ~ (VCC2 - ~1.2..1.8V)
  shift595Write(PAT_REV);
  delay(SETTLE_MS);
  reportState(F("REV   "));
  delay(PAUSE_MS);

  // BRAKE: IN1=1, IN2=1 -> both high (no differential across motor)
  shift595Write(PAT_BRAKE);
  delay(SETTLE_MS);
  reportState(F("BRAKE "));
  delay(PAUSE_MS);
}
