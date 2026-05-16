#include <Arduino.h>

// ===== CONFIG =====
static const int      ADC_PIN = 34;       // ADC1 recomendado (GPIO34/35/32/33/36/39)
static const int      FS_HZ   = 2000;     // ajuste: 2000, 5000...
static const uint32_t BAUD   = 921600;   // use 921600 (ou 460800)
// ==================

static inline void write_u16_le(uint16_t v) {
  uint8_t b[2];
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  Serial.write(b, 2);
}

static inline void write_u32_le(uint32_t v) {
  uint8_t b[4];
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
  Serial.write(b, 4);
}

void setup() {
  Serial.begin(BAUD);
  delay(200);

  analogReadResolution(12);                    // 0..4095
  analogSetPinAttenuation(ADC_PIN, ADC_11db);  // ~0..3.3V (aprox)

  // sem prints, como você pediu
}

void loop() {
  static uint32_t next_us = micros();
  const uint32_t period_us = 1000000UL / FS_HZ;

  // agenda simples (jitter bem menor sem Serial.print)
  while ((int32_t)(micros() - next_us) < 0) { /* wait */ }
  next_us += period_us;

  uint32_t t_us = micros();
  uint16_t adc  = (uint16_t)analogRead(ADC_PIN);

  // Frame: AA 55 + t_us(u32 LE) + adc(u16 LE)  => 2 + 4 + 2 = 8 bytes
  Serial.write((uint8_t)0xAA);
  Serial.write((uint8_t)0x55);
  write_u32_le(t_us);
  write_u16_le(adc);
}