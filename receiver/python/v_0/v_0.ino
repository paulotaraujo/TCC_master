// esp32_adc_receiver_binary.ino
// ESP-WROOM-32 (receptora)
// Envia amostras ADC em binário com frame robusto:
// [0]  0xA5
// [1]  0x5A
// [2:3]  seq   (uint16 LE)
// [4:7]  t_us  (uint32 LE, tempo da ESP32 em micros())
// [8:9]  adc   (uint16 LE)
// [10] checksum = sum(frame[0..9]) & 0xFF

#include <Arduino.h>

static const int ADC_PIN = 34;              // ADC1: 32,33,34,35,36,39
static const uint32_t BAUD = 921600;
static const uint8_t FRAME_H0 = 0xA5;
static const uint8_t FRAME_H1 = 0x5A;

static const int N_AVG = 8;                 // média simples para reduzir ruído
static const uint32_t SAMPLE_RATE_HZ = 2000; // taxa de envio ao PC

static inline uint8_t checksum8(const uint8_t *buf, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum += buf[i];
  return (uint8_t)(sum & 0xFF);
}

uint16_t readAdcAvg(int pin, int n) {
  uint32_t acc = 0;
  for (int i = 0; i < n; i++) {
    acc += analogRead(pin);
  }
  return (uint16_t)(acc / (uint32_t)n);
}

void setup() {
  Serial.begin(BAUD);
  delay(300);

  analogReadResolution(12);                 // 0..4095
  analogSetPinAttenuation(ADC_PIN, ADC_11db); // faixa ~0..3.3V
}

void loop() {
  static uint32_t next_us = 0;
  static uint16_t seq = 0;
  const uint32_t period_us = 1000000UL / SAMPLE_RATE_HZ;

  uint32_t now = micros();

  if (next_us == 0) {
    next_us = now + period_us;
  }

  if ((int32_t)(now - next_us) < 0) return;
  next_us += period_us;

  if ((int32_t)(now - next_us) > (int32_t)(4 * period_us)) {
    next_us = now + period_us;
  }

  uint16_t adc = readAdcAvg(ADC_PIN, N_AVG);
  if (adc > 4095) adc = 4095;

  uint32_t t_us = micros();

  uint8_t frame[11];
  frame[0] = FRAME_H0;
  frame[1] = FRAME_H1;
  frame[2] = (uint8_t)(seq & 0xFF);
  frame[3] = (uint8_t)((seq >> 8) & 0xFF);
  frame[4] = (uint8_t)(t_us & 0xFF);
  frame[5] = (uint8_t)((t_us >> 8) & 0xFF);
  frame[6] = (uint8_t)((t_us >> 16) & 0xFF);
  frame[7] = (uint8_t)((t_us >> 24) & 0xFF);
  frame[8] = (uint8_t)(adc & 0xFF);
  frame[9] = (uint8_t)((adc >> 8) & 0xFF);
  frame[10] = checksum8(frame, 10);

  Serial.write(frame, sizeof(frame));
  seq++;
}
