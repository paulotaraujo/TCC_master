#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

#define I2C_SDA      21
#define I2C_SCL      22
#define MCP4728_ADDR 0x60
#define SERIAL_BAUD  921600
#define I2C_CLOCK_HZ 400000

// ===== Protocolo serial (alinhado com o Python) =====
// [0]   0xA5
// [1:4] dt_us (uint32 LE)
// [5:6] dac0
// [7:8] dac1
// [9:10] dac2
// [11:12] dac3
// [13]  flags
// [14]  checksum = sum(buf[0:14]) & 0xFF
// [15]  0x5A
static const uint16_t DAC_IDLE_CODE = 2048;
static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;
static const uint8_t FLAG_IDLE_AFTER = 0x01;

static const size_t FRAME_SIZE = 16;
static const size_t QUEUE_LEN = 256;

// Começar só com buffer já preenchido
static const uint16_t START_THRESHOLD = 64;

Adafruit_MCP4728 dac;

struct Frame {
  uint32_t dt_us;
  uint16_t dac[4];
  uint8_t flags;
};

static Frame g_queue[QUEUE_LEN];
static volatile uint16_t g_head = 0;
static volatile uint16_t g_tail = 0;

static bool g_started = false;
static uint32_t g_apply_at_us = 0;

static inline uint16_t clamp12(uint16_t v) {
  return (v > 4095u) ? 4095u : v;
}

static inline uint16_t qNext(uint16_t idx) {
  return (uint16_t)((idx + 1u) % QUEUE_LEN);
}

static inline bool qIsEmpty() {
  return g_head == g_tail;
}

static inline bool qIsFull() {
  return qNext(g_head) == g_tail;
}

static inline uint16_t qCount() {
  if (g_head >= g_tail) {
    return (uint16_t)(g_head - g_tail);
  }
  return (uint16_t)(QUEUE_LEN - g_tail + g_head);
}

static bool qPush(const Frame &fr) {
  if (qIsFull()) return false;
  g_queue[g_head] = fr;
  g_head = qNext(g_head);
  return true;
}

static bool qPop(Frame &fr) {
  if (qIsEmpty()) return false;
  fr = g_queue[g_tail];
  g_tail = qNext(g_tail);
  return true;
}

static uint8_t checksum8(const uint8_t *buf, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; ++i) {
    sum += buf[i];
  }
  return (uint8_t)(sum & 0xFF);
}

static void writeDAC4(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
  dac.setChannelValue(MCP4728_CHANNEL_A, clamp12(a), MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  dac.setChannelValue(MCP4728_CHANNEL_B, clamp12(b), MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  dac.setChannelValue(MCP4728_CHANNEL_C, clamp12(c), MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  dac.setChannelValue(MCP4728_CHANNEL_D, clamp12(d), MCP4728_VREF_VDD, MCP4728_GAIN_1X);
}

static inline void writeIdle() {
  writeDAC4(DAC_IDLE_CODE, DAC_IDLE_CODE, DAC_IDLE_CODE, DAC_IDLE_CODE);
}

static bool tryReadOneFrame(Frame &out) {
  static uint8_t buf[FRAME_SIZE];

  while (Serial.available() >= 1) {
    int c0 = Serial.peek();
    if (c0 < 0) return false;

    if ((uint8_t)c0 != MAGIC0) {
      Serial.read();
      continue;
    }

    if (Serial.available() < (int)FRAME_SIZE) return false;
    Serial.readBytes(buf, FRAME_SIZE);

    if (buf[0] != MAGIC0) continue;
    if (buf[15] != MAGIC1) continue;
    if (checksum8(buf, 14) != buf[14]) continue;

    out.dt_us = (uint32_t)buf[1]
              | ((uint32_t)buf[2] << 8)
              | ((uint32_t)buf[3] << 16)
              | ((uint32_t)buf[4] << 24);

    out.dac[0] = (uint16_t)buf[5]  | ((uint16_t)buf[6]  << 8);
    out.dac[1] = (uint16_t)buf[7]  | ((uint16_t)buf[8]  << 8);
    out.dac[2] = (uint16_t)buf[9]  | ((uint16_t)buf[10] << 8);
    out.dac[3] = (uint16_t)buf[11] | ((uint16_t)buf[12] << 8);
    out.flags  = buf[13];
    return true;
  }

  return false;
}

static void pumpSerialToQueue() {
  Frame fr;
  while (!qIsFull() && tryReadOneFrame(fr)) {
    qPush(fr);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1);
  delay(2000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    Serial.println("DAC_FAIL");
    while (true) {
      delay(1000);
    }
  }

  writeIdle();
  Serial.println("READY");
}

void loop() {
  pumpSerialToQueue();

  // Espera prebuffer antes de começar a tocar
  if (!g_started) {
    if (qCount() >= START_THRESHOLD) {
      g_apply_at_us = micros();
      g_started = true;
    } else {
      delayMicroseconds(50);
      return;
    }
  }

  // Se houve underflow, pausa e espera reencher
  if (qIsEmpty()) {
    g_started = false;
    delayMicroseconds(50);
    return;
  }

  int32_t due = (int32_t)(micros() - g_apply_at_us);
  if (due < 0) {
    return;
  }

  Frame fr;
  if (!qPop(fr)) {
    g_started = false;
    delayMicroseconds(50);
    return;
  }

  writeDAC4(fr.dac[0], fr.dac[1], fr.dac[2], fr.dac[3]);

  if (fr.flags & FLAG_IDLE_AFTER) {
    writeIdle();
    g_started = false;
    return;
  }

  g_apply_at_us += fr.dt_us;

  // Se atrasou demais, resincroniza com o tempo atual
  if ((int32_t)(micros() - g_apply_at_us) > (int32_t)(4 * fr.dt_us)) {
    g_apply_at_us = micros();
  }
}