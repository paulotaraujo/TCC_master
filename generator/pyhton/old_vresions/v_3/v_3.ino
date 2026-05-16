#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

#ifndef MCP4728_PD_NORMAL
  #define MCP4728_PD_NORMAL MCP4728_PD_MODE_NORMAL
#endif

// ==============================
// Hardware
// ==============================
#define I2C_SDA       21
#define I2C_SCL       22
#define MCP4728_ADDR  0x60

static const uint32_t SERIAL_BAUD = 921600;

// ==============================
// Protocolo legado PC -> ESP32
// [0]   0xA5
// [1:4] dt_us (uint32 LE)
// [5:6] dac0
// [7:8] dac1
// [9:10] dac2
// [11:12] dac3
// [13]  flags
// [14]  checksum = sum(buf[0:14]) & 0xFF
// [15]  0x5A
// ==============================
static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;
static const uint8_t FLAG_IDLE_AFTER = 0x01;
static const uint8_t FLAG_RESET_CLOCK = 0x02;

static const uint16_t DAC_IDLE_CODE = 2048;
static const size_t FRAME_SIZE = 16;

// Buffer
static const uint16_t SAMPLE_BUFFER_CAPACITY = 4096;
static const uint16_t START_THRESHOLD = 64;   // menor para iniciar mais rápido

Adafruit_MCP4728 dac;

struct Frame {
  uint32_t dt_us;
  uint16_t dac[4];
  uint8_t flags;
};

struct FrameRing {
  Frame items[SAMPLE_BUFFER_CAPACITY];
  volatile uint16_t head = 0;
  volatile uint16_t tail = 0;t
  volatile uint16_t count = 0;

  bool push(const Frame &s) {
    if (count >= SAMPLE_BUFFER_CAPACITY) return false;
    items[head] = s;
    head = (uint16_t)((head + 1) % SAMPLE_BUFFER_CAPACITY);
    count++;
    return true;
  }

  bool pop(Frame &out) {
    if (count == 0) return false;
    out = items[tail];
    tail = (uint16_t)((tail + 1) % SAMPLE_BUFFER_CAPACITY);
    count--;
    return true;
  }

  void clear() {
    head = 0;
    tail = 0;
    count = 0;
  }
};

FrameRing g_frameRing;

bool g_started = false;
bool g_pendingValid = false;
bool g_underflowActive = false;

Frame g_pendingFrame;

uint32_t g_applyAtUs = 0;
uint32_t g_playedFrames = 0;
uint32_t g_receivedFrames = 0;
uint32_t g_checksumErrors = 0;
uint32_t g_overflows = 0;
uint32_t g_underflows = 0;

// ==============================
// Utilidades
// ==============================
static inline uint16_t clamp12(int x) {
  if (x < 0) return 0;
  if (x > 4095) return 4095;
  return (uint16_t)x;
}

static inline uint8_t checksum8(const uint8_t *buf, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum += buf[i];
  return (uint8_t)(sum & 0xFF);
}

static void setIdleOutput() {
  dac.setChannelValue(MCP4728_CHANNEL_A, DAC_IDLE_CODE, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, DAC_IDLE_CODE, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, DAC_IDLE_CODE, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, DAC_IDLE_CODE, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
}

static void resetPlaybackState() {
  g_frameRing.clear();
  g_started = false;
  g_pendingValid = false;
  g_underflowActive = false;
  g_applyAtUs = 0;
  g_playedFrames = 0;
  g_receivedFrames = 0;
  g_checksumErrors = 0;
  g_overflows = 0;
  g_underflows = 0;
  setIdleOutput();
}

// ==============================
// Parser do protocolo legado
// ==============================
static bool tryReadOneFrame(Frame &out) {
  static uint8_t buf[FRAME_SIZE];

  while (Serial.available() >= 1) {
    int c0 = Serial.peek();
    if (c0 < 0) return false;

    if ((uint8_t)c0 != MAGIC0) {
      Serial.read();
      continue;
    }

    if (Serial.available() < (int)FRAME_SIZE) {
      return false;
    }

    size_t n = Serial.readBytes(buf, FRAME_SIZE);
    if (n != FRAME_SIZE) return false;

    if (buf[0] != MAGIC0) continue;
    if (buf[15] != MAGIC1) continue;

    uint8_t chk = checksum8(buf, 14);
    if (chk != buf[14]) {
      g_checksumErrors++;
      continue;
    }

    out.dt_us = (uint32_t)buf[1]
              | ((uint32_t)buf[2] << 8)
              | ((uint32_t)buf[3] << 16)
              | ((uint32_t)buf[4] << 24);

    out.dac[0] = clamp12((uint16_t)buf[5]  | ((uint16_t)buf[6]  << 8));
    out.dac[1] = clamp12((uint16_t)buf[7]  | ((uint16_t)buf[8]  << 8));
    out.dac[2] = clamp12((uint16_t)buf[9]  | ((uint16_t)buf[10] << 8));
    out.dac[3] = clamp12((uint16_t)buf[11] | ((uint16_t)buf[12] << 8));
    out.flags  = buf[13];

    return true;
  }

  return false;
}

static void serviceSerialInput() {
  Frame fr;

  while (tryReadOneFrame(fr)) {
    if (!g_frameRing.push(fr)) {
      g_overflows++;
      break;
    }
    g_receivedFrames++;
  }
}

// ==============================
// Reprodução
// ==============================
static inline void outputFrame(const Frame &fr) {
  // Atualiza apenas os canais recebidos, sem leituras extras
  dac.setChannelValue(MCP4728_CHANNEL_A, fr.dac[0], MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, fr.dac[1], MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, fr.dac[2], MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, fr.dac[3], MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
  g_playedFrames++;
}

static void maybeStartPlayback() {
  if (g_started) return;
  if (g_frameRing.count < START_THRESHOLD) return;

  g_started = true;
  g_underflowActive = false;
  g_applyAtUs = micros();
}

static void servicePlayback() {
  maybeStartPlayback();

  if (!g_started) return;

  if (!g_pendingValid) {
    if (g_frameRing.pop(g_pendingFrame)) {
      g_pendingValid = true;

      if (g_pendingFrame.flags & FLAG_RESET_CLOCK) {
        g_applyAtUs = micros();
      } else {
        g_applyAtUs += g_pendingFrame.dt_us;
      }

      g_underflowActive = false;
    } else {
      if (!g_underflowActive) {
        g_underflows++;
        g_underflowActive = true;
      }
      g_started = false;
      return;
    }
  }

  if ((int32_t)(micros() - g_applyAtUs) < 0) {
    return;
  }

  outputFrame(g_pendingFrame);

  bool idleAfter = (g_pendingFrame.flags & FLAG_IDLE_AFTER) != 0;
  uint32_t dt_us = g_pendingFrame.dt_us;

  g_pendingValid = false;

  if (idleAfter) {
    setIdleOutput();
    g_started = false;
    return;
  }

  // Se atrasou demais, resincroniza
  if (dt_us > 0 && (int32_t)(micros() - g_applyAtUs) > (int32_t)(4 * dt_us)) {
    g_applyAtUs = micros();
  }
}

// ==============================
// Setup / Loop
// ==============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1);
  delay(2000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    Serial.println("ERR code=MCP4728_NOT_FOUND");
    while (true) delay(1000);
  }

  setIdleOutput();
  resetPlaybackState();

  Serial.printf("READY buffer_capacity=%u start_threshold=%u\n",
                (unsigned)SAMPLE_BUFFER_CAPACITY,
                (unsigned)START_THRESHOLD);
}

void loop() {
  serviceSerialInput();
  servicePlayback();
}