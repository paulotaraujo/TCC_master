// =============================================================================
// v_5.ino — ESP32 DAC Playback (refatorado)
//
// Melhorias aplicadas:
//   1. fastWrite() — todos os 4 canais em uma única transação I2C
//   2. I2C clock 1 MHz (fast-mode plus)
//   3. Dual-core: Core 0 = RX serial / Core 1 = playback DAC
//   4. Resincronização suave (avança metade do atraso acumulado)
//   5. Mutex protegendo o FrameRing entre os dois cores
//   6. Protocolo de flow-control: ESP32 reporta occupancy periodicamente
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#ifndef MCP4728_PD_NORMAL
  #define MCP4728_PD_NORMAL MCP4728_PD_MODE_NORMAL
#endif

// ==============================
// Hardware
// ==============================
#define I2C_SDA       21
#define I2C_SCL       22
#define MCP4728_ADDR  0x60

// I2C fast-mode plus: 1 MHz (estratégia 2)
#define I2C_FREQ_HZ   1000000UL

static const uint32_t SERIAL_BAUD = 921600;

// ==============================
// Protocolo PC -> ESP32
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
static const uint8_t MAGIC0        = 0xA5;
static const uint8_t MAGIC1        = 0x5A;
static const uint8_t FLAG_IDLE_AFTER   = 0x01;
static const uint8_t FLAG_RESET_CLOCK  = 0x02;

static const uint16_t DAC_IDLE_CODE = 2048;
static const size_t   FRAME_SIZE    = 16;

// ==============================
// Buffer
// ==============================
static const uint16_t SAMPLE_BUFFER_CAPACITY = 4096;
static const uint16_t START_THRESHOLD        = 64;

// Intervalo para reportar occupancy ao PC (flow-control, estratégia 3/6)
static const uint32_t REPORT_INTERVAL_MS = 50;

Adafruit_MCP4728 dac;

struct Frame {
  uint32_t dt_us;
  uint16_t dac[4];
  uint8_t  flags;
};

// ==============================
// FrameRing — acesso protegido por mutex (estratégia 3/4)
// ==============================
struct FrameRing {
  Frame    items[SAMPLE_BUFFER_CAPACITY];
  uint16_t head  = 0;
  uint16_t tail  = 0;
  uint16_t count = 0;

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

  void clear() { head = 0; tail = 0; count = 0; }
};

static FrameRing      g_frameRing;
static SemaphoreHandle_t g_ringMutex = nullptr;  // mutex dual-core (estratégia 4)

// ==============================
// Estado de playback (Core 1)
// ==============================
static volatile bool     g_started         = false;
static volatile bool     g_pendingValid     = false;
static volatile bool     g_underflowActive  = false;

static Frame    g_pendingFrame;
static uint32_t g_applyAtUs    = 0;

// ==============================
// Estatísticas
// ==============================
static volatile uint32_t g_playedFrames   = 0;
static volatile uint32_t g_receivedFrames = 0;
static volatile uint32_t g_checksumErrors = 0;
static volatile uint32_t g_overflows      = 0;
static volatile uint32_t g_underflows     = 0;

// ==============================
// Handles das tasks
// ==============================
static TaskHandle_t g_rxTaskHandle  = nullptr;
static TaskHandle_t g_dacTaskHandle = nullptr;

// ==============================
// Utilitários
// ==============================
static inline uint16_t clamp12(int x) {
  if (x < 0)    return 0;
  if (x > 4095) return 4095;
  return (uint16_t)x;
}

static inline uint8_t checksum8(const uint8_t *buf, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum += buf[i];
  return (uint8_t)(sum & 0xFF);
}

static void setIdleOutput() {
  // fastWrite mantém os 4 canais em midscale
  dac.fastWrite(DAC_IDLE_CODE, DAC_IDLE_CODE, DAC_IDLE_CODE, DAC_IDLE_CODE);
}

static void resetPlaybackState() {
  if (xSemaphoreTake(g_ringMutex, portMAX_DELAY)) {
    g_frameRing.clear();
    xSemaphoreGive(g_ringMutex);
  }
  g_started        = false;
  g_pendingValid   = false;
  g_underflowActive = false;
  g_applyAtUs      = 0;
  g_playedFrames   = 0;
  g_receivedFrames = 0;
  g_checksumErrors = 0;
  g_overflows      = 0;
  g_underflows     = 0;
  setIdleOutput();
}

// ==============================
// Parser do protocolo
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

    if (Serial.available() < (int)FRAME_SIZE) return false;

    size_t n = Serial.readBytes(buf, FRAME_SIZE);
    if (n != FRAME_SIZE) return false;

    if (buf[0] != MAGIC0 || buf[15] != MAGIC1) continue;

    uint8_t chk = checksum8(buf, 14);
    if (chk != buf[14]) { g_checksumErrors++; continue; }

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

// ==============================
// Task Core 0 — RX serial + flow-control report
// (estratégia 3 e 4: core dedicado + relatório de occupancy)
// ==============================
static void rxTask(void *) {
  uint32_t lastReportMs = 0;

  for (;;) {
    // Drena todos os frames disponíveis na serial
    Frame fr;
    while (tryReadOneFrame(fr)) {
      if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(2))) {
        if (!g_frameRing.push(fr)) {
          g_overflows++;
        } else {
          g_receivedFrames++;
        }
        xSemaphoreGive(g_ringMutex);
      }
    }

    // Flow-control: reporta occupancy ao PC periodicamente (estratégia 6)
    uint32_t now = millis();
    if (now - lastReportMs >= REPORT_INTERVAL_MS) {
      lastReportMs = now;
      uint16_t cnt = 0;
      if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(1))) {
        cnt = g_frameRing.count;
        xSemaphoreGive(g_ringMutex);
      }
      // Formato compacto para não poluir a serial com texto longo
      Serial.printf("OCC %u/%u played=%lu rx=%lu uf=%lu of=%lu cke=%lu\n",
        (unsigned)cnt,
        (unsigned)SAMPLE_BUFFER_CAPACITY,
        (unsigned long)g_playedFrames,
        (unsigned long)g_receivedFrames,
        (unsigned long)g_underflows,
        (unsigned long)g_overflows,
        (unsigned long)g_checksumErrors
      );
    }

    // Yield breve para não starvar o Core 0 de outras tarefas de sistema
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ==============================
// Task Core 1 — Playback DAC
// (estratégia 1, 4 e 5)
// ==============================

static inline void outputFrame(const Frame &fr) {
  // Estratégia 1: fastWrite() — 4 canais em uma única transação I2C
  dac.fastWrite(fr.dac[0], fr.dac[1], fr.dac[2], fr.dac[3]);
  g_playedFrames++;
}

static void maybeStartPlayback() {
  if (g_started) return;
  uint16_t cnt = 0;
  if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(1))) {
    cnt = g_frameRing.count;
    xSemaphoreGive(g_ringMutex);
  }
  if (cnt < START_THRESHOLD) return;
  g_started        = true;
  g_underflowActive = false;
  g_applyAtUs      = micros();
}

static void dacTask(void *) {
  for (;;) {
    maybeStartPlayback();

    if (!g_started) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // Busca próximo frame do ring
    if (!g_pendingValid) {
      Frame fr;
      bool got = false;

      if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(2))) {
        got = g_frameRing.pop(fr);
        xSemaphoreGive(g_ringMutex);
      }

      if (got) {
        g_pendingFrame = fr;
        g_pendingValid = true;
        g_underflowActive = false;

        if (g_pendingFrame.flags & FLAG_RESET_CLOCK) {
          g_applyAtUs = micros();
        } else {
          g_applyAtUs += g_pendingFrame.dt_us;
        }
      } else {
        // Underflow: ring vazio antes da hora
        if (!g_underflowActive) {
          g_underflows++;
          g_underflowActive = true;
        }
        g_started = false;
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
    }

    // Espera busy-wait de precisão enquanto o tempo não chegou
    // (não usa vTaskDelay aqui para manter jitter baixo)
    if ((int32_t)(micros() - g_applyAtUs) < 0) {
      // Pequeno yield para não bloquear o watchdog
      taskYIELD();
      continue;
    }

    // Aplica o frame
    outputFrame(g_pendingFrame);

    bool     idleAfter = (g_pendingFrame.flags & FLAG_IDLE_AFTER) != 0;
    uint32_t dt_us     = g_pendingFrame.dt_us;

    g_pendingValid = false;

    if (idleAfter) {
      setIdleOutput();
      g_started = false;
      continue;
    }

    // Estratégia 5 — Resincronização suave
    // Se atrasou mais que 4× o período, avança apenas metade do atraso
    // para não criar saltos abruptos na saída analógica
    if (dt_us > 0) {
      int32_t atraso = (int32_t)(micros() - g_applyAtUs);
      if (atraso > (int32_t)(4 * dt_us)) {
        g_applyAtUs += (uint32_t)(atraso / 2);
      }
    }
  }
}

// ==============================
// Setup / Loop
// ==============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1);
  delay(2000);

  // Estratégia 2: I2C a 1 MHz
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ_HZ);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    Serial.println("ERR code=MCP4728_NOT_FOUND");
    while (true) delay(1000);
  }

  // Mutex para o FrameRing (estratégia 4)
  g_ringMutex = xSemaphoreCreateMutex();
  if (!g_ringMutex) {
    Serial.println("ERR code=MUTEX_FAIL");
    while (true) delay(1000);
  }

  setIdleOutput();
  resetPlaybackState();

  Serial.printf("READY buffer_capacity=%u start_threshold=%u i2c_hz=%lu\n",
                (unsigned)SAMPLE_BUFFER_CAPACITY,
                (unsigned)START_THRESHOLD,
                (unsigned long)I2C_FREQ_HZ);

  // Task Core 0 — RX serial (estratégia 4)
  xTaskCreatePinnedToCore(
    rxTask,
    "RX",
    4096,
    nullptr,
    2,              // prioridade maior que loop() default
    &g_rxTaskHandle,
    0               // Core 0
  );

  // Task Core 1 — DAC playback (estratégia 4)
  xTaskCreatePinnedToCore(
    dacTask,
    "DAC",
    4096,
    nullptr,
    3,              // prioridade mais alta: deadline crítica
    &g_dacTaskHandle,
    1               // Core 1
  );
}

void loop() {
  // O trabalho real está nas tasks.
  // loop() fica em idle para não interferir.
  vTaskDelay(pdMS_TO_TICKS(100));
}