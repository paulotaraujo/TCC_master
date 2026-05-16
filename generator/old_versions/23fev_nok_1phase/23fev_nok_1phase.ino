#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>
#include <SD.h>

// ======================
// Hardware
// ======================
#define I2C_SDA       21
#define I2C_SCL       22
#define MCP4728_ADDR  0x60

#define ADC_PIN_V     34
#define ADC_PIN_I     35

#define SD_CS_PIN     5   // ajuste conforme seu módulo SD

Adafruit_MCP4728 dac;
File logFile;

// ======================
// Protocolo PC -> ESP
// Frame (8 bytes):
//   A5 5A dt_us(u16 LE) dacV(u16 LE) dacI(u16 LE)
// STOP (4 bytes):
//   55 AA FF 00
// ======================
static const uint8_t RX_SYNC1 = 0xA5;
static const uint8_t RX_SYNC2 = 0x5A;

static const uint8_t STOP0 = 0x55;
static const uint8_t STOP1 = 0xAA;
static const uint8_t STOP2 = 0xFF;
static const uint8_t STOP3 = 0x00;

// ======================
// Buffer RX serial
// ======================
static const size_t RXBUF_CAP = 4096;
static uint8_t rxbuf[RXBUF_CAP];
static size_t rxlen = 0;

// ======================
// Armazenamento em RAM
// ======================
static const uint32_t MAX_SAMPLES = 2500;  // 2000 no seu caso, deixei margem
static uint16_t capV_mV[MAX_SAMPLES];
static uint16_t capI_mV[MAX_SAMPLES];
static uint32_t capCount = 0;

// Para detectar "fim" sem STOP:
static bool got_any_frame = false;
static uint32_t last_frame_ms = 0;
static const uint32_t IDLE_END_MS = 1200; // se ficar 1.2s sem frames, finaliza e grava

// ======================
// Utils
// ======================
static inline uint16_t clamp12u(uint16_t x) { return (x > 4095) ? 4095 : x; }

static inline void busyWaitMicros(uint32_t dt_us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < dt_us) { }
}

static inline uint16_t adcMilliVolts(int pin) {
  uint32_t mv = analogReadMilliVolts(pin);
  if (mv > 65535) mv = 65535;
  return (uint16_t)mv;
}

static void rxbuf_consume(size_t n) {
  if (n == 0) return;
  if (n >= rxlen) { rxlen = 0; return; }
  memmove(rxbuf, rxbuf + n, rxlen - n);
  rxlen -= n;
}

static void rxbuf_fill() {
  while (Serial.available() && rxlen < RXBUF_CAP) {
    int c = Serial.read();
    if (c < 0) break;
    rxbuf[rxlen++] = (uint8_t)c;
  }
}

static int find_stop() {
  if (rxlen < 4) return -1;
  for (size_t i = 0; i + 3 < rxlen; i++) {
    if (rxbuf[i] == STOP0 && rxbuf[i+1] == STOP1 &&
        rxbuf[i+2] == STOP2 && rxbuf[i+3] == STOP3) {
      return (int)i;
    }
  }
  return -1;
}

static int find_sync() {
  for (size_t i = 0; i + 1 < rxlen; i++) {
    if (rxbuf[i] == RX_SYNC1 && rxbuf[i+1] == RX_SYNC2) return (int)i;
  }
  return -1;
}

static String nextLogName() {
  for (int i = 0; i < 1000; i++) {
    char name[20];
    snprintf(name, sizeof(name), "/rx_%03d.csv", i);
    if (!SD.exists(name)) return String(name);
  }
  return String("/rx_overflow.csv");
}

static bool saveCsvToSD(const char* reasonTag) {
  String path = nextLogName();
  File f = SD.open(path.c_str(), FILE_WRITE);
  if (!f) {
    Serial.println("❌ SD: falha ao abrir CSV");
    return false;
  }

  // Cabeçalho
  f.println("sample_idx,adcV_mV,adcI_mV,adcV_V,adcI_V");

  for (uint32_t i = 0; i < capCount; i++) {
    uint16_t v = capV_mV[i];
    uint16_t a = capI_mV[i];

    f.print(i);
    f.print(',');
    f.print(v);
    f.print(',');
    f.print(a);
    f.print(',');
    f.print((float)v / 1000.0f, 6);
    f.print(',');
    f.println((float)a / 1000.0f, 6);
  }

  f.flush();
  f.close();

  Serial.print("SAVED=");
  Serial.println(path);

  if (reasonTag) {
    Serial.print("END_REASON=");
    Serial.println(reasonTag);
  }
  return true;
}

static void resetCaptureSession() {
  capCount = 0;
  got_any_frame = false;
  last_frame_ms = 0;
  rxlen = 0;
}

// ======================
// Setup / Loop
// ======================
void setup() {
  Serial.begin(921600);
  delay(250);

  Serial.println("READY_RAM_THEN_SD");

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN_V, ADC_11db);
  analogSetPinAttenuation(ADC_PIN_I, ADC_11db);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    Serial.println("❌ MCP4728 não encontrado");
    while (true) delay(1000);
  }

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("❌ Falha ao montar SD");
    while (true) delay(1000);
  }
  Serial.println("SD_OK");

  // nível inicial ~1.65V
  const uint16_t mid = 2048;
  dac.setChannelValue(MCP4728_CHANNEL_A, mid,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X,
                      MCP4728_PD_MODE_NORMAL, false);

  dac.setChannelValue(MCP4728_CHANNEL_B, mid,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X,
                      MCP4728_PD_MODE_NORMAL, true);

  Serial.print("MAX_SAMPLES=");
  Serial.println(MAX_SAMPLES);
  Serial.println("GO");
}

void loop() {
  rxbuf_fill();

  // 1) STOP explícito: finaliza e grava CSV
  int s = find_stop();
  if (s >= 0) {
    if (s > 0) rxbuf_consume((size_t)s);
    rxbuf_consume(4);

    Serial.println("STOP_RX");
    saveCsvToSD("STOP");
    Serial.println("OK_STOP");
    resetCaptureSession();
    return;
  }

  // 2) Fim por inatividade: se já recebeu algo e parou de chegar frame
  if (got_any_frame) {
    uint32_t now = millis();
    if ((uint32_t)(now - last_frame_ms) > IDLE_END_MS) {
      Serial.println("IDLE_END");
      saveCsvToSD("IDLE");
      resetCaptureSession();
      return;
    }
  }

  // 3) Processa frames
  int pos = find_sync();
  if (pos < 0) {
    if (rxlen > 64) rxbuf_consume(rxlen - 64);
    return;
  }

  if (pos > 0) rxbuf_consume((size_t)pos);
  if (rxlen < 8) return;

  uint16_t dt_us = (uint16_t)(rxbuf[2] | (rxbuf[3] << 8));
  uint16_t dacV  = (uint16_t)(rxbuf[4] | (rxbuf[5] << 8));
  uint16_t dacI  = (uint16_t)(rxbuf[6] | (rxbuf[7] << 8));
  rxbuf_consume(8);

  dacV = clamp12u(dacV);
  dacI = clamp12u(dacI);

  got_any_frame = true;
  last_frame_ms = millis();

  // temporização do frame
  if (dt_us) busyWaitMicros(dt_us);

  // aplica no MCP4728 (A=V, B=I)
  dac.setChannelValue(MCP4728_CHANNEL_A, dacV,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X,
                      MCP4728_PD_MODE_NORMAL, false);

  dac.setChannelValue(MCP4728_CHANNEL_B, dacI,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X,
                      MCP4728_PD_MODE_NORMAL, true);

  // lê ADC e guarda em RAM
  if (capCount < MAX_SAMPLES) {
    capV_mV[capCount] = adcMilliVolts(ADC_PIN_V);
    capI_mV[capCount] = adcMilliVolts(ADC_PIN_I);
    capCount++;
  } else {
    // estourou RAM de captura -> finaliza forçado e grava o que tem
    Serial.println("⚠️ MAX_SAMPLES atingido -> salvando...");
    saveCsvToSD("MAX_SAMPLES");
    resetCaptureSession();
  }
}