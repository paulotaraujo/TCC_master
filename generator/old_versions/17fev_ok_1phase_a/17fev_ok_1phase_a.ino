#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MCP4728.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

// ==============================
// Hardware
// ==============================
#define I2C_SDA       21
#define I2C_SCL       22
#define MCP4728_ADDR  0x60
#define SD_CS         5

#define ADC_PIN_A     34
#define ADC_PIN_B     35

static const char* CFG_PATH     = "/comtrade_binary/export.cfg";
static const char* BDAT_PATH    = "/comtrade_binary/export.bdat";
static const char* OUT_CSV_PATH = "/comtrade_binary/output.csv";   // ✅ pedido: output.csv

#ifndef MCP4728_PD_NORMAL
  #define MCP4728_PD_NORMAL MCP4728_PD_MODE_NORMAL
#endif

Adafruit_MCP4728 dac;

// ==============================
// Ajustes
// ==============================
// (Opcional) Decimação para debug via Serial (prints por amostra foram removidos)
static const uint32_t PLOT_DECIM = 2;

// settle após atualizar DAC
static const uint16_t DAC_SETTLE_US = 50;

// ==============================
// Utilidades
// ==============================
static inline uint16_t clamp12(int x) {
  if (x < 0) return 0;
  if (x > 4095) return 4095;
  return (uint16_t)x;
}

static inline uint16_t voltsToDac12(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 3.3f) v = 3.3f;
  int code = (int)lroundf((v / 3.3f) * 4095.0f);
  return clamp12(code);
}

static inline void busyWaitMicros(uint32_t dt_us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < dt_us) { }
}

static inline float adcRawToVolts12b(uint16_t raw) {
  if (raw > 4095) raw = 4095;
  return (raw / 4095.0f) * 3.3f;
}

static bool readLine(File &f, char* out, size_t outSize) {
  if (!f || !out || outSize < 2) return false;
  size_t n = 0;
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') break;
    if (n < outSize - 1) out[n++] = c;
  }
  out[n] = '\0';
  return (n > 0) || f.available();
}

static int splitCSV(char* line, char* parts[], int maxParts) {
  int count = 0;
  char* p = line;
  while (*p && count < maxParts) {
    parts[count++] = p;
    while (*p && *p != ',') p++;
    if (*p == ',') { *p = '\0'; p++; }
  }
  return count;
}

static int icasecmp(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (char)tolower((unsigned char)*a);
    char cb = (char)tolower((unsigned char)*b);
    if (ca != cb) return (int)((unsigned char)ca) - (int)((unsigned char)cb);
    a++; b++;
  }
  return (int)((unsigned char)tolower((unsigned char)*a)) -
         (int)((unsigned char)tolower((unsigned char)*b));
}

// ==============================
// COMTRADE
// ==============================
struct AnalogChannel {
  float a = 1.0f;
  float b = 0.0f;
  float fullScaleEng = 1.0f; // scan do BDAT

  float rawToEng(int16_t raw) const { return a * (float)raw + b; }

  float engToVolts(float eng) const {
    float fs = fullScaleEng;
    if (fs < 1e-9f) fs = 1.0f;
    float v = 1.65f + (eng / fs) * 1.65f;
    if (v < 0.0f) v = 0.0f;
    if (v > 3.3f) v = 3.3f;
    return v;
  }
};

struct ComtradeCfg {
  int nAnalog = 0;
  int nDigital = 0;

  float sampRateHz = 0.0f;
  char dataFormat[16] = {0};
  float timeMult = 1.0f;

  AnalogChannel analog[4];

  bool ts64 = false;
  uint32_t recSize = 0;
  uint32_t totalRecords = 0;
  uint32_t digitalBytes = 0;
};

// ==============================
// RAM leve (sem floats)
// 4*uint32 + 4*uint16 = 16 + 8 = 24 bytes por amostra
// ==============================
struct SampleRowLite {
  uint32_t sample;
  uint32_t ts_us;
  uint32_t dt_us;
  uint32_t t_us;
  uint16_t adcA_raw;
  uint16_t adcB_raw;
  uint16_t dacA;
  uint16_t dacB;
};

static SampleRowLite* g_rows = nullptr;
static uint32_t g_rows_count = 0;

// ==============================
// Prototipos
// ==============================
static bool parseCfgFromSD(ComtradeCfg &cfg);
static bool computeBdatLayout(ComtradeCfg &cfg);
static bool scanBdatFullScale(ComtradeCfg &cfg);
static bool allocateRows(uint32_t n);
static bool replayToRamAndPlot(ComtradeCfg &cfg);
static bool writeCsvFromRam(void);

// ==============================
// Parse CFG (mínimo)
// ==============================
static bool parseCfgFromSD(ComtradeCfg &cfg) {
  File f = SD.open(CFG_PATH, FILE_READ);
  if (!f) return false;

  char line[256];
  char* parts[32];

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // id

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // TT,NA,ND
  int n = splitCSV(line, parts, 8);
  if (n < 3) { f.close(); return false; }

  cfg.nAnalog  = atoi(parts[1]);
  cfg.nDigital = atoi(parts[2]);

  int keep = min(cfg.nAnalog, 4);
  for (int i = 0; i < keep; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    int k = splitCSV(line, parts, 24);
    if (k < 7) { f.close(); return false; }
    cfg.analog[i].a = strtof(parts[5], nullptr);
    cfg.analog[i].b = strtof(parts[6], nullptr);
    cfg.analog[i].fullScaleEng = 1.0f;
  }

  for (int i = keep; i < cfg.nAnalog; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  }

  for (int i = 0; i < cfg.nDigital; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  }

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // freq

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // nrates
  int nrates = atoi(line);

  if (nrates > 0) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    int r = splitCSV(line, parts, 8);
    if (r >= 1) cfg.sampRateHz = strtof(parts[0], nullptr);

    for (int i = 1; i < nrates; i++) {
      if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    }
  }

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // start
  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // trigger

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; } // format
  strncpy(cfg.dataFormat, line, sizeof(cfg.dataFormat)-1);

  if (readLine(f, line, sizeof(line))) {
    float tm = strtof(line, nullptr);
    if (tm > 0.0f) cfg.timeMult = tm;
  }

  f.close();
  return true;
}

// ==============================
// Layout BDAT
// ==============================
static bool computeBdatLayout(ComtradeCfg &cfg) {
  if (icasecmp(cfg.dataFormat, "BINARY") != 0) return false;

  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  uint32_t fileSize = bdat.size();
  bdat.close();

  int digitalWords = (cfg.nDigital + 15) / 16;
  cfg.digitalBytes = digitalWords * 2;

  int recSize32 = 4 + 4 + (cfg.nAnalog * 2) + (int)cfg.digitalBytes;
  cfg.ts64 = false;

  if (recSize32 <= 0 || (fileSize % recSize32) != 0) {
    int recSize64 = 4 + 8 + (cfg.nAnalog * 2) + (int)cfg.digitalBytes;
    if (recSize64 > 0 && (fileSize % recSize64) == 0) {
      cfg.ts64 = true;
      cfg.recSize = (uint32_t)recSize64;
    } else {
      return false;
    }
  } else {
    cfg.recSize = (uint32_t)recSize32;
  }

  cfg.totalRecords = fileSize / cfg.recSize;
  return (cfg.totalRecords > 0);
}

// ==============================
// Scan full-scale real
// ==============================
static bool scanBdatFullScale(ComtradeCfg &cfg) {
  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  uint8_t recBuf[128];
  if (cfg.recSize == 0 || cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }

  float maxAbs[4] = {0,0,0,0};
  int keep = min(cfg.nAnalog, 4);

  for (uint32_t i = 0; i < cfg.totalRecords; i++) {
    int rd = bdat.read(recBuf, cfg.recSize);
    if (rd != (int)cfg.recSize) { bdat.close(); return false; }

    int off = 0;
    off += 4;
    off += cfg.ts64 ? 8 : 4;

    for (int ch = 0; ch < cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;

      if (ch < keep) {
        float eng = cfg.analog[ch].rawToEng(raw);
        float ae = fabsf(eng);
        if (ae > maxAbs[ch]) maxAbs[ch] = ae;
      }
    }

    off += (int)cfg.digitalBytes;
  }

  bdat.close();

  for (int ch = 0; ch < keep; ch++) {
    float fs = maxAbs[ch];
    if (fs < 1e-6f) fs = 1.0f;
    cfg.analog[ch].fullScaleEng = fs;
  }
  return true;
}

// ==============================
// Aloca buffer dinamicamente (tenta PSRAM, senão heap)
// ==============================
static bool allocateRows(uint32_t n) {
  if (g_rows) { free(g_rows); g_rows = nullptr; }
  g_rows_count = 0;

  size_t bytes = (size_t)n * sizeof(SampleRowLite);

  // ps_malloc existe no core ESP32 Arduino (se houver PSRAM)
  g_rows = (SampleRowLite*)ps_malloc(bytes);
  if (!g_rows) {
    g_rows = (SampleRowLite*)malloc(bytes);
  }
  return (g_rows != nullptr);
}

// ==============================
// Replay + grava em RAM (sem prints por amostra)
// ==============================
static bool replayToRamAndPlot(ComtradeCfg &cfg) {
  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  uint8_t recBuf[128];
  if (cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }

  uint16_t mid = voltsToDac12(1.65f);

  // inicia DAC nos dois canais usados
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

  int64_t prevTs = 0;
  bool first = true;
  uint32_t t_accum = 0;

  g_rows_count = 0;

  for (uint32_t i = 0; i < cfg.totalRecords; i++) {
    int rd = bdat.read(recBuf, cfg.recSize);
    if (rd != (int)cfg.recSize) { bdat.close(); return false; }

    int off = 0;

    uint32_t sampleNum = 0;
    memcpy(&sampleNum, recBuf + off, 4);
    off += 4;

    int64_t ts = 0;
    if (!cfg.ts64) {
      int32_t t32 = 0;
      memcpy(&t32, recBuf + off, 4);
      off += 4;
      ts = (int64_t)t32;
    } else {
      int64_t t64 = 0;
      memcpy(&t64, recBuf + off, 8);
      off += 8;
      ts = t64;
    }

    uint32_t dt_us = 0;
    if (!first) {
      int64_t dts = ts - prevTs;
      if (dts < 0) dts = 0;
      dt_us = (uint32_t)lroundf((float)dts * cfg.timeMult);
      if (dt_us == 0 && cfg.sampRateHz > 0.1f) {
        dt_us = (uint32_t)lroundf(1000000.0f / cfg.sampRateHz);
      }
      if (dt_us > 0) busyWaitMicros(dt_us);
    } else {
      first = false;
      dt_us = 0;
    }
    prevTs = ts;
    t_accum += dt_us;

    // lê analógicos (assumindo ch0->A, ch1->B)
    float engA = 0.0f, engB = 0.0f;
    float voutA = 1.65f, voutB = 1.65f;
    uint16_t dacA = mid, dacB = mid;

    // lê todos os analógicos do arquivo, mas só usa 0 e 1
    for (int ch = 0; ch < cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;

      if (ch == 0) {
        engA = cfg.analog[0].rawToEng(raw);
        voutA = cfg.analog[0].engToVolts(engA);
        dacA = voltsToDac12(voutA);
      } else if (ch == 1) {
        engB = cfg.analog[1].rawToEng(raw);
        voutB = cfg.analog[1].engToVolts(engB);
        dacB = voltsToDac12(voutB);
      }
    }

    // pula digitais
    off += (int)cfg.digitalBytes;

    // atualiza DAC
    dac.setChannelValue(MCP4728_CHANNEL_A, dacA, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, dacB, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

    delayMicroseconds(DAC_SETTLE_US);

    // lê ADC
    uint16_t adcA_raw = (uint16_t)analogRead(ADC_PIN_A);
    uint16_t adcB_raw = (uint16_t)analogRead(ADC_PIN_B);
    // salva em RAM (leve)
    if (g_rows_count < cfg.totalRecords) {
      SampleRowLite &r = g_rows[g_rows_count++];
      r.sample   = sampleNum;
      r.ts_us    = (uint32_t)ts;
      r.dt_us    = dt_us;
      r.t_us     = t_accum;
      r.adcA_raw = adcA_raw;
      r.adcB_raw = adcB_raw;
      r.dacA     = dacA;
      r.dacB     = dacB;
    } else {
      bdat.close();
      return false;
    }
  }

  bdat.close();
  return true;
}

// ==============================
// Escreve CSV único depois (tudo num arquivo só)
// ==============================
static bool writeCsvFromRam(void) {
  SD.remove(OUT_CSV_PATH);
  File csv = SD.open(OUT_CSV_PATH, FILE_WRITE);
  if (!csv) return false;

  // ✅ CSV único com dados medidos + comandos do DAC
  csv.println("sample,ts_comtrade,dt_us,t_us,adcA_raw,adcB_raw,adcA_V,adcB_V,dacA_code,dacB_code");

  for (uint32_t i = 0; i < g_rows_count; i++) {
    const SampleRowLite &r = g_rows[i];
    float vA = adcRawToVolts12b(r.adcA_raw);
    float vB = adcRawToVolts12b(r.adcB_raw);

    csv.print(r.sample); csv.print(",");
    csv.print(r.ts_us);  csv.print(",");
    csv.print(r.dt_us);  csv.print(",");
    csv.print(r.t_us);   csv.print(",");

    csv.print(r.adcA_raw); csv.print(",");
    csv.print(r.adcB_raw); csv.print(",");

    csv.print(vA, 6); csv.print(",");
    csv.print(vB, 6); csv.print(",");

    csv.print(r.dacA); csv.print(",");
    csv.println(r.dacB);
  }

  csv.flush();
  csv.close();
  return true;
}

// ==============================
// Setup / Loop
// ==============================
ComtradeCfg g_cfg;

void setup() {
  Serial.begin(115200);
  delay(150);

  // ADC (12 bits, 0..3.3V)
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN_A, ADC_11db);
  analogSetPinAttenuation(ADC_PIN_B, ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    while (true) delay(1000);
  }

  if (!SD.begin(SD_CS)) {
    while (true) delay(1000);
  }

  if (!parseCfgFromSD(g_cfg)) {
    while (true) delay(1000);
  }

  if (!computeBdatLayout(g_cfg)) {
    while (true) delay(1000);
  }

  if (!scanBdatFullScale(g_cfg)) {
    while (true) delay(1000);
  }

  // aloca RAM do tamanho real (linhas do BDAT)
  if (!allocateRows(g_cfg.totalRecords)) {
    while (true) delay(1000);
  }

  // replay: reproduz BDAT no MCP4728 e captura ADC em RAM (sem prints por amostra)
  if (!replayToRamAndPlot(g_cfg)) {
    while (true) delay(1000);
  }

  // depois escreve o CSV único no SD (sem prints no serial monitor)
  if (!writeCsvFromRam()) {
    while (true) delay(1000);
  }


  Serial.print("✅ Arquivo de saída salvo no SD: ");
  Serial.println(OUT_CSV_PATH);
  // encerra (silêncio)
  while (true) delay(1000);
}

void loop() {
  delay(1000);
}