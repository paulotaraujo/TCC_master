#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MCP4728.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#ifndef MCP4728_PD_NORMAL
  #define MCP4728_PD_NORMAL MCP4728_PD_MODE_NORMAL
#endif

// ==============================
// Hardware
// ==============================
#define I2C_SDA       21
#define I2C_SCL       22
#define MCP4728_ADDR  0x60
#define SD_CS         5

// Loopback ADC (se estiver medindo as saídas do MCP4728)
// ⚠️ ajuste conforme seu circuito/divisor/buffer
#define ADC_PIN_V     34
#define ADC_PIN_I     35

static const char* CFG_PATH     = "/comtrade_binary/export.cfg";
static const char* BDAT_PATH    = "/comtrade_binary/export.bdat";
static const char* OUT_CSV_PATH = "/comtrade_binary/output.csv";

Adafruit_MCP4728 dac;

// ==============================
// Reprodução / timing
// ==============================
static const uint16_t DAC_SETTLE_US = 80;   // ajuste conforme seu RC/buffer
static const float    V_MID   = 1.65f;
static const float    V_AMP   = 1.55f;      // use 1.55V (headroom) ao invés de 1.65V cravado

// ==============================
// Escala “física” (FIXA) — tensão
// ==============================
// Você disse: 188 kV é o pico nominal do sinal (valor ENG após aplicar a,b).
// Então: |V_kV| = 188 => sai em ±V_AMP em torno de 1.65V.
static const float V_NOM_PEAK_KV = 188.0f;

// ==============================
// Escala corrente
// ==============================
// Opção 1 (recomendado p/ falta): calibrar corrente pelo PRÉ-FALTA (RMS dos primeiros ciclos) e manter fixo
static const bool  I_CALIBRATE_FROM_PREFault_RMS = true;

// Opção 2: se quiser fixo também, preencha o pico nominal (A) aqui e desligue a calibração
// (Se ficar 0 e calibração OFF, cai num fallback simples)
static const float I_NOM_PEAK_A = 0.0f;

// Quantos ciclos usar para “pré-falta”
static const int   PREF_CYCLES = 10;

// Soft limiter para evitar degrau duro quando extrapolar (não “mascara” falta; só comprime topo)
static inline float softLimit(float x) {
  // tanh: suave, rápido e barato
  return tanhf(x);
}

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

static inline float adcToVoltsCal(int pin) {
  // Melhor que raw*(3.3/4095) quando usa atenuação
  uint32_t mv = analogReadMilliVolts(pin);
  return (float)mv / 1000.0f;
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
// COMTRADE (mínimo necessário)
// ==============================
struct AnalogCh {
  float a = 1.0f;
  float b = 0.0f;

  inline float rawToEng(int16_t raw) const { return a * (float)raw + b; }
};

struct ComtradeCfg {
  int nAnalog = 0;
  int nDigital = 0;

  float freqHz = 60.0f;     // se não vier, assume 60
  float sampRateHz = 0.0f;
  float timeMult = 1.0f;

  char dataFormat[16] = {0};

  AnalogCh chV; // analógico 0 (tensão)
  AnalogCh chI; // analógico 1 (corrente)

  bool ts64 = false;
  uint32_t recSize = 0;
  uint32_t totalRecords = 0;
  uint32_t digitalBytes = 0;
};

static bool parseCfgFromSD(ComtradeCfg &cfg);
static bool computeBdatLayout(ComtradeCfg &cfg);
static bool calibrateCurrentFromPrefaultRMS(ComtradeCfg &cfg, float &out_INomPeakA);

// ==============================
// Parse CFG (pega a,b dos 2 primeiros analógicos)
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

  // linhas de analógicos
  for (int i = 0; i < cfg.nAnalog; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    int k = splitCSV(line, parts, 32);
    if (k < 7) { f.close(); return false; }

    float a = strtof(parts[5], nullptr);
    float b = strtof(parts[6], nullptr);

    if (i == 0) { cfg.chV.a = a; cfg.chV.b = b; }
    if (i == 1) { cfg.chI.a = a; cfg.chI.b = b; }
  }

  // linhas de digitais
  for (int i = 0; i < cfg.nDigital; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  }

  // freq (linha seguinte) — em muitos CFG é só um número
  if (readLine(f, line, sizeof(line))) {
    float fr = strtof(line, nullptr);
    if (fr > 1.0f && fr < 500.0f) cfg.freqHz = fr;
  }

  // nrates
  if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  int nrates = atoi(line);

  if (nrates > 0) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    int r = splitCSV(line, parts, 8);
    if (r >= 1) cfg.sampRateHz = strtof(parts[0], nullptr);
    for (int i = 1; i < nrates; i++) {
      if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    }
  }

  // start / trigger (descarta)
  readLine(f, line, sizeof(line));
  readLine(f, line, sizeof(line));

  // format
  if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  strncpy(cfg.dataFormat, line, sizeof(cfg.dataFormat)-1);

  // timemult (opcional)
  if (readLine(f, line, sizeof(line))) {
    float tm = strtof(line, nullptr);
    if (tm > 0.0f) cfg.timeMult = tm;
  }

  f.close();
  return true;
}

// ==============================
// Layout BDAT (BINARY 32/64 timestamp)
// ==============================
static bool computeBdatLayout(ComtradeCfg &cfg) {
  if (icasecmp(cfg.dataFormat, "BINARY") != 0) return false;

  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  uint32_t fileSize = bdat.size();
  bdat.close();

  int digitalWords = (cfg.nDigital + 15) / 16;
  cfg.digitalBytes = (uint32_t)digitalWords * 2;

  int recSize32 = 4 + 4 + (cfg.nAnalog * 2) + (int)cfg.digitalBytes;
  cfg.ts64 = false;

  if (recSize32 <= 0 || (fileSize % (uint32_t)recSize32) != 0) {
    int recSize64 = 4 + 8 + (cfg.nAnalog * 2) + (int)cfg.digitalBytes;
    if (recSize64 > 0 && (fileSize % (uint32_t)recSize64) == 0) {
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
// Calibração corrente pelo PRÉ-FALTA (RMS dos primeiros ciclos)
// Retorna I_nom_peak = RMS*sqrt(2)
// ==============================
static bool calibrateCurrentFromPrefaultRMS(ComtradeCfg &cfg, float &out_INomPeakA) {
  out_INomPeakA = 0.0f;

  if (cfg.sampRateHz < 1.0f || cfg.freqHz < 1.0f) return false;

  uint32_t samplesPerCycle = (uint32_t)lroundf(cfg.sampRateHz / cfg.freqHz);
  if (samplesPerCycle < 8) samplesPerCycle = 8;

  uint32_t N = samplesPerCycle * (uint32_t)PREF_CYCLES;
  if (N > cfg.totalRecords) N = cfg.totalRecords;
  if (N < 32) return false;

  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  // buffer de registro (maior para não estourar se tiver muitos canais)
  static uint8_t recBuf[512];
  if (cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }

  double sum = 0.0;
  double sumSq = 0.0;
  uint32_t count = 0;

  for (uint32_t i = 0; i < N; i++) {
    int rd = bdat.read(recBuf, cfg.recSize);
    if (rd != (int)cfg.recSize) { bdat.close(); return false; }

    int off = 0;
    off += 4;
    off += cfg.ts64 ? 8 : 4;

    // percorre analógicos; pega apenas ch1 (corrente)
    for (int ch = 0; ch < cfg.nAnalog; ch++) {
      int16_t raw;
      memcpy(&raw, recBuf + off, 2);
      off += 2;

      if (ch == 1) {
        float engI = cfg.chI.rawToEng(raw);
        sum   += (double)engI;
        sumSq += (double)engI * (double)engI;
        count++;
      }
    }

    // pula digitais
    off += (int)cfg.digitalBytes;
  }

  bdat.close();
  if (count < 16) return false;

  double mean = sum / (double)count;
  double var  = (sumSq / (double)count) - (mean * mean);
  if (var < 0.0) var = 0.0;
  double rms = sqrt(var); // RMS AC (remove DC)

  out_INomPeakA = (float)(rms * 1.41421356237); // pico ≈ RMS*sqrt(2)
  if (out_INomPeakA < 1e-6f) return false;
  return true;
}

// ==============================
// Map ENG -> volts (fixo por nominal)
// - Tensão: eng em kV (pico nominal 188 kV)
// - Corrente: eng em A (pico nominal calibrado ou manual)
// ==============================
static inline float mapEngToVolts_FixedPeak(float eng, float nomPeak, bool useSoftLimiter) {
  if (nomPeak < 1e-9f) nomPeak = 1.0f;
  float x = eng / nomPeak;      // -1..+1 (nominal)
  if (useSoftLimiter) x = softLimit(x);
  // se quiser clamp duro:
  // if (x > 1.0f) x = 1.0f;
  // if (x < -1.0f) x = -1.0f;
  return V_MID + x * V_AMP;
}

// ==============================
// Setup / Loop
// ==============================
ComtradeCfg g_cfg;

void setup() {
  Serial.begin(115200);
  delay(150);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN_V, ADC_11db);
  analogSetPinAttenuation(ADC_PIN_I, ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    Serial.println("❌ MCP4728 não encontrado");
    while (true) delay(1000);
  }

  if (!SD.begin(SD_CS)) {
    Serial.println("❌ Falha ao montar SD");
    while (true) delay(1000);
  }

  if (!parseCfgFromSD(g_cfg)) {
    Serial.println("❌ Falha parse CFG");
    while (true) delay(1000);
  }

  if (!computeBdatLayout(g_cfg)) {
    Serial.println("❌ Falha layout BDAT");
    while (true) delay(1000);
  }

  float iNomPeak = I_NOM_PEAK_A;
  if (I_CALIBRATE_FROM_PREFault_RMS) {
    float calib = 0.0f;
    if (calibrateCurrentFromPrefaultRMS(g_cfg, calib)) iNomPeak = calib;
  }
  if (iNomPeak < 1e-6f) iNomPeak = 1.0f; // fallback

  Serial.printf("CFG: nA=%d nD=%d fs=%.2fHz f=%.2fHz timeMult=%.6f ts64=%d rec=%u total=%u\n",
                g_cfg.nAnalog, g_cfg.nDigital, g_cfg.sampRateHz, g_cfg.freqHz, g_cfg.timeMult,
                (int)g_cfg.ts64, (unsigned)g_cfg.recSize, (unsigned)g_cfg.totalRecords);

  Serial.printf("Escalas: V_nom_peak=%.3f kV | I_nom_peak=%.6f A | VAMP=%.3fV\n",
                V_NOM_PEAK_KV, iNomPeak, V_AMP);

  // abre CSV
  SD.remove(OUT_CSV_PATH);
  File csv = SD.open(OUT_CSV_PATH, FILE_WRITE);
  if (!csv) {
    Serial.println("❌ Não abriu output.csv");
    while (true) delay(1000);
  }
  csv.println("sample,t_us,dt_us,dacV_code,dacI_code,adcV_raw,adcI_raw,adcV_V,adcI_V");

  // abre BDAT
  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) {
    Serial.println("❌ Não abriu BDAT");
    while (true) delay(1000);
  }

  static uint8_t recBuf[512];
  if (g_cfg.recSize > sizeof(recBuf)) {
    Serial.println("❌ recSize > recBuf");
    while (true) delay(1000);
  }

  uint16_t mid = voltsToDac12(V_MID);
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

  int64_t prevTs = 0;
  bool first = true;
  uint32_t t_accum = 0;

  for (uint32_t i = 0; i < g_cfg.totalRecords; i++) {
    int rd = bdat.read(recBuf, g_cfg.recSize);
    if (rd != (int)g_cfg.recSize) {
      Serial.println("❌ Leitura BDAT incompleta");
      break;
    }

    int off = 0;

    uint32_t sampleNum = 0;
    memcpy(&sampleNum, recBuf + off, 4);
    off += 4;

    int64_t ts = 0;
    if (!g_cfg.ts64) {
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
      dt_us = (uint32_t)lroundf((float)dts * g_cfg.timeMult);
      if (dt_us == 0 && g_cfg.sampRateHz > 0.1f) {
        dt_us = (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz);
      }
      if (dt_us > 0) busyWaitMicros(dt_us);
    } else {
      first = false;
      dt_us = 0;
    }
    prevTs = ts;
    t_accum += dt_us;

    // lê analógicos: ch0=V, ch1=I
    float engV = 0.0f;
    float engI = 0.0f;

    for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;

      if (ch == 0) engV = g_cfg.chV.rawToEng(raw);
      else if (ch == 1) engI = g_cfg.chI.rawToEng(raw);
    }

    // pula digitais
    off += (int)g_cfg.digitalBytes;

    // mapeia para DAC (tensão em kV, corrente em A)
    // ✅ tensão fixo 188kV pico
    float vOutV = mapEngToVolts_FixedPeak(engV, V_NOM_PEAK_KV, true);

    // ✅ corrente: pico nominal calibrado ou manual (também com soft limiter)
    float vOutI = mapEngToVolts_FixedPeak(engI, iNomPeak, true);

    uint16_t dacV = voltsToDac12(vOutV);
    uint16_t dacI = voltsToDac12(vOutI);

    // atualiza MCP4728
    dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

    delayMicroseconds(DAC_SETTLE_US);

    // mede (se houver loopback)
    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);

    // salva CSV
    csv.print(sampleNum); csv.print(",");
    csv.print(t_accum);   csv.print(",");
    csv.print(dt_us);     csv.print(",");
    csv.print(dacV);      csv.print(",");
    csv.print(dacI);      csv.print(",");
    csv.print(adcV_raw);  csv.print(",");
    csv.print(adcI_raw);  csv.print(",");
    csv.print(adcV_V, 6); csv.print(",");
    csv.println(adcI_V, 6);
  }

  bdat.close();
  csv.flush();
  csv.close();

  Serial.print("✅ output.csv salvo: ");
  Serial.println(OUT_CSV_PATH);
  while (true) delay(1000);
}

void loop() {
  delay(1000);
}
