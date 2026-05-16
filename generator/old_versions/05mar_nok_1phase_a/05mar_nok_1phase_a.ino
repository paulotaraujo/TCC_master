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

static const char* CFG_PATH  = "/comtrade_binary/export.cfg";
static const char* BDAT_PATH = "/comtrade_binary/export.bdat";

Adafruit_MCP4728 dac;

// ==============================
// Reprodução / timing
// ==============================
static const uint16_t DAC_SETTLE_US = 80;   // ajuste conforme seu RC/buffer
static const float    V_MID   = 1.65f;
static const float    V_AMP   = 1.55f;      // headroom (evita bater 0/3.3)

// ==============================
// Auto-nominal por janela fixa (60 Hz)
// ==============================
static const int   NOM_CYCLES = 5;              // ✅ 5 ciclos para estimar amplitude de pico nominal
static const float V_FAULT_LIMIT_MULT = 2.0f;   // ✅ tensão clipa acima de 2x nominal
static const float I_FAULT_LIMIT_MULT = 3.0f;   // ✅ corrente clipa acima de 3x nominal

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

static bool computeNominalPeaksFromFirstCycles(
  ComtradeCfg &cfg,
  float &out_V_nom_peak,
  float &out_I_nom_peak,
  float &out_V_clip_peak,
  float &out_I_clip_peak
);

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

  // freq
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

  // start / trigger
  readLine(f, line, sizeof(line));
  readLine(f, line, sizeof(line));

  // format
  if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  strncpy(cfg.dataFormat, line, sizeof(cfg.dataFormat)-1);

  // timemult
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
// ✅ Nominal por 5 ciclos: pico nominal = média dos picos por ciclo
// Depois: pico de clip = mult * pico_nominal
// ==============================
static bool computeNominalPeaksFromFirstCycles(
  ComtradeCfg &cfg,
  float &out_V_nom_peak,
  float &out_I_nom_peak,
  float &out_V_clip_peak,
  float &out_I_clip_peak
) {
  out_V_nom_peak = 0.0f;
  out_I_nom_peak = 0.0f;
  out_V_clip_peak = 0.0f;
  out_I_clip_peak = 0.0f;

  if (cfg.sampRateHz < 1.0f || cfg.freqHz < 1.0f) return false;

  uint32_t samplesPerCycle = (uint32_t)lroundf(cfg.sampRateHz / cfg.freqHz);
  if (samplesPerCycle < 8) samplesPerCycle = 8;

  uint32_t N = samplesPerCycle * (uint32_t)NOM_CYCLES;
  if (N > cfg.totalRecords) N = cfg.totalRecords;
  if (N < samplesPerCycle) return false;

  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  static uint8_t recBuf[512];
  if (cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }

  double sumPeakV = 0.0;
  double sumPeakI = 0.0;
  uint32_t cyclesCounted = 0;

  float cyclePeakV = 0.0f;
  float cyclePeakI = 0.0f;
  uint32_t idxInCycle = 0;

  for (uint32_t i = 0; i < N; i++) {
    int rd = bdat.read(recBuf, cfg.recSize);
    if (rd != (int)cfg.recSize) { bdat.close(); return false; }

    int off = 0;
    off += 4;
    off += cfg.ts64 ? 8 : 4;

    float engV = 0.0f;
    float engI = 0.0f;

    for (int ch = 0; ch < cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;
      if (ch == 0) engV = cfg.chV.rawToEng(raw);
      else if (ch == 1) engI = cfg.chI.rawToEng(raw);
    }
    off += (int)cfg.digitalBytes;

    float aV = fabsf(engV);
    float aI = fabsf(engI);
    if (aV > cyclePeakV) cyclePeakV = aV;
    if (aI > cyclePeakI) cyclePeakI = aI;

    idxInCycle++;
    if (idxInCycle >= samplesPerCycle) {
      sumPeakV += (double)cyclePeakV;
      sumPeakI += (double)cyclePeakI;
      cyclesCounted++;

      cyclePeakV = 0.0f;
      cyclePeakI = 0.0f;
      idxInCycle = 0;
    }
  }

  bdat.close();
  if (cyclesCounted == 0) return false;

  out_V_nom_peak = (float)(sumPeakV / (double)cyclesCounted);
  out_I_nom_peak = (float)(sumPeakI / (double)cyclesCounted);

  if (out_V_nom_peak < 1e-6f) out_V_nom_peak = 1.0f;
  if (out_I_nom_peak < 1e-6f) out_I_nom_peak = 1.0f;

  out_V_clip_peak = out_V_nom_peak * V_FAULT_LIMIT_MULT;
  out_I_clip_peak = out_I_nom_peak * I_FAULT_LIMIT_MULT;
  return true;
}

// ==============================
// ✅ Map ENG -> volts com FS no CLIP_PEAK
// ==============================
static inline float mapEngToVolts_ClipPeakFS(float eng, float clipPeak) {
  if (clipPeak < 1e-9f) clipPeak = 1.0f;

  if (eng >  clipPeak) eng =  clipPeak;
  if (eng < -clipPeak) eng = -clipPeak;

  float x = eng / clipPeak;

  if (x >  1.0f) x =  1.0f;
  if (x < -1.0f) x = -1.0f;

  return V_MID + x * V_AMP;
}

// ==============================
// Setup / Loop
// ==============================
ComtradeCfg g_cfg;

enum RunState : uint8_t {
  STATE_IDLE = 0,
  STATE_LOOP_FIRST_CYCLE,
  STATE_PLAY_REST
};

static RunState g_state = STATE_IDLE;
static bool     g_pendingFault = false;

// Pico nominal / limites (calculados no boot)
static float g_vNomPeak  = 0.0f;
static float g_iNomPeak  = 0.0f;
static float g_vClipPeak = 0.0f;
static float g_iClipPeak = 0.0f;

// Buffer do 1º ciclo
static uint32_t g_samplesPerCycle = 0;
static uint32_t g_dtLoopUs = 0;
static float*   g_cycleEngV = nullptr;
static float*   g_cycleEngI = nullptr;
static int64_t  g_lastTsFirstCycle = 0;

// Execução
static uint32_t g_cycleIdx = 0;
static uint32_t g_tAccumUs = 0;

static File g_bdat;
static bool g_bdatOpen = false;

static bool   g_firstRest = true;
static int64_t g_prevTsRest = 0;

static inline void dacWriteMid() {
  uint16_t mid = voltsToDac12(V_MID);
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
}

static inline void stopAllAndIdle() {
  if (g_bdatOpen) {
    g_bdat.close();
    g_bdatOpen = false;
  }
  g_pendingFault = false;
  g_cycleIdx = 0;
  g_tAccumUs = 0;
  g_firstRest = true;
  g_prevTsRest = 0;
  g_state = STATE_IDLE;
  dacWriteMid();
}

static bool preloadFirstCycle() {
  if (g_cfg.sampRateHz < 1.0f || g_cfg.freqHz < 1.0f) return false;

  g_samplesPerCycle = (uint32_t)lroundf(g_cfg.sampRateHz / g_cfg.freqHz);
  if (g_samplesPerCycle < 8) g_samplesPerCycle = 8;
  if (g_samplesPerCycle > g_cfg.totalRecords) g_samplesPerCycle = g_cfg.totalRecords;
  if (g_samplesPerCycle < 2) return false;

  g_dtLoopUs = (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz);
  if (g_dtLoopUs == 0) g_dtLoopUs = 1;

  g_cycleEngV = (float*)malloc(sizeof(float) * g_samplesPerCycle);
  g_cycleEngI = (float*)malloc(sizeof(float) * g_samplesPerCycle);
  if (!g_cycleEngV || !g_cycleEngI) return false;

  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  static uint8_t recBuf[512];
  if (g_cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }

  int64_t lastTs = 0;
  for (uint32_t i = 0; i < g_samplesPerCycle; i++) {
    int rd = bdat.read(recBuf, g_cfg.recSize);
    if (rd != (int)g_cfg.recSize) { bdat.close(); return false; }

    int off = 0;
    off += 4; // sample

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
    lastTs = ts;

    float engV = 0.0f;
    float engI = 0.0f;
    for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;
      if (ch == 0) engV = g_cfg.chV.rawToEng(raw);
      else if (ch == 1) engI = g_cfg.chI.rawToEng(raw);
    }
    off += (int)g_cfg.digitalBytes;

    g_cycleEngV[i] = engV;
    g_cycleEngI[i] = engI;
  }
  bdat.close();
  g_lastTsFirstCycle = lastTs;
  return true;
}

static inline void applyOneSample(float engV, float engI, uint16_t &outDacV, uint16_t &outDacI, int &outClipV, int &outClipI) {
  outClipV = (fabsf(engV) > g_vClipPeak) ? 1 : 0;
  outClipI = (fabsf(engI) > g_iClipPeak) ? 1 : 0;

  float vOutV = mapEngToVolts_ClipPeakFS(engV, g_vClipPeak);
  float vOutI = mapEngToVolts_ClipPeakFS(engI, g_iClipPeak);

  outDacV = voltsToDac12(vOutV);
  outDacI = voltsToDac12(vOutI);

  dac.setChannelValue(MCP4728_CHANNEL_A, outDacV, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, outDacI, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
}

static void runLoopFirstCycleStep() {
  uint32_t dt_us = (g_tAccumUs == 0) ? 0u : g_dtLoopUs;
  if (dt_us) busyWaitMicros(dt_us);
  g_tAccumUs += dt_us;

  float engV = g_cycleEngV[g_cycleIdx];
  float engI = g_cycleEngI[g_cycleIdx];

  uint16_t dacV = 0, dacI = 0;
  int clipV = 0, clipI = 0;
  applyOneSample(engV, engI, dacV, dacI, clipV, clipI);
  delayMicroseconds(DAC_SETTLE_US);

  // (opcional) mede loopback, mas não salva
  (void)analogRead(ADC_PIN_V);
  (void)analogRead(ADC_PIN_I);
  (void)adcToVoltsCal(ADC_PIN_V);
  (void)adcToVoltsCal(ADC_PIN_I);

  bool endOfCycle = (g_cycleIdx + 1u >= g_samplesPerCycle);
  if (!endOfCycle) {
    g_cycleIdx++;
  } else {
    g_cycleIdx = 0;
    if (g_pendingFault) {
      g_state = STATE_PLAY_REST;
      g_pendingFault = false;

      g_firstRest = true;
      g_prevTsRest = 0;

      g_bdat = SD.open(BDAT_PATH, FILE_READ);
      if (!g_bdat) {
        Serial.println("❌ Não abriu BDAT para reprodução completa");
        stopAllAndIdle();
        return;
      }
      g_bdatOpen = true;
      uint32_t offset = g_samplesPerCycle * g_cfg.recSize;
      if (!g_bdat.seek(offset)) {
        Serial.println("❌ Falha seek no BDAT (início do 2º ciclo)");
        stopAllAndIdle();
        return;
      }
    }
  }
}

static void runPlayRestStep() {
  static uint8_t recBuf[512];
  if (g_cfg.recSize > sizeof(recBuf)) {
    Serial.println("❌ recSize > recBuf");
    stopAllAndIdle();
    return;
  }
  if (!g_bdatOpen) {
    Serial.println("❌ BDAT não está aberto");
    stopAllAndIdle();
    return;
  }

  // fim do arquivo
  if (g_bdat.position() >= g_bdat.size()) {
    if (g_bdatOpen) { g_bdat.close(); g_bdatOpen = false; }
    g_state = STATE_IDLE;
    g_firstRest = true;
    g_prevTsRest = 0;
    dacWriteMid();
    return;
  }

  int rd = g_bdat.read(recBuf, g_cfg.recSize);
  if (rd != (int)g_cfg.recSize) {
    Serial.println("❌ Leitura BDAT incompleta (modo completo)");
    stopAllAndIdle();
    return;
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

  if (g_firstRest) {
    g_prevTsRest = g_lastTsFirstCycle;
    g_firstRest = false;
  }

  int64_t dts = ts - g_prevTsRest;
  if (dts < 0) dts = 0;
  uint32_t dt_us = (uint32_t)lroundf((float)dts * g_cfg.timeMult);
  if (dt_us == 0 && g_cfg.sampRateHz > 0.1f) {
    dt_us = (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz);
  }
  if (dt_us) busyWaitMicros(dt_us);
  g_prevTsRest = ts;
  g_tAccumUs += dt_us;

  float engV = 0.0f;
  float engI = 0.0f;
  for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
    int16_t raw = 0;
    memcpy(&raw, recBuf + off, 2);
    off += 2;
    if (ch == 0) engV = g_cfg.chV.rawToEng(raw);
    else if (ch == 1) engI = g_cfg.chI.rawToEng(raw);
  }
  off += (int)g_cfg.digitalBytes;

  uint16_t dacV = 0, dacI = 0;
  int clipV = 0, clipI = 0;
  applyOneSample(engV, engI, dacV, dacI, clipV, clipI);
  delayMicroseconds(DAC_SETTLE_US);

  // (opcional) mede loopback, mas não salva
  (void)analogRead(ADC_PIN_V);
  (void)analogRead(ADC_PIN_I);
  (void)adcToVoltsCal(ADC_PIN_V);
  (void)adcToVoltsCal(ADC_PIN_I);
}

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

  if (!computeNominalPeaksFromFirstCycles(g_cfg, g_vNomPeak, g_iNomPeak, g_vClipPeak, g_iClipPeak)) {
    Serial.println("❌ Falha em calcular picos nominais (primeiros ciclos)");
    while (true) delay(1000);
  }

  Serial.printf("CFG: nA=%d nD=%d fs=%.2fHz f=%.2fHz timeMult=%.6f ts64=%d rec=%u total=%u\n",
                g_cfg.nAnalog, g_cfg.nDigital, g_cfg.sampRateHz, g_cfg.freqHz, g_cfg.timeMult,
                (int)g_cfg.ts64, (unsigned)g_cfg.recSize, (unsigned)g_cfg.totalRecords);

  Serial.printf("Nominais (média %d ciclos): V_nom_peak=%.6f | I_nom_peak=%.6f\n",
                NOM_CYCLES, g_vNomPeak, g_iNomPeak);
  Serial.printf("Limites: V_clip=%.6f (%.1fx) | I_clip=%.6f (%.1fx)\n",
                g_vClipPeak, V_FAULT_LIMIT_MULT, g_iClipPeak, I_FAULT_LIMIT_MULT);

  if (!preloadFirstCycle()) {
    Serial.println("❌ Falha ao pré-carregar o 1º ciclo do BDAT");
    while (true) delay(1000);
  }

  dacWriteMid();
  g_state = STATE_IDLE;

  Serial.println("\nComandos:");
  Serial.println("  s = iniciar loop contínuo do 1º ciclo");
  Serial.println("  f = (durante o loop) tocar o restante do COMTRADE após terminar o ciclo atual");
  Serial.println("  Q = parar tudo e voltar a 1.65V\n");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') continue;

    if (c == 'Q' || c == 'q') {
      Serial.println("⏹️  Parando e voltando ao IDLE (1.65V)");
      stopAllAndIdle();
      continue;
    }

    if (c == 's' || c == 'S') {
      if (g_state == STATE_IDLE) {
        Serial.println("▶️  Loop do 1º ciclo iniciado");
        g_state = STATE_LOOP_FIRST_CYCLE;
        g_pendingFault = false;
        g_cycleIdx = 0;
        g_tAccumUs = 0;
      } else {
        Serial.println("⚠️  Já em execução. Use 'Q' para parar.");
      }
      continue;
    }

    if (c == 'f' || c == 'F') {
      if (g_state == STATE_LOOP_FIRST_CYCLE) {
        Serial.println("⚡ Falta solicitada: vou esperar terminar o ciclo atual e tocar o restante.");
        g_pendingFault = true;
      } else {
        Serial.println("⚠️  'f' só faz sentido durante o loop do 1º ciclo. Use 's' antes.");
      }
      continue;
    }

    Serial.print("(ignorado) cmd=");
    Serial.println(c);
  }

  switch (g_state) {
    case STATE_IDLE:
      dacWriteMid();
      delay(20);
      break;
    case STATE_LOOP_FIRST_CYCLE:
      runLoopFirstCycleStep();
      break;
    case STATE_PLAY_REST:
      runPlayRestStep();
      break;
  }
}