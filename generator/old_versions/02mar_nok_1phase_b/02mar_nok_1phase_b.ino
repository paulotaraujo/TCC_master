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
static const float    V_AMP   = 1.55f;      // headroom

// ==============================
// Auto-nominal (picos) e clip
// ==============================
static const int   NOM_CYCLES = 5;              // 5 ciclos p/ estimar nominal
static const float V_FAULT_LIMIT_MULT = 2.0f;   // tensão clipa acima de 2x nominal
static const float I_FAULT_LIMIT_MULT = 3.0f;   // corrente clipa acima de 3x nominal

// ==============================
// Pré-falta (buffer)
// ==============================
static const int PRE_CYCLES = 5;

// ==============================
// ✅ STRUCTS (devem vir antes de qualquer função)
// ==============================
struct AnalogCh {
  float a = 1.0f;
  float b = 0.0f;
  inline float rawToEng(int16_t raw) const { return a * (float)raw + b; }
};

struct ComtradeCfg {
  int nAnalog = 0;
  int nDigital = 0;

  float freqHz = 60.0f;
  float sampRateHz = 0.0f;
  float timeMult = 1.0f;

  char dataFormat[16] = {0};

  AnalogCh chV;
  AnalogCh chI;

  bool ts64 = false;
  uint32_t recSize = 0;
  uint32_t totalRecords = 0;
  uint32_t digitalBytes = 0;
};

struct PrefaultSample {
  uint16_t dacV;
  uint16_t dacI;
  uint32_t dt_us;
  uint32_t sampleNum;
  int64_t  ts;
  uint8_t  clipV;
  uint8_t  clipI;
};

// ==============================
// Estado
// ==============================
enum PlayerState : uint8_t {
  ST_IDLE = 0,
  ST_PREFLOOP,
  ST_FAULT_PREFSHOT,
  ST_FAULT_STREAM
};

// ==============================
// Globais
// ==============================
ComtradeCfg g_cfg;

static float g_vNomPeak = 0.0f;
static float g_iNomPeak = 0.0f;
static float g_vClipPeak = 0.0f;
static float g_iClipPeak = 0.0f;

static uint32_t g_samplesPerCycle = 0;
static uint32_t g_preSamples = 0;

static PrefaultSample* g_preBuf = nullptr;
static int64_t g_lastTsPrefault = 0;

static File g_csv;
static File g_bdat;

static PlayerState g_state = ST_IDLE;
static bool g_faultRequested = false;

static uint32_t g_bufPos = 0;
static uint32_t g_bufRemaining = 0;

static bool g_streamFirst = true;
static int64_t g_prevTsStream = 0;

static uint32_t g_tAccum = 0;
static bool g_promptPrinted = false;

// ✅ robustez CSV / WDT
static uint32_t g_csvLines = 0;
static uint32_t g_yieldTick = 0;

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

static inline void outputDacCodes(uint16_t dacV, uint16_t dacI) {
  dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
}

static inline void setIdleOutput() {
  uint16_t idle = voltsToDac12(V_MID);
  outputDacCodes(idle, idle);
}

static inline char readCommandCharNonBlocking() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') continue;
    return (char)c;
  }
  return 0;
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
// COMTRADE: parse CFG / layout BDAT
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

  for (int i = 0; i < cfg.nAnalog; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
    int k = splitCSV(line, parts, 32);
    if (k < 7) { f.close(); return false; }

    float a = strtof(parts[5], nullptr);
    float b = strtof(parts[6], nullptr);

    if (i == 0) { cfg.chV.a = a; cfg.chV.b = b; }
    if (i == 1) { cfg.chI.a = a; cfg.chI.b = b; }
  }

  for (int i = 0; i < cfg.nDigital; i++) {
    if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  }

  if (readLine(f, line, sizeof(line))) {
    float fr = strtof(line, nullptr);
    if (fr > 1.0f && fr < 500.0f) cfg.freqHz = fr;
  }

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

  readLine(f, line, sizeof(line));
  readLine(f, line, sizeof(line));

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
  strncpy(cfg.dataFormat, line, sizeof(cfg.dataFormat)-1);

  if (readLine(f, line, sizeof(line))) {
    float tm = strtof(line, nullptr);
    if (tm > 0.0f) cfg.timeMult = tm;
  }

  f.close();
  return true;
}

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
// Nominal por ciclos (média dos picos por ciclo)
// ==============================
static bool computeNominalPeaksFromFirstCycles(
  ComtradeCfg &cfg,
  float &out_V_nom_peak,
  float &out_I_nom_peak,
  float &out_V_clip_peak,
  float &out_I_clip_peak
) {
  out_V_nom_peak = out_I_nom_peak = 0.0f;
  out_V_clip_peak = out_I_clip_peak = 0.0f;

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
      cyclePeakV = cyclePeakI = 0.0f;
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
// Map ENG -> volts com clip por pico
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
// BDAT: leitura de record
// ==============================
static bool readBdatRecord(
  File &bdat,
  const ComtradeCfg &cfg,
  uint8_t *recBuf,
  size_t recBufSize,
  uint32_t &outSampleNum,
  int64_t &outTs,
  float &outEngV,
  float &outEngI
) {
  if (!bdat) return false;
  if (cfg.recSize > recBufSize) return false;

  int rd = bdat.read(recBuf, cfg.recSize);
  if (rd != (int)cfg.recSize) return false;

  int off = 0;
  memcpy(&outSampleNum, recBuf + off, 4);
  off += 4;

  if (!cfg.ts64) {
    int32_t t32 = 0;
    memcpy(&t32, recBuf + off, 4);
    off += 4;
    outTs = (int64_t)t32;
  } else {
    int64_t t64 = 0;
    memcpy(&t64, recBuf + off, 8);
    off += 8;
    outTs = t64;
  }

  outEngV = 0.0f;
  outEngI = 0.0f;

  for (int ch = 0; ch < cfg.nAnalog; ch++) {
    int16_t raw = 0;
    memcpy(&raw, recBuf + off, 2);
    off += 2;
    if (ch == 0) outEngV = cfg.chV.rawToEng(raw);
    else if (ch == 1) outEngI = cfg.chI.rawToEng(raw);
  }

  off += (int)cfg.digitalBytes;
  (void)off;
  return true;
}

// ==============================
// CSV helpers
// ==============================
static bool openCsvForFault() {
  if (g_csv) g_csv.close();
  SD.remove(OUT_CSV_PATH);

  g_csv = SD.open(OUT_CSV_PATH, FILE_WRITE);
  if (!g_csv) return false;

  g_csv.println("sample,t_us,dt_us,dacV_code,dacI_code,adcV_raw,adcI_raw,adcV_V,adcI_V,clipV,clipI");
  g_csv.flush();

  g_csvLines = 0;
  return true;
}

static void closeCsv() {
  if (g_csv) {
    g_csv.flush();
    g_csv.close();
  }
}

static void closeBdat() {
  if (g_bdat) g_bdat.close();
}

// ==============================
// Monta buffer dos 5 ciclos iniciais
// ==============================
static bool buildPrefaultBuffer() {
  if (g_cfg.sampRateHz < 1.0f || g_cfg.freqHz < 1.0f) return false;

  g_samplesPerCycle = (uint32_t)lroundf(g_cfg.sampRateHz / g_cfg.freqHz);
  if (g_samplesPerCycle < 8) g_samplesPerCycle = 8;

  g_preSamples = (uint32_t)PRE_CYCLES * g_samplesPerCycle;
  if (g_preSamples > g_cfg.totalRecords) g_preSamples = g_cfg.totalRecords;
  if (g_preSamples < g_samplesPerCycle) return false;

  if (g_preBuf) {
    free(g_preBuf);
    g_preBuf = nullptr;
  }

  g_preBuf = (PrefaultSample*)malloc(sizeof(PrefaultSample) * g_preSamples);
  if (!g_preBuf) return false;

  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;

  static uint8_t recBuf[512];
  if (g_cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }

  const uint32_t defaultDtUs = (g_cfg.sampRateHz > 0.1f)
    ? (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz)
    : 0;

  int64_t prevTs = 0;
  bool first = true;

  for (uint32_t i = 0; i < g_preSamples; i++) {
    uint32_t sampleNum = 0;
    int64_t ts = 0;
    float engV = 0.0f, engI = 0.0f;

    if (!readBdatRecord(bdat, g_cfg, recBuf, sizeof(recBuf), sampleNum, ts, engV, engI)) {
      bdat.close();
      return false;
    }

    uint32_t dt_us = 0;
    if (first) {
      dt_us = defaultDtUs;
      first = false;
    } else {
      int64_t dts = ts - prevTs;
      if (dts < 0) dts = 0;
      dt_us = (uint32_t)lroundf((float)dts * g_cfg.timeMult);
      if (dt_us == 0) dt_us = defaultDtUs;
    }
    prevTs = ts;

    const uint8_t clipV = (fabsf(engV) > g_vClipPeak) ? 1 : 0;
    const uint8_t clipI = (fabsf(engI) > g_iClipPeak) ? 1 : 0;

    float vOutV = mapEngToVolts_ClipPeakFS(engV, g_vClipPeak);
    float vOutI = mapEngToVolts_ClipPeakFS(engI, g_iClipPeak);

    g_preBuf[i].dacV = voltsToDac12(vOutV);
    g_preBuf[i].dacI = voltsToDac12(vOutI);
    g_preBuf[i].dt_us = dt_us;
    g_preBuf[i].sampleNum = sampleNum;
    g_preBuf[i].ts = ts;
    g_preBuf[i].clipV = clipV;
    g_preBuf[i].clipI = clipI;
  }

  bdat.close();
  g_lastTsPrefault = g_preBuf[g_preSamples - 1].ts;
  return true;
}

// ==============================
// Toca 1 sample do buffer
// ==============================
static bool playOnePrefaultSample(bool logCsv) {
  if (!g_preBuf || g_preSamples == 0) return false;

  const PrefaultSample &s = g_preBuf[g_bufPos];

  if (s.dt_us > 0) {
    busyWaitMicros(s.dt_us);
    g_tAccum += s.dt_us;
  }

  outputDacCodes(s.dacV, s.dacI);
  delayMicroseconds(DAC_SETTLE_US);

  if (logCsv && g_csv) {
    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);

    char line[180];
    int n = snprintf(line, sizeof(line),
                     "%lu,%lu,%lu,%u,%u,%u,%u,%.6f,%.6f,%u,%u\n",
                     (unsigned long)s.sampleNum,
                     (unsigned long)g_tAccum,
                     (unsigned long)s.dt_us,
                     (unsigned)s.dacV,
                     (unsigned)s.dacI,
                     (unsigned)adcV_raw,
                     (unsigned)adcI_raw,
                     adcV_V, adcI_V,
                     (unsigned)s.clipV,
                     (unsigned)s.clipI);

    if (n > 0) g_csv.write((const uint8_t*)line, (size_t)n);

    g_csvLines++;
    if ((g_csvLines % 50) == 0) g_csv.flush();
  }

  g_bufPos++;
  if (g_bufPos >= g_preSamples) g_bufPos = 0;

  if (g_bufRemaining > 0) g_bufRemaining--;
  return true;
}

// ==============================
// Stream: toca 1 record do BDAT
// ==============================
static bool playOneBdatRecordStream() {
  if (!g_bdat) return false;

  static uint8_t recBuf[512];
  if (g_cfg.recSize > sizeof(recBuf)) return false;

  uint32_t sampleNum = 0;
  int64_t ts = 0;
  float engV = 0.0f, engI = 0.0f;

  if (!readBdatRecord(g_bdat, g_cfg, recBuf, sizeof(recBuf), sampleNum, ts, engV, engI)) {
    return false; // EOF
  }

  const uint32_t defaultDtUs = (g_cfg.sampRateHz > 0.1f)
    ? (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz)
    : 0;

  uint32_t dt_us = 0;

  if (!g_streamFirst) {
    int64_t dts = ts - g_prevTsStream;
    if (dts < 0) dts = 0;
    dt_us = (uint32_t)lroundf((float)dts * g_cfg.timeMult);
    if (dt_us == 0) dt_us = defaultDtUs;
    if (dt_us > 0) { busyWaitMicros(dt_us); g_tAccum += dt_us; }
  } else {
    int64_t dts = ts - g_prevTsStream;
    if (dts < 0) dts = 0;
    dt_us = (uint32_t)lroundf((float)dts * g_cfg.timeMult);
    if (dt_us == 0) dt_us = defaultDtUs;
    if (dt_us > 0) { busyWaitMicros(dt_us); g_tAccum += dt_us; }
    g_streamFirst = false;
  }

  g_prevTsStream = ts;

  const uint8_t clipV = (fabsf(engV) > g_vClipPeak) ? 1 : 0;
  const uint8_t clipI = (fabsf(engI) > g_iClipPeak) ? 1 : 0;

  float vOutV = mapEngToVolts_ClipPeakFS(engV, g_vClipPeak);
  float vOutI = mapEngToVolts_ClipPeakFS(engI, g_iClipPeak);

  uint16_t dacV = voltsToDac12(vOutV);
  uint16_t dacI = voltsToDac12(vOutI);

  outputDacCodes(dacV, dacI);
  delayMicroseconds(DAC_SETTLE_US);

  if (g_csv) {
    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);

    char line[180];
    int n = snprintf(line, sizeof(line),
                     "%lu,%lu,%lu,%u,%u,%u,%u,%.6f,%.6f,%u,%u\n",
                     (unsigned long)sampleNum,
                     (unsigned long)g_tAccum,
                     (unsigned long)dt_us,
                     (unsigned)dacV,
                     (unsigned)dacI,
                     (unsigned)adcV_raw,
                     (unsigned)adcI_raw,
                     adcV_V, adcI_V,
                     (unsigned)clipV,
                     (unsigned)clipI);

    if (n > 0) g_csv.write((const uint8_t*)line, (size_t)n);

    g_csvLines++;
    if ((g_csvLines % 50) == 0) g_csv.flush();
  }

  return true;
}

// ==============================
// setup / loop
// ==============================
void setup() {
  Serial.begin(115200);
  delay(150);

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
    Serial.println("❌ Falha em calcular picos nominais");
    while (true) delay(1000);
  }

  if (!buildPrefaultBuffer()) {
    Serial.println("❌ Falha em montar buffer de pré-falta (5 ciclos)");
    while (true) delay(1000);
  }

  Serial.printf("CFG: nA=%d nD=%d fs=%.2fHz f=%.2fHz timeMult=%.6f ts64=%d rec=%u total=%u\n",
                g_cfg.nAnalog, g_cfg.nDigital, g_cfg.sampRateHz, g_cfg.freqHz, g_cfg.timeMult,
                (int)g_cfg.ts64, (unsigned)g_cfg.recSize, (unsigned)g_cfg.totalRecords);

  Serial.printf("Nominais (média %d ciclos): V_nom_peak=%.6f | I_nom_peak=%.6f\n",
                NOM_CYCLES, g_vNomPeak, g_iNomPeak);
  Serial.printf("Limites: V_clip=%.6f (%.1fx) | I_clip=%.6f (%.1fx)\n",
                g_vClipPeak, V_FAULT_LIMIT_MULT, g_iClipPeak, I_FAULT_LIMIT_MULT);

  Serial.printf("Pré-falta: %u ciclos x %u amostras/ciclo = %u amostras\n",
                (unsigned)PRE_CYCLES, (unsigned)g_samplesPerCycle, (unsigned)g_preSamples);

  setIdleOutput();

  Serial.println();
  Serial.println("🟡 Pronto.");
  Serial.println("  - 's' = start (loop pré-falta)");
  Serial.println("  - 'f' = fault (termina ciclo atual, toca 5 ciclos 1x e segue BDAT)");
  Serial.println("  - 'q' = stop");
}

void loop() {
  // anti-WDT/RTOS starvation
  g_yieldTick++;
  if ((g_yieldTick % 100) == 0) yield();

  const char cmd = readCommandCharNonBlocking();
  if (cmd) {
    if (cmd == 'q' || cmd == 'Q') {
      if (g_csv) g_csv.flush();
      g_state = ST_IDLE;
      g_faultRequested = false;
      closeBdat();
      closeCsv();
      setIdleOutput();
      Serial.println("🛑 Stop (q). Voltou para IDLE.");
      delay(50);
      return;
    }

    if (cmd == 's' || cmd == 'S') {
      g_state = ST_PREFLOOP;
      g_faultRequested = false;
      g_bufPos = 0;
      g_promptPrinted = false;
      setIdleOutput();
      Serial.println("▶️ Pré-falta: reproduzindo continuamente (5 ciclos). Envie 'f' para falta.");
    }

    if (cmd == 'f' || cmd == 'F') {
      if (g_state == ST_PREFLOOP) {
        g_faultRequested = true;
        Serial.println("⚡ Comando 'f' recebido. Vou terminar o ciclo atual e iniciar sequência de falta.");
      } else {
        Serial.println("ℹ️ 'f' só funciona após 's' (durante pré-falta).");
      }
    }
  }

  switch (g_state) {
    case ST_IDLE: {
      setIdleOutput();
      if (!g_promptPrinted) {
        Serial.println("Digite 's' para iniciar (pré-falta).");
        g_promptPrinted = true;
      }
      delay(20);
      break;
    }

    case ST_PREFLOOP: {
      (void)playOnePrefaultSample(false);

      if (g_faultRequested) {
        const uint32_t posPlayed = (g_bufPos == 0) ? (g_preSamples - 1) : (g_bufPos - 1);
        const uint32_t cyclePos = posPlayed % g_samplesPerCycle;

        if (cyclePos == (g_samplesPerCycle - 1)) {
          if (!openCsvForFault()) {
            Serial.println("❌ Não abriu output.csv");
            g_state = ST_IDLE;
            g_faultRequested = false;
            break;
          }

          g_tAccum = 0;
          g_bufPos = 0;
          g_bufRemaining = g_preSamples; // toca 5 ciclos UMA vez
          g_state = ST_FAULT_PREFSHOT;

          Serial.println("▶️ Falta: tocando 5 ciclos (1x) e depois seguindo BDAT pós pré-falta.");
        }
      }
      break;
    }

    case ST_FAULT_PREFSHOT: {
      if (g_bufRemaining == 0) {
        closeBdat();
        g_bdat = SD.open(BDAT_PATH, FILE_READ);
        if (!g_bdat) {
          Serial.println("❌ Não abriu BDAT para streaming");
          if (g_csv) g_csv.flush();
          closeCsv();
          g_state = ST_IDLE;
          break;
        }

        const uint32_t startRecord = g_preSamples;
        const uint32_t startByte = startRecord * g_cfg.recSize;

        if (!g_bdat.seek(startByte)) {
          Serial.println("❌ Falha no seek do BDAT (pós pré-falta)");
          closeBdat();
          if (g_csv) g_csv.flush();
          closeCsv();
          g_state = ST_IDLE;
          break;
        }

        g_prevTsStream = g_lastTsPrefault;
        g_streamFirst = true;

        if (g_csv) g_csv.flush(); // ✅ garante integridade antes de iniciar o stream
        g_state = ST_FAULT_STREAM;

        Serial.println("⚡ Stream do restante do arquivo iniciado.");
        break;
      }

      (void)playOnePrefaultSample(true);
      break;
    }

    case ST_FAULT_STREAM: {
      if (!playOneBdatRecordStream()) {
        closeBdat();
        if (g_csv) g_csv.flush();
        closeCsv();
        setIdleOutput();

        Serial.print("✅ output.csv salvo: ");
        Serial.println(OUT_CSV_PATH);
        Serial.println("🟡 Fim do arquivo. Envie 's' para voltar ao pré-falta.");
        g_state = ST_IDLE;
        g_faultRequested = false;
      }
      break;
    }

    default:
      g_state = ST_IDLE;
      break;
  }
}