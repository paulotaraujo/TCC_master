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

#define ADC_PIN_V     34
#define ADC_PIN_I     35

static const char* OUT_CSV_PATH = "/comtrade_binary/output.csv";

Adafruit_MCP4728 dac;

// ==============================
// Reprodução / timing
// ==============================
static const uint16_t DAC_SETTLE_US = 80;
static const float    V_MID   = 1.65f;
static const float    V_AMP   = 1.55f;

// ==============================
// Auto-nominal por janela fixa (60 Hz)
// ==============================
static const int   NOM_CYCLES = 5;
static const float V_FAULT_LIMIT_MULT = 2.0f;
static const float I_FAULT_LIMIT_MULT = 3.0f;

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
// Serial helpers (binário)
// ==============================
static bool serialReadExact(uint8_t* dst, size_t n, uint32_t timeout_ms) {
  uint32_t start = millis();
  size_t got = 0;

  while (got < n) {
    if ((millis() - start) > timeout_ms) return false;
    int avail = Serial.available();
    if (avail <= 0) { delay(1); continue; }
    int r = Serial.readBytes((char*)dst + got, n - got);
    if (r > 0) got += (size_t)r;
  }
  return true;
}

// lê 1 linha da Serial (até '\n') com timeout
static bool serialReadLine(char* out, size_t outSize, uint32_t timeout_ms) {
  if (!out || outSize < 2) return false;
  uint32_t start = millis();
  size_t n = 0;

  while (true) {
    if ((millis() - start) > timeout_ms) return false;

    int c = Serial.read();
    if (c < 0) { delay(1); continue; }

    char ch = (char)c;
    if (ch == '\r') continue;
    if (ch == '\n') break;

    if (n < outSize - 1) out[n++] = ch;
  }

  out[n] = '\0';
  return true;
}

// ==============================
// COMTRADE (mínimo necessário) - igual ao seu
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

  AnalogCh chV; // analógico 0
  AnalogCh chI; // analógico 1

  bool ts64 = false;
  uint32_t recSize = 0;
  uint32_t totalRecords = 0;
  uint32_t digitalBytes = 0;
};

static bool parseCfgFromSerial(ComtradeCfg &cfg) {
  char line[256];
  char* parts[32];

  // 1) id
  if (!serialReadLine(line, sizeof(line), 15000)) return false;

  // 2) TT,NA,ND
  if (!serialReadLine(line, sizeof(line), 15000)) return false;
  int n = splitCSV(line, parts, 8);
  if (n < 3) return false;

  cfg.nAnalog  = atoi(parts[1]);
  cfg.nDigital = atoi(parts[2]);

  // analógicos
  for (int i = 0; i < cfg.nAnalog; i++) {
    if (!serialReadLine(line, sizeof(line), 15000)) return false;
    int k = splitCSV(line, parts, 32);
    if (k < 7) return false;

    float a = strtof(parts[5], nullptr);
    float b = strtof(parts[6], nullptr);

    if (i == 0) { cfg.chV.a = a; cfg.chV.b = b; }
    if (i == 1) { cfg.chI.a = a; cfg.chI.b = b; }
  }

  // digitais (ignora linhas)
  for (int i = 0; i < cfg.nDigital; i++) {
    if (!serialReadLine(line, sizeof(line), 15000)) return false;
  }

  // freq
  if (!serialReadLine(line, sizeof(line), 15000)) return false;
  float fr = strtof(line, nullptr);
  if (fr > 1.0f && fr < 500.0f) cfg.freqHz = fr;

  // nrates
  if (!serialReadLine(line, sizeof(line), 15000)) return false;
  int nrates = atoi(line);

  if (nrates > 0) {
    if (!serialReadLine(line, sizeof(line), 15000)) return false;
    int r = splitCSV(line, parts, 8);
    if (r >= 1) cfg.sampRateHz = strtof(parts[0], nullptr);

    for (int i = 1; i < nrates; i++) {
      if (!serialReadLine(line, sizeof(line), 15000)) return false;
    }
  }

  // start / trigger (pula)
  serialReadLine(line, sizeof(line), 15000);
  serialReadLine(line, sizeof(line), 15000);

  // format
  if (!serialReadLine(line, sizeof(line), 15000)) return false;
  strncpy(cfg.dataFormat, line, sizeof(cfg.dataFormat)-1);

  // timemult (pode existir)
  if (!serialReadLine(line, sizeof(line), 15000)) return false;
  float tm = strtof(line, nullptr);
  if (tm > 0.0f) cfg.timeMult = tm;

  // última linha do bloco cfg: o PC manda "ENDCFG"
  if (!serialReadLine(line, sizeof(line), 15000)) return false;
  if (strcmp(line, "ENDCFG") != 0) return false;

  return true;
}

// header BDAT vindo do PC
#pragma pack(push,1)
struct BdatHeader {
  char magic[4];      // "BDAT"
  uint8_t ts64;       // 0/1
  uint32_t totalRecords;
};
#pragma pack(pop)

static bool computeBdatLayoutFromHeader(ComtradeCfg &cfg, const BdatHeader &h) {
  if (icasecmp(cfg.dataFormat, "BINARY") != 0) return false;

  int digitalWords = (cfg.nDigital + 15) / 16;
  cfg.digitalBytes = (uint32_t)digitalWords * 2;

  cfg.ts64 = (h.ts64 != 0);
  cfg.totalRecords = h.totalRecords;

  int recSize = 4 + (cfg.ts64 ? 8 : 4) + (cfg.nAnalog * 2) + (int)cfg.digitalBytes;
  if (recSize <= 0) return false;

  cfg.recSize = (uint32_t)recSize;
  return (cfg.totalRecords > 0 && cfg.recSize <= 512);
}

// ✅ nominal por 5 ciclos — agora lendo registros do BDAT via Serial
// Como não dá “rewind”, a gente bufferiza os N primeiros registros para tocar depois.
struct PreSample {
  uint32_t sampleNum;
  uint32_t dt_us;
  float engV;
  float engI;
};

static bool computeNominalAndBufferFirstCycles(
  ComtradeCfg &cfg,
  PreSample* pre,
  uint32_t preN,
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
  if (preN < N) return false;

  static uint8_t recBuf[512];

  double sumPeakV = 0.0, sumPeakI = 0.0;
  uint32_t cyclesCounted = 0;
  float cyclePeakV = 0.0f, cyclePeakI = 0.0f;
  uint32_t idxInCycle = 0;

  int64_t prevTs = 0;
  bool first = true;

  for (uint32_t i = 0; i < N; i++) {
    if (!serialReadExact(recBuf, cfg.recSize, 15000)) return false;

    int off = 0;
    uint32_t sampleNum = 0;
    memcpy(&sampleNum, recBuf + off, 4); off += 4;

    int64_t ts = 0;
    if (!cfg.ts64) {
      int32_t t32 = 0; memcpy(&t32, recBuf + off, 4); off += 4;
      ts = (int64_t)t32;
    } else {
      int64_t t64 = 0; memcpy(&t64, recBuf + off, 8); off += 8;
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
    } else {
      first = false;
      dt_us = 0;
    }
    prevTs = ts;

    float engV = 0.0f, engI = 0.0f;
    for (int ch = 0; ch < cfg.nAnalog; ch++) {
      int16_t raw = 0; memcpy(&raw, recBuf + off, 2); off += 2;
      if (ch == 0) engV = cfg.chV.rawToEng(raw);
      else if (ch == 1) engI = cfg.chI.rawToEng(raw);
    }
    off += (int)cfg.digitalBytes;

    // salva no buffer para tocar depois
    pre[i].sampleNum = sampleNum;
    pre[i].dt_us = dt_us;
    pre[i].engV = engV;
    pre[i].engI = engI;

    float aV = fabsf(engV), aI = fabsf(engI);
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
// Map ENG -> volts (igual ao seu)
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
// Setup
// ==============================
ComtradeCfg g_cfg;

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

  // handshake
  Serial.println("READY");

  // 1) CFG via Serial (texto)
  if (!parseCfgFromSerial(g_cfg)) {
    Serial.println("❌ Falha parse CFG (serial)");
    while (true) delay(1000);
  }

  // 2) header BDAT via Serial
  BdatHeader bh;
  if (!serialReadExact((uint8_t*)&bh, sizeof(bh), 15000)) {
    Serial.println("❌ Timeout header BDAT");
    while (true) delay(1000);
  }
  if (memcmp(bh.magic, "BDAT", 4) != 0) {
    Serial.println("❌ magic BDAT inválido");
    while (true) delay(1000);
  }

  if (!computeBdatLayoutFromHeader(g_cfg, bh)) {
    Serial.println("❌ Falha layout BDAT (header)");
    while (true) delay(1000);
  }

  // buffer dos primeiros N (5 ciclos)
  uint32_t samplesPerCycle = (uint32_t)lroundf(g_cfg.sampRateHz / g_cfg.freqHz);
  if (samplesPerCycle < 8) samplesPerCycle = 8;
  uint32_t N = samplesPerCycle * (uint32_t)NOM_CYCLES;
  if (N > g_cfg.totalRecords) N = g_cfg.totalRecords;

  PreSample* pre = (PreSample*)malloc(sizeof(PreSample) * N);
  if (!pre) {
    Serial.println("❌ Sem RAM p/ buffer inicial");
    while (true) delay(1000);
  }

  float vNomPeak = 0.0f, iNomPeak = 0.0f;
  float vClipPeak = 0.0f, iClipPeak = 0.0f;

  if (!computeNominalAndBufferFirstCycles(g_cfg, pre, N, vNomPeak, iNomPeak, vClipPeak, iClipPeak)) {
    Serial.println("❌ Falha em calcular picos nominais (serial)");
    free(pre);
    while (true) delay(1000);
  }

  Serial.printf("CFG: nA=%d nD=%d fs=%.2fHz f=%.2fHz timeMult=%.6f ts64=%d rec=%u total=%u\n",
                g_cfg.nAnalog, g_cfg.nDigital, g_cfg.sampRateHz, g_cfg.freqHz, g_cfg.timeMult,
                (int)g_cfg.ts64, (unsigned)g_cfg.recSize, (unsigned)g_cfg.totalRecords);

  Serial.printf("Nominais (média %d ciclos): V_nom_peak=%.6f | I_nom_peak=%.6f\n",
                NOM_CYCLES, vNomPeak, iNomPeak);
  Serial.printf("Limites: V_clip=%.6f (%.1fx) | I_clip=%.6f (%.1fx)\n",
                vClipPeak, V_FAULT_LIMIT_MULT, iClipPeak, I_FAULT_LIMIT_MULT);

  // abre CSV no SD
  SD.remove(OUT_CSV_PATH);
  File csv = SD.open(OUT_CSV_PATH, FILE_WRITE);
  if (!csv) {
    Serial.println("❌ Não abriu output.csv");
    free(pre);
    while (true) delay(1000);
  }
  csv.println("sample,t_us,dt_us,dacV_code,dacI_code,adcV_raw,adcI_raw,adcV_V,adcI_V,clipV,clipI");

  uint16_t mid = voltsToDac12(V_MID);
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

  uint32_t t_accum = 0;

  // 3) toca os N primeiros (buffer)
  for (uint32_t i = 0; i < N; i++) {
    uint32_t dt_us = pre[i].dt_us;
    if (dt_us > 0) busyWaitMicros(dt_us);
    t_accum += dt_us;

    float engV = pre[i].engV;
    float engI = pre[i].engI;

    int clipV = (fabsf(engV) > vClipPeak) ? 1 : 0;
    int clipI = (fabsf(engI) > iClipPeak) ? 1 : 0;

    float vOutV = mapEngToVolts_ClipPeakFS(engV, vClipPeak);
    float vOutI = mapEngToVolts_ClipPeakFS(engI, iClipPeak);

    uint16_t dacV = voltsToDac12(vOutV);
    uint16_t dacI = voltsToDac12(vOutI);

    dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

    delayMicroseconds(DAC_SETTLE_US);

    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);

    csv.print(pre[i].sampleNum); csv.print(",");
    csv.print(t_accum);         csv.print(",");
    csv.print(dt_us);           csv.print(",");
    csv.print(dacV);            csv.print(",");
    csv.print(dacI);            csv.print(",");
    csv.print(adcV_raw);        csv.print(",");
    csv.print(adcI_raw);        csv.print(",");
    csv.print(adcV_V, 6);       csv.print(",");
    csv.print(adcI_V, 6);       csv.print(",");
    csv.print(clipV);           csv.print(",");
    csv.println(clipI);
  }

  free(pre);

  // 4) continua tocando o resto lendo BDAT via Serial (registros restantes)
  static uint8_t recBuf[512];
  for (uint32_t i = N; i < g_cfg.totalRecords; i++) {
    if (!serialReadExact(recBuf, g_cfg.recSize, 15000)) {
      Serial.println("❌ Timeout/leitura BDAT incompleta");
      break;
    }

    int off = 0;
    uint32_t sampleNum = 0;
    memcpy(&sampleNum, recBuf + off, 4); off += 4;

    int64_t ts = 0;
    if (!g_cfg.ts64) { off += 4; } else { off += 8; } // aqui não recalculamos dt por ts (já vem no fluxo real);
    // Para manter igual ao seu, o dt é o espaçamento fixo se ts não for usado:
    uint32_t dt_us = (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz);
    busyWaitMicros(dt_us);
    t_accum += dt_us;

    float engV = 0.0f, engI = 0.0f;
    for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
      int16_t raw = 0; memcpy(&raw, recBuf + off, 2); off += 2;
      if (ch == 0) engV = g_cfg.chV.rawToEng(raw);
      else if (ch == 1) engI = g_cfg.chI.rawToEng(raw);
    }
    off += (int)g_cfg.digitalBytes;

    int clipV = (fabsf(engV) > vClipPeak) ? 1 : 0;
    int clipI = (fabsf(engI) > iClipPeak) ? 1 : 0;

    float vOutV = mapEngToVolts_ClipPeakFS(engV, vClipPeak);
    float vOutI = mapEngToVolts_ClipPeakFS(engI, iClipPeak);

    uint16_t dacV = voltsToDac12(vOutV);
    uint16_t dacI = voltsToDac12(vOutI);

    dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

    delayMicroseconds(DAC_SETTLE_US);

    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);

    csv.print(sampleNum); csv.print(",");
    csv.print(t_accum);   csv.print(",");
    csv.print(dt_us);     csv.print(",");
    csv.print(dacV);      csv.print(",");
    csv.print(dacI);      csv.print(",");
    csv.print(adcV_raw);  csv.print(",");
    csv.print(adcI_raw);  csv.print(",");
    csv.print(adcV_V, 6); csv.print(",");
    csv.print(adcI_V, 6); csv.print(",");
    csv.print(clipV);     csv.print(",");
    csv.println(clipI);
  }

  csv.flush();
  csv.close();

  Serial.print("✅ output.csv salvo: ");
  Serial.println(OUT_CSV_PATH);

  while (true) delay(1000);
}

void loop() { delay(1000); }