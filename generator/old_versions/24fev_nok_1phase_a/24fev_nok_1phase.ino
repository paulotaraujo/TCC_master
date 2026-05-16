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

static const char* OUT_CSV_PATH = "/comtrade_binary/output.csv";

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
static const int   NOM_CYCLES = 5;              // 5 ciclos para estimar amplitude nominal
static const float V_FAULT_LIMIT_MULT = 2.0f;   // tensão clipa acima de 2x nominal
static const float I_FAULT_LIMIT_MULT = 3.0f;   // corrente clipa acima de 3x nominal

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

// ==============================
// Serial line reader (robusto)
// ==============================
static bool readLineSerial(char* out, size_t outSize, uint32_t timeoutMs) {
  if (!out || outSize < 2) return false;
  uint32_t t0 = millis();
  size_t n = 0;
  while (millis() - t0 < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        out[n] = '\0';
        return (n > 0);
      }
      if (n < outSize - 1) out[n++] = c;
    }
    delay(1);
  }
  out[n] = '\0';
  return (n > 0);
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

// ==============================
// COMTRADE mínimo (a,b)
// ==============================
struct AnalogCh {
  float a = 1.0f;
  float b = 0.0f;
  inline float rawToEng(int16_t raw) const { return a * (float)raw + b; }
};

struct ComtradeCfg {
  float freqHz = 60.0f;
  float sampRateHz = 2000.0f;
  float timeMult = 1.0f;
  AnalogCh chV; // analógico 0 (tensão)
  AnalogCh chI; // analógico 1 (corrente)
};

ComtradeCfg g_cfg;

// ==============================
// Mapeamento eng -> volts com clip pelo pico
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
// Recebe CFG do PC
// ==============================
static bool waitCfgFromPC(uint32_t timeoutMs) {
  char line[256];
  char* parts[16];

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!readLineSerial(line, sizeof(line), 2000)) continue;

    if (strncmp(line, "CFG", 3) == 0) {
      int n = splitCSV(line, parts, 16);
      // CFG,fs,freq,timeMult,aV,bV,aI,bI
      if (n < 8) return false;

      g_cfg.sampRateHz = strtof(parts[1], nullptr);
      g_cfg.freqHz     = strtof(parts[2], nullptr);
      g_cfg.timeMult   = strtof(parts[3], nullptr);
      g_cfg.chV.a      = strtof(parts[4], nullptr);
      g_cfg.chV.b      = strtof(parts[5], nullptr);
      g_cfg.chI.a      = strtof(parts[6], nullptr);
      g_cfg.chI.b      = strtof(parts[7], nullptr);

      if (g_cfg.sampRateHz < 1.0f) g_cfg.sampRateHz = 2000.0f;
      if (g_cfg.freqHz < 1.0f) g_cfg.freqHz = 60.0f;
      if (g_cfg.timeMult <= 0.0f) g_cfg.timeMult = 1.0f;

      return true;
    }
  }
  return false;
}

// ==============================
// Calcula picos nominais (5 ciclos) usando buffers
// (mesma ideia do seu original, só que o "BDAT" agora vem da Serial)
// ==============================
static bool computeNominalFromBuffered(
  const int16_t* bufV,
  const int16_t* bufI,
  uint32_t nSamples,
  uint32_t samplesPerCycle,
  float &out_V_nom_peak,
  float &out_I_nom_peak,
  float &out_V_clip_peak,
  float &out_I_clip_peak
) {
  out_V_nom_peak = 0.0f;
  out_I_nom_peak = 0.0f;
  out_V_clip_peak = 0.0f;
  out_I_clip_peak = 0.0f;

  if (!bufV || !bufI || nSamples < samplesPerCycle) return false;
  if (samplesPerCycle < 8) samplesPerCycle = 8;

  double sumPeakV = 0.0;
  double sumPeakI = 0.0;
  uint32_t cyclesCounted = 0;

  float cyclePeakV = 0.0f;
  float cyclePeakI = 0.0f;
  uint32_t idxInCycle = 0;

  for (uint32_t i = 0; i < nSamples; i++) {
    float engV = g_cfg.chV.rawToEng(bufV[i]);
    float engI = g_cfg.chI.rawToEng(bufI[i]);

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
// Processa 1 amostra (reproduz + mede + loga)
// ==============================
static void processSampleToHW(
  File &csv,
  uint32_t sampleNum,
  uint32_t &t_accum_us,
  uint32_t dt_us,
  int16_t rawV,
  int16_t rawI,
  float vClipPeak,
  float iClipPeak
) {
  if (dt_us > 0) busyWaitMicros(dt_us);
  t_accum_us += dt_us;

  float engV = g_cfg.chV.rawToEng(rawV);
  float engI = g_cfg.chI.rawToEng(rawI);

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
  csv.print(t_accum_us); csv.print(",");
  csv.print(dt_us); csv.print(",");
  csv.print(dacV); csv.print(",");
  csv.print(dacI); csv.print(",");
  csv.print(adcV_raw); csv.print(",");
  csv.print(adcI_raw); csv.print(",");
  csv.print(adcV_V, 6); csv.print(",");
  csv.print(adcI_V, 6); csv.print(",");
  csv.print(clipV); csv.print(",");
  csv.println(clipI);
}

void setup() {
  Serial.begin(921600);
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

  // SD só para salvar CSV
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ Falha ao montar SD");
    while (true) delay(1000);
  }

  if (!SD.exists("/comtrade_binary")) SD.mkdir("/comtrade_binary");

  SD.remove(OUT_CSV_PATH);
  File csv = SD.open(OUT_CSV_PATH, FILE_WRITE);
  if (!csv) {
    Serial.println("❌ Não abriu output.csv");
    while (true) delay(1000);
  }
  csv.println("sample,t_us,dt_us,dacV_code,dacI_code,adcV_raw,adcI_raw,adcV_V,adcI_V,clipV,clipI");

  // Zera saídas em 1.65V
  uint16_t mid = voltsToDac12(V_MID);
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

  Serial.println("=== ESP32 COMTRADE Player (PC->Serial->MCP4728) + CSV no SD ===");
  Serial.println("Envie: CFG,fs,freq,timeMult,aV,bV,aI,bI");
  Serial.println("Depois: S,sampleNum,rawV,rawI");
  Serial.println("Finalize: END");

  if (!waitCfgFromPC(120000)) {
    Serial.println("❌ Timeout/falha recebendo CFG");
    csv.close();
    while (true) delay(1000);
  }

  uint32_t samplesPerCycle = (uint32_t)lroundf(g_cfg.sampRateHz / g_cfg.freqHz);
  if (samplesPerCycle < 8) samplesPerCycle = 8;

  uint32_t Ncal = samplesPerCycle * (uint32_t)NOM_CYCLES;
  if (Ncal < samplesPerCycle) Ncal = samplesPerCycle;
  if (Ncal > 2000) Ncal = 2000; // segurança RAM (igual sua filosofia)

  Serial.printf("CFG OK: fs=%.2fHz f=%.2fHz timeMult=%.6f | samplesPerCycle=%u | Ncal=%u\n",
                g_cfg.sampRateHz, g_cfg.freqHz, g_cfg.timeMult,
                (unsigned)samplesPerCycle, (unsigned)Ncal);

  // Buffer para os primeiros 5 ciclos
  int16_t *bufV = (int16_t*)malloc(Ncal * sizeof(int16_t));
  int16_t *bufI = (int16_t*)malloc(Ncal * sizeof(int16_t));
  uint32_t *bufS = (uint32_t*)malloc(Ncal * sizeof(uint32_t));
  if (!bufV || !bufI || !bufS) {
    Serial.println("❌ Falha malloc buffer calibração");
    csv.close();
    while (true) delay(1000);
  }

  // Recebe Ncal amostras para calibrar nominal
  char line[256];
  char* parts[8];
  uint32_t got = 0;

  while (got < Ncal) {
    if (!readLineSerial(line, sizeof(line), 120000)) {
      Serial.println("❌ Timeout recebendo amostras para calibração");
      csv.close();
      while (true) delay(1000);
    }

    if (strncmp(line, "END", 3) == 0) break;
    if (strncmp(line, "S", 1) != 0) continue;

    int n = splitCSV(line, parts, 8);
    if (n < 4) continue;

    uint32_t sampleNum = (uint32_t)strtoul(parts[1], nullptr, 10);
    int32_t rv = (int32_t)strtol(parts[2], nullptr, 10);
    int32_t ri = (int32_t)strtol(parts[3], nullptr, 10);

    if (rv < -32768) rv = -32768;
    if (rv >  32767) rv =  32767;
    if (ri < -32768) ri = -32768;
    if (ri >  32767) ri =  32767;

    bufS[got] = sampleNum;
    bufV[got] = (int16_t)rv;
    bufI[got] = (int16_t)ri;
    got++;
  }

  if (got < samplesPerCycle) {
    Serial.println("❌ Poucas amostras para calibrar nominal");
    csv.close();
    while (true) delay(1000);
  }

  float vNomPeak = 0.0f, iNomPeak = 0.0f;
  float vClipPeak = 0.0f, iClipPeak = 0.0f;

  if (!computeNominalFromBuffered(bufV, bufI, got, samplesPerCycle, vNomPeak, iNomPeak, vClipPeak, iClipPeak)) {
    Serial.println("❌ Falha ao calcular nominal (buffer)");
    csv.close();
    while (true) delay(1000);
  }

  Serial.printf("Nominais (média %d ciclos): V_nom_peak=%.6f | I_nom_peak=%.6f\n", NOM_CYCLES, vNomPeak, iNomPeak);
  Serial.printf("Limites: V_clip=%.6f (%.1fx) | I_clip=%.6f (%.1fx)\n", vClipPeak, V_FAULT_LIMIT_MULT, iClipPeak, I_FAULT_LIMIT_MULT);

  uint32_t dt_us_base = (uint32_t)lroundf((1000000.0f / g_cfg.sampRateHz) * g_cfg.timeMult);

  // Reproduz + loga o buffer inicial
  uint32_t t_accum_us = 0;
  for (uint32_t i = 0; i < got; i++) {
    uint32_t dt = (i == 0) ? 0 : dt_us_base;
    processSampleToHW(csv, bufS[i], t_accum_us, dt, bufV[i], bufI[i], vClipPeak, iClipPeak);
  }

  free(bufV); free(bufI); free(bufS);

  Serial.println("▶️ Calibração OK. Continue enviando amostras... (END para finalizar)");

  // Stream até END
  while (true) {
    if (!readLineSerial(line, sizeof(line), 120000)) {
      Serial.println("❌ Timeout recebendo stream");
      break;
    }

    if (strncmp(line, "END", 3) == 0) {
      Serial.println("✅ END recebido");
      break;
    }

    if (strncmp(line, "S", 1) != 0) continue;

    int n = splitCSV(line, parts, 8);
    if (n < 4) continue;

    uint32_t sampleNum = (uint32_t)strtoul(parts[1], nullptr, 10);
    int32_t rv = (int32_t)strtol(parts[2], nullptr, 10);
    int32_t ri = (int32_t)strtol(parts[3], nullptr, 10);

    if (rv < -32768) rv = -32768;
    if (rv >  32767) rv =  32767;
    if (ri < -32768) ri = -32768;
    if (ri >  32767) ri =  32767;

    processSampleToHW(csv, sampleNum, t_accum_us, dt_us_base, (int16_t)rv, (int16_t)ri, vClipPeak, iClipPeak);
  }

  csv.flush();
  csv.close();

  Serial.print("✅ output.csv salvo no SD: ");
  Serial.println(OUT_CSV_PATH);

  while (true) delay(1000);
}

void loop() {
  delay(1000);
}