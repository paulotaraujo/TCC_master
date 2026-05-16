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

// Loopback ADC
#define ADC_PIN_V     34
#define ADC_PIN_I     35

// ==============================
// Configurações do Buffer de Pré-Falta
// ==============================
#define PRE_FAULT_CYCLES 5                    // 5 ciclos de pré-falta
#define MAX_PREFAULT_SAMPLES 5000              // Buffer seguro
#define SERIAL_CHECK_INTERVAL 100               // Verificar serial a cada N samples

static const char* CFG_PATH     = "/comtrade_binary/export.cfg";
static const char* BDAT_PATH    = "/comtrade_binary/export.bdat";
static const char* OUT_CSV_PATH = "/comtrade_binary/output.csv";

Adafruit_MCP4728 dac;

// ==============================
// Reprodução / timing
// ==============================
static const uint16_t DAC_SETTLE_US = 80;
static const float    V_MID   = 1.65f;
static const float    V_AMP   = 1.55f;

// ==============================
// Auto-nominal por janela fixa
// ==============================
static const int   NOM_CYCLES = 5;
static const float V_FAULT_LIMIT_MULT = 2.0f;
static const float I_FAULT_LIMIT_MULT = 3.0f;

// ==============================
// Estrutura do Buffer Circular
// ==============================
struct PreFaultBuffer {
  uint16_t dacV[MAX_PREFAULT_SAMPLES];
  uint16_t dacI[MAX_PREFAULT_SAMPLES];
  uint32_t dt_us[MAX_PREFAULT_SAMPLES];
  uint32_t t_accum[MAX_PREFAULT_SAMPLES];
  int size = 0;
  int currentIdx = 0;
  
  void addSample(uint16_t v, uint16_t i, uint32_t dt, uint32_t t_acc) {
    if (size < MAX_PREFAULT_SAMPLES) {
      dacV[size] = v;
      dacI[size] = i;
      dt_us[size] = dt;
      t_accum[size] = t_acc;
      size++;
    }
  }
  
  void playNext(uint16_t &v, uint16_t &i, uint32_t &dt) {
    v = dacV[currentIdx];
    i = dacI[currentIdx];
    dt = dt_us[currentIdx];
    currentIdx = (currentIdx + 1) % size;
  }
  
  void reset() {
    currentIdx = 0;
  }
  
  uint32_t getLastTimestamp() {
    return (size > 0) ? t_accum[size-1] : 0;
  }
  
  int getSize() {
    return size;
  }
  
  uint32_t getLastDt() {
    return (size > 0) ? dt_us[size-1] : 0;
  }
};

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
// COMTRADE
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
// Implementações das funções COMTRADE
// ==============================
static bool parseCfgFromSD(ComtradeCfg &cfg) {
  File f = SD.open(CFG_PATH, FILE_READ);
  if (!f) return false;

  char line[256];
  char* parts[32];

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; }

  if (!readLine(f, line, sizeof(line))) { f.close(); return false; }
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
// Variáveis Globais
// ==============================
ComtradeCfg g_cfg;
PreFaultBuffer preFaultBuffer;
bool reproductionMode = false;
uint32_t lastTimestamp = 0;
float vClipPeak = 0.0f, iClipPeak = 0.0f;
uint32_t baseTimestamp = 0;
uint32_t loopCount = 0;

// ==============================
// Setup
// ==============================
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

  float vNomPeak = 0.0f, iNomPeak = 0.0f;
  if (!computeNominalPeaksFromFirstCycles(g_cfg, vNomPeak, iNomPeak, vClipPeak, iClipPeak)) {
    Serial.println("❌ Falha em calcular picos nominais");
    while (true) delay(1000);
  }

  Serial.printf("CFG: nA=%d nD=%d fs=%.2fHz f=%.2fHz\n",
                g_cfg.nAnalog, g_cfg.nDigital, g_cfg.sampRateHz, g_cfg.freqHz);
  Serial.printf("Nominais: V_nom=%.6f | I_nom=%.6f\n", vNomPeak, iNomPeak);
  Serial.printf("Limites: V_clip=%.6f | I_clip=%.6f\n", vClipPeak, iClipPeak);

  // Calcular samples dos primeiros N ciclos
  uint32_t samplesPerCycle = (uint32_t)lroundf(g_cfg.sampRateHz / g_cfg.freqHz);
  uint32_t preFaultSamples = samplesPerCycle * PRE_FAULT_CYCLES;
  
  Serial.printf("Samples por ciclo: %d, Total pré-falta: %d\n", 
                samplesPerCycle, preFaultSamples);

  // Abrir arquivos
  SD.remove(OUT_CSV_PATH);
  File csv = SD.open(OUT_CSV_PATH, FILE_WRITE);
  if (!csv) {
    Serial.println("❌ Não abriu output.csv");
    while (true) delay(1000);
  }
  csv.println("sample,t_us,dt_us,dacV_code,dacI_code,adcV_raw,adcI_raw,adcV_V,adcI_V,clipV,clipI,mode");

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

  // Inicializar DAC
  uint16_t mid = voltsToDac12(V_MID);
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, 
                      MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, 
                      MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

  // ==============================
  // FASE 1: Carregar buffer com N ciclos
  // ==============================
  Serial.printf("Carregando buffer com pré-falta (primeiros %d ciclos)...\n", PRE_FAULT_CYCLES);
  
  int64_t prevTs = 0;
  bool first = true;
  uint32_t t_accum = 0;

  for (uint32_t i = 0; i < preFaultSamples && i < g_cfg.totalRecords; i++) {
    int rd = bdat.read(recBuf, g_cfg.recSize);
    if (rd != (int)g_cfg.recSize) {
      Serial.println("❌ Leitura BDAT incompleta no pré-falta");
      break;
    }

    int off = 0;
    off += 4; // Pular sample number

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
    } else {
      first = false;
      dt_us = 0;
    }
    prevTs = ts;

    // Ler valores engineering
    float engV = 0.0f, engI = 0.0f;
    for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;
      if (ch == 0) engV = g_cfg.chV.rawToEng(raw);
      else if (ch == 1) engI = g_cfg.chI.rawToEng(raw);
    }

    // Mapear para DAC
    float vOutV = mapEngToVolts_ClipPeakFS(engV, vClipPeak);
    float vOutI = mapEngToVolts_ClipPeakFS(engI, iClipPeak);
    uint16_t dacV = voltsToDac12(vOutV);
    uint16_t dacI = voltsToDac12(vOutI);

    // Adicionar ao buffer
    t_accum += dt_us;
    preFaultBuffer.addSample(dacV, dacI, dt_us, t_accum);
  }

  Serial.printf("Buffer carregado com %d samples (%.3f segundos)\n", 
                preFaultBuffer.getSize(), 
                preFaultBuffer.getLastTimestamp() / 1000000.0f);
  
  Serial.println("\n=== Sistema Pronto ===");
  Serial.println("Comandos disponíveis:");
  Serial.println("  'f' - Iniciar reprodução da falta");
  Serial.println("  'r' - Reiniciar ciclo de pré-falta");
  Serial.println("  's' - Status do sistema");
  Serial.println("Aguardando comando...\n");

  // ==============================
  // FASE 2: Loop de pré-falta (reproduz buffer continuamente)
  // ==============================
  preFaultBuffer.reset();
  uint32_t sampleCount = 0;
  bool waitingForTrigger = true;
  
  // Variáveis para controle de tempo contínuo
  uint32_t loopStartTime = micros();
  uint32_t idealTime = 0;
  uint32_t lastSampleTime = 0;
  uint32_t bufferDuration = preFaultBuffer.getLastTimestamp();
  t_accum = 0;

  while (waitingForTrigger) {
    // Reproduzir próximo sample do buffer
    uint16_t dacV, dacI;
    uint32_t dt_us;
    preFaultBuffer.playNext(dacV, dacI, dt_us);
    
    // Calcular tempo ideal baseado no buffer (contínuo)
    if (sampleCount == 0) {
      // Primeiro sample de cada loop - continua do tempo anterior
      idealTime = loopCount * bufferDuration;
      lastSampleTime = idealTime;
    } else {
      idealTime = lastSampleTime + dt_us;
    }
    
    // Tempo real decorrido
    uint32_t realTime = micros() - loopStartTime;
    
    // Ajustar para manter sincronismo (compensar deriva)
    if (idealTime > realTime) {
      uint32_t waitTime = idealTime - realTime;
      if (waitTime > 0 && waitTime < 10000) {  // Máx 10ms de espera
        busyWaitMicros(waitTime);
      }
    }
    
    // Atualizar para próximo sample
    lastSampleTime = idealTime;
    t_accum = idealTime;  // Tempo contínuo para CSV
    
    // Enviar para DAC
    dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, 
                        MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, 
                        MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
    
    delayMicroseconds(DAC_SETTLE_US);
    
    // Ler ADC (loopback)
    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);
    
    // Salvar no CSV
    csv.print(sampleCount); csv.print(",");
    csv.print(t_accum); csv.print(",");
    csv.print(dt_us); csv.print(",");
    csv.print(dacV); csv.print(",");
    csv.print(dacI); csv.print(",");
    csv.print(adcV_raw); csv.print(",");
    csv.print(adcI_raw); csv.print(",");
    csv.print(adcV_V, 6); csv.print(",");
    csv.print(adcI_V, 6); csv.print(",");
    csv.print("0"); csv.print(","); // clipV
    csv.print("0"); csv.print(","); // clipI
    
    // Identificar modo
    if (sampleCount < preFaultBuffer.size) {
      csv.println("PRE");
    } else {
      csv.println("PRE_LOOP");
    }
    
    sampleCount++;
    
    // Verificar se completou um ciclo do buffer
    if (sampleCount >= preFaultBuffer.size) {
      sampleCount = 0;
      loopCount++;
      preFaultBuffer.reset();
    }
    
    // Verificar comando serial
    if (sampleCount % SERIAL_CHECK_INTERVAL == 0 || sampleCount == 0) {
      if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'f' || cmd == 'F') {
          waitingForTrigger = false;
          Serial.println("\n=== INICIANDO REPRODUÇÃO DA FALTA ===");
          
          // Registrar transição no CSV
          csv.print(sampleCount); csv.print(",");
          csv.print(t_accum + dt_us); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.print("0"); csv.print(",");
          csv.println("TRANSITION");
          
          // Posicionar no início da falta
          bdat.seek(preFaultSamples * g_cfg.recSize);
          
          // Resetar para calcular novos dts
          prevTs = 0;
          first = true;
          
        } else if (cmd == 'r' || cmd == 'R') {
          // Reiniciar mas manter tempo contínuo
          sampleCount = 0;
          loopCount = 0;
          preFaultBuffer.reset();
          loopStartTime = micros();
          idealTime = 0;
          lastSampleTime = 0;
          t_accum = 0;
          Serial.println("Reiniciando ciclo de pré-falta com tempo contínuo...");
          
        } else if (cmd == 's' || cmd == 'S') {
          uint32_t currentTime = micros() - loopStartTime;
          Serial.printf("Status: Loop %d, Sample %d, Tempo real: %.3fs, Tempo ideal: %.3fs\n", 
                       loopCount, sampleCount, 
                       currentTime / 1000000.0f, 
                       idealTime / 1000000.0f);
        }
      }
    }
  }

  // ==============================
  // FASE 3: Reproduzir o restante do arquivo (falta + pós-falta)
  // ==============================
  Serial.printf("Reproduzindo %d amostras restantes...\n", 
                g_cfg.totalRecords - preFaultSamples);

  for (uint32_t i = preFaultSamples; i < g_cfg.totalRecords; i++) {
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
    } else {
      // Primeiro sample da falta: usar último dt do buffer
      dt_us = preFaultBuffer.getLastDt();
      first = false;
    }
    
    // Manter tempo contínuo
    t_accum += dt_us;
    
    // Esperar o tempo correto
    if (dt_us > DAC_SETTLE_US) {
      busyWaitMicros(dt_us - DAC_SETTLE_US);
    }
    
    prevTs = ts;

    // Ler valores engineering
    float engV = 0.0f, engI = 0.0f;
    for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;
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

    dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, 
                        MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, 
                        MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);

    delayMicroseconds(DAC_SETTLE_US);

    uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
    uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
    float adcV_V = adcToVoltsCal(ADC_PIN_V);
    float adcI_V = adcToVoltsCal(ADC_PIN_I);

    csv.print(sampleNum); csv.print(",");
    csv.print(t_accum); csv.print(",");
    csv.print(dt_us); csv.print(",");
    csv.print(dacV); csv.print(",");
    csv.print(dacI); csv.print(",");
    csv.print(adcV_raw); csv.print(",");
    csv.print(adcI_raw); csv.print(",");
    csv.print(adcV_V, 6); csv.print(",");
    csv.print(adcI_V, 6); csv.print(",");
    csv.print(clipV); csv.print(",");
    csv.print(clipI); csv.print(",");
    csv.println("FAULT");
    
    // Verificar comandos durante a falta
    if (i % SERIAL_CHECK_INTERVAL == 0 && Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'q' || cmd == 'Q') {
        Serial.println("Reprodução interrompida pelo usuário");
        break;
      }
    }
  }

  // Finalizar
  bdat.close();
  csv.flush();
  csv.close();

  Serial.print("\n✅ Reprodução concluída! CSV salvo em: ");
  Serial.println(OUT_CSV_PATH);
  Serial.println("Envie 'r' para reiniciar ou qualquer tecla para finalizar");

  // Aguardar comando para reiniciar
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'r' || cmd == 'R') {
        Serial.println("Reiniciando sistema...");
        delay(100);
        ESP.restart();
      }
    }
    delay(100);
  }
}

void loop() {
  // Não utilizado - tudo ocorre no setup
  delay(1000);
}