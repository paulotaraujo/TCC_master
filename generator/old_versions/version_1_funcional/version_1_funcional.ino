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
static const float    V_AMP   = 1.55f;      // headroom (evita bater 0/3.3)

// ==============================
// Auto-nominal por janela fixa (60 Hz)
// ==============================
static const int   NOM_CYCLES = 5;              // 5 ciclos para estimar amplitude de pico nominal
static const float V_FAULT_LIMIT_MULT = 2.0f;   // tensão clipa acima de 2x nominal
static const float I_FAULT_LIMIT_MULT = 3.0f;   // corrente clipa acima de 3x nominal

// ==============================
// Estados do sistema
// ==============================
enum SystemState {
  STATE_IDLE,           // Envia 1.65V apenas
  STATE_LOOP_MODE,      // Reproduz primeiro ciclo em loop
  STATE_TRANSITION,     // Transição do loop para full (última amostra do ciclo)
  STATE_FULL_MODE       // Reproduz arquivo completo uma vez
};

static SystemState g_state = STATE_IDLE;
static bool g_logging_enabled = false;
static bool g_transition_pending = false;
static bool g_quit_pending = false;

// ==============================
// Buffers para o primeiro ciclo
// ==============================
static float* g_firstCycleV = nullptr;
static float* g_firstCycleI = nullptr;
static uint32_t g_samplesPerCycle = 0;
static uint32_t g_cycleIndex = 0;
static uint32_t g_fullFileIndex = 0;
static uint32_t g_firstCycleSize = 0;

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

static ComtradeCfg g_cfg;
static float g_vNomPeak = 0.0f, g_iNomPeak = 0.0f;
static float g_vClipPeak = 0.0f, g_iClipPeak = 0.0f;

static bool parseCfgFromSD(ComtradeCfg &cfg);
static bool computeBdatLayout(ComtradeCfg &cfg);
static bool computeNominalPeaksFromFirstCycles(ComtradeCfg &cfg, float &out_V_nom_peak, float &out_I_nom_peak, float &out_V_clip_peak, float &out_I_clip_peak);
static bool loadFirstCycleToRAM(ComtradeCfg &cfg, float* &bufV, float* &bufI, uint32_t &size);
static inline float mapEngToVolts_ClipPeakFS(float eng, float clipPeak);

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

static bool computeNominalPeaksFromFirstCycles(ComtradeCfg &cfg, float &out_V_nom_peak, float &out_I_nom_peak, float &out_V_clip_peak, float &out_I_clip_peak) {
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

static bool loadFirstCycleToRAM(ComtradeCfg &cfg, float* &bufV, float* &bufI, uint32_t &size) {
  if (cfg.sampRateHz < 1.0f || cfg.freqHz < 1.0f) return false;
  
  size = (uint32_t)lroundf(cfg.sampRateHz / cfg.freqHz);
  if (size < 8) size = 8;
  if (size > cfg.totalRecords) size = cfg.totalRecords;
  
  bufV = new float[size];
  bufI = new float[size];
  if (!bufV || !bufI) return false;
  
  File bdat = SD.open(BDAT_PATH, FILE_READ);
  if (!bdat) return false;
  
  static uint8_t recBuf[512];
  if (cfg.recSize > sizeof(recBuf)) { bdat.close(); return false; }
  
  for (uint32_t i = 0; i < size; i++) {
    int rd = bdat.read(recBuf, cfg.recSize);
    if (rd != (int)cfg.recSize) { 
      delete[] bufV;
      delete[] bufI;
      bdat.close(); 
      return false; 
    }
    
    int off = 0;
    off += 4;
    off += cfg.ts64 ? 8 : 4;
    
    for (int ch = 0; ch < cfg.nAnalog; ch++) {
      int16_t raw = 0;
      memcpy(&raw, recBuf + off, 2);
      off += 2;
      if (ch == 0) bufV[i] = cfg.chV.rawToEng(raw);
      else if (ch == 1) bufI[i] = cfg.chI.rawToEng(raw);
    }
  }
  
  bdat.close();
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
// Processamento de comandos Serial
// ==============================
void processSerialCommands() {
  if (!Serial.available()) return;
  
  char cmd = tolower(Serial.read());
  
  switch (cmd) {
    case 's':  // Start loop mode
      if (g_state == STATE_IDLE) {
        g_state = STATE_LOOP_MODE;
        g_cycleIndex = 0;
        g_logging_enabled = true;
        g_transition_pending = false;
        g_quit_pending = false;
        Serial.println("▶️ Modo loop: reproduzindo primeiro ciclo continuamente");
      }
      break;
      
    case 'f':  // Start full mode (after current cycle)
      if (g_state == STATE_LOOP_MODE) {
        g_transition_pending = true;
        Serial.println("⏸️ Transição pendente: aguardando fim do ciclo atual...");
      }
      break;
      
    case 'q':  // Quit - return to idle
      if (g_state != STATE_IDLE) {
        g_quit_pending = true;
        Serial.println("⏹️ Parando reprodução...");
      }
      break;
      
    default:
      break;
  }
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  delay(150);
  
  Serial.println("\n=== Inicializando Sistema COMTRADE Player ===");
  
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
    Serial.println("❌ Falha em calcular picos nominais");
    while (true) delay(1000);
  }
  
  // Carrega primeiro ciclo para RAM
  if (!loadFirstCycleToRAM(g_cfg, g_firstCycleV, g_firstCycleI, g_firstCycleSize)) {
    Serial.println("❌ Falha ao carregar primeiro ciclo");
    while (true) delay(1000);
  }
  g_samplesPerCycle = g_firstCycleSize;
  
  Serial.printf("CFG: nA=%d nD=%d fs=%.2fHz f=%.2fHz\n",
                g_cfg.nAnalog, g_cfg.nDigital, g_cfg.sampRateHz, g_cfg.freqHz);
  Serial.printf("Primeiro ciclo: %u amostras\n", g_firstCycleSize);
  Serial.printf("Nominais: V_peak=%.6f | I_peak=%.6f\n", g_vNomPeak, g_iNomPeak);
  Serial.printf("Limites: V_clip=%.6f (%.1fx) | I_clip=%.6f (%.1fx)\n",
                g_vClipPeak, V_FAULT_LIMIT_MULT, g_iClipPeak, I_FAULT_LIMIT_MULT);
  
  // Estado inicial: 1.65V
  uint16_t mid = voltsToDac12(V_MID);
  dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
  
  Serial.println("✅ Sistema pronto. Comandos: 's' (loop), 'f' (full), 'q' (parar)");
  Serial.println("Estado inicial: 1.65V\n");
}

// ==============================
// Loop principal
// ==============================
void loop() {
  static File bdatFile;
  static File csvFile;
  static bool firstSample = true;
  static int64_t prevTs = 0;
  static uint32_t t_accum = 0;
  static uint32_t fullFileIndex = 0;
  static bool fullFileStarted = false;
  
  processSerialCommands();
  
  // Se quit pendente, volta para IDLE
  if (g_quit_pending && g_state != STATE_IDLE) {
    // Fecha arquivos se abertos
    if (bdatFile) bdatFile.close();
    if (csvFile) {
      csvFile.flush();
      csvFile.close();
    }
    
    // Libera buffers
    if (g_firstCycleV) { delete[] g_firstCycleV; g_firstCycleV = nullptr; }
    if (g_firstCycleI) { delete[] g_firstCycleI; g_firstCycleI = nullptr; }
    
    // Volta para 1.65V
    uint16_t mid = voltsToDac12(V_MID);
    dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
    dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
    
    g_state = STATE_IDLE;
    g_logging_enabled = false;
    g_quit_pending = false;
    fullFileStarted = false;
    firstSample = true;
    
    Serial.println("⏹️ Sistema em IDLE - 1.65V");
  }
  
  // ===== ESTADO IDLE =====
  if (g_state == STATE_IDLE) {
    delay(10);  // Reduz consumo
    return;
  }
  
  // ===== INICIALIZAÇÃO DOS MODOS ATIVOS =====
  if (g_state == STATE_LOOP_MODE && !fullFileStarted) {
    // Abre CSV se logging habilitado e ainda não aberto
    if (g_logging_enabled && !csvFile) {
      SD.remove(OUT_CSV_PATH);
      csvFile = SD.open(OUT_CSV_PATH, FILE_WRITE);
      if (csvFile) {
        csvFile.println("sample,t_us,dt_us,dacV_code,dacI_code,adcV_raw,adcI_raw,adcV_V,adcI_V,clipV,clipI,state");
        Serial.println("📝 Gravação CSV iniciada");
      }
    }
    fullFileStarted = true;
    firstSample = true;
    t_accum = 0;
  }
  
  // ===== AMOSTRAGEM E REPRODUÇÃO =====
  uint32_t dt_us = 0;
  
  if (!firstSample) {
    // Calcula dt baseado na taxa de amostragem
    dt_us = (uint32_t)lroundf(1000000.0f / g_cfg.sampRateHz);
    if (dt_us > 0) busyWaitMicros(dt_us);
  } else {
    firstSample = false;
  }
  t_accum += dt_us;
  
  float engV = 0.0f;
  float engI = 0.0f;
  uint32_t currentSample = 0;
  String stateStr;
  
  // ===== MÁQUINA DE ESTADOS =====
  switch (g_state) {
    case STATE_LOOP_MODE:
      {
        // Reproduz do buffer do primeiro ciclo
        engV = g_firstCycleV[g_cycleIndex];
        engI = g_firstCycleI[g_cycleIndex];
        currentSample = g_cycleIndex;
        stateStr = "LOOP";
        
        g_cycleIndex++;
        if (g_cycleIndex >= g_firstCycleSize) {
          g_cycleIndex = 0;  // Volta ao início do ciclo
          
          // Se transição pendente, vai para FULL ao completar o ciclo
          if (g_transition_pending) {
            g_state = STATE_FULL_MODE;
            g_transition_pending = false;
            fullFileIndex = g_firstCycleSize;  // Começa do próximo sample após o primeiro ciclo
            Serial.println("▶️ Transição para modo FULL");
            
            // Abre arquivo BDAT para leitura do restante
            if (bdatFile) bdatFile.close();
            bdatFile = SD.open(BDAT_PATH, FILE_READ);
            if (bdatFile) {
              // Posiciona no início do segundo ciclo
              bdatFile.seek(g_firstCycleSize * g_cfg.recSize);
            }
          }
        }
      }
      break;
      
    case STATE_FULL_MODE:
      {
        stateStr = "FULL";
        
        if (!bdatFile) {
          // Se não abriu ainda, abre e posiciona
          bdatFile = SD.open(BDAT_PATH, FILE_READ);
          if (!bdatFile) {
            Serial.println("❌ Erro ao abrir BDAT para modo FULL");
            g_state = STATE_IDLE;
            break;
          }
          bdatFile.seek(g_firstCycleSize * g_cfg.recSize);  // Pula primeiro ciclo
          fullFileIndex = g_firstCycleSize;
        }
        
        // Lê próximo registro
        static uint8_t recBuf[512];
        int rd = bdatFile.read(recBuf, g_cfg.recSize);
        
        if (rd != (int)g_cfg.recSize) {
          // Fim do arquivo
          Serial.println("✅ Fim da reprodução completa");
          bdatFile.close();
          
          // Volta para IDLE
          uint16_t mid = voltsToDac12(V_MID);
          dac.setChannelValue(MCP4728_CHANNEL_A, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
          dac.setChannelValue(MCP4728_CHANNEL_B, mid, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
          
          // Finaliza CSV
          if (csvFile) {
            csvFile.flush();
            csvFile.close();
            Serial.print("✅ CSV salvo: ");
            Serial.println(OUT_CSV_PATH);
          }
          
          g_logging_enabled = false;
          g_state = STATE_IDLE;
          fullFileStarted = false;
          break;
        }
        
        // Parse do registro
        int off = 0;
        uint32_t sampleNum = 0;
        memcpy(&sampleNum, recBuf + off, 4);
        off += 4;
        
        off += g_cfg.ts64 ? 8 : 4;  // Pula timestamp
        
        for (int ch = 0; ch < g_cfg.nAnalog; ch++) {
          int16_t raw = 0;
          memcpy(&raw, recBuf + off, 2);
          off += 2;
          if (ch == 0) engV = g_cfg.chV.rawToEng(raw);
          else if (ch == 1) engI = g_cfg.chI.rawToEng(raw);
        }
        
        currentSample = sampleNum;
        fullFileIndex++;
      }
      break;
      
    default:
      break;
  }
  
  // ===== MAPEAMENTO PARA DAC =====
  int clipV = (fabsf(engV) > g_vClipPeak) ? 1 : 0;
  int clipI = (fabsf(engI) > g_iClipPeak) ? 1 : 0;
  
  float vOutV = mapEngToVolts_ClipPeakFS(engV, g_vClipPeak);
  float vOutI = mapEngToVolts_ClipPeakFS(engI, g_iClipPeak);
  
  uint16_t dacV = voltsToDac12(vOutV);
  uint16_t dacI = voltsToDac12(vOutI);
  
  // Atualiza DAC
  dac.setChannelValue(MCP4728_CHANNEL_A, dacV, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, dacI, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_NORMAL, true);
  
  delayMicroseconds(DAC_SETTLE_US);
  
  // ===== LEITURA ADC (loopback) =====
  uint16_t adcV_raw = (uint16_t)analogRead(ADC_PIN_V);
  uint16_t adcI_raw = (uint16_t)analogRead(ADC_PIN_I);
  float adcV_V = adcToVoltsCal(ADC_PIN_V);
  float adcI_V = adcToVoltsCal(ADC_PIN_I);
  
  // ===== LOG CSV =====
  if (g_logging_enabled && csvFile) {
    csvFile.print(currentSample); csvFile.print(",");
    csvFile.print(t_accum);       csvFile.print(",");
    csvFile.print(dt_us);         csvFile.print(",");
    csvFile.print(dacV);          csvFile.print(",");
    csvFile.print(dacI);          csvFile.print(",");
    csvFile.print(adcV_raw);      csvFile.print(",");
    csvFile.print(adcI_raw);      csvFile.print(",");
    csvFile.print(adcV_V, 6);     csvFile.print(",");
    csvFile.print(adcI_V, 6);     csvFile.print(",");
    csvFile.print(clipV);         csvFile.print(",");
    csvFile.print(clipI);         csvFile.print(",");
    csvFile.println(stateStr);
  }
  
  // Pequeno delay para evitar sobrecarga da serial
  delay(1);
}