// =============================================================================
// v_6.ino - ESP32 COMTRADE local processor + DAC playback
//
// PC side only uploads .cfg/.bdat and sends playback commands.
// ESP32 stores files in LittleFS, processes COMTRADE locally, then unlocks:
//   s  prefault loop
//   f  full once after current prefault cycle
//   p  full once
//   q  idle
//   x  idle/stop
//   t  test sine
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <LittleFS.h>
#include <math.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#ifndef MCP4728_PD_NORMAL
  #define MCP4728_PD_NORMAL MCP4728_PD_MODE_NORMAL
#endif

#define I2C_SDA       21
#define I2C_SCL       22
#define MCP4728_ADDR  0x60
#define I2C_FREQ_HZ   1000000UL

static const uint32_t SERIAL_BAUD = 921600;
static const uint32_t UPLOAD_IDLE_TIMEOUT_MS = 15000;

static const char *CFG_PATH  = "/input.cfg";
static const char *BDAT_PATH = "/input.bdat";

static const uint8_t FLAG_IDLE_AFTER  = 0x01;
static const uint8_t FLAG_RESET_CLOCK = 0x02;

static const float V_MID = 1.65f;
static const float V_AMP = 1.55f;
static const float DAC_VREF = 3.3f;
static const uint16_t DAC_MAX = 4095;
static const uint16_t DAC_IDLE_CODE = 2048;

static const uint8_t NOM_CYCLES = 5;
static const float V_FAULT_LIMIT_MULT = 5.0f;
static const float I_FAULT_LIMIT_MULT = 10.0f;

static const uint16_t SAMPLE_BUFFER_CAPACITY = 4096;
static const uint16_t START_THRESHOLD = 1;
static const uint16_t PREF_LOOP_MIN_BUFFER_CYCLES = 1;
static const uint8_t MAX_INTERP_STEPS = 4;
static const uint16_t INTERP_DAC_STEP = 64;
static const uint32_t INTERP_MIN_DT_US = 250;

Adafruit_MCP4728 dac;

struct Frame {
  uint32_t dt_us;
  uint16_t dac_v;
  uint16_t dac_i;
  uint8_t flags;
};

struct AnalogCh {
  float a = 1.0f;
  float b = 0.0f;

  float rawToEng(int16_t raw) const {
    return a * (float)raw + b;
  }
};

struct ComtradeCfg {
  int nAnalog = 0;
  int nDigital = 0;
  float freqHz = 60.0f;
  float sampleRateHz = 0.0f;
  float timeMult = 1.0f;
  String dataFormat;
  AnalogCh chV;
  AnalogCh chI;
  bool ts64 = false;
  size_t recSize = 0;
  uint32_t totalRecords = 0;
  size_t digitalBytes = 0;
};

struct RawRecord {
  int64_t ts = 0;
  float engV = 0.0f;
  float engI = 0.0f;
};

struct ScaleRefs {
  float vNomPeak = 0.0f;
  float iNomPeak = 0.0f;
  float vNomRms = 0.0f;
  float iNomRms = 0.0f;
  float vClipPeak = 0.0f;
  float iClipPeak = 0.0f;
};

struct PrefSample {
  uint32_t t_us = 0;
  float engV = 0.0f;
  float engI = 0.0f;
};

struct FrameRing {
  Frame items[SAMPLE_BUFFER_CAPACITY];
  uint16_t head = 0;
  uint16_t tail = 0;
  uint16_t count = 0;

  bool push(const Frame &frame) {
    if (count >= SAMPLE_BUFFER_CAPACITY) return false;
    items[head] = frame;
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

  void clear() {
    head = 0;
    tail = 0;
    count = 0;
  }
};

enum Command : uint8_t {
  CMD_NONE,
  CMD_S,
  CMD_F,
  CMD_P,
  CMD_Q,
  CMD_X,
  CMD_T
};

static FrameRing g_frameRing;
static SemaphoreHandle_t g_ringMutex = nullptr;
static SemaphoreHandle_t g_cmdMutex = nullptr;
static Command g_currentCommand = CMD_NONE;

static volatile bool g_started = false;
static volatile bool g_pendingValid = false;
static volatile bool g_underflowActive = false;
static volatile bool g_readyPlayback = false;
static volatile bool g_processing = false;

static Frame g_pendingFrame;
static uint32_t g_applyAtUs = 0;

static Frame *g_fullFrames = nullptr;
static uint32_t g_fullCount = 0;
static Frame *g_prefaultFrames = nullptr;
static uint32_t g_prefaultCount = 0;
static Frame *g_testFrames = nullptr;
static uint32_t g_testCount = 0;

static volatile uint32_t g_playedFrames = 0;
static volatile uint32_t g_underflows = 0;
static volatile uint32_t g_overflows = 0;

static TaskHandle_t g_serialTaskHandle = nullptr;
static TaskHandle_t g_dacTaskHandle = nullptr;
static TaskHandle_t g_controlTaskHandle = nullptr;

static uint16_t clamp12(int value) {
  if (value < 0) return 0;
  if (value > 4095) return 4095;
  return (uint16_t)value;
}

static uint16_t voltsToDac12(float volts) {
  volts = constrain(volts, 0.0f, DAC_VREF);
  return clamp12((int)lroundf((volts / DAC_VREF) * (float)DAC_MAX));
}

static float mapEngToVolts(float eng, float clipPeak) {
  if (clipPeak < 1e-9f) clipPeak = 1.0f;
  eng = constrain(eng, -clipPeak, clipPeak);
  float x = constrain(eng / clipPeak, -1.0f, 1.0f);
  return V_MID + x * V_AMP;
}

static Frame makeFrame(uint32_t dtUs, float engV, float engI, float vClip, float iClip) {
  uint8_t clipV = fabsf(engV) > vClip ? 1 : 0;
  uint8_t clipI = fabsf(engI) > iClip ? 1 : 0;
  Frame frame;
  frame.dt_us = dtUs;
  frame.dac_v = voltsToDac12(mapEngToVolts(engV, vClip));
  frame.dac_i = voltsToDac12(mapEngToVolts(engI, iClip));
  frame.flags = (clipV & 0x01) | ((clipI & 0x01) << 1);
  return frame;
}

static void setIdleOutput() {
  dac.fastWrite(DAC_IDLE_CODE, DAC_IDLE_CODE, DAC_IDLE_CODE, DAC_IDLE_CODE);
}

static void resetPlaybackState() {
  if (xSemaphoreTake(g_ringMutex, portMAX_DELAY)) {
    g_frameRing.clear();
    xSemaphoreGive(g_ringMutex);
  }
  g_started = false;
  g_pendingValid = false;
  g_underflowActive = false;
  g_applyAtUs = 0;
  setIdleOutput();
}

static void *allocFrames(uint32_t count) {
  size_t bytes = (size_t)count * sizeof(Frame);
  void *ptr = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ptr) ptr = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
  return ptr;
}

static void freeFrameBuffer(Frame *&ptr, uint32_t &count) {
  if (ptr) {
    free(ptr);
    ptr = nullptr;
  }
  count = 0;
}

static Command parseCommand(const String &text) {
  if (text == "s") return CMD_S;
  if (text == "f") return CMD_F;
  if (text == "p") return CMD_P;
  if (text == "q") return CMD_Q;
  if (text == "x") return CMD_X;
  if (text == "t") return CMD_T;
  return CMD_NONE;
}

static void setCommand(Command cmd) {
  if (xSemaphoreTake(g_cmdMutex, portMAX_DELAY)) {
    g_currentCommand = cmd;
    xSemaphoreGive(g_cmdMutex);
  }
}

static Command takeCommand() {
  Command out = CMD_NONE;
  if (xSemaphoreTake(g_cmdMutex, portMAX_DELAY)) {
    out = g_currentCommand;
    g_currentCommand = CMD_NONE;
    xSemaphoreGive(g_cmdMutex);
  }
  return out;
}

static Command peekCommand() {
  Command out = CMD_NONE;
  if (xSemaphoreTake(g_cmdMutex, portMAX_DELAY)) {
    out = g_currentCommand;
    xSemaphoreGive(g_cmdMutex);
  }
  return out;
}

static bool readLittleEndian(File &file, uint8_t *buf, size_t len) {
  return file.read(buf, len) == (int)len;
}

static uint32_t leU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t leI32(const uint8_t *p) {
  return (int32_t)leU32(p);
}

static int16_t leI16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int64_t leI64(const uint8_t *p) {
  uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | p[i];
  }
  return (int64_t)value;
}

static int parseCountToken(String token, char suffix) {
  token.trim();
  token.toUpperCase();
  if (token.endsWith(String(suffix))) token.remove(token.length() - 1);
  return token.toInt();
}

static int splitCsv(const String &line, String *parts, int maxParts) {
  int count = 0;
  int start = 0;
  while (count < maxParts) {
    int comma = line.indexOf(',', start);
    String part = comma < 0 ? line.substring(start) : line.substring(start, comma);
    part.trim();
    parts[count++] = part;
    if (comma < 0) break;
    start = comma + 1;
  }
  return count;
}

static bool parseCfgFile(ComtradeCfg &cfg, String &error) {
  File file = LittleFS.open(CFG_PATH, "r");
  if (!file) {
    error = "CFG_OPEN_FAIL";
    return false;
  }

  const int MAX_LINES = 160;
  String lines[MAX_LINES];
  int lineCount = 0;
  while (file.available() && lineCount < MAX_LINES) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) lines[lineCount++] = line;
  }
  file.close();

  if (lineCount < 6) {
    error = "CFG_TOO_SHORT";
    return false;
  }

  String parts[24];
  if (splitCsv(lines[1], parts, 24) < 3) {
    error = "CFG_COUNTS";
    return false;
  }

  cfg = ComtradeCfg();
  cfg.nAnalog = parseCountToken(parts[1], 'A');
  cfg.nDigital = parseCountToken(parts[2], 'D');

  int idx = 2;
  for (int ch = 0; ch < cfg.nAnalog; ++ch) {
    if (idx >= lineCount) {
      error = "CFG_ANALOG_MISSING";
      return false;
    }
    int n = splitCsv(lines[idx++], parts, 24);
    if (n < 7) {
      error = "CFG_ANALOG_INVALID";
      return false;
    }
    float a = parts[5].toFloat();
    float b = parts[6].toFloat();
    if (ch == 0) {
      cfg.chV.a = a;
      cfg.chV.b = b;
    } else if (ch == 1) {
      cfg.chI.a = a;
      cfg.chI.b = b;
    }
  }

  idx += cfg.nDigital;
  if (idx < lineCount) {
    float freq = lines[idx].toFloat();
    if (freq > 1.0f && freq < 500.0f) cfg.freqHz = freq;
  }
  idx++;

  if (idx >= lineCount) {
    error = "CFG_RATE_COUNT";
    return false;
  }
  int nrates = lines[idx++].toInt();
  if (nrates > 0) {
    if (idx >= lineCount) {
      error = "CFG_RATE_MISSING";
      return false;
    }
    splitCsv(lines[idx], parts, 24);
    cfg.sampleRateHz = parts[0].toFloat();
    idx += nrates;
  }

  idx += 2;
  if (idx >= lineCount) {
    error = "CFG_FORMAT";
    return false;
  }
  cfg.dataFormat = lines[idx++];
  cfg.dataFormat.trim();
  cfg.dataFormat.toUpperCase();

  if (idx < lineCount) {
    float tm = lines[idx].toFloat();
    if (tm > 0.0f) cfg.timeMult = tm;
  }

  return true;
}

static bool computeBdatLayout(ComtradeCfg &cfg, String &error) {
  if (cfg.dataFormat != "BINARY") {
    error = "FORMAT_NOT_BINARY";
    return false;
  }

  File file = LittleFS.open(BDAT_PATH, "r");
  if (!file) {
    error = "BDAT_OPEN_FAIL";
    return false;
  }
  size_t fileSize = file.size();
  file.close();

  int digitalWords = (cfg.nDigital + 15) / 16;
  cfg.digitalBytes = (size_t)digitalWords * 2;
  size_t recSize32 = 4 + 4 + ((size_t)cfg.nAnalog * 2) + cfg.digitalBytes;
  size_t recSize64 = 4 + 8 + ((size_t)cfg.nAnalog * 2) + cfg.digitalBytes;

  if (recSize32 > 0 && fileSize % recSize32 == 0) {
    cfg.ts64 = false;
    cfg.recSize = recSize32;
  } else if (recSize64 > 0 && fileSize % recSize64 == 0) {
    cfg.ts64 = true;
    cfg.recSize = recSize64;
  } else {
    error = "BDAT_LAYOUT";
    return false;
  }

  cfg.totalRecords = fileSize / cfg.recSize;
  if (cfg.totalRecords == 0) {
    error = "BDAT_EMPTY";
    return false;
  }
  return true;
}

static bool readRawRecord(File &file, const ComtradeCfg &cfg, uint8_t *rec, RawRecord &out) {
  if (!readLittleEndian(file, rec, cfg.recSize)) return false;
  size_t off = 4;
  if (cfg.ts64) {
    out.ts = leI64(rec + off);
    off += 8;
  } else {
    out.ts = leI32(rec + off);
    off += 4;
  }

  out.engV = 0.0f;
  out.engI = 0.0f;
  for (int ch = 0; ch < cfg.nAnalog; ++ch) {
    int16_t raw = leI16(rec + off);
    off += 2;
    if (ch == 0) out.engV = cfg.chV.rawToEng(raw);
    if (ch == 1) out.engI = cfg.chI.rawToEng(raw);
  }
  return true;
}

static bool computeScaleRefs(const ComtradeCfg &cfg, ScaleRefs &refs, String &error) {
  if (cfg.sampleRateHz < 1.0f || cfg.freqHz < 1.0f) {
    error = "BAD_RATE";
    return false;
  }

  File file = LittleFS.open(BDAT_PATH, "r");
  if (!file) {
    error = "BDAT_OPEN_FAIL";
    return false;
  }

  uint8_t *rec = (uint8_t *)malloc(cfg.recSize);
  if (!rec) {
    file.close();
    error = "REC_ALLOC";
    return false;
  }

  int samplesPerCycle = max(8, (int)lroundf(cfg.sampleRateHz / cfg.freqHz));
  uint32_t nominalWindow = (uint32_t)(samplesPerCycle * NOM_CYCLES);
  uint32_t limitN = cfg.totalRecords < nominalWindow ? cfg.totalRecords : nominalWindow;
  float cyclePeakV = 0.0f;
  float cyclePeakI = 0.0f;
  float sumPeakV = 0.0f;
  float sumPeakI = 0.0f;
  int peakCount = 0;
  int idxInCycle = 0;

  for (uint32_t idx = 0; idx < limitN; ++idx) {
    RawRecord rr;
    if (!readRawRecord(file, cfg, rec, rr)) break;
    cyclePeakV = max(cyclePeakV, fabsf(rr.engV));
    cyclePeakI = max(cyclePeakI, fabsf(rr.engI));
    idxInCycle++;
    if (idxInCycle >= samplesPerCycle) {
      sumPeakV += cyclePeakV;
      sumPeakI += cyclePeakI;
      peakCount++;
      cyclePeakV = 0.0f;
      cyclePeakI = 0.0f;
      idxInCycle = 0;
    }
  }

  free(rec);
  file.close();

  if (peakCount == 0) {
    error = "SCALE_REFS";
    return false;
  }

  refs.vNomPeak = max(sumPeakV / (float)peakCount, 1.0f);
  refs.iNomPeak = max(sumPeakI / (float)peakCount, 1.0f);
  refs.vNomRms = refs.vNomPeak / sqrtf(2.0f);
  refs.iNomRms = refs.iNomPeak / sqrtf(2.0f);
  refs.vClipPeak = refs.vNomPeak * V_FAULT_LIMIT_MULT;
  refs.iClipPeak = refs.iNomPeak * I_FAULT_LIMIT_MULT;
  return true;
}

static float lerpFloat(float a, float b, float alpha) {
  return a + alpha * (b - a);
}

static bool buildPrefaultFrames(const ComtradeCfg &cfg, const PrefSample *samples, uint32_t count,
                                float vClip, float iClip, String &error) {
  freeFrameBuffer(g_prefaultFrames, g_prefaultCount);
  if (count < 6) {
    error = "PREFAULT_TOO_SHORT";
    return false;
  }

  const PrefSample &start = samples[0];
  float targetV = start.engV;
  float targetI = start.engI;
  float targetPeriodUs = 1000000.0f / cfg.freqHz;
  int approxSamples = max(8, (int)lroundf(cfg.sampleRateHz / cfg.freqHz));
  int window = max(6, (int)(0.35f * (float)approxSamples));
  int searchLeft = max(1, approxSamples - window);
  int searchRight = min((int)count - 2, approxSamples + window);

  int endIndex = -1;
  float endTUs = 0.0f;
  float endI = targetI;
  float bestScore = 1e30f;
  bool found = false;

  for (int i = searchLeft; i <= searchRight; ++i) {
    const PrefSample &s0 = samples[i];
    const PrefSample &s1 = samples[i + 1];
    float dv0 = s0.engV - targetV;
    float dv1 = s1.engV - targetV;
    bool crosses = (dv0 == 0.0f) || (dv1 == 0.0f) || (dv0 < 0.0f && dv1 > 0.0f) || (dv1 < 0.0f && dv0 > 0.0f);
    if (!crosses) continue;

    float denom = s1.engV - s0.engV;
    float alpha = fabsf(denom) < 1e-12f ? 0.5f : constrain((targetV - s0.engV) / denom, 0.0f, 1.0f);
    float crossTUs = lerpFloat((float)s0.t_us, (float)s1.t_us, alpha);
    float crossI = lerpFloat(s0.engI, s1.engI, alpha);
    float periodErr = fabsf(crossTUs - targetPeriodUs) / targetPeriodUs;
    float currentErr = fabsf(crossI - targetI) / max(1.0f, max(fabsf(targetI), fabsf(crossI)));
    float slopeRef = samples[1].engV - samples[0].engV;
    float slopeErr = fabsf((s1.engV - s0.engV) - slopeRef) / max(1.0f, fabsf(slopeRef));
    float score = periodErr + 0.35f * currentErr + 0.15f * slopeErr;
    if (!found || score < bestScore) {
      found = true;
      bestScore = score;
      endIndex = i;
      endTUs = crossTUs;
      endI = crossI;
    }
  }

  if (!found) {
    for (int i = searchLeft; i <= searchRight; ++i) {
      const PrefSample &s = samples[i];
      float periodErr = fabsf((float)s.t_us - targetPeriodUs) / targetPeriodUs;
      float ampErr = fabsf(s.engV - targetV) / max(1.0f, max(fabsf(targetV), fabsf(s.engV)));
      ampErr += fabsf(s.engI - targetI) / max(1.0f, max(fabsf(targetI), fabsf(s.engI)));
      float score = periodErr + 0.5f * ampErr;
      if (endIndex < 0 || score < bestScore) {
        bestScore = score;
        endIndex = i;
        endTUs = (float)s.t_us;
        endI = targetI;
      }
    }
  }

  if (endIndex < 1) {
    error = "PREFAULT_DETECT";
    return false;
  }

  uint32_t prefCount = (uint32_t)endIndex + 2;
  Frame *frames = (Frame *)allocFrames(prefCount);
  if (!frames) {
    error = "PREFAULT_ALLOC";
    return false;
  }

  uint32_t out = 0;
  uint32_t prevRel = 0;
  frames[out++] = makeFrame(0, start.engV, start.engI, vClip, iClip);

  for (int i = 1; i <= endIndex; ++i) {
    uint32_t rel = max(prevRel, samples[i].t_us - start.t_us);
    frames[out++] = makeFrame(rel - prevRel, samples[i].engV, samples[i].engI, vClip, iClip);
    prevRel = rel;
  }

  uint32_t finalRel = max(prevRel, (uint32_t)lroundf(endTUs - (float)start.t_us));
  frames[out++] = makeFrame(finalRel - prevRel, targetV, targetI, vClip, iClip);

  g_prefaultFrames = frames;
  g_prefaultCount = out;
  return true;
}

static bool buildTestSineFrames() {
  freeFrameBuffer(g_testFrames, g_testCount);
  const float freqHz = 60.0f;
  const float amplitudeV = 3.0f;
  const float sampleRate = 1000.0f;
  const int total = (int)(sampleRate / freqHz);
  Frame *frames = (Frame *)allocFrames(total);
  if (!frames) return false;
  uint32_t dtUs = (uint32_t)lroundf(1000000.0f / sampleRate);

  for (int i = 0; i < total; ++i) {
    float t = (float)i / sampleRate;
    float value = V_MID + (amplitudeV / 2.0f) * sinf(2.0f * PI * freqHz * t);
    frames[i].dt_us = dtUs;
    frames[i].dac_v = voltsToDac12(value);
    frames[i].dac_i = voltsToDac12(value);
    frames[i].flags = 0;
  }

  g_testFrames = frames;
  g_testCount = total;
  return true;
}

static uint8_t interpolationSteps(const Frame &prev, const Frame &cur) {
  if (cur.dt_us < 2 * INTERP_MIN_DT_US) return 1;
  uint16_t deltaV = prev.dac_v > cur.dac_v ? prev.dac_v - cur.dac_v : cur.dac_v - prev.dac_v;
  uint16_t deltaI = prev.dac_i > cur.dac_i ? prev.dac_i - cur.dac_i : cur.dac_i - prev.dac_i;
  uint16_t delta = max(deltaV, deltaI);
  if (delta <= INTERP_DAC_STEP) return 1;

  uint8_t byStep = (uint8_t)min((uint16_t)MAX_INTERP_STEPS,
                                (uint16_t)((delta + INTERP_DAC_STEP - 1) / INTERP_DAC_STEP));
  uint8_t byTime = (uint8_t)max((uint32_t)1, min((uint32_t)MAX_INTERP_STEPS,
                                                 cur.dt_us / INTERP_MIN_DT_US));
  return max((uint8_t)1, min(byStep, byTime));
}

static bool expandInterpolatedFrames(Frame *&frames, uint32_t &count, const char *name) {
  if (!frames || count < 2) return true;

  uint32_t originalCount = count;
  uint32_t expandedCount = 1;
  for (uint32_t i = 1; i < count; ++i) {
    expandedCount += interpolationSteps(frames[i - 1], frames[i]);
  }

  if (expandedCount == count) {
    Serial.printf("STATUS INTERP name=%s original=%lu expanded=%lu\n",
                  name, (unsigned long)count, (unsigned long)expandedCount);
    return true;
  }

  Frame *expanded = (Frame *)allocFrames(expandedCount);
  if (!expanded) {
    Serial.printf("WARN code=INTERP_ALLOC name=%s original=%lu expanded=%lu\n",
                  name, (unsigned long)count, (unsigned long)expandedCount);
    return true;
  }

  uint32_t out = 0;
  expanded[out++] = frames[0];

  for (uint32_t i = 1; i < count; ++i) {
    const Frame &prev = frames[i - 1];
    const Frame &cur = frames[i];
    uint8_t steps = interpolationSteps(prev, cur);

    uint32_t baseDt = cur.dt_us / steps;
    uint32_t remDt = cur.dt_us % steps;

    for (uint8_t step = 1; step <= steps; ++step) {
      Frame f;
      f.dt_us = baseDt + (step <= remDt ? 1 : 0);
      f.dac_v = (uint16_t)lroundf((float)prev.dac_v + ((float)cur.dac_v - (float)prev.dac_v) * ((float)step / (float)steps));
      f.dac_i = (uint16_t)lroundf((float)prev.dac_i + ((float)cur.dac_i - (float)prev.dac_i) * ((float)step / (float)steps));
      f.flags = step == steps ? cur.flags : 0;
      expanded[out++] = f;
    }
  }

  free(frames);
  frames = expanded;
  count = out;
  Serial.printf("STATUS INTERP name=%s original=%lu expanded=%lu\n",
                name, (unsigned long)originalCount, (unsigned long)count);
  return true;
}

static bool buildFullFrames(const ComtradeCfg &cfg, float vClip, float iClip, String &error) {
  freeFrameBuffer(g_fullFrames, g_fullCount);

  Frame *frames = (Frame *)allocFrames(cfg.totalRecords);
  if (!frames) {
    error = "FULL_ALLOC";
    return false;
  }

  int approxSamples = max(8, (int)lroundf(cfg.sampleRateHz / cfg.freqHz));
  int window = max(6, (int)(0.35f * (float)approxSamples));
  uint32_t desiredPrefCapture = (uint32_t)(approxSamples + window + 3);
  uint32_t prefCaptureCount = cfg.totalRecords < desiredPrefCapture ? cfg.totalRecords : desiredPrefCapture;
  PrefSample *prefSamples = (PrefSample *)malloc(sizeof(PrefSample) * prefCaptureCount);
  if (!prefSamples) {
    free(frames);
    error = "PREF_CAPTURE_ALLOC";
    return false;
  }

  File file = LittleFS.open(BDAT_PATH, "r");
  if (!file) {
    free(frames);
    free(prefSamples);
    error = "BDAT_OPEN_FAIL";
    return false;
  }

  uint8_t *rec = (uint8_t *)malloc(cfg.recSize);
  if (!rec) {
    file.close();
    free(frames);
    free(prefSamples);
    error = "REC_ALLOC";
    return false;
  }

  int64_t prevTs = 0;
  bool havePrevTs = false;
  uint32_t tAccum = 0;
  uint32_t out = 0;
  uint32_t prefOut = 0;

  for (uint32_t idx = 0; idx < cfg.totalRecords; ++idx) {
    RawRecord rr;
    if (!readRawRecord(file, cfg, rec, rr)) {
      free(rec);
      file.close();
      free(frames);
      free(prefSamples);
      error = "BDAT_READ";
      return false;
    }

    uint32_t dtUs = 0;
    if (havePrevTs) {
      int64_t dts = rr.ts - prevTs;
      if (dts < 0) dts = 0;
      dtUs = (uint32_t)lroundf((float)dts * cfg.timeMult);
      if (dtUs == 0 && cfg.sampleRateHz > 0.1f) {
        dtUs = (uint32_t)lroundf(1000000.0f / cfg.sampleRateHz);
      }
    }
    prevTs = rr.ts;
    havePrevTs = true;
    tAccum += dtUs;

    frames[out++] = makeFrame(dtUs, rr.engV, rr.engI, vClip, iClip);
    if (prefOut < prefCaptureCount) {
      prefSamples[prefOut].t_us = tAccum;
      prefSamples[prefOut].engV = rr.engV;
      prefSamples[prefOut].engI = rr.engI;
      prefOut++;
    }
  }

  free(rec);
  file.close();

  bool prefOk = buildPrefaultFrames(cfg, prefSamples, prefOut, vClip, iClip, error);
  free(prefSamples);
  if (!prefOk) {
    free(frames);
    return false;
  }

  g_fullFrames = frames;
  g_fullCount = out;
  expandInterpolatedFrames(g_fullFrames, g_fullCount, "full");
  expandInterpolatedFrames(g_prefaultFrames, g_prefaultCount, "prefault");
  return true;
}

static bool processComtrade() {
  g_processing = true;
  g_readyPlayback = false;
  resetPlaybackState();
  Serial.println("STATUS PROCESSING");

  String error;
  ComtradeCfg cfg;
  if (!parseCfgFile(cfg, error) || !computeBdatLayout(cfg, error)) {
    Serial.printf("ERR code=%s\n", error.c_str());
    g_processing = false;
    return false;
  }

  ScaleRefs refs;
  if (!computeScaleRefs(cfg, refs, error)) {
    Serial.printf("ERR code=%s\n", error.c_str());
    g_processing = false;
    return false;
  }

  Serial.printf("STATUS CFG nA=%d nD=%d fs=%.2f f=%.2f records=%lu recSize=%u ts64=%u\n",
                cfg.nAnalog, cfg.nDigital, cfg.sampleRateHz, cfg.freqHz,
                (unsigned long)cfg.totalRecords, (unsigned)cfg.recSize, cfg.ts64 ? 1 : 0);
  Serial.printf("STATUS SCALE vNom=%.6f iNom=%.6f vNomRms=%.6f iNomRms=%.6f vClip=%.6f iClip=%.6f\n",
                refs.vNomPeak, refs.iNomPeak, refs.vNomRms, refs.iNomRms,
                refs.vClipPeak, refs.iClipPeak);

  if (!buildFullFrames(cfg, refs.vClipPeak, refs.iClipPeak, error)) {
    Serial.printf("ERR code=%s\n", error.c_str());
    g_processing = false;
    return false;
  }

  buildTestSineFrames();
  g_readyPlayback = true;
  g_processing = false;
  Serial.printf("READY_PLAYBACK samples=%lu prefault=%lu test=%lu free_heap=%lu\n",
                (unsigned long)g_fullCount,
                (unsigned long)g_prefaultCount,
                (unsigned long)g_testCount,
                (unsigned long)ESP.getFreeHeap());
  return true;
}

static bool receiveFile(const char *path, size_t size) {
  File file = LittleFS.open(path, "w");
  if (!file) return false;

  const size_t BUF_SIZE = 256;
  uint8_t buf[BUF_SIZE];
  size_t remaining = size;
  size_t received = 0;
  uint32_t lastProgress = millis();

  while (remaining > 0) {
    size_t chunk = min(remaining, BUF_SIZE);
    size_t got = Serial.readBytes(buf, chunk);
    if (got == 0) {
      if (millis() - lastProgress > UPLOAD_IDLE_TIMEOUT_MS) {
        file.close();
        Serial.printf("ERR code=UPLOAD_TIMEOUT missing=%lu\n", (unsigned long)remaining);
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    size_t written = file.write(buf, got);
    if (written != got) {
      file.close();
      Serial.printf("ERR code=UPLOAD_WRITE written=%lu got=%lu\n",
                    (unsigned long)written, (unsigned long)got);
      return false;
    }
    remaining -= got;
    received += got;
    lastProgress = millis();
    Serial.printf("RX %lu\n", (unsigned long)received);
  }

  file.close();
  return true;
}

static void serialTask(void *) {
  Serial.setTimeout(5000);

  for (;;) {
    if (!Serial.available()) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("UPLOAD ")) {
      if (g_processing) {
        Serial.println("BUSY PROCESSING");
        continue;
      }
      int firstSpace = line.indexOf(' ');
      int secondSpace = line.indexOf(' ', firstSpace + 1);
      if (secondSpace < 0) {
        Serial.println("ERR code=UPLOAD_SYNTAX");
        continue;
      }

      String kind = line.substring(firstSpace + 1, secondSpace);
      kind.toUpperCase();
      size_t size = (size_t)line.substring(secondSpace + 1).toInt();
      const char *path = nullptr;
      if (kind == "CFG") path = CFG_PATH;
      if (kind == "BDAT") path = BDAT_PATH;
      if (!path || size == 0) {
        Serial.println("ERR code=UPLOAD_KIND_OR_SIZE");
        continue;
      }

      g_readyPlayback = false;
      resetPlaybackState();
      freeFrameBuffer(g_fullFrames, g_fullCount);
      freeFrameBuffer(g_prefaultFrames, g_prefaultCount);
      freeFrameBuffer(g_testFrames, g_testCount);
      Serial.printf("READY_RECEIVE %s %lu\n", kind.c_str(), (unsigned long)size);
      bool ok = receiveFile(path, size);
      Serial.printf("%s %s bytes=%lu\n", ok ? "OK" : "ERR", kind.c_str(), (unsigned long)size);
      continue;
    }

    if (line == "PROCESS") {
      processComtrade();
      continue;
    }

    Command cmd = parseCommand(line);
    if (cmd == CMD_NONE) {
      Serial.printf("ERR code=UNKNOWN_CMD cmd=%s\n", line.c_str());
      continue;
    }

    if (!g_readyPlayback && cmd != CMD_Q && cmd != CMD_X) {
      Serial.println("BUSY NOT_READY");
      continue;
    }
    setCommand(cmd);
    Serial.printf("ACK cmd=%s\n", line.c_str());
  }
}

static inline void outputFrame(const Frame &frame) {
  dac.fastWrite(frame.dac_v, frame.dac_i, DAC_IDLE_CODE, DAC_IDLE_CODE);
  g_playedFrames++;
}

static void maybeStartPlayback() {
  if (g_started) return;
  uint16_t count = 0;
  if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(1))) {
    count = g_frameRing.count;
    xSemaphoreGive(g_ringMutex);
  }
  if (count < START_THRESHOLD) return;
  g_started = true;
  g_underflowActive = false;
  g_applyAtUs = micros();
}

static void dacTask(void *) {
  for (;;) {
    maybeStartPlayback();

    if (!g_started) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (!g_pendingValid) {
      Frame frame;
      bool got = false;
      if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(2))) {
        got = g_frameRing.pop(frame);
        xSemaphoreGive(g_ringMutex);
      }

      if (got) {
        g_pendingFrame = frame;
        g_pendingValid = true;
        g_underflowActive = false;
        if (g_pendingFrame.flags & FLAG_RESET_CLOCK) {
          g_applyAtUs = micros();
        } else {
          g_applyAtUs += g_pendingFrame.dt_us;
        }
      } else {
        if (!g_underflowActive) {
          g_underflows++;
          g_underflowActive = true;
        }
        g_started = false;
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
    }

    if ((int32_t)(micros() - g_applyAtUs) < 0) {
      taskYIELD();
      continue;
    }

    outputFrame(g_pendingFrame);
    bool idleAfter = (g_pendingFrame.flags & FLAG_IDLE_AFTER) != 0;
    uint32_t dtUs = g_pendingFrame.dt_us;
    g_pendingValid = false;

    if (idleAfter) {
      setIdleOutput();
      g_started = false;
      continue;
    }

    if (dtUs > 0) {
      int32_t delayUs = (int32_t)(micros() - g_applyAtUs);
      if (delayUs > (int32_t)(4 * dtUs)) {
        g_applyAtUs += (uint32_t)(delayUs / 2);
      }
    }
  }
}

static bool pushRingBlocking(const Frame &frame) {
  for (;;) {
    Command cmd = peekCommand();
    if (cmd == CMD_Q || cmd == CMD_X) return false;

    bool ok = false;
    if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(2))) {
      ok = g_frameRing.push(frame);
      xSemaphoreGive(g_ringMutex);
    }
    if (ok) return true;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static bool playbackIdle() {
  uint16_t count = 0;
  if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(2))) {
    count = g_frameRing.count;
    xSemaphoreGive(g_ringMutex);
  }
  return !g_started && !g_pendingValid && count == 0;
}

static uint16_t ringCount() {
  uint16_t count = 0;
  if (xSemaphoreTake(g_ringMutex, pdMS_TO_TICKS(2))) {
    count = g_frameRing.count;
    xSemaphoreGive(g_ringMutex);
  }
  return count;
}

static bool pushIdleFrameBlocking() {
  Frame idle;
  idle.dt_us = 0;
  idle.dac_v = DAC_IDLE_CODE;
  idle.dac_i = DAC_IDLE_CODE;
  idle.flags = FLAG_IDLE_AFTER;
  return pushRingBlocking(idle);
}

static bool playFrames(const Frame *frames, uint32_t count, const char *name, bool idleAfter = true) {
  if (!frames || count == 0) {
    Serial.printf("ERR code=NO_FRAMES name=%s\n", name);
    return false;
  }

  resetPlaybackState();
  Serial.printf("PLAY_START name=%s count=%lu\n", name, (unsigned long)count);

  for (uint32_t i = 0; i < count; ++i) {
    if (!pushRingBlocking(frames[i])) {
      resetPlaybackState();
      takeCommand();
      Serial.printf("PLAY_ABORT name=%s\n", name);
      return false;
    }
  }

  if (idleAfter) {
    pushIdleFrameBlocking();
  }

  while (!playbackIdle()) {
    Command cmd = peekCommand();
    if (cmd == CMD_Q || cmd == CMD_X) {
      resetPlaybackState();
      takeCommand();
      Serial.printf("PLAY_ABORT name=%s\n", name);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  Serial.printf("PLAY_DONE name=%s\n", name);
  return true;
}

static bool enqueueFramesRange(const Frame *frames, uint32_t startIndex, uint32_t count,
                               const char *name, bool resetBefore) {
  if (!frames || count == 0) {
    Serial.printf("ERR code=NO_FRAMES name=%s\n", name);
    return false;
  }
  if (startIndex >= count) return true;

  if (resetBefore) {
    resetPlaybackState();
    Serial.printf("PLAY_START name=%s count=%lu\n", name, (unsigned long)(count - startIndex));
  }

  for (uint32_t i = startIndex; i < count; ++i) {
    if (!pushRingBlocking(frames[i])) {
      resetPlaybackState();
      takeCommand();
      Serial.printf("PLAY_ABORT name=%s\n", name);
      return false;
    }
  }
  return true;
}

static bool enqueueFrames(const Frame *frames, uint32_t count, const char *name, bool resetBefore) {
  return enqueueFramesRange(frames, 0, count, name, resetBefore);
}

static bool appendFullOnceSeamless(uint32_t startIndex) {
  Serial.printf("PLAY_CHAIN name=full_once start=%lu count=%lu\n",
                (unsigned long)startIndex,
                (unsigned long)(g_fullCount > startIndex ? g_fullCount - startIndex : 0));
  if (!enqueueFramesRange(g_fullFrames, startIndex, g_fullCount, "full_once", false)) return false;
  if (!pushIdleFrameBlocking()) return false;
  while (!playbackIdle()) {
    Command cmd = peekCommand();
    if (cmd == CMD_Q || cmd == CMD_X) {
      resetPlaybackState();
      takeCommand();
      Serial.println("PLAY_ABORT name=full_once");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  Serial.println("PLAY_DONE name=full_once");
  return true;
}

static void controlTask(void *) {
  enum Mode : uint8_t {
    MODE_IDLE,
    MODE_PREF_LOOP
  };

  Mode mode = MODE_IDLE;
  bool prefLoopPrimed = false;

  for (;;) {
    if (!g_readyPlayback) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    Command cmd = takeCommand();

    if (cmd == CMD_Q) {
      mode = MODE_IDLE;
      prefLoopPrimed = false;
      resetPlaybackState();
      Serial.println("IDLE");
      continue;
    }
    if (cmd == CMD_X) {
      mode = MODE_IDLE;
      prefLoopPrimed = false;
      resetPlaybackState();
      Serial.println("IDLE EXIT_ACK");
      continue;
    }
    if (cmd == CMD_S) {
      mode = MODE_PREF_LOOP;
      prefLoopPrimed = false;
      Serial.println("MODE PREF_LOOP");
      continue;
    }
    if (cmd == CMD_P) {
      mode = MODE_IDLE;
      prefLoopPrimed = false;
      playFrames(g_fullFrames, g_fullCount, "full_once");
      continue;
    }
    if (cmd == CMD_T) {
      mode = MODE_IDLE;
      prefLoopPrimed = false;
      playFrames(g_testFrames, g_testCount, "test_sine");
      continue;
    }

    if (mode == MODE_PREF_LOOP) {
      if (cmd == CMD_F || cmd == CMD_P) {
        mode = MODE_IDLE;
        prefLoopPrimed = false;
        appendFullOnceSeamless(1);
        continue;
      }

      uint16_t target = (uint16_t)min((uint32_t)SAMPLE_BUFFER_CAPACITY - 1,
                                      max(g_prefaultCount,
                                          (uint32_t)PREF_LOOP_MIN_BUFFER_CYCLES * g_prefaultCount));
      if (!prefLoopPrimed || ringCount() < target) {
        enqueueFrames(g_prefaultFrames, g_prefaultCount, "prefault_cycle", !prefLoopPrimed);
        prefLoopPrimed = true;
      } else {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      Command next = takeCommand();
      if (next == CMD_F || next == CMD_P) {
        mode = MODE_IDLE;
        prefLoopPrimed = false;
        appendFullOnceSeamless(1);
      } else if (next == CMD_Q || next == CMD_X) {
        mode = MODE_IDLE;
        prefLoopPrimed = false;
        resetPlaybackState();
        Serial.println("IDLE");
      }
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.begin(SERIAL_BAUD);
  delay(2000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ_HZ);

  if (!dac.begin(MCP4728_ADDR, &Wire)) {
    Serial.println("ERR code=MCP4728_NOT_FOUND");
    while (true) delay(1000);
  }

  if (!LittleFS.begin(true)) {
    Serial.println("ERR code=LITTLEFS_FAIL");
    while (true) delay(1000);
  }

  g_ringMutex = xSemaphoreCreateMutex();
  g_cmdMutex = xSemaphoreCreateMutex();
  if (!g_ringMutex || !g_cmdMutex) {
    Serial.println("ERR code=MUTEX_FAIL");
    while (true) delay(1000);
  }

  setIdleOutput();
  resetPlaybackState();

  Serial.printf("READY_UPLOAD baud=%lu fs_total=%lu fs_used=%lu i2c_hz=%lu\n",
                (unsigned long)SERIAL_BAUD,
                (unsigned long)LittleFS.totalBytes(),
                (unsigned long)LittleFS.usedBytes(),
                (unsigned long)I2C_FREQ_HZ);

  xTaskCreatePinnedToCore(serialTask, "SERIAL", 8192, nullptr, 2, &g_serialTaskHandle, 0);
  xTaskCreatePinnedToCore(dacTask, "DAC", 4096, nullptr, 3, &g_dacTaskHandle, 1);
  xTaskCreatePinnedToCore(controlTask, "CTRL", 4096, nullptr, 1, &g_controlTaskHandle, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}
