#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

#ifndef MCP4728_PD_NORMAL
  #define MCP4728_PD_NORMAL MCP4728_PD_MODE_NORMAL
#endif

// MCP4728 (geracao)
static const uint8_t  I2C_SDA_PIN = 21;
static const uint8_t  I2C_SCL_PIN = 22;
static const uint8_t  MCP4728_I2C_ADDR = 0x60;
static const uint32_t I2C_FREQ_HZ = 400000UL;

// ADC loopback interno na mesma ESP32 (ADC1)
// Conectar fisicamente:
//   AOUT0 (MCP4728 canal A) -> GPIO34
//   AOUT1 (MCP4728 canal B) -> GPIO35
//   GND comum
static const int ADC_PIN_V = 34;
static const int ADC_PIN_I = 35;

static const uint32_t SERIAL_BAUD = 921600;
static const uint32_t SAMPLE_RATE_HZ = 1000;
static const int N_AVG = 8;

static const uint16_t DAC_IDLE_MV = 1650;

// Frame binario enviado ao PC (14 bytes):
// [0] 0xAB
// [1] 0xCD
// [2:5]  t_us      (uint32 LE)
// [6:7]  adc_v     (uint16 LE)
// [8:9]  adc_i     (uint16 LE)
// [10:11] set_mv   (uint16 LE)
// [12] checksum    (sum bytes [0..11] & 0xFF)
// [13] 0xBA
static const uint8_t H0 = 0xAB;
static const uint8_t H1 = 0xCD;
static const uint8_t TAIL = 0xBA;

Adafruit_MCP4728 dac;

static uint16_t g_set_mv = DAC_IDLE_MV;

static bool configureChannelsVdd1x() {
  bool ok = true;
  ok &= dac.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  ok &= dac.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  return ok;
}

static uint16_t mvToCode(uint16_t mv) {
  if (mv > 3300) mv = 3300;
  uint32_t code = ((uint32_t)mv * 4095U + 1650U) / 3300U;
  if (code > 4095U) code = 4095U;
  return (uint16_t)code;
}

static void setGeneratorMv(uint16_t mv) {
  g_set_mv = mv;
  uint16_t code = mvToCode(mv);
  // A = tensao, B = corrente. C e D em midscale para nao interferir.
  dac.fastWrite(code, code, mvToCode(DAC_IDLE_MV), mvToCode(DAC_IDLE_MV));
}

static uint16_t readAdcAvg(int pin, int n) {
  uint32_t acc = 0;
  for (int i = 0; i < n; i++) {
    acc += (uint32_t)analogRead(pin);
  }
  return (uint16_t)(acc / (uint32_t)n);
}

static uint8_t checksum8(const uint8_t *buf, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += buf[i];
  return (uint8_t)(sum & 0xFF);
}

static void emitSampleFrame() {
  uint16_t adc_v = readAdcAvg(ADC_PIN_V, N_AVG);
  uint16_t adc_i = readAdcAvg(ADC_PIN_I, N_AVG);
  if (adc_v > 4095) adc_v = 4095;
  if (adc_i > 4095) adc_i = 4095;

  uint8_t fr[14];
  uint32_t t_us = micros();

  fr[0] = H0;
  fr[1] = H1;
  fr[2] = (uint8_t)(t_us & 0xFF);
  fr[3] = (uint8_t)((t_us >> 8) & 0xFF);
  fr[4] = (uint8_t)((t_us >> 16) & 0xFF);
  fr[5] = (uint8_t)((t_us >> 24) & 0xFF);

  fr[6] = (uint8_t)(adc_v & 0xFF);
  fr[7] = (uint8_t)((adc_v >> 8) & 0xFF);
  fr[8] = (uint8_t)(adc_i & 0xFF);
  fr[9] = (uint8_t)((adc_i >> 8) & 0xFF);

  fr[10] = (uint8_t)(g_set_mv & 0xFF);
  fr[11] = (uint8_t)((g_set_mv >> 8) & 0xFF);

  fr[12] = checksum8(fr, 12);
  fr[13] = TAIL;

  Serial.write(fr, sizeof(fr));
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("SET ") || line.startsWith("set ")) {
    long mv = line.substring(4).toInt();
    if (mv < 0) mv = 0;
    if (mv > 3300) mv = 3300;
    setGeneratorMv((uint16_t)mv);
    return;
  }

  if (line.equalsIgnoreCase("STOP")) {
    setGeneratorMv(DAC_IDLE_MV);
    return;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(10);
  delay(1000);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQ_HZ);

  if (!dac.begin(MCP4728_I2C_ADDR, &Wire)) {
    while (true) delay(1000);
  }

  if (!configureChannelsVdd1x()) {
    while (true) delay(1000);
  }

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN_V, ADC_11db);
  analogSetPinAttenuation(ADC_PIN_I, ADC_11db);

  setGeneratorMv(DAC_IDLE_MV);

  Serial.println("READY_LOOPBACK");
}

void loop() {
  static String line;
  static uint32_t next_us = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      handleLine(line);
      line = "";
    } else {
      line += c;
      if (line.length() > 64) line = "";
    }
  }

  const uint32_t period_us = 1000000UL / SAMPLE_RATE_HZ;
  uint32_t now = micros();
  if ((int32_t)(now - next_us) >= 0) {
    next_us = now + period_us;
    emitSampleFrame();
  }
}
