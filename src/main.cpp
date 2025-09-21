#include "AudioTools.h"
#include "AudioTools/AudioLibs/AudioBoardStream.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include <Wire.h>
#include "driver/gpio.h"   // pour gpio_reset_pin()
#include "soc/io_mux_reg.h"

// -------- Audio --------
AudioInfo info(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
AudioBoardStream i2s(AudioKitEs8388V1);
const int16_t BYTES_PER_FRAME = 4;

// Seuil de silence
const int16_t SILENCE_THRESHOLD = 400;

// -------- LED RGB --------
const int LED_RED   = 4;
const int LED_BLUE  = 22;
const int LED_GREEN = 13;   // GPIO13 (MTCK) -> nécessite désactivation JTAG

// -------- Bouton -------- 
const int BUTTON_PIN = 5;

unsigned long lastUpdate = 0;
int ledState = 0;

// -------- I2C BQ25186 --------
const uint8_t BQ25186_ADDR = 0x6A;  // 7-bit address du BQ25186

// --- Fonctions basiques I2C ---
void bq25186_writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BQ25186_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t bq25186_readRegister(uint8_t reg) {
  Wire.beginTransmission(BQ25186_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);   // restart
  Wire.requestFrom(BQ25186_ADDR, (uint8_t)1);
  return Wire.read();
}

// --- Read-Modify-Write ---
void bq25186_updateBits(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current = bq25186_readRegister(reg);
  uint8_t updated = (current & ~mask) | (value & mask);
  bq25186_writeRegister(reg, updated);

  Serial.print("Reg 0x");
  Serial.print(reg, HEX);
  Serial.print(" updated: 0x");
  Serial.println(updated, HEX);
}

void bq25186_init() {
  // TMR_ILIM (0x08), bits 6-7 = 00
  bq25186_updateBits(0x08, 0b11000000, 0b00000000);

  // IC_CTRL (0x07), bit 7 = 0
  bq25186_updateBits(0x07, 0b10000011, 0b00000011);
  bq25186_updateBits(0x04, 0b01111111, 0b00101111);


  Serial.println("BQ25186 init done");
}

// -------- Audio Callback --------
int32_t get_sound_data(Frame* data, int32_t frameCount) {
  int byteCount = i2s.readBytes((uint8_t*)data, frameCount * BYTES_PER_FRAME);
  int16_t* samples = (int16_t*)data;
  int numSamples = byteCount / 2;
  int64_t totalAmplitude = 0;

  for (int i = 0; i < 100 && i < numSamples; ++i) {
    totalAmplitude += abs(samples[i]);
  }

  int averageAmplitude = totalAmplitude / numSamples;
  if (averageAmplitude < SILENCE_THRESHOLD) {
    memset(data, 0, byteCount);
  }

  return byteCount / BYTES_PER_FRAME;
}

// -------- LED function --------
void updateLed() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  switch (ledState) {
    case 0: digitalWrite(LED_RED, LOW); break;
    case 1: digitalWrite(LED_BLUE, LOW); break;
    case 2: digitalWrite(LED_GREEN, LOW); break;
    case 3: break; // tout éteint
  }
  ledState = (ledState + 1) % 4;
}

// -------- Désactivation du JTAG --------
void disableJTAG() {
  // JTAG utilise GPIO12-15 par défaut
  gpio_reset_pin(GPIO_NUM_12);
  gpio_reset_pin(GPIO_NUM_13);
  gpio_reset_pin(GPIO_NUM_14);
  gpio_reset_pin(GPIO_NUM_15);

  // on configure GPIO13 pour notre LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.println("JTAG disabled, GPIO13 libre pour LED");
}

// -------- Setup --------
void setup() {
  Serial.begin(115200);

  // Désactive JTAG avant d'utiliser GPIO13
  disableJTAG();

  // I2C (SDA=33, SCL=32)
  Wire.begin(33, 32);
  bq25186_init();

  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  updateLed();

  // Audio
  Serial.println("starting I2S...");
  auto cfg = i2s.defaultConfig(RX_MODE);
  cfg.i2s_format = I2S_STD_FORMAT;
  cfg.copyFrom(info);
  cfg.input_device = ADC_INPUT_LINE2;
  i2s.begin(cfg);

  Serial.println("starting A2DP...");
  a2dp_source.set_data_callback_in_frames(get_sound_data);
  a2dp_source.start("D-58");
}

// -------- Loop --------
void loop() {
unsigned long now = millis();

  // ---- Bouton + LED ----
  if (digitalRead(BUTTON_PIN) == LOW) {
    // Si la pin 5 est à 0 -> mettre pin 4 à 0
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  } else {
    // Sinon -> mettre pin 4 à 1
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);

  }
}
