#include "AudioTools.h"
#include "AudioTools/AudioLibs/AudioBoardStream.h"
#include "AudioTools/AudioLibs/A2DPStream.h"

#include <Wire.h>

AudioInfo info(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
AudioBoardStream i2s(AudioKitEs8388V1);
const int16_t BYTES_PER_FRAME = 4;

// callback pour l’A2DP
int32_t get_sound_data(Frame* data, int32_t frameCount) {
  return i2s.readBytes((uint8_t*)data, frameCount * BYTES_PER_FRAME) / BYTES_PER_FRAME;
}

void setup() {


  Serial.begin(115200);
  //AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Info);


  // setup du codec ES8388
  Serial.println("starting I2S...");
  auto cfg = i2s.defaultConfig(RX_MODE);
  cfg.i2s_format = I2S_STD_FORMAT;
  cfg.copyFrom(info);
  cfg.input_device = ADC_INPUT_LINE2;
  i2s.begin(cfg);  // maintenant c’est safe

  // démarrage du Bluetooth A2DP
  Serial.println("starting A2DP...");
  a2dp_source.set_data_callback_in_frames(get_sound_data);
  a2dp_source.start("D-58");
}

void loop() {
  delay(1000);
}
