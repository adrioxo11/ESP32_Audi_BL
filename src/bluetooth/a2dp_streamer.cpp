#include "a2dp_streamer.h"
#include <AudioTools/AudioLibs/AudioBoardStream.h>

static AudioInfo info(44100, 2, 16);
static AudioBoardStream i2s(AudioKitEs8388V1);
static BluetoothA2DPSource a2dp_source;

const int16_t SILENCE_THRESHOLD = 400;
const int BYTES_PER_FRAME = 4;

int32_t A2DPStreamer::audioCallback(Frame* data, int32_t frameCount) {
    int byteCount = i2s.readBytes((uint8_t*)data, frameCount * BYTES_PER_FRAME);
    int16_t* samples = (int16_t*)data;

    int numSamples = byteCount / 2;
    int64_t total = 0;

    for (int i = 0; i < 100 && i < numSamples; i++)
        total += abs(samples[i]);

    int average = total / numSamples;

    if (average < SILENCE_THRESHOLD)
        memset(data, 0, byteCount);

    return byteCount / BYTES_PER_FRAME;
}

void A2DPStreamer::begin() {
    auto cfg = i2s.defaultConfig(RX_MODE);
    cfg.copyFrom(info);
    cfg.i2s_format = I2S_STD_FORMAT;
    i2s.begin(cfg);

    a2dp_source.set_data_callback_in_frames(audioCallback);
    a2dp_source.start("D-58");
}