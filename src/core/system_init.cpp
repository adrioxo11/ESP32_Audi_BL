#include "system_init.h"
#include "../drivers/bq25186.h"
#include "../bluetooth/a2dp_streamer.h"

BQ25186 charger;
A2DPStreamer audio;

void system_init() {
    Serial.begin(115200);
    setCpuFrequencyMhz(240);


    Wire.begin(33, 32);
    charger.begin();

    audio.begin();
}
