#include "../bluetooth/a2dp_streamer.h"

static A2DPStreamer audio;
void task_bl_a2dp_stream(void *pv) {
    audio.begin();
    vTaskDelete(NULL);
}
