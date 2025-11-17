#pragma once
#include <AudioTools.h>
#include <AudioTools/AudioLibs/A2DPStream.h>

class A2DPStreamer {
public:
    void begin();
private:
    static int32_t audioCallback(Frame* data, int32_t frameCount);
};
