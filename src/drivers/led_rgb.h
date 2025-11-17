#pragma once
#include <Arduino.h>

class LedRGB {
public:
    LedRGB(int r, int g, int b);
    void begin();
    void setColor(bool r, bool g, bool b);
    void cycle();

private:
    int pinR, pinG, pinB;
    int state = 0;
};
