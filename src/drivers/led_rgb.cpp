#include "led_rgb.h"

LedRGB::LedRGB(int r, int g, int b) : pinR(r), pinG(g), pinB(b) {}

void LedRGB::begin() {
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
}

void LedRGB::setColor(bool r, bool g, bool b) {
    digitalWrite(pinR, r);
    digitalWrite(pinG, g);
    digitalWrite(pinB, b);
}

void LedRGB::cycle() {
    setColor(HIGH, HIGH, HIGH);

    switch (state) {
        case 0: digitalWrite(pinR, LOW); break;
        case 1: digitalWrite(pinG, LOW); break;
        case 2: digitalWrite(pinB, LOW); break;
    }

    state = (state + 1) % 3;
}
