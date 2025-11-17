#pragma once
#include <Arduino.h>
#include <Wire.h>

class BQ25186 {
public:
    BQ25186(uint8_t address = 0x6A);
    void begin();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void updateBits(uint8_t reg, uint8_t mask, uint8_t value);

private:
    uint8_t addr;
};