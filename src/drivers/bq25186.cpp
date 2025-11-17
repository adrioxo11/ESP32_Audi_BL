#include "bq25186.h"

BQ25186::BQ25186(uint8_t address) : addr(address) {}

void BQ25186::begin() {
    // Exemple de config tirée de ton code
    updateBits(0x08, 0b11000000, 0b00000000);
    updateBits(0x07, 0b10000011, 0b00000011);
    updateBits(0x04, 0b01111111, 0b00101111);

    Serial.println("BQ25186 init done");
}

void BQ25186::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t BQ25186::readRegister(uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

void BQ25186::updateBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = readRegister(reg);
    uint8_t updated = (current & ~mask) | (value & mask);
    writeRegister(reg, updated);

    Serial.printf("Reg 0x%02X updated: 0x%02X\n", reg, updated);
}
