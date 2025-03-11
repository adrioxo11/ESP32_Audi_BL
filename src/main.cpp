#include <Arduino.h>
/*
  Complete Getting Started Guide: https://RandomNerdTutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
  Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "BluetoothA2DPSource.h"
#include <math.h>

// Nom de l'enceinte Bluetooth cible
const char* speaker_name = "D-58";

// Objet Bluetooth A2DP Source
BluetoothA2DPSource a2dp_source;

// Paramètres audio
const int sample_rate = 44100; // Hz (standard)
const int frequency = 440;     // Hz (Note LA)
const int amplitude = 3000;    // Amplitude du signal
const int buffer_size = 256;   // Taille du buffer audio

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Callback pour générer et envoyer l'audio
int32_t get_sound_data(uint8_t* data, int32_t len) {
  static float phase = 0.0;
  float phase_increment = 2.0 * M_PI * frequency / sample_rate;

  int16_t* samples = (int16_t*) data; // Conversion du buffer en 16 bits
  int sample_count = len / 2; // Chaque sample = 2 octets

  for (int i = 0; i < sample_count; i += 2) {
      int16_t sample = amplitude * sin(phase);
      phase += phase_increment;
      if (phase >= 2.0 * M_PI) {
          phase -= 2.0 * M_PI;
      }

      // Stéréo : on met le même signal sur les deux canaux (L/R)
      samples[i] = sample;     // Canal gauche
      samples[i + 1] = sample; // Canal droit
  }

  return len; // Retourne le nombre d'octets écrits
}


void setup() {
  Serial.begin(115200);

  //Audio source
  Serial.println("ESP32-A2DP Sinusoidal Sound Test");

  // Définir le callback pour générer du son
  a2dp_source.set_data_callback(get_sound_data);

  // Démarrer l'ESP32 en tant que source Bluetooth
  a2dp_source.start(speaker_name);


  Serial.println("Starting BLE work!");

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
}