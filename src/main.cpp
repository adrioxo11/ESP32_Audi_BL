#include <Arduino.h>
#include <vector>
#include <string>

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

#include <NimBLEDevice.h>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/AudioBoardStream.h"
#include "AudioTools/AudioLibs/A2DPStream.h"

// --- Paramètres Audio ---
AudioInfo info(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
AudioBoardStream i2s(AudioKitEs8388V1);
const int16_t BYTES_PER_FRAME = 4;

// --- Stockage des devices ---
struct BtDevice {
  String name;
  String mac_str; // adresse MAC format texte "AA:BB:CC:DD:EE:FF"
};

std::vector<BtDevice> devices_found;

// --- BLE Service et caractéristiques ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_NOTIFY_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8" // envoie la liste d'appareils
#define CHAR_WRITE_UUID     "d3b5f2e0-9f3a-11ee-be56-0242ac120002" // reçoit l'adresse MAC choisie

NimBLECharacteristic* pCharacteristicNotify = nullptr;
NimBLECharacteristic* pCharacteristicWrite = nullptr;

bool notifyClients = false;

// --- Prototype ---
void startBTScan();
void connectToSelectedDevice(const String& mac);

// ---------------------------------------------------
// Convertit adresse binaire en string "XX:XX:XX:XX:XX:XX"
String macToString(esp_bd_addr_t bda) {
  char mac[18];
  sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return String(mac);
}

// --- Bluetooth Classique callback pour scan ---
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
      esp_bd_addr_t bda;
      memcpy(bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
      String mac = macToString(bda);
      String device_name = "Inconnu";

      // Cherche nom dans props
      for (int i = 0; i < param->disc_res.num_prop; i++) {
        esp_bt_gap_dev_prop_t *p = param->disc_res.prop + i;
        if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
          device_name = String((char*)p->val, p->len);
        }
      }

      // Vérifie si déjà dans la liste
      bool found = false;
      for (auto& d : devices_found) {
        if (d.mac_str == mac) {
          found = true;
          break;
        }
      }
      if (!found) {
        devices_found.push_back({device_name, mac});
        Serial.printf("Appareil trouvé: %s [%s]\n", device_name.c_str(), mac.c_str());
      }
      break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        Serial.println("Scan terminé.");

        // Envoie la liste via BLE (notifications)
        if (notifyClients && pCharacteristicNotify) {
          for (auto& d : devices_found) {
            String msg = d.mac_str + " - " + d.name;
            pCharacteristicNotify->setValue(msg.c_str());
            pCharacteristicNotify->notify();
            delay(100); // petit délai entre notifications
          }
        }
      } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        Serial.println("Scan démarré...");
      }
      break;
    }
    default:
      break;
  }
}

// --- Callback quand un client BLE écrit dans la caractéristique (adresse MAC reçue) ---
class WriteCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string rxValue = pChar->getValue();
    if (rxValue.length() == 17) { // format MAC "XX:XX:XX:XX:XX:XX"
      String receivedMac = String(rxValue.c_str());

      Serial.printf("Adresse reçue via BLE: %s\n", receivedMac.c_str());

      // Vérifie dans la liste
      bool found = false;
      for (auto& d : devices_found) {
        if (d.mac_str == receivedMac) {
          found = true;
          break;
        }
      }

      if (found) {
        Serial.println("Adresse valide, connexion à l'appareil...");
        connectToSelectedDevice(receivedMac);
      } else {
        Serial.println("Adresse non reconnue, attente nouvelle adresse...");
      }
    } else {
      Serial.println("Format adresse incorrect.");
    }
  }
};

// --- Connexion A2DP vers l'appareil choisi ---
void connectToSelectedDevice(const String& mac) {
  // TODO : adapter la connexion A2DP en fonction de la MAC reçue
  // Cette partie dépend de ta librairie A2DP.
  // Exemple (pseudocode) :

  // Convertir mac string en esp_bd_addr_t (tableau de 6 octets)
  esp_bd_addr_t bdaddr;
  int values[6];
  if (sscanf(mac.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) {
      bdaddr[i] = (uint8_t)values[i];
    }
  } else {
    Serial.println("Erreur conversion MAC");
    return;
  }

  // Ici, utiliser la fonction de ta lib A2DP pour connecter :
  // a2dp_source.connect(bdaddr);  <-- si ta lib supporte ce call

  Serial.printf("Connexion à l'adresse %s...\n", mac.c_str());

  // Pour l’exemple, on simule le début du stream audio
  a2dp_source.start("D-58"); // garde ton nom d’enceinte ici ou dynamique
}

// --- Setup audio I2S ---
int32_t get_sound_data(Frame* data, int32_t frameCount) {
  return i2s.readBytes((uint8_t*)data, frameCount * BYTES_PER_FRAME) / BYTES_PER_FRAME;
}

// --- Setup BLE & BT class ---
void setup() {
  Serial.begin(115200);

  // Bluetooth classique init
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

  esp_bluedroid_init();
  esp_bluedroid_enable();

  esp_bt_gap_register_callback(bt_app_gap_cb);

  // Audio I2S init
  auto cfg = i2s.defaultConfig(RX_MODE);
  cfg.i2s_format = I2S_STD_FORMAT;
  cfg.copyFrom(info);
  cfg.input_device = ADC_INPUT_LINE2;
  i2s.begin(cfg);

  // A2DP source init
  a2dp_source.set_data_callback_in_frames(get_sound_data);

  // --- Init BLE ---
  NimBLEDevice::init("ESP32_BLE_Scanner");
  NimBLEServer* pServer = NimBLEDevice::createServer();

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristicNotify = pService->createCharacteristic(
      CHAR_NOTIFY_UUID,
      NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristicWrite = pService->createCharacteristic(
      CHAR_WRITE_UUID,
      NIMBLE_PROPERTY::WRITE
  );
  pCharacteristicWrite->setCallbacks(new WriteCallback());

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  notifyClients = true;

  // Démarre scan Bluetooth classique
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}

void loop() {
  // Tu peux ici relancer un scan périodique si tu veux
}
