#include <hkAuthContext.h>
#include <HomeKey.h>
#include <array>
#include <utils.h>
#include <HomeSpan.h>
#include <PN532_SPI.h>
#include <PN532.h>
#include <HAP.h>
#include <chrono>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <HK_HomeKit.h>
#include "config.h"
#include <esp_ota_ops.h>
#include <esp_task.h>
#include <pins_arduino.h>

const char* TAG = "MAIN";

AsyncWebServer webServer(80);
PN532_SPI *pn532spi;
PN532 *nfc;
QueueHandle_t gpio_led_handle = nullptr;
QueueHandle_t gpio_lock_handle = nullptr;
TaskHandle_t gpio_led_task_handle = nullptr;
TaskHandle_t gpio_lock_task_handle = nullptr;

nvs_handle savedData;
readerData_t readerData;
uint8_t ecpData[18] = { 0x6A, 0x2, 0xCB, 0x2, 0x6, 0x2, 0x11, 0x0 };
const std::array<std::array<uint8_t, 6>, 4> hk_color_vals = { {{0x01,0x04,0xce,0xd5,0xda,0x00}, {0x01,0x04,0xaa,0xd6,0xec,0x00}, {0x01,0x04,0xe3,0xe3,0xe3,0x00}, {0x01,0x04,0x00,0x00,0x00,0x00}} };
struct gpioLockAction
{
  enum
  {
    HOMEKIT = 1,
    HOMEKEY = 2,
    OTHER = 3
  };
  uint8_t source;
  uint8_t action;
};

std::string platform_create_id_string(void) {
  uint8_t mac[6];
  char id_string[32];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  sprintf(id_string, "ESP32_%02x%02X%02X", mac[3], mac[4], mac[5]);
  return std::string(id_string);
}

namespace espConfig
{
  struct misc_config_t
  {
    std::string deviceName = DEVICE_NAME;
    std::string otaPasswd = OTA_PWD;
    uint8_t hk_key_color = HOMEKEY_COLOR;
    std::string setupCode = SETUP_CODE;
    uint8_t gpioActionPin = GPIO_ACTION_PIN;
    bool gpioActionLockState = GPIO_ACTION_LOCK_STATE;
    bool gpioActionUnlockState = GPIO_ACTION_UNLOCK_STATE;
    uint8_t gpioActionMomentaryEnabled = GPIO_ACTION_MOMENTARY_STATE;
    bool hkGpioControlledState = true;
    uint16_t gpioActionMomentaryTimeout = GPIO_ACTION_MOMENTARY_TIMEOUT;
    bool webAuthEnabled = WEB_AUTH_ENABLED;
    std::string webUsername = WEB_AUTH_USERNAME;
    std::string webPassword = WEB_AUTH_PASSWORD;
    std::array<uint8_t, 4> nfcGpioPins{SS, SCK, MISO, MOSI};
    uint8_t btrLowStatusThreshold = 10;
    bool proxBatEnabled = false;
    bool hkDumbSwitchMode = false;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(misc_config_t, deviceName, otaPasswd, hk_key_color, setupCode, gpioActionPin, gpioActionLockState, gpioActionUnlockState, gpioActionMomentaryEnabled, gpioActionMomentaryTimeout, webAuthEnabled, webUsername, webPassword, nfcGpioPins, btrLowStatusThreshold, proxBatEnabled, hkDumbSwitchMode)
  } miscConfig;
};

KeyFlow hkFlow = KeyFlow::kFlowFAST;
SpanCharacteristic* lockCurrentState;
SpanCharacteristic* lockTargetState;

bool save_to_nvs() {
  std::vector<uint8_t> serialized = nlohmann::json::to_msgpack(readerData);
  esp_err_t set_nvs = nvs_set_blob(savedData, "READERDATA", serialized.data(), serialized.size());
  esp_err_t commit_nvs = nvs_commit(savedData);
  LOG(D, "NVS SET STATUS: %s", esp_err_to_name(set_nvs));
  LOG(D, "NVS COMMIT STATUS: %s", esp_err_to_name(commit_nvs));
  return !set_nvs && !commit_nvs;
}

struct LockManagement : Service::LockManagement
{
  SpanCharacteristic* lockControlPoint;
  SpanCharacteristic* version;
  const char* TAG = "LockManagement";

  LockManagement() : Service::LockManagement() {

    LOG(D, "Configuring LockManagement"); // initialization message

    lockControlPoint = new Characteristic::LockControlPoint();
    version = new Characteristic::Version();

  } // end constructor

}; // end LockManagement

// Function to calculate CRC16
void crc16a(unsigned char* data, unsigned int size, unsigned char* result) {
  unsigned short w_crc = 0x6363;

  for (unsigned int i = 0; i < size; ++i) {
    unsigned char byte = data[i];
    byte = (byte ^ (w_crc & 0x00FF));
    byte = ((byte ^ (byte << 4)) & 0xFF);
    w_crc = ((w_crc >> 8) ^ (byte << 8) ^ (byte << 3) ^ (byte >> 4)) & 0xFFFF;
  }

  result[0] = static_cast<unsigned char>(w_crc & 0xFF);
  result[1] = static_cast<unsigned char>((w_crc >> 8) & 0xFF);
}

// Function to append CRC16 to data
void with_crc16(unsigned char* data, unsigned int size, unsigned char* result) {
  crc16a(data, size, result);
}


void gpio_task(void* arg) {
  gpioLockAction status;
  while (1) {
    if (gpio_lock_handle != nullptr) {
      status = {};
      if (uxQueueMessagesWaiting(gpio_lock_handle) > 0) {
        xQueueReceive(gpio_lock_handle, &status, 0);
        LOG(D, "Got something in queue - source = %d action = %d", status.source, status.action);
        if (status.action == 0) {
          LOG(D, "%d - %d - %d -%d", espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionMomentaryEnabled);
            int currentState = lockCurrentState->getVal();
            if (status.source != gpioLockAction::HOMEKIT) {
              lockTargetState->setVal(!currentState);
            }
            digitalWrite(espConfig::miscConfig.gpioActionPin, currentState == lockStates::UNLOCKED ? espConfig::miscConfig.gpioActionLockState : espConfig::miscConfig.gpioActionUnlockState);
            lockCurrentState->setVal(!currentState);
            if ((static_cast<uint8_t>(espConfig::miscConfig.gpioActionMomentaryEnabled) & status.source) && currentState == lockStates::LOCKED) {
              delay(espConfig::miscConfig.gpioActionMomentaryTimeout);
              lockTargetState->setVal(currentState);
              digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionLockState);
              lockCurrentState->setVal(currentState);
            }
        } else if (status.action == 2) {
          vTaskDelete(NULL);
          return;
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

struct LockMechanism : Service::LockMechanism
{
  const char* TAG = "LockMechanism";

  LockMechanism() : Service::LockMechanism() {
    LOG(I, "Configuring LockMechanism"); // initialization message
    lockCurrentState = new Characteristic::LockCurrentState(1, true);
    lockTargetState = new Characteristic::LockTargetState(1, true);
    memcpy(ecpData + 8, readerData.reader_gid.data(), readerData.reader_gid.size());
    with_crc16(ecpData, 16, ecpData + 16);
    if (espConfig::miscConfig.gpioActionPin != 255) {
      if (lockCurrentState->getVal() == lockStates::LOCKED) {
        digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionLockState);
      } else if (lockCurrentState->getVal() == lockStates::UNLOCKED) {
        digitalWrite(espConfig::miscConfig.gpioActionPin, espConfig::miscConfig.gpioActionUnlockState);
      }
    }
  } // end constructor

  boolean update() {
    int targetState = lockTargetState->getNewVal();
    LOG(I, "New LockState=%d, Current LockState=%d", targetState, lockCurrentState->getVal());
    if (espConfig::miscConfig.gpioActionPin != 255) {
      const gpioLockAction gpioAction{ .source = gpioLockAction::HOMEKIT, .action = 0 };
      xQueueSend(gpio_lock_handle, &gpioAction, 0);
    } else if (espConfig::miscConfig.hkDumbSwitchMode) {
      lockCurrentState->setVal(targetState);
    }
    return (true);
  }
};

struct NFCAccess : Service::NFCAccess
{
  SpanCharacteristic* configurationState;
  SpanCharacteristic* nfcControlPoint;
  SpanCharacteristic* nfcSupportedConfiguration;
  const char* TAG = "NFCAccess";

  NFCAccess() : Service::NFCAccess() {
    LOG(I, "Configuring NFCAccess"); // initialization message
    configurationState = new Characteristic::ConfigurationState();
    nfcControlPoint = new Characteristic::NFCAccessControlPoint();
    TLV8 conf(NULL, 0);
    conf.add(0x01, 0x10);
    conf.add(0x02, 0x10);
    nfcSupportedConfiguration = new Characteristic::NFCAccessSupportedConfiguration(conf);
  }

  boolean update() {
    LOG(D, "PROVISIONED READER KEY: %s", utils::bufToHexString(readerData.reader_pk.data(), readerData.reader_pk.size()).c_str());
    LOG(D, "READER GROUP IDENTIFIER: %s", utils::bufToHexString(readerData.reader_gid.data(), readerData.reader_gid.size()).c_str());
    LOG(D, "READER UNIQUE IDENTIFIER: %s", utils::bufToHexString(readerData.reader_id.data(), readerData.reader_id.size()).c_str());

    TLV8 ctrlData(NULL, 0);
    nfcControlPoint->getNewTLV(ctrlData);
    std::vector<uint8_t> tlvData(ctrlData.pack_size());
    ctrlData.pack(tlvData.data());
    if (tlvData.size() == 0)
      return false;
    LOG(D, "Decoded data: %s", utils::bufToHexString(tlvData.data(), tlvData.size()).c_str());
    LOG(D, "Decoded data length: %d", tlvData.size());
    HK_HomeKit hkCtx(readerData, savedData, "READERDATA", tlvData);
    std::vector<uint8_t> result = hkCtx.processResult();
    if (readerData.reader_gid.size() > 0) {
      memcpy(ecpData + 8, readerData.reader_gid.data(), readerData.reader_gid.size());
      with_crc16(ecpData, 16, ecpData + 16);
    }
    TLV8 res(NULL, 0);
    res.unpack(result.data(), result.size());
    nfcControlPoint->setTLV(res, false);
    return true;
  }

};

void deleteReaderData(const char* buf = "") {
  esp_err_t erase_nvs = nvs_erase_key(savedData, "READERDATA");
  esp_err_t commit_nvs = nvs_commit(savedData);
  readerData.issuers.clear();
  readerData.reader_gid.clear();
  readerData.reader_id.clear();
  readerData.reader_pk.clear();
  readerData.reader_pk_x.clear();
  readerData.reader_sk.clear();
  LOG(D, "*** NVS W STATUS");
  LOG(D, "ERASE: %s", esp_err_to_name(erase_nvs));
  LOG(D, "COMMIT: %s", esp_err_to_name(commit_nvs));
  LOG(D, "*** NVS W STATUS");
}

void pairCallback() {
  if (HAPClient::nAdminControllers() == 0) {
    deleteReaderData(NULL);
    return;
  }
  for (auto it = homeSpan.controllerListBegin(); it != homeSpan.controllerListEnd(); ++it) {
    std::vector<uint8_t> id = utils::getHashIdentifier(it->getLTPK(), 32, true);
    LOG(D, "Found allocated controller - Hash: %s", utils::bufToHexString(id.data(), 8).c_str());
    hkIssuer_t* foundIssuer = nullptr;
    for (auto&& issuer : readerData.issuers) {
      if (std::equal(issuer.issuer_id.begin(), issuer.issuer_id.end(), id.begin())) {
        LOG(D, "Issuer %s already added, skipping", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size()).c_str());
        foundIssuer = &issuer;
        break;
      }
    }
    if (foundIssuer == nullptr) {
      LOG(D, "Adding new issuer - ID: %s", utils::bufToHexString(id.data(), 8).c_str());
      hkIssuer_t newIssuer;
      newIssuer.issuer_id = std::vector<uint8_t>{ id.begin(), id.begin() + 8 };
      newIssuer.issuer_pk.insert(newIssuer.issuer_pk.begin(), it->getLTPK(), it->getLTPK() + 32);
      readerData.issuers.emplace_back(newIssuer);
    }
  }
  save_to_nvs();
}

void setFlow(const char* buf) {
  switch (buf[1]) {
  case '0':
    hkFlow = KeyFlow::kFlowFAST;
    Serial.println("FAST Flow");
    break;

  case '1':
    hkFlow = KeyFlow::kFlowSTANDARD;
    Serial.println("STANDARD Flow");
    break;
  case '2':
    hkFlow = KeyFlow::kFlowATTESTATION;
    Serial.println("ATTESTATION Flow");
    break;

  default:
    Serial.println("0 = FAST flow, 1 = STANDARD Flow, 2 = ATTESTATION Flow");
    break;
  }
}

void setLogLevel(const char* buf) {
  esp_log_level_t level = esp_log_level_get("*");
  if (strncmp(buf + 1, "E", 1) == 0) {
    level = ESP_LOG_ERROR;
    Serial.println("ERROR");
  } else if (strncmp(buf + 1, "W", 1) == 0) {
    level = ESP_LOG_WARN;
    Serial.println("WARNING");
  } else if (strncmp(buf + 1, "I", 1) == 0) {
    level = ESP_LOG_INFO;
    Serial.println("INFO");
  } else if (strncmp(buf + 1, "D", 1) == 0) {
    level = ESP_LOG_DEBUG;
    Serial.println("DEBUG");
  } else if (strncmp(buf + 1, "V", 1) == 0) {
    level = ESP_LOG_VERBOSE;
    Serial.println("VERBOSE");
  } else if (strncmp(buf + 1, "N", 1) == 0) {
    level = ESP_LOG_NONE;
    Serial.println("NONE");
  }

  esp_log_level_set(TAG, level);
  esp_log_level_set("HK_HomeKit", level);
  esp_log_level_set("HKAuthCtx", level);
  esp_log_level_set("HKFastAuth", level);
  esp_log_level_set("HKStdAuth", level);
  esp_log_level_set("HKAttestAuth", level);
  esp_log_level_set("PN532", level);
  esp_log_level_set("PN532_SPI", level);
  esp_log_level_set("ISO18013_SC", level);
  esp_log_level_set("LockMechanism", level);
  esp_log_level_set("NFCAccess", level);
  esp_log_level_set("actions-config", level);
  esp_log_level_set("misc-config", level);
}

void print_issuers(const char* buf) {
  for (auto&& issuer : readerData.issuers) {
    LOG(I, "Issuer ID: %s, Public Key: %s", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size()).c_str(), utils::bufToHexString(issuer.issuer_pk.data(), issuer.issuer_pk.size()).c_str());
    for (auto&& endpoint : issuer.endpoints) {
      LOG(I, "Endpoint ID: %s, Public Key: %s", utils::bufToHexString(endpoint.endpoint_id.data(), endpoint.endpoint_id.size()).c_str(), utils::bufToHexString(endpoint.endpoint_pk.data(), endpoint.endpoint_pk.size()).c_str());
    }
  }
}

void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

String miscHtmlProcess(const String& var) {
  if (var == "DEVICENAME") {
    return String(espConfig::miscConfig.deviceName.c_str());
  } else if (var == "OTAPASSWD") {
    return String(espConfig::miscConfig.otaPasswd.c_str());
  } else if (var == "HKSETUPCODE") {
    return String(espConfig::miscConfig.setupCode.c_str());
  } else if (var == "HWFINISH") {
    return String(espConfig::miscConfig.hk_key_color);
  } else if (var == "WEBENABLE") {
    return String(espConfig::miscConfig.webAuthEnabled);
  } else if (var == "WEBUSERNAME") {
    return String(espConfig::miscConfig.webUsername.c_str());
  } else if (var == "WEBPASSWORD") {
    return String(espConfig::miscConfig.webPassword.c_str());
  } else if (var == "NFCSSGPIOPIN") {
    return String(espConfig::miscConfig.nfcGpioPins[0]);
  } else if (var == "NFCSCKGPIOPIN") {
    return String(espConfig::miscConfig.nfcGpioPins[1]);
  } else if (var == "NFCMISOGPIOPIN") {
    return String(espConfig::miscConfig.nfcGpioPins[2]);
  } else if (var == "NFCMOSIGPIOPIN") {
    return String(espConfig::miscConfig.nfcGpioPins[3]);
  }
  return String();
}

String hkInfoHtmlProcess(const String& var) {
  String result = "";
  if (var == "READERGID") {
    return String(utils::bufToHexString(readerData.reader_gid.data(), readerData.reader_gid.size(), true).c_str());
  } else if (var == "READERID") {
    return String(utils::bufToHexString(readerData.reader_id.data(), readerData.reader_id.size(), true).c_str());
  } else if (var == "ISSUERSNO") {
    return String(readerData.issuers.size());
  } else if (var == "ISSUERSLIST") {
    for (auto&& issuer : readerData.issuers) {
      char issuerBuff[21 + 8];
      result += "<li>";
      snprintf(issuerBuff, sizeof(issuerBuff), "Issuer ID: %s", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size(), true).c_str());
      result += issuerBuff;
      result += "</li>\n";
      result += "\t\t<ul>";
      for (auto&& endpoint : issuer.endpoints) {
        char endBuff[23 + 6];
        result += "\n\t\t\t<li>";
        snprintf(endBuff, sizeof(endBuff), "Endpoint ID: %s", utils::bufToHexString(endpoint.endpoint_id.data(), endpoint.endpoint_id.size(), true).c_str());
        result += endBuff;
        result += "</li>\n";
      }
      result += "\t\t</ul>";
    }
    return result;
  }
  return result;
}

String indexProcess(const String& var) {
  if (var == "VERSION") {
    const esp_app_desc_t* app_desc = esp_ota_get_app_description();
    std::string app_version = app_desc->version;
    return String(app_version.c_str());
  }
  return "";
}

String actionsProcess(const String& var) {
  if (var == "GPIOAPIN") {
    return String(espConfig::miscConfig.gpioActionPin);
  } else if (var == "GPIOALOCK") {
    return String(espConfig::miscConfig.gpioActionLockState);
  } else if (var == "GPIOAUNLOCK") {
    return String(espConfig::miscConfig.gpioActionUnlockState);
  } else if (var == "GPIOAMOEN") {
    return String(espConfig::miscConfig.gpioActionMomentaryEnabled);
  } else if (var == "GPIOAMOTIME") {
    return String(espConfig::miscConfig.gpioActionMomentaryTimeout);
  } else if (var == "HKGPIOCONTROLSTATE") {
    return String(espConfig::miscConfig.hkGpioControlledState);
  }
  return "";
}
bool headersFix(AsyncWebServerRequest* request) { request->addInterestingHeader("ANY"); return true; };
void setupWeb() {
  auto infoHandle = new AsyncStaticWebHandler("/info", LittleFS, "/routes/info.html", NULL);
  webServer.addHandler(infoHandle);
  infoHandle->setTemplateProcessor(hkInfoHtmlProcess).setFilter(headersFix);
  auto miscHandle = new AsyncStaticWebHandler("/misc", LittleFS, "/routes/misc.html", NULL);
  webServer.addHandler(miscHandle);
  miscHandle->setTemplateProcessor(miscHtmlProcess).setFilter(headersFix);
  auto actionsHandle = new AsyncStaticWebHandler("/actions", LittleFS, "/routes/actions.html", NULL);
  webServer.addHandler(actionsHandle);
  actionsHandle->setTemplateProcessor(actionsProcess).setFilter(headersFix);
  auto assetsHandle = new AsyncStaticWebHandler("/assets", LittleFS, "/assets/", NULL);
  webServer.addHandler(assetsHandle);
  actionsHandle->setFilter(headersFix);
  AsyncCallbackWebHandler* rootHandle = new AsyncCallbackWebHandler();
  webServer.addHandler(rootHandle);
  rootHandle->setUri("/");
  rootHandle->setMethod(HTTP_GET);
  rootHandle->onRequest([](AsyncWebServerRequest* req) {
    req->send(LittleFS, "/index.html", "text/html", false, indexProcess);
  });
  auto miscConfigHandle = new AsyncCallbackWebHandler();
  miscConfigHandle->setUri("/misc-config");
  miscConfigHandle->setMethod(HTTP_POST);
  miscConfigHandle->onRequest([](AsyncWebServerRequest* request) {
    const char* TAG = "misc-config";
    int params = request->params();
    for (int i = 0; i < params; i++) {
      const AsyncWebParameter* p = request->getParam(i);
      LOG(V, "POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      if (!strcmp(p->name().c_str(), "device-name")) {
        espConfig::miscConfig.deviceName = p->value().c_str();
      } else if (!strcmp(p->name().c_str(), "ota-passwd")) {
        espConfig::miscConfig.otaPasswd = p->value().c_str();
      } else if (!strcmp(p->name().c_str(), "hk-setupcode")) {
        if (strcmp(espConfig::miscConfig.setupCode.c_str(), p->value().c_str()) && p->value().length() == 8) {
          if (homeSpan.controllerListBegin() == homeSpan.controllerListEnd()) {
            homeSpan.setPairingCode(p->value().c_str());
            espConfig::miscConfig.setupCode = p->value().c_str();
          }
        }
      } else if (!strcmp(p->name().c_str(), "hk-hwfinish")) {
        espConfig::miscConfig.hk_key_color = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "web-auth-enable")) {
        espConfig::miscConfig.webAuthEnabled = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "web-auth-username")) {
        espConfig::miscConfig.webUsername = p->value().c_str();
      } else if (!strcmp(p->name().c_str(), "web-auth-password")) {
        espConfig::miscConfig.webPassword = p->value().c_str();
      } else if (!strcmp(p->name().c_str(), "nfc-ss-gpio-pin")) {
        if (!GPIO_IS_VALID_GPIO(p->value().toInt()) && !GPIO_IS_VALID_OUTPUT_GPIO(p->value().toInt()) && p->value().toInt() != 255) {
          std::string msg = p->value().c_str();
          msg.append(" is not a valid GPIO Pin");
          request->send(200, "text/plain", msg.c_str());
          return;
        }
        espConfig::miscConfig.nfcGpioPins[0] = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "nfc-sck-gpio-pin")) {
        if (!GPIO_IS_VALID_GPIO(p->value().toInt()) && !GPIO_IS_VALID_OUTPUT_GPIO(p->value().toInt()) && p->value().toInt() != 255) {
          std::string msg = p->value().c_str();
          msg.append(" is not a valid GPIO Pin");
          request->send(200, "text/plain", msg.c_str());
          return;
        }
        espConfig::miscConfig.nfcGpioPins[1] = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "nfc-miso-gpio-pin")) {
        if (!GPIO_IS_VALID_GPIO(p->value().toInt()) && !GPIO_IS_VALID_OUTPUT_GPIO(p->value().toInt()) && p->value().toInt() != 255) {
          std::string msg = p->value().c_str();
          msg.append(" is not a valid GPIO Pin");
          request->send(200, "text/plain", msg.c_str());
          return;
        }
        espConfig::miscConfig.nfcGpioPins[2] = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "nfc-mosi-gpio-pin")) {
        if (!GPIO_IS_VALID_GPIO(p->value().toInt()) && !GPIO_IS_VALID_OUTPUT_GPIO(p->value().toInt()) && p->value().toInt() != 255) {
          std::string msg = p->value().c_str();
          msg.append(" is not a valid GPIO Pin");
          request->send(200, "text/plain", msg.c_str());
          return;
        }
        espConfig::miscConfig.nfcGpioPins[3] = p->value().toInt();
      }
    }
    json json_misc_config = espConfig::miscConfig;
    std::vector<uint8_t> misc_buf = nlohmann::json::to_msgpack(json_misc_config);
    esp_err_t set_nvs = nvs_set_blob(savedData, "MISCDATA", misc_buf.data(), misc_buf.size());
    esp_err_t commit_nvs = nvs_commit(savedData);
    LOG(V, "SET_STATUS: %s", esp_err_to_name(set_nvs));
    LOG(V, "COMMIT_STATUS: %s", esp_err_to_name(commit_nvs));

    request->send(200, "text/plain", "Config Saved, Restarting...");
    delay(1000);
    ESP.restart();
    });
  webServer.addHandler(miscConfigHandle);
  auto actionsConfigHandle = new AsyncCallbackWebHandler();
  actionsConfigHandle->setUri("/actions-config");
  actionsConfigHandle->setMethod(HTTP_POST);
  actionsConfigHandle->onRequest([](AsyncWebServerRequest* request) {
    const char* TAG = "actions-config";
    int params = request->params();
    for (int i = 0; i < params; i++) {
      const AsyncWebParameter* p = request->getParam(i);
      LOG(V, "POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      if (!strcmp(p->name().c_str(), "gpio-a-pin")) {
        if (!GPIO_IS_VALID_GPIO(p->value().toInt()) && !GPIO_IS_VALID_OUTPUT_GPIO(p->value().toInt()) && p->value().toInt() != 255) {
          std::string msg = p->value().c_str();
          msg.append(" is not a valid GPIO Pin");
          request->send(200, "text/plain", msg.c_str());
          return;
        }
        if (espConfig::miscConfig.gpioActionPin == 255 && p->value().toInt() != 255 && gpio_lock_task_handle == nullptr) {
          pinMode(p->value().toInt(), OUTPUT);
          xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 2, &gpio_lock_task_handle);
        } else if (espConfig::miscConfig.gpioActionPin != 255 && p->value().toInt() == 255 && gpio_lock_task_handle != nullptr) {
          gpioLockAction status{ .source = gpioLockAction::OTHER, .action = 2 };
          xQueueSend(gpio_lock_handle, &status, 0);
          gpio_lock_task_handle = nullptr;
        }
        espConfig::miscConfig.gpioActionPin = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "gpio-a-lock")) {
        espConfig::miscConfig.gpioActionLockState = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "homekey-gpio-state")) {
        espConfig::miscConfig.hkGpioControlledState = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "gpio-a-unlock")) {
        espConfig::miscConfig.gpioActionUnlockState = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "gpio-a-momentary")) {
        espConfig::miscConfig.gpioActionMomentaryEnabled = p->value().toInt();
      } else if (!strcmp(p->name().c_str(), "gpio-a-mo-timeout")) {
        espConfig::miscConfig.gpioActionMomentaryTimeout = p->value().toInt();
      }
    }
    json json_misc_config = espConfig::miscConfig;
    std::vector<uint8_t> misc_buf = nlohmann::json::to_msgpack(json_misc_config);
    esp_err_t set_nvs = nvs_set_blob(savedData, "MISCDATA", misc_buf.data(), misc_buf.size());
    esp_err_t commit_nvs = nvs_commit(savedData);
    LOG(V, "SET_STATUS: %s", esp_err_to_name(set_nvs));
    LOG(V, "COMMIT_STATUS: %s", esp_err_to_name(commit_nvs));

    request->send(200, "text/plain", "Configuration applied!");
    });
  webServer.addHandler(actionsConfigHandle);
  auto rebootDeviceHandle = new AsyncCallbackWebHandler();
  rebootDeviceHandle->setUri("/reboot_device");
  rebootDeviceHandle->setMethod(HTTP_GET);
  rebootDeviceHandle->onRequest([](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Rebooting the device...");
    delay(1000);
    ESP.restart();
    });
  webServer.addHandler(rebootDeviceHandle);
  auto resetHkHandle = new AsyncCallbackWebHandler();
  resetHkHandle->setUri("/reset_hk_pair");
  resetHkHandle->setMethod(HTTP_GET);
  resetHkHandle->onRequest([](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Erasing HomeKit pairings and restarting...");
    delay(1000);
    deleteReaderData();
    homeSpan.processSerialCommand("H");
    });
  webServer.addHandler(resetHkHandle);
  auto resetWifiHandle = new AsyncCallbackWebHandler();
  resetWifiHandle->setUri("/reset_wifi_cred");
  resetWifiHandle->setMethod(HTTP_GET);
  resetWifiHandle->onRequest([](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Erasing WiFi credentials and restarting, AP will start on boot...");
    delay(1000);
    homeSpan.processSerialCommand("X");
    });
  webServer.addHandler(resetWifiHandle);
  auto getWifiRssi = new AsyncCallbackWebHandler();
  getWifiRssi->setUri("/get_wifi_rssi");
  getWifiRssi->setMethod(HTTP_GET);
  getWifiRssi->onRequest([](AsyncWebServerRequest* request) {
    std::string rssi_val = std::to_string(WiFi.RSSI());
    request->send(200, "text/plain", rssi_val.c_str());
    });
  webServer.addHandler(getWifiRssi);
  if (espConfig::miscConfig.webAuthEnabled) {
    LOG(I, "Web Authentication Enabled");
    infoHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    miscHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    actionsHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    assetsHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    rootHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    miscConfigHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    actionsConfigHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    resetHkHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
    resetWifiHandle->setAuthentication(espConfig::miscConfig.webUsername.c_str(), espConfig::miscConfig.webPassword.c_str());
  }
  webServer.onNotFound(notFound);
  webServer.begin();
}


std::string hex_representation(const std::vector<uint8_t>& v) {
  std::string hex_tmp;
  for (auto x : v) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (unsigned)x;
    hex_tmp += oss.str();
  }
  return hex_tmp;
}

void nfc_thread_entry(void* arg) {
  nfc->begin();

  uint32_t versiondata = nfc->getFirmwareVersion();
  if (!versiondata) {
    ESP_LOGE("NFC_SETUP", "Didn't find PN53x board");
  } else {
    unsigned int model = (versiondata >> 24) & 0xFF;
    ESP_LOGI("NFC_SETUP", "Found chip PN5%x", model);
    int maj = (versiondata >> 16) & 0xFF;
    int min = (versiondata >> 8) & 0xFF;
    ESP_LOGI("NFC_SETUP", "Firmware ver. %d.%d", maj, min);
    nfc->SAMConfig();
    nfc->setRFField(0x02, 0x01);
    nfc->setPassiveActivationRetries(0);
    ESP_LOGI("NFC_SETUP", "Waiting for an ISO14443A card");
  }
  memcpy(ecpData + 8, readerData.reader_gid.data(), readerData.reader_gid.size());
  with_crc16(ecpData, 16, ecpData + 16);
  while (1) {
    uint8_t res[4];
    uint16_t resLen = 4;
    nfc->writeRegister(0x633d, 0, true);
    nfc->inCommunicateThru(ecpData, sizeof(ecpData), res, &resLen, 100, true);
    uint8_t uid[16];
    uint8_t uidLen = 0;
    uint8_t atqa[2];
    uint8_t sak[1];
    bool passiveTarget = nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, atqa, sak, 500, true, true);
    if (passiveTarget) {
      nfc->setPassiveActivationRetries(5);
      LOG(D, "ATQA: %02x", atqa[0]);
      LOG(D, "SAK: %02x", sak[0]);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, (size_t)uidLen, ESP_LOG_VERBOSE);
      LOG(I, "*** PASSIVE TARGET DETECTED ***");
      auto startTime = std::chrono::high_resolution_clock::now();
      uint8_t data[13] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x08, 0x58, 0x01, 0x01, 0x0 };
      uint8_t selectCmdRes[9];
      uint16_t selectCmdResLength = 9;
      LOG(I, "Requesting supported HomeKey versions");
      LOG(D, "SELECT HomeKey Applet, APDU: ");
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, sizeof(data), ESP_LOG_VERBOSE);
      bool status = nfc->inDataExchange(data, sizeof(data), selectCmdRes, &selectCmdResLength);
      LOG(D, "SELECT HomeKey Applet, Response");
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, selectCmdRes, selectCmdResLength, ESP_LOG_VERBOSE);
      if (status && selectCmdRes[selectCmdResLength - 2] == 0x90 && selectCmdRes[selectCmdResLength - 1] == 0x00) {
        LOG(D, "*** SELECT HOMEKEY APPLET SUCCESSFUL ***");
        LOG(D, "Reader Private Key: %s", utils::bufToHexString(readerData.reader_pk.data(), readerData.reader_pk.size()).c_str());
        HKAuthenticationContext authCtx(*nfc, readerData, savedData);
        auto authResult = authCtx.authenticate(hkFlow);
        if (std::get<2>(authResult) != kFlowFailed) {
          if (espConfig::miscConfig.gpioActionPin != 255 && espConfig::miscConfig.hkGpioControlledState) {
            const gpioLockAction action{ .source = gpioLockAction::HOMEKEY, .action = 0 };
            xQueueSend(gpio_lock_handle, &action, 0);
          }
          json payload;
          payload["issuerId"] = hex_representation(std::get<0>(authResult));
          payload["endpointId"] = hex_representation(std::get<1>(authResult));
          payload["readerId"] = hex_representation(readerData.reader_id);
          payload["homekey"] = true;
          std::string payloadStr = payload.dump();

          auto stopTime = std::chrono::high_resolution_clock::now();
          LOG(I, "Total Time (detection->auth->gpio): %lli ms", std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime).count());
        } else {
          LOG(W, "We got status FlowFailed!");
        }
        nfc->setRFField(0x02, 0x01);
      }
      vTaskDelay(50 / portTICK_PERIOD_MS);
      nfc->inRelease();
      int counter = 50;
      bool deviceStillInField = nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen);
      LOG(D, "Target still present: %d", deviceStillInField);
      while (deviceStillInField) {
        if (counter == 0) break;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        nfc->inRelease();
        deviceStillInField = nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen);
        --counter;
        LOG(D, "Target still present: %d Counter=%d", deviceStillInField, counter);
      }
      nfc->inRelease();
      nfc->setPassiveActivationRetries(0);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  const esp_app_desc_t* app_desc = esp_ota_get_app_description();
  std::string app_version = "0.2.0";
  gpio_led_handle = xQueueCreate(2, sizeof(uint8_t));
  gpio_lock_handle = xQueueCreate(2, sizeof(gpioLockAction));
  size_t len;
  const char* TAG = "SETUP";
  nvs_open("SAVED_DATA", NVS_READWRITE, &savedData);
  if (!nvs_get_blob(savedData, "READERDATA", NULL, &len)) {
    std::vector<uint8_t> savedBuf(len);
    nvs_get_blob(savedData, "READERDATA", savedBuf.data(), &len);
    LOG(D, "NVS READERDATA LENGTH: %d", len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, savedBuf.data(), savedBuf.size(), ESP_LOG_VERBOSE);
    nlohmann::json data = nlohmann::json::from_msgpack(savedBuf);
    if (!data.is_discarded()) {
      data.get_to<readerData_t>(readerData);
      LOG(I, "Reader Data loaded from NVS");
    }
  }
  if (!nvs_get_blob(savedData, "MISCDATA", NULL, &len)) {
    std::vector<uint8_t> dataBuf(len);
    nvs_get_blob(savedData, "MISCDATA", dataBuf.data(), &len);
    std::string str(dataBuf.begin(), dataBuf.end());
    LOG(D, "NVS MQTTDATA LENGTH: %d", len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, dataBuf.data(), dataBuf.size(), ESP_LOG_VERBOSE);
    auto isValidJson = nlohmann::json::accept(dataBuf);
    if (isValidJson) {
      nlohmann::json data = nlohmann::json::parse(str);
      if (!data.is_discarded()) {
        data.get_to<espConfig::misc_config_t>(espConfig::miscConfig);
        LOG(I, "Misc Config loaded from NVS");
      }
    } else {
      nlohmann::json data = nlohmann::json::from_msgpack(dataBuf);
      if (!data.is_discarded()) {
        data.get_to<espConfig::misc_config_t>(espConfig::miscConfig);
        LOG(I, "Misc Config loaded from NVS");
      }
    }
  }
  pn532spi = new PN532_SPI(espConfig::miscConfig.nfcGpioPins[0], espConfig::miscConfig.nfcGpioPins[1], espConfig::miscConfig.nfcGpioPins[2], espConfig::miscConfig.nfcGpioPins[3]);
  nfc = new PN532(*pn532spi);
  if (espConfig::miscConfig.gpioActionPin && espConfig::miscConfig.gpioActionPin != 255) {
    pinMode(espConfig::miscConfig.gpioActionPin, OUTPUT);
  }
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LITTLEFS");
    return;
  }
  listDir(LittleFS, "/", 0);
  homeSpan.setStatusAutoOff(15);
  homeSpan.setLogLevel(0);
  homeSpan.setSketchVersion(app_version.c_str());

  LOG(I, "READER GROUP ID (%d): %s", readerData.reader_gid.size(), utils::bufToHexString(readerData.reader_gid.data(), readerData.reader_gid.size()).c_str());
  LOG(I, "READER UNIQUE ID (%d): %s", readerData.reader_id.size(), utils::bufToHexString(readerData.reader_id.data(), readerData.reader_id.size()).c_str());

  LOG(I, "HOMEKEY ISSUERS: %d", readerData.issuers.size());
  for (auto&& issuer : readerData.issuers) {
    LOG(D, "Issuer ID: %s, Public Key: %s", utils::bufToHexString(issuer.issuer_id.data(), issuer.issuer_id.size()).c_str(), utils::bufToHexString(issuer.issuer_pk.data(), issuer.issuer_pk.size()).c_str());
  }
  homeSpan.enableAutoStartAP();
  homeSpan.enableOTA(espConfig::miscConfig.otaPasswd.c_str());
  homeSpan.setPortNum(1201);
  homeSpan.begin(Category::Locks, espConfig::miscConfig.deviceName.c_str(), "MI", "MIAU");

  new SpanUserCommand('D', "Delete Home Key Data", deleteReaderData);
  new SpanUserCommand('L', "Set Log Level", setLogLevel);
  new SpanUserCommand('F', "Set HomeKey Flow", setFlow);
  new SpanUserCommand('P', "Print Issuers", print_issuers);

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();
  new Characteristic::Manufacturer("Ursem Motorwerken");
  new Characteristic::Model("MIAU");
  new Characteristic::HardwareRevision("0.2.0");
  new Characteristic::Name(DEVICE_NAME);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[18] = { 0 };
  sprintf(macStr, "%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3]);
  std::string serialNumber = "MI-";
  serialNumber.append(macStr);
  new Characteristic::SerialNumber(serialNumber.c_str());
  new Characteristic::FirmwareRevision(app_version.c_str());
  std::array<uint8_t, 6> decB64 = hk_color_vals[HK_COLOR(espConfig::miscConfig.hk_key_color)];
  TLV8 hwfinish(NULL, 0);
  hwfinish.unpack(decB64.data(), decB64.size());
  new Characteristic::HardwareFinish(hwfinish);

  new LockManagement();
  new LockMechanism();
  new NFCAccess();
  new Service::HAPProtocolInformation();
  new Characteristic::Version();
  homeSpan.setControllerCallback(pairCallback);
  homeSpan.setWifiCallback([]() { setupWeb(); });
  if (espConfig::miscConfig.gpioActionPin != 255) {
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 2, &gpio_lock_task_handle);
  }
  xTaskCreate(nfc_thread_entry, "nfc_task", 8192, NULL, 1, NULL);
}

//////////////////////////////////////

void loop() {
  homeSpan.poll();
  vTaskDelay(5);
}
