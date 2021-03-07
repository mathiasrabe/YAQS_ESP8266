
#ifndef CONFIG
  #define CONFIG

// Import required libraries
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "error.h"

// struct for settings like network credentials
struct Config {
  String wifiSSID = "";
  String wifiPassword = "";
  String mqttHost = "";
  uint16_t mqttPort = 1883;
  String mqttUser = "";
  String mqttPassword = "";
  String mqttTopTopic = "";
  float altitude = 0.0;  // in m
  float temp_offset = 0.0;  // in °C
  uint16_t tinyV1vref = 1100;  // in mV
  uint16_t tinyVccmin = 3700;  // in mV
  uint16_t tinyVccmax = 4200;  // in mV
  uint16_t tinyVcchyst = 4000;  // in mV
};
struct Config cfg;

// Use arduinojson.org/assistant to compute the capacity.
const uint16_t fileSize = 512;


void handleJsonDoc(const JsonDocument& jsonDoc, Config& _cfg) {
  // read keys and write them to the config struct _cfg
  _cfg.wifiSSID = jsonDoc["wifi_ssid"].as<String>();
  _cfg.wifiPassword = jsonDoc["wifi_pw"].as<String>();
  _cfg.mqttHost = jsonDoc["mqtt_host"].as<String>();
  _cfg.mqttPort = jsonDoc["mqtt_port"];
  _cfg.mqttTopTopic = jsonDoc["mqtt_top_topic"].as<String>();
  _cfg.altitude = jsonDoc["altitude"];
  _cfg.temp_offset = jsonDoc["temp_offset"];
  _cfg.tinyV1vref = jsonDoc["tiny_v2vref"];
  _cfg.tinyVccmin = jsonDoc["tiny_vccmin"];
  _cfg.tinyVccmax = jsonDoc["tiny_vccmax"];
  _cfg.tinyVcchyst = jsonDoc["tiny_vcchyst"];
}

void deserialize(Config& _cfg, fs::File& input) {
  // deserialize input and write it to _cfg. input must be of type FILE
  StaticJsonDocument<fileSize> jsonDoc;
  deserializeJson(jsonDoc, input);

  handleJsonDoc(jsonDoc, _cfg);
}

void deserialize(Config& _cfg, const char* input) {
  // deserialize input and write it to _cfg. input must be of type char*
  StaticJsonDocument<fileSize> jsonDoc;
  deserializeJson(jsonDoc, input);

  handleJsonDoc(jsonDoc, _cfg);
}

int8_t readConfig() {
  File configFile = LittleFS.open("config.json", "r");

  if (!configFile) {
    errorLog("FATAL: No config file found");
    configFile.close();
    return -1;
  }
  Serial.println("Config file found");
  size_t size = configFile.size();
  if (size > fileSize) {
    errorLog("FATAL: Config file size is too large");
    return -1;
  }

  // deserialize config parameters
  deserialize(cfg, configFile);

  // test config parameters
  if (cfg.wifiSSID == NULL || cfg.wifiPassword == NULL) {
    errorLog("Error: No WIFI credentials found");
    return -1;
  }
  if (cfg.mqttHost == NULL || cfg.mqttPort == 0) {
    errorLog("Error: No MQTT server settings found");
    return -2;
  }
  if (cfg.mqttTopTopic == NULL) {
    Serial.println("Warning: No MQTT top level topic found");
    cfg.mqttTopTopic = "/";
  } else if (!cfg.mqttTopTopic.endsWith("/")) {
    cfg.mqttTopTopic += String("/");
  }
  /*
  if (cfg.altitude == 0) {
    Serial.println("Warning: No altitude found, using 0");
  if (cfg.temp_offset == 0) {
    Serial.println("Warning: No temperature offset found, using 0");
  }*/

  configFile.close();
  return 1;
}

int8_t writeConfig(String message) {
  // Will write message to config.json. Will return -1 if config file could not be opened to write; 2 if settings
  // of ATtiny were changed and file was written; 1 if settings if ATtiny did not change and file was written
  bool write2tiny = false;

  struct Config _cfg_new;
  deserialize(_cfg_new, message.c_str());

  // read old config file to compare settings with thw new ones
  File configFile = LittleFS.open("config.json", "r");
  if (configFile) {
    struct Config _cfg_old;
    deserialize(_cfg_old, configFile);
    configFile.close();
    // compare old ATtiny settings with new ones
    if ((_cfg_new.tinyV1vref != _cfg_old.tinyV1vref) ||  
        (_cfg_new.tinyVccmin != _cfg_old.tinyVccmin) ||
        (_cfg_new.tinyVccmax != _cfg_old.tinyVccmax) ||
        (_cfg_new.tinyVcchyst != _cfg_old.tinyVcchyst)) {
      // settings changed - write them to the ATtiny later
      write2tiny = true;
    }
  } else {
    // there were no old settings so we should write them to the Attiny anyway
    write2tiny = true;
  }

  configFile = LittleFS.open("config.json", "w");
  
  if (!configFile) {
    Serial.println("FATAL: Could not open config file for writing");
    configFile.close();
    return -1;
  }
  configFile.print(message);
  configFile.close();

  // read settings back to struct
  readConfig();

  if (write2tiny) {
    return 2;
  }
  return 1;
}

#endif
