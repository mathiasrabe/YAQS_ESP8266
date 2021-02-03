/*
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// I2C addresses
#define I2C_ADD_ATTINY 0x4
#define I2C_ADD_BME280 0x76

// I2C number of I2C retry attemts if transmission failes
#define I2C_RETRY 3

// I2C ATtiny register addresses
#define ATTINY_REG_STAT 0x0
#define ATTINY_REG_SLEEP 0x1
#define ATTINY_REG_VCC 0x2
// How long should the device sleep between readings
#define ATTINY_SLEEPTIME 0x70  // 15min*60sec / 8sec = 112,5 = 0x70
// If there is no correct communication to the ATtiny the ESP8266 will go into deep sleep with 3.3V enabled.
// How long should this sleep state last
#define ESP_EMERGANCY_SLEEP 900e6  // 1e6 == 1sec

// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>


// I2C for BME280
BME280 bme;

// struct for settings like network credentials
struct Config {
  String wifiSSID = "";
  String wifiPassword = "";
  String mqttHost = "";
  uint16_t mqttPort = 1883;
  String mqttTopTopic = "";
  float altitude = 0.0;  // in m
  float temp_offset = 0.0;  // in °C
};
struct Config cfg;

// Create asyncronous MQTT Client
AsyncMqttClient mqttClient;
// This will be true if all MQTT messages were successfully sent
bool mqttSuccessfull = 0;

// Save all MQTT message ids to check if all messages were sent
uint16_t mqttMessageList[6] = {0};

// If true the ESP will go to deep sleep without sending the off signal to the ATtiny
bool fatal = false;

int valueInArray(uint16_t val, uint16_t arr[]) {
  int16_t i;
  for(i = 0; (uint16_t)i < sizeof(arr); i++)
  {
    if(arr[i] == val)
      return i;
  }
  return -1;
}

bool arrayIsEmpty(uint16_t arr[]) {
  uint16_t i;
  for(i = 0; i < sizeof(arr); i++)
  {
    if(arr[i] != 0)
      return false;
  }
  return true;
}

void errorLog(const char* message) {
  // Write message to FS and print it to Serial
  Serial.println(message);

  File errorFile = LittleFS.open("error.log", "a");
  
  if (!errorFile) {
    Serial.println("FATAL: Could not open error file for writing");
    errorFile.close();
    return;
  }
  errorFile.println(message);
  errorFile.close();
}

String readAndRemoveOldErrors() {
  // Will return all lines in error log and delete the log afterwards
  const char* fileName = "error.log";
  File errorFile = LittleFS.open(fileName, "r");
  
  if (!errorFile) {
    // Could not open file, probably there was no error
    errorFile.close();
    return String("");
  }

  String messages = errorFile.readString();

  errorFile.close();
  if (!LittleFS.remove(fileName)) {
    Serial.print("Error: Could not remove old error log");
  }
  return messages;
}

void I2CEndTransmission(uint8_t retryCounter = I2C_RETRY) {
  uint8_t statusI2C = 0;

  while ((statusI2C = Wire.endTransmission()) != 0) {
    if (retryCounter == 0) {
      // I2C is disturbed. This is an emergancy routine!
      errorLog("FATAL: Could not write to ATtiny...");
      fatal = true;  // this will cause the ESP to go to deep sleep without sending something to the ATtiny
      mqttClient.disconnect();  // this will disconnect von MQTT, WIFI and we will sleep
    }
    retryCounter--;
    Serial.println("Warning: Could not write to ATtiny because of:");
    switch(statusI2C) {
      case 1: Serial.println(" Data too long to fit in transmit buffer"); break;
      case 2: Serial.println(" Received NACK on transmit of address"); break;
      case 3: Serial.println(" Received NACK on transmit of data"); break;
      case 4: Serial.println(" Other error"); break;
    }
    Serial.println("Retrying ...");
  }
}

void I2CRequest(uint8_t address, uint8_t bytes) {
  uint8_t retryCounter = I2C_RETRY;

  while (Wire.requestFrom(address, bytes) != bytes) {
    if (retryCounter == 0) {
      // I2C is disturbed. The is an emergancy routine!
      errorLog("FATAL: Could not read from  ATtiny... Going to sleep");
      fatal = true;  // this will cause the ESP to go to deep sleep without sending something to the ATtiny
      mqttClient.disconnect();  // this will disconnect von MQTT, WIFI and we will sleep
    }
    retryCounter--;
    Serial.println("Warning: Could not read from ATtiny. Retrying ...");
  }
}

float getTemperature() {
  float temperature = bme.readTempC();
  return temperature;
}
  
float getHumidity() {
  float humidity = bme.readFloatHumidity();
  return humidity;
}

float getPressure() {
  float pressure = bme.readFloatPressure()/ 100.0F;
  return pressure;
}

int8_t writeConfig(String message) {
  File configFile = LittleFS.open("config.json", "w");
  
  if (!configFile) {
    Serial.println("FATAL: Could not open config file for writing");
    configFile.close();
    return -1;
  }
  configFile.print(message);
  configFile.close();
  return 1;
}

int8_t readConfig() {
  File configFile = LittleFS.open("config.json", "r");
  const uint16_t fileSize = 512;

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

  // Allocate the memory pool on the stack.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<fileSize> jsonDoc;

  deserializeJson(jsonDoc, configFile);

  // read keys and write them to the config struct
  cfg.wifiSSID = jsonDoc["wifi_ssid"].as<String>();
  cfg.wifiPassword = jsonDoc["wifi_pw"].as<String>();
  cfg.mqttHost = jsonDoc["mqtt_host"].as<String>();
  cfg.mqttPort = jsonDoc["mqtt_port"];
  cfg.mqttTopTopic = jsonDoc["mqtt_top_topic"].as<String>();
  cfg.altitude = jsonDoc["altitude"];
  cfg.temp_offset = jsonDoc["temp_offset"];

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

float getVoltage() {
  // Read status from ATtiny
  Wire.beginTransmission(I2C_ADD_ATTINY);
  Wire.write(ATTINY_REG_STAT);  // set the first register to read status from Attiny
  I2CEndTransmission();

  // Request 1 byte from ATtiny (status, sleep, low byte of voltage, high byte of voltage)
  I2CRequest(I2C_ADD_ATTINY, 1);
  uint8_t status = Wire.read();
  if (status == 0) {
    // The ATtiny did not read any voltage?!
    errorLog("Error: Status of ATtiny was 0");
  }

  // Read battery voltage from ATtiny
  Wire.beginTransmission(I2C_ADD_ATTINY);
  // set the third register to read battery voltage from Attiny
  Wire.write(ATTINY_REG_VCC);
  I2CEndTransmission();
  // Request 2 bytes from ATtiny (low byte of voltage, high byte of voltage)
  I2CRequest(I2C_ADD_ATTINY, 2);
  uint8_t voltage_low_byte = Wire.read();
  uint8_t voltage_high_byte = Wire.read();
  uint16_t voltage = makeWord(voltage_high_byte, voltage_low_byte);
  return (float)(voltage / 1000.0);  // from mV to V
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");

  String tempTopic = cfg.mqttTopTopic + String("temperature");
  String humiTopic = cfg.mqttTopTopic + String("humidity");
  String presTopic = cfg.mqttTopTopic + String("pressure");
  String voltTopic = cfg.mqttTopTopic + String("voltage");
  String confTopic = cfg.mqttTopTopic + String("config");
  String errorsTopic = cfg.mqttTopTopic + String("errors");

  // subscribe to config. This can be used to change the config.json
  if (!mqttClient.subscribe(confTopic.c_str(), 1)) {
    Serial.println("Warning: Could not subscribe to config topic");
  }

  // check if BME is still measuring
  if (bme.isMeasuring()) {
    errorLog("Error: BME 280 needs too long to measure");
  }

  // read the temperature and send it to the MQTT broker
  float temp = getTemperature();
  mqttMessageList[0] = mqttClient.publish(tempTopic.c_str(), 1, true, String(temp).c_str());

  // read the humidity and send it to the MQTT broker
  float humidity = getHumidity();
  mqttMessageList[1] = mqttClient.publish(humiTopic.c_str(), 1, true, String(humidity).c_str());

  // read the pressure and send it to the MQTT broker
  float pressure = getPressure();
  // calculate the pressure for sea level
  pressure = pressure / pow(1.0 - (cfg.altitude / 44330.0), 5.255);
  mqttMessageList[2] = mqttClient.publish(presTopic.c_str(), 1, true, String(pressure).c_str());

  // read the battery voltage and send it to the MQTT broker
  float voltage = getVoltage();
  mqttMessageList[3] = mqttClient.publish(voltTopic.c_str(), 1, true, String(voltage).c_str());

  // send and delete error.log file
  String errors = readAndRemoveOldErrors();
  if (errors != String("")) {
    mqttMessageList[4] = mqttClient.publish(errorsTopic.c_str(), 1, true, errors.c_str());
  }
}

void onMqttPublish(uint16_t packetId) {
  // if all values in mqttMessageList are 0 then no mqtt publish process remains and we can shut down

  int16_t i = valueInArray(packetId, mqttMessageList);
  if (i == -1) {
    // packetId was not in mqttMessageList
    return;
  }
  mqttMessageList[i] = 0;
  if (arrayIsEmpty(mqttMessageList)) {
    // now we can disconnect
    mqttSuccessfull = true;
    Serial.println("All messages has been published - disconnect now");
    mqttClient.disconnect();
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("MQTT message received");
  String confTopic = cfg.mqttTopTopic + String("config");
  if (String(topic) != confTopic) {
    Serial.println("Warning: Received a topic that we do not have a subscription to");
    return;
  }

  // Write the new config to LittleFS
  String fixedStr = ((String)payload).substring(0,len);
  writeConfig(fixedStr);
  Serial.println("New config has been written");

  // overwrite MQTT topic, so that we will not write it a second time to FS
  mqttMessageList[5] = mqttClient.publish(confTopic.c_str(), 1, true, NULL);
}

void sleepNow() {
  if (fatal) {
    // This is an emergancy because of disturbed I2C! The ESP will go to sleep and will not
    // send anything to the ATtiny so the 3.3V will remain enabled!
    errorLog("Emergancy shut down! ESP8266 will be send to deep sleep with 3.3V enabled!");
    ESP.deepSleep(ESP_EMERGANCY_SLEEP);  // 1e6 == 1 sec
    delay(100);
  } else {
    Serial.println("Good bye!");
    // send off signal to ATtiny
    Wire.beginTransmission(I2C_ADD_ATTINY);
    // select the sleep register
    Wire.write(ATTINY_REG_SLEEP);
    // how long the system should be off
    Wire.write(ATTINY_SLEEPTIME);
    I2CEndTransmission();  // Beware! This will cause a recursion but should end up in ESP.deepSleep
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  // check for disconnect reason
  if (mqttSuccessfull) {
    Serial.println("MQTT disconnected");
  } else {
    errorLog("Error: MQTT disconnected unexpectedly because of this: ");
    switch(reason) {
      case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
        errorLog("TCP disconnected!"); break;
      case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
        errorLog("MQTT unacceptable protocol version!"); break;
      case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
        errorLog("MQTT identifier rejected!"); break;
      case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
        errorLog("MQTT server unavailable!"); break;
      case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
        errorLog("MQTT malformed credentials!"); break;
      case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
        errorLog("MQTT not authorized!"); break;
      case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:
        errorLog("ESP8266 not enough space!"); break;
      case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
        errorLog("TLS bad fingerprint!"); break;
    }
  }

  // disconnect WIFI
  WiFi.disconnect();
  Serial.println("WiFi disconnected");
  // turn off
  sleepNow();
}
 
void setup(){
  // Serial port for debugging purposes
  Serial.begin(9600);
  Serial.println("");

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("FATAL: An Error has occurred while mounting LittleFS");
    // turn off
    sleepNow();
  }

  // Configure I2C for ATtiny
  Wire.begin();
  Wire.setClockStretchLimit(1500);

  // Initialize the BME280 sensor
  // For more details on the following scenarious, see chapter
  // 3.5 "Recommended modes of operation" in the datasheet
  bme.settings.I2CAddress = I2C_ADD_BME280;
  bme.settings.runMode = MODE_FORCED;
  bme.settings.filter = 0;
  bme.settings.tempOverSample = 1;
  bme.settings.pressOverSample = 1;
  bme.settings.humidOverSample = 1;
  bme.settings.tempCorrection = cfg.temp_offset;
  if (!bme.beginI2C()) {
    errorLog("FATAL: Could not find a valid BME280 sensor, check wiring!");
    // turn off
    sleepNow();
  }

  // Read config
  if (readConfig()) {  // config was successfully loaded
    // Connect to Wi-Fi
    WiFi.begin(cfg.wifiSSID, cfg.wifiPassword);
    Serial.print("Connecting to ");
    Serial.print(cfg.wifiSSID);
    uint8_t wifiConnectCounter = 20;
    while (WiFi.status() != WL_CONNECTED) {
      if (wifiConnectCounter == 0) {
        // We could not connect to wifi. Let's sleep now
        Serial.println("");
        errorLog("Error: Could not connect WIFI");
        sleepNow();
        break;
      }
      wifiConnectCounter--;
      delay(500);
      Serial.print(".");
    }

    // Connect to MQTT
    // Print local IP address
    Serial.println("");
    Serial.print("My IP is ");
    Serial.println(WiFi.localIP());
    
    // Connect MQTT
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
    Serial.print("Connect to MQTT Broker ");
    Serial.print(cfg.mqttHost);
    Serial.print(":");
    Serial.println(cfg.mqttPort);
    mqttClient.connect();
  } else {
    sleepNow();
  }
}
 
void loop(){
  // nothing to do here
}
