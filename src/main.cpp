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
#define ATTINY_REG_V1VREF 0x4
#define ATTINY_REG_VCCMIN 0x6
#define ATTINY_REG_VCCMAX 0x8
#define ATTINY_REG_VCCHYST 0xA
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
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <ArduinoJson.h>
#include <AsyncMqttClient.h>

#include "error.h"
#include "config.h"


// I2C for BME280
Adafruit_BME280 bme;

// Create asyncronous MQTT Client
AsyncMqttClient mqttClient;
// This will be true if all MQTT messages were successfully sent
bool mqttSuccessfull = 0;

// Save all MQTT message ids to check if all messages were sent
#define MQTT_LIST_SIZE 7
uint16_t mqttMessageList[MQTT_LIST_SIZE] = {0};

// Save the received size of all OTA update packages
size_t receivedOTASize = 0;
// We should not sleep when we are in an OTA update process
bool delaySleep = false;
// we need to restart after a successfull update
bool espRestart = false;

// If true the ESP will go to deep sleep without sending the off signal to the ATtiny
bool fatal = false;

int valueInArray(uint16_t val, uint16_t arr[], uint16_t arr_size) {
  int16_t i;
  for(i = 0; (uint16_t)i < arr_size; i++)
  {
    if(arr[i] == val)
      return i;
  }
  return -1;
}

bool arrayIsEmpty(uint16_t arr[], uint16_t arr_size) {
  uint16_t i;
  for(i = 0; i < arr_size; i++)
  {
    if(arr[i] != 0)
      return false;
  }
  return true;
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
  float temperature = bme.readTemperature();
  return temperature;
}
  
float getHumidity() {
  float humidity = bme.readHumidity();
  return humidity;
}

float getPressure() {
  float pressure = bme.readPressure()/ 100.0F;
  return pressure;
}

float getVoltage() {
  // Read status from ATtiny
  Wire.beginTransmission(I2C_ADD_ATTINY);
  Wire.write(ATTINY_REG_STAT);  // set the first register to read status from Attiny
  I2CEndTransmission();

  // Request 1 byte from ATtiny (status, sleep, low byte of voltage, high byte of voltage)
  I2CRequest(I2C_ADD_ATTINY, 1);
  uint8_t status = Wire.read();
  if (bitRead(status, 0) == 0) {
    // The ATtiny did not read any voltage?!
    errorLog("Error: ATtiny did not read any voltage");
  }

  float voltage = 0.0;
  for (uint8_t i = 1; i <= 2; i++) {
    // Read battery voltage from ATtiny
    Wire.beginTransmission(I2C_ADD_ATTINY);
    // set the third register to read battery voltage from Attiny
    Wire.write(ATTINY_REG_VCC);
    I2CEndTransmission();
    // Request 2 bytes from ATtiny (low byte of voltage, high byte of voltage)
    I2CRequest(I2C_ADD_ATTINY, 2);
    uint8_t voltage_low_byte = Wire.read();
    uint8_t voltage_high_byte = Wire.read();
    // uint16_t voltage = makeWord(voltage_high_byte, voltage_low_byte);
    voltage = (float)makeWord(voltage_high_byte, voltage_low_byte) / 1000.0;  // from mV to V
    // check if voltage is between its limits
    if ((voltage > 1.8) && (voltage < 5.5)) {
      break;
    }
  }
  return voltage;
}

bool updateSketch(char* payload, size_t len, size_t index, size_t total) {
  // This will install the update file received in several peckages (payload)
  // This function will return true when the last peckage is written

  // check if we received more data than expected
  receivedOTASize += len;
  if (receivedOTASize > total) {
    errorLog("ERROR: OTA update is bigger than expected!");
  }

  // Start update process if it is not running
  if (!Update.isRunning()) {
    Update.runAsync(true);
    if (!Update.begin(total)) {
      errorLog("ERROR: Could not start OTA update");
      return false;
    }
    Serial.println("Start OTA update");
  }
  // write the received payload
  //Serial.println("Writing OTA update");
  if (Update.write((uint8_t*)payload, len) != len) {
    errorLog("ERROR: Could not write OTA update");
    return false;
  }
  // end update process when all packeges were received
  if (receivedOTASize >= total) {
    if (!Update.end()) {
      errorLog("ERROR: Could not end OTA update");
    } else {
      Serial.println("End OTA update");
    }
    return true;
  }
  return false;
}

void writeAttinySettings() {
  Wire.beginTransmission(I2C_ADD_ATTINY);
  // set the register to write settings
  Wire.write(ATTINY_REG_V1VREF);
  // write setttings to ATtiny
  Serial.print("V1Ref: ");
  Serial.println(cfg.tinyV1vref);
  Serial.print("VCCmin: ");
  Serial.println(cfg.tinyVccmin);
  Serial.print("VCCmax: ");
  Serial.println(cfg.tinyVccmax);
  Serial.print("VCChyst: ");
  Serial.println(cfg.tinyVcchyst);
  Wire.write(lowByte(cfg.tinyV1vref));
  Wire.write(highByte(cfg.tinyV1vref));
  Wire.write(lowByte(cfg.tinyVccmin));
  Wire.write(highByte(cfg.tinyVccmin));
  Wire.write(lowByte(cfg.tinyVccmax));
  Wire.write(highByte(cfg.tinyVccmax));
  Wire.write(lowByte(cfg.tinyVcchyst));
  Wire.write(highByte(cfg.tinyVcchyst));
  I2CEndTransmission();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");

  String tempTopic = cfg.mqttTopTopic + String("temperature");
  String humiTopic = cfg.mqttTopTopic + String("humidity");
  String presTopic = cfg.mqttTopTopic + String("pressure");
  String voltTopic = cfg.mqttTopTopic + String("voltage");
  String otaTopic = cfg.mqttTopTopic + String("ota");
  String confTopic = cfg.mqttTopTopic + String("config");
  String errorsTopic = cfg.mqttTopTopic + String("errors");

  // check for OTA update
  if (!mqttClient.subscribe(otaTopic.c_str(), 1)) {
    Serial.println("Warning: Could not subscribe to OTA topic");
  }

  // subscribe to config. This can be used to change the config.json
  if (!mqttClient.subscribe(confTopic.c_str(), 1)) {
    Serial.println("Warning: Could not subscribe to config topic");
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
  pressure = bme.seaLevelForAltitude(cfg.altitude, pressure);
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

  int16_t i = valueInArray(packetId, mqttMessageList, MQTT_LIST_SIZE);
  if (i == -1) {
    // packetId was not in mqttMessageList
    return;
  }
  if (i == 6) {  // TODO: Zahlen ändern in enum?
    // Restart ESP because we made an sketch update
    espRestart = true;
    delaySleep = false;
  }
  mqttMessageList[i] = 0;
  if (arrayIsEmpty(mqttMessageList, MQTT_LIST_SIZE) && !delaySleep) {
    // now we can disconnect
    mqttSuccessfull = true;
    Serial.println("All messages has been published - disconnect now");
    mqttClient.disconnect();
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Serial.println("MQTT message received");
  String otaTopic = cfg.mqttTopTopic + String("ota");
  String confTopic = cfg.mqttTopTopic + String("config");

  if (total <= 0) {
    return;
  }
  if (String(topic) == confTopic) {

    // Write the new config to LittleFS
    String fixedStr = ((String)payload).substring(0,len);
    if (writeConfig(fixedStr) == 2) {
      // settings of Attiny were changed, write them to Attiny
      Serial.println("Write new Attiny settings");
      writeAttinySettings();
    }
    Serial.println("New config has been written");

    // overwrite MQTT topic, so that we will not write it a second time to FS
    mqttMessageList[5] = mqttClient.publish(confTopic.c_str(), 1, true, NULL);
  } else if (String(topic) == otaTopic) {
    // we should not sleep until we received the entire update
    delaySleep = true;
    // write ota update and restart when finished
    if (updateSketch(payload, len, index, total)) {
      Serial.println("OTA update was successful");
      // overwrite MQTT topic, so that we will not write it a second
      mqttMessageList[6] = mqttClient.publish(otaTopic.c_str(), 1, true, NULL);
    }
  } else {
    Serial.println("Warning: Received a topic that we do not have a subscription to");
  }
}

void sleepNow() {
  if (delaySleep) {
    return;
  } 
  if (espRestart) {
    // Restart ESP after OTA update
    Serial.println("Restart ESP");
    ESP.restart();
    delay(100);
    return;
  }
  if (fatal) {
    // This is an emergancy because of disturbed I2C! The ESP will go to sleep and will not
    // send anything to the ATtiny so the 3.3V will remain enabled!
    errorLog("Emergancy shut down! ESP8266 will be send to deep sleep with 3.3V enabled!");
    delay(3000);
    ESP.deepSleep(ESP_EMERGANCY_SLEEP);  // 1e6 == 1 sec
    delay(100);
    return;
  }
  Serial.println("Good bye!");
  // send off signal to ATtiny
  Wire.beginTransmission(I2C_ADD_ATTINY);
  // select the sleep register
  Wire.write(ATTINY_REG_SLEEP);
  // how long the system should be off
  Wire.write(ATTINY_SLEEPTIME);
  I2CEndTransmission();  // Beware! This will cause a recursion but should end up in ESP.deepSleep
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
  delay(1000);
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
  if (!bme.begin(I2C_ADD_BME280)) {
    errorLog("FATAL: Could not find a valid BME280 sensor, check wiring!");
    // turn off
    sleepNow();
  }
  // For more details on the following scenarious, see chapter
  // 3.5 "Recommended modes of operation" in the datasheet
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
  bme.setTemperatureCompensation(cfg.temp_offset);
  // take a measurement
  if (!bme.takeForcedMeasurement()) {
    errorLog("FATAL: Could not take measurements with BME280");
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
    Serial.println("");

    // Check status of ATtiny
    Wire.beginTransmission(I2C_ADD_ATTINY);
    Wire.write(ATTINY_REG_STAT);  // set the first register to read status from Attiny
    I2CEndTransmission();
    // Request 1 byte from ATtiny (status)
    I2CRequest(I2C_ADD_ATTINY, 1);
    uint8_t status = Wire.read();
    Serial.print("ATtiny status: ");
    Serial.println(status);
    if (bitRead(status, 1) == 0) {
      // The ATtiny EEPROM needs to be conditioned now
      Serial.println("ATtiny is not conditioned. Writing settings to ATtiny");
      writeAttinySettings();
    }

    // Connect to MQTT
    // Print local IP address
    Serial.print("My IP is ");
    Serial.println(WiFi.localIP());
    
    // Connect MQTT
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
    if (cfg.mqttUser != "") {
      mqttClient.setCredentials(cfg.mqttUser.c_str(), cfg.mqttPassword.c_str());
    }
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
