# ESP6288 programm for IOT weather station

This is a small programm for my small IOT weather station. The weatherstation consists of the following parts:

- ATtiny85 and LT1763-3.3 for power control
- TP4056 to charge the li-ion battery with solar
- BME280 to measure temperature, humidity and pressure
- ESP8266 to connect to WiFi and send the reedings via MQTT

This scetch is the part of the ESP8266. The ESP will be powered every 15 minutes by the ATtiny and LT1763 to collect all the sensor reading. It will also connect to WiFi and send all the data via MQTT. After all, the ESP will send a signal to the ATtiny and the ATtiny will disable the LET1763 and the ESP as well so that the next iteration can begin.

## Configure your weatherstation

There are two ways to configure your weatherstation. The first and initial variant is to rename the example.config.json file to config.json. You can find this file in the data folder. After you made your configurations, you need to upload it to the file system of the ESP8266. You can do this by running "pio run -t uploadfs". But you can also simply click on "Upload Filesystem Image" when you are using Platform IO with Visual Studio Code.

You can also change your config by using MQTT. Therefor you need to have an allready running MQTT setup. You can simple copy the content of your config.json and send it to the **mqtt_top_topic** config topic by using a MQTT client of your choice.

The maximum file size of your config is limited to 512 bytes. You can test your configuration on arduinojson.org/assistant

#### Options of config.json

Option | Description
--- | --- | ---
**wifi_ssid** | SSID of your WiFi
**wifi_pw** | Passsword of your WiFi
**mqtt_host** | IP address of your MQTT master
**mqtt_port** | Port of your MQTT master
**mqtt_top_topic** | This string will be used for the MQTT topics. *mqtt_top_topic* will be supplemented by `temperature`, `humidity`, `pressure`, `voltage`, `config` and `errors`. Example: `sensors/temperature`
**altitude** | The BME280 will return the actual air pressure which was measured. It is also possible to calculate the related preasure at sea level when you add the altitude of your localtion.
**temp_offset** | If the temperature reading of your BME280 is not accurate enough you can define a correction offset.

## Debuging

You can debug the ESP8266 with the serial interface. Moreover the ESP will write an error.log file to its file system. This file will be send via MQTT when the next connection is establshied. The topic `errors` will be used for that and the log file will be deleted afterwards.
