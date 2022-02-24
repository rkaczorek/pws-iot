/*
  Name: Personal Weather Station IoT
  Version: 1.0
  By: Radek Kaczorek, July 2021
  License: GNU General Public License v3.0

  This code reads various environmental sensors and publishes values to MQTT server via wireless connection.

  Supported MCUs:
  - Arduino Nano 33 IoT
  - Arduino MKR WiFi 1000
  - Arduino MKR WiFi 1010
  - Arduino Uno Wifi Rev2
  - ESP8266

  Supported GPIO connected (A0, D10, D11) sensors:
  - Weather Meters (wind direction, wind speed, rainfall) (https://www.sparkfun.com/products/8942)

  Supported I2C sensors:
  - BME280 (temperature, humidity, pressure)
  - MLX90614 (ambient temperature, object temperature)
  - VEML7700 (light)

  Set these variables in PWS_IoT.h:
    SECRET_SSID "wifi_name"
    SECRET_PASS "wifi_password"
    MQTT_HOST "mqtt_address"
    MQTT_PORT 1883
    MQTT_USER "mqtt_user"
    MQTT_PASS "mqtt_password"
    MQTT_ROOT_TOPIC "environment"
    MQTT_DEVICE_NAME "pws"
    MQTT_DEVICE_NAME_TO_HOSTNAME true
    DEBUG_MQTT_MSGS true
    DEBUG_SENSORS false
    WIND_DIR_CORRECTION 60
    WIND_AVERAGING_TIME 10000
    CLOUDS_AVERAGING_TIME 5000
*/

#define VERSION 1.0

#include "PWS_IoT.h"

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h> // https://www.arduino.cc/en/Reference/WiFiNINA // https://github.com/arduino-libraries/WiFiNINA
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h> // https://www.arduino.cc/en/Reference/WiFi101
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h> // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#endif

#include <vector>
#include <numeric>
#include <Wire.h> // https://github.com/esp8266/Arduino/tree/master/libraries/Wire
#include <ArduinoMqttClient.h> // https://github.com/arduino-libraries/ArduinoMqttClient
#include <ArduinoOTA.h> // https://github.com/jandrassy/ArduinoOTA
#include <ArduinoJson.h>  // https://github.com/bblanchon/ArduinoJson
#include <Adafruit_BME280.h> // https://github.com/adafruit/Adafruit_BME280_Library
#include <SparkFunMLX90614.h> // https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library
#include "DFRobot_VEML7700.h" // https://github.com/DFRobot/DFRobot_VEML7700
#include <Cardinal.h> // https://github.com/DaAwesomeP/arduino-cardinal

#define SEALEVELPRESSURE_HPA  (1013.25) // used for altitude calculation based on pressure
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0)) // used for arithmetic sign of a value (needed by cloud detection algorithm)

// hardware pins definitions
const byte WDIR = A0;
const byte WSPEED = 10;
const byte RAIN = 11;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile long windClicks = 0;
volatile long lastRainIRQ = 0;
volatile long rainClicks = 0;

// last checks
unsigned long lastWindMillis = 0;
unsigned long lastCloudsMillis = 0;
unsigned long lastCheck = 0;

// statistical vars
std::vector<float> windSpeed;
std::vector<unsigned int> windDir;
float windGustSpeed = 0;
std::vector<unsigned int> windGustDir;
std::vector<float> irAmbient;
std::vector<float> irSky;

// polling vars
unsigned long timestamp = 0; // set tickmark for auto polling
unsigned int polling = 0; // auto polling time

// wifi
const char ssid[] = SECRET_SSID; // your network SSID (name)
const char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)

// mqtt
const char* mqtt_host = MQTT_HOST;
int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_pass = MQTT_PASS;

// mqtt topics
char mqtt_root_topic[32] = MQTT_ROOT_TOPIC; // eg. environment
char mqtt_device_name[32] = MQTT_DEVICE_NAME; // eg. mysensor
char mqtt_setroot_topic[36]; // eg. environment/set
char mqtt_setname_topic[70]; // eg. environment/mysensor/set
char mqtt_connected_topic[76]; // eg. environment/mysensor/connected
char mqtt_poll_topic[71]; // eg. environment/mysensor/poll
char mqtt_restart_topic[74]; // eg. environment/mysensor/restart
char mqtt_reset_topic[72]; // eg. environment/mysensor/reset

byte mac[6]; // 6 byte array to hold the MAC address
char hostname[32]; // an array to hold wifi hostname

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// sensors are not available by default and activated only if detected
int bme_sensor = 0;
int mlx_sensor = 0;
//int veml_sensor = 0;

Adafruit_BME280 bme; // BME280 sensor - temperature, humidity, pressure
IRTherm mlx; // MLX90614 sensor - infrared termometer
DFRobot_VEML7700 als; // VEML7700 - light

Cardinal cardinal;

void rainfallIRQ()
{
  if (millis() - lastRainIRQ > 10) // ignore switch-bounce glitches less than 10ms
  {
    lastRainIRQ = millis();
    rainClicks++;
  }
}

void windspeedIRQ()
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms
  {
    lastWindIRQ = millis();
    windClicks++;
  }
}

void setup() {
  pinMode(WDIR, INPUT); // input from winddir sensor
  pinMode(WSPEED, INPUT_PULLUP); // input from windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from rain gauge sensor

  // attach external interrupt pins to IRQ functions
  attachInterrupt(digitalPinToInterrupt(RAIN), rainfallIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), windspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  Serial.begin(9600);
  delay(3000); // wait for serial port to connect. Needed for native USB port only

  Serial.print("Initializing Personal Weather Station v");
  Serial.println(VERSION);
  Serial.println("");

  Serial.println("Initializing sensors");

  // Init BME280
  if (bme.begin(0x77)) { // Adarfruit module 0x77
    Serial.println("  BME280 sensor detected");
    bme_sensor = 1;

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  } else if (bme.begin(0x76)) { // standalone sensor 0x76
    Serial.println("  BME280 sensor detected");
    bme_sensor = 1;
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );    
  } else {
    Serial.println("  BME280 sensor not detected");
  }

  // Init MLX
  if (mlx.begin()) {
    Serial.println("  MLX90614 sensor detected");
    mlx.setUnit(TEMP_C); // Set the library's units to Celsius
    mlx_sensor = 1;
  } else {
    Serial.println("  MLX90614 sensor not detected");
  }

  // Init VEML
  als.begin();

  Serial.println("");

  // Connect to WiFi

  // prepare hostname
  char id0[1];
  char id1[1];
  WiFi.macAddress(mac);
  sprintf(id0, "%x", mac[0]);
  sprintf(id1, "%x", mac[1]);
  strcpy(hostname, "pws-");
  strcat(hostname, (const char*)id1);
  strcat(hostname, (const char*)id0);

  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);

  WiFi.setHostname(hostname);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("");
  Serial.println("Successfully connected to the network");
  Serial.print("  Hostname: ");
  Serial.println(hostname);
  Serial.print("  IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("  Subnet mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("  Default gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.println();

  //start the WiFi OTA
  ArduinoOTA.begin(WiFi.localIP(), "Arduino", "1234", InternalStorage);

  // Prepare MQTT variables

  // set MQTT device name to hostname
  if (MQTT_DEVICE_NAME_TO_HOSTNAME) {
    strcpy(mqtt_device_name, hostname);
  }

  // convert MQTT device name to lower case
  for (int i = 0; i < strlen(mqtt_device_name); i++)
    mqtt_device_name[i] = tolower(mqtt_device_name[i]);

  // construct MQTT topic for setting will
  strcpy(mqtt_connected_topic, mqtt_root_topic);
  strcat(mqtt_connected_topic, "/");
  strcat(mqtt_connected_topic, mqtt_device_name);
  strcat(mqtt_connected_topic, "/connected");

  // construct MQTT topic for setting root name
  strcpy(mqtt_setroot_topic, mqtt_root_topic);
  strcat(mqtt_setroot_topic, "/set");

  // construct MQTT topic for setting device name
  strcpy(mqtt_setname_topic, mqtt_root_topic);
  strcat(mqtt_setname_topic, "/");
  strcat(mqtt_setname_topic, mqtt_device_name);
  strcat(mqtt_setname_topic, "/set");

  // construct MQTT topic for sensors reading 
  strcpy(mqtt_poll_topic, mqtt_root_topic);
  strcat(mqtt_poll_topic, "/");
  strcat(mqtt_poll_topic, mqtt_device_name);
  strcat(mqtt_poll_topic, "/poll");

  // construct MQTT topic for restarting device
  strcpy(mqtt_restart_topic, mqtt_root_topic);
  strcat(mqtt_restart_topic, "/");
  strcat(mqtt_restart_topic, mqtt_device_name);
  strcat(mqtt_restart_topic, "/restart");

  // construct MQTT topic for resetting device to defaults
  strcpy(mqtt_reset_topic, mqtt_root_topic);
  strcat(mqtt_reset_topic, "/");
  strcat(mqtt_reset_topic, mqtt_device_name);
  strcat(mqtt_reset_topic, "/reset");

  // Connect to MQTT server

  // set MQTT last will
  mqttClient.beginWill(mqtt_connected_topic, true, 0);
  mqttClient.print(0);
  mqttClient.endWill();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.print(mqtt_host);
  Serial.print(":");
  Serial.println(mqtt_port);

  mqttClient.setId(mqtt_device_name);
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);
  //mqttClient.setCleanSession(true); // http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Keep_Alive
  //mqttClient.setKeepAliveInterval(60*1000L);
  //mqttClient.setConnectionTimeout(60*1000L);

  while (!mqttConnect()) {
    Serial.print(".");
    delay(5000);
  }
  
  // set callback
  mqttClient.onMessage(mqttCallback);

  // Subscribe to control topics
  if (mqttSubscribe()) {
    Serial.println("======================================================================================================================");
    Serial.print("Publish '0' to ");
    Serial.print(mqtt_poll_topic);
    Serial.println(" to read sensors on request or publish a number to set auto polling in seconds");
    Serial.print("Publish 'yes' to ");
    Serial.print(mqtt_restart_topic);
    Serial.println(" to restart the device");
    Serial.println("======================================================================================================================");
    Serial.println("");
  }

  // Init msg
  Serial.println("Personal Weather Station ready");
  Serial.println("");
}

void loop() {
  // auto reconnect wifi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Network connection lost! Reconnecting...");
    //mqttClient.stop();
    //WiFi.disconnect();
    //WiFi.end();
    //WiFi.begin(ssid, pass);
    Serial.println("Network connection lost! Restarting...");
    delay(500);
    NVIC_SystemReset();    
  }

  // auto reconnect MQTT
  if  (!mqttClient.connected()) {
    if (mqttConnect()) {
      mqttSubscribe();
    }
  }

  // check for WiFi OTA updates
  ArduinoOTA.poll();
  
  // send MQTT keep alives
  mqttClient.poll();

  // collect wind sensors data every loop
  if (millis() - lastWindMillis > WIND_AVERAGING_TIME) {

    // get wind speed
    float windspeed = (float) windClicks / (WIND_AVERAGING_TIME / 1000); // clicks per second
    windClicks = 0;
    windspeed *= 2.4011412; // 1 click per second equals 1.492MPH = 2.4011412 km/h
    windSpeed.push_back(windspeed);

    // get wind direction
    int winddir = get_wind_direction();
    if (winddir != -1) {
      // calculate corrected wind direction
      winddir += WIND_DIR_CORRECTION;
      if(winddir >= 360) winddir -= 360;
      if(winddir < 0) winddir += 360;
      windDir.push_back(winddir);
    }

    // get wind gust and wind gust direction
    if (windspeed > windGustSpeed) {
      windGustSpeed = windspeed;
      if (winddir != -1) {
        windGustDir.push_back(winddir);
      }
    }
    lastWindMillis = millis();
  }

  // collect clouds sensors data every loop
  if (millis() - lastCloudsMillis > CLOUDS_AVERAGING_TIME) {

    // get ambient temperature and sky temperature
    if (mlx_sensor) {
      mlx.read(); // the firt read is not reliable; according to datasheet On-chip IIR filter is skipped for the first measurement (p.19)
      mlx.read();
      float temperature_ambient = ((int) (mlx.ambient() * 100)) / 100.0; // celcius
      float temperature_sky = ((int) (mlx.object() * 100)) / 100.0; // celcius

      if (temperature_ambient > -273.0 && temperature_sky > -273.0) { // sanity check
        irAmbient.push_back(temperature_ambient);
        irSky.push_back(temperature_sky);
      }
    }
    lastCloudsMillis = millis();
  }

  autoPolling();
}

void autoPolling() {
  if ( polling == 0 || millis() - timestamp < polling * 1000)
    return;
  getSensors();
  timestamp = millis();
}

bool mqttConnect() {
  if (mqttClient.connect(mqtt_host, mqtt_port)) {
    // successfully connected to MQTT server
    Serial.println();
    Serial.println("Successfully connected to the MQTT broker");
    Serial.print("  Client ID: ");
    Serial.println(mqtt_device_name);
    Serial.print("  Topic: ");
    Serial.print(mqtt_root_topic);
    Serial.print("/");
    Serial.println(mqtt_device_name);
    Serial.println();

    // publish connection status
    //mqttPublish("connected", "1");
    mqttClient.beginMessage(mqtt_connected_topic, true, 0, false);
    mqttClient.print(1);
    mqttClient.endMessage();

    // publish device ip once when connected to MQTT
    char localip[16];
    IPAddress ip = WiFi.localIP();
    sprintf(localip, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    mqttPublish("ip", localip);
    return true;
  } else {
    Serial.println("");
    Serial.print("MQTT connection failed! ");
    switch (mqttClient.connectError()) {
      case -2:
        Serial.println("Connection refused.");
        break;
      case -1:
        Serial.println("Connection timeout.");
        break;
      case 1:
        Serial.println("Unacceptable protocol version.");
        break;
      case 2:
        Serial.println("Identifier rejected.");
        break;
      case 3:
        Serial.println("Server unavailable.");
        break;
      case 4:
        Serial.println("Bad username or password.");
        break;
      case 5:
        Serial.println("Not authorized.");
        break;
      default:
        Serial.println("Connection refused.");
    }
    return false;
  }
}

bool mqttSubscribe() {
  if (mqttClient.subscribe(mqtt_poll_topic) && mqttClient.subscribe(mqtt_restart_topic)) {
    // successfully subscribed to topics
    Serial.println("");
    Serial.println("Successfully subscribed to control topics");

    // publish connection status
    //mqttPublish("connected", "2");
    mqttClient.beginMessage(mqtt_connected_topic, true, 0, false);
    mqttClient.print(2);
    mqttClient.endMessage();

    return true;
  } else {
    Serial.println("");
    Serial.println("Error subscribing to control topics!");
    return false;
  }
}

void mqttPublish(char* topic, float val) {
  char mqtt_topic[512] = "";
  char msg[16];
  strcpy(mqtt_topic, mqtt_root_topic);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, mqtt_device_name);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, topic);
  sprintf(msg, "%0.2f", val);

  mqttClient.beginMessage(mqtt_topic);
  mqttClient.print(msg);
  mqttClient.endMessage();

  if (DEBUG_MQTT_MSGS) {
    Serial.println("");
    Serial.println("MQTT message published");
    Serial.print("  topic: ");
    Serial.print(mqtt_topic);
    Serial.println("");
    Serial.print("  message: ");
    Serial.println(msg);
    Serial.println("");
  }  
}

void mqttPublish(char* topic, const char* msg) {
  char mqtt_topic[512] = "";
  strcpy(mqtt_topic, mqtt_root_topic);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, mqtt_device_name);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, topic);

  size_t payloadSize = strlen(msg);
  mqttClient.beginMessage(mqtt_topic, payloadSize, false, 0, false);
  mqttClient.print(msg);
  mqttClient.endMessage();

  if (DEBUG_MQTT_MSGS) {
    Serial.println("");
    Serial.println("MQTT message published");
    Serial.print("  topic: ");
    Serial.print(mqtt_topic);
    Serial.println("");
    Serial.print("  message: ");
    Serial.println(msg);
    Serial.println("");
  }
}

void mqttCallback(int length) {
  char payload[length];
  String topic = mqttClient.messageTopic();

  // get message
  for (int i = 0; i < length; i++) {
    payload[i] = (char)mqttClient.read();
    payload[i+1] = '\0';
  }

  if (DEBUG_MQTT_MSGS) {
    Serial.println("");
    Serial.print("Message arrived at ");
    Serial.print(topic);
    Serial.print(": ");  
    Serial.println(payload);
    Serial.println();
  }

  if (topic == mqtt_poll_topic) { // handle change of polling time
    char new_polling[length];
    strncpy(new_polling, (char*)payload, length);
    unsigned int new_polling_val = atoi(new_polling);
    if(new_polling_val != polling && new_polling_val < 86400) { // set max to 24h
      Serial.print("Setting polling time to ");
      Serial.print(new_polling);
      Serial.println(" seconds");
      polling = new_polling_val;
      if(polling == 0) {
        Serial.println("Auto polling disabled. Polling on request");
      }
    }
    if(polling == 0) {
      getSensors();
    }
  } else if (topic == mqtt_restart_topic) { // handle restart
    char restcmd[length];
    strncpy(restcmd, (char*)payload, length);
    if (!strcmp(restcmd, "yes")) {
      Serial.println("Restarting device");
      mqttClient.beginMessage(mqtt_connected_topic, true, 0, false);
      mqttClient.print(0);
      mqttClient.endMessage();
      NVIC_SystemReset();
    }
  }
}

void getSensors() {
  DynamicJsonDocument jsonBuffer(512);
  JsonObject sensors = jsonBuffer.to<JsonObject>();
  char json[512];

  //unsigned long unixtime = WiFi.getTime();
  int rssi = WiFi.RSSI();

  // publish mqtt messages
  //mqttPublish("timestamp", unixtime);
  mqttPublish("rssi", rssi);

  // prepare json
  //sensors["timestamp"] = unixtime;
  sensors["rssi"] = rssi;

  Serial.println("Reading sensors...");

  // Get sensors data

  // ========================= BME280 SENSOR ==========================
  if (bme_sensor) {
    bme.takeForcedMeasurement();
    float temperature = bme.readTemperature(); // celcius
    float humidity = bme.readHumidity(); // %
    float pressure = bme.readPressure() / 100.0F; // hPa

    if (temperature > -90.0 && temperature < 60.0) { // sanity check
      temperature = ((int) (temperature * 100)) / 100.0;
      mqttPublish("temperature", temperature);
      sensors["temperature"] = temperature;
    }

    if (humidity >= 0.0 && humidity <= 100.0) { // sanity check      
      humidity = ((int) (humidity * 100)) / 100.0;
      mqttPublish("humidity", humidity);
      sensors["humidity"] = humidity;
    }

    if (pressure > 850.0 && pressure < 1100.0) { // sanity check
      pressure = ((long) (pressure * 100)) / 100.0;
      //float altitude = ((long) (bme.readAltitude(SEALEVELPRESSURE_HPA) * 100) / 100.0); // meters
      mqttPublish("pressure", pressure);
      //mqttPublish("altitude", altitude);
      sensors["pressure"] = pressure;
      //sensors["altitude"] = altitude;
    }

    if (temperature > -90.0 && temperature < 60.0 && humidity >= 0.0 && humidity <= 100.0) { // sanity check
      float dewpoint = ((int) ((pow(humidity / 100.0, 0.125) * (112.0 + (0.9 * temperature)) + (0.1 * temperature) - 112.0) * 100)) / 100.0; // celcius
      mqttPublish("dew_point", dewpoint);
      sensors["dew_point"] = dewpoint;
    }
        
    if (DEBUG_SENSORS) {
      Serial.println("  BME280: OK");
    }
  }

  // ========================= MLX SENSOR ==========================
  if (irAmbient.size() > 0 && irSky.size() > 0) {
    float temperature_ambient = 1.0 * accumulate(irAmbient.begin(), irAmbient.end(), 0LL) / irAmbient.size(); // average ambient temperature over CLOUDS_AVERAGING_TIME
    float temperature_sky = 1.0 * accumulate(irSky.begin(), irSky.end(), 0LL) / irSky.size(); // average sky temperature over CLOUDS_AVERAGING_TIME

    // Based on AAG CloudWatcher: http://lunaticoastro.com/aagcw/TechInfo/SkyTemperatureModel.pdf
    // Cloudy sky is warmer that clear sky. Thus sky temperature meassured by IR sensor is a good indicator to estimate cloud cover.
    // However IR actually meassures the temperature of all the air column above, which is increassing with ambient temperature.
    // So it is important to include some correction factor.
    //
    // Corrected sky temp Tsky = Tobj – Td
    // Correction factor Td = (K1 / 100) * (Tamb – K2 / 10) + (K3 / 100) * pow((exp (K4 / 1000 * Tamb)), (K5 / 100)) + Tcw
    // Tcw is cold weather factor:
    //  if (abs(K2 / 10.0 - temperature_ambient) < 1.0) {
    //    Tcw = sgn(K6) * sgn(temperature_ambient - K2 / 10.0) * abs(K2 / 10.0 - temperature_ambient);
    //  } else {
    //    Tcw = K6 / 10.0 * sgn(temperature_ambient - K2 / 10.0) * (log(abs(K2 / 10.0 - temperature_ambient)) / log(10) + K7 / 100.0);
    //  }
    //
    // Tuning Sky Temperature Parameters: https://lunaticoastro.com/aagcw/enhelp/
    // From empirical observation, the limit between CLEAR and CLOUDY conditions corresponds to a value between -6°C and -3°C (the program default value = -5°C)
    // whereas the limit between CLOUDY and OVERCAST conditions is a value between 0°C and 2 °C (the program default value = 0°C).
    // However, during warm days / evenings, one notices that the measured sky temperature values reflect a large component due to other atmospheric radiation.
    // In order to cope with this effect a temperature correction model has been introduced (please refer to Sky Temperature Correction Coefficients).
    // This model calculates a correction value for the measured sky temperature as a function of the surface temperature.
    // The model is a combination of a linear and an exponential relationship where the linear relationship predominates for ambient temperatures below 20°C
    // whereas the exponential becomes more pronounced for ambient temperatures above this value.
    // To tune up this model, one should observe the Cloud conditions graph from sunup to sunset for a clear day.
    // As the ambient temperature changes during the day the Cloud conditions graph should remain horizontal, provided the sky conditions remain stable and cloudless.
    // If one notices that there is a downward trend in the graph line as the ambient temperature increases – this means that the calculated sky temperature correction factor must be increased.
    // If this trend occurs for temperatures below 25°C, one should try to increase the coefficient K1 by a small amount. (Suggestion: try a value from 33 to 38)
    // If the trend is more noticeable for values above 30°C, then one should adjust coefficients K3, K4 and K5. (Suggestion: try the following combination - K3 a value between 4 and 10 with K4=100 and K5=100)
    // On the other hand, if one notices that the graph is horizontal but it is too low, one may adjust coefficient K2.
    // This coefficient shifts upwards the calculated correction value as the coefficient gets smaller and vice-versa (Note that negative values are allowed for this coefficient).
    // The default values for coefficients K1, K2, K3, K4 and K5 are 33, 0, 0, 0 and 0 and this corresponds to a simple linear relationship.
    // The following coefficients have proved to yield good results too: K1=33, K2=0, K3=8, K4=100 and K5=100. This combination is more nonlinear for ambient temperatures above 30°C.
    //
    // Sky Temperature Correction Coefficients: https://lunaticoastro.com/aagcw/enhelp/index.htm#page=Operational%20Aspects/23-TemperatureFactor-.htm
    //
    // CloudWatcher default values
    // K1 = 33.0; K2 = 0.0; K3 = 4.0; K4 = 100.0; K5 = 100.0; K6 = 0.0; K7 = 0.0;
    // Tclear = -5.0; // Clear sky corrected temperature (temp below means 0% clouds)
    // Tcloudy = 0.0; // Covered sky corrected temperature (temp above means 100% clouds)

    float K1 = 33.0; // K1 mainly affects the slope of the curve in its linear section (i.e. ambient temperature below 25ºC);
    float K2 = 0.0; // K2 mainly affects the x-axis crossing point – it shifts the curve upwards as K2 gets smaller and vice-versa;
    float K3 = 8.0; // K3, K4 and K5  have a large effect on the shape of the curve for ambient temperatures above 30ºC;
    float K4 = 100.0;
    float K5 = 100.0;
    float K6 = 0.0; // K6 and K7 introduce a S bent around x-axis crossing point;
    float K7 = 0.0;
    float Tclear = -10.0; //Clear sky corrected temperature (temp below means 0% clouds)
    float Tcloudy = 0.0; //Covered sky corrected temperature (temp above means 100% clouds)
    float Tcw = 0.0; // Tcw is cold weather factor

    if (abs(K2 / 10.0 - temperature_ambient) < 1.0) {
      Tcw = sgn(K6) * sgn(temperature_ambient - K2 / 10.0) * abs(K2 / 10.0 - temperature_ambient);
    } else {
      Tcw = K6 / 10.0 * sgn(temperature_ambient - K2 / 10.0) * (log(abs(K2 / 10.0 - temperature_ambient)) / log(10) + K7 / 100.0);
    }

    // Calculate corrected sky temperature
    float Td = (K1 / 100.0) * (temperature_ambient - K2 / 10.0) + (K3 / 100.0) * pow((exp (K4 / 1000.0 * temperature_ambient)) , (K5 / 100.0)) + Tcw;
    float Tsky = temperature_sky - Td;
      
    if (Tsky < Tclear) Tsky = Tclear;
    if (Tsky > Tcloudy) Tsky = Tcloudy;
  
    float clouds = (((int) ((Tsky - Tclear) * 100.0 / (Tcloudy - Tclear)) * 100)) / 100.0;

    mqttPublish("ir_ambient", temperature_ambient);
    sensors["ir_ambient"] = temperature_ambient;

    mqttPublish("ir_sky", temperature_sky);
    sensors["ir_sky"] = temperature_sky;

    mqttPublish("clouds", clouds);
    sensors["clouds"] = clouds;

    // clear values for next reading
    irAmbient.clear();
    irSky.clear();

    if (DEBUG_SENSORS) {
      Serial.println("  MLX: OK");
    }
  }

  // ========================= LIGHT SENSOR ==========================
  // Example Lux Levels:
  // Direct Sunlight    100000
  // Full Daylight      10000
  // Overcast Day       1000
  // Dark Overcast Day  100
  // Twilight           10
  // Deep Twilight      1
  // Full Moon          0.1
  // Quarter Moon       0.01
  // Starlight          0.001
  // Overcast Night     0.0001

  float lux;
  int als_status = als.getALSLux(lux);
  if (als_status == 0) {
    mqttPublish("light", lux);
    sensors["light"] = lux;
    if (DEBUG_SENSORS) {
      Serial.println("  VEML: OK");
    }
  }

  // ========================= WIND SPEED ==========================
  float windspeed = 0;
  if (windSpeed.size() > 0) {
  
    for (int i = 0; i < windSpeed.size(); i++) {
      windspeed = windspeed + windSpeed[i];
    }
  
    windspeed = windspeed / windSpeed.size();
    windspeed = ((int)(windspeed * 100)) / 100.0;
  
    mqttPublish("wind_speed", windspeed);
    sensors["wind_speed"] = windspeed;      

    // clear values for next reading
    windSpeed.clear();

    if (DEBUG_SENSORS) {
      Serial.println("  Wind speed: OK");
    }
  }


  // ========================= WIND DIRECTION ==========================
  if (windDir.size() > 0) {
    // We can't just take an average of spot measures.
    // We need to use "mean of circular quantities" method instead
    // We are using Mitsuta method for the calculation of mean direction
    // Based on: http://abelian.org/vlf/bearings.html
    // Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
  
    int windDirSum = windDir[0];
    int windDirD = windDir[0];
    for(int i = 1; i < windDir.size(); i++)
    {
      int windDirDelta = windDir[i] - windDirD;

      if (windDirDelta <= -360) {
        windDirD += windDirDelta + 360;
      } else if (windDirDelta > 360) {
        windDirD += windDirDelta - 360;
      } else {
        windDirD += windDirDelta;
      }

      windDirSum += windDirD;
    }
    
    int winddir = windDirSum / windDir.size();
    if(winddir >= 360) winddir -= 360;
    if(winddir < 0) winddir += 360;
  
    String winddir_cardinal = cardinal.getString(2, winddir);

    mqttPublish("wind_dir", winddir);
    sensors["wind_dir"] = winddir;
    mqttPublish("wind_dir_cardinal", winddir_cardinal.c_str());    
    sensors["wind_dir_cardinal"] = winddir_cardinal;    

    // clear values for next reading
    windDir.clear();

    if (DEBUG_SENSORS) {
      Serial.println("  Wind direction: OK");
    }
  }

  // ========================= WIND GUST SPEED ==========================
  // We get maximum value from spot measurements in loop

  windGustSpeed = ((int)(windGustSpeed * 100)) / 100.0;

  mqttPublish("wind_gust_speed", windGustSpeed);
  sensors["wind_gust_speed"] = windGustSpeed;      

  // clear values for next reading
  windGustSpeed = 0;

  if (DEBUG_SENSORS) {
    Serial.println("  Wind gust: OK");
  }

  // ========================= WIND GUST DIRECTION ==========================
  if (windGustDir.size() > 0) {
    // We can't just take an average of spot measures.
    // We need to use "mean of circular quantities" method instead
    // We are using Mitsuta method for the calculation of mean direction
    // Based on: http://abelian.org/vlf/bearings.html
    // Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
  
    int windGustSum = windGustDir[0];
    int windGustD = windGustDir[0];

    for(int i = 1; i < windGustDir.size(); i++)
    {
        int windGustDelta = windGustDir[i] - windGustD;
  
        if (windGustDelta <= -360) {
          windGustD += windGustDelta + 360;
        } else if (windGustDelta > 360) {
          windGustD += windGustDelta - 360;
        } else {
          windGustD += windGustDelta;
        }
  
        windGustSum += windGustD;
    }

    int windgustdir = windGustSum / windGustDir.size();
    if(windgustdir >= 360) windgustdir -= 360;
    if(windgustdir < 0) windgustdir += 360;

    String windgustdir_cardinal = cardinal.getString(2, windgustdir);

    if (windGustSpeed > 0) {
      mqttPublish("wind_gust_dir", windgustdir);
      sensors["wind_gust_dir"] = windgustdir;
      mqttPublish("wind_gust_dir_cardinal", windgustdir_cardinal.c_str());
      sensors["wind_gust_dir_cardinal"] = windgustdir_cardinal;      
    }

    // clear values for next reading
    windGustDir.clear();

    if (DEBUG_SENSORS) {
      Serial.println("  Wind gust direction: OK");
    }
  }

  // ========================= DEW ==========================
  // TODO

  // ========================= RAIN FALL ==========================
  unsigned long rainCheck = millis();
  float rainfall = rainClicks * 0.2794; // There is 0.011" = 0.2794 mm of rainfall for each click
  rainfall = ((int)(rainfall * 100)) / 100.0;

  mqttPublish("rain", rainfall);
  sensors["rain"] = rainfall;

  // clear values for next reading
  rainClicks = 0;

  if (DEBUG_SENSORS) {
    Serial.println("  Rain fall: OK");
  }

  // ========================= JSON ==========================
  serializeJson(sensors, json);
  mqttPublish("json", json);

}

int get_wind_direction()
{
  unsigned int adc;
  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}
