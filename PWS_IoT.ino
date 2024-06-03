/*
  Name: Personal Weather Station IoT
  Version: 1.3
  By: Radek Kaczorek, (C) 2021 - 2024
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
    WIFI_SSID "wifi_name"
    WIFI_PASS "wifi_password"
    MQTT_HOST "mqtt_address"
    MQTT_PORT 1883
    MQTT_USER "mqtt_user"
    MQTT_PASS "mqtt_password"
    MQTT_ROOT_TOPIC "environment"
    MQTT_DEVICE_ID "pws"
    MQTT_DEVICE_ID_TO_HOSTNAME true
    DEBUG false
    WIND_DIR_CORRECTION 60
    WIND_AVERAGING_TIME 10000
    CLOUDS_AVERAGING_TIME 5000
    HOMEASSISTANT
    HA_DISCOVERY_TOPIC
*/

#define VERSION 1.3.1

#include "PWS_IoT.h"

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h> // https://www.arduino.cc/en/Reference/WiFiNINA // https://github.com/arduino-libraries/WiFiNINA
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h> // https://www.arduino.cc/en/Reference/WiFi101
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h> // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#endif

#include <Wire.h> // https://github.com/esp8266/Arduino/tree/master/libraries/Wire
#include <ArduinoMqttClient.h> // https://github.com/arduino-libraries/ArduinoMqttClient
#include <ArduinoOTA.h> // https://github.com/jandrassy/ArduinoOTA
#include <ArduinoJson.h>  // https://github.com/bblanchon/ArduinoJson
#include <Adafruit_BME280.h> // https://github.com/adafruit/Adafruit_BME280_Library
#include <SparkFunMLX90614.h> // https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library
#include "Adafruit_VEML7700.h" // https://github.com/adafruit/Adafruit_VEML7700
#include <Cardinal.h> // https://github.com/DaAwesomeP/arduino-cardinal
#include <SimpleTime.h>

#include <vector>
#include <numeric>

#define SEALEVELPRESSURE_HPA  (1013.25) // used for altitude calculation based on pressure
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0)) // used for arithmetic sign of a value (needed by cloud detection algorithm)

// ===================================================================
//                       Enable/Disable sensors
// ===================================================================
// -------------------------- autodetected ---------------------------
int bme_sensor = 0; // Temperature, Humidity, Pressure
int mlx_sensor = 0; // Sky temperature / Clouds
int wind_sensor = 0; // Wind Speed - set to 1 to enable
int veml_sensor = 0; // Light
// -------------------------- set manually ---------------------------
int rainfall_sensor = 1; // Rainfall - set to 1 to enable
// ===================================================================

// hardware pins definitions
const byte WDIR = A0;
const byte WSPEED = 10;
const byte RAINFALL = 11;

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
unsigned int polling = 60; // auto polling time

// mqtt
const char* mqtt_host = MQTT_HOST;
int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_pass = MQTT_PASS;

// mqtt topics
char mqtt_root_topic[32] = MQTT_ROOT_TOPIC; // eg. environment
char mqtt_device_id[32] = MQTT_DEVICE_ID; // eg. mysensor
char mqtt_device_topic[64]; // = mqtt_root_topic/mqtt_device_id
char mqtt_setroot_topic[36]; // eg. environment/set
char mqtt_setname_topic[70]; // eg. environment/mysensor/set
char mqtt_status_topic[76]; // eg. environment/mysensor/status
char mqtt_poll_topic[71]; // eg. environment/mysensor/poll
char mqtt_control_topic[74]; // eg. environment/mysensor/control

byte mac[6]; // 6 byte array to hold the MAC address
char hostname[32]; // an array to hold wifi hostname

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
int wifiStatus = WL_IDLE_STATUS;
WiFiServer server(23);

Adafruit_BME280 bme; // BME280 sensor - temperature, humidity, pressure
IRTherm mlx; // MLX90614 sensor - infrared termometer
Adafruit_VEML7700 veml = Adafruit_VEML7700(); // VEML7700 sensor - ambient light
Cardinal cardinal; // Cardinal direction NESW

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
  pinMode(LED_BUILTIN, OUTPUT); // led
  pinMode(WDIR, INPUT); // input from winddir sensor
  pinMode(WSPEED, INPUT_PULLUP); // input from windspeed sensor
  pinMode(RAINFALL, INPUT_PULLUP); // input from rain gauge sensor

  // attach external interrupt pins to IRQ functions
  attachInterrupt(digitalPinToInterrupt(RAINFALL), rainfallIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), windspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  Serial.begin(9600);
  delay(3000); // wait for serial port to connect. Needed for native USB port only

  blinkLED(1);

  Serial.println("====================================");
  Serial.print("|  Personal Weather Station v");
  Serial.print(VERSION);
  Serial.println("  |");
  Serial.println("====================================");
  Serial.println("");

  // List all available I2C devices
  if (DEBUG)
    getI2Cdevices();

  // Init sensors
  Serial.println("Initializing sensors...");

  // Init BME280
  if (bme.begin(0x76) || bme.begin(0x77)) { // i2c slave address: 0x76 or 0x77
    Serial.println("  - BME280 sensor\tOK");
    bme_sensor = 1;
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF
                  );
  } else {
    Serial.println("  - BME280 sensor\tERROR");
    bme_sensor = 0;
  }

  // Init MLX
  if (mlx.begin()) { // i2c slave address: 0x5a
    Serial.println("  - MLX90614 sensor\tOK");
    mlx.setUnit(TEMP_C); // Set the library's units to Celsius
    mlx_sensor = 1;
  } else {
    Serial.println("  - MLX90614 sensor\tERROR");
    mlx_sensor = 0;
  }

  // Init VEML
  if (veml.begin()) { // i2c slave address: 0x10
    veml_sensor = 1;
    Serial.println("  - VEML sensor\t\tOK");
  } else {
    veml_sensor = 0;
    Serial.println("  - VEML sensor\t\tERROR");
  }

  // Rainfall sensor
  if (rainfall_sensor) {
    Serial.println("  - Rainfall sensor\tOK");
  } else {
    Serial.println("  - Rainfall sensor\tERROR");
  }

  // Wind sensors
  int winddir = get_wind_direction();
  if (winddir != -1)
  {
    wind_sensor = 1;
  } else {
    wind_sensor = 0;
  }

  if (wind_sensor) {
    Serial.println("  - Wind sensors\tOK");
  } else {
    Serial.println("  - Wind sensors\tERROR");
  }

  // Enable autodiscovery of sensors for Home Assistant
  if (HOMEASSISTANT) {
    Serial.println("Home Assistant MQTT integration ENABLED");

    if (DEBUG) {
      Serial.print("Autodiscovery topic: ");
      Serial.println(HA_DISCOVERY_TOPIC);
    }
  
    // Default - always available
    initHAEntity("status", "sensor", NULL, NULL, "mdi:connection", "Status");
    initHAEntity("last_seen", "sensor", "timestamp", NULL, "mdi:clock", "Last Seen");
    initHAEntity("poll", "sensor", NULL, "s", "mdi:clock", "Update Interval");
    initHAEntity("rssi", "sensor", "signal_strength", "dBm", NULL, "RSSI");

    // Rainfall
    if (rainfall_sensor) {
      initHAEntity("rainfall", "sensor", "precipitation", "mm", NULL, "Rainfall");
    }

    // Wind Speed & Direction
    if (wind_sensor) {
      initHAEntity("wind_speed", "sensor", "wind_speed", "km/h", NULL, "Wind Speed");
      initHAEntity("wind_gust_speed", "sensor", "wind_speed", "km/h", NULL, "Wind Gust Speed");
      initHAEntity("wind_dir", "sensor", NULL, "°", "mdi:compass", "Wind Direction");
      initHAEntity("wind_dir_cardinal", "sensor", NULL, NULL, "mdi:compass", "Wind Direction");
      initHAEntity("wind_gust_dir", "sensor", NULL, "°", "mdi:compass", "Wind Gust Direction");
      initHAEntity("wind_gust_dir_cardinal", "sensor", NULL, NULL, "mdi:compass", "Wind Gust Direction");
    }

    // BME
    if (bme_sensor) {
      initHAEntity("temperature", "sensor", "temperature", "°C", NULL, "Temperature");
      initHAEntity("humidity", "sensor", "humidity", "%", NULL, "Humidity");
      initHAEntity("dew_point", "sensor", "temperature", "°C", NULL, "Dew Point");
      initHAEntity("pressure", "sensor", "atmospheric_pressure", "hPa", NULL, "Pressure");
    }    

    // MLX
    if (mlx_sensor) {
      initHAEntity("ir_ambient", "sensor", "temperature", "°C", NULL, "IR Ambient Temperature");
      initHAEntity("ir_sky", "sensor", "temperature", "°C", NULL, "IR Sky Temperature");
      initHAEntity("clouds", "sensor", NULL, "%", "mdi:cloud", "Clouds");
    }

    // VEML
    if (veml_sensor) {
      initHAEntity("light", "sensor", "illuminance", "lx", NULL, "Light");
    }

  } else {
      Serial.println("Home Assistant integration DISABLED");
  }

  Serial.println("");

  // Ready
  Serial.println("Personal Weather Station OK");
  blinkLED(4);
}

void loop() {
  wifiConnect(); // Autoconnect WiFi
  mqttConnect(); // Autoconnect MQTT

  // collect wind sensors data every loop
  if (millis() - lastWindMillis > WIND_AVERAGING_TIME) {

    // get wind direction
    int winddir = get_wind_direction();

    telnetPrint(winddir);
    
    if (winddir != -1) {
      wind_sensor = 1; // we have readings - ENABLE
      // calculate corrected wind direction
      winddir += WIND_DIR_CORRECTION;
      if(winddir >= 360) winddir -= 360;
      if(winddir < 0) winddir += 360;
      windDir.push_back(winddir);
    } else {
      wind_sensor = 0; // we do not have readings - DISABLE
    }

    if (wind_sensor) {
      // get wind speed
      float windspeed = (float) windClicks / (WIND_AVERAGING_TIME / 1000); // clicks per second

      telnetPrint(windspeed);

      windClicks = 0;
      windspeed *= 2.4011412; // 1 click per second equals 1.492MPH = 2.4011412 km/h
      windSpeed.push_back(windspeed);      

      // get wind gust and wind gust direction
      if (windspeed > windGustSpeed) {
        windGustSpeed = windspeed;
        if (winddir != -1) {
          windGustDir.push_back(winddir);
        }
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

      telnetPrint(temperature_ambient);
      telnetPrint(temperature_sky);

      if (temperature_ambient > -273.0 && temperature_sky > -273.0) { // sanity check
        irAmbient.push_back(temperature_ambient);
        irSky.push_back(temperature_sky);
      }
    }
    lastCloudsMillis = millis();
  }
  autoPolling();
}

void telnetPrint(const char* payload) {
  WiFiClient client = server.available();
  if (client && client.connected()) {
    for (int i = 0; i < strlen(payload); i++)
      server.write(payload[i]);
    server.write("\n");
  }
}

void telnetPrint(float val) {
    char payload[16];
    sprintf(payload, "%f", val);
    telnetPrint(payload);
}

void restartDevice() {
  Serial.println("Restarting device in 3 seconds...");
  delay(3000);
  NVIC_SystemReset();
}

void autoPolling() {
  if ( polling == 0 || millis() - timestamp < polling * 1000)
    return;
  getSensors();
  timestamp = millis();
}

void wifiConnect() {
  int count = 0;

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.poll(); // check for WiFi OTA updates
    return;
  }

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    restartDevice();
  }
  
  WiFi.macAddress(mac);
  sprintf(hostname, "pws-%x%x", mac[1], mac[0]);
  WiFi.setHostname(hostname);

  Serial.print("Connecting to WiFi... ");
  do
  {
    wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(100);
  }
  while (wifiStatus != WL_CONNECTED && count++ < 5 );

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("OK");
    if (DEBUG) {
      Serial.print("  - SSID: ");
      Serial.println(WIFI_SSID);
      Serial.print("  - Hostname: ");
      Serial.println(hostname);
      Serial.print("  - IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("  - Subnet mask: ");
      Serial.println(WiFi.subnetMask());
      Serial.print("  - Default gateway: ");
      Serial.println(WiFi.gatewayIP());
      Serial.println();
    }
    ArduinoOTA.begin(WiFi.localIP(), OTA_USER, OTA_PASS, InternalStorage); // Start the WiFi OTA
    server.begin(); // Start telnet server
  } else {
    Serial.println("ERROR");
    delay(3000);
    NVIC_SystemReset();
  }
}

void mqttConnect() {
  mqttClient.poll(); // send MQTT keep alives

  if (mqttClient.connected()) {
    return;
  }

  Serial.print("Connecting to the MQTT broker... ");

  // set MQTT device name to hostname
  if (MQTT_DEVICE_ID_TO_HOSTNAME) {
    strcpy(mqtt_device_id, hostname);
  }

  // convert MQTT device name to lower case
  for (int i = 0; i < strlen(mqtt_device_id); i++)
    mqtt_device_id[i] = tolower(mqtt_device_id[i]);

  // construct MQTT topics
  sprintf(mqtt_device_topic, "%s/%s", mqtt_root_topic, mqtt_device_id); // simplified device topic
  sprintf(mqtt_status_topic, "%s/status", mqtt_device_topic); // device status

  // device control via MQTT
  sprintf(mqtt_setroot_topic, "%s/set", mqtt_root_topic); // setting root name
  sprintf(mqtt_setname_topic, "%s/set", mqtt_device_topic);   // setting device name
  sprintf(mqtt_control_topic, "%s/control", mqtt_device_topic); // controling device
  sprintf(mqtt_poll_topic, "%s/poll", mqtt_device_topic); // setting polling time (configurable via mqtt)

  mqttClient.setId(mqtt_device_id);
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);
  //mqttClient.setCleanSession(true); // http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Keep_Alive
  //mqttClient.setKeepAliveInterval(60*1000L);
  //mqttClient.setConnectionTimeout(60*1000L);

  // set callback
  mqttClient.onMessage(mqttCallback);
  
  if (mqttClient.connect(mqtt_host, mqtt_port)) {
    Serial.println("OK");
    if (DEBUG) {
      Serial.print("  - Host: ");
      Serial.print(mqtt_host);
      Serial.print(":");
      Serial.println(mqtt_port);
      Serial.print("  - Client ID: ");
      Serial.println(mqtt_device_id);
      Serial.print("  - Topic: ");
      Serial.print(mqtt_root_topic);
      Serial.print("/");
      Serial.println(mqtt_device_id);
      Serial.println();
    }

    // set last will message
    String lastWill = "offline";
    mqttClient.beginWill(mqtt_status_topic, lastWill.length(), true, 1);
    mqttClient.print(lastWill);
    mqttClient.endWill();
  
    // Subscribe to control topics
    mqttSubscribe();

    // Publish status
    mqttPublishStatus(1);
  } else {
    Serial.print("ERROR");
    if (DEBUG) {
      Serial.print(" (");
      switch (mqttClient.connectError()) {
        case -2:
          Serial.print("Connection refused");
          break;
        case -1:
          Serial.print("Connection timeout");
          break;
        case 1:
          Serial.print("Unacceptable protocol version");
          break;
        case 2:
          Serial.print("Identifier rejected");
          break;
        case 3:
          Serial.print("Server unavailable");
          break;
        case 4:
          Serial.print("Bad username or password");
          break;
        case 5:
          Serial.print("Not authorized");
          break;
        default:
          Serial.print("Connection refused");
      }
      Serial.println(")");
    } else {
      Serial.println();
    }
  }
}

bool mqttSubscribe() {
  Serial.print("Subscribing to control topics... ");
  if (mqttClient.subscribe(mqtt_poll_topic) && mqttClient.subscribe(mqtt_control_topic)) {
    // successfully subscribed to topics
    Serial.println("OK");
    if(DEBUG) {
      Serial.print("  - Publish '0' to ");
      Serial.print(mqtt_poll_topic);
      Serial.println(" to read sensors on request or publish a number to set auto polling in seconds");
      Serial.print("  - Publish command to "); // note: 'restart' command available only
      Serial.print(mqtt_control_topic);
      Serial.println(" to control the device");
      Serial.println("");
    }
    return true;
  } else {
    Serial.println("ERROR");
    Serial.println("");
    return false;
  }
}

void mqttPublishStatus(int status)
{
  String payload = "online";
  bool retain = false;
  int qos = 0;

  if ( status == 0 ) {
    payload = "offline";
    retain = false;
    qos = 0;
  }

  mqttClient.beginMessage(mqtt_status_topic, payload.length(), retain, qos, false);
  mqttClient.print(payload);
  mqttClient.endMessage();
}

void mqttPublishWeather(char* topic, float val) {
  char mqtt_topic[512] = "";
  char msg[16];
  sprintf(mqtt_topic, "%s/%s", mqtt_device_topic, topic);
  sprintf(msg, "%0.2f", val);

  bool retain = false;
  int qos = 0;

  mqttClient.beginMessage(mqtt_topic, retain, qos);
  mqttClient.print(msg);
  mqttClient.endMessage();

  if (DEBUG) {
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

void mqttPublishWeather(char* topic, const char* msg) {
  char mqtt_topic[512] = "";
  sprintf(mqtt_topic, "%s/%s", mqtt_device_topic, topic);
  size_t payloadSize = strlen(msg);

  bool retain = false;
  int qos = 0;

  mqttClient.beginMessage(mqtt_topic, payloadSize, retain, qos, false);
  mqttClient.print(msg);
  mqttClient.endMessage();

  if (DEBUG) {
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

void mqttPublishHA(char* topic, const char* msg) {
  char mqtt_topic[128] = "";
  strcpy(mqtt_topic, topic);
  size_t payloadSize = strlen(msg);

  bool retain = true;
  int qos = 1;

  mqttClient.beginMessage(mqtt_topic, payloadSize, retain, qos, false);
  mqttClient.print(msg);
  mqttClient.endMessage();
}

void mqttCallback(int length) {
  char payload[length];
  String topic = mqttClient.messageTopic();

  // get message
  for (int i = 0; i < length; i++) {
    payload[i] = (char)mqttClient.read();
    payload[i+1] = '\0';
  }

  if (DEBUG) {
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
      if (DEBUG) {
        Serial.print("Setting polling time to ");
        Serial.print(new_polling);
        Serial.println(" seconds");        
      }
      polling = new_polling_val;
      if(polling == 0) {
        Serial.println("Auto polling disabled. Polling on request");
      }
    }
    if(polling == 0) {
      getSensors();
    }
  } else if (topic == mqtt_control_topic) { // handle 'restart' command
    char restcmd[length];
    strncpy(restcmd, (char*)payload, length);
    if (!strcmp(restcmd, "restart")) {
      mqttPublishStatus(0);
      restartDevice();
    }
  }
}

void initHAEntity(const char* sensor, const char* ha_sensor_type, const char* ha_device_class, const char* uom, const char* icon, const char* ha_friendly_device_name)
{
  DynamicJsonDocument jsonBufferConfig(512);
  JsonObject jhc = jsonBufferConfig.to<JsonObject>();

  DynamicJsonDocument jsonBufferDevice(128);
  JsonObject jdev = jsonBufferDevice.to<JsonObject>();

  // prepare topics
  char ha_state_topic[128];
  char ha_config_topic[128];
  char ha_config[512];

  sprintf(ha_state_topic, "%s/%s", mqtt_device_topic, sensor);
  sprintf(ha_config_topic, "%s/%s/%s/%s/config", HA_DISCOVERY_TOPIC, ha_sensor_type, mqtt_device_id, sensor);

  // prepare config
  char ha_dev_unique_id[32];
  sprintf(ha_dev_unique_id, "%s_%s", mqtt_device_id, sensor);

  jdev["identifiers"] = mqtt_device_id;
  jdev["name"] = "Personal Weather Station";
  jdev["manufacturer"] = "Radek Kaczorek";
  jdev["model"] = "Arduino Nano 33 IoT";
  jdev["sw_version"] = VERSION;

  // prepare json
  if (jdev)
    jhc["device"] = jdev;
  if (ha_dev_unique_id)
    jhc["unique_id"] = ha_dev_unique_id;
  if (ha_device_class)
    jhc["device_class"] = ha_device_class;
  if (ha_dev_unique_id)
    jhc["object_id"] = ha_dev_unique_id;
  if (ha_friendly_device_name)
    jhc["name"] = ha_friendly_device_name;
  if (ha_state_topic)
    jhc["state_topic"] = ha_state_topic;
  if (uom)
    jhc["unit_of_measurement"] = uom;
  if (icon)
    jhc["icon"] = icon;

  // publish HA config
  serializeJson(jhc, ha_config);
  mqttPublishHA(ha_config_topic, ha_config);

  if (DEBUG) {
    Serial.print("  - ");
    Serial.println(sensor);
    /*
    Serial.print("    ha_config_topic:");
    Serial.println(strlen(ha_config_topic));
    Serial.print("    ha_state_topic:");
    Serial.println(strlen(ha_state_topic));
    Serial.print("    ha_config:");
    Serial.println(strlen(ha_config));
    */
  }
}

void getSensors() {
  DynamicJsonDocument jsonBuffer(512);
  JsonObject sensors = jsonBuffer.to<JsonObject>();
  char json[512];

  blinkLED(1);

  // Update PWS status
  mqttPublishStatus(1);
  sensors["status"] = "online";

  // Last seen
  // 2024-01-04T21:45:19+00:00
  char last_seen[32];
  time_t t = WiFi.getTime();
  sprintf(last_seen, "%d-%02d-%02dT%02d:%02d:%02d+00:00", year(t), month(t), day(t), hour(t), minute(t), second(t));
  mqttPublishWeather("last_seen", last_seen);
  sensors["last_seen"] = last_seen;

  // IP
  char localip[16], iptopic[32];
  IPAddress ip = WiFi.localIP();
  sprintf(localip, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  mqttPublishWeather("ip", localip);
  sensors["ip"] = localip;

  // RSSI
  int rssi = WiFi.RSSI();
  mqttPublishWeather("rssi", rssi);
  sensors["rssi"] = rssi;

  // RSSI
  mqttPublishWeather("poll", polling);
  sensors["poll"] = polling;

  if (DEBUG) {
    Serial.println("Reading sensors...");
  }

  // Get sensors data

  // ========================= BME280 SENSOR ==========================
  if (bme_sensor) {
    bme.takeForcedMeasurement();
    float temperature = bme.readTemperature(); // celcius
    float humidity = bme.readHumidity(); // %
    float pressure = bme.readPressure() / 100.0F; // hPa

    telnetPrint(temperature);
    telnetPrint(humidity);
    telnetPrint(pressure);

    if (C_ENABLE)
      temperature += C_TEMPERATURE; // sensor reading correction

    if (temperature > -90.0 && temperature < 60.0) { // sanity check
      temperature = ((int) (temperature * 100)) / 100.0;
      mqttPublishWeather("temperature", temperature);
      sensors["temperature"] = temperature;
    } else {
      Serial.print("Temperature sensor error! Raw value: ");
      Serial.println(temperature);
    }

    if (humidity >= 0.0 && humidity <= 100.0) { // sanity check      
      humidity = ((int) (humidity * 100)) / 100.0;
      mqttPublishWeather("humidity", humidity);
      sensors["humidity"] = humidity;
    } else {
      Serial.print("Humidity sensor error! Raw value: ");
      Serial.println(humidity);
    }

    if (pressure > 850.0 && pressure < 1100.0) { // sanity check
      pressure = ((long) (pressure * 100)) / 100.0;
      //float altitude = ((long) (bme.readAltitude(SEALEVELPRESSURE_HPA) * 100) / 100.0); // meters
      mqttPublishWeather("pressure", pressure);
      //mqttPublishWeather("altitude", altitude);
      sensors["pressure"] = pressure;
      //sensors["altitude"] = altitude;
    } else {
      Serial.print("Pressure sensor error! Raw value: ");
      Serial.println(pressure);
    }

    if (temperature > -90.0 && temperature < 60.0 && humidity >= 0.0 && humidity <= 100.0) { // sanity check
      float dewpoint = ((int) ((pow(humidity / 100.0, 0.125) * (112.0 + (0.9 * temperature)) + (0.1 * temperature) - 112.0) * 100)) / 100.0; // celcius
      mqttPublishWeather("dew_point", dewpoint);
      sensors["dew_point"] = dewpoint;
    } else {
      Serial.println("Dew point calculation error! Check temperature and humidity sensors.");
    }
        
    if (DEBUG) {
      Serial.println("  BME280: OK");
    }
  }

  // ========================= MLX SENSOR ==========================
  if (mlx_sensor) {
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
  
      float K1 = 34.0; // K1 mainly affects the slope of the curve in its linear section (i.e. ambient temperature below 25ºC);
      float K2 = 0.0; // K2 mainly affects the x-axis crossing point – it shifts the curve upwards as K2 gets smaller and vice-versa;
      float K3 = 6.0; // K3, K4 and K5  have a large effect on the shape of the curve for ambient temperatures above 30ºC;
      float K4 = 100.0;
      float K5 = 100.0;
      float K6 = 0.0; // K6 and K7 introduce a S bent around x-axis crossing point;
      float K7 = 0.0;
      float Tclear = -5.0; //Clear sky corrected temperature (temp below means 0% clouds) // changed from -8 on 09/07/2023
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
  
      mqttPublishWeather("ir_ambient", temperature_ambient);
      sensors["ir_ambient"] = temperature_ambient;
  
      mqttPublishWeather("ir_sky", temperature_sky);
      sensors["ir_sky"] = temperature_sky;
  
      mqttPublishWeather("clouds", clouds);
      sensors["clouds"] = clouds;
  
      // clear values for next reading
      irAmbient.clear();
      irSky.clear();
  
      if (DEBUG) {
        Serial.println("  MLX: OK");
      }
    }
  }

  if (veml_sensor) {
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
  
    float lux = veml.readLux(VEML_LUX_AUTO);

    telnetPrint(lux);

    if (lux >= 0 && lux < 100000 ) { // sanity check
      mqttPublishWeather("light", lux);
      sensors["light"] = lux;
      if (DEBUG) {
        Serial.println("  VEML: OK");
      }
    }
  }

  if (wind_sensor) {
    // ========================= WIND SPEED ==========================
    float windspeed = 0;
    if (windSpeed.size() > 0) {
    
      for (int i = 0; i < windSpeed.size(); i++) {
        windspeed = windspeed + windSpeed[i];
      }
    
      windspeed = windspeed / windSpeed.size();
      windspeed = ((int)(windspeed * 100)) / 100.0;
  
      // maximum reported wind speed - hurricane
      if (windspeed > 120.0) {
        windspeed = 120.0;
      }
  
      mqttPublishWeather("wind_speed", windspeed);
      sensors["wind_speed"] = windspeed;
  
      // clear values for next reading
      windSpeed.clear();
  
      if (DEBUG) {
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
  
      mqttPublishWeather("wind_dir", winddir);
      sensors["wind_dir"] = winddir;
      mqttPublishWeather("wind_dir_cardinal", winddir_cardinal.c_str());    
      sensors["wind_dir_cardinal"] = winddir_cardinal;    
  
      // clear values for next reading
      windDir.clear();
  
      if (DEBUG) {
        Serial.println("  Wind direction: OK");
      }
    }

    // ========================= WIND GUST SPEED ==========================
    // We get maximum value from spot measurements in loop
  
    windGustSpeed = ((int)(windGustSpeed * 100)) / 100.0;
  
     // maximum reported wind gust speed - hurricane
    if (windGustSpeed > 120.0) {
      windGustSpeed = 120.0;
    }

    mqttPublishWeather("wind_gust_speed", windGustSpeed);
    sensors["wind_gust_speed"] = windGustSpeed;      
  
    // clear values for next reading
    windGustSpeed = 0;
  
    if (DEBUG) {
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
  
      mqttPublishWeather("wind_gust_dir", windgustdir);
      sensors["wind_gust_dir"] = windgustdir;
      mqttPublishWeather("wind_gust_dir_cardinal", windgustdir_cardinal.c_str());
      sensors["wind_gust_dir_cardinal"] = windgustdir_cardinal;
  
      // clear values for next reading
      windGustDir.clear();
  
      if (DEBUG) {
        Serial.println("  Wind gust direction: OK");
      }
    }
  }

  if (rainfall_sensor) {
    // ========================= RAINFALL ==========================
    /*
    Rain gauge provides buckets which measure 0.2794 mm (0.011") of rainfall for each click.

    Light rain - less than 2.5 mm/h (<0.1”/hr) or <0.04 mm/min (<0.0007”/min) rain rate, which is 9 full tipping buckets per hour (9 pulses/hr).    
    Moderate rain - rain rate of fall is 2.6 to 7.5 mm/h (0.1 to 0.3”/hr) or 0.04 to 0.125 mm/min (0.0017 to 0.005”/hr), which is 10 to 27 full tipping buckets per hour (10 to 27 pulses/hr).
    Heavy rain - rain rate is greater than 7.6 to 50 mm/h (0.3 to 2”/hr) or 0.125 to 0.83 mm/min (0.005" to 0.033”/min), which is 28 or more full tipping buckets per hour (28+ pulses/hr).
    Violent rain - rain rate grater than >50 mm/hr (>2 in/hr) or >0.83 mm/min (>0.033”/min), which is 180 or more full tipping buckets per hour (180+ pulses/hr, 3+ pulses/minute).

    Note: Rainfall intensity calculation (mm/h or mm/min) must be handled on client side. PWS IoT returns only raw rainfall values in mm every N seconds, where N = configurable polling time.
    */
    
    float rainfall = rainClicks * 0.2794; // There is 0.011" = 0.2794 mm of rainfall for each click

    telnetPrint(rainfall);
    
    rainClicks = 0; // clear values for next reading
    
    rainfall = ((int)(rainfall * 10000)) / 10000.0;

    mqttPublishWeather("rainfall", rainfall);
    sensors["rainfall"] = rainfall;
  
    if (DEBUG) {
      Serial.println("  Rainfall: OK");
    }
  }

  // ========================= JSON ==========================

  serializeJson(sensors, json);
  mqttPublishWeather("json", json);
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

void getI2Cdevices()
{
  byte error, address;
  int nDevices;

  Wire.begin();

  Serial.println("Scanning I2C bus for slaves...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("  - I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");

      Serial.println(address,HEX);

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("  - Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");

      Serial.println(address,HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found");

  Serial.println("");
}

void blinkLED(int count) {
  for (int i=0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(120);
    digitalWrite(LED_BUILTIN, LOW);
    delay(120);
  }
}
