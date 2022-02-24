// WiFi
#define SECRET_SSID "ssid"
#define SECRET_PASS "pass"

// MQTT
#define MQTT_HOST "ip or hostname"
#define MQTT_PORT port_number
#define MQTT_USER ""
#define MQTT_PASS ""
#define MQTT_ROOT_TOPIC "environment"
#define MQTT_DEVICE_NAME "pws"
#define MQTT_DEVICE_NAME_TO_HOSTNAME true // autoset device name to hostname

// Debug
#define DEBUG_MQTT_MSGS true
#define DEBUG_SENSORS false

// Other
#define WIND_DIR_CORRECTION 60; // correction factor for real wind direction
#define WIND_AVERAGING_TIME 10000 // averaging time for wind  in ms
#define CLOUDS_AVERAGING_TIME 5000 // averaging time for wind  in ms
