# pws-iot
Personal Weather Station IoT. It reads various environmental sensors and publishes values to MQTT server via wireless connection.

![alt img1](https://github.com/rkaczorek/pws-iot/raw/main/media/screenshot.png)

# Supported MCUs
- Arduino Nano 33 IoT
- Arduino MKR WiFi 1000
- Arduino MKR WiFi 1010
- Arduino Uno Wifi Rev2
- ESP8266

# Supported GPIO connected (A0, D10, D11) sensors
- Weather Meters (wind direction, wind speed, rainfall) (https://www.sparkfun.com/products/8942)

# Supported I2C sensors
- BME280 (temperature, humidity, pressure)
- MLX90614 (ambient temperature, object temperature)
- VEML7700 (light)
