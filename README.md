# EarthPod
Code for the [EarthPod](https://twitter.com/ASU_GDCS/status/1277711451856211975) environmental sensor pod.


## Datalogging-Arduino

### Hardware

The code in this folder is intended to be used in a sensor pod based on a Adafruit Feather M0 Adalogger. Connected sensors include:

| Sensor | [V1](https://github.com/cdbrauer/EarthPod/tree/master/Datalogging-Arduino/feather-sensor-logging) | [V2](https://github.com/cdbrauer/EarthPod/tree/master/Datalogging-Arduino/feather-sensor-logging-v2) |
| :------------- | :----------: | :-----------: |
| RTC | DS3231 | DS3231 |
| Gyroscope | FXAS21002C | FXAS21002C |
| Acclerometer/Magnetometer | FXOS8700 | FXOS8700 |
| Temperature/Humidity | si7021 | SHT21 |
| Air Pressure/Temperature | BME280 | SPL06-007 |
| Light Level | SI1145 | BH1750 |
| Wind Direction | TCS34725 | N/A |
| Wind Speed | Handheld Anemometer | N/A |

### feather-sensor-logging.ino

*Compatible with V1 only*

This program will create a .csv file on the Adalogger's SD card that records the measurements captured by the sensors. This program is only compatible with the Adafruit Feather M0 Adalogger, and will not fit on the Adafruit Feather 32u4 Adalogger due to memory constraints.

### feather-sensor-logging-v2.ino

*Compatible with V2 Only*

This program will create a .csv file on the Adalogger's SD card that records the measurements captured by the sensors. This program is only compatible with the Adafruit Feather M0 Adalogger, and will not fit on the Adafruit Feather 32u4 Adalogger due to memory constraints.

### feather-all-i2c.ino

*Compatible with V1 only*

This program will initialize all sensors and print readings to the serial terminal

### feather-anemometer.ino

Test initialization and interrupts for a wind speed sensor from a [handheld anemometer](https://www.amazon.com/Anemometer-Velocity-Measurement-Thermometer-Windsurfing/dp/B01JOTJMU6/).

## Datalogging-Python

### Hardware

The code in this folder is intended to be used in a sensor pod based on a Raspberry Pi or Google Coral. The primary
sensing hardware is the Coral Environmental Sensor Board, which provides ambient light, barometric pressure, humidity, and temperature measurements. This board is compatible with both the Google Coral and Raspberry Pi. Additional sensors can be connected to the Environmental Sensor Board through a series of Grove connectors.

### enviro\_logging.py

This program will create a .csv file that records the measurements captured by the sensors in the Environmental Sensor
Board.

### enviro\_logging\_wind.py

This program adds functionality for reading wind speed using the sensor from a [handheld anemometer](https://www.amazon.com/Anemometer-Velocity-Measurement-Thermometer-Windsurfing/dp/B01JOTJMU6/).

### enviro\_logging\_wind\_airflow.py

This program adds functionality for reading wind speed using a [F662 Board Mount Airflow Sensor](https://www.degreec.com/pages/f660-662) from Degree Controls.


## MagnetRelease

### MagnetRelease.ino

Arduino program for the twist-release magnet system used to deploy the pod.
