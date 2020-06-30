# EarthPod-Mini
Code for the EarthPod Mini environmental sensor pod.

## Hardware

The code in this project is intended to be used in a sensor pod based on a Raspberry Pi or Google Coral. The primary sensing hardware is the Coral Environmental Sensor Board, which provides ambient light, barometric pressure, humidity, and temperature measurements. This board is compatible with both Google Coral and Raspberry Pi. Additional sensors can be connected to the Environmental Sensor Board through a series of Grove connectors.

## Datalogging

### enviro_logging.py

This program will create a .csv file that records the measurements captured by the sensors in the Environmental Sensor Board.

### enviro_logging_wind.py

This program adds functionality for reading wind speed using the sensor from a handheld anemometer.

## MagnetRelease

### MagnetRelease.ino

Arduino program for the twist-release magnet system used to deploy the pod.