#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "RTClib.h"
#include "Adafruit_FXAS21002C.h"
#include "Adafruit_FXOS8700.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TCS34725.h"
#include "Adafruit_SI1145.h"

RTC_DS3231 rtc;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BME280 bme;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_SI1145 uv = Adafruit_SI1145();

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
#define SEALEVELPRESSURE_HPA (1013.25)

void setup() {
  Serial.begin(9600);
  
  while (!Serial) {
    delay(1);
  }

  if (! rtc.begin()) {
    Serial.println("No RTC found");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!gyro.begin()) {
    Serial.println("No FXAS21002C found");
    while(1);
  }

  if (!accelmag.begin(ACCEL_RANGE_4G)) {
    Serial.println("No FXOS8700 found");
    while(1);
  }

  if (!bme.begin()) {
    Serial.println("No BME280 found");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    while(1);
  }

  if (!tcs.begin()) {
    Serial.println("No TCS34725 found");
    while(1);
  }

  if (!uv.begin()) {
    Serial.println("No Si1145 found");
    while(1);
  }
}

void loop() {
  Serial.println("===================");
  
  // RTC
  DateTime now = rtc.now();
  Serial.print("RTC  ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  // Gyro
  sensors_event_t event;
  gyro.getEvent(&event);
  Serial.print("Gyro ");
  Serial.print("X: ");
  Serial.print(event.gyro.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(event.gyro.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(event.gyro.z);
  Serial.print("  ");
  Serial.print("rad/s ");
  Serial.println();

  // Accelerometer
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);
  Serial.print("Acc  ");
  Serial.print("X: ");
  Serial.print(aevent.acceleration.x, 4);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(aevent.acceleration.y, 4);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(aevent.acceleration.z, 4);
  Serial.print("  ");
  Serial.print("m/s^2");
  Serial.println();

  // Magnetometer
  Serial.print("Mag  ");
  Serial.print("X: ");
  Serial.print(mevent.magnetic.x, 1);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(mevent.magnetic.y, 1);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(mevent.magnetic.z, 1);
  Serial.print("  ");
  Serial.print("uT");
  Serial.println();

  // Air Pressure/temperature/humidity  
  Serial.print("Air  ");
  Serial.print("*C: ");
  Serial.print(bme.readTemperature());
  Serial.print("  ");
  Serial.print("hPa: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.print("  ");
  Serial.print("Alt: ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print("m  ");
  Serial.print("Humid: ");
  Serial.print(bme.readHumidity());
  Serial.print("%");
  Serial.println();

  // Color (wind direction)
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.setInterrupt(0);
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  tcs.setInterrupt(1);
  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print("  ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print("  ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print("  ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print("  ");
  Serial.println();

  // Light
  Serial.print("Vis: "); Serial.print(uv.readVisible()); Serial.print("  ");
  Serial.print("IR: "); Serial.print(uv.readIR()); Serial.print("  ");
  float UVindex = uv.readUV();
  UVindex /= 100.0;  
  Serial.print("UV: "); Serial.print(UVindex); Serial.print("  ");
  Serial.println();
  
  Serial.println();
  delay(5000);
}
