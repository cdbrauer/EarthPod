#include "Wire.h"
#include "SPI.h"
#include "SD.h"
#include "Adafruit_Sensor.h"
#include "RTClib.h"
#include "Adafruit_FXAS21002C.h"
#include "Adafruit_FXOS8700.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TCS34725.h"
#include "Adafruit_SI1145.h"

#define CHIP_SELECT 4
#define FILE_BASE_NAME "Data"
#define INTERRUPT_PIN 0
#define ENABLE_PIN A5

#define SEALEVELPRESSURE_HPA (1013.25)
#define HEADINGS "year,month,day,hour,minute,second,time step,wind,gyro x,gyro y,gyro z,accel x,accel y,accel z,mag x,mag y,mag z,temperature,pressure,altitude,humidity,r,g,b,vis,ir,uv"
#define SAMPLE_INTERVAL_MS 5000

RTC_DS3231 rtc;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BME280 bme;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_SI1145 uv = Adafruit_SI1145();

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char filename[13] = FILE_BASE_NAME "00.csv";

double t0 = 0;
long counter = 0;

void setup() {  
  Serial.begin(9600);

  // Initialize the SD card ///////////////////////////////////////////////////////////////
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("Card initialized");

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    Serial.println("FILE_BASE_NAME too long");
  }
  while (SD.exists(filename)) {
    if (filename[BASE_NAME_SIZE + 1] != '9') {
      filename[BASE_NAME_SIZE + 1]++;
    } else if (filename[BASE_NAME_SIZE] != '9') {
      filename[BASE_NAME_SIZE + 1] = '0';
      filename[BASE_NAME_SIZE]++;
    } else {
      Serial.println("Can't create file name");
    }
  }

  Serial.print(F("Logging to: "));
  Serial.println(filename);

  // Initialize sensors ///////////////////////////////////////////////////////////////////
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

  // Start anemometer /////////////////////////////////////////////////////////////////////
  pinMode(ENABLE_PIN, INPUT);
  delay(5000);  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  delay(3000);
  pinMode(ENABLE_PIN, INPUT);
  
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), increment_counter, RISING);

  // Create file //////////////////////////////////////////////////////////////////////////
  File file = SD.open(filename, FILE_WRITE);  
  file.println(HEADINGS);
  file.close();

  // Get start time ///////////////////////////////////////////////////////////////////////
  t0 = millis();
}

void loop() {
  File file = SD.open(filename, FILE_WRITE);
  file.seek(EOF);

  Serial.println("===================");
  
  // RTC
  DateTime now = rtc.now();
  Serial.print("RTC  ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  
  // RTC
  file.print(String(now.year()));
  file.print(",");
  file.print(String(now.month()));
  file.print(",");
  file.print(String(now.day()));
  file.print(",");
  file.print(String(now.hour()));
  file.print(",");
  file.print(String(now.minute()));
  file.print(",");
  file.print(String(now.second()));
  file.print(",");

  // Time step
  double ts = (millis() - t0)/1000.0;
  Serial.print("Time Step  ");
  Serial.println(String(ts));
  file.print(String(ts));
  file.print(",");
  t0 = millis();

  // Wind speed
  file.print(String((counter/ts)*0.053));
  file.print(",");
  counter = 0;

  // Gyro
  sensors_event_t event;
  gyro.getEvent(&event);
  file.print(String(event.gyro.x));
  file.print(",");
  file.print(String(event.gyro.y));
  file.print(",");
  file.print(String(event.gyro.z));
  file.print(",");

  // Accelerometer
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);
  file.print(String(aevent.acceleration.x));
  file.print(",");
  file.print(String(aevent.acceleration.y));
  file.print(",");
  file.print(String(aevent.acceleration.z));
  file.print(",");

  // Magnetometer
  file.print(String(mevent.magnetic.x));
  file.print(",");
  file.print(String(mevent.magnetic.y));
  file.print(",");
  file.print(String(mevent.magnetic.z));
  file.print(",");

  // Air Pressure/temperature/humidity
  file.print(String(bme.readTemperature()));
  file.print(",");
  file.print(String(bme.readPressure() / 100.0F));
  file.print(",");
  file.print(String(bme.readAltitude(SEALEVELPRESSURE_HPA)));
  file.print(",");
  file.print(String(bme.readHumidity()));
  file.print(",");

  // Color (wind direction)
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.setInterrupt(0);
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  tcs.setInterrupt(1);
  file.print(String(r));
  file.print(",");
  file.print(String(g));
  file.print(",");
  file.print(String(b));
  file.print(",");
  
  // Light
  file.print(String(uv.readVisible()));
  file.print(",");
  file.print(String(uv.readIR()));
  file.print(",");
  float UVindex = uv.readUV();
  UVindex /= 100.0;
  file.print(String(UVindex));
  
  file.println();
  file.close();

  Serial.println();
  delay(SAMPLE_INTERVAL_MS);
}

void increment_counter() {
  counter++;
}
