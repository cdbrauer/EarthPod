#include "Wire.h"
#include "SPI.h"
#include "SdFat.h"
#include "Adafruit_Sensor.h"
#include "RTClib.h"
// #include "Adafruit_FXAS21002C.h"
// #include "Adafruit_FXOS8700.h"
#include "Adafruit_BME280.h"
// #include "Adafruit_TCS34725.h"
#include "Adafruit_SI1145.h"

#define CHIP_SELECT 4
#define FILE_BASE_NAME "Data"
#define error(msg) sd.errorHalt(F(msg))

#define SEALEVELPRESSURE_HPA (1013.25)
// #define HEADINGS "year,month,day,hour,minute,second,gyro x,gyro y,gyro z,accel x, accel y, accel z,mag x, mag y, mag z,temperature,pressure,altitude,humidity,r,g,b,vis,ir,uv"
#define HEADINGS "year,month,day,hour,minute,second,temperature,pressure,altitude,humidity,vis,ir,uv"
#define SAMPLE_INTERVAL_MS 5000

SdFat sd;
SdFile file;
RTC_DS3231 rtc;
// Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
// Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BME280 bme;
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_SI1145 uv = Adafruit_SI1145();

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.csv";

void setup() {  
  Serial.begin(9600);
  
  while (!Serial) {
    delay(1);
  }

  // Initialize the SD card ///////////////////////////////////////////////////////////////
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(CHIP_SELECT, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {error("open");}

  Serial.print(F("Logging to: "));
  Serial.println(fileName);

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

  /*if (!gyro.begin()) {
    Serial.println("No FXAS21002C found");
    while(1);
  }

  if (!accelmag.begin(ACCEL_RANGE_4G)) {
    Serial.println("No FXOS8700 found");
    while(1);
  }*/

  if (!bme.begin()) {
    Serial.println("No BME280 found");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    while(1);
  }

  /*if (!tcs.begin()) {
    Serial.println("No TCS34725 found");
    while(1);
  }*/

  if (!uv.begin()) {
    Serial.println("No Si1145 found");
    while(1);
  }
  
  file.println(HEADINGS);
  if (!file.close()) error("close");
}

void loop() {
  if (!file.open(fileName, O_CREAT | O_APPEND | O_WRITE)) error("open");

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

  // Gyro
  /*sensors_event_t event;
  gyro.getEvent(&event);
  file.print(String(event.gyro.x));
  file.print(",");
  file.print(String(event.gyro.y));
  file.print(",");
  file.print(String(event.gyro.z));
  file.print(",");*/

  // Accelerometer
  /*sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);
  file.print(String(aevent.acceleration.x));
  file.print(",");
  file.print(String(aevent.acceleration.y));
  file.print(",");
  file.print(String(aevent.acceleration.z));
  file.print(",");*/

  // Magnetometer
  /*file.print(String(mevent.magnetic.x));
  file.print(",");
  file.print(String(mevent.magnetic.y));
  file.print(",");
  file.print(String(mevent.magnetic.z));
  file.print(",");*/

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
  /*uint16_t r, g, b, c, colorTemp, lux;
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
  file.print(",");*/
  
  // Light
  file.print(String(uv.readVisible()));
  file.print(",");
  file.print(String(uv.readIR()));
  file.print(",");
  float UVindex = uv.readUV();
  UVindex /= 100.0;
  file.print(String(UVindex));
  
  file.println();
  if (!file.sync() || file.getWriteError()) {error("write");}
  if (!file.close()) error("close");

  Serial.println();
  delay(SAMPLE_INTERVAL_MS);
}
