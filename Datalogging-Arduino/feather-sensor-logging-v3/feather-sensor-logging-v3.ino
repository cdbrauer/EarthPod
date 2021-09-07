#include "Arduino.h"
#include <Adafruit_SleepyDog.h>
#include "Wire.h"
#include "SPI.h"
#include "SD.h"
#include "Adafruit_Sensor.h"
#include "RTClib.h"                         // RTC (0x68)
#include "BH1750.h"                         // Light level (0x23)

// This library is in the libraries folder on the GitHub repo
// The library with this name in the Arduino library manager is different and will NOT work
#include "SHT2x.h"                          // Humidity and temperature (0x40)

#include "Adafruit_BMP280.h"                // Air pressure, altitude, and temperature (0x76, 0x77)
// SPL06-007 is manually accessed if used   // Air pressure, altitude, and temperature (0x76, 0x77)
#include "Adafruit_FXOS8700.h"              // Accelerometer and magnetometer (0x1F)
#include "Adafruit_FXAS21002C.h"            // Gyroscope (0x21)
#include "WiFi101.h"
#include "secrets.h"
#include "ThingSpeak.h"
#include "Adafruit_AM2315.h"

// Function prototypes
void blink_led(uint16_t delayTime = 500); // Warning - sensor malfunction, low battery
void enable_led(uint16_t delayTime = 1000); // Logging halted - dead battery, no SD card, no RTC
void log_info(String msg);
void init_SPL06_007(void);
void read_SPL06_007(void);

// Settings
#define LED_ERROR 13
#define SAMPLE_INTERVAL_MS 60000
#define FILE_BASE_NAME "Data"
#define HEADINGS "ID,Year,Month,Day,Hour,Minute,Second,Time Step,Light (BH1750),Humidity,Pressure,Altitude,Temp (DS3231),Temp (SHT21),Temp (BMP280/SPL06),Accel X,Accel Y,Accel Z,Mag X,Mag Y,Mag Z,Gyro X,Gyro Y,Gyro Z,Battery,Moisture (2cm),Moisture (5cm),Temp (2cm),Temp (5cm),Light,Temp (SRH 1),Humidity (SRH1),Temp (SRH2),Humidity (SRH2)"
#define VBATPIN A7
#define LOWBATTERY 3.6

// Addresses
const int ADDR_PRESSURE_BMP = 0x76;
const int ADDR_PRESSURE_SPL = 0x77;
const int ADDR_LIGHT = 0x23;
const int ADDR_ACCEL = 0x1F;
const int ADDR_GYRO = 0x21;

// Soil sensor pins
#define SOIL_2CM A0
#define SOIL_5CM A1
#define TEMP_2CM A4
#define TEMP_5CM A2
#define LIGHT_OUT A5

// SD card settings
bool sd_present = false;

// V1
//#define CHIP_DETECT 7
//#define CHIP_SELECT 4
//#define LED_WRITE 8

// V2
//#define CHIP_DETECT A4
//#define CHIP_SELECT A5
//#define LED_WRITE -1

// V3
#define CHIP_DETECT -1
#define CHIP_SELECT 12
#define LED_WRITE -1

// Sensor status
bool BH1750_present = true;
bool BMP280_present = true;
bool SPL06_present = false;
bool FXOS8700_present = true;
bool FXAS21002C_present = true;
bool AM2315_0_present = true;

// Pressure sensor addresses
const byte REG_PSR       = 0x00;    // Register Address: Pressure Value (3 bytes)
const byte REG_TMP       = 0x03;    // Register Address: Temperature Value (3 bytes)
const byte REG_PRS_CFG   = 0x06;    // Register Address: Pressure configuration (1 byte)
const byte REG_TMP_CFG   = 0x07;    // Register Address: Temperature configuration (1 byte)
const byte REG_MEAS_CFG  = 0x08;    // Register Address: Measurement configuration (1 byte)
const byte REG_CFG       = 0x09;    // Register Address: Interrupt and FIFO configuration (1 byte)
const byte REG_ID        = 0x0D;    // Register Address: Device ID (1 byte)
const byte REG_COEF      = 0x10;    // Register Address: Calibration coefficients (18 bytes)

// Pressure sensor calibration coefficients
int16_t c0, c1;
int32_t c00, c10;
int16_t c01, c11, c20, c21, c30;

// Pressure sensor scaling factors
float kt;
float kp;
static float scale_factor(int oversampling_rate){
    float k;
    switch (oversampling_rate) {
        case 1:
            k = 524288.0f;
        break;

        case 2:
            k = 1572864.0f;
        break;

        case 4:
            k = 3670016.0f;
        break;

        case 8:
            k = 7864320.0f;
        break;

        case 16:
            k = 253952.0f;
        break;

        case 32:
            k = 516096.0f;
        break;

        case 64:
            k = 1040384.0f;
        break;

        case 128:
            k = 2088960.0f;
        break;
    }
    return k;
}
 
// Tropospheric properties (0-11km) for standard atmosphere
const float T1 = 15.0f + 273.15f;       // temperature at base height in Kelvin
const float a  = -6.5f / 1000.0f;       // temperature gradient in degrees per metre
const float g  = 9.80665f;              // gravity constant in m/s/s
const float R  = 287.05f;               // ideal gas constant in J/kg/K
const float msl_pressure = 101325.0f;   // in Pa

// Light sensor results
float lux = 0;

// Pressure sensor results
float bmp_spl_temperature = 25;
float bmp_spl_pressure = 101325;
float bmp_spl_altitude = 0;

// File variables
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char filename[13] = FILE_BASE_NAME "00.csv";
char logfilename[8] = "log.txt";

// Measurement time and number
double t0 = 0;
double counter = 0;

// Battery voltage variable
float measuredvbat;

// Create sensor objects
RTC_DS3231 rtc;
BH1750 lightMeter(ADDR_LIGHT);
Adafruit_BMP280 bmp;
Adafruit_FXOS8700 accelmag(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro(0x0021002C);
WiFiClient client;
Adafruit_AM2315 AM2315_0;

// Wifi channel fields
unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the WiFi radio's status

// Analog sensor variables
int soil_moisture_2cm = 0;
int soil_moisture_5cm = 0;
int soil_temp_2cm = 0;
int soil_temp_5cm = 0;
int light_analog = 0;

// RH Soil sensor varaibles
float AM2315_temp0, AM2315_hum0;

void setup() {
  if (LED_ERROR >= 0) {
    pinMode(LED_ERROR, OUTPUT);
    digitalWrite(LED_ERROR, LOW);
  }
  
  if (LED_WRITE >= 0) {
    pinMode(LED_WRITE, OUTPUT);
    digitalWrite(LED_WRITE, LOW);
  }
  
  analogReadResolution(12);
  
  WiFi.setPins(8,7,4,2); // feather M0 wifi module pins 
  WiFi.lowPowerMode();
  Wire.begin();
  
  ThingSpeak.begin(client);
  
  Serial.begin(9600);
  delay(2000);
  
  // Check the Battery Voltage ////////////////////////////////////////////////////////////
  measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 4096; // convert to voltage
  log_info("Battery Voltage: " );
  log_info(measuredvbat);

  if (measuredvbat <= LOWBATTERY) {
    log_info("Low battery, halting");
    Serial.flush();
    while(1){enable_led();}
  }

  // Initialize the SD card ///////////////////////////////////////////////////////////////  
  if (CHIP_DETECT >= 0) {
    pinMode(CHIP_DETECT, INPUT);
    if (!digitalRead(CHIP_DETECT)) {
      sd_present = false;
      log_info("SD card not present");
    }
  }

  // Try to initialize even if card is not detected
  if (!SD.begin(CHIP_SELECT)) {
    sd_present = false;
    if (LED_ERROR >= 0) {digitalWrite(LED_ERROR, HIGH);}
    log_info("SD card failed");
  } else {
    sd_present = true;
    log_info("------");
    log_info("SD card initialized");
  }

  // Initialize sensors ///////////////////////////////////////////////////////////////////
  if (!rtc.begin()) {
    log_info("RTC not found, halting");
    Serial.flush();
    while(1){enable_led();}
  }

  if (rtc.lostPower()) {
    log_info("RTC lost power, setting time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  char dt[16];
  char tm[16];
  DateTime now = rtc.now();
  sprintf(dt, "%02d/%02d/%02d", now.year(),now.month(),now.day());
  sprintf(tm, "%02d:%02d:%02d", now.hour(),now.minute(),now.second());
  log_info(dt);
  log_info(tm);

  delay(1000);
  
  log_info("Sensor Soil RH Test 1");
  if (!AM2315_0.begin()) {
     log_info("Sensor Soil RH not found, check wiring & pullups!");
     AM2315_0_present = false;
  }
  else log_info("Soil RH Sensor 1 Found");
  
  if (lightMeter.begin()) {
    log_info("BH1750 connected");
  } else {
    BH1750_present = false;
    log_info("Light sensor not found");
  }

  if (!bmp.begin(ADDR_PRESSURE_BMP)) {
    log_info("BMP280 not found, searching for SPL06_007");
    BMP280_present = false;
    SPL06_present = true;
    init_SPL06_007();
  } else {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }

  if (BMP280_present) {
    log_info("BMP280 connected");
  } else if (SPL06_present) {
    log_info("SPL06_007 connected");
  } else {
    log_info("Pressure sensor not found");
  }

  if (accelmag.begin(ADDR_ACCEL)) {
    log_info("FXOS8700 connected");
  } else {
    FXOS8700_present = false;
    log_info("FXOS8700 not found");
  }

  if (gyro.begin(ADDR_GYRO)) {
    log_info("FXAS21002C connected");
  } else {
    FXAS21002C_present = false;
    log_info("FXAS21002C not found");
  }

  // Create sensor data file //////////////////////////////////////////////////////////////
  if (sd_present) {
    // Find an unused file name.
    if (BASE_NAME_SIZE > 6) {
      log_info("FILE_BASE_NAME too long");
    }
    while (SD.exists(filename)) {
      if (filename[BASE_NAME_SIZE + 1] != '9') {
        filename[BASE_NAME_SIZE + 1]++;
      } else if (filename[BASE_NAME_SIZE] != '9') {
        filename[BASE_NAME_SIZE + 1] = '0';
        filename[BASE_NAME_SIZE]++;
      } else {
        log_info("Can't create file name");
        sd_present = false;
      }
    }
  }
  
  if (sd_present) {
    File file = SD.open(filename, FILE_WRITE);  
    file.println(HEADINGS);
    file.close();
    log_info("Sensor data file created");
    log_info(filename);
  }

  // check for the presence of the wifi module:
  if (WiFi.status() == WL_NO_SHIELD) {
    log_info("WiFi shield not present, halting");
    Serial.flush();
    while(1){enable_led();}
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    log_info("Attempting to connect to WPA SSID: ");
    log_info(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  log_info("You're connected to the network");
  
  // Get start time ///////////////////////////////////////////////////////////////////////
  t0 = millis();
  int wdtCountdown = Watchdog.enable(2000);
}

void loop() {  
  Watchdog.reset();
  Serial.println("===================");
  
  // RTC
  char dt[16];
  char tm[16];
  DateTime now = rtc.now();
  sprintf(dt, "%02d/%02d/%02d", now.year(),now.month(),now.day());
  sprintf(tm, "%02d:%02d:%02d", now.hour(),now.minute(),now.second());
  
  Serial.print("RTC: ");
  Serial.print(dt);
  Serial.print(' ');
  Serial.print(tm);
  Serial.println();

  // Time step
  double ts = (millis() - t0)/1000.0;
  t0 = millis();
  Serial.print("Time Step: ");
  Serial.println(String(ts));

  // Light level
  if (BH1750_present) {
    lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.println(lux);
  }

  // Humidity
  float rh = SHT2x.GetHumidity();
  Serial.print("Ambient Humidity(%RH): ");
  Serial.println(rh);

  // Analog signals
  soil_moisture_2cm = analogRead(SOIL_2CM);
  soil_moisture_5cm = analogRead(SOIL_5CM);
  soil_temp_2cm = analogRead(TEMP_2CM);
  soil_temp_5cm = analogRead(TEMP_5CM);
  light_analog = analogRead(LIGHT_OUT);
  
  Serial.print("Soil Moisture Readings: ");
  Serial.print(soil_moisture_2cm);
  Serial.print(", ");
  Serial.println(soil_moisture_5cm);

  Serial.print("Soil Temp Readings: ");
  Serial.print(soil_temp_2cm);
  Serial.print(", ");
  Serial.println(soil_temp_5cm);  

  Serial.print("Light Reading: ");
  Serial.println(light_analog);

  // Pressure and altitude
  if (BMP280_present) {
    bmp_spl_temperature = bmp.readTemperature();
    bmp_spl_pressure = bmp.readPressure();
    bmp_spl_altitude = bmp.readAltitude(1013.25);
  } else if (SPL06_present) {
    read_SPL06_007();
  }

  // Temperature
  float temp_ds3231 = rtc.getTemperature();
  float temp_sht21 = SHT2x.GetTemperature();  
  Serial.println("Temperature(C)");
  Serial.print("     DS3231: ");
  Serial.println(temp_ds3231);
  Serial.print("     SHT21: ");
  Serial.println(temp_sht21);

  if (BMP280_present || SPL06_present) {
    Serial.print("     BMP280/SPL06: ");
    Serial.println(bmp_spl_temperature);    
    Serial.print("Pressure: ");
    Serial.println(bmp_spl_pressure, DEC);
    Serial.print("Altitude: ");
    Serial.println(bmp_spl_altitude, DEC);
  }

  // RH Soil Sensors
  if (AM2315_0_present == true) {
    if (! AM2315_0.readTemperatureAndHumidity(&AM2315_temp0, &AM2315_hum0)) {
      Serial.println("Failed to read data from AM2315-0");
    }
    else {
      Serial.print("AM2315 0 soil Temp *C: "); Serial.println(AM2315_temp0);
      Serial.print("AM2315 0 soil Hum %: "); Serial.println(AM2315_hum0);
    }
  }

  // Battery Voltage
  measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 4096; // convert to voltage
  Serial.print("Battery Voltage: " );
  Serial.println(measuredvbat);
  
  // SD Card
  Serial.print("SD Card Status: " );
  Serial.println(sd_present);

  if(sd_present){
    // Open file
    if (LED_WRITE >= 0) {digitalWrite(LED_WRITE, HIGH);}
    File file = SD.open(filename, FILE_WRITE);
    file.seek(EOF);
    Serial.print("Filename: " );
    Serial.println(filename);

    // Trial number
    file.print(String(counter));
    file.print(",");

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

    // Time Step
    file.print(String(ts));
    file.print(",");

    // Light Level
    if (BH1750_present) {
      file.print(String(lux));
      file.print(",");
    } else {
      file.print(",");
    }    

    // Humidity
    file.print(String(rh));
    file.print(",");

    // Pressure and Altitude
    if (BMP280_present || SPL06_present) {
      file.print(String(bmp_spl_pressure));
      file.print(",");
      file.print(String(bmp_spl_altitude));
      file.print(",");
    } else {
      file.print(",,");
    }

    // Temperature
    file.print(String(temp_ds3231));
    file.print(",");
    file.print(String(temp_sht21));
    file.print(",");

    if (BMP280_present || SPL06_present) {
      file.print(String(bmp_spl_temperature));
      file.print(",");
    } else {
      file.print(",");
    }

    // Accelerometer/Magnetometer
    if (FXOS8700_present) {
      sensors_event_t aevent, mevent;
      accelmag.getEvent(&aevent, &mevent);
      file.print(String(aevent.acceleration.x));
      file.print(",");
      file.print(String(aevent.acceleration.y));
      file.print(",");
      file.print(String(aevent.acceleration.z));
      file.print(",");

      file.print(String(mevent.magnetic.x));
      file.print(",");
      file.print(String(mevent.magnetic.y));
      file.print(",");
      file.print(String(mevent.magnetic.z));
      file.print(",");
    } else {
      file.print(",,,,,,");
    }
  
    // Gyro
    if (FXAS21002C_present) {
      sensors_event_t event;
      gyro.getEvent(&event);
      file.print(String(event.gyro.x));
      file.print(",");
      file.print(String(event.gyro.y));
      file.print(",");
      file.print(String(event.gyro.z));
      file.print(",");
    } else {
      file.print(",,,");
    }

    // Battery Voltage
    file.print(String(measuredvbat));
    file.print(",");

    // Soil Mositure & Temperature
    file.print(String(soil_moisture_2cm));
    file.print(",");
    file.print(String(soil_moisture_5cm));
    file.print(",");
    file.print(String(soil_temp_2cm));
    file.print(",");
    file.print(String(soil_temp_5cm));
    file.print(",");
    
    // Light
    file.print(String(light_analog));
    file.print(",");

    // Relative soil humidity sensor data
    if (AM2315_0_present == true) {
      file.print(String(AM2315_temp0));
      file.print(",");
      file.print(String(AM2315_hum0));
      file.print(",");
    }
    else file.print(",,"); 
    
    file.println();
    file.close();
    if (LED_WRITE >= 0) {digitalWrite(LED_WRITE, LOW);}
  }

  
/*  
// Upload to ThingSpeak
  ThingSpeak.setField(1, temp_ds3231);
  ThingSpeak.setField(2, temp_sht21);
  ThingSpeak.setField(3, rh);
  ThingSpeak.setField(4, measuredvbat);

  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
*/

  Serial.println();
  counter++;

  while((millis() - t0) < SAMPLE_INTERVAL_MS){
    Watchdog.reset();
    
    if (!sd_present) {
      enable_led(); // If the SD card is missing or failed to initialize, turn the status led on
    } else if (measuredvbat <= LOWBATTERY) {
      blink_led(); // If the battery is low, make the status led blink
    } else if ((!SPL06_present) && (!BMP280_present)) {
      blink_led(); // If the pressure sensor is missing or failed to initialize, make the status led blink
    } else {
      delay(1000);
    }

    if (Serial.available() > 0) {
      char receivedChar = Serial.read();
      if (receivedChar == 'c') {
        log_info("c -> Clock reset requested, setting time");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
        char dt[16];
        char tm[16];
        DateTime now = rtc.now();
        sprintf(dt, "%02d/%02d/%02d", now.year(),now.month(),now.day());
        sprintf(tm, "%02d:%02d:%02d", now.hour(),now.minute(),now.second());
        log_info(dt);
        log_info(tm);
      } else if (receivedChar == 's') {
        log_info("s -> Manual sample requested");
        break; // Skip waiting for sample interval
      } else if (receivedChar == 'r') {
        log_info("r -> Reset requested");
        Serial.flush();
        while(1){enable_led();} // Allow watchdog timer to expire
      }
    }
  }
}

void blink_led(uint16_t delayTime) {
  if (LED_ERROR >= 0) {
    digitalWrite(LED_ERROR, HIGH); // turn the LED on
    delay(delayTime);
    digitalWrite(LED_ERROR, LOW);  // turn the LED off
    delay(delayTime);
  } else {
    delay(delayTime * 2);
  }
}

void enable_led(uint16_t delayTime) {
  if (LED_ERROR >= 0) {
    digitalWrite(LED_ERROR, HIGH); // turn the LED on
    delay(delayTime);
  } else {
    delay(delayTime);
  }
}

void log_info(String msg) {
  Serial.println(msg);
  
  if (sd_present) {
    if (LED_WRITE >= 0) {digitalWrite(LED_WRITE, HIGH);}
    File logfile = SD.open(logfilename, FILE_WRITE);
    logfile.seek(EOF);
    logfile.println(msg);
    logfile.close();
    if (LED_WRITE >= 0) {digitalWrite(LED_WRITE, LOW);}
  }
}

void init_SPL06_007() {
  // Try to start pressure sensor
  Serial.println("Initializing pressure sensor...");
  uint8_t tries = 5;
  while(tries > 0){
      // Check chip ID
      Wire.beginTransmission (ADDR_PRESSURE_SPL);
      Wire.write             (REG_ID);
      Wire.endTransmission   ();
      Wire.requestFrom       (ADDR_PRESSURE_SPL, 1);
      byte device_id = Wire.read();
      Serial.print("Device ID: ");
      Serial.println(device_id, BIN);
      
      if (device_id == 0x10) {
          // Check if sensors are ready
          Wire.beginTransmission (ADDR_PRESSURE_SPL);
          Wire.write             (REG_MEAS_CFG);
          Wire.endTransmission   ();
          Wire.requestFrom       (ADDR_PRESSURE_SPL, 1);        
          byte meas_cfg = Wire.read();
          Serial.print("Measurement configuration: ");       
          Serial.println(meas_cfg, BIN);
          
          if (meas_cfg & (1 << 7) && meas_cfg & (1 << 6)) {
              break;
          }
      }

      tries = tries - 1;
      delay(1000);
  }

  // If initialization failed
  if (!tries) {
      Serial.println("Could not initialize pressure sensor");
      SPL06_present = false;
  }

  if (SPL06_present) {
    // Read calibration coefficients
    Wire.beginTransmission (ADDR_PRESSURE_SPL);
    Wire.write             (REG_COEF);
    Wire.endTransmission   ();
    Wire.requestFrom       (ADDR_PRESSURE_SPL, 18);
  
    byte coef_bytes[18];
    for(uint8_t i = 0; i < 18; i++){
      coef_bytes[i] = Wire.read();
    }
  
    c0 = (uint16_t)coef_bytes[0] << 4 | (uint16_t)coef_bytes[1] >> 4;
    c0 = (c0 & 1 << 11) ? (0xf000 | c0) : c0;
  
    c1 = (uint16_t)(coef_bytes[1] & 0x0f) << 8 | (uint16_t)coef_bytes[2];
    c1 = (c1 & 1 << 11) ? (0xf000 | c1) : c1;
  
    c00 = (uint32_t)coef_bytes[3] << 12 | (uint32_t)coef_bytes[4] << 4 | (uint16_t)coef_bytes[5] >> 4;
    c00 = (c00 & 1 << 19) ? (0xfff00000 | c00) : c00;
  
    c10 = (uint32_t)(coef_bytes[5] & 0x0f) << 16 | (uint32_t)coef_bytes[6] << 8 | (uint32_t)coef_bytes[7];
    c10 = (c10 & 1 << 19) ? (0xfff00000 | c10) : c10;
  
    c01 = (uint16_t)coef_bytes[8] << 8 | coef_bytes[9];
    c11 = (uint16_t)coef_bytes[10] << 8 | coef_bytes[11];
    c20 = (uint16_t)coef_bytes[12] << 8 | coef_bytes[13];
    c21 = (uint16_t)coef_bytes[14] << 8 | coef_bytes[15];
    c30 = (uint16_t)coef_bytes[16] << 8 | coef_bytes[17];
    
    Serial.println("----------");
    Serial.println("Calibration coeffecients: ");
    Serial.print("C0: ");       
    Serial.println(c0, DEC);
    Serial.print("C1: ");       
    Serial.println(c1, DEC);
    Serial.print("C00: ");       
    Serial.println(c00, DEC);
    Serial.print("C10: ");       
    Serial.println(c10, DEC);
    Serial.print("C01: ");       
    Serial.println(c01, DEC);
    Serial.print("C11: ");       
    Serial.println(c11, DEC);
    Serial.print("C20: ");       
    Serial.println(c20, DEC);
    Serial.print("C21: ");       
    Serial.println(c21, DEC);
    Serial.print("C30: ");       
    Serial.println(c30, DEC);  
    Serial.println("----------");
  
    // Write configuration settings
  
    // configuration of pressure measurement rate (PM_RATE) and resolution (PM_PRC)
    //
    // bit[7]: reserved
    //
    // PM_RATE[6:4]    : 0 | 1 | 2 | 3 | 4  | 5  | 6  | 7
    // measurement rate: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128
    // note: applicable for measurements in background mode only
    //
    // PM_PRC[3:0]         : 0      | 1   | 2   | 3    | 4    | 5    | 6     | 7
    // oversampling (times): single | 2   | 4   | 8    | 16   | 32   | 64    | 128
    // measurement time(ms): 3.6    | 5.2 | 8.4 | 14.8 | 27.6 | 53.2 | 104.4 | 206.8
    // precision(PaRMS)    : 5.0    |     | 2.5 |      | 1.2  | 0.9  | 0.5   |
    // note: use in combination with a bit shift when the oversampling rate is > 8 times. see CFG_REG(0x19) register
    Wire.beginTransmission (ADDR_PRESSURE_SPL);
    Wire.write             (REG_PRS_CFG);
    Wire.write             (0 << 4 | 4);
    Wire.endTransmission   ();
  
    // configuration of temperature measurment rate (TMP_RATE) and resolution (TMP_PRC)
    //
    // temperature measurement: internal sensor (in ASIC) | external sensor (in pressure sensor MEMS element)
    // TMP_EXT[7]             : 0                         | 1
    // note: it is highly recommended to use the same temperature sensor as the source of the calibration coefficients wihch can be read from reg 0x28
    //
    // TMP_RATE[6:4]   : 0 | 1 | 2 | 3 | 4  | 5  | 6  | 7
    // measurement rate: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128
    // note: applicable for measurements in background mode only
    //
    // bit[3]: reserved
    //
    // TMP_PRC[2:0]        : 0      | 1 | 2 | 3 | 4  | 5  | 6  | 7
    // oversampling (times): single | 2 | 4 | 8 | 16 | 32 | 64 | 128
    // note: single(default) measurement time 3.6ms, other settings are optional, and may not be relevant
    // note: use in combination with a bit shift when the oversampling rate is > 8 times. see CFG_REG(0x19) register
    Wire.beginTransmission (ADDR_PRESSURE_SPL);
    Wire.write             (REG_TMP_CFG);
    Wire.write             (1 << 7 | 0 << 4 | 3);
    Wire.endTransmission   ();
  
    // sensor operating mode
    // measurement mode: stop meas |  command mode(single) |    na |            background mode(continuous) |
    // measurement type:      idle | pres meas | temp meas |    na | pres meas | temp meas | pres&temp meas |
    // MEAS_CTRL[2:0]  :         0 |         1 |         2 | 3 | 4 |         5 |         6 |              7 |
    Wire.beginTransmission (ADDR_PRESSURE_SPL);
    Wire.write             (REG_MEAS_CFG);
    Wire.write             (7);
    Wire.endTransmission   ();  
  
    // diasble all interrupts and FIFO
    //
    // bit7: interrupt(on SDO pin) active level
    //       0-active low  1-active hight
    //
    // bit6: set to '1' for FIFO full interrupt
    // bit5: set to '1' for pressure measurement ready interrupt
    // bit4: set to '1' for temperature measurement ready interrupt
    //
    // note: bit3 must be set to '1' when the temperature oversampling rate is > 8 times
    // note: bit2 must be set to '1' when the pressure oversampling rate is > 8 times
    //
    // bit1: set to '1' for FIFO enable
    //
    // bit0: set to '0' for 4-wire interface
    //       set to '1' for 3-wire interface
    Wire.beginTransmission (ADDR_PRESSURE_SPL);
    Wire.write             (REG_CFG);
    Wire.write             (1 << 2);
    Wire.endTransmission   ();
    
    // compensation scale factors
    // oversampling rate  : single | 2       | 4       | 8       | 16     | 32     | 64      | 128
    // scale factor(KP/KT): 524288 | 1572864 | 3670016 | 7864320 | 253952 | 516096 | 1040384 | 2088960  
    kp = scale_factor(16);
    kt = scale_factor(8);
  }
}

void read_SPL06_007() {
  // Read temperature
  Wire.beginTransmission (ADDR_PRESSURE_SPL);
  Wire.write             (REG_TMP);
  Wire.endTransmission   ();
  Wire.requestFrom       (ADDR_PRESSURE_SPL, 3);

  byte tmp_bytes[3];
  for(uint8_t i = 0; i < 3; i++){
    tmp_bytes[i] = Wire.read();
  }
  
  int32_t tmp_raw = (uint32_t)tmp_bytes[0] << 16 | (uint32_t)tmp_bytes[1] << 8 | (uint32_t)tmp_bytes[2];
  tmp_raw = (tmp_raw & 1 << 23) ? (0xff000000 | tmp_raw) : tmp_raw;

  float ftsc = (float)tmp_raw / kt;
  bmp_spl_temperature = (float)c0 * 0.5f + (float)c1 * ftsc;

  // Read pressure
  Wire.beginTransmission (ADDR_PRESSURE_SPL);
  Wire.write             (REG_PSR);
  Wire.endTransmission   ();
  Wire.requestFrom       (ADDR_PRESSURE_SPL, 3);

  byte psr_bytes[3];
  for(uint8_t i = 0; i < 3; i++){
    psr_bytes[i] = Wire.read();
  }
  
  int32_t psr_raw = (uint32_t)psr_bytes[0] << 16 | (uint32_t)psr_bytes[1] << 8 | (uint32_t)psr_bytes[2];
  psr_raw = (psr_raw & 1 << 23) ? (0xff000000 | psr_raw) : psr_raw;
  
  float fpsc = (float)psr_raw / kp;
  float qua2 = (float)c10 + fpsc * ((float)c20 + fpsc * (float)c30);
  float qua3 = ftsc * fpsc * ((float)c11 + fpsc * (float)c21);
  float fp = (float)c00 + fpsc * qua2 + ftsc * (float)c01 + qua3;
  bmp_spl_pressure = fp;

  // Calculate altitude
  float pK = fp / msl_pressure;
  bmp_spl_altitude = (((powf(pK, (-(a * R) / g))) * T1) - T1) / a;
}
