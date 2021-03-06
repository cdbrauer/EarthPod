#include "Wire.h"

// Pressure sensor addresses
const int  SPL06_007_I2C = 0x77;    // I2C Address for the temperature sensor
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

// Tropospheric properties (0-11km) for standard atmosphere
const float T1 = 15.0f + 273.15f;       // temperature at base height in Kelvin
const float a  = -6.5f / 1000.0f;       // temperature gradient in degrees per metre
const float g  = 9.80665f;              // gravity constant in m/s/s
const float R  = 287.05f;               // ideal gas constant in J/kg/K
const float msl_pressure = 101325.0f;   // in Pa

// Pressure sensor results
float spl06_temperature = 25;
float spl06_pressure = 101325;
float spl06_altitude = 0;

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
 
void setup()
{
  Wire.begin();
  Serial.begin(9600); 
  delay(2000);

  init_SPL06_007();
}
 
void loop()
{   
  read_SPL06_007();

  // Print results
  Serial.print("Temperature: ");
  Serial.print(spl06_temperature, DEC);
  Serial.print(", Pressure: ");
  Serial.print(spl06_pressure, DEC);
  Serial.print(", Altitude: ");
  Serial.print(spl06_altitude, DEC);
  Serial.println("");
  
  delay(1000);
}
 
void init_SPL06_007() {
  // Try to start pressure sensor
  Serial.println("Initializing pressure sensor...");
  uint8_t tries = 5;
  while(tries > 0){
      // Check chip ID
      Wire.beginTransmission (SPL06_007_I2C);
      Wire.write             (REG_ID);
      Wire.endTransmission   ();
      Wire.requestFrom       (SPL06_007_I2C, 1);
      byte device_id = Wire.read();
      Serial.print("Device ID: ");
      Serial.println(device_id, BIN);
      
      if (device_id == 0x10) {
          // Check if sensors are ready
          Wire.beginTransmission (SPL06_007_I2C);
          Wire.write             (REG_MEAS_CFG);
          Wire.endTransmission   ();
          Wire.requestFrom       (SPL06_007_I2C, 1);        
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
      while(true){delay(1000);};
  }

  // Read calibration coefficients
  Wire.beginTransmission (SPL06_007_I2C);
  Wire.write             (REG_COEF);
  Wire.endTransmission   ();
  Wire.requestFrom       (SPL06_007_I2C, 18);

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
  Wire.beginTransmission (SPL06_007_I2C);
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
  Wire.beginTransmission (SPL06_007_I2C);
  Wire.write             (REG_TMP_CFG);
  Wire.write             (1 << 7 | 0 << 4 | 3);
  Wire.endTransmission   ();

  // sensor operating mode
  // measurement mode: stop meas |  command mode(single) |    na |            background mode(continuous) |
  // measurement type:      idle | pres meas | temp meas |    na | pres meas | temp meas | pres&temp meas |
  // MEAS_CTRL[2:0]  :         0 |         1 |         2 | 3 | 4 |         5 |         6 |              7 |
  Wire.beginTransmission (SPL06_007_I2C);
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
  Wire.beginTransmission (SPL06_007_I2C);
  Wire.write             (REG_CFG);
  Wire.write             (1 << 2);
  Wire.endTransmission   ();
  
  // compensation scale factors
  // oversampling rate  : single | 2       | 4       | 8       | 16     | 32     | 64      | 128
  // scale factor(KP/KT): 524288 | 1572864 | 3670016 | 7864320 | 253952 | 516096 | 1040384 | 2088960  
  kp = scale_factor(16);
  kt = scale_factor(8);
}

void read_SPL06_007() {
  // Read temperature
  Wire.beginTransmission (SPL06_007_I2C);
  Wire.write             (REG_TMP);
  Wire.endTransmission   ();
  Wire.requestFrom       (SPL06_007_I2C, 3);

  byte tmp_bytes[3];
  for(uint8_t i = 0; i < 3; i++){
    tmp_bytes[i] = Wire.read();
  }
  
  int32_t tmp_raw = (uint32_t)tmp_bytes[0] << 16 | (uint32_t)tmp_bytes[1] << 8 | (uint32_t)tmp_bytes[2];
  tmp_raw = (tmp_raw & 1 << 23) ? (0xff000000 | tmp_raw) : tmp_raw;

  float ftsc = (float)tmp_raw / kt;
  spl06_temperature = (float)c0 * 0.5f + (float)c1 * ftsc;

  // Read pressure
  Wire.beginTransmission (SPL06_007_I2C);
  Wire.write             (REG_PSR);
  Wire.endTransmission   ();
  Wire.requestFrom       (SPL06_007_I2C, 3);

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
  spl06_pressure = fp;

  // Calculate altitude
  float pK = fp / msl_pressure;
  spl06_altitude = (((powf(pK, (-(a * R) / g))) * T1) - T1) / a;
}
