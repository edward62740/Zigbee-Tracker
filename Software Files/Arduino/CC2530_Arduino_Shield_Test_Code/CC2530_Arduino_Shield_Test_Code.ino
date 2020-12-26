/* This code is for testing the CC2530 Arduino Shield: it initializes the devices, requests for sensor data at regular intervals, converts raw sensor data into byte arrays and
  sends the data over UART to the CC2530 Zigbee Module for transmission to the coordinator as an end device. This code should be used for testing and development purposes only;
  there has been no testing to ensure long-term reliability. */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ClosedCube_OPT3001.h>
#include <SoftwareSerial.h>

// Byte arrays of commonly used commands for the Zigbee Module
uint8_t reset_cmd[4] = {0xFD, 0x01, 0x12, 0xFF};
//uint8_t set_sleep_on_cmd[5] = {0xFD, 0x02, 0x0D, 0x1F, 0xFF}; // 120 secs
//uint8_t set_sleep_off_cmd[5] = {0xFD, 0x02, 0x04, 0x00, 0xFF}; //off
//uint8_t set_pan_id_cmd[6] = {0xFD, 0x03, 0x03, 0xFF, 0xFF, 0xFF}; // ID = FFFF
uint8_t set_dev_type_cmd[5] = {0xFD, 0x02, 0x01, 0x02, 0xFF}; // end device
                            //{0xFD, 0x02, 0x01, 0x01, 0xFF); // router
                            //{0xFD, 0x02, 0x01, 0x00, 0xFF); // coordinator
uint8_t req_addrself_cmd[4] = {0xFE, 0x01, 0x06, 0xFF};
//uint8_t req_addrpar_cmd[4] = {0xFE, 0x01, 0x08, 0xFF};
uint8_t send_message[6] = {0xFC, 0x1F, 0x03, 0x02, 0x00, 0x00};
//uint8_t serial_wakeup_buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t software_current_mitigation_config[8] = {0xFD, 0x05, 0x20, 0xFF, 0xFF, 0x07, 0x00, 0xFF};
//uint8_t software_current_mitigation_set[8] = {0xFD, 0x05, 0x21, 0xFF, 0xFF, 0x07, 0x00, 0xFF};
//uint8_t seperation_byte[1] = {0xFB};
uint8_t res_addrself[7];

#define SEALEVELPRESSURE_HPA (1013.25)
#define OPT_ADDRESS 0x45
#define BME_ADDRESS 0x76

#define AT_MODE 4
#define BAUD 5
#define nRESET 6
#define EXT_INT 7
#define PIR 8

Adafruit_BME680 bme;
ClosedCube_OPT3001 opt;
SoftwareSerial Serial1(3, 2);

float HUM_WEIGHT = 0.25;
float GAS_WEIGHT = 0.75;
float HUM_SCORE, GAS_SCORE;
float GAS_REF = 250000;
float HUM_REF = 40;
int GAS_REF_COUNT = 0;
int sampling_period = 5000;
unsigned long time_now = 0;

float tx_temperature;
float tx_humidity;
float tx_gas;
float tx_light;
float tx_stat;


void setup() {

  Serial.begin(9600);
  Serial1.begin(115200);
  pinMode(AT_MODE, OUTPUT);
  pinMode(BAUD, OUTPUT);
  pinMode(nRESET, OUTPUT);
  pinMode(EXT_INT, INPUT);
  pinMode(PIR, INPUT);
  digitalWrite(nRESET, HIGH);
  digitalWrite(AT_MODE, LOW);
  Serial1.write(reset_cmd, sizeof(reset_cmd));
  delay(5000);
  Serial1.write(set_dev_type_cmd, sizeof(set_dev_type_cmd));
  delay(200);
  Serial1.write(reset_cmd, sizeof(reset_cmd));
  delay(5000);


  // Flush serial buffer
  Serial1.end();
  Serial1.begin(115200);

  // Get self MAC address
  Serial1.write(req_addrself_cmd, sizeof(req_addrself_cmd));
  delay(50);
  Serial1.readBytes((char *)res_addrself, 7);

  // Instantiate and initialize BME680 sensor
  bme.begin(BME_ADDRESS);
  // Set up BME680 oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Instantiate and initialize OPT3001 sensor
  opt.begin(OPT_ADDRESS);
  // Configure OPT3001 parameters
  OPT3001_Config newConfig;
  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;
  opt.writeConfig(newConfig);

}

void loop() {

  if (!measure()) {
    Serial.println("Measurement Failed");
  }


  time_now = millis();
  while (millis() < time_now + sampling_period) {}

}
bool measure() {

  /* Convert sensor readings to byte array */
  // Read temperature
  typedef union
  {
    float x1;
    uint8_t bme_temp[4];
  } union1;
  union1 temp;
  temp.x1 = bme.readTemperature();

  // Read pressure
  typedef union
  {
    float x2;
    uint8_t bme_pres[4];
  } union2;
  union2 pres;
  pres.x2 = bme.readPressure() / 100.0;

  // Read humidity
  typedef union
  {
    float x3;
    uint8_t bme_humi[4];
  } union3;
  union3 humi;
  humi.x3 = bme.readHumidity();

  // Read light
  OPT3001 result = opt.readResult();
  typedef union
  {
    float x4;
    uint8_t opt_lux[4];
  } union4;
  union4 opt;
  opt.x4 = result.lux;

  // Read gas resistance and calculate IAQ
  float current_humidity = bme.readHumidity();
  if (current_humidity >= 38 && current_humidity <= 42)
    HUM_SCORE = 0.25 * 100; // Humidity +/-5% around optimum
  else
  { //sub-optimal
    if (current_humidity < 38)
      HUM_SCORE = 0.25 / HUM_REF * current_humidity * 100;
    else
    {
      HUM_SCORE = ((-0.25 / (100 - HUM_REF) * current_humidity) + 0.416666) * 100;
    }
  }
  //Calculate gas contribution to IAQ index
  float gas_lower_limit = 5000;   // Bad air quality limit
  float gas_upper_limit = 50000;  // Good air quality limit
  if (GAS_REF > gas_upper_limit) GAS_REF = gas_upper_limit;
  if (GAS_REF < gas_lower_limit) GAS_REF = gas_lower_limit;
  GAS_SCORE = (0.75 / (gas_upper_limit - gas_lower_limit) * GAS_REF - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100;

  //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  float air_quality_score = HUM_SCORE + GAS_SCORE;
  if ((GAS_REF_COUNT++) % 10 == 0) GetGasReference();

  typedef union
  {
    float x5;
    uint8_t bme_iaq[4];
  } union5;
  union5 iaq;
  iaq.x5 = ((100 - air_quality_score) * 5);

  // Request Zigbee Module to send message to coordinator containing sensor data
  Serial1.write(send_message, sizeof(send_message));
  Serial1.write(res_addrself, sizeof(res_addrself));
  Serial1.write(temp.bme_temp, sizeof(temp.bme_temp));
  Serial1.write(pres.bme_pres, sizeof(pres.bme_pres));
  Serial1.write(humi.bme_humi, sizeof(humi.bme_humi));
  Serial1.write(opt.opt_lux, sizeof(opt.opt_lux));
  Serial1.write(iaq.bme_iaq, sizeof(iaq.bme_iaq));

  return !bme.performReading() || result.error != NO_ERROR ? 0 : 1;
}


void GetGasReference() {

  // Gas reference value
  int readings = 10;
  for (int i = 1; i <= readings; i++) { // read gas for 10 x 0.150mS = 1.5secs
    GAS_REF += bme.readGas();
  }
  GAS_REF = GAS_REF / readings;
}
