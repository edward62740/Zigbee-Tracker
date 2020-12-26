/* This is the Arduino code to be used with Zigbee Tracker v2.0 end devices. This code sets the entire device~ to sleep, with accelerometer Significant Motion @ 12.5Hz
    used to wake the device on motion for the purpose of tracking. On wake, the code samples the sensors, requests information from the Zigbee Module and sends the
    Zigbee Module a unicast request to 0000 (Coordinator). This unicast message contains all information needed to track the device.

    ~ Only applies when the correct hardware specifications are met as per the Zigbee Tracker System Documentation, and the correct libraries are used.
*/
#include <ClosedCube_OPT3001.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BMI160Gen.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include <Wire.h>

Adafruit_BME280 BME;
ClosedCube_OPT3001 OPT;
const int IMU_ADDRESS = 0x68;
#define OPT_ADDRESS 0x45


uint8_t reset_cmd[4] = {0xFD, 0x01, 0x12, 0xFF};
uint8_t set_sleep_on_cmd[5] = {0xFD, 0x02, 0x0D, 0x1F, 0xFF}; // 120 secs
uint8_t set_sleep_off_cmd[5] = {0xFD, 0x02, 0x04, 0x00, 0xFF}; //off
uint8_t set_pan_id_cmd[6] = {0xFD, 0x03, 0x03, 0xFF, 0xFF, 0xFF}; // ID = FFFF
uint8_t set_dev_type_cmd[5] = {0xFD, 0x02, 0x01, 0x02, 0xFF}; // end device
uint8_t req_addrself_cmd[4] = {0xFE, 0x01, 0x06, 0xFF};
uint8_t req_addrpar_cmd[4] = {0xFE, 0x01, 0x08, 0xFF};
uint8_t send_message[6] = {0xFC, 0x2C, 0x03, 0x02, 0x00, 0x00};
uint8_t serial_wakeup_buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t software_current_mitigation_config[8] = {0xFD, 0x05, 0x20, 0xFF, 0xFF, 0x07, 0x00, 0xFF};
uint8_t software_current_mitigation_set[8] = {0xFD, 0x05, 0x21, 0xFF, 0xFF, 0x07, 0x00, 0xFF};
uint8_t seperation_byte[1] = {0xFB};

uint16_t IMU_temp;
uint8_t res_addrself[9];
uint8_t res_addrpar[11];
uint8_t res_addrparprev[9];


HardwareSerial Serial2(PA3, PA2); // for communication with Zigbee Module
//HardwareSerial Serial1(PA10, PA9); // for communication with external debugger


const int sleepIMUInterrupt = PA15; // accelerometer interrupt wakeup pin
const int sleepLDInterrupt = PB15; // ALS interrupt
volatile int incr = 0; // declared volatile for use in ISR

void IMU_ISR(void) // interrupt service routine for significant motion triggered (reg 0x5F - 0x62)
{
  incr++; // increment volatile

}
void setup()
{
  
  
  Serial.begin(9600);
  Serial2.begin(115200);

  BME.begin();
  LowPower.begin();
  BMI160.begin(BMI160GenClass::I2C_MODE, IMU_ADDRESS, sleepIMUInterrupt);
  LowPower.attachInterruptWakeup(sleepIMUInterrupt, IMU_ISR, CHANGE);
  BMI160.attachInterrupt(IMU_ISR);
  BMI160.setIntMotionEnabled(true); // initialize significant motion detection. additionally, write 0x12 to 0x7E (acc lpr), 1 to acc_us [7] and 7 to [3:0] of 0x40 (50hz odr)
  //BMI160.setGyroRate(0); // set 0x7E to 0x14, 0x12 to disable gyro and ext. mag
  OPT.begin(OPT_ADDRESS);
  configureOPT(); // function to configure OPT registers
  BME.setSampling(Adafruit_BME280::MODE_FORCED, // set BME280 to forced mode
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                   Adafruit_BME280::FILTER_OFF   );

  disableGPIO(); // set unused GPIO to analog input
  defGPIOsetup(); // apply settings to connected GPIO
  if (verifySense() != true) {
    digitalWrite(PA6, !digitalRead(PA6));
    delay(50);
  }
  getselfMACaddr(); // get the MAC address of this node

  
}

void loop()
{
  if (incr < 80) { // prevents noise or glitch from waking up the system. reg 0x5F - 0x62 INT_MOTION for settings.
    LowPower.deepSleep();
  }

  BMI160.detachInterrupt(); // prevent interrupt from disrupting time-sensitive operation
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  disableGPIO(); // set unused GPIO to analog input
  defGPIOsetup(); // apply settings to connected GPIO

  Serial2.begin(115200);

  //BME.write8(0xF4,0x7C);

  IMU_temp = BMI160.getTemperature(); // calibration sensor for IMU - not used

  //memcpy(res_addrparprev, res_addrpar, 3);
  //Serial.flush();

  /* Serial2 Transmission with Zigbee Module begins */
  digitalWrite(PA6, HIGH);
  trackingAlgorithm();
  digitalWrite(PA6, LOW);
  BME.takeForcedMeasurement();
  writeBMEData();
  OPT3001 result = OPT.readResult();
  writeOPTData("OPT3001", result);
  delay(80);
  Serial2.write(set_sleep_on_cmd, sizeof(set_sleep_on_cmd));

  delay(30); //reset Zigbee Module
  digitalWrite(PB12, LOW);
  digitalWrite(PA6, HIGH);
  delay(30);
  digitalWrite(PB12, HIGH);
  digitalWrite(PA6, LOW);


  softwareMitigations(); // software mitigation for cc2530 p2.0 incorrectly tied to ground
  Serial2.end();
  Serial.end();

  /* Serial2 Transmission with Zigbee Module ends */
  
  incr = 0; // reset volatile increment
  PWR->CR &= ~((uint32_t)PWR_CR_PVDE); // disable PVD
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  // GPIO_B and GPIO_C are disabled
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  //TIM1->CR1 |= 0;
  HAL_PWR_DisablePVD();
  lprGPIOsetup(); // set all used GPIO (except interrupt) as analog inputs
  disableGPIO(); // set unused GPIO to analog input
  //__HAL_RCC_PLL_DISABLE();
  LowPower.deepSleep(4000); // sleep buffer to prevent continuous sleep/wake during movement
  BMI160.attachInterrupt(IMU_ISR);
  LowPower.deepSleep(); // enter sleep mode. HSE/LSE, HSI, PLL disabled. Reg low-power mode. SRAM, registers retained.
}
void writeBMEData() {
  Serial2.write(seperation_byte, sizeof(seperation_byte));
  typedef union
  {
    float c;
    uint8_t bme_humi[4];
  } union1;
  union1 humi;
  humi.c = BME.readHumidity();
  Serial2.write(humi.bme_humi, sizeof(humi.bme_humi));
  Serial2.write(seperation_byte, sizeof(seperation_byte));
  typedef union
  {
    float d;
    uint8_t bme_temp[4];
  } union2;
  union2 temp;
  temp.d = BME.readTemperature();
  Serial2.write(temp.bme_temp, sizeof(temp.bme_temp));
  Serial2.write(seperation_byte, sizeof(seperation_byte));
  typedef union
  {
    float e;
    uint8_t bme_prsr[4];
  } union3;
  union3 prsr;
  prsr.e = (BME.readPressure() / 100.00F);
  Serial2.write(prsr.bme_prsr, sizeof(prsr.bme_prsr));
}

void writeOPTData(String text, OPT3001 result) {
  Serial2.write(seperation_byte, sizeof(seperation_byte));
  typedef union
  {
    float f;
    uint8_t opt_als[4];
  } union4;
  union4 opt;
  opt.f = result.lux;
  Serial2.write(opt.opt_als, sizeof(opt.opt_als));
}

bool verifySense() {
  if (BMI160.getDeviceID() != 0xD1) {
    return false;
  }
  if (BME.sensorID() != 0x60) {
    return false;
  }
  return true;
}

void disableGPIO() {
  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA1, INPUT_ANALOG);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA7, INPUT_ANALOG);
  pinMode(PA8, INPUT_ANALOG);
  pinMode(PA9, INPUT_ANALOG);
  pinMode(PA10, INPUT_ANALOG);
  pinMode(PA13, INPUT_ANALOG);
  pinMode(PA14, INPUT_ANALOG);
  pinMode(PB0, INPUT_ANALOG);
  pinMode(PB1, INPUT_ANALOG);
  pinMode(PB2, INPUT_ANALOG);
  pinMode(PB3, INPUT_ANALOG);
  pinMode(PB4, INPUT_ANALOG);
  pinMode(PB5, INPUT_ANALOG);
  pinMode(PB8, INPUT_ANALOG);
  pinMode(PB9, INPUT_ANALOG);
  pinMode(PB13, INPUT_ANALOG);
  pinMode(PB14, INPUT_ANALOG);
  pinMode(PC13, INPUT_ANALOG);

  pinMode(PB15, INPUT_ANALOG);



}

void getselfMACaddr() {
  Serial2.write(serial_wakeup_buffer, sizeof(serial_wakeup_buffer));
  digitalWrite(PA6, HIGH);
  delay(150);
  digitalWrite(PA6, LOW);
  Serial2.write(req_addrself_cmd, sizeof(req_addrself_cmd));
  delay(80);
  digitalWrite(PA6, HIGH);
  Serial2.readBytes((char *)res_addrself, 9);
  while (res_addrself[0] != 0xFB) {
    Serial2.write(req_addrself_cmd, sizeof(req_addrself_cmd));
    delay(80);
    digitalWrite(PA6, LOW);
    Serial2.readBytes((char *)res_addrself, 9);
    digitalWrite(PA6, HIGH);
  }
  digitalWrite(PA6, LOW);
}

void defGPIOsetup() {
  pinMode(PA6, OUTPUT); // status LED
  //pinMode(PA15, INPUT);
  pinMode(PB2, OUTPUT); // boot1 config
  pinMode(PB10, OUTPUT); // zigbee baudrate reset - falling edge
  pinMode(PB11, OUTPUT); // zigbee command mode - 0: HEX, 1: AT
  pinMode(PB12, OUTPUT); // zigbee hardware reset
  //pinMode(PB15, INPUT);
  digitalWrite(PB2, LOW);
  digitalWrite(PB10, LOW);
  digitalWrite(PB11, LOW);
  digitalWrite(PB12, HIGH);
}

void lprGPIOsetup() {
  pinMode(PA6, INPUT_ANALOG); // status LED
  pinMode(PB2, INPUT_ANALOG); // boot1 config
  pinMode(PB10, INPUT_ANALOG); // zigbee baudrate reset - falling edge
  pinMode(PB11, INPUT_ANALOG); // zigbee command mode - 0: HEX, 1: AT
  pinMode(PB12, INPUT_ANALOG); // zigbee hardware reset
}

void configureOPT() {
  OPT3001_Config newConfig;
  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;

  OPT3001_ErrorCode errorConfig = OPT.writeConfig(newConfig);
}

void softwareMitigations() {
  delay(2000);
  Serial2.write(software_current_mitigation_config, sizeof(software_current_mitigation_config));
  delay(80);
  Serial2.write(software_current_mitigation_config, sizeof(software_current_mitigation_config));
  delay(80);
  Serial2.write(software_current_mitigation_set, sizeof(software_current_mitigation_set));
  delay(100);
}


void trackingAlgorithm() {
  Serial2.write(serial_wakeup_buffer, sizeof(serial_wakeup_buffer));
  delay(150);
  Serial2.write(req_addrpar_cmd, sizeof(req_addrpar_cmd));
  delay(80);
  Serial2.readBytes((char *)res_addrpar, 11);
  delay(80);
  Serial2.write(send_message, sizeof(send_message));
  Serial2.write(res_addrpar, sizeof(res_addrpar));
  Serial2.write(res_addrself, sizeof(res_addrself));
}
