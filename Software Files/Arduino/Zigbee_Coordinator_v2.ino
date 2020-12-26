#include <WiFi.h>
#include "FirebaseESP32.h"

#define FIREBASE_HOST ""
#define FIREBASE_AUTH ""
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

FirebaseData firebaseData;
FirebaseJson json_update_dev;
FirebaseJson json_update_coord;
FirebaseJson json_update_coor;

uint8_t device_list;
uint8_t received_array[44] = {};
uint8_t self_addr[8] = {};
uint8_t par_addr[8] = {};
uint8_t humidity[4] = {};
uint8_t temperature[4] = {};
uint8_t light[4] = {};
uint8_t pressure[4] = {};

const long prevLEDTime = 0;
const long LEDInt = 200;
bool ledState;
int i = 0;
char buffer_self_addr[2 * 8 + 1];
char buffer_par_addr[2 * 8 + 1];
void setup() {
  pinMode(4, OUTPUT);
  pinMode(14, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  digitalWrite(4, HIGH);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  Serial2.print("AT+RESET");
}

void loop() {
  digitalWrite(4, HIGH);
  json_update_coor.set("status", "OK");

  if (Firebase.updateNode(firebaseData, "/coord_reporting", json_update_coor)) {
    Serial.println(firebaseData.dataPath());
    Serial.println(firebaseData.dataType());
    Serial.println(firebaseData.jsonString());
  }
  if (Serial2.available()) {
    digitalWrite(4, LOW);
    Serial2.readBytes((char *)received_array, 44);
    if (received_array[2] == 0xFB) {
      memcpy(par_addr, received_array + 3, 8);
      if (received_array[11] == 0xFB) {
        memcpy(self_addr, received_array + 12, 8);
      }
    }


    char* myPtr1 = &buffer_par_addr[0];
    for (byte i = 0; i < 8; i++) {
      snprintf(myPtr1, 3, "%02x", par_addr[i]);
      myPtr1 += 2;
    }
    char* myPtr2 = &buffer_self_addr[0];
    for (byte i = 0; i < 8; i++) {
      snprintf(myPtr2, 3, "%02x", self_addr[i]);
      myPtr2 += 2;
    }
    if (received_array[20] == 0xFB) {
      memcpy(humidity, received_array + 21, 4);
    }
    float humi = *(float*)&humidity;


    if (received_array[25] == 0xFB) {
      memcpy(temperature, received_array + 26, 4);

    }
    float temp = *(float*)&temperature;

    if (received_array[35] == 0xFB) {
      memcpy(light, received_array + 36, 4);

    }
    float li = *(float*)&light;

    if (received_array[30] == 0xFB) {
      memcpy(pressure, received_array + 31, 4);

    }
    float pr = *(float*)&pressure;
    if (received_array[2] == 0xFB) {
      
      received_array[2] = 0x00;

      String par = buffer_par_addr;
      String ed = buffer_self_addr;

      String light = String(li, 2);
      String humidity = String(humi, 2);
      String pressure = String(pr, 2);
      String temperature = String(temp, 2);
      String info = "Temperature: " + temperature + ", Humidity: " + humidity + ", Pressure: " + pressure + ", Light: " + light;
      json_update_dev.set("parent", par);
      json_update_dev.set("device", ed);
      json_update_dev.set("info", info);

      if (Firebase.updateNode(firebaseData, buffer_self_addr, json_update_dev)) {
        Serial.println(firebaseData.dataPath());
        Serial.println(firebaseData.dataType());
        Serial.println(firebaseData.jsonString());
      } else {
      }
      json_update_coord.set("device", "ffffffffffffff1200");
      json_update_coord.set("i2c_stat", "HALL_SENSE detected");
      json_update_coord.set("mcu_stat", "OK");
      json_update_coord.set("status", "TRANSMITTING");
      json_update_coord.set("wifi_name", WIFI_SSID);
      String coordinfo = "Hall Effect Sensor: " + (String)hallRead();
      json_update_coord.set("info", coordinfo);


      if (Firebase.updateNode(firebaseData, "/coord_reporting", json_update_coord)) {
        Serial.println(firebaseData.dataPath());
        Serial.println(firebaseData.dataType());
        Serial.println(firebaseData.jsonString());
      }
    }
  }
  else {
    unsigned long currentLEDTime = millis();


    if (currentLEDTime - prevLEDTime >= LEDInt) {
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(14, ledState);
    }
  }
}
