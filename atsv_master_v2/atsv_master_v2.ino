#include "soc/rtc_wdt.h"
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS.h>

// Structure example to receive data
// Must match the sender structure
typedef struct {
    int speed;
    char move;
} struct_message;

// typedef struct {
//   double lat;
//   double lon;
//   int azimuth;
// } SendData;

// Create a struct_message called myData
struct_message myData;
// SendData sendData;

// QMC5883LCompass compass;
Servo t2001;
Servo t2002;
TinyGPS gps;

// 40:91:51:45:98:39
// c8:f0:9e:a4:96:dc ESP32
// uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xA4, 0x96, 0xDC};
// esp_now_peer_info_t peerInfo;

#define LED 4

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print(myData.speed);
  Serial.print("\t\t");
  Serial.println(myData.move);
}

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   // Serial.print("\r\nLast Packet Send Status:\t");
//   // status == ESP_NOW_SEND_SUCCESS ? digitalWrite(LED, HIGH) : digitalWrite(LED, LOW);
// }
 
void setup() {
  // Initialize Serial Monitor

  rtc_wdt_protect_off();
  rtc_wdt_disable();

  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(9600);

  // compass.init();
  t2001.attach(18);
  t2002.attach(19);
  t2001.writeMicroseconds(1500);
  t2002.writeMicroseconds(1500);
  delay(7000);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.print("ESP NOW ERROR");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // peerInfo.channel = 0;  
  // peerInfo.encrypt = false;
  
  // Add peer        
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   return;
  // }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}

unsigned long timeBefore = 0;
bool _auto = false;

void loop() {
  // compass.read();
  // t2001 kiri
  // t2002 kanan
  int val = 1700;
 if(myData.move == 'a'){
    t2001.writeMicroseconds(1500 - (val - 1500));
    t2002.writeMicroseconds(val);
  } else if (myData.move == 'd'){
    t2001.writeMicroseconds(val); 
    t2002.writeMicroseconds(1500 - (val - 1500));
  } else if (myData.move == 'w'){
    t2001.writeMicroseconds(val );
    t2002.writeMicroseconds(val);
  } else if (myData.move == 'o'){
    digitalWrite(LED, HIGH);
    _auto = true;
  } else if (myData.move == 'p'){
    digitalWrite(LED, LOW);
    _auto = false;
  }

  bool newData = false;

  while (Serial1.available()) {
    char c = Serial1.read();
    // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
      newData = true;
  }

  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("GPS");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 8);
    Serial.print("<>");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 8);

    // sendData.lat = flat;
    // sendData.lon = flon;
    // sendData.azimuth = compass.getAzimuth();
    // Serial.print("SPEED");
    // Serial.println(myData.speed);
    // Serial.print("COMPASS");
    // Serial.println(sendData.azimuth);
    // esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));

  }

  // if(millis() - timeBefore > 500){
  //     timeBefore = millis();
  // }

  if(_auto){
    if(Serial.available()){
      char c = Serial.read();
      if(c == 'L'){
        t2001.writeMicroseconds(val);
        t2002.writeMicroseconds(1500);
      } else if (c == 'R'){
        t2001.writeMicroseconds(1500); 
        t2002.writeMicroseconds(val);
      } else if (c == 'C') {
        t2001.writeMicroseconds(val); 
        t2002.writeMicroseconds(val);
      } else if (c == 'S') {
        t2001.writeMicroseconds(1500); 
        t2002.writeMicroseconds(1500);
      }
    }
  }

}