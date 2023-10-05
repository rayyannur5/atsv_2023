#include <ESP32Servo.h>
#include <TinyGPS.h>
#include "soc/rtc_wdt.h"

// QMC5883LCompass compass;
Servo t2001;
Servo t2002;
TinyGPS gps;


#define LED 4


 
void setup() {
  // Initialize Serial Monitor

  rtc_wdt_protect_off();
  rtc_wdt_disable();

  pinMode(LED, OUTPUT);
  // pinMode(23, OUTPUT);
  // pinMode(13, OUTPUT);
  Serial.begin(115200);
  // Serial1.begin(9600);

  // compass.init();
  t2001.attach(18);
  t2002.attach(19);
  t2001.writeMicroseconds(1500);
  t2002.writeMicroseconds(1500);
  delay(7000);

}

// unsigned long timeBefore = 0;
 
void loop() {
  // compass.read();
  // t2001 kiri
  // t2002 kanan
  int val = 1700;

  // bool newData = false;

  // while (Serial1.available()) {
  //   char c = Serial1.read();
  //   // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
  //   if (gps.encode(c)) // Did a new valid sentence come in?
  //     newData = true;
  // }

  // if (newData) {
  //   float flat, flon;
  //   unsigned long age;
  //   gps.f_get_position(&flat, &flon, &age);
  //   Serial.print("GPS");
  //   Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 8);
  //   Serial.print("<>");
  //   Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 8);

  //   sendData.lat = flat;
  //   sendData.lon = flon;
  //   // sendData.azimuth = compass.getAzimuth();
  //   // Serial.print("SPEED");
  //   // Serial.println(myData.speed);
  //   // Serial.print("COMPASS");
  //   // Serial.println(sendData.azimuth);
  //   // esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));

  // }

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