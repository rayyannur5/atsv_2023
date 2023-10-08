#include <Arduino.h>

#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS.h>
#include <QMC5883LCompass.h>

#define LED 4

unsigned long timeBefore = 0;
bool _auto = true;
bool start_x = false;
String pid_str;
float pid = 0;


// Structure example to receive data
// Must match the sender structure
typedef struct {
    int speed;
    char move;
} struct_message;

typedef struct {
  double lat;
  double lon;
  int azimuth;
} SendData;

// Create a struct_message called myData
struct_message myData;
SendData sendData;
String _gps = "";
String _compass = "";

QMC5883LCompass compass;
Servo t2001;
Servo t2002;
TinyGPS gps;

// 40:91:51:45:98:39
// c8:f0:9e:a4:96:dc ESP32
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xA4, 0x96, 0xDC};
esp_now_peer_info_t peerInfo;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print(myData.speed);
  // Serial.print("\t\t");
  // Serial.println(myData.move);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // status == ESP_NOW_SEND_SUCCESS ? digitalWrite(LED, HIGH) : digitalWrite(LED, LOW);
}


int getCompass(){
  compass.read();

//  int x = compass.getX();
//  int y = compass.getY();
//  
//  x = x/20;
//
//  x = x * -1;
//
//  if (y <= 3500 && x <= 90){
//    x = 180 - x;
//  } else if (x >= 90) {
//    x = 90;
//  }
//
//  if (y >= 3500 && x < 0 ){
//    x = 360 + x;
//  }

  return compass.getAzimuth();
}

void parseGPS(){
  bool newData = false;
//  String val = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
//    val += c;
  }
  
//
//  // Serial.print(val);
//
//  if(val[0] == 'G'){
//    _gps = val;
//  } else if (val[0] == 'C'){
//    _compass = val;
//  }
//  
//  val = "";
//
//  delay(10);

    
  

   if (newData) {
     float flat, flon;
     unsigned long age;
     gps.f_get_position(&flat, &flon, &age);
     Serial.print("GPS");
     Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 10);
     Serial.print("<>");
     Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 10);

     sendData.lat = flat;
     sendData.lon = flon;
   }
}

void initMotor(){
  t2001.attach(18);
  t2002.attach(19);
  t2001.writeMicroseconds(1500);
  t2002.writeMicroseconds(1500);
  delay(7000);
}

void initEspNow(){
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.print("ESP NOW ERROR");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;    
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(9600);

  compass.init();
  
compass.setCalibrationOffsets(-81.00, -670.00, 311.00);
compass.setCalibrationScales(0.71, 0.80, 2.92);
  initMotor();
  initEspNow();

}


void loop() {
  parseGPS();
   sendData.azimuth = getCompass();

  // t2001 kiri
  // t2002 kanan
  int val = map(myData.speed, 0, 100, 1500, 1700);
//   int val = 1700;

  if (myData.move == 'o'){
    digitalWrite(LED, HIGH);
    _auto = true;
  } else if (myData.move == 'p'){
    digitalWrite(LED, LOW);
    myData.move = 'w';
    _auto = false;
  } else if (myData.move == 'r'){
    t2001.writeMicroseconds(1500 - ((val - 1500))); 
    t2002.writeMicroseconds(1500 - ((val - 1500))); 
  }

  if(!_auto){
    if(myData.move == 'a'){
      t2001.writeMicroseconds(val); 
        t2002.writeMicroseconds(1500 - ((val - 1500)/3));
    } else if (myData.move == 'd'){
      t2001.writeMicroseconds(1500 - ((val - 1500)/3)); 
        t2002.writeMicroseconds(val);
    } else if (myData.move == 'w'){
      t2001.writeMicroseconds(val );
      t2002.writeMicroseconds(val);
    } 
  }else{
    if(Serial.available()){
      char c = Serial.read();
      if (c == 'r'){
        ESP.restart();
      }
      else if(c == 'L'){
        t2001.writeMicroseconds(val);
        t2002.writeMicroseconds(1500 - ((val - 1500)/3));
      } else if (c == 'R'){
        t2001.writeMicroseconds(1500 - ((val - 1500)/3)); 
        t2002.writeMicroseconds(val);
      } else if (c == 'C') {
        t2001.writeMicroseconds(val); 
        t2002.writeMicroseconds(val);
      } else if (c == 'S') {
        t2001.writeMicroseconds(1500); 
        t2002.writeMicroseconds(1500);
      }
      else if(c == 'l'){
        t2001.writeMicroseconds(val);
        t2002.writeMicroseconds(1500);
      } else if (c == 'e'){
        t2001.writeMicroseconds(1500); 
        t2002.writeMicroseconds(val);
      } else if (c == 'c') {
        t2001.writeMicroseconds(val); 
        t2002.writeMicroseconds(val);
      } 
      else if(c == 'X'){
        start_x = true;
      }else if( c != 'n' && start_x){
        pid_str += c;
      }else if (c == 'n'){
        start_x = false;
        pid = pid_str.toFloat();
        val = 1700;
        // Serial.println(error_x);
        pid_str = "";

      
        int selisihVal = (val - 1500)*2;
        if(pid > selisihVal) pid = selisihVal; 
        else if(pid < -selisihVal) pid = -selisihVal;
      

        // Serial.println(PID);

        if(pid < 0){
          t2001.writeMicroseconds(val + pid);
          t2002.writeMicroseconds(val);
        } else {
          t2001.writeMicroseconds(val);
          t2002.writeMicroseconds(val - pid);
        }
      }


     
    }
    
  }



  if(millis() - timeBefore > 100){
      Serial.print("SPEED");
      Serial.println(myData.speed);
      Serial.print("COMPASS");
      Serial.println(sendData.azimuth);
      esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
      timeBefore = millis();
  }

 

}
