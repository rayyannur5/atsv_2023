#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
// 58:BF:25:17:C3:6C
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x17, 0xC3, 0x6C};

typedef struct struct_message {
  int speed;
  char move;
} struct_message;

typedef struct {
  double lat;
  double lon;
  int azimuth;
} ReceiveData;

// Create a struct_message called myData
struct_message myData;
ReceiveData receiveData;
int speed = 0;
String tempSpeed = "0";

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0){
    digitalWrite(LED_BUILTIN, LOW);
  }
  else{
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&receiveData, incomingData, sizeof(receiveData));
  // Serial.print("Bytes received: ");
  Serial.print("GPS");
  Serial.print(receiveData.lat, 8);
  Serial.print("<>");
  Serial.println(receiveData.lon, 8);

  // Serial.print("COMPASS");
  // Serial.println(receiveData.azimuth);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  myData.move = 'w';
}

bool isGetSpeed = false;
void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    char c = Serial.read();
    if(c == 'n') {
      isGetSpeed = true;
      tempSpeed = "";
    } else if(isGetSpeed){
      if(c == '*'){
        isGetSpeed = false;
        speed = tempSpeed.toInt();
        myData.speed = speed;
        return;
      }
      tempSpeed = tempSpeed + c;
    } else {
      myData.move = c;
    }
  }
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));


}
