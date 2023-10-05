#include <WiFi.h>
#include <esp_now.h>

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

esp_now_peer_info_t peerInfo;


int speed = 0;
String tempSpeed = "0";

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.print(status);
  if(ESP_NOW_SEND_SUCCESS){
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
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
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
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
    } else if (c == 'o'){
      myData.move = c;
      Serial.println("auto1");
    } else if (c == 'p'){
      myData.move = c;
      Serial.println("auto0");
    }
     else {
      myData.move = c;
    }
  }
  
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));


}
