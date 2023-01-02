/* ESP32 IoT Module from HydroMon
 * by Michael Harditya
 Outsourcing: 
 https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
 https://RandomNerdTutorials.com/esp32-tds-water-quality-sensor/
 https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
 https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/
#define BLYNK_TEMPLATE_ID "TMPLcHtn5Avo"
#define BLYNK_DEVICE_NAME "Hydromon Template"
#define BLYNK_AUTH_TOKEN "MAIgQDV71UPoVE9L8KjSrOiDPTCcZOQK"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <esp_now.h>

#define STA_SSID "Hayo"
#define STA_PASS "mikeh123"

TaskHandle_t Task1;
WiFiServer server(80);
WiFiClient client;
BlynkTimer timer;
String success;
//struct to hold sensor data
typedef struct struct_message {
  int ID;
  float tdsValue;
  int waterLevel;
}msg;
//struct to hold control data
typedef struct struct_control {
  int ID;
  bool remeasure;
}crtl;
crtl controlMessage;
msg dataMessage;
msg sender1;
msg sender2;
msg boardsStruct[2] = {sender1, sender2};

//change to the node addresses
uint8_t nodeAddress[] = {0x7C, 0x9E, 0xBD, 0x37, 0xCA, 0x84};

//Callback for received data from nodes with ESP NOW
void data_receive(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&dataMessage, incomingData, sizeof(dataMessage));
  Serial.printf("Board %u: %u bytes\n", dataMessage.ID, len);

  boardsStruct[dataMessage.ID-1].tdsValue = dataMessage.tdsValue;
  boardsStruct[dataMessage.ID-1].waterLevel = dataMessage.waterLevel;

  Serial.printf("TDS Value: %d \n", boardsStruct[dataMessage.ID-1].tdsValue);
  Serial.printf("Water Level: %d \n", boardsStruct[dataMessage.ID-1].waterLevel);
  Serial.println();
}

// Callback when data is sent with ESP NOW
void data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

//Reset Button Called from Blynk
BLYNK_WRITE(V2){
  int pinValue = param.asInt();
  Serial.println(pinValue);
  if(pinValue == 1){
    controlMessage.remeasure = true;
 } else {
   controlMessage.remeasure = false;
 }
 //send remeasuring state to nodes
    esp_err_t result = esp_now_send(nodeAddress, (uint8_t *) &controlMessage, sizeof(controlMessage));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
}
 //Blynk start
BLYNK_CONNECTED() {
  Blynk.setProperty(V3, "offImageUrl",
  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",
  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url",
  "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

//sending data to Blynk
void dataProcess() {
  Blynk.virtualWrite(V0,sender1.tdsValue);
  Blynk.virtualWrite(V1,sender1.waterLevel);
}

void setup() {
  Serial.begin(115200);
  //Set ESP as AP and Station
  WiFi.mode(WIFI_STA);
  //initiate ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(data_sent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, nodeAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(data_receive);

  //Initiate Station connection to wireless network
  WiFi.begin(STA_SSID,STA_PASS);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //Blynk initialization
  timer.setInterval(10000L,dataProcess);
  Blynk.begin(BLYNK_AUTH_TOKEN,STA_SSID,STA_PASS);
}

void loop() {
  timer.run();
  Blynk.run();
}