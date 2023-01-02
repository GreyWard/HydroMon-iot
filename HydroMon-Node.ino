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

// Libraries
#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include "EEPROM.h"  
#include "DFRobot_ESP_EC.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>

//TDS Initiation
DFRobot_ESP_EC ec;
//Adafruit_ADS1115 ads;
float voltage, ecValue, temperature = 25;

//Sensor Pins
#define echoPin 12
#define trigPin 13
#define TdsSensorPin 27
#define probePin 29           // to release voltage to the water level sensor

//Constants
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

//Sensor Values
long duration;
int distance;
float averageVoltage = 0;
float tdsValue = 0;

// Water Level variables, store the distance value in the array
int bottomDistance = 0;
int waterLevel = 0;
//controller variables
bool remeasuring = false;

//message structure
typedef struct struct_message {
    int ID;
    float tds;
    int water;
} struct_message;
//struct to hold control data
typedef struct struct_control {
  int ID;
  bool remeasure;
}crtl;
crtl controlMessage;
struct_message message;

uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0x37, 0xCA, 0x84};

//Task initialization
TaskHandle_t Task1;
TaskHandle_t Task2;

void data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n Last Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//Callback for received data from nodes with ESP NOW
void data_receive(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&controlMessage, incomingData, sizeof(controlMessage));
  remeasuring = controlMessage.remeasure;
}

/* read data from ultrasonic sensor, in centimeter
 * @return distance(int)
 */
int takeDistance(){
  //clear trigPin
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  
  //Set trigPin to High in 10 microsecs
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  //read the echo
  duration = pulseIn(echoPin,HIGH);
  
  //calculate distance
  distance = duration * 0.034 / 2;
  return distance;
}

void sensorTask(void *pvParameter) {
  for(;;){
    //read tds value
    voltage = analogRead(A0);
      //Serial.print("voltage:");
      //Serial.println(voltage, 4);

      ecValue = ec.readEC(voltage, temperature); // convert voltage to EC with temperature compensation
      tdsValue = ecValue *900; //approximately 1 ec = 900ppm
      Serial.print("EC:");
      //Serial.print(ecValue, 4);
      //Serial.println("ms/cm");
      Serial.println(ecValue); 
      Serial.println(tdsValue); 
    //read water level
    if (remeasuring == true){
      bottomDistance = takeDistance();
      Serial.print("Put new Bottom Distance (Reset Pressed): ");
      Serial.println(bottomDistance);
      delay(1000);
    }else{
      waterLevel = bottomDistance - takeDistance();
      Serial.print("Water Level: ");
      Serial.println(waterLevel);
    }
    ec.calibration(voltage,temperature);
    delay(5000);
  }
}

void sendTask(void *pvParameter){
  for (;;){
    message.ID = 1;
    message.tds = tdsValue;
    message.water = waterLevel;

    esp_err_t outcome = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
    
    if (outcome == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    delay(10000);
  }
}

void setup(){
  Serial.begin(115200);
  // connect ESP NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(data_sent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(data_receive);
  pinMode(TdsSensorPin,INPUT);
  pinMode(probePin,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Connecting");
  EEPROM.begin(32);
  ec.begin();
  xTaskCreatePinnedToCore(
  sendTask,
  "Data Send Task",
  10000,
  NULL,
  1,
  &Task1,
  0);
  delay(500);
xTaskCreatePinnedToCore(
  sensorTask,
  "Sensor Task",
  10000,
  NULL,
  1,
  &Task2,
  1);
  delay(500);
}

void loop(){ //we already divided the tasks above
}
