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
#include <WiFi.h>
#include <HTTPClient.h>
#include "Adafruit_ADS1X15.h"
#include <Arduino.h>
#include "DFRobot_ESP_EC.h"

// Insert your network credentials
#define AP_SSID "BaS3Station"
#define AP_PASS "Bas3b@ll"

const char* serverName = "http://192.168.4.1/post";

//TDS Initiation
DFRobot_ESP_EC ec;
//Adafruit_ADS1115 ads;
float voltage, ecValue, temperature = 25;

//Sensor Pins
#define echoPin 12
#define trigPin 13
#define TdsSensorPin A0

//Sensor Values
long duration;
int distance;
float averageVoltage = 0;
int tdsValue = 0;

// Water Level variables, store the distance value in the array
int bottomDistance = 200;
int waterLevel = 0;

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

void postData() {
  HTTPClient http;

  http.begin(serverName); // specify the POST endpoint
  http.addHeader("Content-Type", "application/x-www-form-urlencoded"); // set the content-type header

  // create a POST request payload
  String payload = "tds="+String(tdsValue)+"&water="+String(waterLevel);
  int httpCode = http.POST(payload); // send the POST request

  if (httpCode > 0) {
    // check the status code
    String response = http.getString(); // get the response payload
    Serial.println(httpCode); // print the status code
    Serial.println(response); // print the response payload
  } else {
    Serial.println("Error: " + http.errorToString(httpCode));
  }

  http.end(); // close the connection
}

void dataProcess() {
  //read tds value
  voltage = analogRead(TdsSensorPin);
  ecValue = ec.readEC(voltage, temperature); // convert voltage to EC with temperature compensation
  tdsValue = ecValue *900; //approximately 1 ec = 900ppm
  Serial.print("TDS:");
  Serial.println(tdsValue); 
  //read water level
  waterLevel = bottomDistance - takeDistance();
  Serial.print("Water Level: ");
  Serial.println(waterLevel);
  postData();
  ec.calibration(voltage,temperature);
}

void setup(){
  Serial.begin(115200);
  pinMode(TdsSensorPin,INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Connecting");
  WiFi.begin(AP_SSID,AP_PASS);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  ec.begin();
}

void loop(){
  dataProcess();
  delay(5000);
}
