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
#include <BlynkSimpleEsp32.h>

// Insert your network credentials
#define WIFI_SSID "Hayo"
#define WIFI_PASSWORD "mikeh123"
BlynkTimer timer;

//TDS Initiation
DFRobot_ESP_EC ec;
//Adafruit_ADS1115 ads;
float voltage, ecValue, temperature = 25;

//Sensor Pins
#define echoPin 18
#define trigPin 19
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

//Reset Button Called from Blynk
BLYNK_WRITE(V2){
  int pinValue = param.asInt();
  Serial.println(pinValue);
  if(pinValue == 1){
   bottomDistance = takeDistance();
   Serial.print("Put new Bottom Distance (Reset Pressed): ");
   Serial.println(bottomDistance);
   delay(1000);
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

void dataProcess() {
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
  waterLevel = bottomDistance - takeDistance();
  Serial.print("Water Level: ");
  Serial.println(waterLevel);
  Blynk.virtualWrite(V0,tdsValue);
  Blynk.virtualWrite(V1,waterLevel);

  ec.calibration(voltage,temperature);
}

void setup(){
  Serial.begin(115200);
  // connect WiFi
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  timer.setInterval(10000L,dataProcess);
  pinMode(TdsSensorPin,INPUT);
  pinMode(probePin,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Connecting");
  Blynk.begin(BLYNK_AUTH_TOKEN,WIFI_SSID,WIFI_PASSWORD);
  EEPROM.begin(32);
  ec.begin();
}

void loop(){
  Blynk.run();
  timer.run();
}
