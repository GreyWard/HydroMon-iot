/* ESP32 IoT Module from HydroMon
 * by Michael Harditya
 Outsourcing: 
 https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
 https://RandomNerdTutorials.com/esp32-tds-water-quality-sensor/
 https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
 https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/

// Libraries
#include "EEPROM.h"
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBpbe1_pyHkzX388NJrgHt9bxuNMCtUct8"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://hydromon-f29cc-default-rtdb.asia-southeast1.firebasedatabase.app/" 

// Insert your network credentials
#define WIFI_SSID "Yak"
#define WIFI_PASSWORD "Blue@231"

// Defining Pins
#define echoPin 2
#define trigPin 0
#define resetPin 15
#define TdsSensorPin 27
#define probePin 29 

// Constants
#define EEPROM_SIZE 1
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  360        /* Time ESP32 will go to sleep (in seconds) */
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
//RTC_DATA_ATTR int bootCount = 0;

//Global Variables
long duration;
int distance;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

// Water Level variables, store the distance value in the array
int bottomDistance = 0;
int waterLevel = 0;

// TDS Registers, store the analog value in the array, read from ADC
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

// read data from ultrasonic sensor, in centimeter
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
//sleep function for ESP32
void goingToSleep(){
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Going to Sleep");
  Serial.flush();
  esp_deep_sleep_start(); //code starting from here wont be run
}
//wakeup func
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

// Send data to Firebase
void send_firebase(int tds, float level){
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    // Write an Int number on the database path test/int
    if (Firebase.RTDB.setInt(&fbdo, "test/int", count)){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    count++;
    
    // Write an Float number on the database path test/float
    if (Firebase.RTDB.setFloat(&fbdo, "test/float", 0.01 + random(0,100))){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
}
void setup(){
  Serial.begin(115200);
  print_wakeup_reason(); //for debugging sleep function
  pinMode(TdsSensorPin,INPUT);
  pinMode(probePin,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(resetPin, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor Test");
  //Starting EEPROM
  if(!EEPROM.begin(EEPROM_SIZE)){
    Serial.println("Failed to initialize BOTTOMDIST");
    Serial.println("Restarting");
    delay(1000);
    ESP.restart();
  }
  //Check the last measurement of bottom tank distance, if no data then save new one
  bottomDistance = EEPROM.read(0);
  if(bottomDistance == -1){
    bottomDistance = takeDistance();
    EEPROM.write(0,bottomDistance);
    EEPROM.commit();
    Serial.print("Put new Bottom Distance: ");
    Serial.println(bottomDistance);
  } else {
    Serial.print("Taken Bottom Distance: ");
    Serial.println(bottomDistance);
  }
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "f617fb2ded7eb9a85da9e53ad1a8224d2acff01196b9e6075931d1e8099164a5anon.com", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop(){
  static unsigned long analogSampleTimepoint = millis();
  // measure ppm
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
    }
  }

  //measure water level
  static unsigned long distanceSampleTimepoint = millis();
  if (millis() - distanceSampleTimepoint>4000U){
    distanceSampleTimepoint = millis();
    waterLevel = bottomDistance - takeDistance();
    //the result
    Serial.print("Water Level: ");
    Serial.println(waterLevel);
    Serial.print("Taken Bottom Distance: ");
    Serial.println(bottomDistance);
  }
  // when the reset button pressed, take new bottom distance
  if (!digitalRead(resetPin)){
    bottomDistance = takeDistance();
    EEPROM.write(0,bottomDistance);
    EEPROM.commit();
    Serial.print("Put new Bottom Distance: ");
    Serial.println(bottomDistance);
  }
  // send data to firebase
  send_firebase(tdsValue,waterLevel);
  // going to sleep
  //goingToSleep();
  
}
