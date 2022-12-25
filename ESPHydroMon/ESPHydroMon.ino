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
#include "time.h"
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

/* Device Credentials */
// Insert Firebase project API Key
#define API_KEY "AIzaSyC5sUXHZKiEUR9CL6PI54x314pI8AE3BXI"

// Insert RTDB URL
#define DATABASE_URL "https://hydromon-backend-default-rtdb.asia-southeast1.firebasedatabase.app/" 

// Insert your network credentials
#define WIFI_SSID "Hayo"
#define WIFI_PASSWORD "mikeh123"

// Defining Pins
#define echoPin 18
#define trigPin 19
#define resetPin 14
#define TdsSensorPin 13
#define probePin 12

// Constants
#define EEPROM_SIZE 1
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  43200        /* Time ESP32 will go to sleep (in seconds) */
#define VREF 3.3              /* analog reference voltage(Volt) of the ADC */
#define SCOUNT  30            /* sum of sample point */
#define USER_EMAIL "hydromontest1@gmail.com"
#define USER_PASSWORD "password"
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
bool nextReset = false;
bool buttonRelease = false;

// TDS Registers, store the analog value in the array, read from ADC
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
int timestamp;
String uid;
unsigned long sendDataPrevMillis = 0;
//int count = 0;
bool signupOK = false;

//database paths
String databasePath;
String tdsPath = "/tds";
String levelPath = "/level";
String timePath = "/timestamp";

//Parent node
String parentPath;

//NTP Server to request epoch time
const char* ntpServer = "pool.ntp.org";

//Variable to save current epoch time
unsigned long epochTime;

/* sleep function for ESP32
 * can be woke up by interrupt pin (14) or time until 1 hour
 * param none
 * return none
 */
void goingToSleep(){
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,0); //1 = High, 0 = Low
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Going to Sleep");
  Serial.flush();
  esp_deep_sleep_start(); //code starting from here wont be run
}
/* Wakeup function for ESP32
 * for debugging purposes, shows what woke the ESP32
 */
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
    Serial.println("Wakeup caused by external signal using RTC_IO"); 
    //if Reset button pressed then it will go Reset State
    nextReset = true;
    break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default : Serial.printf("Wakeup was caused by other: %d\n",wakeup_reason); break;
  }
}
/* Firebase initialization
 * open connection to Firebase database
 */
void firebase_init(){
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
  /* Log In */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

/*  if (Firebase.signUp(&config, &auth, "f617fb2ded7eb9a85da9e53ad1a8224d2acff01196b9e6075931d1e8099164a5anon.com", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }*/
  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
   // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;
  Firebase.begin(&config, &auth);
  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
  Firebase.reconnectWiFi(true);
}
/* Send data to Firebase
 * send two data, tds and water level to Firebase
 * @param tds(int)
 * @param level(float)
 */
void send_firebase(int tds, float level, long epochTime){
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    databasePath = uid + "/data";
    timestamp = getTime();
    Serial.print("time: ");
    Serial.println(timestamp);

    json.set(tdsPath.c_str(), String(tds));
    json.set(levelPath.c_str(), String(level));
    json.set(timePath.c_str(), String (timestamp));
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, databasePath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
  }
}
/* EEPROM initialization
 * saves distance between sensor to the bottom of the tank
 */
 void eeprom_init(){
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
    Serial.print("Put new Bottom Distance (no memory content found): ");
    Serial.println(bottomDistance);
  } else {
    Serial.print("Taken Bottom Distance: ");
    Serial.println(bottomDistance);
  }
 }

//Function to fetch epoch time fron NTP Server
unsigned long getTime(){
  time_t now;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}
 
/* median filtering algorithm
 * to get representative data from 30 measurements
 * @param bArray[](int)
 * @param iFilterLen(int)
 */
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

void setup(){
  Serial.begin(9600);
  print_wakeup_reason(); //for debugging sleep function
  pinMode(TdsSensorPin,INPUT);
  pinMode(probePin,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(resetPin, INPUT_PULLUP);
  Serial.println("Ultrasonic Sensor Test");
  //Starting EEPROM
  eeprom_init();
  //Starting Firebase connection
  firebase_init();
  //init time
  configTime(0,0,ntpServer);
}

void loop(){
  static unsigned long analogSampleTimepoint = millis();
  // when the reset button pressed, take new bottom distance
  if (!digitalRead(resetPin)&& !buttonRelease){
    bottomDistance = takeDistance();
    EEPROM.write(0,bottomDistance);
    EEPROM.commit();
    Serial.print("Put new Bottom Distance: ");
    Serial.println(bottomDistance);
    resetState = true;
    buttonRelease = false;
  }
  else if (digitalRead(resetPin) && !buttonRelease){
    buttonRelease = true;
  }
  else if ((!digitalRead(resetPin) && buttonRelease) || !resetState){
    resetState = false;
    buttonRelease = false;
    // measure ppm
    if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
      analogBufferIndex++;
      if(analogBufferIndex == SCOUNT){ 
        analogBufferIndex = 0;
      }
    }
    // after 30 tds value measurements
    static unsigned long printTimepoint = millis();
    if(millis()-printTimepoint > 1200U){
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
      
    
      //measure water level
      waterLevel = bottomDistance - takeDistance();
      //the result
      Serial.print("Water Level: ");
      Serial.println(waterLevel);
      Serial.print("Taken Bottom Distance: ");
      Serial.println(bottomDistance);

      //get time
      epochTime = getTime();
      Serial.print("Epoch Time: ");
      Serial.println(epochTime);
      
      // send data to firebase
      send_firebase(tdsValue,waterLevel, epochTime);
  
      // going to sleep
      goingToSleep();
    }
  }
  else if(nextReset){
   bottomDistance = takeDistance();
   EEPROM.write(0,bottomDistance);
   EEPROM.commit();
   Serial.print("Put new Bottom Distance (Reset Pressed): ");
   Serial.println(bottomDistance);
   delay(1000);
 }
}
