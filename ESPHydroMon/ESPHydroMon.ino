// Original source code: https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
// Project details: https://RandomNerdTutorials.com/esp32-tds-water-quality-sensor/

//Libraries
#include "EEPROM.h"

//Instantiate eeprom objects
EEPROMClass BOTTOMDIST("eeprom0");

//Sensor Pins
#define TdsSensorPin 27
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
#define TrigPin 38            // ultrasonic pins
#define EchoPin 39
#define probePin 29           // to release voltage to the water level sensor

//Constants
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  360        /* Time ESP32 will go to sleep (in seconds) */
//RTC_DATA_ATTR int bootCount = 0;

// Water Level Registers, store the distance value in the array
int bottomDistance = 0;
int waterLevel = 0;
int distanceBuffer[SCOUNT];      //distance values from water level sensor
int distanceBufferIndex = 0;

// TDS Registers, store the analog value in the array, read from ADC
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

//Sensor Values
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

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

// water level measuring function (using Ultrasonic Sensor)
int measureWaterLevel(){
  static unsigned long distanceSampleTimepoint = millis();
  // measure distance value
  if(millis()-distanceSampleTimepoint > 400U){     //every 400 milliseconds,read the distance between sensor and water.
    analogSampleTimepoint = millis();
    distanceBuffer[distanceBufferIndex] = takeDistance();    //read the analog value and store into the buffer
    distanceBufferIndex++;
    if(distanceBufferIndex == SCOUNT){ 
      distanceBufferIndex = 0;
    }
  }   
  averageDistance = getMedianNum(distanceBuffer,SCOUNT);
  Serial.println("Average water level:"); // water level debugging
  Serial.print(averageDistance);
  return averageDistance;
}

// read data from ultrasonic sensor, in centimeter
int takeDistance(){
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  int selang = pulseIn(EchoPin,HIGH);
  int jarak = 0.0343 * (selang / 2);
  return jarak;
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
void setup(){
  Serial.begin(115200);
  print_wakeup_reason(); //for debugging sleep function
  pinMode(TdsSensorPin,INPUT);
  pinMode(EchoPin,INPUT);
  pinMode(TrigPin,OUTPUT);
  pinMode(probePin,OUTPUT);
  //Starting EEPROMClass
  if(!BOTTOMDIST.begin(0x500)){
    Serial.println("Failed to initialize BOTTOMDIST");
    Serial.println("Restarting");
    delay(1000);
    ESP.restart();
  }
  //Check the last measurement of bottom tank distance, if no data then save new one
  BOTTOMDIST.get(0,bottomDistance);
  if(bottomDistance == 0){
    bottomDistance = measureWaterLevel();
    BOTTOMDIST.put(0,bottomDistance);
  }
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
      
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
    }
  }

  //measure water level
  waterLevel = bottomDistance - measureWaterLevel();
  //going to sleep
  //goingToSleep();
  
}
