//purpose of this code is to take the initial distance, then take another one and check
//the water level

//Libraries
#include "EEPROM.h"

//Instantiate eeprom object
EEPROMClass BOTTOMDIST("eeprom0");

#define echoPin 2
#define trigPin 3
#define SCOUNT 30

//Constants
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  360        /* Time ESP32 will go to sleep (in seconds) */
//RTC_DATA_ATTR int bootCount = 0;

//Basic variables for reading distance
long duration;
int distance;

//Water Level variables
int bottomDistance = 0;
int waterLevel = 0;
int distanceBuffer[SCOUNT];
int distanceBufferIndex = 0;
int averageDistance = 0;

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
    distanceSampleTimepoint = millis();
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
  //reset the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int selang = pulseIn(echoPin,HIGH);
  int jarak = 0.0343 * (selang / 2);
  return jarak;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor Test");
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
  Serial.print("Bottom distance: ");
  Serial.println(bottomDistance);
}

void loop() {
  waterLevel = bottomDistance - measureWaterLevel();
  Serial.print("Water Level: ");
  Serial.print(waterLevel);
  Serial.println("cm");

}
