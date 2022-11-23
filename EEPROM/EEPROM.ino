#define echoPin 2
#define trigPin 0
#define resetPin 15

#include "EEPROM.h"

#define EEPROM_SIZE 1

long duration;
int distance;

// Water Level Registers, store the distance value in the array
int bottomDistance = 0;
int waterLevel = 0;

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
void setup() {
  // put your setup code here, to run once:
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
}

void loop() {
  // put your main code here, to run repeatedly:
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
  if (!digitalRead(resetPin)){
    bottomDistance = takeDistance();
    EEPROM.write(0,bottomDistance);
    EEPROM.commit();
    Serial.print("Put new Bottom Distance: ");
    Serial.println(bottomDistance);
  }
}
