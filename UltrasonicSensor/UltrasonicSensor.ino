#define echoPin 2
#define trigPin 3

long duration;
int distance;

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor Test");
}

void loop() {
  // put your main code here, to run repeatedly:
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
  //the result
  Serial.print("Distance: ");
  Serial.print(distance);

}
