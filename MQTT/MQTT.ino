#include <WiFi.h>
#include <PubSubClient.h>
//WiFi Credential, adjust
const char *SSID = "yak2";
const char *PWD = "4R67$8r2";
//MQTT Credential
WiFiClient wifiClient;
PubSubClient client(wifiClient);
char *mqttServer = "c0c46a42f7444eb19e7fc7bcda2cd313.s2.eu.hivemq.cloud";
int mqttPort = 1883;
//device Credential, adjust
const char* deviceId = "ESP32-testing";
//message buffer
#define MSG_BUFFER_SIZE  50
char datamsg[MSG_BUFFER_SIZE];
void setupMQTT() {
  client.setServer(mqttServer,mqttPort);
}
void connectWiFi() {
  Serial.print("Connecting to ");
  WiFi.begin(SSID,PWD);
  Serial.println(SSID);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected!");
  Serial.println(WiFi.localIP());
}
void reconnect(){
  Serial.println("Connecting to MQTT Broker");
  String clientId = "client-id"; //adjust
  clientId += String(random(0xffff),HEX);
  if (client.connect(clientId.c_str())){
    Serial.println("Connected");
    //subscribe the topic
    char* topic = "/data/";
    topic = strcat(topic,deviceId);
    client.subscribe(topic);
  }
  
}
void setup() {
  // put your setup code here, to run once:
  connectWiFi();

}

void loop() {
  // set MQTT server
  setupMQTT();
  // check conectivity, if it is not connected, then it will try to reconnect
  if(!client.connected()) reconnect();
  client.loop();
  // send data to broker (publish)
  char* topic = strcat("/data/",deviceId);
  long lastTime = 0;
  long timeNow = millis();
  if(timeNow - lastTime > 60000){
    int tds = 1000;
    int waterLevel = 20;
    //send tds
    topic = strcat(topic,"/tds");
    sprintf(datamsg,"%d",tds);
    Serial.println(datamsg);
    client.publish(topic,datamsg);
    //send waterLevel
    topic = strcat("/data/",deviceId);
    topic = strcat(topic,"/waterlvl");
    sprintf(datamsg,"%d",waterLevel);
    Serial.println(datamsg);
    client.publish(topic,datamsg);
    lastTime = timeNow;
  }
}
