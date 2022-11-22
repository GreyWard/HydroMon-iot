#include <WiFi.h>
#include <PubSubClient.h>
//WiFi Credential, adjust
const char *SSID = "wifi_ssid"
const char *PWD = "wifi_pass"
//MQTT Credential
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
char *mqttServer = "broker.hivemq.com";
int mqttPort = 1883;
//device Credential, adjust
const char *deviceId = "ESP32-testing";
void setupMQTT() {
  mqttClient.setServer(mqttServer,mqttPort);
}
void connectWiFi() {
  Serial.print("Connecting to ");
  WiFi.begin(SSID,PWD);
  Serial.println(SSID);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.print("Connected!");
}
void reconnect(){
  Serial.println("Connecting to MQTT Broker");
  String clientId = "client-id"; //adjust
  clientId += String(random(0xffff),HEX);
  if (mqttClient.connect(clientId.c_str())){
    Serial.println("Connected");
    //subscribe the topic
    String topic = "/data/";
    topic += deviceId;
    mqttClient.subscribe(topic);
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
  if(!mqttClient.conected()) reconnect();
  mqttClient.loop();
  // send data to broker (publish)
  String topic = "/data/" + deviceId;
  long lastTime = 0;
  long timeNow = millis();
  if(timeNow - lastTime > 60000){
    int tds = 1000;
    int waterLevel = 20;
    //send tds
    topic += "/tds";
    sprintf(data,"%d",tds);
    Serial.println(data);
    mqttClient.publish(topic,data);
    //send waterLevel
    topic = "/data/" + deviceId + "/waterlvl";
    sprintf(data,"%d",waterLevel);
    Serial.println(data);
    mqttClient.publish(topic,data);
    lastTime = timeNow;
  }
}
