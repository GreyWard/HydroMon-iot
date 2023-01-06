// BaseStation

#define BLYNK_TEMPLATE_ID "TMPLcHtn5Avo"
#define BLYNK_DEVICE_NAME "Hydromon Template"
#define BLYNK_AUTH_TOKEN "MAIgQDV71UPoVE9L8KjSrOiDPTCcZOQK"
#include <BlynkSimpleEsp32.h>
#include <ESPAsyncWebServer.h>
#include "WiFi.h"

AsyncWebServer server(80);

#define STA_SSID "Hayo"
#define STA_PASS "mikeh123"
// AP as Node BaseStation
#define AP_SSID "BaS3Station"
#define AP_PASS "Bas3b@ll"

BlynkTimer timer;
int n = 2;
IPAddress savedIP[2];
IPAddress tempIP;
int tds = 0;
int water = 0;
String callibrate = "0";
//send the data
void dataProcess() {
  int i = 0;
  for (i = 0; i<n;i++){
      if(savedIP[i] == tempIP){
        break;
      }
      else if(i+1 == n){
        savedIP[i] = tempIP;
      }
    }
  Blynk.virtualWrite(i,tds);
  Blynk.virtualWrite((i+1),water);
}
//Reset Button Called from Blynk, only for debugging
BLYNK_WRITE(V2){
  callibrate = param.asString();
  Serial.println(callibrate);
}

void setup() {
  Serial.begin(115200);
  //Blynk initialization
  timer.setInterval(1000L,dataProcess);
  Blynk.begin(BLYNK_AUTH_TOKEN,STA_SSID,STA_PASS);
  //Async Server Initialization
  Serial.print("Setting AP ...");
  WiFi.softAP(AP_SSID,AP_PASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.println(IP);
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Handle POST request
    // Read POST data
    tempIP = request->client()->remoteIP();
    tds = request->getParam("tds",true)->value().toInt();
    water = request->getParam("water",true)->value().toInt();
    Serial.printf("Fetched data: tds = %d, water = %d\n",tds,water);
    request->send(200,"post/String",callibrate);
  });
  server.begin();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  timer.run();
  Blynk.run();
}