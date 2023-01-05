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

int tds = 0;
int water = 0;
String measure = "0";
//send the data
void dataProcess() {
  Blynk.virtualWrite(V0,tds);
  Blynk.virtualWrite(V1,water);
}
//Reset Button Called from Blynk, only for debugging
BLYNK_WRITE(V2){
  measure = param.asString();
  Serial.println(measure);
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
    tds = request->getParam("tds",true)->value().toInt();
    water = request->getParam("water",true)->value().toInt();
    Serial.printf("Fetched data: tds = %d, water = %d\n",tds,water);
    request->send(200);
  });
  server.begin();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  timer.run();
  Blynk.run();
}