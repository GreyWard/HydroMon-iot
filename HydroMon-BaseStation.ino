/* ESP32 IoT Module from HydroMon
 * by Michael Harditya
 Outsourcing: 
 https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
 https://RandomNerdTutorials.com/esp32-tds-water-quality-sensor/
 https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
 https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/
#define BLYNK_TEMPLATE_ID "TMPLcHtn5Avo"
#define BLYNK_DEVICE_NAME "Hydromon Template"
#define BLYNK_AUTH_TOKEN "MAIgQDV71UPoVE9L8KjSrOiDPTCcZOQK"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define AP_SSID = "B4seSt@tion";
#define AP_PASS = "BASEPass";
#define STA_SSID = "Hayo";
#define STA_PASS = "mikeh123";

TaskHandle_t Task1;
WiFiServer server(80);
WiFiClient client;
BlynkTimer timer;

float tdsValue;
float waterLevel;

void onDataReceived(int packetSize) {
  // Read the data from the client
  String data = client.readStringUntil('\n');
  // Print the data to the serial monitor
  Serial.println(data);
  // Process the data here
  int index = data.indexOf(";");
  int length = data.length();
  String tds = data.substring(0,index);
  String water = data.substring(index+1,length);
  tdsValue = tds.toFloat();
  waterLevel = water.toFloat();
}

void dataProcess() {
  Blynk.virtualWrite(V0,tdsValue);
  Blynk.virtualWrite(V1,waterLevel);
}

void listenTask(void *pvParameter){
Serial.print("Listener Task running on core ");
Serial.println(xPortGetCoreID());  
for(;;){
    // Check for a new client
    client = server.available();
    if (client) {
      Serial.println("New client");
      // Set the callback function
      client.setNoDelay(true);
      client.setTimeout(0);
      client.onData(onDataReceived);
    }
  }
}

void setup() {
  Serial.begin(115200);
  //Set ESP as AP and Station
  WiFi.mode(WIFI_MODE_APSTA);
  //Initiate AP
  WiFi.softAP(AP_SSID,AP_PASS);
  //Initiate Station
  WiFi.begin(STA_SSID,STA_PASS);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("ESP32 IP as soft AP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("ESP32 IP on the WiFi network: ");
  Serial.println(WiFi.localIP());
  // Start the server
  server.begin();
  Serial.println("Server started");
  //Initiate Listener Task
xTaskCreatePinnedToCore(
  listenTask,
  "Listener Task",
  10000,
  NULL,
  1,
  &Task1,
  0);
  delay(500);
}

void loop() {
  timer.run();
  Blynk.run();
}