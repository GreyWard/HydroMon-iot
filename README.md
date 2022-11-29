# HydroMon-iot
HydroMon is a Hydroponic Monitoring System
This code is built for the IoT Module
Created by Michael Harditya
# Guides
## Credentials
Our system use Firebase as server, so we need few identifiers to work with
- Firebase project API Key, defined as API_KEY, can be found in Firebase Console > Project Settings > Web API Key
- RealTime Database URL, defined as DATABASE_URL, can be found in FirebaseConsole > Realtime Database > Data
- Network Credentials,
-- WIFI_SSID (your ssid to wifi access point)
-- WIFI_PASSWORD (your wifi access point's password)

## Wiring
Default pins defined in the code are:
Ultrasonic Sensor Pins:
- Echo Pin : 34
- Trigger Pin : 35
TDS Sensor Pins:
- Sensor Input Pin : 27
- Probe Output Pin : 29
Reset Pin for Bottom Distance: 14
