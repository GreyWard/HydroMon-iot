# HydroMon-iot
HydroMon is a Hydroponic Monitoring System
This code is built for the IoT Module
Created by Michael Harditya
# Guides
## Blynk Creation
- Create new Template
-- Log in into the Blynk Console and create new Template
-- Define the device as ESP32, and use name as you like it
-- Set Datastream:
<img width="1170" alt="image" src="https://user-images.githubusercontent.com/70849194/209632570-16f138c1-2d4d-4bf7-a079-f5bb3508a959.png">
- Create new Device
-- Open Devices menu, add new Device
-- Choose the template made before
-- Copy the credential to use it inside the code

## Credentials
Our system use Blynk, we need WIFI Password and SSID, also with Blynk credentials
- Blynk Credentials see Blynk after Template creation and Device creation
- Network Credentials,
-- WIFI_SSID (your ssid to wifi access point)
-- WIFI_PASSWORD (your wifi access point's password)

## Wiring
Default pins defined in the code are:
Ultrasonic Sensor Pins:
- Echo Pin : 34
- Trigger Pin : 35
TDS Sensor Pins:
- Sensor Input Pin : 36 / A0

*Updates:
Reset button pin has been integrated to Blynk 
