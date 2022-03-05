/*
   Geiger.ino

   This code interacts with the Alibaba RadiationD-v1.1 (CAJOE) Geiger counter board
   and reports readings in CPM (Counts Per Minute).
   Connect the output of the Geiger counter to pin inputPin.

   Author: Andreas Spiess
   Based on initial work of Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
   License: MIT License
   Please use freely with attribution. Thank you!
*/


#define PRINT_DEBUG_MESSAGES

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// WiFi pass and such
#include "config.h"

#define WIFI_TIMEOUT_DEF 30
#define PERIOD_LOG 5                  //Logging period 
#define PERIOD_THINKSPEAK 60        // in seconds, >60

IPAddress ip;

WiFiClient client;

WiFiClientSecure secure_client;

const int inputPin = 12;

volatile unsigned long counts = 0;                       // Tube events
float cpm = 0;                                           // CPM
int lastCounts = 0;
unsigned long lastCountTime;                             // Time measurement
unsigned long lastEntryThingspeak;

void IRAM_ATTR ISR_impulse() { // Captures count of events from Geiger counter board
  counts++;
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  esp_restart();
}

void setup() {
  Serial.begin(115200);

  if (PERIOD_LOG > PERIOD_THINKSPEAK) {
    Serial.println("PERIOD_THINKSPEAK has to be bigger than PERIODE_LOG");
    while (1);
  }

  Serial.println("Connecting to Wi-Fi");

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int wifi_loops = 0;
  int wifi_timeout = WIFI_TIMEOUT_DEF;
  while (WiFi.status() != WL_CONNECTED) {
    wifi_loops++;
    Serial.print(".");
    delay(500);
    if (wifi_loops > wifi_timeout) software_Reset();
  }
  Serial.println();
  Serial.println("Wi-Fi Connected");
  pinMode(inputPin, INPUT);                            // Set pin for capturing Tube events
  attachInterrupt(inputPin, ISR_impulse, FALLING);     // Define interrupt on falling edge
  lastEntryThingspeak = millis();
  lastCountTime = millis();
  Serial.println("Initialized");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) software_Reset();

  if (millis() - lastCountTime > (PERIOD_LOG * 1000)) {
    Serial.print("Counts: "); Serial.println(counts);
    cpm = float(counts - lastCounts) / PERIOD_LOG ;
    lastCounts = counts;
    lastCountTime = millis();

    Serial.print("cpm: "); Serial.println(cpm);
  }
}
