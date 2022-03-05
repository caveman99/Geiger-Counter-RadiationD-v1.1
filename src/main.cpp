/*
   Geiger.ino

   This code interacts with the Alibaba RadiationD-v1.1 (CAJOE) Geiger counter board
   and reports readings in CPM (Counts Per Minute).
   Connect the output of the Geiger counter to pin inputPin.

   Author: Andreas Spiess
   Based on initial work of Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
   Hacked by @crazyquark for Adafruit.io
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
#define PERIOD_LOG 5          // Logging period
#define PERIOD_ADAFRUIT_IO 20 // in seconds

const int inputPin = 12;

volatile unsigned long counts = 0; // Tube events
float cpm = 0;                     // CPM
float microSvHour = 0;             // uSV/h
int lastCounts = 0;
unsigned long lastCountTime; // Time measurement
unsigned long lastSend = 0;

// set up the 'cpm' feed
AdafruitIO_Feed *cpm_io = io.feed("cpm");

void IRAM_ATTR ISR_impulse()
{ // Captures count of events from Geiger counter board
  counts++;
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  esp_restart();
}

void setup()
{
  Serial.begin(115200);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  pinMode(inputPin, INPUT);                            // Set pin for capturing Tube events
  attachInterrupt(inputPin, ISR_impulse, FALLING);
  Serial.println("Initialized");
}

void loop()
{
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  unsigned long currentTime = millis();
  if (currentTime - lastCountTime > (PERIOD_LOG * 1000))
  {
    Serial.print("Counts: ");
    Serial.println(counts);
    cpm = float(counts - lastCounts) / PERIOD_LOG;
    microSvHour = cpm / 151;
    lastCounts = counts;
    lastCountTime = millis();
    
    Serial.print("cpm: ");
    Serial.println(cpm);

    Serial.print("uSV/h: ");
    Serial.println(microSvHour);

    if ((currentTime - lastSend) > PERIOD_ADAFRUIT_IO * 1000)
    {
      // save cpm to the 'cpm' feed on Adafruit IO
      Serial.print("sending -> ");
      Serial.println(cpm);
      cpm_io->save(cpm);

      lastSend = currentTime;
    }
  }
}
