#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "lib\include\esp32_lora.h"

void setup()
{
  pinMode(10, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);
  initLoRa(); // This Hangs the output
  //                                   // Initialize WiFi
}

void loop()
{
  // put your main code here, to run repeatedly
  sendLoRaMessage("LoRa Test Message");
  delay(500);
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly