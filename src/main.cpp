#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "esp32_lora.h"
#include "esp32_connect.h"

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  initLoRa();                                  //
  initWifi();                                  // Initialize WiFi
  connectToWifi("your_ssid", "your_password"); // Connect to WiFi
  Serial.println("LoRa and WiFi initialized");
}

void loop()
{
  // put your main code here, to run repeatedly:
  sendLoRaMessage("Hello, LoRa!");
  String receivedMessage;
  String result = receiveLoRaMessage(receivedMessage);
  if (result != "No message received")
  {
    Serial.println("Received: " + receivedMessage);
  }
  else
  {
    Serial.println("No new messages.");
  }
  delay(1000);
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly