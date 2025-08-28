#include <Arduino.h>
#include <queue>
#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "lib\include\esp32_lora.h"

int testPoint = 0;
String Message;
String Flag;

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
  /*digitalWrite(10, HIGH);
  sys_delay_ms(1000);
  sendLoRaMessage("TEST");
  digitalWrite(10, LOW);
  testPoint++;
  Serial.println(testPoint);*/

  sys_delay_ms(5000);
  Message = "";
  Flag = receiveLoRaMessage(Message);
  Serial.println("Flag =" + Flag);

  // TEST SPI TRANSFER
  uint8_t spInt = singleTransfer(0, 0x11);
  Serial.println("");
  Serial.println("Sent: " + spInt);
  testPoint++;
  Serial.println(testPoint);
  sys_delay_ms(5000);
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly