#include <Arduino.h>
#include <queue>
#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "lib\include\esp32_lora.h"
#include "lib\include\SPI_Drive.h"

int testPoint = 0;
String Message;
String Flag;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1);
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  initSPI();
  //                                    // Initialize WiFi
}

void loop()
{
  // put your main code here, to run repeatedly
  /*sys_delay_ms(1000);
  sendLoRaMessage("TEST");
  digitalWrite(10, LOW);
  testPoint++;
  Serial.println(testPoint);

  Message = "";
  Flag = receiveLoRaMessage(Message);
  Serial.println("Flag =" + Flag);*/

  digitalWrite(0, LOW);
  sys_delay_ms(1000);
  digitalWrite(0, HIGH);
  sys_delay_ms(1000);
  loopback();
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly