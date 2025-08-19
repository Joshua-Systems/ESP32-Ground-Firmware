#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "esp32_lora.h"

void setup()
{
  // put your setup code here, to run once:
  initLoRa(); //
}

void loop()
{
  // put your main code here, to run repeatedly:
  sendLoRaMessage("Hello, LoRa!");
  String receivedMessage;
  String result = receiveLoRaMessage(receivedMessage);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}