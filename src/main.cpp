#include <Arduino.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "lib/include/SPI_LORA.h"
#include "lib/include/SPI_TEST.h"
#include "lib/include/UART.h"
#include <string>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <queue>

int testPoint = 0;
String Message;
String Flag;

std::queue<String> MessageQueue;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1);

  setUpSPI();
  ConfDIO();
  fuckingWithDIO();
  LoRaConfig();
  uartInit(115200);

  // Clear IRQ flags

  Serial.println("LoRa SPI initialized and configured.");
  Serial.printf("Status: 0x%02X", readFromReg(RegOpMode));
}
uint8_t payload[32] = {0};
void loop()
{
  /*const char *msg2 = " XXXXXXXXXXXXXXXYURIA YURIA WHY??? YURIA YURIA WHY??? YURIA YURIA WHY??? YURIA YURIA WHY???";
  TxConf((const uint8_t *)msg2, strlen(msg2));
  Serial.println("Escape2d");*/
  if (uartRead(Message))
  {
    Serial.print("Received over UART: ");
    Serial.println(Message);
    Message.trim(); // Remove any leading/trailing whitespace
    if (Message.length() > 0)
    {
      MessageQueue.push(Message);
    }
  }
  String msg = "XXXXXXXXXXXXXXX"; // Preamble Buffer
  msg += MessageQueue.empty() ? "No Message" : MessageQueue.front();
  if (!MessageQueue.empty())
  {
    MessageQueue.pop();
  }
  // Convert String to C-style byte array
  uint8_t msgBuffer[32] = {0};
  size_t msgLen = msg.length();
  msgLen = (msgLen > 31) ? 31 : msgLen; // Reserve space for null terminator if needed
  memcpy(msgBuffer, msg.c_str(), msgLen);
  TxMut(msgBuffer, 32);
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly