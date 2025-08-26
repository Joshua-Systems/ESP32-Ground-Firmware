#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>

// Adjust pin numbers
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQ 915E6 // For 915 MHz modules

// These above parameters will need tweaking

void initLoRa()
{
  // TODO: Set LoRa pins and call LoRa.begin()
  // Start SPI communication with LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ))
  {
    printf("LoRa initialization failed!");
  }
}

void sendLoRaMessage(String message)
{
  // Append a created LoRa packet with message and send it off
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  // Optionally, you can print a confirmation
  printf("Sent message: %s\n", message.c_str());
}

String receiveLoRaMessage(String &message)
{
  // TODO: Check if a packet is available
  // If yes, read into 'message' and return true
  // If no, return false
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    message = "";
    while (LoRa.available())
    {
      message += (char)LoRa.read();
    }
    // Optionally, print the received message
    printf("Received message: %s\n", message.c_str());
    return message;
  }
  else
  {
    // No packet available
    return "No message received";
  }
}