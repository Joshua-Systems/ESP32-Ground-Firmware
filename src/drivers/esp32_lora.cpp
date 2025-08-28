#include <LoRa.h>
#include <SPI.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>

// Adjust pin numbers
// These are for the ESP32 C3 DEVKIT
#define LORA_SS 7
#define LORA_RST 0
#define LORA_DIO0 1
#define LORA_FREQ 915E6 // For 915 MHz modules
#define LORA_SPI_CLK 8E6
#define MSB_FIRST 1
#define SPIMODE 0

int beganpacket;

void initLoRa()
{
  // Start SPI communication with LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ))
  {
    printf("LoRa initialization failed!");
  }
  LoRa.begin(LORA_FREQ);
}

void sendLoRaMessage(String message)
{
  // Append a created LoRa packet with message and send it off via SPI

  beganpacket = LoRa.beginPacket(true);
  if (beganpacket)
  {
    LoRa.print(message);
    LoRa.endPacket();
    // endPacket() activates SPI, may need to check SPI pinout
    Serial.println(message);
    Serial.println("HARDWARE TEST POINT SEND LORA");
    // Optionally, you can print a confirmation
    Serial.printf("Sent message: %s\n", message.c_str());
  }
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