#include <StreamString.h>
#include <WString.h>
#ifndef LORA_H
#define LORA_H
void initLoRa();                            // Call once in setup
void sendLoRaMessage(String message);       // Send message
String receiveLoRaMessage(String &message); // Receive message, if available

#endif