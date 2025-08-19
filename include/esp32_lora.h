#include <StreamString.h>
#include <WString.h>
#define LORA_H
void initLoRa();                            // Call once in setup
void sendLoRaMessage(String message);       // Send message
String receiveLoRaMessage(String &message); // Receive message, if available
