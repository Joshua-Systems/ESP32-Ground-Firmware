#include <StreamString.h>
#include <WString.h>

// Serial Bridge for ESP32 and Computer
//  This code allows the ESP32 to communicate with a computer via serial
//  It can be used to send and receive data between the ESP32 and the computer

void transmitData(const String &data);
void receiveData(String &data);