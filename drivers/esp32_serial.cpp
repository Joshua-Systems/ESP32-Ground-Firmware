#include <StreamString.h>
#include <WString.h>
#include <HardwareSerial.h>

// Serial Bridge for ESP32 and Computer
//  This code allows the ESP32 to communicate with a computer via serial
//  It can be used to send and receive data between the ESP32 and the computer

void transmitData(const String &data)
{
  // This function should send data to the serial port
  if (Serial) // Check if Serial is available
  {
    Serial.println(data); // Send data followed by a newline
  }
  else
  {
    Serial.println("Serial not available");
  }
}
void receiveData(String &data)
{
  // This function should read data from the serial port and store it in 'data'
  if (Serial.available())
  {
    data = Serial.readStringUntil('\n'); // Read until newline character
  }
  else
  {
    data = ""; // No data available
  }
}