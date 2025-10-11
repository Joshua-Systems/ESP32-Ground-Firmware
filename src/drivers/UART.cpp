#include "lib/include/SPI_LORA.h"

static HardwareSerial RxFromMCU(0); // Use UART2

bool uartInit(unsigned long baudRate)
{
  Serial2.begin(baudRate, SERIAL_8N1, 16, 17); // RX2 on GPIO16, TX2 on GPIO17
  delay(100);                                  // Give some time to initialize
  return Serial2 ? true : false;
}

bool uartAvailable()
{
  return Serial2.available() > 0;
}

bool uartRead(String &data)
{
  if (uartAvailable())
  {
    data = Serial2.readStringUntil('\n'); // Read until newline character
    return true;
  }
  return false;
}
