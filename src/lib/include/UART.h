// TX2 on ESP32 DEV V1 Kit

#ifndef UART_H
#define UART_H

#include <Arduino.h>
#include <HardwareSerial.h>

bool uartInit(unsigned long baudRate);
bool uartAvailable();
bool uartRead(String &data);

#endif // UART_H