#include <StreamString.h>
#include <WString.h>
#include <WiFi.h>
#include <WifiUDP.h>

#ifndef ESP32_CONNECT_H
#define ESP32_CONNECT_H

void initWifi();
void connectToWifi(const char *ssid, const char *password);

#endif // ESP32_CONNECT_H