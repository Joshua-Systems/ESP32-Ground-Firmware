#include <StreamString.h>
#include <WString.h>
#include <WiFi.h>
#include <WifiUDP.h>

#ifndef ESP32_CONNECT_H
#define ESP32_CONNECT_H

void initWifi()
{
  // Initialize WiFi
  // Can also be used as a reset function.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Ensure we start with a clean state
  delay(100);
  Serial.println("WiFi initialized");
  // Optionally, you can set a hostname
  // WiFi.setHostname("ESP32_Device");
  Serial.println("WiFi mode set to STA (Station)");
  // You can also enable AP mode if needed
  // WiFi.enableAP(true);
  // Serial.println("AP mode enabled");
  // Note: You can also set up event handlers for WiFi events if needed
}

void connectToWifi(const char *ssid, const char *password)
{
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
}

#endif // ESP32_CONNECT_H