#include <Arduino.h>
#include <String.h>
#include <StreamString.h>
#include <WString.h>
#include "lib/include/SPI_LORA.h"
#include "lib/include/SPI_TEST.h"

int testPoint = 0;
String Message;
String Flag;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1);
  setUpSPI();
  ConfDIO();
  fuckingWithDIO();
  LoRaConfig();

  // Clear IRQ flags

  Serial.println("LoRa SPI initialized and configured.");
  Serial.printf("Status: 0x%02X", readFromReg(RegOpMode));
  Serial.println(" - Should be 0x81 (Standby Mode)");

  //
}

void loop()
{
  /*const char *msg = "Hello LoRa";
  TxConf((const uint8_t *)msg, (uint8_t)strlen(msg));*/
  // delay(500);
  // Serial.println("Attempting first receive...");
  String received = RxMessage();
  // String receivedTimeout = RxMessageWithTimeout(10000); // 10 second timeout
  // String receivedTimeout2 = RxMessageWithTimeout(10000); // 10 second timeout

  if (received.length() > 0 && received != 0)
  {
    Serial.print("Received message: ");
    Serial.println(received);
  }
  else
  {
    Serial.println("No packet or CRC error");
  }

  // wait 5 seconds between polls*/
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly