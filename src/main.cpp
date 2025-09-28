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
  Serial.printf(" Value: 0x%02X \n", ReadDIO());
  readFactoryRegisters();
  Serial.printf(" Value: 0x%02X \n", ReadDIO());
  TestLoraEspCommSPI();
  Serial.printf(" Value: 0x%02X \n", ReadDIO());
  fuckingWithDIO();

  //
}

void loop()
{
  // put your main code here, to run repeatedly
  /*sys_delay_ms(1000);
  sendLoRaMessage("TEST");
  digitalWrite(10, LOW);
  testPoint++;
  Serial.println(testPoint);

  Message = "";
  Flag = receiveLoRaMessage(Message);
  Serial.println("Flag =" + Flag);*/
  // Rx();

  Serial.printf("Dio Value: 0x%02X \n", ReadDIO());
  Serial.printf("Reg Value: 0x%02X \n", readFromReg(RegOpMode));
}

// put function definitions here:
// The ESP32 should be polling for messages in the loop
// There should be an interrupt triggered if there is a message recieved
// The interrupt should call receiveLoRaMessage
// How do we detect if a message is received?

// Note: The LoRa library handles the SPI communication and packet management
// Make sure to adjust the LoRa parameters (frequency, pins) as needed for your setup
// This code assumes that the LoRa module is connected and configured correctly