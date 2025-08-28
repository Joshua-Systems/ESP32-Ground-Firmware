#include <SPI.h>
#include <Arduino.h>

#define SCK_SPI 4
#define MISO_SPI 5
#define MOSI_SPI 6
#define SS_SPI 7

#define MSB_FIRST 1
#define SPI_CLK 8E6
#define SPI_MODE0 0

SPIClass *LoRaSPI = nullptr;

void SPIinit()
{
  LoRaSPI = new SPIClass();
  LoRaSPI->begin(SCK_SPI, MISO_SPI, MOSI_SPI, SS_SPI);

  pinMode(MISO_SPI, OUTPUT);
  digitalWrite(MISO_SPI, LOW);
  pinMode(MOSI_SPI, OUTPUT);
  digitalWrite(MOSI_SPI, LOW);

  pinMode(SS_SPI, OUTPUT);
  digitalWrite(SS_SPI, HIGH);
}

void SPISendLoopBackTest()
{
  // Connect ESP SPI MOSI->MISO
  byte dataToSend = 0xA5;
  byte recievedData = 0x00;

  LoRaSPI->beginTransaction(SPISettings(SPI_CLK, MSB_FIRST, SPI_MODE0));

  digitalWrite(SS_SPI, LOW);

  recievedData = LoRaSPI->transfer(dataToSend);

  LoRaSPI->endTransaction();

  if (dataToSend == recievedData)
  {
    Serial.println("Success!");
  }
  else
  {
    Serial.println("Failure");
  }
}