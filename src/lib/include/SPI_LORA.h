#include <SPI.h>
#include <HardwareSerial.h>
#include <cstddef>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

// Choose pins for ESP32-C3 (adjust if needed)//

#define LORA_HOST SPI2_HOST
#define PIN_MISO 4
#define PIN_MOSI 7
#define PIN_SCLK 9
#define PIN_CS 11 // Chip Select (any free GPIO)

// Registers in

void setUpSPI();       // initialise the SPI bus and add the LoRa Hope RF as a device
void singleTransfer(); // new SPI transfer class