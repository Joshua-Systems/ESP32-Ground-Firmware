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

static const char *TAG = "SPI_LOOPBACK";

void initSPI()
{
  Serial.begin(115200);
}

void transferTest()
{
  SPI_SWAP_DATA_TX(0x12345678, 32);
  Serial.println("Data swapped for TX");
  SPI_SWAP_DATA_RX(0x12345678, 32);
  Serial.println("Data swapped for RX");
}

void loopback()
{
  esp_err_t ret;

  spi_device_handle_t handle;

  spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_MOSI,
      .miso_io_num = PIN_MISO,
      .sclk_io_num = PIN_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 32,
  };
  Serial.println("Initializing SPI bus...");

  ret = spi_bus_initialize(LORA_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  spi_device_interface_config_t devcfg = {
      .mode = 0,                 // SPI mode 0
      .clock_speed_hz = 1000000, // 1 MHz clock speed
      .spics_io_num = PIN_CS,
      .queue_size = 7,
  };
  Serial.println("Configuring self as Device");

  ret = spi_bus_add_device(LORA_HOST, &devcfg, &handle);
  ESP_ERROR_CHECK(ret);

  while (1)
  {
    Serial.println("Initialising Data to Send");
    uint8_t dataToSend[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t recieved[4] = {0};

    spi_transaction_t transfer;

    // Clear the transfer Struct
    Serial.println("Begin Transfer");
    memset(&transfer, 0, sizeof(transfer));
    transfer.length = sizeof(dataToSend) * 8;
    transfer.tx_buffer = dataToSend;
    transfer.rx_buffer = recieved;

    ret = spi_device_transmit(handle, &transfer);
    ESP_ERROR_CHECK(ret);
    if (memcmp(dataToSend, recieved, sizeof(dataToSend)) == 0)
    {
      ESP_LOGI(TAG, "Loopback successful! Sent: 0x%02X 0x%02X 0x%02X 0x%02X, Received: 0x%02X 0x%02X 0x%02X 0x%02X",
               dataToSend[0], dataToSend[1], dataToSend[2], dataToSend[3],
               recieved[0], recieved[1], recieved[2], recieved[3]);
      Serial.println("Loopback successful!");

      Serial.printf("Sent: 0x%02X 0x%02X 0x%02X 0x%02X\n", dataToSend[0], dataToSend[1], dataToSend[2], dataToSend[3]);
      Serial.printf("Received: 0x%02X 0x%02X 0x%02X 0x%02X\n", recieved[0], recieved[1], recieved[2], recieved[3]);
    }
    else
    {
      ESP_LOGE(TAG, "Loopback failed! Sent: 0x%02X 0x%02X 0x%02X 0x%02X, Received: 0x%02X 0x%02X 0x%02X 0x%02X",
               dataToSend[0], dataToSend[1], dataToSend[2], dataToSend[3],
               recieved[0], recieved[1], recieved[2], recieved[3]);
      Serial.println("Loopback failed!");
      Serial.printf("Sent: 0x%02X 0x%02X 0x%02X 0x%02X\n", dataToSend[0], dataToSend[1], dataToSend[2], dataToSend[3]);
      Serial.printf("Received: 0x%02X 0x%02X 0x%02X 0x%02X\n", recieved[0], recieved[1], recieved[2], recieved[3]);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}