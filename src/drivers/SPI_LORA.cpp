#include "lib/include/SPI_LORA.h"

void setUpSPI()
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
      .mode = 2,                 // SPI mode 0
      .clock_speed_hz = 1000000, // 1 MHz clock speed
      .spics_io_num = PIN_CS,
      .queue_size = 7,
  };
  Serial.println("Configuring self as Device");

  ret = spi_bus_add_device(LORA_HOST, &devcfg, &handle);
  ESP_ERROR_CHECK(ret);
}
