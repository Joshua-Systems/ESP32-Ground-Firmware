#include "lib/include/SPI_LORA.h"

void setUpSPI()
{
  extern esp_err_t ret;

  extern spi_device_handle_t handle;

  extern spi_bus_config_t buscfg;

  buscfg = {
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

uint8_t SPITransfer(uint8_t addr, uint8_t data, uint8_t rw)
{
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));

  uint8_t datatosend[2];
  uint8_t received[2] = {0};

  if (rw)
  {
    // READ: MSB = 1, send dummy byte to receive data
    datatosend[0] = addr | 0x80;
    datatosend[1] = 0x00;
  }
  else
  {
    // WRITE: MSB = 0
    datatosend[0] = addr & 0x7F;
    datatosend[1] = data;
  }

  trans.length = sizeof(datatosend) * 8;
  trans.tx_buffer = datatosend;
  trans.rx_buffer = received;

  ret = spi_device_transmit(handle, &trans);
  ESP_ERROR_CHECK(ret);

  if (rw)
  {
    return received[1]; // register value
  }
  else
  {
    return 0; // nothing useful to return on write
  }
}

void initLoRa()
{
}

void configureSingleReception()
{

  uint8_t Regi;
  uint8_t LoRa = LoRaMode;
  uint8_t RxSing = RXSingle;
  uint8_t Base = FifoRxBaseAddr;

  // Set RegFifoPtrToBase
  SPITransfer(RegFifoAddrPtr, Base, 0);

  // Configure the appropriate Dio pins
  uint8_t DIOCFG = (RxDone << 7) | (PayloadCRCError);
  SPITransfer(REGMAP1, DIOCFG, 0);

  // Bit shift to set LoRa and set_Mode_Single
  Regi = (LoRa << 7) | (RxSing);
  SPITransfer(RegOpMode, Regi, 0);
}

uint8_t Rx()
{
  uint8_t irqFlags;
  do
  {
    irqFlags = SPITransfer(RegIrqFlag, 0x00, 1); // read register
  } while ((irqFlags & (1 << LRxDone)) == 0); // wait for RxDone bit

  if (irqFlags & (1 << LPayloadCRCError))
  {
    Serial.println("CRC error!");
    return 0; // bad packet
  }

  // Clear IRQ flags
  SPITransfer(RegIrqFlag, 0xFF, 0);

  // Read FIFO
  uint8_t len = SPITransfer(RegFifo, 0x00, 1);
  Serial.printf("Packet length = %d\n", len);

  return len; // placeholder, later you can fetch the payload from FIFO
}
