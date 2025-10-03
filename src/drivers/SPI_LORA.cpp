#include "lib/include/SPI_LORA.h"
#include "driver/gpio.h"

spi_bus_config_t buscfg = {0};
esp_err_t ret = ESP_OK;
spi_device_handle_t handle = nullptr;

void setUpSPI()
{

  // Initialize all fields of buscfg individually

  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.miso_io_num = PIN_MISO;
  buscfg.sclk_io_num = PIN_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

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
}

uint8_t SPITransfer(uint8_t addr, uint8_t data, uint8_t rw)
{
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));

  uint8_t datatosend[2];
  uint8_t received[2] = {0};

  if (rw)
  {
    // WRITE: MSB = 1, send dummy byte to receive data
    datatosend[0] = addr | 0x80;
    datatosend[1] = data;
  }
  else
  {
    // WRITE: MSB = 0
    datatosend[0] = addr & 0x7F;
    datatosend[1] = 0x00;
  }

  trans.length = sizeof(datatosend) * 8;
  trans.tx_buffer = datatosend;
  trans.rx_buffer = received;

  ret = spi_device_transmit(handle, &trans);
  ESP_ERROR_CHECK(ret);

  if (rw == 1)
  {
    return 0;
  }
  else
  {
    return received[1];
  }
}

void TestLoraEspCommSPI()
{
  uint8_t writeVal = 0xFF;                   // any safe value
  writeVal = SPITransfer(0x42, writeVal, 1); // write to register

  uint8_t readVal = SPITransfer(0x42, 0x00, 0);
  delay(10); // small delay to ensure SPI transaction completes

  Serial.printf("Written: 0x%02X, Read back: 0x%02X\n", writeVal, readVal);

  if (writeVal == readVal)
  {
    Serial.println("SPI write/read SUCCESS!");
  }
  else
  {
    Serial.println("SPI write/read FAILED!");
  }
}

void readFactoryRegisters()
{
  uint8_t version = SPITransfer(0x42, 0x00, 0); // RegVersion
  Serial.printf("LoRa Version register: 0x%02X\n", version);
  version = SPITransfer(0x18, 0x00, 0); // RegVersion
  Serial.printf("LoRa Version register: 0x%02X\n", version);
}

void LoRaConfig()
{

  const uint8_t MODE_SLEEP = 0x00;
  const uint8_t MODE_STDBY = 0x01;
  const uint8_t LONGRANGEMODE = 1 << 7;

  // Send Reg to sleep
  SPITransfer(RegOpMode, 0x00, 1);
  delay(10);
  SPITransfer(RegOpMode, 0x80, 1);
  delay(5);
  SPITransfer(RegOpMode, 0x01, 1);
  SPITransfer(RegFifoAddrPtr, FifoTxBaseAddr, 1); // 0x0E
  SPITransfer(RegFifoAddrPtr, FifoRxBaseAddr, 1);

  uint32_t frf = (uint32_t)(915e6 / (32e6 / 524288.0)); // 32 MHz / 2^19
  SPITransfer(RegFrMsb, (frf >> 16) & 0xFF, 1);
  SPITransfer(RegFrMid, (frf >> 8) & 0xFF, 1);
  SPITransfer(RegFrLsb, (frf >> 0) & 0xFF, 1);

  // Config Payload Length
  // Configure LoRa Modem
  SPITransfer(RegModemConfig1, LoRaConF1, 1);

  // Configure PA to high Power Mode,
  SPITransfer(RegPaConfig, 0x8F, 1);

  // Configure Preamble length
  SPITransfer(RegPreambleMsb, (8 >> 8) & 0xFF, 1);
  SPITransfer(RegPreambleLsb, 8 & 0xFF, 1);
}

void configureSingleReception()
{

  uint8_t Regi;
  uint8_t LoRa = LoRaMode;
  uint8_t RxSing = RXSingle;
  uint8_t Base = FifoRxBaseAddr;

  // Set RegFifoPtrToBase
  uint8_t res = SPITransfer(RegFifoAddrPtr, Base, 0);
  Serial.printf("Regfifo SPI Result: %d\n", res);

  // Configure the appropriate Dio pins
  uint8_t DIOCFG = (RxDone << 7) | (PayloadCRCError);
  Serial.printf("DIOCFG: %d\n", DIOCFG);
  res = SPITransfer(REGMAP1, DIOCFG, 0);
  Serial.printf("DIOCFG SPI Result: %d\n", res);

  // Bit shift to set LoRa and set_Mode_Single
  Regi = (LoRa << 7) | (RxSing);
  res = SPITransfer(RegOpMode, Regi, 0);
  Serial.printf("RegOpMode SPI Result: %d\n", res);

  res = SPITransfer(0x12, 0x00, 1);
  Serial.printf("RegOpMode SPI Result: %d\n", res);
}

String RxMessage()
{
  const uint8_t LONGRANGEMODE = 1 << 7;

  // Ensure LoRa is in standby and ready
  SPITransfer(RegOpMode, LONGRANGEMODE | 0x01, 1);

  // Set FIFO pointer to RX base
  SPITransfer(RegFifoAddrPtr, FifoRxBaseAddr, 1);

  // Clear IRQ flags
  SPITransfer(RegIrqReg, 0xFF, 1);

  // Set DIO0 to RxDone, DIO1 to PreambleDetected (optional)
  SPITransfer(REGMAP1, (1 << LRxDone) | (1 << LPreambleDetected), 1);

  // Enter single receive mode
  SPITransfer(RegOpMode, LONGRANGEMODE | RXSingle, 1);

  uint32_t start = millis();
  while (!(SPITransfer(RegIrqReg, 0x00, 0) & (1 << LPreambleDetected)))
  {
    if (millis() - start > 5000) // 5 seconds timeout
    {
      Serial.println("Preamble timeout!");
      return ""; // return empty string to indicate no packet
    }
    delay(1);
  }

  // Wait for packet received
  while (!(SPITransfer(RegIrqReg, 0x00, 0) & (1 << LRxDone)))
  {
    delay(1);
    Serial.println("Waiting for Packet...");
  }

  // Check CRC error
  if (SPITransfer(RegIrqReg, 0x00, 0) & (1 << PayloadCRCError))
  {
    Serial.println("CRC Error!");
    SPITransfer(RegIrqReg, 0xFF, 1); // clear IRQs
    return "";                       // return empty string
  }

  // Read number of bytes in FIFO
  uint8_t numBytes = SPITransfer(RegNbRxBytes, 0x00, 0);

  if (numBytes == 0)
    return ""; // no valid packet

  // Set FIFO pointer to current RX address
  SPITransfer(RegFifoAddrPtr, FifoRxBaseAddr, 1);

  // Read all bytes
  String msg = "";
  for (uint8_t i = 0; i < numBytes; ++i)
  {
    uint8_t b = SPITransfer(RegFifo, 0x00, 0);
    msg += (char)b; // convert to char
  }

  // Clear IRQs after reading
  SPITransfer(RegIrqReg, 0xFF, 1);

  return msg;
}

void ConfDIO()
// TODO: Have this function become modular by allowing it to
// Either input or output a value dependent on DIO
// Probably should put this in its own Header
{
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << DIO0) |
                      (1ULL << DIO1) |
                      (1ULL << DIO2) |
                      (1ULL << DIO3) |
                      (1ULL << DIO4) |
                      (1ULL << DIO5), // which pins
      .mode = GPIO_MODE_INPUT,        // output mode
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE, // no interrupts
  };
  gpio_config(&io_conf);
}

uint8_t ReadDIO()
{
  uint8_t i = 0;
  uint8_t level[6];
  uint8_t sum;
  level[0] = gpio_get_level((gpio_num_t)DIO0);
  level[1] = gpio_get_level((gpio_num_t)DIO1);
  level[2] = gpio_get_level((gpio_num_t)DIO2);
  level[3] = gpio_get_level((gpio_num_t)DIO3);
  level[4] = gpio_get_level((gpio_num_t)DIO4);
  level[5] = gpio_get_level((gpio_num_t)DIO5);

  for (i = 0; i < 6; i++)
  {
    sum |= (level[i] << i);
  }

  return (uint8_t)sum;
}

void fuckingWithDIO()
{
  uint8_t Bitmask = SPITransfer(RegOpMode, 0x00, 0);
  // Go from Standby mode to Tx mode (001 -> 010)
  Bitmask |= (1 << 1);
  Bitmask &= ~(1 << 0);
  SPITransfer(RegOpMode, Bitmask, 1);
}

uint8_t readFromReg(uint8_t RegAddr)
{
  return SPITransfer(RegAddr, 0x00, 0);
}

uint8_t SPIFifoRead(uint8_t Rx)
{
  if (Rx)
  {
    SPITransfer(RegFifoAddrPtr, FifoTxBaseAddr, 1);
    //  Effectively Dereferences Pointer
    return SPITransfer(SPITransfer(RegFifoAddrPtr, 0x00, 0), 0x00, 0);
  }
  else
  {
    SPITransfer(0x17, 0x47, 1);
    SPITransfer(RegFifoAddrPtr, FifoRxBaseAddr, 1);
    Serial.printf("Payload length Value, 0x%02X\n", SPITransfer(0x17, 0x00, 0));
    Serial.printf("Payload OpReg, 0x%02X\n", SPITransfer(0x01, 0x00, 0));
    return SPITransfer(SPITransfer(RegFifoAddrPtr, 0x00, 0), 0x00, 0);
  }
}

uint32_t computeFrf(uint32_t carrierHz)
{
  // FRF = carrierHz * 2^19 / F_XOSC (F_XOSC = 32 MHz)
  // Use 64-bit intermediate to avoid overflow
  uint64_t frf = ((uint64_t)carrierHz << 19) / (uint64_t)32000000UL;
  return (uint32_t)frf;
}
