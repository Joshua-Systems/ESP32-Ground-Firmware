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

uint8_t Rx()
{
  uint8_t irqFlags;
  int timeout = 1000;
  do
  {
    irqFlags = SPITransfer(RegIrqFlag, 0x00, 0);
    delay(1);
    Serial.printf("Irq SPI Result: %d\n", irqFlags);
    timeout--;
    if (timeout <= 0)
    {
      Serial.printf("RX Timeout");
      return 0;
    }
  } while ((irqFlags & (1 << LRxDone)) == 0); // wait for RxDone bit

  if (irqFlags & (1 << LPayloadCRCError))
  {
    Serial.println("CRC error!");
    return 0; // bad packet
  }

  // Clear IRQ flags
  SPITransfer(RegIrqFlag, 0xFF, 0);

  uint8_t packetLen = SPITransfer(RegNbRxBytes, 0x00, true);

  // Set Base Pointer
  SPITransfer(RegFifoAddrPtr, SPITransfer(RegFifoAddrPtr, 0x00, true), false);

  // Read payload from FIFO
  int i;
  uint8_t buffer[i];
  for (uint8_t i = 0; i < packetLen; i++)
  {
    buffer[i] = SPITransfer(RegFifo, 0x00, true);
  }

  // Clear IRQ flags
  SPITransfer(RegIrqFlag, 0xFF, false);

  // Return packet length
  return packetLen;
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

void messingWithDIO()
{
  uint8_t Reg = SPITransfer(RegDioMapping2, 0b00000000, 1);
  Reg = SPITransfer(RegDioMapping1, 0b00010000, 1);
  Reg = SPITransfer(RegPacketConf, (1 << 6), 1);
  Reg = SPITransfer(RegOpMode, 0b00000101, 1);
}

uint8_t readFromReg(uint8_t RegAddr)
{
  return SPITransfer(RegAddr, 0x00, 0);
}