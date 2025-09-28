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
#define PIN_CS 8 // Chip Select (any free GPIO)

#define DIO3 10
#define DIO4 2
#define DIO0 3
#define DIO1 0
#define DIO2 1
#define DIO5 6

extern spi_bus_config_t buscfg;
extern esp_err_t ret;
extern spi_device_handle_t handle;

#define DIO

// Registers addresses
/*The register address will be a 7bit adress with a 0 at the front of it for read access, the
  data should be returned at the MISO reg
*/

#define RegOpMode 0x01      // 3 bits configuration pg102 RFM95_96_97_98W.docx
#define RegDioMapping1 0x40 // Configuring DIO0-DIO3
#define RegDioMapping2 0x41 // Configuring DIO4-DIO5 and ClkOut
#define RegFifo 0x00        // FIFO read and write

#define RegRxDataAddr 0x26 // Pointer to last bit of data recieved in FIFO
#define RegNbRxBytes 0x1D  // Counter of number of Rx'd bytes

#define RegFifoAddrPtr 0x0D

// Set LongRangeMode bit of RegOpMode
#define RXSingle 0x06
#define RXcontinuous 0x05
#define LoRaMode 0x01

// RegDIOMapping
#define REGMAP1 0x40 // used to configure the DIO pins for various flags
#define REGMAP2 0x41 // pins DIO-3

// Interrupt Address Regs
#define RegIrqFlag 0x12

// Mode Configuration Reg (Pkt -> Cont)
#define RegPacketConf 0x31

// single reception mode
#define FifoRxPtrBaseRST 0x00
#define FifoRxBaseAddr 0x0F
#define RxSingle 0b110
#define ValidHeaderInterrupt

// Defined Rx for FSK and LoRa
#define PayloadCRCError 0b10
#define RxDone 0b00
#define LRxDone 6
#define LPayloadCRCError 5

#define RegPreambleDetect 0x1F

// Reset SPI pointer to base

// Rx Procedure
/*
  RxDone flag done (DIO0)
  PayloadCrcError check this flag if valid access the FIFO
  FifoNbRxBytes Indicates the number of bytes that have been received thus far
  set FiFoPtrAddr to RegRxDataAddr
  Payload can be extracted from the FIFO by reading the RegFifo FifoNbRxBytes times
*/

extern spi_device_handle_t handle;
extern spi_bus_config_t buscfg;
extern esp_err_t ret;

void setUpSPI();                         // initialise the SPI bus and add the LoRa Hope RF as a device
uint8_t singleTransfer(uint8_t Address); // new SPI transfer class

void SPITransfer(uint8_t data);

void configureSingleReception();

void TestLoraEspCommSPI();

void readFactoryRegisters();

void ConfDIO();

void fuckingWithDIO();

uint8_t Rx();

uint8_t ReadDIO();

uint8_t readFromReg(uint8_t RegAddr);