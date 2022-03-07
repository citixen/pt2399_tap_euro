#ifndef MEM25LC256_h
#define MEM25LC256_h

#include <SPI.h>

/* Commands */
#define MEM25LC256_READ    0b00000011 /* Read command */
#define MEM25LC256_WRITE   0b00000010 /* Write command */
#define MEM25LC256_WRDI    0b00000100 /* Reset write-enable latch */
#define MEM25LC256_WREN    0b00000110 /* Set write-enable latch */
#define MEM25LC256_RDSR    0b00000101 /* Read STATUS register */
#define MEM25LC256_WRSR    0b00000001 /* Write STATUS register */

/* Status Flags */
#define MEM25LC256_WIP 0b00000001 /* Write in progress */
#define MEM25LC256_WEL 0b00000010 /* Write Enable Latch */

/* Pins */
#define MEM25LC256_NOT_CS    15 /* Chip Select */
#define MEM25LC256_NOT_WP    6 /* Write Protect */
#define MEM25LC256_NOT_HOLD  7 /* Hold */

/* Properties */
#define MEM25LC256_PAGE_SIZE 64

class MEM25LC256 {
  public: 
  void init();
  uint16_t readword(uint16_t address);
  void writeword(uint16_t address, uint16_t writeWord);
  void writebit(uint16_t address, uint8_t writeBit);
  uint8_t readStatus();
  void writeStatus(uint8_t status);

  private:
  void sendInPage(uint16_t address, uint16_t nBytes, uint8_t *buffer);
};

#endif
