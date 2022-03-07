#include "MEM25LC256.h"

void MEM25LC256::init() {
  SPI.begin(); /* SPI is the communication protocol used by the 25LC256 */

  /* Set pins to output */
  pinMode(MEM25LC256_NOT_CS, OUTPUT);
  pinMode(MEM25LC256_NOT_HOLD, OUTPUT);
  pinMode(MEM25LC256_NOT_WP, OUTPUT);

  /* Set the following pins to NOT SELECTED by default */
  digitalWrite(MEM25LC256_NOT_CS, HIGH);
  digitalWrite(MEM25LC256_NOT_HOLD, HIGH);
  digitalWrite(MEM25LC256_NOT_WP, HIGH); 

  /* Writing status 0 disables Block protection for the entire memory array
   *  as well as disables the software write protection which causes the 
   *  chip to ignore input from the WP (write protect) pin.
   */ 
  writeStatus(0);
}

// My implementation of reading a word and returning it
uint16_t MEM25LC256::readword(uint16_t address) {
  uint16_t readWord;
  uint8_t bit1;
  uint8_t bit2;
  uint16_t address2 = address+1;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); /* No more than 10MHz, MSB first, SPI 0 */
  digitalWrite(MEM25LC256_NOT_CS, LOW);   /* Chip select */
  
  SPI.transfer(MEM25LC256_READ);          /* READ command */
  SPI.transfer16(address);              /* Address */
  bit1 = SPI.transfer(0);        /* Read in data */

  digitalWrite(MEM25LC256_NOT_CS, HIGH);  /* Deselect chip */
  digitalWrite(MEM25LC256_NOT_CS, LOW);   /* Chip select */
  
  SPI.transfer(MEM25LC256_READ);          /* READ command */
  SPI.transfer16(address2);              /* Address */
  bit2 = SPI.transfer(0);         /* Read in data */

  digitalWrite(MEM25LC256_NOT_CS, HIGH);  /* Deselect chip */
  SPI.endTransaction();
  readWord = bit1<<8;
  readWord = readWord | (bit2 & 0xff);
  return readWord;
}
 

void MEM25LC256::writeword(uint16_t address, uint16_t writeWord) {

  // Split the word into two bytes and write them in a row
  uint8_t bit1 = writeWord>>8;
  uint8_t bit2 = writeWord;
  //uint8_t bit1 = 7;
  //uint8_t bit2 = 9;
  uint16_t address2 = address+1;

  writebit(address, bit1);
  writebit(address2, bit2);

}

void MEM25LC256::writebit(uint16_t address, uint8_t writeBit) {

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); /* No more than 10MHz, MSB first, SPI 0 */
  digitalWrite(MEM25LC256_NOT_CS, LOW);   /* Chip select */

  SPI.transfer(MEM25LC256_WREN);          /* WRITE ENABLE command */
  digitalWrite(MEM25LC256_NOT_CS, HIGH);  /* Toggle chip select */
  digitalWrite(MEM25LC256_NOT_CS, LOW);
  SPI.transfer(MEM25LC256_WRITE);         /* WRITE command */
  SPI.transfer16(address);              /* Address */
  SPI.transfer(writeBit); /* Write data */
  digitalWrite(MEM25LC256_NOT_CS, HIGH);  /* Deselect chip */
  SPI.endTransaction();

  while (readStatus() & MEM25LC256_WIP); /* Don't return until write operation complete */

}


uint8_t MEM25LC256::readStatus() {
  uint8_t status = 0;
  digitalWrite(MEM25LC256_NOT_CS, LOW);   /* Chip select */
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); /* No more than 10MHz, MSB first, SPI 0 */

  SPI.transfer(MEM25LC256_RDSR);          /* read status register command */
  status = SPI.transfer(0);             /* Send dummy byte to receive status */

  SPI.endTransaction();
  digitalWrite(MEM25LC256_NOT_CS, HIGH);  /* Deselect chip */

  return status;
}

void MEM25LC256::writeStatus(uint8_t status) {
  digitalWrite(MEM25LC256_NOT_CS, LOW);   /* Chip select */
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); /* No more than 10MHz, MSB first, SPI 0 */

  SPI.transfer(MEM25LC256_WRSR);  /* write status register command */
  SPI.transfer(status);         /* Send status byte */  

  SPI.endTransaction();
  digitalWrite(MEM25LC256_NOT_CS, HIGH);  /* Deselect chip */   
}
