/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 1, 2013
*  Last updated: June 8, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This library is specifically to be used with the BrickPi.
*
*  This is a library for the BrickPi to send and receive UART messages with the RPi.
*/

/*

Sending byte array looks like this:

  RPi to BrickPi:  
    byte   name         description
    0      DEST_ADDR    Destination address for the message, or 0 for BROADCAST.
    1      CHECKSUM     Checksum computed across the entire message.
    2      BYTE_COUNT   The count of bytes in the message body, excluding the header.
    3-n                 The data
  
  BrickPi to RPi (no need for address):
    byte   name         description
    0      CHECKSUM     Checksum computed across the entire message.
    1      BYTE_COUNT   The count of bytes in the message body, excluding the header.
    2-n                 The data  

Returned values

-6 wrong message length
-5 wrong checksum
-4 not even the entire header was received
-3 not my address
-2 timeout           
-1 something went wrong
0  Destination address was BROADCAST
1  Destination address was mine

*/

#include "BrickPiUART.h"

static uint8_t  UART_FULL_ARRAY[128];
static uint8_t  UART_MY_ADDR;

bool UART_Get_Addr() {
  uint8_t temp = EEPROM.read(EEPROM_SETTING_ADDRESS_UART_MY_ADDRESS);
  if (temp != 255 && temp != 0) {
    UART_MY_ADDR = temp;   
  }
  else {
    UART_MY_ADDR = 254;
    return false;
  }
  return true;
}

void UART_Set_Addr(uint8_t NewAddr) {
  if (EEPROM.read(EEPROM_SETTING_ADDRESS_UART_MY_ADDRESS) != NewAddr) {
    EEPROM.write(EEPROM_SETTING_ADDRESS_UART_MY_ADDRESS, NewAddr);
  }
  UART_MY_ADDR = NewAddr;
}

bool UART_Setup(uint32_t speed) {
  Serial.begin(speed);
  return UART_Get_Addr();
}

void UART_Flush() {
  while(Serial.available()) {
    Serial.read();
  }
}

void UART_WriteArray(uint8_t ByteCount, uint8_t *OutArray) {
  uint16_t uart_cksm = ByteCount;
  for (uint8_t i = 0; i < ByteCount; i ++) {
    uart_cksm += OutArray[i];
    UART_FULL_ARRAY[i + 2] = OutArray[i];
  }
  uart_cksm &= 0xFF;

  UART_FULL_ARRAY[0] = uart_cksm;
  UART_FULL_ARRAY[1] = ByteCount;

  Serial.write(UART_FULL_ARRAY, (uint8_t)(ByteCount + 2));
}

int8_t UART_ReadArray(uint8_t &ByteCount, uint8_t *InArray, int timeout) {
  long OriginalTick = millis();
  while (!Serial.available()) {                                                // Wait until data has been received
    if (timeout && (millis() - OriginalTick >= timeout)) {
                                                                               // return -2 if it timed-out waiting for the responce.
      return -2;
    }
  }
  
  uint8_t DataAvailable = 0;
  while (DataAvailable < Serial.available()) {                                 // If it's been 1 ms since the last data was received, assume it's the end of the message.
    DataAvailable = Serial.available();
    delayMicroseconds(50);
  }
  
  if (DataAvailable < 3) {                                                     // Not even the entire header was received.
    UART_Flush();
    return -4;
  }

  for (uint8_t i = 0; i < DataAvailable; i++) {
    UART_FULL_ARRAY[i] = Serial.read();
  }
  
  uint8_t DestAddr = UART_FULL_ARRAY[0];
  uint8_t Checksum = UART_FULL_ARRAY[1];
  ByteCount = UART_FULL_ARRAY[2];

  if (DestAddr == UART_MY_ADDR || DestAddr == 0) {
    if (ByteCount != (DataAvailable - 3)) {
      return -6;
    }

    uint16_t uart_cksm = DestAddr;
    uart_cksm += ByteCount;
    for (uint8_t i = 3; i < DataAvailable; i ++) {
      uart_cksm += UART_FULL_ARRAY[i];
    }
    uart_cksm &= 0xFF;
    
    if (Checksum != uart_cksm) {
      Serial.print("Checksum Fail");
      // Serial.print(Checksum);
      // Serial.print("   Checksum computed: ");
      // Serial.println(uart_cksm);
      return -5;
    }
    
    for (uint8_t i = 0; i < ByteCount; i ++) {
      InArray[i] = UART_FULL_ARRAY[i + 3];
    }
    
    return DestAddr ? 1 : 0;
  }
  else {
    return -3;
  }
  return -1;
}

void UART_Write_Debug(uint8_t ByteCount, uint8_t *OutArray) {
  /*uint16_t uart_cksm = ByteCount;
  for(byte i = 0; i < ByteCount; i ++){
    uart_cksm += OutArray[i];
    UART_FULL_ARRAY[i + 2] = OutArray[i];
  }
  uart_cksm &= 0xFF;

  UART_FULL_ARRAY[0] = uart_cksm;
  UART_FULL_ARRAY[1] = ByteCount;

  Serial.write(UART_FULL_ARRAY, (byte)(ByteCount + 2)); */
  Serial.write(OutArray, byte(ByteCount));
}
