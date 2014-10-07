/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: May 25, 2013
*  Last updated: June 19, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This library is specifically to be used with the BrickPi.
*
*  This is a bit-banged I2C library
*/

#ifndef __BrickPiI2C_h_
#define __BrickPiI2C_h_

#include "Arduino.h"

#define PORT_1 0
#define PORT_2 1

#define I2C_SCL_LOW       (DDRC |= I2C_PORT_DDRC_SCL)                // Pull SCL low
#define I2C_SCL_HIGH      (DDRC &= I2C_PORT_DDRC_SCL_NOT)            // Let SCL go high
#define I2C_SCL_STATE     ((PINC >> I2C_PORT_PIN_NUM_SCL) & 0x01)    // Check the state of SCL
#define I2C_SCL_MSK_STATE (PINC & I2C_PORT_PIN_MSK_SCL)              // Check the state of SCL. Not bit-shifted.

#define I2C_SDA_LOW       (DDRC |= I2C_PORT_DDRC_SDA)                // Pull SDA low
#define I2C_SDA_HIGH      (DDRC &= I2C_PORT_DDRC_SDA_NOT)            // Let SDA go high
#define I2C_SDA_STATE     ((PINC >> I2C_PORT_PIN_NUM_SDA) & 0x01)    // Check the state of SDA
#define I2C_SDA_MSK_STATE (PINC & I2C_PORT_PIN_MSK_SDA)              // Check the state of SDA. Not bit-shifted.

#define I2C_WAIT if(I2C_DELAY)I2C_Delay_us(I2C_DELAY)

uint8_t I2C_Settings(uint8_t port, uint8_t addr, uint8_t speed);
uint8_t I2C_Setup(uint8_t port, uint8_t addr, uint8_t speed);
uint8_t I2C_Transfer(uint8_t port, uint8_t addr, uint8_t speed, uint8_t mid_clock, uint8_t write_length, uint8_t * write_Array, uint8_t read_length, uint8_t * read_Array);
uint8_t I2C_Write(uint8_t length, uint8_t * Array);
uint8_t I2C_Read(uint8_t length, uint8_t * Array);
uint8_t I2C_SCL_CHECK(void);
uint8_t I2C_SDA_CHECK(void);

/* Only to be used by the library */
void I2C_Delay_us(uint16_t us);
uint8_t I2C_Start(void);
uint8_t I2C_Stop(void);
uint8_t I2C_Byte_Out(uint8_t data);
uint8_t I2C_Byte_In(uint8_t & data, uint8_t ack);
void    I2C_TimerOn8(void);
void    I2C_TimerOn256(void);

#endif
