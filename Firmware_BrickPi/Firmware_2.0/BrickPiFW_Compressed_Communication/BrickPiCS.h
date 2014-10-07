/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: May 28, 2013
*  Last updated: June 7, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This library is specifically to be used with the BrickPi.
*
*  This is a library for reading the Lego Color sensor.
*/

#ifndef __BrickPiCS_h_
#define __BrickPiCS_h_

#include "Arduino.h"
#include "BrickPiA.h"

#define PORT_1 0
#define PORT_2 1

#define BLACK 1
#define BLUE 2
#define GREEN 3
#define YELLOW 4
#define RED 5
#define WHITE 6

#define RED_INDEX 0
#define GREEN_INDEX 1
#define BLUE_INDEX 2
#define BLANK_INDEX 3

#define TYPE_COLORFULL 13
#define TYPE_COLORRED 14
#define TYPE_COLORGREEN 15
#define TYPE_COLORBLUE 16
#define TYPE_COLORNONE 17

#define ADVOLTS 3300
#define ADMAX 1023
#define SENSORMAX ADMAX
#define MINBLANKVAL (214 / (ADVOLTS / ADMAX))

#define CAL_COLUMNS 4
#define CAL_ROWS 3
#define CAL_LIMITS 2

#define CS_SET_CLOCK_INPUT(port)       \
	(DDRC &= (~(0x04 << port)));

#define CS_SET_CLOCK_OUTPUT(port)      \
	(DDRC |= (0x04 << port));

#define CS_SET_CLOCK_HIGH(port)        \
	CS_SET_CLOCK_OUTPUT(port);     \
	PORTC |= (0x04 << port);

#define CS_SET_CLOCK_LOW(port)         \
	CS_SET_CLOCK_OUTPUT(port);     \
	PORTC &= (~(0x04 << port));

#define CS_SET_CLOCK(port, state)      \
	CS_SET_CLOCK_OUTPUT(port);     \
	if (state) {                   \
	  PORTC |= (0x04 << port);     \
	}                              \
	else {                         \
	  PORTC &= (~(0x04 << port));  \
	}

#define CS_SET_DATA_INPUT(port)        \
	(DDRC &= (~(0x01 << port)));

#define CS_SET_DATA_OUTPUT(port)       \
	DDRC |= (0x01 << port);

#define CS_SET_DATA_HIGH(port)         \
	CS_SET_DATA_OUTPUT(port);      \
	PORTC |= (0x01 << port);

#define CS_SET_DATA_LOW(port)          \
	CS_SET_DATA_OUTPUT(port);      \
	PORTC &= (~(0x01 << port));

#define CS_SET_DATA(port, state)       \
	CS_SET_DATA_OUTPUT(port);      \
	if (state) {                   \
	  PORTC |= (0x01 << port);     \
	}                              \
	else {                         \
	  PORTC &= (~(0x01 << port));  \
	}

// Access from user program
void     CS_Begin(uint8_t port, uint8_t modetype);
uint16_t CS_Update(uint8_t port);
void     CS_KeepAlive(uint8_t port);   // Simulate reading the sensor, so that it doesn't time-out.
extern uint16_t CS_Values[2][4];

#endif
