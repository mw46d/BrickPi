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

#include "BrickPiCS.h"

static uint32_t calData[2][CAL_ROWS][CAL_COLUMNS];
static int32_t  calLimits[2][CAL_LIMITS];
static uint16_t raw_values[2][4];
static uint16_t cal_values[2][4];
static uint16_t type[2];

uint16_t CS_Values[2][4];

inline uint8_t CS_getData(uint8_t port) {
  CS_SET_DATA_INPUT(port);
//  PORTC &= (~(0x01 << port));   // disable the pullup?
  return ((PINC >> port) & 0x01);
}

inline uint16_t CS_readData(uint8_t port) {
//  CS_SET_DATA_INPUT(port);
//  PORTC &= (~(0x01 << port));
  uint16_t value = A_ReadRawCh(port);
  value *= 50;                           // Convert the 5v readings
  return (value / 33);                   // to 3.3v readings
}

static void     CS_Reset(uint8_t port);
static void     CS_SendMode(uint8_t port, uint8_t mode);
static char     CS_ReadByte(uint8_t port);
static uint16_t CS_CalcCRC(uint8_t port, uint16_t crc, uint16_t val);
static bool     CS_ReadCalibration(uint8_t port);
static void     CS_Calibrate(uint8_t port);
static uint8_t  CS_CalToColor(uint8_t port);

void CS_Begin(uint8_t port, uint8_t modetype) {
  if (port > PORT_2) {
    return;
  }
  
  type[port] = modetype;

  CS_Reset(port);
  CS_SendMode(port, modetype);
  delayMicroseconds(3200);
  CS_ReadCalibration(port);
  delay(120); //This can be removed if you have other setup code that doesn't use the color sensor that takes a while.
}

uint16_t CS_Update(uint8_t port) {
  if (port > PORT_2) {
    return 0;
  }
  
  CS_SET_DATA_INPUT(port);
  PORTC &= (~(0x01 << port));
  
  if (type[port] == TYPE_COLORFULL) {
    CS_Values[port][BLANK_INDEX] = CS_readData(port);
    
    CS_SET_CLOCK_HIGH(port);
    delayMicroseconds(20);
    CS_Values[port][RED_INDEX  ] = CS_readData(port);
    
    CS_SET_CLOCK_LOW(port);
    delayMicroseconds(20);
    CS_Values[port][GREEN_INDEX] = CS_readData(port);
    
    CS_SET_CLOCK_HIGH(port);
    delayMicroseconds(20);
    CS_Values[port][BLUE_INDEX ] = CS_readData(port);
    
    CS_SET_CLOCK_LOW(port);
    
    raw_values[port][BLANK_INDEX] = CS_Values[port][BLANK_INDEX];
    raw_values[port][RED_INDEX  ] = CS_Values[port][RED_INDEX  ];
    raw_values[port][GREEN_INDEX] = CS_Values[port][GREEN_INDEX];
    raw_values[port][BLUE_INDEX ] = CS_Values[port][BLUE_INDEX ];
    
    CS_Calibrate(port);
    return CS_CalToColor(port);    
  }
  else {
    CS_Values[port][type[port] - TYPE_COLORRED] = CS_readData(port);
    raw_values[port][type[port] - TYPE_COLORRED] = CS_Values[port][type[port] - TYPE_COLORRED];
    return CS_Values[port][type[port] - TYPE_COLORRED];
  }
}

void CS_KeepAlive(uint8_t port) {
  if (port > PORT_2) {
    return;
  }

  CS_SET_CLOCK_HIGH(port);
  delayMicroseconds(20);
  CS_SET_CLOCK_LOW(port);
  delayMicroseconds(20);    
  CS_SET_CLOCK_HIGH(port);
  delayMicroseconds(20);
  CS_SET_CLOCK_LOW(port);
}

void CS_Reset(uint8_t port) {
  CS_SET_CLOCK_HIGH(port);
  CS_SET_DATA_HIGH(port);
  delay(1);
  CS_SET_CLOCK_LOW(port);
  delay(1);
  CS_SET_CLOCK_HIGH(port);
  delay(1);
  CS_SET_CLOCK_LOW(port);
  delay(100);
}

void CS_SendMode(uint8_t port, uint8_t mode) {
  for (uint8_t i = 0; i < 8; i++) {
    CS_SET_CLOCK_HIGH(port);
    CS_SET_DATA(port, (mode & 1));
    delayMicroseconds(30);
    CS_SET_CLOCK_LOW(port);
    mode >>= 1;
    delayMicroseconds(30);
  }
}

char CS_ReadByte(uint8_t port) {
  uint8_t val = 0;
  for (uint8_t i = 0; i< 8; i++) {
    CS_SET_CLOCK_HIGH(port);
    
    delayMicroseconds(8);    // MT
    
    val >>= 1;
    if (CS_getData(port)) {
      val |= 0x80;
    }
    CS_SET_CLOCK_LOW(port);
  }
  return val;
}

uint16_t CS_CalcCRC(uint8_t port, uint16_t crc, uint16_t val) {
  for (uint8_t i = 0; i < 8; i++) {
    if (((val ^ crc) & 1) != 0) {
      crc = ((crc >> 1) ^ 0xA001); // the >> should shift a zero in
    }
    else {
      crc >>= 1; //the >> should shift a zero in
    }
    val >>= 1; //the >> should shift a zero in
  }
  return crc & 0xFFFF;
}

bool CS_ReadCalibration(uint8_t port) {
  uint16_t crcVal = 0x5aa5;
  uint8_t input;
  
  CS_SET_DATA_INPUT(port);
  
  for (uint8_t i = 0; i < CAL_ROWS; i++) {
    for (uint8_t col = 0; col < CAL_COLUMNS; col++) {
      uint32_t val = 0;
      uint8_t shift_val = 0;

      for (uint8_t k = 0; k < 4; k++) {
        input = CS_ReadByte(port);
        crcVal = CS_CalcCRC(port, crcVal, input);
        val |= ((uint32_t) input << shift_val);
        shift_val += 8;
      }
      calData[port][i][col] = val;
    }
  }
  for (uint8_t i = 0; i < CAL_LIMITS; i++) {
    uint32_t val = 0;
    uint8_t shift = 0;
    for (uint8_t k = 0; k < 2; k++) {
      input = CS_ReadByte(port);
      crcVal = CS_CalcCRC(port, crcVal, input);
      val |= input << shift;
      shift += 8;
    }
    calLimits[port][i] = val;
  }
  uint16_t crc = (CS_ReadByte(port) << 8);
  crc += CS_ReadByte(port);
  crc &= 0xFFFF;
  delay(2);
  return true;
}

void CS_Calibrate(uint8_t port) {
  //calibrate raw_values to cal_values
  uint8_t cal_tab;
  int blank_val = raw_values[port][BLANK_INDEX];

  if (blank_val < calLimits[port][1]) {
    cal_tab = 2;
  }
  else if (blank_val < calLimits[port][0]) {
    cal_tab = 1;
  }
  else {
    cal_tab = 0;
  }

  for (uint8_t col = RED_INDEX; col <= BLUE_INDEX; col++) {
    if (raw_values[port][col] > blank_val) {
      cal_values[port][col] = ((raw_values[port][col] - blank_val) * calData[port][cal_tab][col]) >> 16; //TODO check shift!
    }
    else {
      cal_values[port][col] = 0;
    }
  }

  if (blank_val > MINBLANKVAL) {
    blank_val -= MINBLANKVAL;
  }
  else {
    blank_val = 0;
  }

  blank_val = (blank_val * 100) / (((SENSORMAX - MINBLANKVAL ) * 100) / ADMAX);
  cal_values[port][BLANK_INDEX] = (blank_val * calData[port][cal_tab][BLANK_INDEX]) >> 16 ; //TODO CHECK SHIFT    
}

/*
void ColorSensor::print_color(uint8_t color) {
//TODO: use PROGMEM for these strings
    switch (color)
    {
    case BLACK:
        Serial.print("black");
        break;
    case BLUE:
        Serial.print("blue");
        break;
    case GREEN:
        Serial.print("green");
        break;
    case YELLOW:
        Serial.print("yellow");
        break;
    case RED:
        Serial.print("red");
        break;
    case WHITE:
        Serial.print("white");
        break;
    default:
        Serial.println("error");
    }
}*/

uint8_t CS_CalToColor(uint8_t port) {
  int red = cal_values[port][RED_INDEX];
  int blue = cal_values[port][BLUE_INDEX];
  int green = cal_values[port][GREEN_INDEX];
  int blank = cal_values[port][BLANK_INDEX];

  if (red > blue && red > green) {
    // red dominant color
    if (red < 65 || (blank < 40 && red < 110)) {
      return BLACK;
    }
    if (((blue >> 2) + (blue >> 3) + blue < green) && ((green << 1) > red)) {
      return YELLOW;
    }
    if ((green << 1) - (green >> 2) < red) {
      return RED;
    }
    if (blue < 70 || green < 70 || (blank < 140 && red < 140)) {
      return BLACK;
    }
    return WHITE;
  }
  else if (green > blue) {
    // green dominant color
    if (green < 40 || (blank < 30 && green < 70)) {
      return BLACK;
    }
    if ((blue << 1) < red) {
      return YELLOW;
    }
    if ((red + (red >> 2)) < green || (blue + (blue>>2)) < green) {
      return GREEN;
    }
    if (red < 70 || blue < 70 || (blank < 140 && green < 140)) {
      return BLACK;
    }
    return WHITE;
  }
  else {
    // blue dominant color
    if (blue < 48 || (blank < 25 && blue < 85)) {
      return BLACK;
    }
    if ((((red*48) >> 5) < blue && ((green*48) >> 5) < blue) || ((red*58) >> 5) < blue || ((green*58) >> 5) < blue) {
      return BLUE;
    }
    if (red < 60 || green < 60 || (blank < 110 && blue < 120)) {
      return BLACK;
    }
    if ((red + (red >> 3)) < blue || (green + (green >> 3)) < blue) {
      return BLUE;
    }
    return WHITE;
  }
}
