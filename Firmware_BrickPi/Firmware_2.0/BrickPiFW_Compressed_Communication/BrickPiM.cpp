/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: May 30, 2013
*  Last updated: July 2, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This library is specifically to be used with the BrickPi.
*
*  This is a library for controlling motors, and reading the encoders.
*/

#include "BrickPiM.h"
#include "BrickPiUART.h"

volatile static int32_t motorEncoder[2];
volatile static uint8_t PCintLast;
volatile static uint8_t State[2] = {0, 0};

//                         0000 0001 0010 0011 0100 0101 0110 0111 1000 1001 1010 1011 1100 1101 1110 1111
static int Enc_States[] = {   0,  -1,   1,   0,   1,   0,   0,  -1,  -1,   0,   0,   1,   0,   1,  -1,   0};

void M_T_ISR(uint8_t port) {
  State[port] = (((State[port] << 2) | (((PIND >> (1 + port)) & 0x02) | ((PIND >> (4 + port)) & 0x01))) & 0x0F);

  motorEncoder[port] += Enc_States[State[port]];
}

ISR(PCINT2_vect) {
  uint8_t curr;
  uint8_t mask;

  // get the pin states for the port.
  curr = PIND;

  // mask is pins that have changed.
  mask = curr ^ PCintLast;

  // Update PCintLast for next time
  PCintLast = curr;

  // screen out non pcint pins.
  mask &= 0x3C;

  if (mask & 0x14){ // PCINT 18 or 20, MAT
    M_T_ISR(PORT_A);
  }
  if (mask & 0x28){ // PCINT 19 or 21, MBT
    M_T_ISR(PORT_B);
  }
}

MotorBank::MotorBank() {
  // Setup the encoder pins as inputs, with pullups disabled.
  DDRD  &= 0xC3;                  // Set PD2 - 5 as input
  PORTD &= 0xC3;                  // Disable pullups on PD2 - 5

  // Enable the PCINT channels for reading the encoders.
  PCMSK2 |= 0x3C;                 // React to PCINT 18, 19, 20, and 21.
  PCICR |= 0x04;                  // Enable PCINT Enable 2

  // Setup EN, PWM and DIR as LOW. Setup the EN, PWM, and DIR pins as outputs.
  PORTB = PORTB & 0xC0;  // Leave PB6 and 7 alone. 0, 1, 2, 3, 4, and 5 LOW.
  DDRB  |= 0x3F;                  // Set PB0 - 5 as output

  reset();
}

bool MotorBank::setEncoderTarget(Motor which_motor, int32_t target) {
  if ((which_motor & Motor_Both) == Motor_Both) {
    targetEncoder[0] = targetEncoder[1] = target;
    motorStatus[0] |= MOTOR_STATUS_TACHO;
    motorStatus[1] |= MOTOR_STATUS_TACHO;
  }
  else {
    if (which_motor >= Motor_Both) {
      return false;
    }

    targetEncoder[which_motor - 1] = target;
    motorStatus[which_motor - 1] |= MOTOR_STATUS_TACHO;
  }

  return true;
}

int32_t MotorBank::getEncoderTarget(Motor which_motor) {
  if (which_motor < Motor_Both) {
    if (motorStatus[which_motor - 1] & MOTOR_STATUS_TACHO) {
      return targetEncoder[which_motor - 1];
    }
  }

  return 0;
}

static void setSpeedInternal(Motor which_motor, int8_t speed) {
  bool reverse = false;

  if (which_motor >= Motor_Both) {
    return;
  }

  if (speed < 0) {
    reverse = true;
    speed = -speed;
  }

  uint8_t finalSpeed = (uint8_t)(((uint16_t)speed * 255) / 100);

  if (which_motor == Motor_1) {
    if (reverse) {
      analogWrite(10, ~finalSpeed);
      PORTB |= 0x01;
    }
    else {
      analogWrite(10, finalSpeed);
      PORTB &= ~0x01;
    }
  }
  else if (which_motor == Motor_2) {
    if (reverse) {
      analogWrite(11, ~finalSpeed);
      PORTB |= 0x02;
    }
    else {
      analogWrite(11, finalSpeed);
      PORTB &= ~0x02;
    }
  }
}

bool MotorBank::setSpeed(Motor which_motor, int8_t speed) {
  if (which_motor > Motor_Both) {
    return false;
  }

  if (which_motor == Motor_Both) {
    bool r1 = setSpeed(Motor_1, speed);
    bool r2 = setSpeed(Motor_2, speed);
    return  r1 && r2;
  }

  motorSpeed[which_motor - 1] = speed;
  motorStatus[which_motor - 1] |= MOTOR_STATUS_SPEED;

  setSpeedInternal(which_motor, speed);

  return true;
}

int8_t MotorBank::getSpeed(Motor which_motor) {
  if (which_motor < Motor_Both) {
    return motorSpeed[which_motor - 1];
  }

  return 0;
}

bool MotorBank::setTimeToRun(Motor which_motor, uint32_t mseconds) {
  if (mseconds == 0) {
    return true;
  }

  if (which_motor > Motor_Both) {
    return false;
  }

  if (which_motor == Motor_Both) {
    finishTime[0] = finishTime[1] = mseconds + millis();
    motorStatus[0] |= MOTOR_STATUS_TIME;
    motorStatus[1] |= MOTOR_STATUS_TIME;
  }
  else {
    finishTime[which_motor - 1] = mseconds + millis();
    motorStatus[which_motor - 1] |= MOTOR_STATUS_TIME;
  }

  return true;
}

uint32_t MotorBank::getTimeToRun(Motor which_motor) {
  if (which_motor < Motor_Both) {
    if (motorStatus[which_motor - 1] & MOTOR_STATUS_TIME) {
      return finishTime[which_motor - 1] - millis();
    }
  }

  return 0;
}

int32_t MotorBank::getEncoderPosition(Motor which_motor) {
  if (which_motor < Motor_Both) {
    return motorEncoder[which_motor - 1];
  }

  return 0;
}

uint8_t MotorBank::getStatusByte(Motor which_motor) {
  if (which_motor < Motor_Both) {
    return motorStatus[which_motor - 1];
  }

  return 0;
}

bool MotorBank::reset() {
  motorFloat(Motor_Both);

  finishTime[0] = finishTime[1] = 0;
  motorControl[0] = motorControl[1] = 0;
  motorSpeed[0] = motorSpeed[1] = 0;
  motorStatus[0] = motorStatus[1] = 0;
  motorEncoder[0] = motorEncoder[1] = 0;
  targetEncoder[0] = targetEncoder[1] = 0;
  lastEncoder[0] = lastEncoder[1] = 0;
  lastEncoderTime[0] = lastEncoderTime[1] = -1;

  return true;
}

bool MotorBank::startBothInSync() {
  lastEncoder[0] = motorEncoder[0];
  lastEncoder[1] = motorEncoder[1];

  PORTB |= 0x30;

  motorStatus[0] |= MOTOR_STATUS_MOVING;
  motorStatus[1] |= MOTOR_STATUS_MOVING;
  lastEncoderTime[0] = lastEncoderTime[1] = -1;

  return true;
}

bool MotorBank::resetEncoder(Motor which_motor) {
  if (which_motor < Motor_Both) {
    motorEncoder[which_motor - 1] = 0;
    lastEncoder[which_motor - 1] = 0;
    lastEncoderTime[which_motor - 1] = -1;
    return true;
  }

  return false;
}

void MotorBank::motorFloat(Motor which_motor) {
  if (which_motor == Motor_Both) {
    PORTB &= ~0x30;
    motorStatus[0] &= ~MOTOR_STATUS_MOVING & ~MOTOR_STATUS_BRK;
    motorStatus[1] &= ~MOTOR_STATUS_MOVING & ~MOTOR_STATUS_BRK;
  }
  else if ((which_motor & Motor_Both) == Motor_1) {
    PORTB &= ~0x10;
    motorStatus[0] &= ~MOTOR_STATUS_MOVING & ~MOTOR_STATUS_BRK;
  }
  else if ((which_motor & Motor_Both) == Motor_2) {
    PORTB &= ~0x20;
    motorStatus[1] &= ~MOTOR_STATUS_MOVING & ~MOTOR_STATUS_BRK;
  }
}

void MotorBank::motorBrake(Motor which_motor) {
  if (which_motor > Motor_Both) {
    return;
  }

  if (which_motor == Motor_Both) {
    motorBrake(Motor_1);
    motorBrake(Motor_2);
  }
  else {
    uint32_t time = millis();

    setSpeedInternal(which_motor, -motorSpeed[which_motor - 1]);

    motorStatus[which_motor - 1] |= MOTOR_STATUS_BRK;
    lastEncoderTime[which_motor - 1] = time;
  }
}

bool MotorBank::motorEnable(Motor which_motor) {
  if (which_motor >= Motor_Both) {
    return false;
  }

  uint8_t control = motorControl[which_motor - 1];

  if (which_motor == Motor_1) {
    if (control & MOTOR_CONTROL_GO) {
      lastEncoder[0] = motorEncoder[0];
      PORTB |= 0x10;
      motorStatus[0] |= MOTOR_STATUS_MOVING;
      lastEncoderTime[0] = -1;
    }
    else {
      PORTB &= ~0x10;
      motorStatus[0] &= ~MOTOR_STATUS_MOVING;
    }
  }
  else {
    if (control & MOTOR_CONTROL_GO) {
      lastEncoder[1] = motorEncoder[1];
      PORTB |= 0x20;
      motorStatus[1] |= MOTOR_STATUS_MOVING;
      lastEncoderTime[1] = -1;
    }
    else {
      PORTB &= ~0x20;
      motorStatus[1] &= ~MOTOR_STATUS_MOVING;
    }
  }

  return true;
}

static inline int8_t calcFinalSpeed(int8_t initialSpeed, Direction direction) {
  if (direction == Direction_Forward) {
    return initialSpeed;
  }

  return -initialSpeed;
}

static inline uint8_t calcNextActionBits(Next_Action next_action) {
  if (next_action == Next_Action_Brake) {
    return MOTOR_CONTROL_BRK;
  }
  else if (next_action == Next_Action_BrakeHold) {
    return MOTOR_CONTROL_BRK | MOTOR_CONTROL_ON;
  }

  return 0;
}

static void sendStatus(uint8_t *array, bool ret) {
  if (ret) {
    array[0] = MSG_TYPE_MOTOR_OK;
  }
  else {
    array[0] = MSG_TYPE_MOTOR_NOK;
  }

  UART_WriteArray(1, array);
}

static uint32_t read_32(uint8_t *array) {
  return ((uint32_t)(array[3]) << 24) | ((uint32_t)(array[2]) << 16) | ((uint32_t)(array[1]) << 8) | ((uint32_t)(array[0]));
}

static void write_32(uint8_t *array, uint32_t val) {
  array[0] = val & 0xff;
  array[1] = (val >> 8) & 0xff;
  array[2] = (val >> 16) & 0xff;
  array[3] = (val >> 24) & 0xff;
}

bool MotorBank::setSpeedTimeAndControl(Motor which_motors, int8_t speed,
                                       uint32_t duration, uint8_t control) {
  if (which_motors > Motor_Both) {
    return false;
  }

  if (which_motors == Motor_Both) {
    control &= ~MOTOR_CONTROL_GO;			// Clear the 'go right now' flag
    motorFloat(Motor_Both);
    bool m1 = setSpeedTimeAndControl(Motor_1, speed, duration, control);
    bool m2 = setSpeedTimeAndControl(Motor_2, speed, duration, control);
    startBothInSync();

    return m1 && m2;
  }

  motorControl[which_motors - 1] = control;
  motorStatus[which_motors - 1] = 0;

  bool r1 = setSpeed(which_motors, speed);
  bool r2 = setTimeToRun(which_motors, duration);
  bool r3 = motorEnable(which_motors);

  return r1 && r2 && r3;
}

bool MotorBank::setEncoderSpeedTimeAndControl(Motor which_motors,
                                              int32_t encoder, int8_t speed,
                                              uint32_t duration, uint8_t control) {
  if (which_motors > Motor_Both) {
    return false;
  }

  if (which_motors == Motor_Both) {
    control &= ~MOTOR_CONTROL_GO;			// Clear the 'go right now' flag
    motorFloat(Motor_Both);
    bool m1 = setEncoderSpeedTimeAndControl(Motor_1, encoder, speed, duration, control);
    bool m2 = setEncoderSpeedTimeAndControl(Motor_2, encoder, speed, duration, control);
    startBothInSync();

    return m1 && m2;
  }

  motorControl[which_motors - 1] = control;
  motorStatus[which_motors - 1] = 0;

  if (control & MOTOR_CONTROL_RELATIVE) {
    encoder = motorEncoder[which_motors - 1] + encoder;
  }

  bool r1 = setEncoderTarget(which_motors, encoder);
  bool r2 = setSpeed(which_motors, speed);
  bool r3 = setTimeToRun(which_motors, duration);
  bool r4 = motorEnable(which_motors);

  return r1 && r2 && r3 && r4;
}

uint8_t MotorBank::isTimeDone(Motor which_motors) {
  uint8_t s = 0;

  if (which_motors > Motor_Both) {
    return 255;
  }
  else if (which_motors == Motor_Both) {
    s = motorStatus[0] | motorStatus[1];
  }
  else {
    s = motorStatus[which_motors - 1];
  }

  if (!(s & MOTOR_STATUS_TIME)) {
    if (s & MOTOR_STATUS_STALL) {
      return MOTOR_STATUS_STALL;
    }
    else {
      return 0;
    }
  }

  return s;
}

uint8_t MotorBank::waitUntilTimeDone(Motor which_motors) {
  uint8_t s;

  delay(50);
  s = isTimeDone(which_motors);
  while (s & MOTOR_STATUS_TIME) {
    if (s & MOTOR_STATUS_STALL) {
      return MOTOR_STATUS_STALL;
    }

    delay(50);
    s = isTimeDone(which_motors);
  }

  return 0;
}

uint8_t MotorBank::isTachoDone(Motor which_motors) {
  uint8_t s;

  if (which_motors > Motor_Both) {
    return 255;
  }
  else if (which_motors == Motor_Both) {
    s = motorStatus[0] | motorStatus[1];
  }
  else {
    s = motorStatus[which_motors - 1];
  }

  if (!(s & MOTOR_STATUS_TACHO)) {
    if (s & MOTOR_STATUS_STALL) {
      return MOTOR_STATUS_STALL;
    }
    else {
      return 0;
    }
  }

  return s;
}

uint8_t MotorBank::waitUntilTachoDone(Motor which_motors) {
  uint8_t s;

  delay(50);
  s = isTachoDone(which_motors);
  while (s & MOTOR_STATUS_TACHO) {
    if (s & MOTOR_STATUS_STALL) {
      return MOTOR_STATUS_STALL;
    }

    delay (50);
    s = isTachoDone(which_motors);
  }

  return 0;
}

void MotorBank::runUnlimited(Motor which_motors, Direction direction,
                             int8_t speed) {
  uint8_t ctrl = MOTOR_CONTROL_SPEED | MOTOR_CONTROL_GO;
  int8_t sp = calcFinalSpeed(speed, direction);
  setSpeedTimeAndControl(which_motors, sp, 0, ctrl);
}

uint8_t MotorBank::runMilliSeconds(Motor which_motors, Direction direction,
                                   int8_t speed, uint32_t duration,
                                   Completion_Wait wait_for_completion,
                                   Next_Action next_action) {
  int8_t sp = calcFinalSpeed(speed, direction);
  uint8_t ctrl = MOTOR_CONTROL_SPEED | MOTOR_CONTROL_TIME | MOTOR_CONTROL_GO;
  ctrl |= calcNextActionBits(next_action);
  setSpeedTimeAndControl(which_motors, sp, duration, ctrl);

  if (wait_for_completion == Completion_Wait_For) {
    return waitUntilTimeDone(which_motors);
  }

  return 0;
}

uint8_t MotorBank::runTachometer(Motor which_motors, Direction direction,
                                 int8_t speed, int32_t tachometer,
                                 Move relative,
                                 Completion_Wait wait_for_completion,
                                 Next_Action next_action) {
  int8_t sp = calcFinalSpeed(speed, direction);
  uint8_t ctrl = MOTOR_CONTROL_SPEED | MOTOR_CONTROL_TACHO | MOTOR_CONTROL_GO;
  ctrl |= calcNextActionBits(next_action);

							// The tachometer can be absolute or relative.
							// If it is absolute, we ignore the direction parameter.
  int32_t final_tach = tachometer;

  if (relative == Move_Relative) {
    ctrl |= MOTOR_CONTROL_RELATIVE;

							// a (relative) forward command is always a positive tachometer reading
    final_tach = abs(tachometer);
    if (sp < 0) {					// and a (relative) reverse command is always negative
      final_tach = -final_tach;
    }
  }

  setEncoderSpeedTimeAndControl(which_motors, final_tach, sp, 0, ctrl);

  if (wait_for_completion == Completion_Wait_For) {
    return waitUntilTachoDone(which_motors);
  }

  return 0;
}

uint8_t MotorBank::runDegrees(Motor which_motors, Direction direction,
                              int8_t speed, int32_t degrees,
                              Completion_Wait wait_for_completion,
                              Next_Action next_action) {
  return runTachometer(which_motors, direction, speed, degrees,
      Move_Relative, wait_for_completion, next_action);
}

uint8_t MotorBank::runRotations(Motor which_motors, Direction direction,
                                int8_t speed, int32_t rotations,
                                Completion_Wait wait_for_completion,
                                Next_Action next_action) {
  return runTachometer(which_motors, direction, speed, 360 * rotations,
      Move_Relative, wait_for_completion, next_action);
}

bool MotorBank::stop(Motor which_motors, Next_Action next_action) {
  if (which_motors > Motor_Both) {
    return false;
  }

  if (next_action != Next_Action_Float) {
    motorBrake(which_motors);
  }
  else {
    motorFloat(which_motors);
  }

  if (which_motors == Motor_Both) {
    motorStatus[0] &= ~MOTOR_STATUS_TACHO & ~MOTOR_STATUS_TIME;
    motorStatus[1] &= ~MOTOR_STATUS_TACHO & ~MOTOR_STATUS_TIME;
  }
  else {
    motorStatus[which_motors - 1] &= ~MOTOR_STATUS_TACHO & ~MOTOR_STATUS_TIME;
  }

  return true;
}

void MotorBank::updateStatus() {
  uint32_t time = millis();

  for (uint8_t m = 0; m < 2; m++) {
    uint8_t ctrl = motorControl[m];
    uint8_t status = motorStatus[m];

    if (!(status & MOTOR_STATUS_MOVING)) {
							// Nothing to do
      continue;
    }

    if (status & MOTOR_STATUS_BRK) {
      if (lastEncoderTime[m] + 50 < time) {
        if (m == 0) {
          analogWrite(10, 0);
          PORTB &= ~0x01;
          PORTB |= 0x10;
        }
        else {
          analogWrite(11, 0);
          PORTB &= ~0x02;
          PORTB |= 0x20;
        }

        motorStatus[m] &= ~MOTOR_STATUS_MOVING;
      }
    }
    else if ((status & MOTOR_STATUS_TIME) &&
        (finishTime[m] <= time)) {			// Stop, time done
      if (ctrl & MOTOR_CONTROL_BRK) {
        motorBrake((Motor)(m + 1));
      }
      else {
        motorFloat((Motor)(m + 1));
      }

      motorStatus[m] &= ~MOTOR_STATUS_TIME;
    }
    else if ((status & MOTOR_STATUS_TACHO) &&
        ((lastEncoder[m] < motorEncoder[m] && motorEncoder[m] >= targetEncoder[m]) ||	// Stop, positive target reached
          (lastEncoder[m] > motorEncoder[m] && motorEncoder[m] <= targetEncoder[m]))) {	// Stop, negative target reached
      if (ctrl & MOTOR_CONTROL_BRK) {
        motorBrake((Motor)(m + 1));
      }
      else {
        motorFloat((Motor)(m + 1));
      }
      motorStatus[m] &= ~MOTOR_STATUS_TACHO;
    }
    else {
      if (lastEncoderTime[m] == -1) {
        lastEncoderTime[m] = time;
      }
      else if (lastEncoderTime[m] + 50 < time) {
        if (lastEncoder[m] == motorEncoder[m]) {
							// Stop, stall
          motorBrake((Motor)(m + 1));
          motorStatus[m] &= ~MOTOR_STATUS_TACHO & ~MOTOR_STATUS_TIME;
          motorStatus[m] |= MOTOR_STATUS_STALL;
        }
        else {
          lastEncoder[m] = motorEncoder[m];
          lastEncoderTime[m] = time;
        }
      }
    }
  }
}

void MotorBank::parseCommand(uint8_t *array, uint8_t bytes) {
  Motor which_motors;
  union {
    int8_t iv;
    uint8_t uv;
  } v8;
  union {
    int32_t iv;
    uint32_t uv;
  } v32;

#define ivalue8 v8.iv
#define uvalue8 v8.uv
#define ivalue32 v32.iv
#define uvalue32 v32.uv

  if (bytes < 1) {
    sendStatus(array, false);
    return;
  }

  switch (array[0]) {
    case 'g': {						// get....()
      if (bytes < 2) {
        sendStatus(array, false);
        return;
      }

      switch (array[1]) {
        case 'B': {					// getStatusByte()
          if (bytes != 3) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = getStatusByte(which_motors);

          array[0] = MSG_TYPE_MOTOR_U8;
          array[1] = uvalue8;

          UART_WriteArray(2, array);
        }
        break;
        case 'E': {					// getEncoderTarget()
          if (bytes != 3) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          ivalue32 = getEncoderTarget(which_motors);

          array[0] = MSG_TYPE_MOTOR_I32;
          write_32(array + 1, uvalue32);

          UART_WriteArray(5, array);
        }
        break;
        case 'P': {					// getEncoderPosition()
          if (bytes != 3) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          ivalue32 = getEncoderPosition(which_motors);

          array[0] = MSG_TYPE_MOTOR_I32;
          write_32(array + 1, uvalue32);

          UART_WriteArray(5, array);
        }
        break;
        case 'S': {					// getSpeed()
          if (bytes != 3) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          ivalue8 = getSpeed(which_motors);

          array[0] = MSG_TYPE_MOTOR_I8;
          array[1] = uvalue8;

          UART_WriteArray(2, array);
        }
        break;
        case 'T': {					// getTimeToRun()
          if (bytes != 3) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue32 = getTimeToRun(which_motors);

          array[0] = MSG_TYPE_MOTOR_U32;
          write_32(array + 1, uvalue32);

          UART_WriteArray(5, array);
        }
        break;
      }
    }
    break;
    case 'i': {						// is....()
      if (bytes != 3) {
        sendStatus(array, false);
        return;
      }

      which_motors = (Motor)array[2];

      switch (array[1]) {
        case 'T': {					// isTimeDone()
          uvalue8 = isTimeDone(which_motors);
        }
        break;
        case 'P': {					// isTachoDone()
          uvalue8 = isTachoDone(which_motors);
        }
        break;
      }

      array[0] = MSG_TYPE_MOTOR_U8;
      array[1] = uvalue8;
      UART_WriteArray(2, array);
    }
    break;
    case 'r': {						// reset...() or run...()
      if (bytes == 1) {					// reset()
        sendStatus(array, reset());
	return;
      }

      switch (array[1]) {
        case 'D': {					// runDegrees()
          if (bytes != 11) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = array[4];
          uvalue32 = read_32(array + 5);

          uvalue8 = runDegrees(which_motors, (Direction)array[3], ivalue8, ivalue32,
                               (Completion_Wait)array[9], (Next_Action)array[10]);

          array[0] = MSG_TYPE_MOTOR_U8;
          array[1] = uvalue8;
          UART_WriteArray(2, array);
        }
        break;
        case 'E': {					// resetEncoder()
          if (bytes != 3) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          sendStatus(array, resetEncoder(which_motors));
        }
        break;
        case 'M': {					// runMilliSeconds()
          if (bytes != 11) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = array[4];
          uvalue32 = read_32(array + 5);

          uvalue8 = runMilliSeconds(which_motors, (Direction)array[3], ivalue8, uvalue32,
                                    (Completion_Wait)array[9], (Next_Action)array[10]);

          array[0] = MSG_TYPE_MOTOR_U8;
          array[1] = uvalue8;
          UART_WriteArray(2, array);
        }
        break;
        case 'P': {					// runTachometer()
          if (bytes != 12) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = array[4];
          uvalue32 = read_32(array + 5);

          uvalue8 = runTachometer(which_motors, (Direction)array[3], ivalue8, ivalue32,
                                  (Move)array[9],
                                  (Completion_Wait)array[10], (Next_Action)array[11]);

          array[0] = MSG_TYPE_MOTOR_U8;
          array[1] = uvalue8;
          UART_WriteArray(2, array);
        }
        break;
        case 'R': {					// runRotations()
          if (bytes != 11) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = array[4];
          uvalue32 = read_32(array + 5);

          uvalue8 = runRotations(which_motors, (Direction)array[3], ivalue8, ivalue32,
                                 (Completion_Wait)array[9], (Next_Action)array[10]);

          array[0] = MSG_TYPE_MOTOR_U8;
          array[1] = uvalue8;
          UART_WriteArray(2, array);
        }
        break;
        case 'U': {					// runUnlimited()
          if (bytes != 5) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = array[4];

          runUnlimited(which_motors, (Direction)array[3], ivalue8);
        }
        break;
      }
    }
    break;
    case 's': {						// set....()
      if (bytes < 2) {
        sendStatus(array, false);
        return;
      }

      switch (array[1]) {
        case 'E': {					// setEncoderTarget()
          if (bytes != 7) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue32 = read_32(array + 3);

          sendStatus(array, setEncoderTarget(which_motors, ivalue32));
        }
        break;
        case 'I': {					// startBothInSync()
          if (bytes != 2) {
            sendStatus(array, false);
            return;
          }

          sendStatus(array, startBothInSync());
        }
        break;
        case 'S': {					// setSpeed()
          if (bytes != 4) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue8 = array[3];

          sendStatus(array, setSpeed(which_motors, ivalue8));
        }
        break;
        case 'T': {					// setTimeToRun()
          if (bytes != 7) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          uvalue32 = read_32(array + 3);

          sendStatus(array, setTimeToRun(which_motors, uvalue32));
        }
        break;
        case 'Y': {					// setEncoderSpeedTimeAndControl()
          int32_t encoder;

          if (bytes != 13) {
            sendStatus(array, false);
            return;
          }
          which_motors = (Motor)array[2];
          uvalue32 = read_32(array + 3);
          encoder = ivalue32;
          uvalue8 = array[7];
          uvalue32 = read_32(array + 8);

          sendStatus(array, setEncoderSpeedTimeAndControl(which_motors, encoder, ivalue8, uvalue32, array[12]));
        }
        break;
        case 'Z': {					// setSpeedTimeAndControl()
          if (bytes != 9) {
            sendStatus(array, false);
            return;
          }
          which_motors = (Motor)array[2];
          uvalue8 = array[3];
          uvalue32 = read_32(array + 4);

          sendStatus(array, setSpeedTimeAndControl(which_motors, ivalue8, uvalue32, array[8]));
        }
        break;
        case 'p': {					// stop()
          if (bytes != 4) {
            sendStatus(array, false);
            return;
          }

          which_motors = (Motor)array[2];
          sendStatus(array, stop(which_motors, (Next_Action)array[3]));
        }
        break;
      }
    }
    break;
    case 'w': {						// wait....()
      if (bytes != 3) {
        sendStatus(array, false);
        return;
      }

      which_motors = (Motor)array[2];

      switch (array[1]) {
        case 'T': {					// waitUntilTimeDone()
          uvalue8 = waitUntilTimeDone(which_motors);
        }
        break;
        case 'P': {					// waitUntilTachoDone()
          uvalue8 = waitUntilTachoDone(which_motors);
        }
        break;
      }

      array[0] = MSG_TYPE_MOTOR_U8;
      array[1] = uvalue8;
      UART_WriteArray(2, array);
    }
    break;
  }
}
