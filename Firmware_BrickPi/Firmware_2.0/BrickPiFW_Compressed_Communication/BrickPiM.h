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

#ifndef __BrickPiM_h_
#define __BrickPiM_h_

#include "Arduino.h"

#define BrickPiVersion 2 // 1 // Tell the compiler what version of HW to compile for. 1 is 1.5.1, and 2 is 1.7.3

#ifndef BrickPiVersion
  #error "BrickPiVersion required, but not defined"
#endif

#if ((BrickPiVersion != 1) && (BrickPiVersion != 2))
  #error "BrickPi version not supported"
#endif

#define PORT_A 0
#define PORT_B 1

#define MOTOR_CONTROL_SPEED      0x01
#define MOTOR_CONTROL_RAMP       0x02
#define MOTOR_CONTROL_RELATIVE   0x04
#define MOTOR_CONTROL_TACHO      0x08
#define MOTOR_CONTROL_BRK        0x10
#define MOTOR_CONTROL_ON         0x20
#define MOTOR_CONTROL_TIME       0x40
#define MOTOR_CONTROL_GO         0x80

#define MOTOR_STATUS_SPEED       0x01
#define MOTOR_STATUS_RAMP        0x02			// XXX Not used yet
#define MOTOR_STATUS_MOVING      0x04
#define MOTOR_STATUS_TACHO       0x08
#define MOTOR_STATUS_BRK         0x10
#define MOTOR_STATUS_OVERLOAD    0x20			// XXX not used yet
#define MOTOR_STATUS_TIME        0x40
#define MOTOR_STATUS_STALL       0x80

#define MSG_TYPE_MOTOR_OK       'M'
#define MSG_TYPE_MOTOR_NOK      MSG_TYPE_MOTOR_OK + 1
#define MSG_TYPE_MOTOR_I8       MSG_TYPE_MOTOR_OK + 2
#define MSG_TYPE_MOTOR_U8       MSG_TYPE_MOTOR_OK + 3
#define MSG_TYPE_MOTOR_I32      MSG_TYPE_MOTOR_OK + 4
#define MSG_TYPE_MOTOR_U32      MSG_TYPE_MOTOR_OK + 5

/* constants to be used by user programs */
/**
 * \enum Motor Motor selection related constants
 */
typedef enum {
  Motor_1     = 0x01,   /*!< Choose Motor 1 for selected operation */
  Motor_2     = 0x02,   /*!< Choose Motor 2 for selected operation  */
  Motor_Both  = 0x03    /*!< Choose Both Motors for selected operation */
} Motor;

/*
 * \enum Next_Action Next action related constants
 */
typedef enum {
  Next_Action_Float  = 0x00, /*!< stop and let the motor coast. */
  Next_Action_Brake = 0x01, /*!< apply brakes, and resist change to tachometer, but if tach position is forcibly changed, do not restore position */
  Next_Action_BrakeHold = 0x01 /*!< apply brakes, and restore externally forced change to tachometer */
  // XXX not implemented yet! Next_Action_BrakeHold = 0x02 /*!< apply brakes, and restore externally forced change to tachometer */
} Next_Action;

/**
 * \enum Direction Motor direction related constants.
 */
typedef enum {
  Direction_Reverse = 0x00,   /*!< Run motor in reverse direction */
  Direction_Forward = 0x01   /*!< Run motor in forward direction */
} Direction;

/*
 * \enum Move Tachometer movement related constants
 */
typedef enum {
  Move_Absolute = 0x00,   /*!< Move the tach to absolute value provided */
  Move_Relative = 0x01   /*!< Move the tach relative to previous position */
} Move;

/*
 * \enum SH_Completion_Wait Whether to wait for motor to finish it's current task or not
 */
typedef enum {
  Completion_Dont_Wait    = 0x00,  /*!< Don't wait for motor to finish, program will continue with next function */
  Completion_Wait_For     = 0x01  /*!< Wait for motor to finish, program will wait until current function finishes it's operation */
} Completion_Wait;

/**
  @brief This class defines methods for the NXShield Bank(s).
  */
class MotorBank {
public:
  MotorBank();

  /** Set the target encoder position for the motor
      "sE<uint8_t><LSB int32_t>"
    @param which_motor    Provide which motor to operate on
    @param target         Encode value to achieve
  */
  bool setEncoderTarget(Motor which_motor, int32_t target);

  /**
  Get the target encoder position for the motor
      "gE<uint8_t>"
    @param which_motor    Provide which motor to operate on
    @return long encoder value that the motor is trying to achieve.
  */
  int32_t getEncoderTarget(Motor which_motor);

  /**
  Set the speed of the motor
      "sS<uint8_t><int8_t>"
    @param which_motor    Provide which motor to operate on
    @param speed          The speed value between -100 and 100
  */
  bool setSpeed(Motor which_motor, int8_t speed);

  /**
  Get the speed of the motor
      "gS<uint8_t>"
    @param which_motor    Provide which motor to operate on
    @return  the speed value set to the motor
  */
  int8_t getSpeed(Motor which_motor);

  /**
  Set the time in milli seconds for which the motor should run for
      "sT<uint8_t><LSB uint32_t>"
    @param which_motor    Provide which motor to operate on
    @param mseconds   The time duration the motor should run
  */
  bool setTimeToRun(Motor which_motor, uint32_t mseconds);

  /**
  Get the time in milli seconds that the motor is running for
      "gT<uint8_t>"
    @param which_motor    Provide which motor to operate on
    @return  time the motor has been running since last start.
  */
  uint32_t getTimeToRun(Motor which_motor);

  /**
  Get the current encoder position of the motor in degrees
      "gP<uint8_t>"
    @param which_motor    Provide which motor to operate on
    @return              current encoder value
  */
  int32_t getEncoderPosition(Motor which_motor);

  /**
  Get the current status of the motor
      "gB<uint8_t>"
    @param which_motor    Provide which motor to operate on
    @return  The current status of the motor.
    This is a byte with various bits set based on motor's state.
    Refer to User Guide for details of bits.
  */
  uint8_t  getStatusByte(Motor which_motor);

  /**
  Reset all the set values for the motors
  Applies to all motors on this bank.
      "r"
  */
  bool reset();

  /**
  Start both motors at the same time to follow the set conditions
  This will execute the commands specified in the command register on both motors at once.
      "sI"
  */
  bool startBothInSync();

  /**
  Reset the current encoder position to zero for the motor
      "rE<uint8_t>"
    @param which_motor    Provide which motor to operate on
  */
  bool resetEncoder(Motor which_motor);

  /**
  Set the speed, duration to run, and control for the motor
      "sZ<uint8_t><int8_t><LSB uint32_t><uint8_t>"
    @param which_motors    Provide which motor(s) to operate on
    @param speed          Speed value between -100 and 100
    @param duration       time to run in milli seconds
    @param control        command register value
  */
  bool setSpeedTimeAndControl(Motor which_motors, int8_t speed,
                              uint32_t duration, uint8_t control);

  /**
  This function sets the speed, the number of seconds, and
  the control
      "sY<uint8_t><LSB int32_t><uint8_t><LSB uint32_t><uint8_t>"
    @param which_motors    Provide which motor(s) to operate on
    @param encoder        Target encoder position to achieve
    @param speed          Speed value between -100 and 100
    @param duration       time to run in seconds
    @param control        command register value
  */
  bool setEncoderSpeedTimeAndControl(Motor which_motors,
                                     int32_t encoder, int8_t speed,
                                     uint32_t duration, uint8_t control);

  /**
  Validate if the motor has finished running for the set time duration
      "iT<uint8_t>"
    @param which_motors    Provide which motor(s) to operate on
    @return                0 when motor(s) has completed a timed move properly,
    If the return value is non-zero, either motor has not finished yet or has encountered an error condition.
  */
  uint8_t isTimeDone(Motor which_motors);

  /**
  Wait until the motor has finished running for its set respective time duration
      "wT<uint8_t>"
    @param which_motors    Provide which motor(s) to operate on
    @return                function waits until when motor(s) has stopped, returns 0 if the set goal was achieved.
    If the return value is non-zero, you should check for error condition such as stall.
  */
  uint8_t waitUntilTimeDone(Motor which_motors);

  /**
  Validate if the motor has reached its set target tachometer position
      "iP<uint8_t>"
    @param which_motors    Provide which motor(s) to operate on
    @return                0 when motor(s) has completed a encoder based move properly,
    If the return value is non-zero, either motor has not finished yet or has encountered an error condition.
  */
  uint8_t isTachoDone(Motor which_motors);

  /**
  Wait until the motor has reached its set target tachometer position
      "wP<uint8_t>"
    @param which_motors    Provide which motor(s) to operate on
    @return                function waits until when motor(s) has stopped, returns 0 if the set goal was achieved.
    If the return value is non-zero, you should check for error condition such as stall.
  */
  uint8_t waitUntilTachoDone(Motor which_motors);

  /**
  Run the motor endlessly at the desired speed in the desired direction
      "rU<uint8_t><uint8_t><int8_t>"
   @param which_motors     specify the motor(s) to operate on
   @param direction        specify the direction to run the motor
   @param speed            the speed value (between -100 and 100)
   @return  Starts the motors and function returns immediately
  */
  void runUnlimited(Motor which_motors, Direction direction,
                    int8_t speed);

  /** Run the motor for a set duration at a set speed and do the next action
      "rM<uint8_t><uint8_t><int8_t><LSB uint32_t><uint8_t><uint8_t>"
   @param which_motors     specify the motor(s) to operate on
   @param direction        specify the direction to run the motor
   @param speed            the speed value (between -100 and 100)
   @param duration         in milli seconds
   @param wait_for_completion    whether this API should wait for competion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t runMilliSeconds(Motor which_motors, Direction direction,
                          int8_t speed, uint32_t duration,
                          Completion_Wait wait_for_completion,
                          Next_Action next_action);

  /**
  run until the tachometer target has been reached and do next action
      "rP<uint8_t><uint8_t><int8_t><LSB int32_t><uint8_t><uint8_t><uint8_t>"
   @param which_motors     specify the motor(s) to operate on
   @param direction        specify the direction to run the motor
   @param speed            the speed value (between -100 and 100)
   @param tachometer       the target for the encoder value to achieve.
   @param relative         is the tachometer relative or absolute.
   @param wait_for_completion    whether this API should wait for competion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t runTachometer(Motor which_motors, Direction direction,
                        int8_t speed, int32_t tachometer,
                        Move relative,
                        Completion_Wait wait_for_completion,
                        Next_Action next_action);

  /**
  Run the motor for a set number of degrees and proceed to the next action
      "rD<uint8_t><uint8_t><int8_t><LSB int32_t><uint8_t><uint8_t>"
   @param which_motors     specify the motor(s) to operate on
   @param direction        specify the direction to run the motor
   @param speed            the speed value (between -100 and 100)
   @param degrees          The degrees the motor should turn through
   @param wait_for_completion    whether this API should wait for competion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t runDegrees(Motor which_motors, Direction direction,
                     int8_t speed, int32_t degrees,
                     Completion_Wait wait_for_completion,
                     Next_Action next_action);

 /**
  Run the motor for a set number of complete rotations and proceed to the next action
      "rR<uint8_t><uint8_t><int8_t><LSB int32_t><uint8_t><uint8_t>"
   @param which_motors     specify the motor(s) to operate on
   @param direction        specify the direction to run the motor
   @param speed            the speed value (between -100 and 100)
   @param rotations        The rotations the motor should rotate through
   @param wait_for_completion    whether this API should wait for competion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t runRotations(Motor which_motors, Direction direction,
                       int8_t speed, int32_t rotations,
                       Completion_Wait wait_for_completion,
                       Next_Action next_action);

  /**
  stop the motor and do the next action
      "sp<uint8_t><uint8_t>"
   @param which_motors     specify the motor(s) to operate on
   @param next_action      for these motor being operated on
  */
  bool stop(Motor which_motors, Next_Action next_action);

  void parseCommand(uint8_t *array, uint8_t bytes);
  void updateStatus();

private:
  uint32_t finishTime[2];
  uint8_t motorControl[2];
  int8_t motorSpeed[2];
  uint8_t motorStatus[2];
  int32_t targetEncoder[2];
  int32_t lastEncoder[2];

  bool motorEnable(Motor which_motor);
  void motorFloat(Motor which_motor);
  void motorBrake(Motor which_motor);

};
#endif
