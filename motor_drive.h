#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

#include "i2c_bus.h"

struct MotorChannelPair {
  uint8_t forward_channel;
  uint8_t reverse_channel;
};

enum MotionPrimitive {
  MOTION_STOP = 0,
  MOTION_TRACKING,
  MOTION_ROTATE
};

struct MotionCommand {
  MotionPrimitive primitive;
  int8_t direction;
};

struct MecanumWheelPwm {
  int16_t front_left;
  int16_t front_right;
  int16_t rear_left;
  int16_t rear_right;
};

struct MotorDriveConfig {
  uint8_t i2c_address;
  uint16_t pwm_frequency_hz;
  int16_t pwm_max;
  int16_t motor_pwm;
  bool front_left_reversed;
  bool front_right_reversed;
  bool rear_left_reversed;
  bool rear_right_reversed;
  MotorChannelPair front_left;
  MotorChannelPair front_right;
  MotorChannelPair rear_left;
  MotorChannelPair rear_right;
};

struct MotorDriveContext {
  I2cBusContext *bus;
  MotorDriveConfig config;
  MecanumWheelPwm last_wheel_pwm;
  bool initialized;
  bool output_seeded;
};

bool motorDriveInit(MotorDriveContext *ctx, const MotorDriveConfig *config, I2cBusContext *bus);
void motorDriveStopAll(MotorDriveContext *ctx);
MecanumWheelPwm motorDriveApplyCommand(MotorDriveContext *ctx, const MotionCommand *command);
const char *motionPrimitiveName(MotionPrimitive primitive);

#endif
