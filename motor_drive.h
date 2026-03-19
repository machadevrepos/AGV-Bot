#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

struct MotorChannelPair {
  uint8_t forward_channel;
  uint8_t reverse_channel;
};

struct MotorDriveConfig {
  uint8_t i2c_address;
  uint16_t pwm_frequency_hz;
  int16_t pwm_max;
  int16_t pwm_deadzone;
  bool invert_left;
  bool invert_right;
  MotorChannelPair right_front;
  MotorChannelPair right_back;
  MotorChannelPair left_front;
  MotorChannelPair left_back;
};

struct MotorDriveContext {
  Adafruit_PWMServoDriver pca;
  MotorDriveConfig config;
  bool initialized;
};

bool motorDriveInit(MotorDriveContext *ctx, const MotorDriveConfig *config, TwoWire *wire);
void motorDriveStopAll(MotorDriveContext *ctx);
void motorDriveWriteChannel(MotorDriveContext *ctx, uint8_t channel, uint16_t pwm_value);
int16_t motorDriveClamp(const MotorDriveContext *ctx, int16_t pwm_value);
void motorDriveDriveMotor(MotorDriveContext *ctx, uint8_t forward_channel, uint8_t reverse_channel, int16_t pwm_value);
void motorDriveSetMotors(MotorDriveContext *ctx, int16_t left_pwm, int16_t right_pwm);

#endif
