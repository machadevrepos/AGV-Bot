#include "motor_drive.h"

#include <Arduino.h>
#include <string.h>

namespace {

int16_t applyDeadzone(const MotorDriveContext *ctx, int16_t pwm_value) {
  if (ctx == nullptr) {
    return 0;
  }

  if (pwm_value == 0) {
    return 0;
  }

  const int16_t magnitude = abs(pwm_value);
  if (magnitude < ctx->config.pwm_deadzone) {
    return 0;
  }

  return pwm_value;
}

}  // namespace

bool motorDriveInit(MotorDriveContext *ctx, const MotorDriveConfig *config, TwoWire *wire) {
  if (ctx == nullptr || config == nullptr || wire == nullptr) {
    return false;
  }

  ctx->config = *config;
  ctx->initialized = false;
  ctx->pca = Adafruit_PWMServoDriver(config->i2c_address, *wire);

  if (!ctx->pca.begin()) {
    return false;
  }

  ctx->pca.setPWMFreq(config->pwm_frequency_hz);
  ctx->initialized = true;
  motorDriveStopAll(ctx);
  return true;
}

void motorDriveWriteChannel(MotorDriveContext *ctx, uint8_t channel, uint16_t pwm_value) {
  if (ctx == nullptr || !ctx->initialized) {
    return;
  }

  const uint16_t clamped = (pwm_value > static_cast<uint16_t>(ctx->config.pwm_max))
                               ? static_cast<uint16_t>(ctx->config.pwm_max)
                               : pwm_value;
  ctx->pca.setPWM(channel, 0, clamped);
}

int16_t motorDriveClamp(const MotorDriveContext *ctx, int16_t pwm_value) {
  if (ctx == nullptr) {
    return 0;
  }

  if (pwm_value > ctx->config.pwm_max) {
    return ctx->config.pwm_max;
  }

  if (pwm_value < -ctx->config.pwm_max) {
    return -ctx->config.pwm_max;
  }

  return pwm_value;
}

void motorDriveDriveMotor(MotorDriveContext *ctx, uint8_t forward_channel, uint8_t reverse_channel, int16_t pwm_value) {
  if (ctx == nullptr || !ctx->initialized) {
    return;
  }

  const int16_t clamped = applyDeadzone(ctx, motorDriveClamp(ctx, pwm_value));

  if (clamped > 0) {
    motorDriveWriteChannel(ctx, forward_channel, static_cast<uint16_t>(clamped));
    motorDriveWriteChannel(ctx, reverse_channel, 0U);
  } else if (clamped < 0) {
    motorDriveWriteChannel(ctx, forward_channel, 0U);
    motorDriveWriteChannel(ctx, reverse_channel, static_cast<uint16_t>(-clamped));
  } else {
    motorDriveWriteChannel(ctx, forward_channel, 0U);
    motorDriveWriteChannel(ctx, reverse_channel, 0U);
  }
}

void motorDriveSetMotors(MotorDriveContext *ctx, int16_t left_pwm, int16_t right_pwm) {
  if (ctx == nullptr || !ctx->initialized) {
    return;
  }

  int16_t left_command = left_pwm;
  int16_t right_command = right_pwm;

  if (ctx->config.invert_left) {
    left_command = -left_command;
  }
  if (ctx->config.invert_right) {
    right_command = -right_command;
  }

  motorDriveDriveMotor(ctx,
                       ctx->config.left_front.forward_channel,
                       ctx->config.left_front.reverse_channel,
                       left_command);
  motorDriveDriveMotor(ctx,
                       ctx->config.left_back.forward_channel,
                       ctx->config.left_back.reverse_channel,
                       left_command);
  motorDriveDriveMotor(ctx,
                       ctx->config.right_front.forward_channel,
                       ctx->config.right_front.reverse_channel,
                       right_command);
  motorDriveDriveMotor(ctx,
                       ctx->config.right_back.forward_channel,
                       ctx->config.right_back.reverse_channel,
                       right_command);
}

void motorDriveStopAll(MotorDriveContext *ctx) {
  if (ctx == nullptr || !ctx->initialized) {
    return;
  }

  motorDriveSetMotors(ctx, 0, 0);
}

