#include "motor_drive.h"

#include <Arduino.h>
#include <string.h>

namespace {

constexpr uint8_t kPcaMode1 = 0x00;
constexpr uint8_t kPcaMode2 = 0x01;
constexpr uint8_t kPcaPreScale = 0xFE;
constexpr uint8_t kPcaLed0OnL = 0x06;

int8_t applyReverse(int8_t direction, bool reversed) {
  return reversed ? static_cast<int8_t>(-direction) : direction;
}

int16_t directionToPwm(const MotorDriveContext *ctx, int8_t direction, bool reversed) {
  if (ctx == nullptr || direction == 0) {
    return 0;
  }

  const int8_t final_direction = applyReverse(direction, reversed);
  return (final_direction > 0) ? ctx->config.motor_pwm : static_cast<int16_t>(-ctx->config.motor_pwm);
}

MecanumWheelPwm makeWheelPwm(const MotorDriveContext *ctx,
                             int8_t fl_direction,
                             int8_t fr_direction,
                             int8_t rl_direction,
                             int8_t rr_direction) {
  MecanumWheelPwm wheel_pwm;
  memset(&wheel_pwm, 0, sizeof(wheel_pwm));

  if (ctx == nullptr) {
    return wheel_pwm;
  }

  wheel_pwm.front_left = motorDriveClamp(
      ctx, directionToPwm(ctx, fl_direction, ctx->config.front_left_reversed));
  wheel_pwm.front_right = motorDriveClamp(
      ctx, directionToPwm(ctx, fr_direction, ctx->config.front_right_reversed));
  wheel_pwm.rear_left = motorDriveClamp(
      ctx, directionToPwm(ctx, rl_direction, ctx->config.rear_left_reversed));
  wheel_pwm.rear_right = motorDriveClamp(
      ctx, directionToPwm(ctx, rr_direction, ctx->config.rear_right_reversed));
  return wheel_pwm;
}

bool wheelPwmEquals(const MecanumWheelPwm *lhs, const MecanumWheelPwm *rhs) {
  if (lhs == nullptr || rhs == nullptr) {
    return false;
  }

  return lhs->front_left == rhs->front_left &&
         lhs->front_right == rhs->front_right &&
         lhs->rear_left == rhs->rear_left &&
         lhs->rear_right == rhs->rear_right;
}

bool pcaWrite8(I2cBusContext *bus, uint8_t address, uint8_t reg, uint8_t value) {
  return i2cBusWriteRegister(bus, address, reg, &value, 1U);
}

bool pcaRead8(I2cBusContext *bus, uint8_t address, uint8_t reg, uint8_t *value) {
  return i2cBusReadRegister(bus, address, reg, value, 1U);
}

bool pcaSetPwmFreq(I2cBusContext *bus, uint8_t address, float freq_hz) {
  if (bus == nullptr || freq_hz <= 0.0f) {
    return false;
  }

  float prescale_value = 25000000.0f;
  prescale_value /= 4096.0f;
  prescale_value /= freq_hz;
  prescale_value -= 1.0f;

  const uint8_t prescale = static_cast<uint8_t>(prescale_value + 0.5f);
  uint8_t old_mode = 0U;
  if (!pcaRead8(bus, address, kPcaMode1, &old_mode)) {
    return false;
  }

  const uint8_t sleep_mode = static_cast<uint8_t>((old_mode & 0x7FU) | 0x10U);
  if (!pcaWrite8(bus, address, kPcaMode1, sleep_mode)) {
    return false;
  }
  if (!pcaWrite8(bus, address, kPcaPreScale, prescale)) {
    return false;
  }
  if (!pcaWrite8(bus, address, kPcaMode1, old_mode)) {
    return false;
  }

  delay(5);
  return pcaWrite8(bus, address, kPcaMode1, static_cast<uint8_t>(old_mode | 0xA1U));
}

bool pcaInitDevice(I2cBusContext *bus, const MotorDriveConfig *config) {
  if (bus == nullptr || config == nullptr) {
    return false;
  }

  if (!pcaWrite8(bus, config->i2c_address, kPcaMode1, 0x00U)) {
    return false;
  }
  if (!pcaWrite8(bus, config->i2c_address, kPcaMode2, 0x04U)) {
    return false;
  }

  delay(10);
  return pcaSetPwmFreq(bus, config->i2c_address, static_cast<float>(config->pwm_frequency_hz));
}

bool usesContiguousDefaultMotorChannels(const MotorDriveConfig *config) {
  if (config == nullptr) {
    return false;
  }

  return config->front_right.forward_channel == 0U &&
         config->front_right.reverse_channel == 1U &&
         config->front_left.forward_channel == 2U &&
         config->front_left.reverse_channel == 3U &&
         config->rear_left.forward_channel == 4U &&
         config->rear_left.reverse_channel == 5U &&
         config->rear_right.forward_channel == 6U &&
         config->rear_right.reverse_channel == 7U;
}

void encodePcaCounts(int16_t pwm_value, uint16_t *forward_off_count, uint16_t *reverse_off_count) {
  if (forward_off_count == nullptr || reverse_off_count == nullptr) {
    return;
  }

  if (pwm_value > 0) {
    *forward_off_count = static_cast<uint16_t>(pwm_value);
    *reverse_off_count = 0U;
  } else if (pwm_value < 0) {
    *forward_off_count = 0U;
    *reverse_off_count = static_cast<uint16_t>(-pwm_value);
  } else {
    *forward_off_count = 0U;
    *reverse_off_count = 0U;
  }
}

bool pcaWriteFrame(I2cBusContext *bus, const MotorDriveConfig *config, const MecanumWheelPwm *wheel_pwm) {
  if (bus == nullptr || config == nullptr || wheel_pwm == nullptr) {
    return false;
  }

  if (!usesContiguousDefaultMotorChannels(config)) {
    return false;
  }

  uint16_t off_counts[8] = {0U};
  encodePcaCounts(wheel_pwm->front_right, &off_counts[0], &off_counts[1]);
  encodePcaCounts(wheel_pwm->front_left, &off_counts[2], &off_counts[3]);
  encodePcaCounts(wheel_pwm->rear_left, &off_counts[4], &off_counts[5]);
  encodePcaCounts(wheel_pwm->rear_right, &off_counts[6], &off_counts[7]);

  uint8_t payload[32];
  for (uint8_t i = 0; i < 8U; ++i) {
    const uint16_t off_count = off_counts[i];
    const uint8_t base = static_cast<uint8_t>(i * 4U);
    payload[base + 0U] = 0U;
    payload[base + 1U] = 0U;
    payload[base + 2U] = static_cast<uint8_t>(off_count & 0xFFU);
    payload[base + 3U] = static_cast<uint8_t>((off_count >> 8) & 0x0FU);
  }

  return i2cBusWriteRegister(bus, config->i2c_address, kPcaLed0OnL, payload, sizeof(payload));
}

}  // namespace

bool motorDriveInit(MotorDriveContext *ctx, const MotorDriveConfig *config, I2cBusContext *bus) {
  if (ctx == nullptr || config == nullptr || bus == nullptr) {
    return false;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->bus = bus;
  ctx->config = *config;

  if (!i2cBusProbe(ctx->bus, config->i2c_address)) {
    return false;
  }
  if (!pcaInitDevice(ctx->bus, config)) {
    return false;
  }

  ctx->initialized = true;
  motorDriveStopAll(ctx);
  return true;
}

bool motorDriveWriteChannel(MotorDriveContext *ctx, uint8_t channel, uint16_t pwm_value) {
  if (ctx == nullptr || !ctx->initialized || channel > 15U) {
    return false;
  }

  const uint16_t clamped = (pwm_value > static_cast<uint16_t>(ctx->config.pwm_max))
                               ? static_cast<uint16_t>(ctx->config.pwm_max)
                               : pwm_value;
  const uint8_t bytes[4] = {
      0U,
      0U,
      static_cast<uint8_t>(clamped & 0xFFU),
      static_cast<uint8_t>((clamped >> 8) & 0x0FU),
  };
  return i2cBusWriteRegister(ctx->bus,
                             ctx->config.i2c_address,
                             static_cast<uint8_t>(kPcaLed0OnL + (4U * channel)),
                             bytes,
                             sizeof(bytes));
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

bool motorDriveDriveMotor(MotorDriveContext *ctx,
                         uint8_t forward_channel,
                         uint8_t reverse_channel,
                         int16_t pwm_value) {
  if (ctx == nullptr || !ctx->initialized) {
    return false;
  }

  const int16_t clamped = motorDriveClamp(ctx, pwm_value);
  if (clamped > 0) {
    return motorDriveWriteChannel(ctx, forward_channel, static_cast<uint16_t>(clamped)) &&
           motorDriveWriteChannel(ctx, reverse_channel, 0U);
  } else if (clamped < 0) {
    return motorDriveWriteChannel(ctx, forward_channel, 0U) &&
           motorDriveWriteChannel(ctx, reverse_channel, static_cast<uint16_t>(-clamped));
  } else {
    return motorDriveWriteChannel(ctx, forward_channel, 0U) &&
           motorDriveWriteChannel(ctx, reverse_channel, 0U);
  }
}

MecanumWheelPwm motorDriveApplyMotion(MotorDriveContext *ctx, MotionPrimitive primitive, int8_t direction) {
  MecanumWheelPwm wheel_pwm;
  memset(&wheel_pwm, 0, sizeof(wheel_pwm));

  if (ctx == nullptr) {
    return wheel_pwm;
  }

  switch (primitive) {
    case MOTION_FORWARD:
      wheel_pwm = makeWheelPwm(ctx, 1, 1, 1, 1);
      break;

    case MOTION_ARC:
      wheel_pwm = (direction < 0) ? makeWheelPwm(ctx, 1, 0, 1, 0) : makeWheelPwm(ctx, 0, 1, 0, 1);
      break;

    case MOTION_ROTATE:
      wheel_pwm = (direction < 0) ? makeWheelPwm(ctx, 1, -1, 1, -1)
                                  : makeWheelPwm(ctx, -1, 1, -1, 1);
      break;

    case MOTION_REAR_PIVOT:
      wheel_pwm = (direction < 0) ? makeWheelPwm(ctx, -1, 1, 0, 0)
                                  : makeWheelPwm(ctx, 1, -1, 0, 0);
      break;

    case MOTION_STOP:
    default:
      wheel_pwm = makeWheelPwm(ctx, 0, 0, 0, 0);
      break;
  }

  if (ctx->output_seeded && wheelPwmEquals(&ctx->last_wheel_pwm, &wheel_pwm)) {
    return wheel_pwm;
  }

  bool wrote = false;
  if (usesContiguousDefaultMotorChannels(&ctx->config)) {
    wrote = pcaWriteFrame(ctx->bus, &ctx->config, &wheel_pwm);
  } else {
    wrote = motorDriveDriveMotor(ctx,
                                 ctx->config.front_left.forward_channel,
                                 ctx->config.front_left.reverse_channel,
                                 wheel_pwm.front_left) &&
            motorDriveDriveMotor(ctx,
                                 ctx->config.front_right.forward_channel,
                                 ctx->config.front_right.reverse_channel,
                                 wheel_pwm.front_right) &&
            motorDriveDriveMotor(ctx,
                                 ctx->config.rear_left.forward_channel,
                                 ctx->config.rear_left.reverse_channel,
                                 wheel_pwm.rear_left) &&
            motorDriveDriveMotor(ctx,
                                 ctx->config.rear_right.forward_channel,
                                 ctx->config.rear_right.reverse_channel,
                                 wheel_pwm.rear_right);
  }

  if (wrote) {
    ctx->last_wheel_pwm = wheel_pwm;
    ctx->output_seeded = true;
  }

  return wheel_pwm;
}

MecanumWheelPwm motorDriveApplyCommand(MotorDriveContext *ctx, const MotionCommand *command) {
  if (command == nullptr) {
    return motorDriveApplyMotion(ctx, MOTION_STOP, 0);
  }
  return motorDriveApplyMotion(ctx, command->primitive, command->direction);
}

void motorDriveStopAll(MotorDriveContext *ctx) {
  motorDriveApplyMotion(ctx, MOTION_STOP, 0);
}

const char *motionPrimitiveName(MotionPrimitive primitive) {
  switch (primitive) {
    case MOTION_FORWARD:
      return "FORWARD";
    case MOTION_ARC:
      return "ARC";
    case MOTION_ROTATE:
      return "ROTATE";
    case MOTION_REAR_PIVOT:
      return "REAR_PIVOT";
    case MOTION_STOP:
    default:
      return "STOP";
  }
}
