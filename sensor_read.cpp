#include "sensor_read.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {

constexpr uint8_t kAdsRegConversion = 0x00;
constexpr uint8_t kAdsRegConfig = 0x01;

constexpr uint16_t kAdsOsSingle = 0x8000;
constexpr uint16_t kAdsModeSingle = 0x0100;
constexpr uint16_t kAdsDr128Sps = 0x0080;
constexpr uint16_t kAdsCompDisable = 0x0003;
constexpr uint16_t kAdsPgaOne = 0x0200;
constexpr uint16_t kAdsMuxSingleEnded[SENSOR_COUNT] = {0x4000, 0x5000, 0x6000};

float clampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

uint16_t adsGainToConfigBits(adsGain_t gain) {
  switch (gain) {
    case GAIN_TWOTHIRDS:
      return 0x0000;
    case GAIN_ONE:
      return 0x0200;
    case GAIN_TWO:
      return 0x0400;
    case GAIN_FOUR:
      return 0x0600;
    case GAIN_EIGHT:
      return 0x0800;
    case GAIN_SIXTEEN:
      return 0x0A00;
    default:
      return kAdsPgaOne;
  }
}

bool adsWriteConfig(I2cBusContext *bus, uint8_t address, uint16_t config) {
  const uint8_t bytes[2] = {
      static_cast<uint8_t>((config >> 8) & 0xFFU),
      static_cast<uint8_t>(config & 0xFFU),
  };
  return i2cBusWriteRegister(bus, address, kAdsRegConfig, bytes, sizeof(bytes));
}

bool adsReadConversion(I2cBusContext *bus, uint8_t address, int16_t *value) {
  if (value == nullptr) {
    return false;
  }

  uint8_t bytes[2] = {0U, 0U};
  if (!i2cBusReadRegister(bus, address, kAdsRegConversion, bytes, sizeof(bytes))) {
    return false;
  }

  *value = static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
  return true;
}

bool adsReadSingleEnded(I2cBusContext *bus,
                        uint8_t address,
                        adsGain_t gain,
                        uint8_t channel,
                        int16_t *result) {
  if (bus == nullptr || result == nullptr || channel >= SENSOR_COUNT) {
    return false;
  }

  const uint16_t config = kAdsOsSingle |
                          kAdsMuxSingleEnded[channel] |
                          adsGainToConfigBits(gain) |
                          kAdsModeSingle |
                          kAdsDr128Sps |
                          kAdsCompDisable;

  if (!adsWriteConfig(bus, address, config)) {
    return false;
  }

  delay(10);
  return adsReadConversion(bus, address, result);
}

}  // namespace

bool sensorReadInit(SensorReadContext *ctx, const SensorReadConfig *config, I2cBusContext *bus) {
  if (ctx == nullptr || config == nullptr || bus == nullptr) {
    return false;
  }

  ctx->bus = bus;
  ctx->config = *config;
  ctx->initialized = false;
  ctx->filter_seeded = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    ctx->filtered_raw[i] = 0.0f;
    ctx->sensor_active[i] = false;
  }

  if (!i2cBusProbe(ctx->bus, config->i2c_address)) {
    return false;
  }

  ctx->initialized = true;
  return true;
}

void sensorReadResetFilters(SensorReadContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    ctx->filtered_raw[i] = 0.0f;
    ctx->sensor_active[i] = false;
  }

  ctx->filter_seeded = false;
}

bool sensorReadReadRaw(SensorReadContext *ctx, SensorRawData *raw_frame) {
  if (ctx == nullptr || raw_frame == nullptr || !ctx->initialized) {
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!adsReadSingleEnded(ctx->bus, ctx->config.i2c_address, ctx->config.gain, i, &raw_frame->value[i])) {
      raw_frame->valid = false;
      return false;
    }
  }

  raw_frame->valid = true;
  raw_frame->timestamp_ms = millis();
  return true;
}

bool sensorReadProcess(SensorReadContext *ctx,
                       const SensorRawData *raw_frame,
                       CalibrationData *calibration,
                       SensorProcessedData *processed_frame) {
  if (ctx == nullptr || raw_frame == nullptr || calibration == nullptr || processed_frame == nullptr) {
    return false;
  }

  if (!ctx->initialized || !raw_frame->valid || !calibration->valid) {
    return false;
  }

  SensorProcessedData local_frame;
  memset(&local_frame, 0, sizeof(local_frame));
  local_frame.valid = true;
  local_frame.timestamp_ms = raw_frame->timestamp_ms;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    local_frame.raw[i] = raw_frame->value[i];

    const float raw_value = static_cast<float>(raw_frame->value[i]);
    float filtered = raw_value;
    if (ctx->filter_seeded) {
      filtered = (ctx->config.filter_alpha * raw_value) +
                 ((1.0f - ctx->config.filter_alpha) * ctx->filtered_raw[i]);
    }

    local_frame.filtered[i] = filtered;
    float delta = filtered - calibration->baseline[i];
    float signal = fabsf(delta) * calibration->scale[i];
    const bool was_active = ctx->sensor_active[i];
    const float activity_threshold =
        was_active ? ctx->config.signal_deactivate_threshold : ctx->config.signal_activate_threshold;
    const bool is_active = signal >= activity_threshold;

    // Follow slow idle drift only while the channel is inactive.
    if (!is_active) {
      calibration->baseline[i] += ctx->config.baseline_follow_alpha * delta;
      delta = filtered - calibration->baseline[i];
      signal = fabsf(delta) * calibration->scale[i];
    }

    if (signal < ctx->config.signal_floor) {
      signal = 0.0f;
    }
    const float scaled_signal = is_active ? signal : 0.0f;
    ctx->sensor_active[i] = is_active;

    local_frame.delta[i] = delta;
    local_frame.signal[i] = signal;
    local_frame.scaled_signal[i] = scaled_signal;
    local_frame.total_signal += signal;
    if (signal > local_frame.peak_signal) {
      local_frame.peak_signal = signal;
    }
    local_frame.scaled_total_signal += scaled_signal;
    if (scaled_signal > local_frame.scaled_peak_signal) {
      local_frame.scaled_peak_signal = scaled_signal;
    }

    ctx->filtered_raw[i] = local_frame.filtered[i];
  }

  ctx->filter_seeded = true;
  *processed_frame = local_frame;
  return true;
}
