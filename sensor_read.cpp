#include "sensor_read.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

bool sensorReadInit(SensorReadContext *ctx, const SensorReadConfig *config, TwoWire *wire) {
  if (ctx == nullptr || config == nullptr || wire == nullptr) {
    return false;
  }

  ctx->config = *config;
  ctx->initialized = false;
  ctx->filter_seeded = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    ctx->filtered_raw[i] = 0.0f;
  }

  if (!ctx->ads.begin(config->i2c_address, wire)) {
    return false;
  }

  ctx->ads.setGain(config->gain);
  ctx->initialized = true;
  ctx->filter_seeded = false;
  return true;
}

void sensorReadResetFilters(SensorReadContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    ctx->filtered_raw[i] = 0.0f;
  }

  ctx->filter_seeded = false;
}

bool sensorReadReadRaw(SensorReadContext *ctx, SensorRawData *raw_frame) {
  if (ctx == nullptr || raw_frame == nullptr || !ctx->initialized) {
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    raw_frame->value[i] = ctx->ads.readADC_SingleEnded(i);
  }

  raw_frame->valid = true;
  raw_frame->timestamp_ms = millis();
  return true;
}

bool sensorReadProcess(SensorReadContext *ctx,
                       const SensorRawData *raw_frame,
                       const CalibrationData *calibration,
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

  float mean_delta = 0.0f;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    local_frame.raw[i] = raw_frame->value[i];

    const float raw_value = static_cast<float>(raw_frame->value[i]);
    float filtered = raw_value;
    if (ctx->filter_seeded) {
      filtered = (ctx->config.filter_alpha * raw_value) +
                 ((1.0f - ctx->config.filter_alpha) * ctx->filtered_raw[i]);
    }

    local_frame.filtered[i] = filtered;
    local_frame.delta[i] = filtered - calibration->baseline[i];
    mean_delta += local_frame.delta[i];
  }

  mean_delta /= static_cast<float>(SENSOR_COUNT);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    float delta = local_frame.delta[i];
    if (ctx->config.common_mode_rejection) {
      delta -= mean_delta;
    }

    // Use magnitude so the same controller works even if sensor polarity is reversed.
    float signal = fabsf(delta) * calibration->scale[i];
    if (signal < ctx->config.signal_floor) {
      signal = 0.0f;
    }

    local_frame.delta[i] = delta;
    local_frame.signal[i] = signal;
    local_frame.total_signal += signal;
    if (signal > local_frame.peak_signal) {
      local_frame.peak_signal = signal;
    }

    ctx->filtered_raw[i] = local_frame.filtered[i];
  }

  ctx->filter_seeded = true;
  *processed_frame = local_frame;
  return true;
}
