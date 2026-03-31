#include "calibration.h"

#include <Arduino.h>
#include <string.h>

void calibrationReset(CalibrationData *data) {
  if (data == nullptr) {
    return;
  }

  memset(data, 0, sizeof(*data));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    data->scale[i] = 1.0f;
  }
}

bool calibrationRunBaseline(CalibrationData *data,
                            CalibrationSampleFn sample_fn,
                            void *user_context,
                            uint16_t sample_count,
                            uint16_t sample_delay_ms) {
  if (data == nullptr || sample_fn == nullptr || sample_count == 0U) {
    return false;
  }

  float sums[SENSOR_COUNT] = {0.0f, 0.0f, 0.0f};
  int16_t sample[SENSOR_COUNT] = {0, 0, 0};

  calibrationReset(data);

  for (uint16_t n = 0; n < sample_count; ++n) {
    if (!sample_fn(user_context, sample)) {
      calibrationReset(data);
      return false;
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      sums[i] += static_cast<float>(sample[i]);
    }

    if (sample_delay_ms > 0U) {
      delay(sample_delay_ms);
    }
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    data->baseline[i] = sums[i] / static_cast<float>(sample_count);
    data->scale[i] = 1.0f;
  }

  data->sample_count = sample_count;
  data->valid = true;
  return true;
}

void calibrationSetScale(CalibrationData *data, const float scale[SENSOR_COUNT]) {
  if (data == nullptr || scale == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    data->scale[i] = (scale[i] > 0.0f) ? scale[i] : 1.0f;
  }
}

const float *calibrationGetBaseline(const CalibrationData *data) {
  return (data != nullptr) ? data->baseline : nullptr;
}

const float *calibrationGetScale(const CalibrationData *data) {
  return (data != nullptr) ? data->scale : nullptr;
}
