#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#include "bot_common.h"

struct CalibrationData {
  float baseline[SENSOR_COUNT];
  float scale[SENSOR_COUNT];
  bool valid;
  uint16_t sample_count;
};

typedef bool (*CalibrationSampleFn)(void *user_context, int16_t out_values[SENSOR_COUNT]);

void calibrationReset(CalibrationData *data);
bool calibrationRunBaseline(CalibrationData *data,
                            CalibrationSampleFn sample_fn,
                            void *user_context,
                            uint16_t sample_count,
                            uint16_t sample_delay_ms);
void calibrationSetScale(CalibrationData *data, const float scale[SENSOR_COUNT]);

#endif
