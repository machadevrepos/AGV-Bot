#ifndef SENSOR_READ_H
#define SENSOR_READ_H

#include <stdint.h>

#include <Adafruit_ADS1X15.h>

#include "bot_common.h"
#include "calibration.h"
#include "i2c_bus.h"

struct SensorRawData {
  int16_t value[SENSOR_COUNT];
  bool valid;
  uint32_t timestamp_ms;
};

struct SensorProcessedData {
  int16_t raw[SENSOR_COUNT];
  float filtered[SENSOR_COUNT];
  float delta[SENSOR_COUNT];
  float signal[SENSOR_COUNT];
  float scaled_signal[SENSOR_COUNT];
  float total_signal;
  float peak_signal;
  float scaled_total_signal;
  float scaled_peak_signal;
  bool valid;
  uint32_t timestamp_ms;
};

struct SensorReadConfig {
  uint8_t i2c_address;
  adsGain_t gain;
  float filter_alpha;
  float signal_floor;
  float signal_activate_threshold;
  float signal_deactivate_threshold;
  float baseline_follow_alpha;
};

struct SensorReadContext {
  I2cBusContext *bus;
  SensorReadConfig config;
  float filtered_raw[SENSOR_COUNT];
  bool sensor_active[SENSOR_COUNT];
  bool initialized;
  bool filter_seeded;
};

bool sensorReadInit(SensorReadContext *ctx, const SensorReadConfig *config, I2cBusContext *bus);
void sensorReadResetFilters(SensorReadContext *ctx);
bool sensorReadReadRaw(SensorReadContext *ctx, SensorRawData *raw_frame);
bool sensorReadProcess(SensorReadContext *ctx,
                       const SensorRawData *raw_frame,
                       CalibrationData *calibration,
                       SensorProcessedData *processed_frame);

#endif
