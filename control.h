#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include "bot_common.h"
#include "bot_state.h"
#include "sensor_read.h"

struct ControlConfig {
  float sensor_positions[SENSOR_COUNT];
  float confidence_total_ref;
  float confidence_peak_ref;
  float confidence_tracking_threshold;
  float confidence_edge_threshold;
  float confidence_lost_threshold;
  float error_deadband;
  float min_base_scale;
  float speed_reduction_gain;
  float kp;
  float kd;
  float max_turn_pwm;
  int16_t base_pwm_tracking;
  int16_t base_pwm_edge;
  int16_t base_pwm_recover;
  int16_t recover_turn_pwm;
};

struct ControlEstimate {
  float position;
  float error;
  float confidence;
  float total_signal;
  float peak_signal;
  bool line_present;
  bool line_strong;
};

struct ControlOutput {
  int16_t left_pwm;
  int16_t right_pwm;
  int16_t base_pwm;
  float position;
  float error;
  float derivative;
  float turn_command;
  float confidence;
  bool line_present;
  bool line_strong;
};

struct ControlContext {
  ControlConfig config;
  float last_error;
  float last_valid_position;
  float last_error_sign;
  uint32_t last_update_ms;
  bool derivative_seeded;
};

void controlInit(ControlContext *ctx, const ControlConfig *config);
void controlEstimateLine(const ControlContext *ctx, const SensorProcessedData *sensor_data, ControlEstimate *estimate);
void controlReset(ControlContext *ctx);
ControlOutput controlCompute(ControlContext *ctx, const ControlEstimate *estimate, BotState state, uint32_t now_ms);

#endif
