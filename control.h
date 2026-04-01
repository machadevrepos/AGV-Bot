#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include "bot_common.h"
#include "bot_state.h"
#include "motor_drive.h"
#include "sensor_read.h"

struct ControlConfig {
  float sensor_positions[SENSOR_COUNT];
  float confidence_total_ref;
  float confidence_peak_ref;
  float confidence_tracking_threshold;
  float confidence_lost_threshold;
  float position_filter_alpha;
  float center_enter_threshold;
  float center_exit_threshold;
  float rotate_threshold;
  uint16_t direction_hold_ms;
  uint16_t motion_hold_ms;
};

struct ControlEstimate {
  float position;
  float sensed_position;
  float error;
  float confidence;
  float total_signal;
  float peak_signal;
  bool line_present;
  bool line_strong;
  uint8_t active_mask;
  uint8_t sensed_mask;
};

struct ControlOutput {
  MotionCommand motion_command;
  float position;
  float error;
  float confidence;
  bool line_present;
  bool line_strong;
  uint8_t active_mask;
};

struct ControlContext {
  ControlConfig config;
  float last_valid_position;
  float filtered_position;
  uint32_t last_direction_change_ms;
  uint32_t last_motion_change_ms;
  bool filter_seeded;
  bool line_seen_once;
  int8_t last_direction;
  MotionPrimitive last_motion;
};

void controlInit(ControlContext *ctx, const ControlConfig *config);
void controlEstimateLine(const ControlContext *ctx, const SensorProcessedData *sensor_data, ControlEstimate *estimate);
void controlReset(ControlContext *ctx);
ControlOutput controlCompute(ControlContext *ctx, const ControlEstimate *estimate, BotState state, uint32_t now_ms);

#endif
