#include "control.h"

#include <math.h>
#include <string.h>

namespace {

float clampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

float signOrStored(float value, float stored_sign) {
  if (value > 0.001f) {
    return 1.0f;
  }
  if (value < -0.001f) {
    return -1.0f;
  }
  return (stored_sign != 0.0f) ? stored_sign : 1.0f;
}

int16_t clampPwm(int32_t value) {
  if (value > 32767L) {
    return 32767;
  }
  if (value < -32768L) {
    return -32768;
  }
  return static_cast<int16_t>(value);
}

}  // namespace

void controlInit(ControlContext *ctx, const ControlConfig *config) {
  if (ctx == nullptr || config == nullptr) {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->config = *config;
  ctx->last_valid_position = 0.0f;
  ctx->last_error_sign = 1.0f;
}

void controlReset(ControlContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  ctx->last_error = 0.0f;
  ctx->last_valid_position = 0.0f;
  ctx->last_error_sign = 1.0f;
  ctx->last_update_ms = 0U;
  ctx->derivative_seeded = false;
}

void controlEstimateLine(const ControlContext *ctx, const SensorProcessedData *sensor_data, ControlEstimate *estimate) {
  if (ctx == nullptr || sensor_data == nullptr || estimate == nullptr) {
    return;
  }

  ControlEstimate local_estimate;
  memset(&local_estimate, 0, sizeof(local_estimate));
  local_estimate.total_signal = sensor_data->total_signal;
  local_estimate.peak_signal = sensor_data->peak_signal;

  // Weighted centroid uses all sensor magnitudes together, so one active sensor or
  // two partially active adjacent sensors both map to a smooth continuous position.
  if (sensor_data->total_signal > 0.0f) {
    float weighted_sum = 0.0f;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      weighted_sum += sensor_data->signal[i] * ctx->config.sensor_positions[i];
    }
    local_estimate.position = weighted_sum / sensor_data->total_signal;
  }

  const float total_component =
      clampFloat(sensor_data->total_signal / ctx->config.confidence_total_ref, 0.0f, 1.0f);
  const float peak_component =
      clampFloat(sensor_data->peak_signal / ctx->config.confidence_peak_ref, 0.0f, 1.0f);
  local_estimate.confidence = (0.60f * total_component) + (0.40f * peak_component);

  local_estimate.line_present = (local_estimate.confidence >= ctx->config.confidence_lost_threshold);
  local_estimate.line_strong = (local_estimate.confidence >= ctx->config.confidence_tracking_threshold);
  local_estimate.error = local_estimate.position;

  *estimate = local_estimate;
}

ControlOutput controlCompute(ControlContext *ctx, const ControlEstimate *estimate, BotState state, uint32_t now_ms) {
  ControlOutput output;
  memset(&output, 0, sizeof(output));

  if (ctx == nullptr || estimate == nullptr) {
    return output;
  }

  output.position = estimate->position;
  output.confidence = estimate->confidence;
  output.line_present = estimate->line_present;
  output.line_strong = estimate->line_strong;
  const bool has_position_hint = (estimate->confidence >= ctx->config.confidence_edge_threshold);

  // If the tape is fully lost, do not keep driving blindly. Hold position until
  // the line is seen again.
  if (!estimate->line_present) {
    ctx->last_update_ms = now_ms;
    ctx->last_error = 0.0f;
    ctx->derivative_seeded = false;
    return output;
  }

  if (estimate->line_strong) {
    ctx->last_valid_position = estimate->position;
    ctx->last_error_sign = signOrStored(estimate->position, ctx->last_error_sign);
  }

  float effective_error = ctx->last_valid_position;
  int16_t base_pwm = 0;

  switch (state) {
    case BOT_TRACKING:
      effective_error = has_position_hint ? estimate->position : ctx->last_valid_position;
      base_pwm = ctx->config.base_pwm_tracking;
      break;

    case BOT_EDGE:
      effective_error = has_position_hint
                            ? ((0.60f * ctx->last_valid_position) + (0.40f * estimate->position))
                            : ctx->last_valid_position;
      base_pwm = ctx->config.base_pwm_edge;
      break;

    case BOT_RECOVER:
      effective_error = 1.35f * signOrStored(ctx->last_valid_position, ctx->last_error_sign);
      base_pwm = ctx->config.base_pwm_recover;
      break;

    case BOT_IDLE:
    case BOT_CALIBRATING:
    case BOT_ERROR:
    default:
      effective_error = 0.0f;
      base_pwm = 0;
      break;
  }

  if (fabsf(effective_error) < ctx->config.error_deadband) {
    effective_error = 0.0f;
  }

  float derivative = 0.0f;
  if (ctx->derivative_seeded) {
    const uint32_t dt_ms = now_ms - ctx->last_update_ms;
    if (dt_ms > 0U) {
      derivative = (effective_error - ctx->last_error) / (static_cast<float>(dt_ms) * 0.001f);
    }
  }

  ctx->last_update_ms = now_ms;
  ctx->last_error = effective_error;
  ctx->derivative_seeded = true;

  float turn = (ctx->config.kp * effective_error) + (ctx->config.kd * derivative);
  turn = clampFloat(turn, -ctx->config.max_turn_pwm, ctx->config.max_turn_pwm);

  if (state == BOT_RECOVER) {
    turn = static_cast<float>(ctx->config.recover_turn_pwm) * signOrStored(effective_error, ctx->last_error_sign);
  }

  float base_scale = 1.0f - (ctx->config.speed_reduction_gain * clampFloat(fabsf(effective_error) / 1.5f, 0.0f, 1.0f));
  base_scale = clampFloat(base_scale, ctx->config.min_base_scale, 1.0f);
  const int32_t scaled_base = static_cast<int32_t>(static_cast<float>(base_pwm) * base_scale);

  output.base_pwm = static_cast<int16_t>(scaled_base);
  output.error = effective_error;
  output.derivative = derivative;
  output.turn_command = turn;
  output.left_pwm = clampPwm(static_cast<int32_t>(scaled_base) - static_cast<int32_t>(turn));
  output.right_pwm = clampPwm(static_cast<int32_t>(scaled_base) + static_cast<int32_t>(turn));


  if (state == BOT_IDLE || state == BOT_CALIBRATING || state == BOT_ERROR) {
    output.left_pwm = 0;
    output.right_pwm = 0;
    output.base_pwm = 0;
    output.turn_command = 0.0f;
    output.error = 0.0f;
    output.derivative = 0.0f;
  }

  return output;
}
