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

int8_t directionFromPosition(float value) {
  if (value > 0.5) {
    return 1;
  }
  if (value < -0.5) {
    return -1;
  }
  return 0;
}

int8_t fallbackDirection(const ControlContext *ctx) {
  if (ctx == nullptr) {
    return 0;
  }

  if (ctx->last_direction < 0) {
    return -1;
  }
  if (ctx->last_direction > 0) {
    return 1;
  }

  if (ctx->last_valid_position < 0.0f) {
    return -1;
  }
  if (ctx->last_valid_position > 0.0f) {
    return 1;
  }

  return 0;
}

constexpr uint8_t kLeftSensorMask = 0x01U;
constexpr uint8_t kCenterSensorMask = 0x02U;
constexpr uint8_t kRightSensorMask = 0x04U;

bool isLeftActive(uint8_t active_mask) {
  return (active_mask & kLeftSensorMask) != 0U;
}

bool isCenterActive(uint8_t active_mask) {
  return (active_mask & kCenterSensorMask) != 0U;
}

bool isRightActive(uint8_t active_mask) {
  return (active_mask & kRightSensorMask) != 0U;
}

bool hasLeftBias(uint8_t active_mask) {
  return isLeftActive(active_mask) && !isRightActive(active_mask);
}

bool hasRightBias(uint8_t active_mask) {
  return isRightActive(active_mask) && !isLeftActive(active_mask);
}

int8_t chooseDirection(const ControlContext *ctx, const ControlEstimate *estimate) {
  const int8_t position_direction = directionFromPosition(estimate->position);
  if (position_direction != 0) {
    return position_direction;
  }

  if (hasLeftBias(estimate->active_mask)) {
    return -1;
  }
  if (hasRightBias(estimate->active_mask)) {
    return 1;
  }

  return fallbackDirection(ctx);
}

}  // namespace

void controlInit(ControlContext *ctx, const ControlConfig *config) {
  if (ctx == nullptr || config == nullptr) {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->config = *config;
  ctx->last_valid_position = 0.0f;
  ctx->filtered_position = 0.0f;
  ctx->filter_seeded = false;
  ctx->line_seen_once = false;
  ctx->last_direction = 0;
}

void controlReset(ControlContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  ctx->last_valid_position = 0.0f;
  ctx->filtered_position = 0.0f;
  ctx->filter_seeded = false;
  ctx->line_seen_once = false;
  ctx->last_direction = 0;
}

void controlEstimateLine(const ControlContext *ctx, const SensorProcessedData *sensor_data, ControlEstimate *estimate) {
  if (ctx == nullptr || sensor_data == nullptr || estimate == nullptr) {
    return;
  }

  ControlEstimate local_estimate;
  memset(&local_estimate, 0, sizeof(local_estimate));

  if (sensor_data->scaled_total_signal > 0.0f) {
    float weighted_sum = 0.0f;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      weighted_sum += sensor_data->scaled_signal[i] * ctx->config.sensor_positions[i];
    }
    local_estimate.position = weighted_sum / sensor_data->scaled_total_signal;
  }

  const float total_component =
      clampFloat(sensor_data->scaled_total_signal / ctx->config.confidence_total_ref, 0.0f, 1.0f);
  const float peak_component =
      clampFloat(sensor_data->scaled_peak_signal / ctx->config.confidence_peak_ref, 0.0f, 1.0f);
  local_estimate.confidence = (0.60f * total_component) + (0.40f * peak_component);

  local_estimate.line_present = (local_estimate.confidence >= ctx->config.confidence_lost_threshold);
  local_estimate.line_strong = (local_estimate.confidence >= ctx->config.confidence_tracking_threshold);
  local_estimate.error = local_estimate.position;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (sensor_data->scaled_signal[i] > 0.0f) {
      local_estimate.active_mask |= static_cast<uint8_t>(1U << i);
    }
  }

  *estimate = local_estimate;
}

ControlOutput controlCompute(ControlContext *ctx, const ControlEstimate *estimate, BotState state) {
  ControlOutput output;
  memset(&output, 0, sizeof(output));

  if (ctx == nullptr || estimate == nullptr) {
    return output;
  }

  output.position = estimate->position;
  output.confidence = estimate->confidence;
  output.line_present = estimate->line_present;
  output.line_strong = estimate->line_strong;
  output.active_mask = estimate->active_mask;
  output.motion_command.primitive = MOTION_STOP;
  output.motion_command.direction = 0;

  if (state == BOT_IDLE || state == BOT_CALIBRATING || state == BOT_ERROR) {
    return output;
  }

  if (estimate->line_present) {
    ctx->line_seen_once = true;

    if (!ctx->filter_seeded) {
      ctx->filtered_position = estimate->position;
      ctx->filter_seeded = true;
    } else {
      const float alpha = clampFloat(ctx->config.position_filter_alpha, 0.0f, 1.0f);
      ctx->filtered_position =
          (alpha * estimate->position) + ((1.0f - alpha) * ctx->filtered_position);
    }

    ctx->last_valid_position = ctx->filtered_position;

    ControlEstimate smoothed_estimate = *estimate;
    smoothed_estimate.position = ctx->filtered_position;
    smoothed_estimate.error = ctx->filtered_position;

    const int8_t seen_direction = chooseDirection(ctx, &smoothed_estimate);
    if (seen_direction != 0) {
      ctx->last_direction = seen_direction;
    }
  }

  if (!ctx->line_seen_once) {
    return output;
  }

  if (state == BOT_ROTATE || !estimate->line_present) {
    const int8_t search_direction = chooseDirection(ctx, estimate);
    output.error = ctx->last_valid_position;
    output.motion_command.direction = search_direction;
    output.motion_command.primitive = (search_direction == 0) ? MOTION_STOP : MOTION_ROTATE;
    return output;
  }

  const float motion_position = ctx->filtered_position;
  const float abs_position = fabsf(motion_position);
  output.position = motion_position;
  output.error = motion_position;

  const bool center_active = isCenterActive(estimate->active_mask);
  if (center_active && abs_position <= ctx->config.tracking_position_threshold) {
    output.motion_command.primitive = MOTION_TRACKING;
    output.motion_command.direction = 0;
  } else {
    ControlEstimate smoothed_estimate = *estimate;
    smoothed_estimate.position = motion_position;
    smoothed_estimate.error = motion_position;
    output.motion_command.primitive = MOTION_ROTATE;
    output.motion_command.direction = chooseDirection(ctx, &smoothed_estimate);
  }

  return output;
}
