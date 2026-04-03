#include "UltrasonicManager.h"

#include <Arduino.h>
#include <string.h>

namespace {

constexpr uint8_t kAllSensorsMask =
    (1U << ULTRASONIC_SENSOR_FRONT) |
    (1U << ULTRASONIC_SENSOR_BACK) |
    (1U << ULTRASONIC_SENSOR_RIGHT) |
    (1U << ULTRASONIC_SENSOR_LEFT);
constexpr float kDistanceCmPerUs = 0.01715f;
constexpr uint8_t kTriggerPulseUs = 10U;

bool isValidSensorId(UltrasonicSensorId sensor_id) {
  return sensor_id >= ULTRASONIC_SENSOR_FRONT && sensor_id < ULTRASONIC_SENSOR_COUNT;
}

bool isDistanceInRange(const UltrasonicManagerContext *ctx, float distance_cm) {
  if (ctx == nullptr) {
    return false;
  }

  return distance_cm >= ctx->config.min_valid_distance_cm &&
         distance_cm <= ctx->config.max_valid_distance_cm;
}

bool isConfigValid(const UltrasonicManagerConfig *config) {
  if (config == nullptr) {
    return false;
  }

  return config->measurement_interval_ms > 0U &&
         config->echo_timeout_us > 0U &&
         config->min_valid_distance_cm >= 0.0f &&
         config->max_valid_distance_cm >= config->min_valid_distance_cm;
}

void setReading(UltrasonicManagerContext *ctx, uint8_t sensor_index, float distance_cm, bool valid, uint32_t now_ms) {
  if (ctx == nullptr || sensor_index >= ULTRASONIC_SENSOR_COUNT) {
    return;
  }

  ctx->readings[sensor_index].distance_cm = valid ? distance_cm : -1.0f;
  ctx->readings[sensor_index].last_update_ms = now_ms;
  ctx->readings[sensor_index].valid = valid;
}

bool consumePulseWidthUs(UltrasonicManagerContext *ctx, uint8_t sensor_index, uint32_t *pulse_width_us) {
  if (ctx == nullptr || pulse_width_us == nullptr || sensor_index >= ULTRASONIC_SENSOR_COUNT) {
    return false;
  }

  bool has_pulse = false;
  noInterrupts();
  if (ctx->echo_state[sensor_index].pulse_complete) {
    *pulse_width_us = ctx->echo_state[sensor_index].pulse_width_us;
    ctx->echo_state[sensor_index].pulse_complete = false;
    has_pulse = true;
  }
  interrupts();
  return has_pulse;
}

void clearMeasurementState(UltrasonicManagerContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  noInterrupts();
  for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; ++i) {
    ctx->echo_state[i].rise_time_us = 0U;
    ctx->echo_state[i].pulse_width_us = 0U;
    ctx->echo_state[i].pulse_complete = false;
    ctx->echo_state[i].waiting_for_fall = false;
  }
  interrupts();
}

void finishMeasurement(UltrasonicManagerContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  ctx->measurement_in_progress = false;
  ctx->pending_mask = 0U;
}

void invalidatePendingReadings(UltrasonicManagerContext *ctx, uint32_t now_ms) {
  if (ctx == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; ++i) {
    const uint8_t sensor_mask = static_cast<uint8_t>(1U << i);
    if ((ctx->pending_mask & sensor_mask) != 0U) {
      setReading(ctx, i, -1.0f, false, now_ms);
    }
  }

  finishMeasurement(ctx);
}

void processCompletedReadings(UltrasonicManagerContext *ctx, uint32_t now_ms) {
  if (ctx == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; ++i) {
    const uint8_t sensor_mask = static_cast<uint8_t>(1U << i);
    if ((ctx->pending_mask & sensor_mask) == 0U) {
      continue;
    }

    uint32_t pulse_width_us = 0U;
    if (!consumePulseWidthUs(ctx, i, &pulse_width_us)) {
      continue;
    }

    const float distance_cm = static_cast<float>(pulse_width_us) * kDistanceCmPerUs;
    setReading(ctx, i, distance_cm, isDistanceInRange(ctx, distance_cm), now_ms);
    ctx->pending_mask &= static_cast<uint8_t>(~sensor_mask);
  }

  if (ctx->pending_mask == 0U) {
    finishMeasurement(ctx);
  }
}

void startMeasurement(UltrasonicManagerContext *ctx, uint32_t now_ms) {
  if (ctx == nullptr || !ctx->initialized) {
    return;
  }

  clearMeasurementState(ctx);
  ctx->pending_mask = kAllSensorsMask;
  ctx->measurement_start_us = micros();
  ctx->last_trigger_ms = now_ms;
  ctx->has_triggered_once = true;
  ctx->measurement_in_progress = true;

  digitalWrite(ctx->config.trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(ctx->config.trigger_pin, HIGH);
  delayMicroseconds(kTriggerPulseUs);
  digitalWrite(ctx->config.trigger_pin, LOW);
}

void IRAM_ATTR handleEchoInterrupt(void *arg) {
  UltrasonicInterruptBinding *binding = static_cast<UltrasonicInterruptBinding *>(arg);
  if (binding == nullptr || binding->owner == nullptr) {
    return;
  }

  UltrasonicManagerContext *ctx = binding->owner;
  if (!ctx->measurement_in_progress || binding->sensor_index >= ULTRASONIC_SENSOR_COUNT) {
    return;
  }

  const uint8_t echo_pin = ctx->config.echo_pins[binding->sensor_index];
  const uint32_t now_us = micros();
  if (digitalRead(echo_pin) == HIGH) {
    ctx->echo_state[binding->sensor_index].rise_time_us = now_us;
    ctx->echo_state[binding->sensor_index].waiting_for_fall = true;
    return;
  }

  if (!ctx->echo_state[binding->sensor_index].waiting_for_fall) {
    return;
  }

  ctx->echo_state[binding->sensor_index].pulse_width_us =
      now_us - ctx->echo_state[binding->sensor_index].rise_time_us;
  ctx->echo_state[binding->sensor_index].pulse_complete = true;
  ctx->echo_state[binding->sensor_index].waiting_for_fall = false;
}

}  // namespace

bool ultrasonicManagerInit(UltrasonicManagerContext *ctx, const UltrasonicManagerConfig *config) {
  if (ctx == nullptr || !isConfigValid(config)) {
    return false;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->config = *config;

  pinMode(ctx->config.trigger_pin, OUTPUT);
  digitalWrite(ctx->config.trigger_pin, LOW);

  for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; ++i) {
    pinMode(ctx->config.echo_pins[i], INPUT);
    setReading(ctx, i, -1.0f, false, 0U);
    ctx->bindings[i].owner = ctx;
    ctx->bindings[i].sensor_index = i;
    attachInterruptArg(digitalPinToInterrupt(ctx->config.echo_pins[i]),
                       handleEchoInterrupt,
                       &ctx->bindings[i],
                       CHANGE);
  }

  ctx->initialized = true;
  return true;
}

void ultrasonicManagerUpdate(UltrasonicManagerContext *ctx, uint32_t now_ms) {
  if (ctx == nullptr || !ctx->initialized) {
    return;
  }

  processCompletedReadings(ctx, now_ms);

  if (ctx->measurement_in_progress) {
    const uint32_t elapsed_us = micros() - ctx->measurement_start_us;
    if (elapsed_us >= ctx->config.echo_timeout_us) {
      invalidatePendingReadings(ctx, now_ms);
    }
    return;
  }

  const bool measurement_due =
      !ctx->has_triggered_once ||
      ((now_ms - ctx->last_trigger_ms) >= ctx->config.measurement_interval_ms);
  if (measurement_due) {
    startMeasurement(ctx, now_ms);
  }
}

bool ultrasonicManagerIsReadingValid(const UltrasonicManagerContext *ctx, UltrasonicSensorId sensor_id) {
  if (ctx == nullptr || !isValidSensorId(sensor_id)) {
    return false;
  }

  return ctx->readings[sensor_id].valid;
}

float ultrasonicManagerGetDistanceCm(const UltrasonicManagerContext *ctx, UltrasonicSensorId sensor_id) {
  if (ctx == nullptr || !isValidSensorId(sensor_id) || !ctx->readings[sensor_id].valid) {
    return -1.0f;
  }

  return ctx->readings[sensor_id].distance_cm;
}

float ultrasonicManagerGetFrontDistanceCm(const UltrasonicManagerContext *ctx) {
  return ultrasonicManagerGetDistanceCm(ctx, ULTRASONIC_SENSOR_FRONT);
}

float ultrasonicManagerGetBackDistanceCm(const UltrasonicManagerContext *ctx) {
  return ultrasonicManagerGetDistanceCm(ctx, ULTRASONIC_SENSOR_BACK);
}

float ultrasonicManagerGetRightDistanceCm(const UltrasonicManagerContext *ctx) {
  return ultrasonicManagerGetDistanceCm(ctx, ULTRASONIC_SENSOR_RIGHT);
}

float ultrasonicManagerGetLeftDistanceCm(const UltrasonicManagerContext *ctx) {
  return ultrasonicManagerGetDistanceCm(ctx, ULTRASONIC_SENSOR_LEFT);
}

bool ultrasonicManagerIsFrontObstacleDetected(const UltrasonicManagerContext *ctx) {
  const float front_distance_cm = ultrasonicManagerGetFrontDistanceCm(ctx);
  if (front_distance_cm < 0.0f || ctx == nullptr) {
    return false;
  }

  return front_distance_cm < ctx->config.obstacle_threshold_cm;
}
