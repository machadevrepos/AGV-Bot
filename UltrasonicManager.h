#ifndef ULTRASONIC_MANAGER_H
#define ULTRASONIC_MANAGER_H

#include <stdint.h>

enum UltrasonicSensorId {
  ULTRASONIC_SENSOR_FRONT = 0,
  ULTRASONIC_SENSOR_BACK,
  ULTRASONIC_SENSOR_RIGHT,
  ULTRASONIC_SENSOR_LEFT,
  ULTRASONIC_SENSOR_COUNT
};

struct UltrasonicManagerConfig {
  uint8_t trigger_pin;
  uint8_t echo_pins[ULTRASONIC_SENSOR_COUNT];
  uint16_t measurement_interval_ms;
  uint32_t echo_timeout_us;
  float obstacle_threshold_cm;
  float min_valid_distance_cm;
  float max_valid_distance_cm;
};

struct UltrasonicReading {
  float distance_cm;
  uint32_t last_update_ms;
  bool valid;
};

struct UltrasonicEchoState {
  volatile uint32_t rise_time_us;
  volatile uint32_t pulse_width_us;
  volatile bool pulse_complete;
  volatile bool waiting_for_fall;
};

struct UltrasonicManagerContext;

struct UltrasonicInterruptBinding {
  UltrasonicManagerContext *owner;
  uint8_t sensor_index;
};

struct UltrasonicManagerContext {
  UltrasonicManagerConfig config;
  UltrasonicReading readings[ULTRASONIC_SENSOR_COUNT];
  UltrasonicEchoState echo_state[ULTRASONIC_SENSOR_COUNT];
  UltrasonicInterruptBinding bindings[ULTRASONIC_SENSOR_COUNT];
  volatile bool measurement_in_progress;
  uint8_t pending_mask;
  uint32_t measurement_start_us;
  uint32_t last_trigger_ms;
  bool has_triggered_once;
  bool initialized;
};

bool ultrasonicManagerInit(UltrasonicManagerContext *ctx, const UltrasonicManagerConfig *config);
void ultrasonicManagerUpdate(UltrasonicManagerContext *ctx, uint32_t now_ms);

bool ultrasonicManagerIsReadingValid(const UltrasonicManagerContext *ctx, UltrasonicSensorId sensor_id);
float ultrasonicManagerGetDistanceCm(const UltrasonicManagerContext *ctx, UltrasonicSensorId sensor_id);

float ultrasonicManagerGetFrontDistanceCm(const UltrasonicManagerContext *ctx);
float ultrasonicManagerGetBackDistanceCm(const UltrasonicManagerContext *ctx);
float ultrasonicManagerGetRightDistanceCm(const UltrasonicManagerContext *ctx);
float ultrasonicManagerGetLeftDistanceCm(const UltrasonicManagerContext *ctx);

bool ultrasonicManagerIsFrontObstacleDetected(const UltrasonicManagerContext *ctx);

#endif
