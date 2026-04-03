#include <Arduino.h>
#include <Wire.h>

#include "bot_state.h"
#include "calibration.h"
#include "control.h"
#include "i2c_bus.h"
#include "motor_drive.h"
#include "sensor_read.h"
#include "UltrasonicManager.h"

namespace AppConfig {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint8_t kAds1115Address = 0x48;
constexpr uint8_t kPca9685Address = 0x40;
constexpr int kI2cSdaPin = 17;
constexpr int kI2cSclPin = 18;
constexpr uint32_t kI2cClockHz = 50000;
constexpr uint8_t kI2cRetryCount = 2U;
constexpr uint16_t kI2cRetryDelayMs = 2U;
constexpr uint16_t kI2cRecoveryDelayMs = 2U;
constexpr uint16_t kI2cTransactionGapUs = 500U;
constexpr uint16_t kI2cFailureCooldownMs = 20U;
constexpr uint8_t kI2cMaxConsecutiveFailures = 2U;

constexpr uint16_t kControlLoopMs = 10;
constexpr uint16_t kUltrasonicMeasurementIntervalMs = 50;
constexpr uint32_t kUltrasonicEchoTimeoutUs = 25000U;
constexpr float kUltrasonicObstacleThresholdCm = 30.0f;
constexpr float kUltrasonicMinDistanceCm = 2.0f;
constexpr float kUltrasonicMaxDistanceCm = 300.0f;
constexpr uint16_t kCalibrationWarmupMs = 1200;
constexpr uint16_t kCalibrationSamples = 130;
constexpr uint16_t kCalibrationSampleDelayMs = 4;
constexpr uint32_t kDebugIntervalMs = 200;

constexpr float kFilterAlpha = 0.25f;
constexpr float kSignalFloor = 10.0f;
constexpr float kSignalActivateThreshold = 18.0f;
constexpr float kSignalDeactivateThreshold = 10.0f;
constexpr float kBaselineFollowAlpha = 0.0025f;
// Apply sensor correction once in the signal pipeline.
constexpr float kSensorScale[SENSOR_COUNT] = {1.000f, 0.480f, 0.495f};

constexpr int16_t kPwmMax = 4095;
constexpr int16_t MOTOR_PWM = 2500;

constexpr float kConfidenceTotalRef = 150.0f;
constexpr float kConfidencePeakRef = 90.0f;
constexpr float kConfidenceTrackingThreshold = 0.25f;
constexpr float kConfidenceLostThreshold = 0.04f;
constexpr float kPositionFilterAlpha = 0.950;
constexpr float kCenterEnterThreshold = 0.4;
constexpr float kCenterExitThreshold = 0.7;
constexpr float kRotateThreshold = 1.0f;
constexpr uint16_t kDirectionHoldMs = 10;
constexpr uint16_t kMotionHoldMs = 10;
constexpr uint8_t kLinePresentConfirmCount = 3;
constexpr uint8_t kLineLostConfirmCount = 5;

// Keep this array aligned with the physical left-to-right sensor order.
// To reverse the sensor order later, only swap these positions.
constexpr float kSensorPositions[SENSOR_COUNT] = {-2.0f, 0.0f, 2.0f};

constexpr bool FL_REVERSED = false;
constexpr bool FR_REVERSED = false;
constexpr bool RL_REVERSED = false;
constexpr bool RR_REVERSED = false;

constexpr uint8_t kUltrasonicTrigPin = 4;
constexpr uint8_t kUltrasonicFrontEchoPin = 20;
constexpr uint8_t kUltrasonicBackEchoPin = 21;
constexpr uint8_t kUltrasonicRightEchoPin = 45;
constexpr uint8_t kUltrasonicLeftEchoPin = 48;

}  // namespace AppConfig

struct CalibrationSamplerContext {
  SensorReadContext *sensor_ctx;
};

static SensorReadContext g_sensor_ctx;
static CalibrationData g_calibration;
static MotorDriveContext g_motor_ctx;
static BotStateMachine g_state_machine;
static ControlContext g_control_ctx;
static I2cBusContext g_i2c_bus;
static UltrasonicManagerContext g_ultrasonic_ctx;
static SensorReadConfig g_sensor_config;
static MotorDriveConfig g_motor_config;
static ControlConfig g_control_config;
static BotStateConfig g_state_config;
static UltrasonicManagerConfig g_ultrasonic_config;

static uint32_t g_last_loop_ms = 0U;
static uint32_t g_last_debug_ms = 0U;
static bool g_init_ok = false;
static bool g_calibration_done = false;
static bool g_ultrasonic_ok = false;

static SensorReadConfig makeSensorConfig() {
  SensorReadConfig config;
  config.i2c_address = AppConfig::kAds1115Address;
  config.gain = GAIN_ONE;
  config.filter_alpha = AppConfig::kFilterAlpha;
  config.signal_floor = AppConfig::kSignalFloor;
  config.signal_activate_threshold = AppConfig::kSignalActivateThreshold;
  config.signal_deactivate_threshold = AppConfig::kSignalDeactivateThreshold;
  config.baseline_follow_alpha = AppConfig::kBaselineFollowAlpha;
  return config;
}

static MotorDriveConfig makeMotorConfig() {
  MotorDriveConfig config;
  config.i2c_address = AppConfig::kPca9685Address;
  config.pwm_frequency_hz = 1000;
  config.pwm_max = AppConfig::kPwmMax;
  config.motor_pwm = AppConfig::MOTOR_PWM;
  config.front_left_reversed = AppConfig::FL_REVERSED;
  config.front_right_reversed = AppConfig::FR_REVERSED;
  config.rear_left_reversed = AppConfig::RL_REVERSED;
  config.rear_right_reversed = AppConfig::RR_REVERSED;
  config.front_right = {0, 1};
  config.front_left = {2, 3};
  config.rear_left = {4, 5};
  config.rear_right = {6, 7};
  return config;
}

static ControlConfig makeControlConfig() {
  ControlConfig config;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    config.sensor_positions[i] = AppConfig::kSensorPositions[i];
  }
  config.confidence_total_ref = AppConfig::kConfidenceTotalRef;
  config.confidence_peak_ref = AppConfig::kConfidencePeakRef;
  config.confidence_tracking_threshold = AppConfig::kConfidenceTrackingThreshold;
  config.confidence_lost_threshold = AppConfig::kConfidenceLostThreshold;
  config.position_filter_alpha = AppConfig::kPositionFilterAlpha;
  config.center_enter_threshold = AppConfig::kCenterEnterThreshold;
  config.center_exit_threshold = AppConfig::kCenterExitThreshold;
  config.rotate_threshold = AppConfig::kRotateThreshold;
  config.direction_hold_ms = AppConfig::kDirectionHoldMs;
  config.motion_hold_ms = AppConfig::kMotionHoldMs;
  return config;
}

static I2cBusConfig makeI2cBusConfig() {
  I2cBusConfig config;
  config.sda_pin = AppConfig::kI2cSdaPin;
  config.scl_pin = AppConfig::kI2cSclPin;
  config.clock_hz = AppConfig::kI2cClockHz;
  config.retry_count = AppConfig::kI2cRetryCount;
  config.retry_delay_ms = AppConfig::kI2cRetryDelayMs;
  config.recovery_delay_ms = AppConfig::kI2cRecoveryDelayMs;
  config.transaction_gap_us = AppConfig::kI2cTransactionGapUs;
  config.failure_cooldown_ms = AppConfig::kI2cFailureCooldownMs;
  config.max_consecutive_failures = AppConfig::kI2cMaxConsecutiveFailures;
  return config;
}

static UltrasonicManagerConfig makeUltrasonicConfig() {
  UltrasonicManagerConfig config = {};
  config.trigger_pin = AppConfig::kUltrasonicTrigPin;
  config.echo_pins[ULTRASONIC_SENSOR_FRONT] = AppConfig::kUltrasonicFrontEchoPin;
  config.echo_pins[ULTRASONIC_SENSOR_BACK] = AppConfig::kUltrasonicBackEchoPin;
  config.echo_pins[ULTRASONIC_SENSOR_RIGHT] = AppConfig::kUltrasonicRightEchoPin;
  config.echo_pins[ULTRASONIC_SENSOR_LEFT] = AppConfig::kUltrasonicLeftEchoPin;
  config.measurement_interval_ms = AppConfig::kUltrasonicMeasurementIntervalMs;
  config.echo_timeout_us = AppConfig::kUltrasonicEchoTimeoutUs;
  config.obstacle_threshold_cm = AppConfig::kUltrasonicObstacleThresholdCm;
  config.min_valid_distance_cm = AppConfig::kUltrasonicMinDistanceCm;
  config.max_valid_distance_cm = AppConfig::kUltrasonicMaxDistanceCm;
  return config;
}

static BotStateConfig makeStateConfig() {
  BotStateConfig config = {};
  config.line_present_confirm_count = AppConfig::kLinePresentConfirmCount;
  config.line_lost_confirm_count = AppConfig::kLineLostConfirmCount;
  return config;
}

static bool sampleCalibrationFrame(void *user_context, int16_t out_values[SENSOR_COUNT]) {
  CalibrationSamplerContext *sampler = static_cast<CalibrationSamplerContext *>(user_context);
  if (sampler == nullptr || sampler->sensor_ctx == nullptr || out_values == nullptr) {
    return false;
  }

  SensorRawData raw_frame;
  if (!sensorReadReadRaw(sampler->sensor_ctx, &raw_frame)) {
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    out_values[i] = raw_frame.value[i];
  }

  return true;
}

static void printUltrasonicDistanceLabel(const UltrasonicManagerContext *ultrasonic_ctx,
                                         UltrasonicSensorId sensor_id) {
  const float distance_cm = ultrasonicManagerGetDistanceCm(ultrasonic_ctx, sensor_id);
  if (distance_cm < 0.0f) {
    Serial.print("invalid");
    return;
  }

  Serial.print(distance_cm, 1);
  Serial.print("cm");
}

static void printRateLimitedDebug(const SensorProcessedData &sensor_data,
                                  const ControlEstimate &estimate,
                                  const ControlOutput &output,
                                  const MecanumWheelPwm &wheel_pwm,
                                  const UltrasonicManagerContext *ultrasonic_ctx,
                                  bool ultrasonic_ok,
                                  BotState state,
                                  uint32_t now_ms) {
  if ((now_ms - g_last_debug_ms) < AppConfig::kDebugIntervalMs) {
    return;
  }

  g_last_debug_ms = now_ms;

  Serial.print("state=");
  Serial.print(botStateName(state));
  Serial.print(" conf=");
  Serial.print(estimate.confidence, 3);
  Serial.print(" pos=");
  Serial.print(output.position, 3);
  Serial.print(" err=");
  Serial.print(output.error, 3);
  Serial.print(" mask=0x");
  Serial.print(output.active_mask, HEX);
  Serial.print(" motion=");
  Serial.print(motionPrimitiveName(output.motion_command.primitive));
  Serial.print(" dir=");
  if (output.motion_command.direction < 0) {
    Serial.print("L");
  } else if (output.motion_command.direction > 0) {
    Serial.print("R");
  } else {
    Serial.print("C");
  }
  Serial.print(" wheels=[");
  Serial.print(wheel_pwm.front_left);
  Serial.print(", ");
  Serial.print(wheel_pwm.front_right);
  Serial.print(", ");
  Serial.print(wheel_pwm.rear_left);
  Serial.print(", ");
  Serial.print(wheel_pwm.rear_right);
  Serial.print("]");
  Serial.print(" sig=[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(sensor_data.signal[i], 1);
    if (i + 1U < SENSOR_COUNT) {
      Serial.print(", ");
    }
  }
  Serial.print("] ctrlSig=[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(sensor_data.scaled_signal[i], 1);
    if (i + 1U < SENSOR_COUNT) {
      Serial.print(", ");
    }
  }
  Serial.println("]");

  if (!ultrasonic_ok || ultrasonic_ctx == nullptr) {
    return;
  }

  Serial.print("ultrasound front=");
  printUltrasonicDistanceLabel(ultrasonic_ctx, ULTRASONIC_SENSOR_FRONT);
  Serial.print(" blocked=");
  Serial.print(ultrasonicManagerIsFrontObstacleDetected(ultrasonic_ctx) ? "YES" : "NO");
  Serial.print(" back=");
  printUltrasonicDistanceLabel(ultrasonic_ctx, ULTRASONIC_SENSOR_BACK);
  Serial.print(" right=");
  printUltrasonicDistanceLabel(ultrasonic_ctx, ULTRASONIC_SENSOR_RIGHT);
  Serial.print(" left=");
  printUltrasonicDistanceLabel(ultrasonic_ctx, ULTRASONIC_SENSOR_LEFT);
  Serial.println();
}

static bool performStartupCalibration() {
  motorDriveStopAll(&g_motor_ctx);
  sensorReadResetFilters(&g_sensor_ctx);
  delay(AppConfig::kCalibrationWarmupMs);

  CalibrationSamplerContext sampler = {&g_sensor_ctx};
  const bool calibrated = calibrationRunBaseline(&g_calibration,
                                                 sampleCalibrationFrame,
                                                 &sampler,
                                                 AppConfig::kCalibrationSamples,
                                                 AppConfig::kCalibrationSampleDelayMs);

  sensorReadResetFilters(&g_sensor_ctx);
  if (calibrated) {
    calibrationSetScale(&g_calibration, AppConfig::kSensorScale);
    Serial.print("baseline=[");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      Serial.print(g_calibration.baseline[i], 1);
      if (i + 1U < SENSOR_COUNT) {
        Serial.print(", ");
      }
    }
    Serial.println("]");
    Serial.print("sensor_scale=[");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      Serial.print(g_calibration.scale[i], 3);
      if (i + 1U < SENSOR_COUNT) {
        Serial.print(", ");
      }
    }
    Serial.println("]");
  }
  return calibrated;
}

void setup() {
  Serial.begin(AppConfig::kSerialBaud);
  delay(1500);

  const I2cBusConfig i2c_bus_config = makeI2cBusConfig();
  const bool i2c_ok = i2cBusInit(&g_i2c_bus, &Wire, &i2c_bus_config);
  delay(200);

  Serial.print("ads_addr=0x");
  Serial.println(AppConfig::kAds1115Address, HEX);
  Serial.print("pca_addr=0x");
  Serial.println(AppConfig::kPca9685Address, HEX);
  Serial.print("i2c_pins=");
  Serial.print(AppConfig::kI2cSdaPin);
  Serial.print(",");
  Serial.println(AppConfig::kI2cSclPin);
  Serial.print("i2c_freq=");
  Serial.println(AppConfig::kI2cClockHz);
  Serial.print("motor_pwm=");
  Serial.println(AppConfig::MOTOR_PWM);
  Serial.print("warmup_ms=");
  Serial.println(AppConfig::kCalibrationWarmupMs);
  Serial.print("signal_hysteresis=");
  Serial.print(AppConfig::kSignalActivateThreshold, 1);
  Serial.print("/");
  Serial.println(AppConfig::kSignalDeactivateThreshold, 1);

  g_sensor_config = makeSensorConfig();
  g_motor_config = makeMotorConfig();
  g_control_config = makeControlConfig();
  g_state_config = makeStateConfig();
  g_ultrasonic_config = makeUltrasonicConfig();

  Serial.print("sensor_scale=[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(AppConfig::kSensorScale[i], 3);
    if (i + 1U < SENSOR_COUNT) {
      Serial.print(", ");
    }
  }
  Serial.println("]");

  botStateInit(&g_state_machine, millis());
  controlInit(&g_control_ctx, &g_control_config);
  calibrationReset(&g_calibration);

  const bool ads_ok = i2c_ok && sensorReadInit(&g_sensor_ctx, &g_sensor_config, &g_i2c_bus);
  const bool pca_ok = i2c_ok && motorDriveInit(&g_motor_ctx, &g_motor_config, &g_i2c_bus);
  g_ultrasonic_ok = ultrasonicManagerInit(&g_ultrasonic_ctx, &g_ultrasonic_config);
  g_init_ok = i2c_ok && ads_ok && pca_ok;

  if (!i2c_ok) {
    Serial.println("i2c_init_failed");
  }
  if (!ads_ok) {
    Serial.println("ads_init_failed");
  }
  if (!pca_ok) {
    Serial.println("pca_init_failed");
  }
  if (!g_ultrasonic_ok) {
    Serial.println("ultrasonic_init_failed");
  }

  if (!g_init_ok) {
    Serial.println("init_failed");
    botStateTransition(&g_state_machine, BOT_ERROR, millis());
    return;
  }

  motorDriveStopAll(&g_motor_ctx);
  botStateTransition(&g_state_machine, BOT_CALIBRATING, millis());

  g_calibration_done = performStartupCalibration();
  if (!g_calibration_done) {
    Serial.println("calibration_failed");
    motorDriveStopAll(&g_motor_ctx);
    botStateTransition(&g_state_machine, BOT_ERROR, millis());
    return;
  }

  controlReset(&g_control_ctx);
  botStateTransition(&g_state_machine, BOT_LINE_LOST, millis());
  Serial.println("tracking_ready");
}

void loop() {
  const uint32_t now_ms = millis();
  if ((now_ms - g_last_loop_ms) < AppConfig::kControlLoopMs) {
    return;
  }
  g_last_loop_ms = now_ms;

  if (g_ultrasonic_ok) {
    ultrasonicManagerUpdate(&g_ultrasonic_ctx, now_ms);
  }

  if (!g_init_ok || g_state_machine.current_state == BOT_ERROR) {
    motorDriveStopAll(&g_motor_ctx);
    return;
  }

  SensorRawData raw_frame;
  if (!sensorReadReadRaw(&g_sensor_ctx, &raw_frame)) {
    botStateTransition(&g_state_machine, BOT_ERROR, now_ms);
    motorDriveStopAll(&g_motor_ctx);
    return;
  }

  SensorProcessedData processed_frame;
  if (!sensorReadProcess(&g_sensor_ctx, &raw_frame, &g_calibration, &processed_frame)) {
    botStateTransition(&g_state_machine, BOT_ERROR, now_ms);
    motorDriveStopAll(&g_motor_ctx);
    return;
  }

  ControlEstimate estimate;
  controlEstimateLine(&g_control_ctx, &processed_frame, &estimate);

  BotStateInputs state_inputs;
  state_inputs.init_ok = g_init_ok;
  state_inputs.calibration_done = g_calibration_done;
  state_inputs.line_present = estimate.line_present;
  state_inputs.line_strong = estimate.line_strong;
  state_inputs.position = estimate.position;
  state_inputs.error = estimate.error;
  state_inputs.now_ms = now_ms;

  const BotState state = botStateUpdate(&g_state_machine, &g_state_config, &state_inputs);
  ControlOutput output = controlCompute(&g_control_ctx, &estimate, state, now_ms);

  // The ultrasonic module stays independent. Main loop only gates the outgoing
  // motor command so future back/right/left reactions can be added the same way.
  if (g_ultrasonic_ok && ultrasonicManagerIsFrontObstacleDetected(&g_ultrasonic_ctx)) {
    output.motion_command.primitive = MOTION_STOP;
    output.motion_command.direction = 0;
  }

  const MecanumWheelPwm wheel_pwm = motorDriveApplyCommand(&g_motor_ctx, &output.motion_command);

  printRateLimitedDebug(
      processed_frame, estimate, output, wheel_pwm, &g_ultrasonic_ctx, g_ultrasonic_ok, state, now_ms);
}









