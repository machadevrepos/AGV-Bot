#include <Arduino.h>
#include <Wire.h>

#include "bot_state.h"
#include "calibration.h"
#include "control.h"
#include "motor_drive.h"
#include "sensor_read.h"

namespace AppConfig {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint8_t kAds1115Address = 0x48;
constexpr uint8_t kPca9685Address = 0x40;
constexpr int kI2cSdaPin = 17;
constexpr int kI2cSclPin = 18;
constexpr uint8_t kOverallSpeedPercent = 160;

constexpr uint16_t kControlLoopMs = 10;
constexpr uint16_t kCalibrationSamples = 130;
constexpr uint16_t kCalibrationSampleDelayMs = 4;
constexpr uint32_t kDebugIntervalMs = 200;

constexpr float kFilterAlpha = 0.7f;
constexpr float kSignalFloor = 12.0f;

constexpr int16_t kPwmMax = 4095;
constexpr int16_t kMotorDeadzone = 120;
constexpr int16_t kBasePwmTracking = 1180;
constexpr int16_t kBasePwmEdge = 500;
constexpr int16_t kBasePwmRecover = 720;
constexpr int16_t kRecoverTurnPwm = 980;

constexpr float kKp = 1100.0f;
constexpr float kKd = 150.0f;
constexpr float kMaxTurnPwm = 3800.0f;
constexpr float kErrorDeadband = 0.008f;
constexpr float kMinBaseScale = 0.15f;
constexpr float kSpeedReductionGain = 0.96f;

constexpr float kConfidenceTotalRef = 240.0f;
constexpr float kConfidencePeakRef = 120.0f;
constexpr float kConfidenceTrackingThreshold = 0.55f;
constexpr float kConfidenceEdgeThreshold = 0.28f;
constexpr float kConfidenceLostThreshold = 0.14f;

constexpr uint32_t kEdgeToRecoverMs = 160;

// Keep this array aligned with the physical left-to-right sensor order.
// To reverse the sensor order later, only swap these positions.
constexpr float kSensorPositions[SENSOR_COUNT] = {-1.5f, -0.5f, 0.5f, 1.5f};

constexpr bool kInvertLeftMotor = false;
constexpr bool kInvertRightMotor = false;

}  // namespace AppConfig

struct CalibrationSamplerContext {
  SensorReadContext *sensor_ctx;
};

static SensorReadContext g_sensor_ctx;
static CalibrationData g_calibration;
static MotorDriveContext g_motor_ctx;
static BotStateMachine g_state_machine;
static ControlContext g_control_ctx;
static SensorReadConfig g_sensor_config;
static MotorDriveConfig g_motor_config;
static ControlConfig g_control_config;
static BotStateConfig g_state_config;

static uint32_t g_last_loop_ms = 0U;
static uint32_t g_last_debug_ms = 0U;
static bool g_init_ok = false;
static bool g_calibration_done = false;


static SensorReadConfig makeSensorConfig() {
  SensorReadConfig config;
  config.i2c_address = AppConfig::kAds1115Address;
  config.gain = GAIN_ONE;
  config.filter_alpha = AppConfig::kFilterAlpha;
  config.common_mode_rejection = true;
  config.signal_floor = AppConfig::kSignalFloor;
  return config;
}

static MotorDriveConfig makeMotorConfig() {
  MotorDriveConfig config;
  config.i2c_address = AppConfig::kPca9685Address;
  config.pwm_frequency_hz = 1000;
  config.pwm_max = AppConfig::kPwmMax;
  config.pwm_deadzone = AppConfig::kMotorDeadzone;
  config.invert_left = AppConfig::kInvertLeftMotor;
  config.invert_right = AppConfig::kInvertRightMotor;
  config.right_front = {0, 1};
  config.right_back = {6, 7};
  config.left_front = {2, 3};
  config.left_back = {4, 5};
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
  config.confidence_edge_threshold = AppConfig::kConfidenceEdgeThreshold;
  config.confidence_lost_threshold = AppConfig::kConfidenceLostThreshold;
  config.error_deadband = AppConfig::kErrorDeadband;
  config.min_base_scale = AppConfig::kMinBaseScale;
  config.speed_reduction_gain = AppConfig::kSpeedReductionGain;
  config.kp = AppConfig::kKp;
  config.kd = AppConfig::kKd;
  config.max_turn_pwm = AppConfig::kMaxTurnPwm;
  config.base_pwm_tracking = static_cast<int16_t>((static_cast<long>(AppConfig::kBasePwmTracking) *
      AppConfig::kOverallSpeedPercent) / 100L);
  config.base_pwm_edge = static_cast<int16_t>((static_cast<long>(AppConfig::kBasePwmEdge) *
      AppConfig::kOverallSpeedPercent) / 100L);
  config.base_pwm_recover = static_cast<int16_t>((static_cast<long>(AppConfig::kBasePwmRecover) *
      AppConfig::kOverallSpeedPercent) / 100L);
  config.recover_turn_pwm = AppConfig::kRecoverTurnPwm;
  return config;
}

static BotStateConfig makeStateConfig() {
  BotStateConfig config;
  config.edge_to_recover_ms = AppConfig::kEdgeToRecoverMs;
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

static void printRateLimitedDebug(const SensorProcessedData &sensor_data,
                                  const ControlEstimate &estimate,
                                  const ControlOutput &output,
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
  Serial.print(" turn=");
  Serial.print(output.turn_command, 1);
  Serial.print(" pwmL=");
  Serial.print(output.left_pwm);
  Serial.print(" pwmR=");
  Serial.print(output.right_pwm);
  Serial.print(" sig=[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(sensor_data.signal[i], 1);
    if (i + 1U < SENSOR_COUNT) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}

static bool performStartupCalibration() {
  motorDriveStopAll(&g_motor_ctx);
  sensorReadResetFilters(&g_sensor_ctx);

  CalibrationSamplerContext sampler = {&g_sensor_ctx};
  const bool calibrated = calibrationRunBaseline(&g_calibration,
                                                 sampleCalibrationFrame,
                                                 &sampler,
                                                 AppConfig::kCalibrationSamples,
                                                 AppConfig::kCalibrationSampleDelayMs);

  sensorReadResetFilters(&g_sensor_ctx);
  if (calibrated) {
    Serial.print("baseline=[");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      Serial.print(g_calibration.baseline[i], 1);
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
  delay(200);

  // Use the known wiring pins directly. On ESP32-S3, board-default I2C pins often
  // do not match the pins actually used on a custom robot controller.
  Wire.begin(AppConfig::kI2cSdaPin, AppConfig::kI2cSclPin);
  Wire.setClock(400000);

  Serial.print("ads_addr=0x");
  Serial.println(AppConfig::kAds1115Address, HEX);
  Serial.print("pca_addr=0x");
  Serial.println(AppConfig::kPca9685Address, HEX);
  Serial.print("i2c_pins=");
  Serial.print(AppConfig::kI2cSdaPin);
  Serial.print(",");
  Serial.println(AppConfig::kI2cSclPin);
  Serial.print("base_speed_percent=");
  Serial.println(AppConfig::kOverallSpeedPercent);

  g_sensor_config = makeSensorConfig();
  g_motor_config = makeMotorConfig();
  g_control_config = makeControlConfig();
  g_state_config = makeStateConfig();

  botStateInit(&g_state_machine, millis());
  controlInit(&g_control_ctx, &g_control_config);
  calibrationReset(&g_calibration);

  const bool ads_ok = sensorReadInit(&g_sensor_ctx, &g_sensor_config, &Wire);
  const bool pca_ok = motorDriveInit(&g_motor_ctx, &g_motor_config, &Wire);
  g_init_ok = ads_ok && pca_ok;

  if (!ads_ok) {
    Serial.println("ads_init_failed");
  }
  if (!pca_ok) {
    Serial.println("pca_init_failed");
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
  botStateTransition(&g_state_machine, BOT_TRACKING, millis());
  Serial.println("tracking_ready");
}

void loop() {
  const uint32_t now_ms = millis();
  if ((now_ms - g_last_loop_ms) < AppConfig::kControlLoopMs) {
    return;
  }
  g_last_loop_ms = now_ms;

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
  const ControlOutput output = controlCompute(&g_control_ctx, &estimate, state, now_ms);

  motorDriveSetMotors(&g_motor_ctx, output.left_pwm, output.right_pwm);
  printRateLimitedDebug(processed_frame, estimate, output, state, now_ms);
}









