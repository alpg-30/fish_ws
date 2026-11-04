#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fish_control {

constexpr std::size_t kMotorCount = 4;
using Array4d = std::array<double, kMotorCount>;
using Array4i = std::array<std::int32_t, kMotorCount>;
using Array4b = std::array<bool, kMotorCount>;

enum class ControllerState {
  BOOT,
  DISARMED,
  ARMING,
  HOLDING,
  RUNNING,
  BRAKING,
  FAULT,
  ESTOP_LATCHED,
};

const char* to_string(ControllerState state);

struct Config {
  std::string backend{"mock"};
  double control_hz{100.0};
  double telemetry_hz{50.0};
  std::vector<std::string> motor_names{"fin_0", "fin_1", "fin_2", "fin_3"};
  std::vector<std::int64_t> motor_ids{1, 2, 3, 4};

  std::string waveform{"fluent_asymmetric"};
  double amplitude_model{0.5};
  double frequency_hz{1.0};
  double tf_coeff{0.35};
  double body_coeff{0.5};

  std::vector<double> x_model{-0.2, -0.05, 0.05, 0.2};
  std::vector<double> z_model{0.2, -0.2, 0.2, -0.2};
  std::string model_unit{"model_unit"};

  std::string mapping_kind{"mock_linear"};
  std::vector<double> mapping_gain{0.5, 0.5, -0.5, -0.5};
  std::vector<double> mapping_offset{0.0, 0.0, 0.0, 0.0};

  std::vector<std::int64_t> zero_raw{2048, 2048, 2048, 2048};
  std::vector<std::int64_t> direction{1, 1, 1, 1};
  std::vector<double> external_ratio{1.0, 1.0, 1.0, 1.0};
  bool calibration_confirmed{false};

  std::vector<double> min_rad = std::vector<double>(4, -1.57);
  std::vector<double> max_rad = std::vector<double>(4, 1.57);
  std::vector<double> max_velocity_rad_s = std::vector<double>(4, 4.0);
  std::vector<double> max_acceleration_rad_s2 = std::vector<double>(4, 20.0);
  std::vector<double> current_trip_a = std::vector<double>(4, 2.0);
  double current_trip_duration_s{0.2};
  std::vector<double> temperature_trip_c = std::vector<double>(4, 80.0);
  std::vector<double> min_voltage_v = std::vector<double>(4, 9.0);
  std::vector<double> max_voltage_v = std::vector<double>(4, 14.0);

  double feedback_timeout_s{0.2};
  std::int64_t bus_watchdog_ms{500};
  std::string normal_stop_policy{"hold_position"};
  bool require_recording{true};

  std::string bus_device{""};
  std::int64_t bus_baudrate{0};
  double bus_protocol_version{2.0};
};

struct ValidationResult {
  bool valid{true};
  std::string error;
};

ValidationResult validate_config(const Config& config);
ValidationResult validate_config_update(
  const Config& current,
  const Config& candidate,
  ControllerState state,
  const std::vector<std::string>& changed_parameters);
bool mapping_is_valid(const Config& config);

}  // namespace fish_control
