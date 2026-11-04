#include "fish_control/config.hpp"

#include <cmath>
#include <limits>
#include <set>

namespace fish_control {
namespace {

ValidationResult reject(const std::string& message) {
  return {false, message};
}

bool finite(double value) {
  return std::isfinite(value);
}

bool finite_vector(const std::vector<double>& values) {
  if (values.size() != kMotorCount) {
    return false;
  }
  for (double value : values) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

bool positive_vector(const std::vector<double>& values) {
  if (!finite_vector(values)) {
    return false;
  }
  for (double value : values) {
    if (value <= 0.0) {
      return false;
    }
  }
  return true;
}

bool same_structure(const Config& a, const Config& b) {
  return a.backend == b.backend && a.control_hz == b.control_hz &&
         a.telemetry_hz == b.telemetry_hz && a.motor_names == b.motor_names &&
         a.motor_ids == b.motor_ids && a.waveform == b.waveform &&
         a.x_model == b.x_model && a.z_model == b.z_model &&
         a.model_unit == b.model_unit && a.mapping_kind == b.mapping_kind &&
         a.mapping_gain == b.mapping_gain && a.mapping_offset == b.mapping_offset &&
         a.zero_raw == b.zero_raw && a.direction == b.direction &&
         a.external_ratio == b.external_ratio &&
         a.calibration_confirmed == b.calibration_confirmed && a.min_rad == b.min_rad &&
         a.max_rad == b.max_rad && a.max_velocity_rad_s == b.max_velocity_rad_s &&
         a.max_acceleration_rad_s2 == b.max_acceleration_rad_s2 &&
         a.current_trip_a == b.current_trip_a &&
         a.current_trip_duration_s == b.current_trip_duration_s &&
         a.temperature_trip_c == b.temperature_trip_c &&
         a.min_voltage_v == b.min_voltage_v && a.max_voltage_v == b.max_voltage_v &&
         a.feedback_timeout_s == b.feedback_timeout_s &&
         a.bus_watchdog_ms == b.bus_watchdog_ms &&
         a.normal_stop_policy == b.normal_stop_policy &&
         a.require_recording == b.require_recording && a.bus_device == b.bus_device &&
         a.bus_baudrate == b.bus_baudrate &&
         a.bus_protocol_version == b.bus_protocol_version;
}

bool runtime_gait_name(const std::string& name) {
  return name == "gait.amplitude_model" || name == "gait.frequency_hz" ||
         name == "gait.tf_coeff" || name == "gait.body_coeff";
}

}  // namespace

const char* to_string(ControllerState state) {
  switch (state) {
    case ControllerState::BOOT:
      return "BOOT";
    case ControllerState::DISARMED:
      return "DISARMED";
    case ControllerState::ARMING:
      return "ARMING";
    case ControllerState::HOLDING:
      return "HOLDING";
    case ControllerState::RUNNING:
      return "RUNNING";
    case ControllerState::BRAKING:
      return "BRAKING";
    case ControllerState::FAULT:
      return "FAULT";
    case ControllerState::ESTOP_LATCHED:
      return "ESTOP_LATCHED";
  }
  return "UNKNOWN";
}

ValidationResult validate_config(const Config& c) {
  if (c.backend != "mock" && c.backend != "dynamixel") {
    return reject("backend must be mock or dynamixel");
  }
  if (!finite(c.control_hz) || c.control_hz <= 0.0 || !finite(c.telemetry_hz) ||
      c.telemetry_hz <= 0.0) {
    return reject("timing frequencies must be finite and positive");
  }
  if (c.motor_names.size() != kMotorCount || c.motor_ids.size() != kMotorCount) {
    return reject("motors.names and motors.ids must have length 4");
  }
  std::set<std::string> names;
  std::set<std::int64_t> ids;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (c.motor_names[i].empty() || !names.insert(c.motor_names[i]).second) {
      return reject("motor names must be nonempty and unique");
    }
    if (c.motor_ids[i] < 0 || c.motor_ids[i] > 255 || !ids.insert(c.motor_ids[i]).second) {
      return reject("motor ids must be unique uint8 values");
    }
  }
  if (c.waveform != "fluent_asymmetric" &&
      c.waveform != "fluent_asymmetric_delayed") {
    return reject("gait.waveform must be fluent_asymmetric or fluent_asymmetric_delayed");
  }
  if (!finite(c.amplitude_model) || c.amplitude_model < 0.0 ||
      !finite(c.frequency_hz) || c.frequency_hz < 0.0 || !finite(c.tf_coeff) ||
      c.tf_coeff <= 0.0 || c.tf_coeff >= 1.0 || !finite(c.body_coeff)) {
    return reject("invalid gait scalar parameter");
  }
  if (!finite_vector(c.x_model) || !finite_vector(c.z_model) || c.model_unit.empty()) {
    return reject("geometry arrays must have four finite values and model_unit must be set");
  }
  if (c.mapping_kind != "mock_linear" || !finite_vector(c.mapping_gain) ||
      !finite_vector(c.mapping_offset)) {
    return reject("only complete mock_linear mapping is supported");
  }
  if (c.zero_raw.size() != kMotorCount || c.direction.size() != kMotorCount ||
      !positive_vector(c.external_ratio)) {
    return reject("calibration arrays must have length 4 and positive finite ratios");
  }
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (c.zero_raw[i] < std::numeric_limits<std::int32_t>::min() ||
        c.zero_raw[i] > std::numeric_limits<std::int32_t>::max() ||
        (c.direction[i] != -1 && c.direction[i] != 1)) {
      return reject("invalid zero_raw or direction calibration");
    }
  }
  if (!finite_vector(c.min_rad) || !finite_vector(c.max_rad) ||
      !positive_vector(c.max_velocity_rad_s) ||
      !positive_vector(c.max_acceleration_rad_s2) || !positive_vector(c.current_trip_a) ||
      !finite_vector(c.temperature_trip_c) || !finite_vector(c.min_voltage_v) ||
      !finite_vector(c.max_voltage_v)) {
    return reject("limit arrays must have length 4 and valid finite values");
  }
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (c.min_rad[i] >= c.max_rad[i] || c.min_voltage_v[i] >= c.max_voltage_v[i]) {
      return reject("limit lower bounds must be below upper bounds");
    }
  }
  if (!finite(c.current_trip_duration_s) || c.current_trip_duration_s <= 0.0 ||
      !finite(c.feedback_timeout_s) || c.feedback_timeout_s <= 0.0 ||
      c.bus_watchdog_ms <= 0) {
    return reject("safety durations and watchdog must be positive");
  }
  if (c.normal_stop_policy != "hold_position" || !c.require_recording) {
    return reject("normal_stop_policy must be hold_position and require_recording must be true");
  }
  if (c.bus_baudrate < 0 || !finite(c.bus_protocol_version) ||
      c.bus_protocol_version != 2.0) {
    return reject("bus baudrate must be nonnegative and protocol_version must be 2.0");
  }
  if (c.backend == "dynamixel" && (c.bus_device.empty() || c.bus_baudrate <= 0)) {
    return reject("dynamixel requires a real bus device and positive baudrate");
  }
  return {};
}

ValidationResult validate_config_update(
  const Config& current,
  const Config& candidate,
  ControllerState state,
  const std::vector<std::string>& changed_parameters) {
  for (const auto& name : changed_parameters) {
    if (!runtime_gait_name(name)) {
      return reject(name + " is read-only after startup");
    }
  }
  if (!same_structure(current, candidate)) {
    return reject("structural parameters are read-only after startup");
  }
  const bool gait_changed = current.amplitude_model != candidate.amplitude_model ||
                            current.frequency_hz != candidate.frequency_hz ||
                            current.tf_coeff != candidate.tf_coeff ||
                            current.body_coeff != candidate.body_coeff;
  if (gait_changed && state != ControllerState::DISARMED &&
      state != ControllerState::HOLDING) {
    return reject("gait parameters can only change in DISARMED or HOLDING");
  }
  return validate_config(candidate);
}

bool mapping_is_valid(const Config& config) {
  return config.backend == "mock" && config.mapping_kind == "mock_linear" &&
         finite_vector(config.mapping_gain) && finite_vector(config.mapping_offset);
}

}  // namespace fish_control
