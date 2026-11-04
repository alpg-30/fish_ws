#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "fish_control/config.hpp"

namespace fish_control {

struct BusResult {
  bool attempted{false};
  bool succeeded{false};
  std::string error;
};

struct MotorState {
  Array4b valid{};
  std::array<std::int16_t, kMotorCount> current_raw{};
  Array4d current_a{};
  Array4i position_raw{};
  Array4d position_rad{};
  Array4i velocity_raw{};
  Array4d velocity_rad_s{};
  Array4d input_voltage_v{};
  Array4d temperature_c{};
  std::array<std::uint8_t, kMotorCount> hardware_error{};
  Array4b hardware_error_valid{};
  Array4d hardware_error_age_s{};
  Array4b torque_enabled{};
  Array4b torque_enabled_valid{};
  Array4d torque_state_age_s{};
  Array4i communication_result{};
  double acquisition_duration_s{0.0};
  std::uint64_t host_steady_time_ns{0};

  MotorState() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    current_a.fill(nan);
    position_rad.fill(nan);
    velocity_rad_s.fill(nan);
    input_voltage_v.fill(nan);
    temperature_c.fill(nan);
    hardware_error_age_s.fill(nan);
    torque_state_age_s.fill(nan);
  }
};

class MotorBus {
public:
  virtual ~MotorBus() = default;
  virtual BusResult open() = 0;
  virtual MotorState read_state() = 0;
  virtual BusResult write_goal_positions(const Array4i& goal_position_raw) = 0;
  virtual BusResult set_torque(bool enabled) = 0;
  virtual void close() = 0;
  virtual std::string backend() const = 0;
};

std::unique_ptr<MotorBus> make_motor_bus(const Config& config);
std::int32_t joint_to_raw(double position_rad, std::size_t index, const Config& config);
double raw_to_joint(std::int32_t raw, std::size_t index, const Config& config);
double current_raw_to_amps(std::int16_t signed_raw);
double current_u16_to_amps(std::uint16_t raw);

}  // namespace fish_control
