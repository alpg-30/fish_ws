#include "fish_control/motor_bus.hpp"
#include "fish_control/motor_bus_dynamixel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace fish_control {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kCurrentScale = 0.00269;

std::uint64_t steady_now_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

class MockMotorBus final : public MotorBus {
public:
  explicit MockMotorBus(Config config) : config_(std::move(config)) {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      position_[i] = config_.mapping_offset[i];
      goal_[i] = position_[i];
    }
  }

  BusResult open() override {
    opened_ = true;
    last_update_ = std::chrono::steady_clock::now();
    return {true, true, ""};
  }

  MotorState read_state() override {
    const auto started = std::chrono::steady_clock::now();
    MotorState state;
    state.host_steady_time_ns = steady_now_ns();
    if (!opened_) {
      state.communication_result.fill(-1);
      return state;
    }
    simulate();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      state.valid[i] = true;
      state.position_rad[i] = position_[i];
      state.position_raw[i] = joint_to_raw(position_[i], i, config_);
      state.velocity_rad_s[i] = velocity_[i];
      state.velocity_raw[i] = static_cast<std::int32_t>(std::llround(
        static_cast<double>(config_.direction[i]) * config_.external_ratio[i] * velocity_[i] *
        4096.0 / (2.0 * kPi)));
      const double simulated_current =
        0.05 * std::abs(velocity_[i]) + 0.1 * std::abs(goal_[i] - position_[i]);
      const long raw = std::clamp(
        std::lround(simulated_current / kCurrentScale), -32768L, 32767L);
      state.current_raw[i] = static_cast<std::int16_t>(raw);
      state.current_a[i] = current_raw_to_amps(state.current_raw[i]);
      state.input_voltage_v[i] = 12.0;
      state.temperature_c[i] = 25.0;
      state.hardware_error[i] = 0;
      state.hardware_error_valid[i] = true;
      state.hardware_error_age_s[i] = 0.0;
      state.torque_enabled[i] = torque_enabled_;
      state.torque_enabled_valid[i] = true;
      state.torque_state_age_s[i] = 0.0;
      state.communication_result[i] = 0;
    }
    state.acquisition_duration_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return state;
  }

  BusResult write_goal_positions(const Array4i& goal_position_raw) override {
    if (!opened_) {
      return {false, false, "mock bus is not open"};
    }
    simulate();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      goal_[i] = raw_to_joint(goal_position_raw[i], i, config_);
    }
    return {true, true, ""};
  }

  BusResult set_torque(bool enabled) override {
    if (!opened_) {
      return {false, false, "mock bus is not open"};
    }
    torque_enabled_ = enabled;
    return {true, true, ""};
  }

  void close() override {
    opened_ = false;
    torque_enabled_ = false;
  }

  std::string backend() const override {
    return "mock";
  }

private:
  void simulate() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::clamp(
      std::chrono::duration<double>(now - last_update_).count(), 0.0, 0.1);
    last_update_ = now;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      double desired_velocity = 0.0;
      if (torque_enabled_) {
        desired_velocity = std::clamp(
          10.0 * (goal_[i] - position_[i]),
          -config_.max_velocity_rad_s[i],
          config_.max_velocity_rad_s[i]);
      }
      const double acceleration = std::clamp(
        15.0 * (desired_velocity - velocity_[i]),
        -config_.max_acceleration_rad_s2[i],
        config_.max_acceleration_rad_s2[i]);
      velocity_[i] += acceleration * dt;
      position_[i] += velocity_[i] * dt;
    }
  }

  Config config_;
  bool opened_{false};
  bool torque_enabled_{false};
  Array4d position_{};
  Array4d velocity_{};
  Array4d goal_{};
  std::chrono::steady_clock::time_point last_update_{};
};
}  // namespace

std::unique_ptr<MotorBus> make_motor_bus(const Config& config) {
  if (config.backend == "mock") {
    return std::make_unique<MockMotorBus>(config);
  }
  if (config.backend == "dynamixel") {
    return make_dynamixel_motor_bus(config);
  }
  throw std::runtime_error("unsupported motor backend: " + config.backend);
}

std::int32_t joint_to_raw(double position_rad, std::size_t index, const Config& config) {
  const long long counts = std::llround(
    static_cast<double>(config.direction[index]) * config.external_ratio[index] * position_rad *
    4096.0 / (2.0 * kPi));
  return static_cast<std::int32_t>(config.zero_raw[index] + counts);
}

double raw_to_joint(std::int32_t raw, std::size_t index, const Config& config) {
  const double counts = static_cast<double>(raw) - static_cast<double>(config.zero_raw[index]);
  return counts * 2.0 * kPi /
         (static_cast<double>(config.direction[index]) * config.external_ratio[index] * 4096.0);
}

double current_raw_to_amps(std::int16_t signed_raw) {
  return static_cast<double>(signed_raw) * kCurrentScale;
}

double current_u16_to_amps(std::uint16_t raw) {
  const std::int32_t signed_raw = raw < 32768 ? raw : static_cast<std::int32_t>(raw) - 65536;
  return static_cast<double>(signed_raw) * kCurrentScale;
}

double velocity_raw_to_rad_s(
  std::int32_t raw, std::size_t index, const Config& config) {
  const double output_rad_per_s =
    static_cast<double>(raw) * 0.229 * 2.0 * kPi / 60.0;
  return output_rad_per_s /
         (static_cast<double>(config.direction[index]) * config.external_ratio[index]);
}

double voltage_raw_to_volts(std::uint16_t raw) {
  return static_cast<double>(raw) * 0.1;
}

double temperature_raw_to_celsius(std::uint8_t raw) {
  return static_cast<double>(raw);
}

}  // namespace fish_control
