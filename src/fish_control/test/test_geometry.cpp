#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "fish_control/fluent_waveform.hpp"
#include "fish_control/motor_bus.hpp"

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[FAIL] " << message << '\n';
    std::exit(1);
  }
}
void near(double actual, double expected) {
  check(std::abs(actual - expected) <= 1e-10, "geometry numeric mismatch");
}
}

int main() {
  fish_control::Config c;
  c.amplitude_model = 0.8;
  c.body_coeff = 0.5;
  c.x_model = {0.1375, 0.1375, 0.1375, 0.1375};
  c.z_model = {0.35, -0.35, 0.0, 0.1};
  c.mapping_gain = {2.0, 2.0, 2.0, 2.0};
  c.mapping_offset = {0.1, 0.1, 0.1, 0.1};
  const auto sample = fish_control::evaluate_model_waveform(kPi / 2.0, 3.0, c);
  near(sample.spatial_phase_rad[0], kPi / 4.0);
  near(sample.wave_value[0], -std::sqrt(0.5));
  check(sample.wave_value[0] < 0.0, "spatial term must be sin(psi - phi)");
  near(sample.model_displacement[0], -0.282842712474619);
  near(sample.model_displacement[1], -0.282842712474619);
  near(sample.model_displacement[2], 0.0);
  near(sample.model_displacement[3], 0.0);
  near(sample.ideal_joint_target_rad[0], 0.1 - 2.0 * 0.282842712474619);
  check(sample.mapping_valid, "mock mapping valid");

  fish_control::Config raw_config;
  raw_config.zero_raw = {2048, 2048, 2048, 2048};
  raw_config.direction = {-1, -1, -1, -1};
  raw_config.external_ratio = {2.0, 2.0, 2.0, 2.0};
  check(fish_control::joint_to_raw(kPi / 2.0, 0, raw_config) == 0,
        "raw conversion includes direction and external ratio exactly once");
  near(fish_control::raw_to_joint(0, 0, raw_config), kPi / 2.0);
  near(fish_control::current_raw_to_amps(static_cast<std::int16_t>(0xFFFF)), -0.00269);
  near(fish_control::current_u16_to_amps(0xFFFF), -0.00269);
  near(fish_control::velocity_raw_to_rad_s(1000, 0, raw_config), -11.9904119612);
  near(fish_control::voltage_raw_to_volts(120), 12.0);
  near(fish_control::temperature_raw_to_celsius(41), 41.0);

  auto not_calibrated = c;
  not_calibrated.mapping_kind = "not_calibrated";
  const fish_control::Array4d displacement{};
  check(!fish_control::map_model_to_joint(displacement, not_calibrated).valid,
        "unconfirmed mapping must be invalid");

  auto hardware = c;
  hardware.backend = "dynamixel";
  bool explicit_failure = false;
  bool backend_is_dynamixel = false;
  try {
    auto bus = fish_control::make_motor_bus(hardware);
    backend_is_dynamixel = bus->backend() == "dynamixel";
  } catch (const std::runtime_error& error) {
    explicit_failure = std::string(error.what()).find("DynamixelSDK") != std::string::npos;
  }
  check(explicit_failure || backend_is_dynamixel,
        "dynamixel backend must be real or fail explicitly, never fall back to mock");
  std::cout << "[PASS] geometry, mapping, raw/current conversion and backend tests\n";
  return 0;
}
