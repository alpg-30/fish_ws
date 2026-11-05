#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "fish_control/config.hpp"

namespace {
void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[FAIL] " << message << '\n';
    std::exit(1);
  }
}
using fish_control::Config;
using fish_control::ControllerState;

void rejected(const Config& candidate, const char* message) {
  check(!fish_control::validate_config(candidate).valid, message);
}
}

int main() {
  const Config current;
  check(fish_control::validate_config(current).valid,
        "mock accepts empty device and zero baudrate");

  auto candidate = current;
  candidate.x_model.pop_back();
  rejected(candidate, "geometry length must be rejected");
  candidate = current;
  candidate.motor_ids = {1, 1, 2, 3};
  rejected(candidate, "duplicate ids must be rejected");
  candidate = current;
  candidate.motor_names = {"fin", "fin", "fin_2", "fin_3"};
  rejected(candidate, "duplicate names must be rejected");
  candidate = current;
  candidate.amplitude_model = std::numeric_limits<double>::quiet_NaN();
  rejected(candidate, "NaN must be rejected");
  candidate = current;
  candidate.mapping_gain[2] = std::numeric_limits<double>::infinity();
  rejected(candidate, "Inf must be rejected");
  for (double tf : {0.0, 1.0, -0.1, 1.1}) {
    candidate = current;
    candidate.tf_coeff = tf;
    rejected(candidate, "tf strict boundaries must be rejected");
  }
  candidate = current;
  candidate.min_rad[0] = candidate.max_rad[0];
  rejected(candidate, "invalid lower/upper bounds must be rejected");
  candidate = current;
  candidate.direction[0] = 0;
  rejected(candidate, "direction must be plus or minus one");

  candidate = current;
  candidate.amplitude_model = 0.7;
  const auto accepted = fish_control::validate_config_update(
    current, candidate, ControllerState::HOLDING, {"gait.amplitude_model"});
  check(accepted.valid, "gait update accepted in HOLDING");
  check(current.amplitude_model == 0.5, "candidate validation does not mutate current config");
  check(!fish_control::validate_config_update(
      current, candidate, ControllerState::RUNNING, {"gait.amplitude_model"}).valid,
    "RUNNING rejects gait update");

  candidate = current;
  candidate.calibration_confirmed = true;
  check(!fish_control::validate_config_update(
      current, candidate, ControllerState::DISARMED, {"calibration.confirmed"}).valid,
    "calibration.confirmed is startup read-only");
  candidate = current;
  candidate.require_recording = false;
  check(!fish_control::validate_config_update(
      current, candidate, ControllerState::DISARMED, {"safety.require_recording"}).valid,
    "require_recording cannot be remotely disabled");
  candidate = current;
  candidate.normal_stop_policy = "return_zero";
  check(!fish_control::validate_config_update(
      current, candidate, ControllerState::DISARMED, {"safety.normal_stop_policy"}).valid,
    "normal stop policy cannot be changed");

  candidate = current;
  candidate.amplitude_model = 0.9;
  candidate.tf_coeff = 1.0;
  check(!fish_control::validate_config_update(
      current, candidate, ControllerState::DISARMED,
      {"gait.amplitude_model", "gait.tf_coeff"}).valid,
    "one invalid member rejects the whole atomic transaction");
  check(current.amplitude_model == 0.5 && current.tf_coeff == 0.35,
        "failed transaction leaves all current values untouched");

  candidate = current;
  candidate.backend = "dynamixel";
  candidate.mapping_kind = "linear_calibrated";
  rejected(candidate, "hardware requires nonempty device and positive baudrate");
  candidate.bus_device = "/dev/ttyUSB0";
  rejected(candidate, "hardware still rejects zero baudrate");
  candidate.bus_baudrate = 1000000;
  check(fish_control::validate_config(candidate).valid,
        "complete hardware config passes pure validation");
  check(!fish_control::mapping_is_valid(candidate),
        "unconfirmed hardware calibration must block arm");
  candidate.calibration_confirmed = true;
  check(fish_control::mapping_is_valid(candidate),
        "confirmed hardware calibration allows arm");
  candidate.mapping_kind = "mock_linear";
  rejected(candidate, "hardware requires linear_calibrated mapping");
  candidate.mapping_kind = "linear_calibrated";
  candidate.backend = "mock";
  rejected(candidate, "mock backend rejects linear_calibrated mapping");

  candidate = current;
  candidate.backend = "dynamixel";
  candidate.mapping_kind = "linear_calibrated";
  candidate.bus_device = "/dev/ttyUSB0";
  candidate.bus_baudrate = 1000000;
  candidate.zero_raw = {0, 0, 0, 0};
  candidate.min_rad = {-1.57, -1.57, -1.57, -1.57};
  rejected(candidate, "joint limits that wrap outside 0..4095 must be rejected");

  std::cout << "[PASS] config validation and atomic transaction tests\n";
  return 0;
}
