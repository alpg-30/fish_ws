#include <cmath>
#include <cstdlib>
#include <iostream>

#include "fish_control/safety.hpp"

namespace {
void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[FAIL] " << message << '\n';
    std::exit(1);
  }
}
void near(double actual, double expected, double tolerance = 1e-12) {
  check(std::abs(actual - expected) <= tolerance, "safety numeric mismatch");
}
}

int main() {
  fish_control::Config config;
  config.max_velocity_rad_s = {1.0, 1.0, 1.0, 1.0};
  config.max_acceleration_rad_s2 = {2.0, 2.0, 2.0, 2.0};
  const fish_control::Array4d ideal{1.0, 1.0, 1.0, 1.0};
  const fish_control::Array4d zero_position{};
  const fish_control::Array4d zero_velocity{};

  auto first = fish_control::limit_joint_step(
    ideal, zero_position, zero_velocity, 0.1, config);
  near(first.position[0], 0.02);
  near(first.velocity[0], 0.2);
  check(first.modified[0], "limited target must be marked safety_modified");
  auto second = fish_control::limit_joint_step(
    ideal, first.position, first.velocity, 0.1, config);
  near(second.position[0], 0.06);
  near(second.velocity[0], 0.4);

  auto position = second.position;
  auto velocity = second.velocity;
  for (int step = 0; step < 100; ++step) {
    const auto previous_velocity = velocity;
    const auto limited = fish_control::limit_joint_step(
      ideal, position, velocity, 0.1, config);
    for (std::size_t i = 0; i < fish_control::kMotorCount; ++i) {
      check(std::abs(limited.velocity[i]) <= 1.0 + 1e-12, "velocity cap");
      check(std::abs(limited.velocity[i] - previous_velocity[i]) <= 0.2 + 1e-12,
            "acceleration cap");
      check(limited.position[i] >= config.min_rad[i] &&
            limited.position[i] <= config.max_rad[i], "position cap");
    }
    position = limited.position;
    velocity = limited.velocity;
  }

  const fish_control::Array4d moving_position{0.4, 0.4, 0.4, 0.4};
  const fish_control::Array4d moving_velocity{1.0, 1.0, 1.0, 1.0};
  const auto braking = fish_control::brake_joint_step(
    moving_position, moving_velocity, 0.1, config);
  near(braking.position[0], 0.49);
  near(braking.velocity[0], 0.8);

  auto brake_position = moving_position;
  auto brake_velocity = moving_velocity;
  for (int step = 0; step < 5; ++step) {
    const auto next = fish_control::brake_joint_step(
      brake_position, brake_velocity, 0.1, config);
    brake_position = next.position;
    brake_velocity = next.velocity;
  }
  near(brake_velocity[0], 0.0);
  near(brake_position[0], 0.65, 1e-10);
  check(brake_position[0] != config.mapping_offset[0],
        "normal braking holds the reached position instead of returning to zero");
  const auto held = fish_control::brake_joint_step(
    brake_position, brake_velocity, 0.1, config);
  near(held.position[0], brake_position[0]);
  near(held.velocity[0], 0.0);

  fish_control::HoldConfirmation confirmation;
  const fish_control::Array4d feedback_position = brake_position;
  check(!confirmation.update(true, feedback_position, zero_velocity, brake_position, 0.02),
        "hold requires sustained confirmation");
  check(!confirmation.update(true, feedback_position, zero_velocity, brake_position, 0.02),
        "hold is not confirmed before 50 ms");
  check(confirmation.update(true, feedback_position, zero_velocity, brake_position, 0.02),
        "stable fresh feedback confirms hold");

  confirmation.reset();
  auto bad_position = feedback_position;
  bad_position[0] += 0.03;
  check(!confirmation.update(true, bad_position, zero_velocity, brake_position, 0.02),
        "tracking error prevents hold confirmation");
  check(!confirmation.update(true, feedback_position, zero_velocity, brake_position, 0.02),
        "tracking error resets confirmation duration");
  check(!confirmation.update(false, feedback_position, zero_velocity, brake_position, 0.02),
        "stale feedback prevents hold confirmation");

  std::cout << "[PASS] safety velocity/acceleration limits and bounded hold tests\n";
  return 0;
}
