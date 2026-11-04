#include "fish_control/safety.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fish_control {
namespace {
constexpr double kModificationEpsilon = 1e-12;
constexpr double kMockVelocityThreshold = 0.02;
constexpr double kMockPositionThreshold = 0.02;
constexpr double kHoldConfirmationDuration = 0.05;

void check_dt(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("safety step dt must be finite and positive");
  }
}
}  // namespace

SafetyStep limit_joint_step(
  const Array4d& ideal_position,
  const Array4d& previous_position,
  const Array4d& previous_velocity,
  double dt,
  const Config& config) {
  check_dt(dt);
  SafetyStep step;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const double max_velocity = config.max_velocity_rad_s[i];
    const double max_delta_velocity = config.max_acceleration_rad_s2[i] * dt;
    const double desired_velocity = (ideal_position[i] - previous_position[i]) / dt;
    double velocity = std::clamp(
      desired_velocity,
      previous_velocity[i] - max_delta_velocity,
      previous_velocity[i] + max_delta_velocity);
    velocity = std::clamp(velocity, -max_velocity, max_velocity);
    double position = previous_position[i] + velocity * dt;
    if (position < config.min_rad[i] || position > config.max_rad[i]) {
      position = std::clamp(position, config.min_rad[i], config.max_rad[i]);
      velocity = (position - previous_position[i]) / dt;
    }
    step.position[i] = position;
    step.velocity[i] = velocity;
    step.modified[i] = std::abs(position - ideal_position[i]) > kModificationEpsilon;
  }
  return step;
}

SafetyStep brake_joint_step(
  const Array4d& previous_position,
  const Array4d& previous_velocity,
  double dt,
  const Config& config) {
  check_dt(dt);
  SafetyStep step;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const double delta = config.max_acceleration_rad_s2[i] * dt;
    double velocity = 0.0;
    if (previous_velocity[i] > 0.0) {
      velocity = std::max(0.0, previous_velocity[i] - delta);
    } else if (previous_velocity[i] < 0.0) {
      velocity = std::min(0.0, previous_velocity[i] + delta);
    }
    double position =
      previous_position[i] + 0.5 * (previous_velocity[i] + velocity) * dt;
    if (position < config.min_rad[i] || position > config.max_rad[i]) {
      position = std::clamp(position, config.min_rad[i], config.max_rad[i]);
      velocity = 0.0;
    }
    step.position[i] = position;
    step.velocity[i] = velocity;
    step.modified[i] =
      std::abs(position - previous_position[i]) > kModificationEpsilon ||
      std::abs(velocity) > kModificationEpsilon;
  }
  return step;
}

void HoldConfirmation::reset() {
  elapsed_s_ = 0.0;
}

bool HoldConfirmation::update(
  bool feedback_fresh,
  const Array4d& feedback_position,
  const Array4d& feedback_velocity,
  const Array4d& hold_position,
  double dt) {
  check_dt(dt);
  bool settled = feedback_fresh;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!std::isfinite(feedback_position[i]) || !std::isfinite(feedback_velocity[i]) ||
        std::abs(feedback_velocity[i]) > kMockVelocityThreshold ||
        std::abs(feedback_position[i] - hold_position[i]) > kMockPositionThreshold) {
      settled = false;
    }
  }
  elapsed_s_ = settled ? elapsed_s_ + dt : 0.0;
  return elapsed_s_ >= kHoldConfirmationDuration;
}

}  // namespace fish_control
