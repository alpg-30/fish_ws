#pragma once

#include "fish_control/config.hpp"

namespace fish_control {

struct SafetyStep {
  Array4d position{};
  Array4d velocity{};
  Array4b modified{};
};

SafetyStep limit_joint_step(
  const Array4d& ideal_position,
  const Array4d& previous_position,
  const Array4d& previous_velocity,
  double dt,
  const Config& config);

SafetyStep brake_joint_step(
  const Array4d& previous_position,
  const Array4d& previous_velocity,
  double dt,
  const Config& config);

class HoldConfirmation {
public:
  void reset();
  bool update(
    bool feedback_fresh,
    const Array4d& feedback_position,
    const Array4d& feedback_velocity,
    const Array4d& hold_position,
    double dt);

private:
  double elapsed_s_{0.0};
};

}  // namespace fish_control
