#pragma once

#include <memory>

#include "fish_control/motor_bus.hpp"

namespace fish_control {

// 未在编译期找到 DynamixelSDK 时抛错，绝不回退 mock。
std::unique_ptr<MotorBus> make_dynamixel_motor_bus(const Config& config);

}  // namespace fish_control
