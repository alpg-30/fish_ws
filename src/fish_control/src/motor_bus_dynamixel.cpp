// DYNAMIXEL X 系列（430）Protocol 2.0 硬件总线实现。
// 使用仓库内 src/DynamixelSDK 的 C++ 库，只做位置控制模式下的同步读/同步写。
#include "fish_control/motor_bus_dynamixel.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#ifdef FISH_HAVE_DYNAMIXEL_SDK
#include <dynamixel_sdk/dynamixel_sdk.h>
#endif

namespace fish_control {
namespace {

#ifdef FISH_HAVE_DYNAMIXEL_SDK

// X 系列 430 控制表（Protocol 2.0）
constexpr std::uint16_t kAddrTorqueEnable = 64;
constexpr std::uint16_t kAddrHardwareError = 70;
constexpr std::uint16_t kAddrGoalPosition = 116;
constexpr std::uint16_t kAddrPresentCurrent = 126;
constexpr std::uint16_t kAddrPresentVelocity = 128;
constexpr std::uint16_t kAddrPresentPosition = 132;
constexpr std::uint16_t kAddrInputVoltage = 144;
constexpr std::uint16_t kAddrTemperature = 146;
// 126..146 连续 21 字节；64..70 连续 7 字节（扭矩使能 + 硬件错误）。
constexpr std::uint16_t kFastBlockStart = 126;
constexpr std::uint16_t kFastBlockLength = 21;
constexpr std::uint16_t kStatusBlockStart = 64;
constexpr std::uint16_t kStatusBlockLength = 7;
constexpr double kPacketTimeoutMs = 30.0;

std::uint64_t steady_now_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

class DynamixelMotorBus final : public MotorBus {
public:
  explicit DynamixelMotorBus(Config config) : config_(std::move(config)) {}
  ~DynamixelMotorBus() override { close(); }

  BusResult open() override {
    if (opened_) {
      return {true, true, ""};
    }
    if (config_.bus_device.empty() || config_.bus_baudrate <= 0) {
      return {true, false, "dynamixel requires a nonempty bus.device and positive bus.baudrate"};
    }
    port_ = dynamixel::PortHandler::getPortHandler(config_.bus_device.c_str());
    if (port_ == nullptr) {
      return {true, false, "cannot create Dynamixel port handler"};
    }
    if (!port_->openPort()) {
      const std::string error = "cannot open " + config_.bus_device;
      close();
      return {true, false, error};
    }
    if (!port_->setBaudRate(config_.bus_baudrate)) {
      const std::string error = "cannot set baudrate " + std::to_string(config_.bus_baudrate);
      close();
      return {true, false, error};
    }
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0f);
    if (packet_ == nullptr) {
      close();
      return {true, false, "cannot create protocol 2.0 packet handler"};
    }
    port_->setPacketTimeout(kPacketTimeoutMs);

    for (std::size_t i = 0; i < kMotorCount; ++i) {
      std::uint16_t model_number = 0;
      std::uint8_t dxl_error = 0;
      const int result = packet_->ping(port_, motor_id(i), &model_number, &dxl_error);
      if (result != COMM_SUCCESS) {
        const std::string error = "ping id " + std::to_string(motor_id(i)) + " failed: " +
          packet_->getTxRxResult(result);
        close();
        return {true, false, error};
      }
      if (dxl_error != 0) {
        const std::string error = "ping id " + std::to_string(motor_id(i)) +
          " reported hardware error " + std::to_string(dxl_error);
        close();
        return {true, false, error};
      }
    }

    fast_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_, packet_, kFastBlockStart, kFastBlockLength);
    status_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_, packet_, kStatusBlockStart, kStatusBlockLength);
    goal_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_, packet_, kAddrGoalPosition, 4);
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!fast_read_->addParam(motor_id(i)) || !status_read_->addParam(motor_id(i))) {
        close();
        return {true, false, "duplicate motor id in sync read group"};
      }
    }

    opened_ = true;
    const auto initial = read_state();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!initial.valid[i]) {
        const std::string error = "no valid initial feedback from id " +
          std::to_string(motor_id(i));
        close();
        return {true, false, error};
      }
    }
    return {true, true, ""};
  }

  MotorState read_state() override {
    MotorState state;
    state.host_steady_time_ns = steady_now_ns();
    const auto started = std::chrono::steady_clock::now();
    if (!opened_) {
      state.communication_result.fill(-1);
      return state;
    }
    const int fast_result = fast_read_->txRxPacket();
    const int status_result = status_read_->txRxPacket();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      const std::uint8_t id = motor_id(i);
      const bool fast_ok = fast_result == COMM_SUCCESS &&
        fast_read_->isAvailable(id, kFastBlockStart, kFastBlockLength);
      const bool status_ok = status_result == COMM_SUCCESS &&
        status_read_->isAvailable(id, kStatusBlockStart, kStatusBlockLength);
      if (!fast_ok || !status_ok) {
        int code = fast_ok ? status_result : fast_result;
        if (code == COMM_SUCCESS) {
          code = COMM_RX_FAIL;
        }
        state.communication_result[i] = code;
        continue;
      }
      const auto current_raw = static_cast<std::int16_t>(
        fast_read_->getData(id, kAddrPresentCurrent, 2));
      const auto velocity_raw = static_cast<std::int32_t>(
        fast_read_->getData(id, kAddrPresentVelocity, 4));
      const auto position_raw = static_cast<std::int32_t>(
        fast_read_->getData(id, kAddrPresentPosition, 4));
      const auto voltage_raw = static_cast<std::uint16_t>(
        fast_read_->getData(id, kAddrInputVoltage, 2));
      const auto temperature_raw = static_cast<std::uint8_t>(
        fast_read_->getData(id, kAddrTemperature, 1));
      const auto torque_raw = static_cast<std::uint8_t>(
        status_read_->getData(id, kAddrTorqueEnable, 1));
      const auto hardware_error = static_cast<std::uint8_t>(
        status_read_->getData(id, kAddrHardwareError, 1));

      state.valid[i] = true;
      state.current_raw[i] = current_raw;
      state.current_a[i] = current_raw_to_amps(current_raw);
      state.position_raw[i] = position_raw;
      state.position_rad[i] = raw_to_joint(position_raw, i, config_);
      state.velocity_raw[i] = velocity_raw;
      state.velocity_rad_s[i] = velocity_raw_to_rad_s(velocity_raw, i, config_);
      state.input_voltage_v[i] = voltage_raw_to_volts(voltage_raw);
      state.temperature_c[i] = temperature_raw_to_celsius(temperature_raw);
      state.hardware_error[i] = hardware_error;
      state.hardware_error_valid[i] = true;
      state.hardware_error_age_s[i] = 0.0;
      state.torque_enabled[i] = torque_raw != 0;
      state.torque_enabled_valid[i] = true;
      state.torque_state_age_s[i] = 0.0;
      state.communication_result[i] = 0;
    }
    state.acquisition_duration_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    return state;
  }

  BusResult write_goal_positions(const Array4i& goal_position_raw) override {
    if (!opened_) {
      return {false, false, "dynamixel bus is not open"};
    }
    goal_write_->clearParam();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      const auto raw = static_cast<std::uint32_t>(goal_position_raw[i]);
      std::uint8_t param[4] = {
        static_cast<std::uint8_t>(raw & 0xFF),
        static_cast<std::uint8_t>((raw >> 8) & 0xFF),
        static_cast<std::uint8_t>((raw >> 16) & 0xFF),
        static_cast<std::uint8_t>((raw >> 24) & 0xFF),
      };
      if (!goal_write_->addParam(motor_id(i), param)) {
        goal_write_->clearParam();
        return {true, false, "failed to add sync write param"};
      }
    }
    const int result = goal_write_->txPacket();
    goal_write_->clearParam();
    if (result != COMM_SUCCESS) {
      return {true, false, packet_->getTxRxResult(result)};
    }
    return {true, true, ""};
  }

  BusResult set_torque(bool enabled) override {
    if (!opened_) {
      return {false, false, "dynamixel bus is not open"};
    }
    bool failed = false;
    std::string error;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      std::uint8_t dxl_error = 0;
      const int result = packet_->write1ByteTxRx(
        port_, motor_id(i), kAddrTorqueEnable, enabled ? 1 : 0, &dxl_error);
      if (result != COMM_SUCCESS) {
        failed = true;
        error = "torque write id " + std::to_string(motor_id(i)) + ": " +
          packet_->getTxRxResult(result);
      } else if (dxl_error != 0) {
        failed = true;
        error = "torque write id " + std::to_string(motor_id(i)) +
          " hardware error " + std::to_string(dxl_error);
      }
    }
    return {true, !failed, error};
  }

  void close() override {
    goal_write_.reset();
    status_read_.reset();
    fast_read_.reset();
    if (port_ != nullptr) {
      port_->closePort();
      delete port_;
      port_ = nullptr;
    }
    if (packet_ != nullptr) {
      delete packet_;
      packet_ = nullptr;
    }
    opened_ = false;
  }

  std::string backend() const override {
    return "dynamixel";
  }

private:
  std::uint8_t motor_id(std::size_t index) const {
    return static_cast<std::uint8_t>(config_.motor_ids[index]);
  }

  Config config_;
  dynamixel::PortHandler* port_{nullptr};
  dynamixel::PacketHandler* packet_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead> fast_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> status_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> goal_write_;
  bool opened_{false};
};

#endif  // FISH_HAVE_DYNAMIXEL_SDK

}  // namespace

std::unique_ptr<MotorBus> make_dynamixel_motor_bus(const Config& config) {
#ifdef FISH_HAVE_DYNAMIXEL_SDK
  if (config.motor_ids.size() != kMotorCount) {
    throw std::runtime_error("dynamixel backend requires exactly four motor ids");
  }
  return std::make_unique<DynamixelMotorBus>(config);
#else
  (void)config;
  throw std::runtime_error(
    "DynamixelSDK was not available at build time; refusing to fall back to mock");
#endif
}

}  // namespace fish_control
