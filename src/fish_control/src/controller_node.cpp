#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fish_interfaces/msg/experiment_state.hpp>
#include <fish_interfaces/msg/flapping_command.hpp>
#include <fish_interfaces/msg/gait_state.hpp>
#include <fish_interfaces/msg/motor_telemetry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fish_control/config.hpp"
#include "fish_control/fluent_waveform.hpp"
#include "fish_control/motor_bus.hpp"
#include "fish_control/safety.hpp"

namespace fish_control {
namespace {
using Trigger = std_srvs::srv::Trigger;
constexpr double kRecorderTimeoutS = 1.0;
constexpr double kBrakingTimeoutS = 5.0;

std::uint64_t steady_now_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool assign_parameter(Config& c, const rclcpp::Parameter& p) {
  const auto& n = p.get_name();
  if (n == "backend") c.backend = p.as_string();
  else if (n == "timing.control_hz") c.control_hz = p.as_double();
  else if (n == "timing.telemetry_hz") c.telemetry_hz = p.as_double();
  else if (n == "motors.names") c.motor_names = p.as_string_array();
  else if (n == "motors.ids") c.motor_ids = p.as_integer_array();
  else if (n == "gait.waveform") c.waveform = p.as_string();
  else if (n == "gait.amplitude_model") c.amplitude_model = p.as_double();
  else if (n == "gait.frequency_hz") c.frequency_hz = p.as_double();
  else if (n == "gait.tf_coeff") c.tf_coeff = p.as_double();
  else if (n == "gait.body_coeff") c.body_coeff = p.as_double();
  else if (n == "geometry.x_model") c.x_model = p.as_double_array();
  else if (n == "geometry.z_model") c.z_model = p.as_double_array();
  else if (n == "geometry.model_unit") c.model_unit = p.as_string();
  else if (n == "mapping.kind") c.mapping_kind = p.as_string();
  else if (n == "mapping.gain_rad_per_model_unit") c.mapping_gain = p.as_double_array();
  else if (n == "mapping.offset_rad") c.mapping_offset = p.as_double_array();
  else if (n == "calibration.zero_raw") c.zero_raw = p.as_integer_array();
  else if (n == "calibration.direction") c.direction = p.as_integer_array();
  else if (n == "calibration.external_ratio") c.external_ratio = p.as_double_array();
  else if (n == "calibration.confirmed") c.calibration_confirmed = p.as_bool();
  else if (n == "limits.min_rad") c.min_rad = p.as_double_array();
  else if (n == "limits.max_rad") c.max_rad = p.as_double_array();
  else if (n == "limits.max_velocity_rad_s") c.max_velocity_rad_s = p.as_double_array();
  else if (n == "limits.max_acceleration_rad_s2") c.max_acceleration_rad_s2 = p.as_double_array();
  else if (n == "limits.current_trip_a") c.current_trip_a = p.as_double_array();
  else if (n == "limits.current_trip_duration_s") c.current_trip_duration_s = p.as_double();
  else if (n == "limits.temperature_trip_c") c.temperature_trip_c = p.as_double_array();
  else if (n == "limits.min_voltage_v") c.min_voltage_v = p.as_double_array();
  else if (n == "limits.max_voltage_v") c.max_voltage_v = p.as_double_array();
  else if (n == "safety.feedback_timeout_s") c.feedback_timeout_s = p.as_double();
  else if (n == "safety.bus_watchdog_ms") c.bus_watchdog_ms = p.as_int();
  else if (n == "safety.normal_stop_policy") c.normal_stop_policy = p.as_string();
  else if (n == "safety.require_recording") c.require_recording = p.as_bool();
  else if (n == "bus.device") c.bus_device = p.as_string();
  else if (n == "bus.baudrate") c.bus_baudrate = p.as_int();
  else if (n == "bus.protocol_version") c.bus_protocol_version = p.as_double();
  else return false;
  return true;
}

class ControllerNode final : public rclcpp::Node {
public:
  ControllerNode() : Node("controller") {
    declare_config_parameters();
    accepted_config_ = read_config();
    const auto validation = validate_config(accepted_config_);
    if (!validation.valid) {
      throw std::invalid_argument("invalid startup configuration: " + validation.error);
    }
    active_config_ = accepted_config_;
    bus_ = make_motor_bus(accepted_config_);
    last_sample_ = evaluate_waveform(0.0, accepted_config_);
    last_sample_.phase_rate_rad_s = 0.0;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      applied_position_[i] = accepted_config_.mapping_offset[i];
      hold_position_[i] = applied_position_[i];
    }

    const auto command_qos =
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
    const auto joint_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto latched_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto diagnostics_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    command_pub_ = create_publisher<fish_interfaces::msg::FlappingCommand>(
      "flapping_command", command_qos);
    telemetry_pub_ = create_publisher<fish_interfaces::msg::MotorTelemetry>(
      "motor_telemetry", command_qos);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", joint_qos);
    gait_pub_ = create_publisher<fish_interfaces::msg::GaitState>("gait_state", latched_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", diagnostics_qos);
    experiment_sub_ = create_subscription<fish_interfaces::msg::ExperimentState>(
      "experiment_state", latched_qos,
      [this](fish_interfaces::msg::ExperimentState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_experiment_ = std::move(*message);
        last_experiment_ns_ = steady_now_ns();
        have_experiment_ = true;
      });

    arm_service_ = make_request_service("arm", Request::ARM);
    start_service_ = make_request_service("start", Request::START);
    stop_service_ = make_request_service("stop_hold", Request::STOP_HOLD);
    disarm_service_ = make_request_service("disarm", Request::DISARM);
    estop_service_ = make_request_service("estop", Request::ESTOP);
    reset_service_ = make_request_service("reset_fault", Request::RESET_FAULT);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        return on_parameters(parameters);
      });

    const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / accepted_config_.control_hz));
    control_timer_ = create_wall_timer(control_period, [this]() { control_tick(); });
    last_control_ns_ = steady_now_ns();
    next_telemetry_ns_ = last_control_ns_;
    last_gait_publish_ns_ = 0;
    RCLCPP_INFO(get_logger(), "controller initialized with backend=%s", bus_->backend().c_str());
  }

  ~ControllerNode() override {
    if (bus_) {
      bus_->close();
    }
  }

private:
  enum class Request { NONE, ARM, START, STOP_HOLD, DISARM, ESTOP, RESET_FAULT };

  rclcpp::Service<Trigger>::SharedPtr make_request_service(
    const std::string& name, Request request) {
    return create_service<Trigger>(
      "controller/" + name,
      [this, request](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        queue_request(request, response);
      });
  }

  void declare_config_parameters() {
    const Config c;
    declare_parameter<std::string>("backend", c.backend);
    declare_parameter<double>("timing.control_hz", c.control_hz);
    declare_parameter<double>("timing.telemetry_hz", c.telemetry_hz);
    declare_parameter<std::vector<std::string>>("motors.names", c.motor_names);
    declare_parameter<std::vector<std::int64_t>>("motors.ids", c.motor_ids);
    declare_parameter<std::string>("gait.waveform", c.waveform);
    declare_parameter<double>("gait.amplitude_model", c.amplitude_model);
    declare_parameter<double>("gait.frequency_hz", c.frequency_hz);
    declare_parameter<double>("gait.tf_coeff", c.tf_coeff);
    declare_parameter<double>("gait.body_coeff", c.body_coeff);
    declare_parameter<std::vector<double>>("geometry.x_model", c.x_model);
    declare_parameter<std::vector<double>>("geometry.z_model", c.z_model);
    declare_parameter<std::string>("geometry.model_unit", c.model_unit);
    declare_parameter<std::string>("mapping.kind", c.mapping_kind);
    declare_parameter<std::vector<double>>("mapping.gain_rad_per_model_unit", c.mapping_gain);
    declare_parameter<std::vector<double>>("mapping.offset_rad", c.mapping_offset);
    declare_parameter<std::vector<std::int64_t>>("calibration.zero_raw", c.zero_raw);
    declare_parameter<std::vector<std::int64_t>>("calibration.direction", c.direction);
    declare_parameter<std::vector<double>>("calibration.external_ratio", c.external_ratio);
    declare_parameter<bool>("calibration.confirmed", c.calibration_confirmed);
    declare_parameter<std::vector<double>>("limits.min_rad", c.min_rad);
    declare_parameter<std::vector<double>>("limits.max_rad", c.max_rad);
    declare_parameter<std::vector<double>>("limits.max_velocity_rad_s", c.max_velocity_rad_s);
    declare_parameter<std::vector<double>>(
      "limits.max_acceleration_rad_s2", c.max_acceleration_rad_s2);
    declare_parameter<std::vector<double>>("limits.current_trip_a", c.current_trip_a);
    declare_parameter<double>("limits.current_trip_duration_s", c.current_trip_duration_s);
    declare_parameter<std::vector<double>>("limits.temperature_trip_c", c.temperature_trip_c);
    declare_parameter<std::vector<double>>("limits.min_voltage_v", c.min_voltage_v);
    declare_parameter<std::vector<double>>("limits.max_voltage_v", c.max_voltage_v);
    declare_parameter<double>("safety.feedback_timeout_s", c.feedback_timeout_s);
    declare_parameter<std::int64_t>("safety.bus_watchdog_ms", c.bus_watchdog_ms);
    declare_parameter<std::string>("safety.normal_stop_policy", c.normal_stop_policy);
    declare_parameter<bool>("safety.require_recording", c.require_recording);
    declare_parameter<std::string>("bus.device", c.bus_device);
    declare_parameter<std::int64_t>("bus.baudrate", c.bus_baudrate);
    declare_parameter<double>("bus.protocol_version", c.bus_protocol_version);
  }

  Config read_config() const {
    Config c;
    c.backend = get_parameter("backend").as_string();
    c.control_hz = get_parameter("timing.control_hz").as_double();
    c.telemetry_hz = get_parameter("timing.telemetry_hz").as_double();
    c.motor_names = get_parameter("motors.names").as_string_array();
    c.motor_ids = get_parameter("motors.ids").as_integer_array();
    c.waveform = get_parameter("gait.waveform").as_string();
    c.amplitude_model = get_parameter("gait.amplitude_model").as_double();
    c.frequency_hz = get_parameter("gait.frequency_hz").as_double();
    c.tf_coeff = get_parameter("gait.tf_coeff").as_double();
    c.body_coeff = get_parameter("gait.body_coeff").as_double();
    c.x_model = get_parameter("geometry.x_model").as_double_array();
    c.z_model = get_parameter("geometry.z_model").as_double_array();
    c.model_unit = get_parameter("geometry.model_unit").as_string();
    c.mapping_kind = get_parameter("mapping.kind").as_string();
    c.mapping_gain = get_parameter("mapping.gain_rad_per_model_unit").as_double_array();
    c.mapping_offset = get_parameter("mapping.offset_rad").as_double_array();
    c.zero_raw = get_parameter("calibration.zero_raw").as_integer_array();
    c.direction = get_parameter("calibration.direction").as_integer_array();
    c.external_ratio = get_parameter("calibration.external_ratio").as_double_array();
    c.calibration_confirmed = get_parameter("calibration.confirmed").as_bool();
    c.min_rad = get_parameter("limits.min_rad").as_double_array();
    c.max_rad = get_parameter("limits.max_rad").as_double_array();
    c.max_velocity_rad_s = get_parameter("limits.max_velocity_rad_s").as_double_array();
    c.max_acceleration_rad_s2 = get_parameter("limits.max_acceleration_rad_s2").as_double_array();
    c.current_trip_a = get_parameter("limits.current_trip_a").as_double_array();
    c.current_trip_duration_s = get_parameter("limits.current_trip_duration_s").as_double();
    c.temperature_trip_c = get_parameter("limits.temperature_trip_c").as_double_array();
    c.min_voltage_v = get_parameter("limits.min_voltage_v").as_double_array();
    c.max_voltage_v = get_parameter("limits.max_voltage_v").as_double_array();
    c.feedback_timeout_s = get_parameter("safety.feedback_timeout_s").as_double();
    c.bus_watchdog_ms = get_parameter("safety.bus_watchdog_ms").as_int();
    c.normal_stop_policy = get_parameter("safety.normal_stop_policy").as_string();
    c.require_recording = get_parameter("safety.require_recording").as_bool();
    c.bus_device = get_parameter("bus.device").as_string();
    c.bus_baudrate = get_parameter("bus.baudrate").as_int();
    c.bus_protocol_version = get_parameter("bus.protocol_version").as_double();
    return c;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    Config candidate = accepted_config_;
    std::vector<std::string> changed;
    changed.reserve(parameters.size());
    for (const auto& parameter : parameters) {
      if (!assign_parameter(candidate, parameter)) {
        return parameter_rejected("unknown parameter: " + parameter.get_name());
      }
      changed.push_back(parameter.get_name());
    }
    const auto validation =
      validate_config_update(accepted_config_, candidate, state_, changed);
    if (!validation.valid) {
      return parameter_rejected(validation.error);
    }
    accepted_config_ = candidate;
    ++accepted_config_revision_;
    state_dirty_ = true;
    publish_gait_locked(steady_now_ns());
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  rcl_interfaces::msg::SetParametersResult parameter_rejected(const std::string& reason) {
    RCLCPP_WARN(get_logger(), "parameter transaction rejected: %s", reason.c_str());
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
  }

  void queue_request(Request request, Trigger::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request == Request::ESTOP) {
      if (state_ == ControllerState::ESTOP_LATCHED) {
        response->success = true;
        response->message = "already ESTOP_LATCHED";
        return;
      }
      pending_request_ = request;
      response->success = true;
      response->message = "estop queued";
      return;
    }
    if (pending_request_ != Request::NONE) {
      response->success = false;
      response->message = "another controller request is pending";
      return;
    }
    bool allowed = false;
    std::string message;
    switch (request) {
      case Request::ARM:
        allowed = state_ == ControllerState::DISARMED && mapping_is_valid(accepted_config_);
        message = allowed ? "arm queued" : "arm requires DISARMED and valid mapping";
        break;
      case Request::START: {
        response->success = start_locked(steady_now_ns());
        response->message = response->success ? "RUNNING" :
          "start requires HOLDING, fresh recorder_ready session, positive frequency and valid mapping";
        if (response->success) {
          publish_gait_locked(steady_now_ns());
        }
        return;
      }
      case Request::STOP_HOLD:
        if (state_ == ControllerState::HOLDING || state_ == ControllerState::BRAKING) {
          response->success = true;
          response->message = to_string(state_);
          return;
        }
        allowed = state_ == ControllerState::RUNNING;
        message = allowed ? "stop_hold queued" : "stop_hold requires RUNNING";
        break;
      case Request::DISARM:
        if (state_ == ControllerState::DISARMED) {
          response->success = true;
          response->message = "already DISARMED";
          return;
        }
        allowed = state_ == ControllerState::HOLDING;
        message = allowed ? "disarm queued" : "disarm requires HOLDING";
        break;
      case Request::RESET_FAULT:
        allowed = state_ == ControllerState::FAULT ||
                  state_ == ControllerState::ESTOP_LATCHED;
        message = allowed ? "reset queued" : "reset_fault requires a latched fault";
        break;
      case Request::NONE:
      case Request::ESTOP:
        break;
    }
    if (allowed) {
      pending_request_ = request;
    }
    response->success = allowed;
    response->message = message;
  }

  bool recorder_valid_locked(std::uint64_t now_ns, const std::string& expected_run_id) const {
    if (!accepted_config_.require_recording) {
      return true;
    }
    if (!have_experiment_ || !last_experiment_.recorder_ready ||
        last_experiment_.run_id.empty() ||
        (last_experiment_.state != "STARTING" && last_experiment_.state != "RUNNING" &&
         last_experiment_.state != "STOPPING") ||
        (!expected_run_id.empty() && last_experiment_.run_id != expected_run_id)) {
      return false;
    }
    const double age = static_cast<double>(now_ns - last_experiment_ns_) * 1e-9;
    return age >= 0.0 && age <= kRecorderTimeoutS;
  }

  void control_tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t now_ns = steady_now_ns();
    const double dt = std::clamp(
      static_cast<double>(now_ns - last_control_ns_) * 1e-9, 1e-6, 0.1);
    last_control_ns_ = now_ns;
    last_goal_write_ = BusResult{};
    try {
      if (state_ == ControllerState::BOOT) {
        const auto opened = bus_->open();
        if (!opened.succeeded) {
          enter_fault_locked("bus_open_failed: " + opened.error);
        } else {
          set_state_locked(ControllerState::DISARMED);
        }
      }
      if (now_ns >= next_telemetry_ns_) {
        read_feedback_locked(true);
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / accepted_config_.telemetry_hz)).count();
        next_telemetry_ns_ = now_ns + static_cast<std::uint64_t>(period);
      }
      process_request_locked(now_ns);
      if (state_ == ControllerState::RUNNING &&
          !recorder_valid_locked(now_ns, active_run_id_)) {
        last_warning_ = "recorder session invalid; bounded stop requested";
        RCLCPP_WARN(get_logger(), "%s", last_warning_.c_str());
        begin_brake_locked();
      }
      if (torque_expected_locked()) {
        check_feedback_safety_locked(now_ns, dt);
      }
      generate_target_locked(now_ns, dt);
      if (torque_expected_locked()) {
        write_goal_locked(now_ns);
      }
      publish_command_locked(now_ns);
      if (state_dirty_ || now_ns - last_gait_publish_ns_ >= 1000000000ULL) {
        publish_gait_locked(now_ns);
      }
    } catch (const std::exception& error) {
      enter_fault_locked(std::string("controller_exception: ") + error.what());
      publish_command_locked(now_ns);
      publish_gait_locked(now_ns);
    }
  }

  void process_request_locked(std::uint64_t now_ns) {
    const Request request = pending_request_;
    pending_request_ = Request::NONE;
    switch (request) {
      case Request::NONE:
        return;
      case Request::ARM:
        arm_locked(now_ns);
        return;
      case Request::START:
        (void)start_locked(now_ns);
        return;
      case Request::STOP_HOLD:
        if (state_ == ControllerState::RUNNING) {
          begin_brake_locked();
        }
        return;
      case Request::DISARM:
        disarm_locked();
        return;
      case Request::ESTOP:
        enter_estop_locked();
        return;
      case Request::RESET_FAULT:
        if (state_ == ControllerState::FAULT || state_ == ControllerState::ESTOP_LATCHED) {
          fault_code_.clear();
          last_warning_.clear();
          applied_velocity_.fill(0.0);
          active_run_id_.clear();
          hold_confirmation_.reset();
          set_state_locked(ControllerState::DISARMED);
        }
        return;
    }
  }

  bool start_locked(std::uint64_t now_ns) {
    if (state_ != ControllerState::HOLDING || !recorder_valid_locked(now_ns, "") ||
        accepted_config_.frequency_hz <= 0.0 || !mapping_is_valid(accepted_config_)) {
      return false;
    }
    active_config_ = accepted_config_;
    active_config_revision_ = accepted_config_revision_;
    active_run_id_ = last_experiment_.run_id;
    phase_start_ns_ = now_ns;
    last_sample_ = evaluate_waveform(0.0, active_config_);
    set_state_locked(ControllerState::RUNNING);
    return true;
  }

  void arm_locked(std::uint64_t now_ns) {
    if (state_ != ControllerState::DISARMED || !mapping_is_valid(accepted_config_)) {
      return;
    }
    set_state_locked(ControllerState::ARMING);
    read_feedback_locked(false);
    if (!feedback_fresh_locked(now_ns)) {
      enter_fault_locked("arm_feedback_invalid");
      return;
    }
    applied_position_ = last_motor_state_.position_rad;
    applied_velocity_.fill(0.0);
    hold_position_ = applied_position_;
    update_goal_raw_locked();
    last_goal_write_ = bus_->write_goal_positions(goal_raw_);
    if (!last_goal_write_.succeeded) {
      enter_fault_locked("arm_goal_write_failed: " + last_goal_write_.error);
      return;
    }
    last_successful_write_ns_ = now_ns;
    const auto torque = bus_->set_torque(true);
    if (!torque.succeeded) {
      enter_fault_locked("arm_torque_write_failed: " + torque.error);
      return;
    }
    read_feedback_locked(false);
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!last_motor_state_.valid[i] || !last_motor_state_.torque_enabled_valid[i] ||
          !last_motor_state_.torque_enabled[i]) {
        enter_fault_locked("arm_torque_readback_not_enabled");
        return;
      }
    }
    const Config& waveform_config =
      active_config_revision_ == 0 ? accepted_config_ : active_config_;
    last_sample_ = evaluate_waveform(0.0, waveform_config);
    last_sample_.phase_rate_rad_s = 0.0;
    hold_confirmation_.reset();
    set_state_locked(ControllerState::HOLDING);
  }

  void disarm_locked() {
    if (state_ != ControllerState::HOLDING) {
      return;
    }
    const auto torque = bus_->set_torque(false);
    if (!torque.succeeded) {
      enter_fault_locked("disarm_torque_write_failed: " + torque.error);
      return;
    }
    read_feedback_locked(false);
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!last_motor_state_.valid[i] || !last_motor_state_.torque_enabled_valid[i] ||
          last_motor_state_.torque_enabled[i]) {
        enter_fault_locked("disarm_torque_readback_still_enabled");
        return;
      }
    }
    applied_velocity_.fill(0.0);
    active_run_id_.clear();
    set_state_locked(ControllerState::DISARMED);
  }

  void begin_brake_locked() {
    if (state_ != ControllerState::RUNNING) {
      return;
    }
    braking_elapsed_s_ = 0.0;
    hold_confirmation_.reset();
    last_sample_.phase_rate_rad_s = 0.0;
    set_state_locked(ControllerState::BRAKING);
  }

  void generate_target_locked(std::uint64_t now_ns, double dt) {
    if (state_ == ControllerState::RUNNING) {
      const double time_s = static_cast<double>(now_ns - phase_start_ns_) * 1e-9;
      last_sample_ = evaluate_waveform(time_s, active_config_);
      if (!last_sample_.mapping_valid) {
        enter_fault_locked("mapping_invalid");
        return;
      }
      const auto limited = limit_joint_step(
        last_sample_.ideal_joint_target_rad, applied_position_, applied_velocity_, dt,
        active_config_);
      applied_position_ = limited.position;
      applied_velocity_ = limited.velocity;
    } else if (state_ == ControllerState::BRAKING) {
      const auto braking = brake_joint_step(
        applied_position_, applied_velocity_, dt, active_config_);
      applied_position_ = braking.position;
      applied_velocity_ = braking.velocity;
      braking_elapsed_s_ += dt;
      bool zero_command_velocity = true;
      for (double velocity : applied_velocity_) {
        zero_command_velocity = zero_command_velocity && velocity == 0.0;
      }
      if (zero_command_velocity) {
        hold_position_ = applied_position_;
        if (hold_confirmation_.update(
              feedback_fresh_locked(now_ns), last_motor_state_.position_rad,
              last_motor_state_.velocity_rad_s, hold_position_, dt)) {
          applied_velocity_.fill(0.0);
          active_run_id_.clear();
          set_state_locked(ControllerState::HOLDING);
        }
      } else {
        hold_confirmation_.reset();
      }
      if (state_ == ControllerState::BRAKING && braking_elapsed_s_ > kBrakingTimeoutS) {
        enter_fault_locked("bounded_stop_timeout");
        return;
      }
    } else if (state_ == ControllerState::HOLDING) {
      applied_position_ = hold_position_;
      applied_velocity_.fill(0.0);
    }
    update_goal_raw_locked();
  }

  void update_goal_raw_locked() {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      goal_raw_[i] = joint_to_raw(applied_position_[i], i, active_config_);
    }
  }

  void write_goal_locked(std::uint64_t now_ns) {
    last_goal_write_ = bus_->write_goal_positions(goal_raw_);
    if (last_goal_write_.succeeded) {
      last_successful_write_ns_ = now_ns;
    } else {
      enter_fault_locked("goal_write_failed: " + last_goal_write_.error);
    }
  }

  bool torque_expected_locked() const {
    return state_ == ControllerState::ARMING || state_ == ControllerState::HOLDING ||
           state_ == ControllerState::RUNNING || state_ == ControllerState::BRAKING;
  }

  bool feedback_fresh_locked(std::uint64_t now_ns) const {
    if (!have_feedback_) {
      return false;
    }
    for (bool valid : last_motor_state_.valid) {
      if (!valid) {
        return false;
      }
    }
    const double age =
      static_cast<double>(now_ns - last_motor_state_.host_steady_time_ns) * 1e-9;
    return age >= 0.0 && age <= active_config_.feedback_timeout_s;
  }

  void read_feedback_locked(bool publish) {
    last_motor_state_ = bus_->read_state();
    have_feedback_ = true;
    if (publish) {
      publish_telemetry_locked();
      publish_diagnostics_locked();
    }
  }

  void check_feedback_safety_locked(std::uint64_t now_ns, double dt) {
    if (!feedback_fresh_locked(now_ns)) {
      enter_fault_locked("feedback_timeout_or_invalid");
      return;
    }
    bool over_current = false;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!last_motor_state_.hardware_error_valid[i] ||
          !last_motor_state_.torque_enabled_valid[i] ||
          !last_motor_state_.torque_enabled[i]) {
        enter_fault_locked("hardware_or_torque_feedback_invalid");
        return;
      }
      if (last_motor_state_.hardware_error[i] != 0) {
        enter_fault_locked("motor_hardware_error");
        return;
      }
      if (!std::isfinite(last_motor_state_.current_a[i]) ||
          !std::isfinite(last_motor_state_.input_voltage_v[i]) ||
          !std::isfinite(last_motor_state_.temperature_c[i]) ||
          last_motor_state_.input_voltage_v[i] < active_config_.min_voltage_v[i] ||
          last_motor_state_.input_voltage_v[i] > active_config_.max_voltage_v[i] ||
          last_motor_state_.temperature_c[i] > active_config_.temperature_trip_c[i]) {
        enter_fault_locked("motor_limit_or_invalid_feedback");
        return;
      }
      over_current = over_current ||
        std::abs(last_motor_state_.current_a[i]) > active_config_.current_trip_a[i];
    }
    over_current_elapsed_s_ = over_current ? over_current_elapsed_s_ + dt : 0.0;
    if (over_current_elapsed_s_ >= active_config_.current_trip_duration_s) {
      enter_fault_locked("sustained_overcurrent");
      return;
    }
    if (last_successful_write_ns_ != 0) {
      const double write_age = static_cast<double>(now_ns - last_successful_write_ns_) * 1e-9;
      if (write_age > static_cast<double>(active_config_.bus_watchdog_ms) * 1e-3) {
        enter_fault_locked("bus_write_watchdog");
      }
    }
  }

  void enter_fault_locked(const std::string& code) {
    if (state_ == ControllerState::FAULT || state_ == ControllerState::ESTOP_LATCHED) {
      return;
    }
    fault_code_ = code;
    pending_request_ = Request::NONE;
    active_run_id_.clear();
    applied_velocity_.fill(0.0);
    bus_->set_torque(false);
    RCLCPP_ERROR(get_logger(), "FAULT latched: %s", code.c_str());
    set_state_locked(ControllerState::FAULT);
  }

  void enter_estop_locked() {
    fault_code_ = "software_estop";
    pending_request_ = Request::NONE;
    active_run_id_.clear();
    applied_velocity_.fill(0.0);
    bus_->set_torque(false);
    RCLCPP_ERROR(get_logger(), "software ESTOP latched");
    set_state_locked(ControllerState::ESTOP_LATCHED);
  }

  void set_state_locked(ControllerState state) {
    if (state_ != state) {
      state_ = state;
      state_dirty_ = true;
    }
  }

  void publish_command_locked(std::uint64_t now_ns) {
    fish_interfaces::msg::FlappingCommand msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "fish";
    msg.sequence = ++command_sequence_;
    msg.host_steady_time_ns = now_ns;
    msg.backend = accepted_config_.backend;
    msg.active_config_revision = active_config_revision_;
    const Config& waveform_config = active_config_revision_ == 0 ? accepted_config_ : active_config_;
    msg.cycle_time_s = waveform_config.frequency_hz > 0.0 ?
      1.0 / waveform_config.frequency_hz : std::numeric_limits<double>::quiet_NaN();
    msg.phase_rad = last_sample_.phase_rad;
    msg.phase_rate_rad_s = state_ == ControllerState::RUNNING ?
      last_sample_.phase_rate_rad_s : 0.0;
    msg.model_unit = waveform_config.model_unit;
    msg.mapping_valid = last_sample_.mapping_valid;
    msg.transport_write_attempted = last_goal_write_.attempted;
    msg.transport_write_succeeded = last_goal_write_.succeeded;
    msg.controller_state = to_string(state_);
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      msg.names[i] = accepted_config_.motor_names[i];
      msg.spatial_phase_rad[i] = last_sample_.spatial_phase_rad[i];
      msg.wave_value[i] = last_sample_.wave_value[i];
      msg.model_displacement[i] = last_sample_.model_displacement[i];
      msg.ideal_joint_target_rad[i] = last_sample_.ideal_joint_target_rad[i];
      msg.applied_joint_target_rad[i] = applied_position_[i];
      msg.goal_position_raw[i] = goal_raw_[i];
      msg.command_valid[i] = last_sample_.mapping_valid &&
        std::isfinite(applied_position_[i]);
      msg.safety_modified[i] = last_sample_.mapping_valid &&
        std::abs(last_sample_.ideal_joint_target_rad[i] - applied_position_[i]) > 1e-12;
    }
    command_pub_->publish(msg);
  }

  void publish_telemetry_locked() {
    const auto& state = last_motor_state_;
    fish_interfaces::msg::MotorTelemetry msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "fish";
    msg.sequence = ++telemetry_sequence_;
    msg.host_steady_time_ns = state.host_steady_time_ns;
    msg.backend = bus_->backend();
    msg.active_config_revision = active_config_revision_;
    msg.acquisition_duration_s = state.acquisition_duration_s;
    sensor_msgs::msg::JointState joint;
    joint.header = msg.header;
    joint.name.resize(kMotorCount);
    joint.position.resize(kMotorCount);
    joint.velocity.resize(kMotorCount);
    joint.effort.clear();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      msg.ids[i] = static_cast<std::uint8_t>(accepted_config_.motor_ids[i]);
      msg.names[i] = accepted_config_.motor_names[i];
      msg.valid[i] = state.valid[i];
      msg.current_raw[i] = state.current_raw[i];
      msg.position_raw[i] = state.position_raw[i];
      msg.velocity_raw[i] = state.velocity_raw[i];
      msg.current_a[i] = state.valid[i] ? state.current_a[i] : nan;
      msg.position_rad[i] = state.valid[i] ? state.position_rad[i] : nan;
      msg.velocity_rad_s[i] = state.valid[i] ? state.velocity_rad_s[i] : nan;
      msg.input_voltage_v[i] = state.valid[i] ? state.input_voltage_v[i] : nan;
      msg.temperature_c[i] = state.valid[i] ? state.temperature_c[i] : nan;
      msg.hardware_error[i] = state.hardware_error[i];
      msg.hardware_error_valid[i] = state.hardware_error_valid[i];
      msg.hardware_error_age_s[i] = state.hardware_error_valid[i] ?
        state.hardware_error_age_s[i] : nan;
      msg.torque_enabled[i] = state.torque_enabled[i];
      msg.torque_enabled_valid[i] = state.torque_enabled_valid[i];
      msg.torque_state_age_s[i] = state.torque_enabled_valid[i] ?
        state.torque_state_age_s[i] : nan;
      msg.communication_result[i] = state.communication_result[i];
      joint.name[i] = msg.names[i];
      joint.position[i] = msg.position_rad[i];
      joint.velocity[i] = msg.velocity_rad_s[i];
    }
    telemetry_pub_->publish(msg);
    joint_pub_->publish(joint);
  }

  void publish_gait_locked(std::uint64_t now_ns) {
    fish_interfaces::msg::GaitState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "fish";
    msg.backend = accepted_config_.backend;
    msg.controller_state = to_string(state_);
    msg.accepted_config_revision = accepted_config_revision_;
    msg.active_config_revision = active_config_revision_;
    msg.pending_activation = accepted_config_revision_ != active_config_revision_;
    const auto validation = validate_config(accepted_config_);
    msg.configuration_valid = validation.valid;
    msg.configuration_error = validation.error;
    msg.waveform = accepted_config_.waveform;
    msg.amplitude_model = accepted_config_.amplitude_model;
    msg.frequency_hz = accepted_config_.frequency_hz;
    msg.tf_coeff = accepted_config_.tf_coeff;
    msg.body_coeff = accepted_config_.body_coeff;
    msg.model_unit = accepted_config_.model_unit;
    msg.fault_code = fault_code_;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      msg.x_model[i] = accepted_config_.x_model[i];
      msg.z_model[i] = accepted_config_.z_model[i];
      msg.hold_position_rad[i] = hold_position_[i];
    }
    gait_pub_->publish(msg);
    last_gait_publish_ns_ = now_ns;
    state_dirty_ = false;
  }

  void publish_diagnostics_locked() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fish/controller";
    status.hardware_id = bus_->backend();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (state_ == ControllerState::FAULT || state_ == ControllerState::ESTOP_LATCHED) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (!last_warning_.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    status.message = fault_code_.empty() ? to_string(state_) : fault_code_;
    add_diagnostic(status, "controller_state", to_string(state_));
    add_diagnostic(status, "backend", bus_->backend());
    add_diagnostic(status, "accepted_config_revision", std::to_string(accepted_config_revision_));
    add_diagnostic(status, "active_config_revision", std::to_string(active_config_revision_));
    add_diagnostic(status, "feedback_fresh", feedback_fresh_locked(steady_now_ns()) ? "true" : "false");
    add_diagnostic(status, "recorder_valid",
      recorder_valid_locked(steady_now_ns(), active_run_id_) ? "true" : "false");
    add_diagnostic(status, "warning", last_warning_);
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  static void add_diagnostic(
    diagnostic_msgs::msg::DiagnosticStatus& status,
    const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  std::mutex mutex_;
  Config accepted_config_;
  Config active_config_;
  std::uint64_t accepted_config_revision_{1};
  std::uint64_t active_config_revision_{0};
  ControllerState state_{ControllerState::BOOT};
  Request pending_request_{Request::NONE};
  std::unique_ptr<MotorBus> bus_;

  Array4d applied_position_{};
  Array4d applied_velocity_{};
  Array4d hold_position_{};
  Array4i goal_raw_{};
  WaveformSample last_sample_;
  std::uint64_t phase_start_ns_{0};
  std::string active_run_id_;
  double braking_elapsed_s_{0.0};
  double over_current_elapsed_s_{0.0};
  HoldConfirmation hold_confirmation_;
  std::string fault_code_;
  std::string last_warning_;
  bool state_dirty_{true};

  MotorState last_motor_state_;
  bool have_feedback_{false};
  std::uint64_t last_successful_write_ns_{0};
  BusResult last_goal_write_;
  fish_interfaces::msg::ExperimentState last_experiment_;
  bool have_experiment_{false};
  std::uint64_t last_experiment_ns_{0};

  std::uint64_t command_sequence_{0};
  std::uint64_t telemetry_sequence_{0};
  std::uint64_t last_control_ns_{0};
  std::uint64_t next_telemetry_ns_{0};
  std::uint64_t last_gait_publish_ns_{0};

  rclcpp::Publisher<fish_interfaces::msg::FlappingCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<fish_interfaces::msg::MotorTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<fish_interfaces::msg::GaitState>::SharedPtr gait_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<fish_interfaces::msg::ExperimentState>::SharedPtr experiment_sub_;
  rclcpp::Service<Trigger>::SharedPtr arm_service_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<Trigger>::SharedPtr disarm_service_;
  rclcpp::Service<Trigger>::SharedPtr estop_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace
}  // namespace fish_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<fish_control::ControllerNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("controller"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
