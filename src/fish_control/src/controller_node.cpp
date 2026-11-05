#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <fish_interfaces/ExperimentState.h>
#include <fish_interfaces/FlappingCommand.h>
#include <fish_interfaces/GaitState.h>
#include <fish_interfaces/MotorTelemetry.h>
#include <fish_interfaces/SetGait.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include "fish_control/config.hpp"
#include "fish_control/fluent_waveform.hpp"
#include "fish_control/motor_bus.hpp"
#include "fish_control/safety.hpp"

namespace fish_control {
namespace {
using Trigger = std_srvs::Trigger;
constexpr double kRecorderTimeoutS = 1.0;
constexpr double kBrakingTimeoutS = 5.0;

std::uint64_t steady_now_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool assign_vector(const ros::NodeHandle& pnh, const std::string& key,
                   std::vector<double>& target) {
  std::vector<double> values = target;
  if (!pnh.getParam(key, values)) {
    return false;
  }
  target = values;
  return true;
}

bool assign_vector(const ros::NodeHandle& pnh, const std::string& key,
                   std::vector<std::int64_t>& target) {
  std::vector<int> values;
  if (!pnh.getParam(key, values)) {
    return false;
  }
  target.assign(values.begin(), values.end());
  return true;
}

class ControllerNode {
public:
  ControllerNode() {
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

    command_pub_ = nh_.advertise<fish_interfaces::FlappingCommand>("flapping_command", 50);
    telemetry_pub_ = nh_.advertise<fish_interfaces::MotorTelemetry>("motor_telemetry", 50);
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    gait_pub_ = nh_.advertise<fish_interfaces::GaitState>("gait_state", 1, true);
    diagnostics_pub_ =
      nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
    experiment_sub_ = nh_.subscribe(
      "experiment_state", 1, &ControllerNode::onExperimentState, this);

    arm_service_ = nh_.advertiseService("controller/arm", &ControllerNode::onArm, this);
    start_service_ = nh_.advertiseService("controller/start", &ControllerNode::onStart, this);
    stop_service_ =
      nh_.advertiseService("controller/stop_hold", &ControllerNode::onStopHold, this);
    disarm_service_ =
      nh_.advertiseService("controller/disarm", &ControllerNode::onDisarm, this);
    estop_service_ = nh_.advertiseService("controller/estop", &ControllerNode::onEstop, this);
    reset_service_ =
      nh_.advertiseService("controller/reset_fault", &ControllerNode::onResetFault, this);
    set_gait_service_ =
      nh_.advertiseService("controller/set_gait", &ControllerNode::onSetGait, this);

    const auto control_period = ros::Duration(1.0 / accepted_config_.control_hz);
    control_timer_ = nh_.createTimer(control_period, &ControllerNode::controlTick, this);
    last_control_ns_ = steady_now_ns();
    next_telemetry_ns_ = last_control_ns_;
    last_gait_publish_ns_ = 0;
    ROS_INFO("controller initialized with backend=%s", bus_->backend().c_str());
  }

  ~ControllerNode() {
    if (bus_) {
      bus_->close();
    }
  }

private:
  enum class Request { NONE, ARM, START, STOP_HOLD, DISARM, ESTOP, RESET_FAULT };

  void onExperimentState(const fish_interfaces::ExperimentState::ConstPtr& message) {
    last_experiment_ = *message;
    last_experiment_ns_ = steady_now_ns();
    have_experiment_ = true;
  }

  bool onArm(Trigger::Request&, Trigger::Response& response) {
    return queue_request(Request::ARM, response);
  }
  bool onStart(Trigger::Request&, Trigger::Response& response) {
    return queue_request(Request::START, response);
  }
  bool onStopHold(Trigger::Request&, Trigger::Response& response) {
    return queue_request(Request::STOP_HOLD, response);
  }
  bool onDisarm(Trigger::Request&, Trigger::Response& response) {
    return queue_request(Request::DISARM, response);
  }
  bool onEstop(Trigger::Request&, Trigger::Response& response) {
    return queue_request(Request::ESTOP, response);
  }
  bool onResetFault(Trigger::Request&, Trigger::Response& response) {
    return queue_request(Request::RESET_FAULT, response);
  }

  bool onSetGait(fish_interfaces::SetGait::Request& request,
                 fish_interfaces::SetGait::Response& response) {
    Config candidate = accepted_config_;
    candidate.amplitude_model = request.amplitude_model;
    candidate.frequency_hz = request.frequency_hz;
    candidate.tf_coeff = request.tf_coeff;
    candidate.body_coeff = request.body_coeff;
    const std::vector<std::string> changed = {
      "gait.amplitude_model", "gait.frequency_hz", "gait.tf_coeff", "gait.body_coeff"};
    const auto validation =
      validate_config_update(accepted_config_, candidate, state_, changed);
    if (!validation.valid) {
      response.success = false;
      response.message = validation.error;
      response.accepted_config_revision = accepted_config_revision_;
      ROS_WARN("set_gait rejected: %s", validation.error.c_str());
      return true;
    }
    accepted_config_ = candidate;
    ++accepted_config_revision_;
    state_dirty_ = true;
    publish_gait_locked(steady_now_ns());
    response.success = true;
    response.message = "accepted";
    response.accepted_config_revision = accepted_config_revision_;
    return true;
  }

  Config read_config() const {
    Config c;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("backend", c.backend, c.backend);
    pnh.param<double>("timing/control_hz", c.control_hz, c.control_hz);
    pnh.param<double>("timing/telemetry_hz", c.telemetry_hz, c.telemetry_hz);

    std::vector<std::string> names = c.motor_names;
    if (pnh.getParam("motors/names", names)) {
      c.motor_names = names;
    }
    assign_vector(pnh, "motors/ids", c.motor_ids);

    pnh.param<std::string>("gait/waveform", c.waveform, c.waveform);
    pnh.param<double>("gait/amplitude_model", c.amplitude_model, c.amplitude_model);
    pnh.param<double>("gait/frequency_hz", c.frequency_hz, c.frequency_hz);
    pnh.param<double>("gait/tf_coeff", c.tf_coeff, c.tf_coeff);
    pnh.param<double>("gait/body_coeff", c.body_coeff, c.body_coeff);

    assign_vector(pnh, "geometry/x_model", c.x_model);
    assign_vector(pnh, "geometry/z_model", c.z_model);
    pnh.param<std::string>("geometry/model_unit", c.model_unit, c.model_unit);

    pnh.param<std::string>("mapping/kind", c.mapping_kind, c.mapping_kind);
    assign_vector(pnh, "mapping/gain_rad_per_model_unit", c.mapping_gain);
    assign_vector(pnh, "mapping/offset_rad", c.mapping_offset);

    assign_vector(pnh, "calibration/zero_raw", c.zero_raw);
    assign_vector(pnh, "calibration/direction", c.direction);
    assign_vector(pnh, "calibration/external_ratio", c.external_ratio);
    pnh.param<bool>("calibration/confirmed", c.calibration_confirmed,
                    c.calibration_confirmed);

    assign_vector(pnh, "limits/min_rad", c.min_rad);
    assign_vector(pnh, "limits/max_rad", c.max_rad);
    assign_vector(pnh, "limits/max_velocity_rad_s", c.max_velocity_rad_s);
    assign_vector(pnh, "limits/max_acceleration_rad_s2", c.max_acceleration_rad_s2);
    assign_vector(pnh, "limits/current_trip_a", c.current_trip_a);
    pnh.param<double>("limits/current_trip_duration_s", c.current_trip_duration_s,
                      c.current_trip_duration_s);
    assign_vector(pnh, "limits/temperature_trip_c", c.temperature_trip_c);
    assign_vector(pnh, "limits/min_voltage_v", c.min_voltage_v);
    assign_vector(pnh, "limits/max_voltage_v", c.max_voltage_v);

    pnh.param<double>("safety/feedback_timeout_s", c.feedback_timeout_s,
                      c.feedback_timeout_s);
    int watchdog_ms = static_cast<int>(c.bus_watchdog_ms);
    pnh.param<int>("safety/bus_watchdog_ms", watchdog_ms, watchdog_ms);
    c.bus_watchdog_ms = watchdog_ms;
    pnh.param<std::string>("safety/normal_stop_policy", c.normal_stop_policy,
                           c.normal_stop_policy);
    pnh.param<bool>("safety/require_recording", c.require_recording, c.require_recording);

    pnh.param<std::string>("bus/device", c.bus_device, c.bus_device);
    int baudrate = static_cast<int>(c.bus_baudrate);
    pnh.param<int>("bus/baudrate", baudrate, baudrate);
    c.bus_baudrate = baudrate;
    pnh.param<double>("bus/protocol_version", c.bus_protocol_version,
                      c.bus_protocol_version);
    return c;
  }

  bool queue_request(Request request, Trigger::Response& response) {
    if (request == Request::ESTOP) {
      if (state_ == ControllerState::ESTOP_LATCHED) {
        response.success = true;
        response.message = "already ESTOP_LATCHED";
        return true;
      }
      pending_request_ = request;
      response.success = true;
      response.message = "estop queued";
      return true;
    }
    if (pending_request_ != Request::NONE) {
      response.success = false;
      response.message = "another controller request is pending";
      return true;
    }
    bool allowed = false;
    std::string message;
    switch (request) {
      case Request::ARM:
        allowed = state_ == ControllerState::DISARMED && mapping_is_valid(accepted_config_);
        message = allowed ? "arm queued" : "arm requires DISARMED and valid mapping";
        break;
      case Request::START: {
        response.success = start_locked(steady_now_ns());
        response.message = response.success ? "RUNNING" :
          "start requires HOLDING, fresh recorder_ready session, positive frequency and valid mapping";
        if (response.success) {
          publish_gait_locked(steady_now_ns());
        }
        return true;
      }
      case Request::STOP_HOLD:
        if (state_ == ControllerState::HOLDING || state_ == ControllerState::BRAKING) {
          response.success = true;
          response.message = to_string(state_);
          return true;
        }
        allowed = state_ == ControllerState::RUNNING;
        message = allowed ? "stop_hold queued" : "stop_hold requires RUNNING";
        break;
      case Request::DISARM:
        if (state_ == ControllerState::DISARMED) {
          response.success = true;
          response.message = "already DISARMED";
          return true;
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
    response.success = allowed;
    response.message = message;
    return true;
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

  void controlTick(const ros::TimerEvent&) {
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
        ROS_WARN("%s", last_warning_.c_str());
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
    ROS_ERROR("FAULT latched: %s", code.c_str());
    set_state_locked(ControllerState::FAULT);
  }

  void enter_estop_locked() {
    fault_code_ = "software_estop";
    pending_request_ = Request::NONE;
    active_run_id_.clear();
    applied_velocity_.fill(0.0);
    bus_->set_torque(false);
    ROS_ERROR("software ESTOP latched");
    set_state_locked(ControllerState::ESTOP_LATCHED);
  }

  void set_state_locked(ControllerState state) {
    if (state_ != state) {
      state_ = state;
      state_dirty_ = true;
    }
  }

  void publish_command_locked(std::uint64_t now_ns) {
    fish_interfaces::FlappingCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "fish";
    msg.sequence = ++command_sequence_;
    msg.host_steady_time_ns = now_ns;
    msg.backend = accepted_config_.backend;
    msg.active_config_revision = active_config_revision_;
    const Config& waveform_config =
      active_config_revision_ == 0 ? accepted_config_ : active_config_;
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
    command_pub_.publish(msg);
  }

  void publish_telemetry_locked() {
    const auto& state = last_motor_state_;
    fish_interfaces::MotorTelemetry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "fish";
    msg.sequence = ++telemetry_sequence_;
    msg.host_steady_time_ns = state.host_steady_time_ns;
    msg.backend = bus_->backend();
    msg.active_config_revision = active_config_revision_;
    msg.acquisition_duration_s = state.acquisition_duration_s;
    sensor_msgs::JointState joint;
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
    telemetry_pub_.publish(msg);
    joint_pub_.publish(joint);
  }

  void publish_gait_locked(std::uint64_t now_ns) {
    fish_interfaces::GaitState msg;
    msg.header.stamp = ros::Time::now();
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
    gait_pub_.publish(msg);
    last_gait_publish_ns_ = now_ns;
    state_dirty_ = false;
  }

  void publish_diagnostics_locked() {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "fish/controller";
    status.hardware_id = bus_->backend();
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    if (state_ == ControllerState::FAULT || state_ == ControllerState::ESTOP_LATCHED) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    } else if (!last_warning_.empty()) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    status.message = fault_code_.empty() ? to_string(state_) : fault_code_;
    add_diagnostic(status, "controller_state", to_string(state_));
    add_diagnostic(status, "backend", bus_->backend());
    add_diagnostic(status, "accepted_config_revision",
                   std::to_string(accepted_config_revision_));
    add_diagnostic(status, "active_config_revision",
                   std::to_string(active_config_revision_));
    add_diagnostic(status, "feedback_fresh",
                   feedback_fresh_locked(steady_now_ns()) ? "true" : "false");
    add_diagnostic(status, "recorder_valid",
      recorder_valid_locked(steady_now_ns(), active_run_id_) ? "true" : "false");
    add_diagnostic(status, "warning", last_warning_);
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  static void add_diagnostic(
    diagnostic_msgs::DiagnosticStatus& status,
    const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  ros::NodeHandle nh_;
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
  fish_interfaces::ExperimentState last_experiment_;
  bool have_experiment_{false};
  std::uint64_t last_experiment_ns_{0};

  std::uint64_t command_sequence_{0};
  std::uint64_t telemetry_sequence_{0};
  std::uint64_t last_control_ns_{0};
  std::uint64_t next_telemetry_ns_{0};
  std::uint64_t last_gait_publish_ns_{0};

  ros::Publisher command_pub_;
  ros::Publisher telemetry_pub_;
  ros::Publisher joint_pub_;
  ros::Publisher gait_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Subscriber experiment_sub_;
  ros::ServiceServer arm_service_;
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  ros::ServiceServer disarm_service_;
  ros::ServiceServer estop_service_;
  ros::ServiceServer reset_service_;
  ros::ServiceServer set_gait_service_;
  ros::Timer control_timer_;
};

}  // namespace
}  // namespace fish_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  try {
    fish_control::ControllerNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("controller startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
