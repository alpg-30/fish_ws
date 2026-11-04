#pragma once

#include "fish_control/config.hpp"

namespace fish_control {

struct PhaseSample {
  double phase_rad{0.0};
  double phase_rate_rad_s{0.0};
};

struct MappingResult {
  bool valid{false};
  Array4d joint_target_rad{};
};

struct WaveformSample {
  double phase_rad{0.0};
  double phase_rate_rad_s{0.0};
  Array4d spatial_phase_rad{};
  Array4d wave_value{};
  Array4d model_displacement{};
  bool mapping_valid{false};
  Array4d ideal_joint_target_rad{};
};

PhaseSample fluent_phase(double time_s, double frequency_hz, double tf_coeff);
// 周期延拓版本：支持负时间，用于按空间相位做时间延迟。
PhaseSample fluent_phase_periodic(double time_s, double frequency_hz, double tf_coeff);
MappingResult map_model_to_joint(const Array4d& displacement, const Config& config);
WaveformSample evaluate_model_waveform(
  double phase_rad, double phase_rate_rad_s, const Config& config);
WaveformSample evaluate_fluent_waveform(double time_s, const Config& config);
// 快慢段切换点落在每个电机各自的波峰/波谷（空间相位按时间延迟实现）。
WaveformSample evaluate_delayed_waveform(double time_s, const Config& config);
// 按 config.waveform 选择实现。
WaveformSample evaluate_waveform(double time_s, const Config& config);

}  // namespace fish_control
