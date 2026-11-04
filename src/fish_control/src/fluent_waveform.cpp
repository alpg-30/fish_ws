#include "fish_control/fluent_waveform.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace fish_control {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
}

PhaseSample fluent_phase(double time_s, double frequency_hz, double tf_coeff) {
  if (!std::isfinite(time_s) || !std::isfinite(frequency_hz) || frequency_hz <= 0.0 ||
      !std::isfinite(tf_coeff) || tf_coeff <= 0.0 || tf_coeff >= 1.0) {
    throw std::invalid_argument("invalid Fluent phase input");
  }
  const double initial_rate = kPi * frequency_hz / tf_coeff;
  if (time_s <= 1e-12) {
    return {0.0, initial_rate};
  }
  const double period = 1.0 / frequency_hz;
  const double u = std::fmod(time_s, period) / period;
  const double b1 = tf_coeff / 2.0;
  const double b2 = 1.0 - tf_coeff / 2.0;
  if (u < b1) {
    return {kPi * u / tf_coeff, initial_rate};
  }
  if (u <= b2) {
    return {kPi / 2.0 + kPi * (u - b1) / (1.0 - tf_coeff),
            kPi * frequency_hz / (1.0 - tf_coeff)};
  }
  return {3.0 * kPi / 2.0 + kPi * (u - b2) / tf_coeff, initial_rate};
}

MappingResult map_model_to_joint(const Array4d& displacement, const Config& config) {
  MappingResult result;
  result.joint_target_rad.fill(std::numeric_limits<double>::quiet_NaN());
  if (!mapping_is_valid(config)) {
    return result;
  }
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!std::isfinite(displacement[i])) {
      return result;
    }
    result.joint_target_rad[i] =
      config.mapping_offset[i] + config.mapping_gain[i] * displacement[i];
  }
  result.valid = true;
  return result;
}

WaveformSample evaluate_model_waveform(
  double phase_rad, double phase_rate_rad_s, const Config& config) {
  if (!std::isfinite(phase_rad) || !std::isfinite(phase_rate_rad_s) ||
      config.x_model.size() != kMotorCount || config.z_model.size() != kMotorCount) {
    throw std::invalid_argument("invalid Fluent model input");
  }
  WaveformSample sample;
  sample.phase_rad = phase_rad;
  sample.phase_rate_rad_s = phase_rate_rad_s;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const double psi = config.x_model[i] * config.body_coeff * 2.0 * kPi / 0.55;
    const double wave = std::sin(psi - phase_rad);
    const double z = config.z_model[i];
    double shape = 0.0;
    if (z > 0.1) {
      shape = (z - 0.1) / 0.5;
    } else if (z < -0.1) {
      shape = (-0.1 - z) / 0.5;
    }
    sample.spatial_phase_rad[i] = psi;
    sample.wave_value[i] = wave;
    sample.model_displacement[i] = shape * config.amplitude_model * wave;
  }
  const auto mapping = map_model_to_joint(sample.model_displacement, config);
  sample.mapping_valid = mapping.valid;
  sample.ideal_joint_target_rad = mapping.joint_target_rad;
  return sample;
}

WaveformSample evaluate_fluent_waveform(double time_s, const Config& config) {
  const auto phase = fluent_phase(time_s, config.frequency_hz, config.tf_coeff);
  return evaluate_model_waveform(phase.phase_rad, phase.phase_rate_rad_s, config);
}

PhaseSample fluent_phase_periodic(double time_s, double frequency_hz, double tf_coeff) {
  if (!std::isfinite(time_s) || !std::isfinite(frequency_hz) || frequency_hz <= 0.0 ||
      !std::isfinite(tf_coeff) || tf_coeff <= 0.0 || tf_coeff >= 1.0) {
    throw std::invalid_argument("invalid Fluent phase input");
  }
  const double period = 1.0 / frequency_hz;
  double wrapped = std::fmod(time_s, period);
  if (wrapped < 0.0) {
    wrapped += period;
  }
  const double u = wrapped / period;
  const double b1 = tf_coeff / 2.0;
  const double b2 = 1.0 - tf_coeff / 2.0;
  const double fast_rate = kPi * frequency_hz / tf_coeff;
  if (u < b1) {
    return {kPi * u / tf_coeff, fast_rate};
  }
  if (u <= b2) {
    return {kPi / 2.0 + kPi * (u - b1) / (1.0 - tf_coeff),
            kPi * frequency_hz / (1.0 - tf_coeff)};
  }
  return {3.0 * kPi / 2.0 + kPi * (u - b2) / tf_coeff, fast_rate};
}

WaveformSample evaluate_delayed_waveform(double time_s, const Config& config) {
  if (!std::isfinite(time_s) || config.x_model.size() != kMotorCount ||
      config.z_model.size() != kMotorCount) {
    throw std::invalid_argument("invalid delayed waveform input");
  }
  const auto master = fluent_phase(time_s, config.frequency_hz, config.tf_coeff);
  WaveformSample sample;
  sample.phase_rad = master.phase_rad;
  sample.phase_rate_rad_s = master.phase_rate_rad_s;
  const double period = 1.0 / config.frequency_hz;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const double psi = config.x_model[i] * config.body_coeff * 2.0 * kPi / 0.55;
    // 空间相位差等价为时间延迟，使快慢段切换点落在该电机自己的极值处。
    const double delay_s = psi / (2.0 * kPi) * period;
    const double local_phase =
      fluent_phase_periodic(time_s - delay_s, config.frequency_hz, config.tf_coeff).phase_rad;
    const double wave = std::sin(-local_phase);
    const double z = config.z_model[i];
    double shape = 0.0;
    if (z > 0.1) {
      shape = (z - 0.1) / 0.5;
    } else if (z < -0.1) {
      shape = (-0.1 - z) / 0.5;
    }
    sample.spatial_phase_rad[i] = psi;
    sample.wave_value[i] = wave;
    sample.model_displacement[i] = shape * config.amplitude_model * wave;
  }
  const auto mapping = map_model_to_joint(sample.model_displacement, config);
  sample.mapping_valid = mapping.valid;
  sample.ideal_joint_target_rad = mapping.joint_target_rad;
  return sample;
}

WaveformSample evaluate_waveform(double time_s, const Config& config) {
  if (config.waveform == "fluent_asymmetric_delayed") {
    return evaluate_delayed_waveform(time_s, config);
  }
  return evaluate_fluent_waveform(time_s, config);
}

}  // namespace fish_control
