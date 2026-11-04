// 用 fish_control 的真实 C++ 波形实现导出 Fluent 参数组合的波形数据。
// 仅依赖纯逻辑源码，不需要 ROS、SDK 或 GPU。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "fish_control/config.hpp"
#include "fish_control/fluent_waveform.hpp"

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr int kSamplesPerCycle = 800;
constexpr int kCycles = 2;

struct Combo {
  const char* tag;
  double amplitude_model;
  double frequency_hz;
  double body_coeff;
  double tf_coeff;
  const char* waveform;
  const char* source;
};

// Fluent 组来自 01_sim/data 中已实际跑过的参数组合。
const std::vector<Combo> kCombos{
  {"data1_baseline", 0.20, 0.31830989, 0.50, 0.35, "fluent_asymmetric",
   "var.txt / data1_soft.csv"},
  {"data2_low_amp_low_phi", 0.15, 0.25, 0.35, 0.45, "fluent_asymmetric",
   "data2_soft.csv"},
  {"data21_high_freq", 0.21, 0.35, 0.35, 0.35, "fluent_asymmetric",
   "data21_soft.csv"},
  {"data22_low_duty", 0.22, 0.30, 0.45, 0.30, "fluent_asymmetric",
   "data22_soft.csv"},
  {"data16_high_duty", 0.25, 0.27, 0.38, 0.50, "fluent_asymmetric",
   "data16_soft.csv"},
  {"data10_high_amp", 0.35, 0.31, 0.40, 0.41, "fluent_asymmetric",
   "data10_soft.csv"},
  // 机器人目标工况：下扑慢、上抬快，且每个电机在自己的波峰/波谷换向。
  {"down_slow_delayed_duty065", 0.20, 0.31830989, 0.50, 0.65,
   "fluent_asymmetric_delayed", "derived from data1; robot target"},
  // 同一参数、全局相位模式，用于对比换向点位置。
  {"down_slow_global_duty065", 0.20, 0.31830989, 0.50, 0.65,
   "fluent_asymmetric", "derived from data1; global phase reference"},
};

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "[FAIL] " << message << '\n';
    std::exit(1);
  }
}

// 与 plan 中给出的 mock 合成安装点和 mock_linear 示例映射保持一致。
fish_control::Config make_config(const Combo& combo) {
  fish_control::Config config;
  config.backend = "mock";
  config.mapping_kind = "mock_linear";
  config.model_unit = "fluent_model_unit";
  config.waveform = combo.waveform;
  config.amplitude_model = combo.amplitude_model;
  config.frequency_hz = combo.frequency_hz;
  config.body_coeff = combo.body_coeff;
  config.tf_coeff = combo.tf_coeff;
  config.x_model = {0.0, 0.12, 0.24, 0.36};
  config.z_model = {0.20, -0.20, 0.25, -0.25};
  config.mapping_gain = {1.0, 1.0, 1.0, 1.0};
  config.mapping_offset = {0.0, 0.0, 0.0, 0.0};
  return config;
}

double shape(std::size_t index, const fish_control::Config& config) {
  const double z = config.z_model[index];
  if (z > 0.1) {
    return (z - 0.1) / 0.5;
  }
  if (z < -0.1) {
    return (-0.1 - z) / 0.5;
  }
  return 0.0;
}

double spatial_delay(std::size_t index, const fish_control::Config& config) {
  const double psi = config.x_model[index] * config.body_coeff * 2.0 * kPi / 0.55;
  return psi / (2.0 * kPi) / config.frequency_hz;
}

void write_waveform_csv(const std::filesystem::path& path, const Combo& combo,
                        const fish_control::Config& config) {
  std::ofstream out(path);
  require(out.good(), "cannot write " + path.string());
  out << std::setprecision(12);
  out << "# tag=" << combo.tag << " waveform=" << combo.waveform
      << " amplitude_model=" << combo.amplitude_model
      << " frequency_hz=" << combo.frequency_hz << " body_coeff=" << combo.body_coeff
      << " tf_coeff=" << combo.tf_coeff << '\n';
  out << "# source=" << combo.source
      << " geometry=x[0,0.12,0.24,0.36] z[0.20,-0.20,0.25,-0.25] mapping=mock_linear(gain=1,offset=0)\n";
  out << "time_s,phase_rad,phase_rate_rad_s";
  for (int i = 0; i < 4; ++i) out << ",displacement_" << i;
  for (int i = 0; i < 4; ++i) out << ",wave_" << i;
  for (int i = 0; i < 4; ++i) out << ",ideal_joint_" << i;
  out << '\n';

  const double period = 1.0 / combo.frequency_hz;
  const int total = kSamplesPerCycle * kCycles + 1;
  for (int step = 0; step < total; ++step) {
    const double time_s = period * kCycles * step / (total - 1);
    const auto sample = fish_control::evaluate_waveform(time_s, config);
    out << time_s << ',' << sample.phase_rad << ',' << sample.phase_rate_rad_s;
    for (double value : sample.model_displacement) out << ',' << value;
    for (double value : sample.wave_value) out << ',' << value;
    for (double value : sample.ideal_joint_target_rad) out << ',' << value;
    out << '\n';
  }
}

// 用采样位移统计某个电机的下扑时间占比。
double downstroke_fraction(const fish_control::Config& config, std::size_t index) {
  const double period = 1.0 / config.frequency_hz;
  const double dt = period / kSamplesPerCycle;
  int down = 0;
  int up = 0;
  double previous = fish_control::evaluate_waveform(0.0, config).model_displacement[index];
  for (int step = 1; step <= kSamplesPerCycle * kCycles; ++step) {
    const auto sample = fish_control::evaluate_waveform(step * dt, config);
    const double current = sample.model_displacement[index];
    if (current < previous) {
      ++down;
    } else if (current > previous) {
      ++up;
    }
    previous = current;
  }
  return static_cast<double>(down) / static_cast<double>(down + up);
}

// 检查极值时间与理论换向时间（t1/t2 加该电机延迟）是否重合。
bool switches_at_extrema(const fish_control::Config& config, std::size_t index) {
  const double period = 1.0 / config.frequency_hz;
  const double t1 = period * config.tf_coeff / 2.0;
  const double t2 = period * (1.0 - config.tf_coeff / 2.0);
  const double delay = spatial_delay(index, config);
  const int samples = kSamplesPerCycle * kCycles;
  const double dt = period / kSamplesPerCycle;
  std::vector<double> values(samples + 1);
  for (int step = 0; step <= samples; ++step) {
    values[step] = fish_control::evaluate_waveform(step * dt, config)
      .model_displacement[index];
  }
  std::vector<double> extrema;
  for (int step = 1; step < samples; ++step) {
    if ((values[step] - values[step - 1]) * (values[step + 1] - values[step]) < 0.0) {
      extrema.push_back(step * dt);
    }
  }
  for (double base : {t1, t2}) {
    double expected = std::fmod(base + delay, period);
    if (expected < 0.0) {
      expected += period;
    }
    bool found = false;
    for (double candidate : extrema) {
      double distance = std::abs(candidate - expected);
      distance = std::min(distance, std::abs(distance - period));
      if (distance < 5.0 * dt) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

void check_combo(const Combo& combo, const fish_control::Config& config) {
  const double period = 1.0 / combo.frequency_hz;
  const double t1 = period * combo.tf_coeff / 2.0;
  const double t2 = period * (1.0 - combo.tf_coeff / 2.0);
  const double fast_rate = kPi * combo.frequency_hz / combo.tf_coeff;
  const double slow_rate = kPi * combo.frequency_hz / (1.0 - combo.tf_coeff);

  const auto at_t1 = fish_control::evaluate_waveform(t1, config);
  const auto at_t2 = fish_control::evaluate_waveform(t2, config);
  const auto at_period = fish_control::evaluate_waveform(period, config);
  require(std::abs(at_t1.phase_rad - kPi / 2.0) < 1e-9, std::string(combo.tag) + " t1 phase");
  require(std::abs(at_t2.phase_rad - 3.0 * kPi / 2.0) < 1e-9, std::string(combo.tag) + " t2 phase");
  require(at_period.phase_rad == 0.0, std::string(combo.tag) + " period wrap");

  const auto early = fish_control::evaluate_waveform(t1 * 0.5, config);
  const auto middle = fish_control::evaluate_waveform((t1 + t2) * 0.5, config);
  const auto late = fish_control::evaluate_waveform((t2 + period) * 0.5, config);
  require(std::abs(early.phase_rate_rad_s - fast_rate) < 1e-9, std::string(combo.tag) + " fast rate");
  require(std::abs(middle.phase_rate_rad_s - slow_rate) < 1e-9, std::string(combo.tag) + " slow rate");
  require(std::abs(late.phase_rate_rad_s - fast_rate) < 1e-9, std::string(combo.tag) + " tail rate");

  for (std::size_t i = 0; i < fish_control::kMotorCount; ++i) {
    const double envelope = std::abs(shape(i, config) * combo.amplitude_model);
    double peak = 0.0;
    for (int step = 0; step <= kSamplesPerCycle * kCycles; ++step) {
      const double time_s = period * kCycles * step / (kSamplesPerCycle * kCycles);
      const auto sample = fish_control::evaluate_waveform(time_s, config);
      peak = std::max(peak, std::abs(sample.model_displacement[i]));
    }
    require(peak <= envelope + 1e-12, std::string(combo.tag) + " envelope exceeded");
    require(envelope - peak < 0.01 * envelope + 1e-6, std::string(combo.tag) + " peak not sampled");
  }

  const bool delayed = std::string(combo.waveform) == "fluent_asymmetric_delayed";
  const std::size_t checked = delayed ? fish_control::kMotorCount : 1;
  for (std::size_t i = 0; i < checked; ++i) {
    require(std::abs(downstroke_fraction(config, i) - combo.tf_coeff) < 0.01,
            std::string(combo.tag) + " downstroke fraction must equal tf_coeff");
    if (delayed) {
      require(switches_at_extrema(config, i),
              std::string(combo.tag) + " rate switch must sit at each motor's extrema");
    }
  }
}

std::string summary_header() {
  std::ostringstream line;
  line << "tag,amplitude_model,frequency_hz,body_coeff,tf_coeff,waveform,period_s,t1_s,t2_s,"
          "fast_rate_rad_s,slow_rate_rad_s,measured_downstroke_fraction,"
          "delay_0,delay_1,delay_2,delay_3,peak_0,peak_1,peak_2,peak_3,source";
  return line.str();
}
}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path output_dir =
    argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path("output");
  std::filesystem::create_directories(output_dir);

  std::ofstream summary(output_dir / "summary.csv");
  require(summary.good(), "cannot write summary.csv");
  summary << summary_header() << '\n';
  summary << std::setprecision(10);

  for (const auto& combo : kCombos) {
    const auto config = make_config(combo);
    check_combo(combo, config);
    write_waveform_csv(output_dir / (std::string(combo.tag) + ".csv"), combo, config);

    const double period = 1.0 / combo.frequency_hz;
    summary << combo.tag << ',' << combo.amplitude_model << ',' << combo.frequency_hz << ','
            << combo.body_coeff << ',' << combo.tf_coeff << ',' << combo.waveform << ','
            << period << ',' << period * combo.tf_coeff / 2.0 << ','
            << period * (1.0 - combo.tf_coeff / 2.0) << ','
            << kPi * combo.frequency_hz / combo.tf_coeff << ','
            << kPi * combo.frequency_hz / (1.0 - combo.tf_coeff) << ','
            << downstroke_fraction(config, 0);
    for (std::size_t i = 0; i < fish_control::kMotorCount; ++i) {
      summary << ',' << spatial_delay(i, config);
    }
    for (std::size_t i = 0; i < fish_control::kMotorCount; ++i) {
      summary << ',' << std::abs(shape(i, config) * combo.amplitude_model);
    }
    summary << ',' << combo.source << '\n';
    std::cout << "[PASS] " << combo.tag << " phase/rate/envelope/duty checks\n";
  }
  std::cout << "[PASS] wrote " << kCombos.size() << " waveform csv files to "
            << output_dir << '\n';
  return 0;
}
