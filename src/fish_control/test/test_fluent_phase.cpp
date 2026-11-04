#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fish_control/fluent_waveform.hpp"

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[FAIL] " << message << '\n';
    std::exit(1);
  }
}

double independent_oracle(double t, double f, double tf) {
  if (t <= 1e-12) {
    return 0.0;
  }
  const double u = std::fmod(t, 1.0 / f) * f;
  const double left = tf * 0.5;
  const double right = 1.0 - left;
  if (u < left) {
    return kPi * u / tf;
  }
  if (u > right) {
    return 1.5 * kPi + kPi * (u - right) / tf;
  }
  return 0.5 * kPi + kPi * (u - left) / (1.0 - tf);
}

void check_near(double actual, double expected, const char* message) {
  if (std::abs(actual - expected) > 1e-10) {
    std::cerr << std::setprecision(17) << "[FAIL] " << message << ": " << actual
              << " != " << expected << '\n';
    std::exit(1);
  }
}

// 延迟模式：每个电机的快慢段切换点必须落在它自己的波峰/波谷，且下扑占比等于 tf_coeff。
void check_delayed_mode() {
  fish_control::Config config;
  config.waveform = "fluent_asymmetric_delayed";
  config.amplitude_model = 0.2;
  config.frequency_hz = 0.4;
  config.tf_coeff = 0.65;
  config.body_coeff = 0.5;
  config.x_model = {0.0, 0.12, 0.24, 0.36};
  config.z_model = {0.20, -0.20, 0.25, -0.25};

  const double period = 1.0 / config.frequency_hz;
  const double t1 = period * config.tf_coeff / 2.0;
  const double t2 = period * (1.0 - config.tf_coeff / 2.0);
  const int samples_per_cycle = 8000;
  const double dt = period / samples_per_cycle;

  for (std::size_t i = 0; i < fish_control::kMotorCount; ++i) {
    const double psi = config.x_model[i] * config.body_coeff * 2.0 * kPi / 0.55;
    const double delay = psi / (2.0 * kPi) * period;
    std::vector<double> values(2 * samples_per_cycle + 1);
    for (int step = 0; step <= 2 * samples_per_cycle; ++step) {
      values[step] = fish_control::evaluate_waveform(step * dt, config)
        .model_displacement[i];
    }
    std::vector<double> extrema;
    for (int step = 1; step < 2 * samples_per_cycle; ++step) {
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
        std::cerr << "[FAIL] motor " << i
                  << " rate switch is not at its own peak/trough\n";
        std::exit(1);
      }
    }

    int down = 0;
    int up = 0;
    for (int step = 1; step <= 2 * samples_per_cycle; ++step) {
      if (values[step] < values[step - 1]) {
        ++down;
      } else if (values[step] > values[step - 1]) {
        ++up;
      }
    }
    const double fraction = static_cast<double>(down) / static_cast<double>(down + up);
    if (std::abs(fraction - config.tf_coeff) >= 0.01) {
      std::cerr << "[FAIL] motor " << i << " downstroke fraction " << fraction
                << " != tf_coeff " << config.tf_coeff << '\n';
      std::exit(1);
    }
  }
}
}  // namespace

int main() {
  using fish_control::fluent_phase;
  const std::vector<std::pair<double, double>> analytic{
    {0.0, 0.0}, {0.0875, kPi / 4.0}, {0.175, kPi / 2.0}, {0.5, kPi},
    {0.825, 3.0 * kPi / 2.0}, {0.9125, 7.0 * kPi / 4.0}, {1.0, 0.0}};
  for (const auto& sample : analytic) {
    check_near(fluent_phase(sample.first, 1.0, 0.35).phase_rad, sample.second,
               "hard-coded Fluent phase sample");
  }

  for (double f : {0.7, 1.0, 2.3}) {
    for (double tf : {0.2, 0.35, 0.8}) {
      const double period = 1.0 / f;
      const double b1 = tf * 0.5 * period;
      const double b2 = (1.0 - tf * 0.5) * period;
      for (int cycle = 0; cycle < 5; ++cycle) {
        const double offset = cycle * period;
        for (double t : {b1 - 1e-8, b1, b1 + 1e-8, 0.5 * period,
                         b2 - 1e-8, b2, b2 + 1e-8}) {
          const double sample_t = offset + t;
          check_near(fluent_phase(sample_t, f, tf).phase_rad,
                     independent_oracle(sample_t, f, tf),
                     "independent boundary/multi-cycle oracle");
        }
      }
      check_near(fluent_phase(b1, f, tf).phase_rad, kPi / 2.0, "left boundary phi");
      check_near(fluent_phase(b2, f, tf).phase_rad, 3.0 * kPi / 2.0,
                 "right boundary phi");
      check_near(fluent_phase(0.0, f, tf).phase_rate_rad_s, kPi * f / tf,
                 "initial phase rate");
      check_near(fluent_phase(b1, f, tf).phase_rate_rad_s, kPi * f / (1.0 - tf),
                 "middle phase rate");
      check_near(fluent_phase(b2 + 1e-8, f, tf).phase_rate_rad_s, kPi * f / tf,
                 "tail phase rate");
    }
  }
  check(fluent_phase(1e-12, 1.0, 0.35).phase_rad == 0.0, "zero-time special case");
  check(fluent_phase(-1.0, 1.0, 0.35).phase_rad == 0.0, "t <= 1e-12 special case");
  check_delayed_mode();
  std::cout << "[PASS] Fluent phase analytic, boundary, rate and multi-cycle tests\n";
  std::cout << "[PASS] delayed-mode switches at each motor's own extrema\n";
  return 0;
}
