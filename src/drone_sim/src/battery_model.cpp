// SPDX-License-Identifier: MIT
#include "drone_sim/battery_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_sim {

BatteryModel::BatteryModel(BatteryConfig config) : config_(config), percent_(config.percent) {
  if (config_.hover_drain_pct_per_s < 0.0 || config_.move_drain_pct_per_s < 0.0) {
    throw std::invalid_argument("battery drain must be non-negative");
  }
  if (config_.critical_percent > config_.low_percent) {
    throw std::invalid_argument("critical battery threshold must be <= low threshold");
  }
  percent_ = std::clamp(percent_, 0.0, 100.0);
}

void BatteryModel::update(double speed_m_s, double dt) {
  if (!std::isfinite(speed_m_s) || speed_m_s < 0.0) {
    throw std::invalid_argument("speed must be finite and non-negative");
  }
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("dt must be finite and non-negative");
  }
  last_drain_pct_per_s_ =
      config_.hover_drain_pct_per_s + config_.move_drain_pct_per_s * speed_m_s;
  percent_ = std::clamp(percent_ - last_drain_pct_per_s_ * dt, 0.0, 100.0);
}

}  // namespace drone_sim
