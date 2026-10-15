// SPDX-License-Identifier: MIT
#pragma once

namespace drone_sim {

struct BatteryConfig {
  double percent{100.0};
  double hover_drain_pct_per_s{0.02};
  // Extra drain added once per m/s of speed.
  double move_drain_pct_per_s{0.05};
  double low_percent{25.0};
  double critical_percent{10.0};
};

// Percent budget with a hover term plus a term proportional to speed.
// This is not a cell, current, or propulsion model.
class BatteryModel {
 public:
  explicit BatteryModel(BatteryConfig config = {});

  void update(double speed_m_s, double dt);

  double percent() const { return percent_; }
  double last_drain_pct_per_s() const { return last_drain_pct_per_s_; }
  bool low() const { return percent_ <= config_.low_percent; }
  bool critical() const { return percent_ <= config_.critical_percent; }
  const BatteryConfig& config() const { return config_; }

 private:
  BatteryConfig config_;
  double percent_;
  double last_drain_pct_per_s_{0.0};
};

}  // namespace drone_sim
