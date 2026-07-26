#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace sonic::safety {

constexpr std::size_t kJointCount = 29;
constexpr std::size_t kMaxWindowSamples = 250;

enum class JointState { kNormal, kRateLimited, kBraking, kJointFault };

enum class JointFaultReason {
  kNone,
  kInvalidMeasuredPosition,
  kInvalidMeasuredVelocity,
  kInvalidDesiredPosition,
  kMeasuredOverspeed,
  kEmergencyTrackingCorrection,
};

struct Limits {
  std::array<double, kJointCount> min_position{};
  std::array<double, kJointCount> max_position{};
  std::array<double, kJointCount> max_instant_velocity{};
  std::array<double, kJointCount> max_acceleration{};
  std::array<double, kJointCount> max_window_velocity{};
  std::array<double, kJointCount> max_tracking_error{};
  std::array<double, kJointCount> measured_velocity_brake{};
  std::array<double, kJointCount> measured_velocity_fault{};
  std::array<double, kJointCount> brake_target_error{};
  double writer_dt = 0.002;
  double window_duration = 0.100;
  double normal_window_fraction = 0.75;
  int overspeed_samples_to_fault = 3;
};

struct JointStats {
  std::uint64_t ticks = 0;
  std::uint64_t rate_limited_ticks = 0;
  std::uint64_t braking_ticks = 0;
  std::uint64_t lifecycle_hold_ticks = 0;
  std::uint64_t fault_ticks = 0;
  std::uint64_t fault_events = 0;
  std::uint64_t local_damping_ticks = 0;
  std::uint64_t emergency_tracking_corrections = 0;
  std::uint64_t max_rate_limited_run = 0;
  std::uint64_t current_rate_limited_run = 0;
  double max_command_velocity = 0.0;
  double max_command_acceleration = 0.0;
  double max_window_motion = 0.0;
  double max_measured_velocity = 0.0;
  double max_tracking_error = 0.0;
};

struct Output {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> velocity{};
  std::array<double, kJointCount> window_motion{};
  std::array<JointState, kJointCount> state{};
  std::array<bool, kJointCount> local_damping{};
  std::array<bool, kJointCount> window_limited{};
  std::array<bool, kJointCount> step_limited{};
  std::array<bool, kJointCount> acceleration_limited{};
  std::array<bool, kJointCount> tracking_limited{};
  bool accepting_desired = false;
};

inline Limits BaseLimits() {
  Limits limits;
  limits.min_position = {
      -2.4807, -0.4736, -2.7076, -0.037267, -0.82267, -0.2118,
      -2.4807, -2.9171, -2.7076, -0.037267, -0.82267, -0.2118,
      -2.5680, -0.4700, -0.4700,
      -3.0392, -1.5382, -2.5680, -0.9972, -1.92222, -1.56443, -1.56443,
      -3.0392, -2.2015, -2.5680, -0.9972, -1.92222, -1.56443, -1.56443,
  };
  limits.max_position = {
      2.8298, 2.9171, 2.7076, 2.8298, 0.4736, 0.2118,
      2.8298, 0.4736, 2.7076, 2.8298, 0.4736, 0.2118,
      2.5680, 0.4700, 0.4700,
      2.6204, 2.2015, 2.5680, 2.0444, 1.92222, 1.56443, 1.56443,
      2.6204, 1.5382, 2.5680, 2.0444, 1.92222, 1.56443, 1.56443,
  };
  return limits;
}

// The single limiter envelope used by both simulation and hardware. There is
// deliberately no simulation compatibility profile: simulation must expose
// any policy or balance failure caused by the deployment safety limits.
// These conservative values still require mechanically supported commissioning
// before hardware use; sharing them does not itself approve unsupported motion.
inline Limits SharedSafetyLimits() {
  Limits limits = BaseLimits();
  limits.brake_target_error = {
      .04, .04, .04, .04, .03, .03, .04, .04, .04, .04, .03, .03,
      .03, .03, .03,
      .03, .03, .03, .03, .02, .02, .02,
      .03, .03, .03, .03, .02, .02, .02,
  };
  limits.max_instant_velocity = {
      .60, .60, .60, .60, .60, .60, .60, .60, .60, .60, .60, .60,
      .40, .40, .40,
      .35, .35, .35, .35, .30, .25, .25,
      .35, .35, .35, .35, .30, .25, .25,
  };
  limits.max_window_velocity = {
      .35, .35, .35, .35, .35, .35, .35, .35, .35, .35, .35, .35,
      .25, .25, .25,
      .20, .20, .20, .20, .18, .15, .15,
      .20, .20, .20, .20, .18, .15, .15,
  };
  limits.max_acceleration = {
      2., 2., 2., 2., 2., 2., 2., 2., 2., 2., 2., 2.,
      1.5, 1.5, 1.5,
      1., 1., 1., 1., .8, .7, .7,
      1., 1., 1., 1., .8, .7, .7,
  };
  limits.max_tracking_error = {
      .10, .10, .10, .10, .08, .08, .10, .10, .10, .10, .08, .08,
      .08, .08, .08,
      .12, .12, .12, .12, .10, .10, .10,
      .12, .12, .12, .12, .10, .10, .10,
  };
  limits.measured_velocity_brake.fill(0.7);
  limits.measured_velocity_fault.fill(1.2);
  return limits;
}

// Scale the amount of limiting without maintaining separate simulation and
// hardware profiles. A level of 1.0 is the full shared envelope. For levels in
// (0, 1), use a sixth-power response before reciprocally scaling dynamic
// ceilings. This keeps the low end of the runtime slider genuinely mild (0.5
// means sixty-four times the velocity, acceleration, window, tracking, and
// measured-motion ceilings) while preserving the exact shared envelope at
// 1.0. Level 0 is an exact controller bypass; the command writer handles it,
// not this function. Mechanical position limits are never interpolated.
inline Limits SafetyLimitsForLevel(double level) {
  Limits limits = SharedSafetyLimits();
  const double bounded_level = std::clamp(level, 1.0e-6, 1.0);
  const double squared_level = bounded_level * bounded_level;
  const double scale = 1.0 / (squared_level * squared_level * squared_level);
  const auto scale_array = [scale](auto& values) {
    for (double& value : values) value *= scale;
  };
  scale_array(limits.max_instant_velocity);
  scale_array(limits.max_acceleration);
  scale_array(limits.max_window_velocity);
  scale_array(limits.max_tracking_error);
  scale_array(limits.measured_velocity_brake);
  scale_array(limits.measured_velocity_fault);
  scale_array(limits.brake_target_error);
  return limits;
}

class PerJointMotionLimiter {
 public:
  explicit PerJointMotionLimiter(Limits limits) : limits_(std::move(limits)) {
    const auto samples = static_cast<std::size_t>(
        std::llround(limits_.window_duration / limits_.writer_dt));
    window_samples_ = std::clamp<std::size_t>(samples, 1, kMaxWindowSamples);
    states_.fill(JointState::kNormal);
    fault_reasons_.fill(JointFaultReason::kNone);
  }

  bool Seed(const std::array<double, kJointCount>& measured) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (!std::isfinite(measured[joint]) ||
          measured[joint] < limits_.min_position[joint] ||
          measured[joint] > limits_.max_position[joint]) {
        return false;
      }
    }
    position_ = measured;
    velocity_.fill(0.0);
    window_sum_.fill(0.0);
    for (auto& history : window_history_) history.fill(0.0);
    overspeed_counts_.fill(0);
    states_.fill(JointState::kNormal);
    fault_reasons_.fill(JointFaultReason::kNone);
    stats_.fill(JointStats{});
    window_cursor_ = 0;
    seeded_ = true;
    return true;
  }

  Output Step(const std::array<double, kJointCount>& desired,
              const std::array<double, kJointCount>& measured,
              const std::array<double, kJointCount>& measured_velocity,
              bool accept_desired) {
    Output output;
    output.position = position_;
    output.velocity = velocity_;
    output.state = states_;
    output.accepting_desired = accept_desired;
    if (!seeded_) return output;

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      window_sum_[joint] -= window_history_[joint][window_cursor_];
      window_history_[joint][window_cursor_] = 0.0;
      window_sum_[joint] = std::max(0.0, window_sum_[joint]);
      StepJoint(joint, desired[joint], measured[joint],
                measured_velocity[joint], accept_desired, output);
    }
    window_cursor_ = (window_cursor_ + 1) % window_samples_;
    output.position = position_;
    output.velocity = velocity_;
    output.window_motion = window_sum_;
    output.state = states_;
    return output;
  }

  [[nodiscard]] bool seeded() const { return seeded_; }
  [[nodiscard]] const Limits& limits() const { return limits_; }
  [[nodiscard]] const std::array<JointStats, kJointCount>& stats() const {
    return stats_;
  }
  [[nodiscard]] const std::array<JointFaultReason, kJointCount>& fault_reasons() const {
    return fault_reasons_;
  }
  [[nodiscard]] std::size_t window_samples() const { return window_samples_; }

 private:
  void LatchFault(std::size_t joint, JointFaultReason reason) {
    if (states_[joint] != JointState::kJointFault) {
      ++stats_[joint].fault_events;
      fault_reasons_[joint] = reason;
    }
    states_[joint] = JointState::kJointFault;
  }

  void StepJoint(std::size_t joint, double desired, double measured,
                 double measured_velocity, bool accept_desired, Output& output) {
    auto& stats = stats_[joint];
    ++stats.ticks;
    const double dt = limits_.writer_dt;
    const double previous_position = position_[joint];
    const double previous_velocity = velocity_[joint];

    if (!std::isfinite(measured)) {
      LatchFault(joint, JointFaultReason::kInvalidMeasuredPosition);
      output.local_damping[joint] = true;
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
      return;
    }
    if (!std::isfinite(measured_velocity)) {
      LatchFault(joint, JointFaultReason::kInvalidMeasuredVelocity);
      output.local_damping[joint] = true;
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
      return;
    }
    stats.max_measured_velocity =
        std::max(stats.max_measured_velocity, std::abs(measured_velocity));
    if (!std::isfinite(desired)) {
      LatchFault(joint, JointFaultReason::kInvalidDesiredPosition);
      desired = measured;
    }

    if (std::abs(measured_velocity) > limits_.measured_velocity_fault[joint]) {
      if (++overspeed_counts_[joint] >= limits_.overspeed_samples_to_fault) {
        LatchFault(joint, JointFaultReason::kMeasuredOverspeed);
      }
    } else {
      overspeed_counts_[joint] = 0;
    }

    const bool faulted = states_[joint] == JointState::kJointFault;
    const bool braking = faulted ||
        std::abs(measured_velocity) > limits_.measured_velocity_brake[joint];
    double target = desired;
    if (!accept_desired) {
      target = measured;
    } else if (braking) {
      const double direction = measured_velocity > 0.0 ? 1.0 :
                               measured_velocity < 0.0 ? -1.0 : 0.0;
      target = measured - direction * limits_.brake_target_error[joint];
    }

    const double bounded_position = std::clamp(
        target, limits_.min_position[joint], limits_.max_position[joint]);
    output.tracking_limited[joint] |= bounded_position != target;
    target = bounded_position;
    const double tracking_bounded = std::clamp(
        target,
        measured - limits_.max_tracking_error[joint],
        measured + limits_.max_tracking_error[joint]);
    output.tracking_limited[joint] |= tracking_bounded != target;
    target = tracking_bounded;

    const double full_window_budget =
        limits_.max_window_velocity[joint] * limits_.window_duration;
    const double target_window_budget = (braking || !accept_desired)
        ? full_window_budget
        : full_window_budget * limits_.normal_window_fraction;
    const double target_remaining_budget =
        std::max(0.0, target_window_budget - window_sum_[joint]);
    const double full_remaining_budget =
        std::max(0.0, full_window_budget - window_sum_[joint]);
    double requested_velocity = (target - previous_position) / dt;
    const double step_bounded = std::clamp(
        requested_velocity,
        -limits_.max_instant_velocity[joint],
        limits_.max_instant_velocity[joint]);
    output.step_limited[joint] = step_bounded != requested_velocity;
    requested_velocity = step_bounded;

    const double acceleration = limits_.max_acceleration[joint];
    const auto stoppable_velocity = [acceleration, dt](double room) {
      return std::max(
          0.0, -acceleration * dt +
          std::sqrt(acceleration * acceleration * dt * dt +
                    2.0 * acceleration * std::max(0.0, room)));
    };
    const double upper_safe_bound = std::min(
        limits_.max_position[joint],
        measured + limits_.max_tracking_error[joint]);
    const double lower_safe_bound = std::max(
        limits_.min_position[joint],
        measured - limits_.max_tracking_error[joint]);
    const double positive_tracking_room =
        std::max(0.0, upper_safe_bound - previous_position);
    const double negative_tracking_room =
        std::max(0.0, previous_position - lower_safe_bound);
    const double positive_stoppable = stoppable_velocity(
        std::min(target_remaining_budget, positive_tracking_room));
    const double negative_stoppable = stoppable_velocity(
        std::min(target_remaining_budget, negative_tracking_room));
    const double window_bounded = std::clamp(
        requested_velocity, -negative_stoppable, positive_stoppable);
    output.window_limited[joint] = window_bounded != requested_velocity;
    requested_velocity = window_bounded;

    const double acceleration_step = acceleration * dt;
    double next_velocity = std::clamp(
        requested_velocity,
        previous_velocity - acceleration_step,
        previous_velocity + acceleration_step);
    output.acceleration_limited[joint] = next_velocity != requested_velocity;
    double delta = next_velocity * dt;
    if (std::abs(delta) > full_remaining_budget) {
      delta = std::copysign(full_remaining_budget, delta);
      next_velocity = delta / dt;
      output.window_limited[joint] = true;
    }

    const double next_position = previous_position + delta;
    if (next_position < lower_safe_bound || next_position > upper_safe_bound) {
      output.tracking_limited[joint] = true;
      // The measured-position envelope can move faster than the command
      // envelope when the physical joint is disturbed or already overspeeding.
      // Snapping q_out to that moving envelope would itself create an unsafe
      // command step. Preserve the velocity/acceleration-bounded output and
      // remove stored spring energy by damping only this joint instead.
      LatchFault(joint, JointFaultReason::kEmergencyTrackingCorrection);
      ++stats.emergency_tracking_corrections;
    }

    position_[joint] = next_position;
    velocity_[joint] = next_velocity;
    const double motion = std::abs(delta);
    window_history_[joint][window_cursor_] = motion;
    window_sum_[joint] += motion;

    const bool limited = output.window_limited[joint] ||
        output.step_limited[joint] || output.acceleration_limited[joint] ||
        output.tracking_limited[joint];
    if (states_[joint] == JointState::kJointFault) {
      output.local_damping[joint] = true;
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
    } else if (braking) {
      states_[joint] = JointState::kBraking;
      ++stats.braking_ticks;
    } else if (!accept_desired) {
      states_[joint] = JointState::kBraking;
      ++stats.lifecycle_hold_ticks;
    } else if (limited) {
      states_[joint] = JointState::kRateLimited;
    } else {
      states_[joint] = JointState::kNormal;
    }
    if (limited) {
      ++stats.rate_limited_ticks;
      ++stats.current_rate_limited_run;
      stats.max_rate_limited_run =
          std::max(stats.max_rate_limited_run, stats.current_rate_limited_run);
    } else {
      stats.current_rate_limited_run = 0;
    }
    stats.max_command_velocity =
        std::max(stats.max_command_velocity, std::abs(next_velocity));
    stats.max_command_acceleration = std::max(
        stats.max_command_acceleration,
        std::abs((next_velocity - previous_velocity) / dt));
    stats.max_window_motion = std::max(stats.max_window_motion, window_sum_[joint]);
    stats.max_tracking_error = std::max(
        stats.max_tracking_error, std::abs(next_position - measured));
  }

  Limits limits_;
  bool seeded_ = false;
  std::size_t window_samples_ = 1;
  std::size_t window_cursor_ = 0;
  std::array<double, kJointCount> position_{};
  std::array<double, kJointCount> velocity_{};
  std::array<double, kJointCount> window_sum_{};
  std::array<std::array<double, kMaxWindowSamples>, kJointCount> window_history_{};
  std::array<int, kJointCount> overspeed_counts_{};
  std::array<JointState, kJointCount> states_{};
  std::array<JointFaultReason, kJointCount> fault_reasons_{};
  std::array<JointStats, kJointCount> stats_{};
};

inline const char* JointStateName(JointState state) {
  switch (state) {
    case JointState::kNormal: return "normal";
    case JointState::kRateLimited: return "rate_limited";
    case JointState::kBraking: return "braking";
    case JointState::kJointFault: return "joint_fault";
  }
  return "unknown";
}

inline const char* JointFaultReasonName(JointFaultReason reason) {
  switch (reason) {
    case JointFaultReason::kNone: return "none";
    case JointFaultReason::kInvalidMeasuredPosition: return "invalid_measured_position";
    case JointFaultReason::kInvalidMeasuredVelocity: return "invalid_measured_velocity";
    case JointFaultReason::kInvalidDesiredPosition: return "invalid_desired_position";
    case JointFaultReason::kMeasuredOverspeed: return "measured_overspeed";
    case JointFaultReason::kEmergencyTrackingCorrection:
      return "emergency_tracking_correction";
  }
  return "unknown";
}

}  // namespace sonic::safety
