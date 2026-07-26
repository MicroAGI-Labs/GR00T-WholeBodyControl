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
constexpr std::size_t kMaxMeasuredVelocitySamples = 64;

enum class JointState {
  kFollowing,
  kCommandLimited,
  kMeasuredMotionBraking,
  kLifecycleHold,
  kJointFault,
};

enum class JointFaultReason {
  kNone,
  kInvalidMeasuredPosition,
  kInvalidMeasuredVelocity,
  kInvalidDesiredPosition,
  kMeasuredOverspeed,
  kMeasuredOveracceleration,
  kEmergencyTrackingCorrection,
};

enum class LimitReason : std::uint32_t {
  kNone = 0,
  kMechanicalPosition = 1u << 0,
  kTrackingDistance = 1u << 1,
  kInstantVelocity = 1u << 2,
  kInstantAcceleration = 1u << 3,
  kInstantJerk = 1u << 4,
  kPositionWindow = 1u << 5,
  kDeltaVWindow = 1u << 6,
  kMeasuredVelocity = 1u << 7,
  kMeasuredAcceleration = 1u << 8,
  kStaleTarget = 1u << 9,
  kTermination = 1u << 10,
  kTargetStopping = 1u << 11,
};

constexpr std::uint32_t ReasonBit(LimitReason reason) {
  return static_cast<std::uint32_t>(reason);
}

constexpr bool HasReason(std::uint32_t reasons, LimitReason reason) {
  return (reasons & ReasonBit(reason)) != 0;
}

struct Limits {
  std::array<double, kJointCount> min_position{};
  std::array<double, kJointCount> max_position{};
  std::array<double, kJointCount> max_instant_velocity{};
  std::array<double, kJointCount> max_acceleration{};
  std::array<double, kJointCount> max_jerk{};
  std::array<double, kJointCount> max_window_velocity{};
  std::array<double, kJointCount> max_window_acceleration{};
  std::array<double, kJointCount> max_tracking_error{};
  std::array<double, kJointCount> measured_velocity_brake{};
  std::array<double, kJointCount> measured_velocity_fault{};
  std::array<double, kJointCount> measured_acceleration_brake{};
  std::array<double, kJointCount> measured_acceleration_fault{};
  std::array<double, kJointCount> brake_target_error{};
  double writer_dt = 0.002;
  double window_duration = 0.100;
  double normal_window_fraction = 0.75;
  double nominal_target_interval = 0.020;
  double min_target_interval = 0.005;
  double max_target_interval = 0.100;
  double measured_acceleration_window = 0.020;
  int overspeed_samples_to_fault = 3;
  int overacceleration_samples_to_fault = 3;
  bool enforce_measured_acceleration = true;
};

struct JointStats {
  std::uint64_t ticks = 0;
  std::uint64_t command_limited_ticks = 0;
  std::uint64_t measured_motion_braking_ticks = 0;
  std::uint64_t lifecycle_hold_ticks = 0;
  std::uint64_t fault_ticks = 0;
  std::uint64_t fault_events = 0;
  std::uint64_t local_damping_ticks = 0;
  std::uint64_t emergency_tracking_corrections = 0;
  std::uint64_t max_command_limited_run = 0;
  std::uint64_t current_command_limited_run = 0;
  std::uint64_t mechanical_limit_ticks = 0;
  std::uint64_t tracking_limit_ticks = 0;
  std::uint64_t velocity_limit_ticks = 0;
  std::uint64_t acceleration_limit_ticks = 0;
  std::uint64_t jerk_limit_ticks = 0;
  std::uint64_t position_window_limit_ticks = 0;
  std::uint64_t delta_v_window_limit_ticks = 0;
  std::uint64_t measured_velocity_brake_ticks = 0;
  std::uint64_t measured_acceleration_brake_ticks = 0;
  std::uint64_t target_stopping_ticks = 0;
  std::uint64_t stale_target_ticks = 0;
  std::uint64_t termination_ticks = 0;
  double max_command_velocity = 0.0;
  double max_command_acceleration = 0.0;
  double max_command_jerk = 0.0;
  double max_window_motion = 0.0;
  double max_window_delta_v = 0.0;
  double max_measured_velocity = 0.0;
  double max_measured_acceleration = 0.0;
  double max_tracking_error = 0.0;
};

struct TargetStats {
  std::uint64_t new_targets = 0;
  std::uint64_t repeated_writer_ticks = 0;
  double interval_min = std::numeric_limits<double>::infinity();
  double interval_max = 0.0;
  double interval_sum = 0.0;

  [[nodiscard]] double interval_mean() const {
    return new_targets > 1 ? interval_sum / static_cast<double>(new_targets - 1)
                           : 0.0;
  }
};

struct Output {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> velocity{};
  std::array<double, kJointCount> acceleration{};
  std::array<double, kJointCount> window_motion{};
  std::array<double, kJointCount> window_delta_v{};
  std::array<double, kJointCount> measured_acceleration{};
  std::array<JointState, kJointCount> state{};
  std::array<std::uint32_t, kJointCount> reasons{};
  std::array<bool, kJointCount> local_damping{};
  std::array<bool, kJointCount> window_limited{};
  std::array<bool, kJointCount> step_limited{};
  std::array<bool, kJointCount> acceleration_limited{};
  std::array<bool, kJointCount> jerk_limited{};
  std::array<bool, kJointCount> acceleration_window_limited{};
  std::array<bool, kJointCount> tracking_limited{};
  std::array<bool, kJointCount> measured_acceleration_populated{};
  bool accepting_desired = false;
  bool new_target = false;
  bool target_endpoint_sample = false;
  double target_interval = 0.0;
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

// The single limiter envelope used by both simulation and hardware. These are
// study values, not an approval for unsupported hardware operation.
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
  limits.measured_acceleration_brake.fill(12.0);
  limits.measured_acceleration_fault.fill(20.0);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    // The rolling delta-v window is the sustained-acceleration constraint. A
    // comparatively high instantaneous jerk ceiling keeps the 500 Hz follower
    // responsive to genuine 50 Hz balance targets while still making every
    // emitted acceleration transition explicit and bounded.
    limits.max_jerk[joint] = 2500.0 * limits.max_acceleration[joint];
    // One normal-window acceleration budget plus enough reserve to remove the
    // maximum permitted velocity. This makes the reserve physically useful.
    limits.max_window_acceleration[joint] =
        limits.max_acceleration[joint] +
        limits.max_instant_velocity[joint] / limits.window_duration;
  }
  return limits;
}

// Level zero is handled as an exact writer bypass. All non-mechanical dynamic
// limits use the same response curve in simulation and deployment.
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
  scale_array(limits.max_jerk);
  scale_array(limits.max_window_velocity);
  scale_array(limits.max_window_acceleration);
  scale_array(limits.max_tracking_error);
  scale_array(limits.measured_velocity_brake);
  scale_array(limits.measured_velocity_fault);
  scale_array(limits.measured_acceleration_brake);
  scale_array(limits.measured_acceleration_fault);
  scale_array(limits.brake_target_error);
  return limits;
}

class PerJointMotionLimiter {
 public:
  explicit PerJointMotionLimiter(Limits limits) : limits_(std::move(limits)) {
    const auto position_samples = static_cast<std::size_t>(
        std::llround(limits_.window_duration / limits_.writer_dt));
    window_samples_ =
        std::clamp<std::size_t>(position_samples, 1, kMaxWindowSamples);
    const auto measured_samples = static_cast<std::size_t>(
        std::llround(limits_.measured_acceleration_window / limits_.writer_dt));
    measured_velocity_samples_ = std::clamp<std::size_t>(
        measured_samples, 2, kMaxMeasuredVelocitySamples);
    const double measured_mean_index =
        0.5 * static_cast<double>(measured_velocity_samples_ - 1);
    for (std::size_t sample = 0; sample < measured_velocity_samples_; ++sample) {
      const double centered =
          static_cast<double>(sample) - measured_mean_index;
      measured_velocity_regression_denominator_ += centered * centered;
    }
    states_.fill(JointState::kFollowing);
    fault_reasons_.fill(JointFaultReason::kNone);
    target_interval_ = limits_.nominal_target_interval;
    tracking_time_ = target_interval_;
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
    target_ = measured;
    velocity_.fill(0.0);
    acceleration_.fill(0.0);
    window_sum_.fill(0.0);
    delta_v_window_sum_.fill(0.0);
    for (auto& history : window_history_) history.fill(0.0);
    for (auto& history : delta_v_window_history_) history.fill(0.0);
    for (auto& history : measured_velocity_history_) history.fill(0.0);
    measured_velocity_sum_.fill(0.0);
    measured_velocity_weighted_sum_.fill(0.0);
    overspeed_counts_.fill(0);
    overacceleration_counts_.fill(0);
    states_.fill(JointState::kFollowing);
    fault_reasons_.fill(JointFaultReason::kNone);
    stats_.fill(JointStats{});
    target_stats_ = TargetStats{};
    window_cursor_ = 0;
    measured_velocity_cursor_ = 0;
    measured_velocity_population_ = 0;
    target_interval_ = limits_.nominal_target_interval;
    target_elapsed_ = 0.0;
    target_endpoint_reported_ = false;
    tracking_time_ = target_interval_;
    seeded_ = true;
    return true;
  }

  Output Step(const std::array<double, kJointCount>& desired,
              const std::array<double, kJointCount>& measured,
              const std::array<double, kJointCount>& measured_velocity,
              bool accept_desired, bool new_target = true,
              double target_interval = 0.020) {
    Output output;
    output.accepting_desired = accept_desired;
    output.new_target = new_target;
    if (!seeded_) {
      output.position = position_;
      output.velocity = velocity_;
      output.acceleration = acceleration_;
      output.state = states_;
      return output;
    }

    if (accept_desired && new_target) {
      const double bounded_interval =
          std::isfinite(target_interval)
              ? std::clamp(target_interval, limits_.min_target_interval,
                           limits_.max_target_interval)
              : limits_.nominal_target_interval;
      target_interval_ = bounded_interval;
      target_elapsed_ = 0.0;
      target_endpoint_reported_ = false;
      output.target_interval = bounded_interval;
      if (target_stats_.new_targets > 0) {
        target_stats_.interval_min =
            std::min(target_stats_.interval_min, bounded_interval);
        target_stats_.interval_max =
            std::max(target_stats_.interval_max, bounded_interval);
        target_stats_.interval_sum += bounded_interval;
      }
      ++target_stats_.new_targets;
    } else if (accept_desired) {
      ++target_stats_.repeated_writer_ticks;
    }
    output.target_interval = target_interval_;
    output.target_endpoint_sample = accept_desired && !new_target &&
        !target_endpoint_reported_ &&
        target_elapsed_ + limits_.writer_dt >= target_interval_ - kTolerance;
    tracking_time_ = std::max(
        limits_.writer_dt, target_interval_ - target_elapsed_);
    const bool target_is_terminal =
        !accept_desired || target_elapsed_ >= target_interval_;

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      window_sum_[joint] -= window_history_[joint][window_cursor_];
      delta_v_window_sum_[joint] -=
          delta_v_window_history_[joint][window_cursor_];
      window_history_[joint][window_cursor_] = 0.0;
      delta_v_window_history_[joint][window_cursor_] = 0.0;
      window_sum_[joint] = std::max(0.0, window_sum_[joint]);
      delta_v_window_sum_[joint] =
          std::max(0.0, delta_v_window_sum_[joint]);
      StepJoint(joint, desired[joint], measured[joint],
                measured_velocity[joint], accept_desired, new_target,
                target_is_terminal, output);
    }
    window_cursor_ = (window_cursor_ + 1) % window_samples_;
    measured_velocity_cursor_ =
        (measured_velocity_cursor_ + 1) % measured_velocity_samples_;
    measured_velocity_population_ =
        std::min(measured_velocity_population_ + 1, measured_velocity_samples_);
    target_elapsed_ += limits_.writer_dt;
    if (output.target_endpoint_sample) target_endpoint_reported_ = true;
    output.position = position_;
    output.velocity = velocity_;
    output.acceleration = acceleration_;
    output.window_motion = window_sum_;
    output.window_delta_v = delta_v_window_sum_;
    output.state = states_;
    return output;
  }

  [[nodiscard]] bool seeded() const { return seeded_; }
  [[nodiscard]] const Limits& limits() const { return limits_; }
  [[nodiscard]] const std::array<JointStats, kJointCount>& stats() const {
    return stats_;
  }
  [[nodiscard]] const TargetStats& target_stats() const { return target_stats_; }
  [[nodiscard]] const std::array<JointFaultReason, kJointCount>& fault_reasons()
      const {
    return fault_reasons_;
  }
  [[nodiscard]] std::size_t window_samples() const { return window_samples_; }
  [[nodiscard]] std::size_t measured_velocity_samples() const {
    return measured_velocity_samples_;
  }
  [[nodiscard]] double measured_acceleration_filter_delay() const {
    return 0.5 * static_cast<double>(measured_velocity_samples_ - 1) *
           limits_.writer_dt;
  }

 private:
  static constexpr double kTolerance = 1.0e-12;

  void AddReason(Output& output, std::size_t joint, LimitReason reason) {
    output.reasons[joint] |= ReasonBit(reason);
  }

  void LatchFault(std::size_t joint, JointFaultReason reason) {
    if (states_[joint] != JointState::kJointFault) {
      ++stats_[joint].fault_events;
      fault_reasons_[joint] = reason;
    }
    states_[joint] = JointState::kJointFault;
  }

  double EstimateMeasuredAcceleration(std::size_t joint,
                                      double measured_velocity,
                                      bool& populated) {
    const std::size_t previous_count = measured_velocity_population_;
    if (previous_count < measured_velocity_samples_) {
      measured_velocity_history_[joint][measured_velocity_cursor_] =
          measured_velocity;
      measured_velocity_sum_[joint] += measured_velocity;
      measured_velocity_weighted_sum_[joint] +=
          static_cast<double>(previous_count) * measured_velocity;
    } else {
      const double oldest =
          measured_velocity_history_[joint][measured_velocity_cursor_];
      const double previous_sum = measured_velocity_sum_[joint];
      measured_velocity_history_[joint][measured_velocity_cursor_] =
          measured_velocity;
      measured_velocity_weighted_sum_[joint] =
          measured_velocity_weighted_sum_[joint] - previous_sum + oldest +
          static_cast<double>(measured_velocity_samples_ - 1) *
              measured_velocity;
      measured_velocity_sum_[joint] =
          previous_sum - oldest + measured_velocity;
    }
    const std::size_t count =
        std::min(previous_count + 1, measured_velocity_samples_);
    populated = count == measured_velocity_samples_;
    if (!populated) return 0.0;

    const double mean_index = 0.5 * static_cast<double>(count - 1);
    const double numerator = measured_velocity_weighted_sum_[joint] -
        mean_index * measured_velocity_sum_[joint];
    return numerator /
        (limits_.writer_dt * measured_velocity_regression_denominator_);
  }

  // Conservative stopping-distance upper bound without a square root. During
  // the finite jerk ramp, assume any acceleration already pointing along the
  // motion persists for the entire ramp, then brake the resulting upper-bound
  // velocity at a_max. This overestimates rather than understates stopping room.
  static double JerkLimitedStoppingDistance(double velocity,
                                            double directed_acceleration,
                                            double max_acceleration,
                                            double max_jerk) {
    if (!(velocity > 0.0)) return 0.0;
    const double acceleration =
        std::clamp(directed_acceleration, -max_acceleration, max_acceleration);
    const double ramp_time =
        std::max(0.0, (acceleration + max_acceleration) / max_jerk);
    const double positive_acceleration = std::max(0.0, acceleration);
    const double ramp_distance = velocity * ramp_time +
        0.5 * positive_acceleration * ramp_time * ramp_time;
    const double upper_velocity =
        velocity + positive_acceleration * ramp_time;
    return ramp_distance +
        upper_velocity * upper_velocity / (2.0 * max_acceleration);
  }

  static void Intersect(double lower, double upper, double& allowed_lower,
                        double& allowed_upper) {
    allowed_lower = std::max(allowed_lower, lower);
    allowed_upper = std::min(allowed_upper, upper);
  }

  void CountReasons(std::size_t joint, std::uint32_t reasons) {
    auto& stats = stats_[joint];
    if (HasReason(reasons, LimitReason::kMechanicalPosition))
      ++stats.mechanical_limit_ticks;
    if (HasReason(reasons, LimitReason::kTrackingDistance))
      ++stats.tracking_limit_ticks;
    if (HasReason(reasons, LimitReason::kInstantVelocity))
      ++stats.velocity_limit_ticks;
    if (HasReason(reasons, LimitReason::kInstantAcceleration))
      ++stats.acceleration_limit_ticks;
    if (HasReason(reasons, LimitReason::kInstantJerk))
      ++stats.jerk_limit_ticks;
    if (HasReason(reasons, LimitReason::kPositionWindow))
      ++stats.position_window_limit_ticks;
    if (HasReason(reasons, LimitReason::kDeltaVWindow))
      ++stats.delta_v_window_limit_ticks;
    if (HasReason(reasons, LimitReason::kMeasuredVelocity))
      ++stats.measured_velocity_brake_ticks;
    if (HasReason(reasons, LimitReason::kMeasuredAcceleration))
      ++stats.measured_acceleration_brake_ticks;
    if (HasReason(reasons, LimitReason::kTargetStopping))
      ++stats.target_stopping_ticks;
    if (HasReason(reasons, LimitReason::kStaleTarget))
      ++stats.stale_target_ticks;
    if (HasReason(reasons, LimitReason::kTermination))
      ++stats.termination_ticks;
  }

  void StepJoint(std::size_t joint, double desired, double measured,
                 double measured_velocity, bool accept_desired, bool new_target,
                 bool target_is_terminal, Output& output) {
    auto& stats = stats_[joint];
    ++stats.ticks;
    const double dt = limits_.writer_dt;
    const double previous_position = position_[joint];
    const double previous_velocity = velocity_[joint];
    const double previous_acceleration = acceleration_[joint];

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

    bool measured_acceleration_populated = false;
    const double measured_acceleration = EstimateMeasuredAcceleration(
        joint, measured_velocity, measured_acceleration_populated);
    output.measured_acceleration[joint] = measured_acceleration;
    output.measured_acceleration_populated[joint] =
        measured_acceleration_populated;
    if (measured_acceleration_populated) {
      stats.max_measured_acceleration = std::max(
          stats.max_measured_acceleration, std::abs(measured_acceleration));
    }

    if (std::abs(measured_velocity) > limits_.measured_velocity_fault[joint]) {
      if (++overspeed_counts_[joint] >= limits_.overspeed_samples_to_fault) {
        LatchFault(joint, JointFaultReason::kMeasuredOverspeed);
      }
    } else {
      overspeed_counts_[joint] = 0;
    }
    if (measured_acceleration_populated &&
        std::abs(measured_acceleration) >
            limits_.measured_acceleration_fault[joint]) {
      if (++overacceleration_counts_[joint] >=
              limits_.overacceleration_samples_to_fault &&
          limits_.enforce_measured_acceleration) {
        LatchFault(joint, JointFaultReason::kMeasuredOveracceleration);
      }
    } else {
      overacceleration_counts_[joint] = 0;
    }

    const bool measured_velocity_braking =
        std::abs(measured_velocity) > limits_.measured_velocity_brake[joint];
    const bool measured_acceleration_braking =
        measured_acceleration_populated &&
        std::abs(measured_acceleration) >
            limits_.measured_acceleration_brake[joint];
    const bool enforce_measured_acceleration_braking =
        measured_acceleration_braking && limits_.enforce_measured_acceleration;
    const bool measured_motion_braking =
        measured_velocity_braking || enforce_measured_acceleration_braking;
    if (measured_velocity_braking)
      AddReason(output, joint, LimitReason::kMeasuredVelocity);
    if (measured_acceleration_braking)
      AddReason(output, joint, LimitReason::kMeasuredAcceleration);

    if (accept_desired && new_target && std::isfinite(desired)) {
      target_[joint] = desired;
    } else if (!accept_desired) {
      target_[joint] = measured;
      AddReason(output, joint, LimitReason::kStaleTarget);
      AddReason(output, joint, LimitReason::kTermination);
    }

    const bool faulted = states_[joint] == JointState::kJointFault;
    double target = target_[joint];
    if (faulted || measured_motion_braking) {
      const double motion =
          std::abs(measured_velocity) > kTolerance
              ? measured_velocity
              : measured_acceleration;
      const double direction = motion > 0.0 ? 1.0 : motion < 0.0 ? -1.0 : 0.0;
      target = measured - direction * limits_.brake_target_error[joint];
    }

    const double mechanically_bounded =
        std::clamp(target, limits_.min_position[joint],
                   limits_.max_position[joint]);
    if (std::abs(mechanically_bounded - target) > kTolerance) {
      AddReason(output, joint, LimitReason::kMechanicalPosition);
      output.tracking_limited[joint] = true;
    }
    target = mechanically_bounded;
    const double lower_safe_bound = std::max(
        limits_.min_position[joint],
        measured - limits_.max_tracking_error[joint]);
    const double upper_safe_bound = std::min(
        limits_.max_position[joint],
        measured + limits_.max_tracking_error[joint]);
    const double tracking_bounded =
        std::clamp(target, lower_safe_bound, upper_safe_bound);
    if (std::abs(tracking_bounded - target) > kTolerance) {
      AddReason(output, joint, LimitReason::kTrackingDistance);
      output.tracking_limited[joint] = true;
    }
    target = tracking_bounded;

    const double error = target - previous_position;
    const double max_velocity = limits_.max_instant_velocity[joint];
    const double max_acceleration = limits_.max_acceleration[joint];
    const double max_jerk = limits_.max_jerk[joint];
    double velocity_goal = error / tracking_time_;
    const double bounded_velocity_goal =
        std::clamp(velocity_goal, -max_velocity, max_velocity);
    if (std::abs(bounded_velocity_goal - velocity_goal) > kTolerance) {
      AddReason(output, joint, LimitReason::kInstantVelocity);
      output.step_limited[joint] = true;
    }
    velocity_goal = bounded_velocity_goal;

    const double full_position_budget =
        limits_.max_window_velocity[joint] * limits_.window_duration;
    const double normal_position_budget =
        full_position_budget * limits_.normal_window_fraction;
    const double full_position_remaining =
        std::max(0.0, full_position_budget - window_sum_[joint]);
    const double normal_position_remaining =
        std::max(0.0, normal_position_budget - window_sum_[joint]);
    const double full_delta_v_budget =
        limits_.max_window_acceleration[joint] * limits_.window_duration;
    const double braking_delta_v_reserve =
        std::min(max_velocity, full_delta_v_budget);
    const double normal_delta_v_budget =
        std::max(0.0, full_delta_v_budget - braking_delta_v_reserve);
    const double full_delta_v_remaining =
        std::max(0.0, full_delta_v_budget - delta_v_window_sum_[joint]);
    const double normal_delta_v_remaining =
        std::max(0.0, normal_delta_v_budget - delta_v_window_sum_[joint]);

    bool target_stopping = false;
    if (std::abs(previous_velocity) > kTolerance) {
      const double motion_direction = std::copysign(1.0, previous_velocity);
      const double directed_velocity = std::abs(previous_velocity);
      const double directed_acceleration =
          previous_acceleration * motion_direction;
      double room = motion_direction > 0.0
                        ? upper_safe_bound - previous_position
                        : previous_position - lower_safe_bound;
      room = std::min(room, full_position_remaining);
      if (target_is_terminal && error * motion_direction > 0.0)
        room = std::min(room, std::abs(error));
      const double stopping_distance = JerkLimitedStoppingDistance(
          directed_velocity, directed_acceleration, max_acceleration, max_jerk);
      const double stopping_delta_v =
          directed_velocity +
          std::max(0.0, directed_acceleration) *
              (directed_acceleration + max_acceleration) / max_jerk;
      if ((target_is_terminal && error * motion_direction <= 0.0) ||
          stopping_distance + 4.0 * std::abs(previous_velocity) * dt +
                  2.0 * std::abs(previous_acceleration) * dt * dt >=
              std::max(0.0, room) ||
          stopping_delta_v >= full_delta_v_remaining) {
        velocity_goal = 0.0;
        target_stopping = true;
        AddReason(output, joint, LimitReason::kTargetStopping);
      }
    }

    // Also test the state that one normal tracking tick would create. Looking
    // only at the current state can cross the viability boundary in that tick,
    // especially during a target reversal while acceleration still points in
    // the old direction.
    if (!target_stopping) {
      const double provisional_acceleration_goal = std::clamp(
          (velocity_goal - previous_velocity) / dt, -max_acceleration,
          max_acceleration);
      const double provisional_acceleration = std::clamp(
          provisional_acceleration_goal, previous_acceleration - max_jerk * dt,
          previous_acceleration + max_jerk * dt);
      const double provisional_velocity =
          previous_velocity + provisional_acceleration * dt;
      const double provisional_position =
          previous_position + provisional_velocity * dt;
      if (std::abs(provisional_velocity) > kTolerance) {
        const double direction = std::copysign(1.0, provisional_velocity);
        double next_room = direction > 0.0
                               ? upper_safe_bound - provisional_position
                               : provisional_position - lower_safe_bound;
        next_room = std::min(
            next_room,
            std::max(0.0, full_position_remaining -
                              std::abs(provisional_velocity) * dt));
        const double next_error = target - provisional_position;
        if (target_is_terminal && next_error * direction > 0.0)
          next_room = std::min(next_room, std::abs(next_error));
        const double next_stopping_distance = JerkLimitedStoppingDistance(
            std::abs(provisional_velocity),
            provisional_acceleration * direction, max_acceleration, max_jerk);
        if ((target_is_terminal && next_error * direction <= 0.0) ||
            next_stopping_distance +
                    4.0 * std::abs(provisional_velocity) * dt +
                    2.0 * std::abs(provisional_acceleration) * dt * dt >=
                std::max(0.0, next_room)) {
          velocity_goal = 0.0;
          target_stopping = true;
          AddReason(output, joint, LimitReason::kTargetStopping);
        }
      }
    }

    double acceleration_goal =
        (velocity_goal - previous_velocity) / dt;
    const double bounded_acceleration_goal =
        std::clamp(acceleration_goal, -max_acceleration, max_acceleration);
    if (std::abs(bounded_acceleration_goal - acceleration_goal) > kTolerance) {
      AddReason(output, joint, LimitReason::kInstantAcceleration);
      output.acceleration_limited[joint] = true;
    }
    acceleration_goal = bounded_acceleration_goal;

    const bool command_braking = target_stopping ||
        (previous_velocity * acceleration_goal < 0.0);
    const bool use_braking_reserve = faulted || !accept_desired ||
        measured_motion_braking || command_braking;
    const double position_remaining = use_braking_reserve
                                          ? full_position_remaining
                                          : normal_position_remaining;
    const double delta_v_remaining = use_braking_reserve
                                         ? full_delta_v_remaining
                                         : normal_delta_v_remaining;

    double allowed_acceleration_lower = -max_acceleration;
    double allowed_acceleration_upper = max_acceleration;
    const double jerk_step = max_jerk * dt;
    Intersect(previous_acceleration - jerk_step,
              previous_acceleration + jerk_step,
              allowed_acceleration_lower, allowed_acceleration_upper);
    const double jerk_clamped = std::clamp(
        acceleration_goal, previous_acceleration - jerk_step,
        previous_acceleration + jerk_step);
    if (std::abs(jerk_clamped - acceleration_goal) > kTolerance) {
      AddReason(output, joint, LimitReason::kInstantJerk);
      output.jerk_limited[joint] = true;
    }

    Intersect((-max_velocity - previous_velocity) / dt,
              (max_velocity - previous_velocity) / dt,
              allowed_acceleration_lower, allowed_acceleration_upper);
    const double delta_v_acceleration_cap = delta_v_remaining / dt;
    Intersect(-delta_v_acceleration_cap, delta_v_acceleration_cap,
              allowed_acceleration_lower, allowed_acceleration_upper);

    const double velocity_from_position_budget = position_remaining / dt;
    Intersect((-velocity_from_position_budget - previous_velocity) / dt,
              (velocity_from_position_budget - previous_velocity) / dt,
              allowed_acceleration_lower, allowed_acceleration_upper);

    // Target updates may legitimately move behind a reference that is already
    // in flight. Treat the target as a stopping objective, not a hard position
    // boundary: snapping to a newly moved target would violate jerk. Mechanical
    // and measured-tracking bounds remain hard state constraints.
    const double next_lower_position = lower_safe_bound;
    const double next_upper_position = upper_safe_bound;
    Intersect((next_lower_position - previous_position -
               previous_velocity * dt) /
                  (dt * dt),
              (next_upper_position - previous_position -
               previous_velocity * dt) /
                  (dt * dt),
              allowed_acceleration_lower, allowed_acceleration_upper);

    if (allowed_acceleration_lower > allowed_acceleration_upper + kTolerance) {
      // A moving measured-position envelope can become unreachable without a
      // command discontinuity. Preserve the last bounded reference and damp
      // only this joint instead of snapping the target or damping the body.
      LatchFault(joint, JointFaultReason::kEmergencyTrackingCorrection);
      ++stats.emergency_tracking_corrections;
      output.tracking_limited[joint] = true;
      AddReason(output, joint, LimitReason::kTrackingDistance);
      output.local_damping[joint] = true;
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
      CountReasons(joint, output.reasons[joint]);
      return;
    }

    double next_acceleration = std::clamp(
        acceleration_goal, allowed_acceleration_lower,
        allowed_acceleration_upper);
    if (std::abs(next_acceleration - acceleration_goal) > kTolerance) {
      if (next_acceleration <= -delta_v_acceleration_cap + kTolerance ||
          next_acceleration >= delta_v_acceleration_cap - kTolerance) {
        AddReason(output, joint, LimitReason::kDeltaVWindow);
        output.acceleration_window_limited[joint] = true;
      }
      if (std::abs(previous_velocity + next_acceleration * dt) >=
          velocity_from_position_budget - kTolerance) {
        AddReason(output, joint, LimitReason::kPositionWindow);
        output.window_limited[joint] = true;
      }
    }

    const double next_velocity =
        previous_velocity + next_acceleration * dt;
    const double next_position = previous_position + next_velocity * dt;
    const double position_motion = std::abs(next_position - previous_position);
    const double delta_v = std::abs(next_velocity - previous_velocity);

    position_[joint] = next_position;
    velocity_[joint] = next_velocity;
    acceleration_[joint] = next_acceleration;
    window_history_[joint][window_cursor_] = position_motion;
    delta_v_window_history_[joint][window_cursor_] = delta_v;
    window_sum_[joint] += position_motion;
    delta_v_window_sum_[joint] += delta_v;

    const std::uint32_t command_reason_mask =
        ReasonBit(LimitReason::kMechanicalPosition) |
        ReasonBit(LimitReason::kTrackingDistance) |
        ReasonBit(LimitReason::kInstantVelocity) |
        ReasonBit(LimitReason::kInstantAcceleration) |
        ReasonBit(LimitReason::kInstantJerk) |
        ReasonBit(LimitReason::kPositionWindow) |
        ReasonBit(LimitReason::kDeltaVWindow) |
        ReasonBit(LimitReason::kTargetStopping);
    const bool command_limited =
        (output.reasons[joint] & command_reason_mask) != 0;
    if (states_[joint] == JointState::kJointFault) {
      output.local_damping[joint] = true;
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
    } else if (!accept_desired) {
      states_[joint] = JointState::kLifecycleHold;
      ++stats.lifecycle_hold_ticks;
    } else if (measured_motion_braking) {
      states_[joint] = JointState::kMeasuredMotionBraking;
      ++stats.measured_motion_braking_ticks;
    } else if (command_limited) {
      states_[joint] = JointState::kCommandLimited;
      ++stats.command_limited_ticks;
    } else {
      states_[joint] = JointState::kFollowing;
    }
    if (command_limited) {
      ++stats.current_command_limited_run;
      stats.max_command_limited_run = std::max(
          stats.max_command_limited_run, stats.current_command_limited_run);
    } else {
      stats.current_command_limited_run = 0;
    }
    CountReasons(joint, output.reasons[joint]);
    stats.max_command_velocity =
        std::max(stats.max_command_velocity, std::abs(next_velocity));
    stats.max_command_acceleration =
        std::max(stats.max_command_acceleration, std::abs(next_acceleration));
    stats.max_command_jerk = std::max(
        stats.max_command_jerk,
        std::abs((next_acceleration - previous_acceleration) / dt));
    stats.max_window_motion =
        std::max(stats.max_window_motion, window_sum_[joint]);
    stats.max_window_delta_v =
        std::max(stats.max_window_delta_v, delta_v_window_sum_[joint]);
    stats.max_tracking_error = std::max(
        stats.max_tracking_error, std::abs(next_position - measured));
  }

  Limits limits_;
  bool seeded_ = false;
  std::size_t window_samples_ = 1;
  std::size_t window_cursor_ = 0;
  std::size_t measured_velocity_samples_ = 2;
  std::size_t measured_velocity_cursor_ = 0;
  std::size_t measured_velocity_population_ = 0;
  double measured_velocity_regression_denominator_ = 0.0;
  double tracking_time_ = 0.020;
  double target_interval_ = 0.020;
  double target_elapsed_ = 0.0;
  bool target_endpoint_reported_ = false;
  std::array<double, kJointCount> target_{};
  std::array<double, kJointCount> position_{};
  std::array<double, kJointCount> velocity_{};
  std::array<double, kJointCount> acceleration_{};
  std::array<double, kJointCount> window_sum_{};
  std::array<double, kJointCount> delta_v_window_sum_{};
  std::array<std::array<double, kMaxWindowSamples>, kJointCount> window_history_{};
  std::array<std::array<double, kMaxWindowSamples>, kJointCount>
      delta_v_window_history_{};
  std::array<std::array<double, kMaxMeasuredVelocitySamples>, kJointCount>
      measured_velocity_history_{};
  std::array<double, kJointCount> measured_velocity_sum_{};
  std::array<double, kJointCount> measured_velocity_weighted_sum_{};
  std::array<int, kJointCount> overspeed_counts_{};
  std::array<int, kJointCount> overacceleration_counts_{};
  std::array<JointState, kJointCount> states_{};
  std::array<JointFaultReason, kJointCount> fault_reasons_{};
  std::array<JointStats, kJointCount> stats_{};
  TargetStats target_stats_{};
};

inline const char* JointStateName(JointState state) {
  switch (state) {
    case JointState::kFollowing: return "following";
    case JointState::kCommandLimited: return "command_limited";
    case JointState::kMeasuredMotionBraking: return "measured_motion_braking";
    case JointState::kLifecycleHold: return "lifecycle_hold";
    case JointState::kJointFault: return "joint_fault";
  }
  return "unknown";
}

inline const char* JointFaultReasonName(JointFaultReason reason) {
  switch (reason) {
    case JointFaultReason::kNone: return "none";
    case JointFaultReason::kInvalidMeasuredPosition:
      return "invalid_measured_position";
    case JointFaultReason::kInvalidMeasuredVelocity:
      return "invalid_measured_velocity";
    case JointFaultReason::kInvalidDesiredPosition:
      return "invalid_desired_position";
    case JointFaultReason::kMeasuredOverspeed: return "measured_overspeed";
    case JointFaultReason::kMeasuredOveracceleration:
      return "measured_overacceleration";
    case JointFaultReason::kEmergencyTrackingCorrection:
      return "emergency_tracking_correction";
  }
  return "unknown";
}

}  // namespace sonic::safety
