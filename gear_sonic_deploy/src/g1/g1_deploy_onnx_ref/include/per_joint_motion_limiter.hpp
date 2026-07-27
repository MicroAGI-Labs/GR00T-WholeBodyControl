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
  kTargetOffset = 1u << 0,
  kMechanicalPosition = 1u << 1,
  kCommandVelocity = 1u << 2,
  kPeakAcceleration = 1u << 3,
  kSustainedAcceleration = 1u << 4,
  kBrakingAcceleration = 1u << 5,
  kStoppingDistance = 1u << 6,
  kPositionWindow = 1u << 7,
  kDeltaVWindow = 1u << 8,
  kTrackingEnvelope = 1u << 9,
  kMeasuredVelocity = 1u << 10,
  kMeasuredAcceleration = 1u << 11,
  kStaleTarget = 1u << 12,
  kTermination = 1u << 13,
  kJointFault = 1u << 14,
  kAccelerationSlew = 1u << 15,

  // Compatibility names retained for existing diagnostics and tests.
  kTrackingDistance = kTargetOffset,
  kInstantVelocity = kCommandVelocity,
  kInstantAcceleration = kSustainedAcceleration,
  kInstantJerk = kAccelerationSlew,
  kTargetStopping = kStoppingDistance,
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
  std::array<double, kJointCount> peak_acceleration{};
  std::array<double, kJointCount> sustained_acceleration{};
  std::array<double, kJointCount> braking_acceleration{};
  std::array<double, kJointCount> burst_distance{};
  std::array<double, kJointCount> burst_velocity{};
  std::array<double, kJointCount> max_acceleration{};
  std::array<double, kJointCount> max_jerk{};
  std::array<double, kJointCount> max_window_velocity{};
  std::array<double, kJointCount> max_window_acceleration{};
  // Maximum accepted SONIC target displacement from the current measured
  // joint position. This caps the goal presented to the 500 Hz reference
  // governor; it never causes an instantaneous change in the emitted command.
  std::array<double, kJointCount> max_target_offset{};
  std::array<double, kJointCount> max_tracking_error{};
  std::array<double, kJointCount> measured_velocity_brake{};
  std::array<double, kJointCount> measured_velocity_fault{};
  std::array<double, kJointCount> measured_acceleration_brake{};
  std::array<double, kJointCount> measured_acceleration_fault{};
  std::array<double, kJointCount> brake_target_error{};
  double writer_dt = 0.002;
  double window_duration = 0.100;
  double delta_v_window_duration = 0.020;
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
  std::uint64_t target_offset_ticks = 0;
  std::uint64_t tracking_limit_ticks = 0;
  std::uint64_t velocity_limit_ticks = 0;
  std::uint64_t acceleration_limit_ticks = 0;
  std::uint64_t peak_acceleration_limit_ticks = 0;
  std::uint64_t sustained_acceleration_limit_ticks = 0;
  std::uint64_t braking_acceleration_limit_ticks = 0;
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
  std::array<double, kJointCount> raw_target{};
  std::array<double, kJointCount> target_position{};
  std::array<double, kJointCount> requested_velocity{};
  std::array<double, kJointCount> permitted_velocity{};
  std::array<double, kJointCount> requested_acceleration{};
  std::array<double, kJointCount> drive_acceleration_limit{};
  std::array<double, kJointCount> burst_distance_factor{};
  std::array<double, kJointCount> burst_velocity_factor{};
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
  limits.peak_acceleration = limits.max_acceleration;
  limits.sustained_acceleration = limits.max_acceleration;
  limits.braking_acceleration = limits.max_acceleration;
  limits.burst_distance.fill(0.03);
  limits.burst_velocity.fill(0.30);
  limits.max_tracking_error = {
      .10, .10, .10, .10, .08, .08, .10, .10, .10, .10, .08, .08,
      .08, .08, .08,
      .12, .12, .12, .12, .10, .10, .10,
      .12, .12, .12, .12, .10, .10, .10,
  };
  limits.max_target_offset = limits.max_tracking_error;
  // A measured-relative target cap and a measured-relative q_out envelope are
  // different constraints.  The continuous reference needs room to brake if
  // the measured joint moves opposite it; making both envelopes identical can
  // make the acceleration intersection infeasible at the target-cap boundary.
  // The outer envelope is still finite and per-joint, while the inner target
  // cap remains the stored-energy limit presented to the governor.
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
    limits.max_tracking_error[joint] = 2.0 * limits.max_target_offset[joint];
  // Experimental full-limiting reference-governor profile for the twelve leg
  // joints. These are simulator qualification values, not approved hardware
  // limits. Lower limiter percentages relax them through the shared curve.
  for (std::size_t joint = 0; joint < 12; ++joint) {
    limits.max_instant_velocity[joint] = 2.0;
    limits.peak_acceleration[joint] = 1500.0;
    limits.sustained_acceleration[joint] = 200.0;
    limits.braking_acceleration[joint] = 1000.0;
    limits.max_acceleration[joint] = 1500.0;
    limits.burst_distance[joint] = 0.03;
    limits.burst_velocity[joint] = 0.30;
  }
  // The original waist/arm acceleration ceilings predated the continuous
  // reference governor and were too small for it to converge during the
  // supported handoff (at 40%, only 4.4--9.4 rad/s^2). Keep these joints on
  // independent, explicitly lower profiles than the legs, but give the
  // governor enough authority to follow the policy rather than accumulating a
  // large measured-to-command tracking error while support is active.
  for (std::size_t joint = 12; joint < kJointCount; ++joint) {
    limits.max_instant_velocity[joint] = 2.0;
    limits.peak_acceleration[joint] = 500.0;
    limits.sustained_acceleration[joint] = 50.0;
    limits.braking_acceleration[joint] = 300.0;
    limits.max_acceleration[joint] = 500.0;
  }
  limits.measured_velocity_brake.fill(0.7);
  limits.measured_velocity_fault.fill(1.2);
  limits.measured_acceleration_brake.fill(12.0);
  limits.measured_acceleration_fault.fill(20.0);
  for (std::size_t joint = 0; joint < 12; ++joint) {
    limits.measured_velocity_brake[joint] = 2.2;
    limits.measured_velocity_fault[joint] = 3.0;
    limits.measured_acceleration_brake[joint] = 1000.0;
    limits.measured_acceleration_fault[joint] = 1600.0;
  }
  for (std::size_t joint = 12; joint < kJointCount; ++joint) {
    limits.measured_acceleration_brake[joint] = 100.0;
    limits.measured_acceleration_fault[joint] = 160.0;
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    // The rolling delta-v window is the sustained-acceleration constraint. A
    // comparatively high instantaneous jerk ceiling keeps the 500 Hz follower
    // responsive to genuine 50 Hz balance targets while still making every
    // emitted acceleration transition explicit and bounded.
    limits.max_jerk[joint] = 1250000.0;
    // One normal-window acceleration budget plus enough reserve to remove the
    // maximum permitted velocity. This makes the reserve physically useful.
    // The 20 ms delta-v budget contains a normal drive allowance plus a full
    // velocity-change reserve for braking. Drive can never consume the reserve.
    limits.max_window_acceleration[joint] =
        limits.max_acceleration[joint] +
        2.0 * limits.max_instant_velocity[joint] /
            limits.delta_v_window_duration;
  }
  for (std::size_t joint = 0; joint < 12; ++joint)
    limits.max_window_acceleration[joint] = 300.0;
  return limits;
}

// Level zero is handled as an exact writer bypass. All non-mechanical dynamic
// limits use the same inverse-fifth-power response curve in simulation and
// deployment: 100% is the configured full limiter and 40% permits 97.65625x
// each ceiling. The steep response is intentional: balance performance drops
// sharply when a low percentage still phase-limits the 50 Hz SONIC reference.
// The proposed experimental values remain anchored exactly at 100%.
inline Limits SafetyLimitsForLevel(double level) {
  Limits limits = SharedSafetyLimits();
  const double bounded_level = std::clamp(level, 1.0e-6, 1.0);
  const double squared_level = bounded_level * bounded_level;
  const double scale =
      1.0 / (squared_level * squared_level * bounded_level);
  const auto scale_array = [scale](auto& values) {
    for (double& value : values) value *= scale;
  };
  scale_array(limits.max_instant_velocity);
  scale_array(limits.peak_acceleration);
  scale_array(limits.sustained_acceleration);
  scale_array(limits.braking_acceleration);
  scale_array(limits.burst_distance);
  scale_array(limits.burst_velocity);
  scale_array(limits.max_acceleration);
  scale_array(limits.max_jerk);
  scale_array(limits.max_window_velocity);
  scale_array(limits.max_window_acceleration);
  scale_array(limits.max_target_offset);
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
    const auto delta_v_samples = static_cast<std::size_t>(
        std::llround(limits_.delta_v_window_duration / limits_.writer_dt));
    delta_v_window_samples_ =
        std::clamp<std::size_t>(delta_v_samples, 1, kMaxWindowSamples);
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
    delta_v_window_cursor_ = 0;
    measured_velocity_cursor_ = 0;
    measured_velocity_population_ = 0;
    target_interval_ = limits_.nominal_target_interval;
    target_elapsed_ = 0.0;
    target_endpoint_reported_ = false;
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
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      window_sum_[joint] -= window_history_[joint][window_cursor_];
      delta_v_window_sum_[joint] -=
          delta_v_window_history_[joint][delta_v_window_cursor_];
      window_history_[joint][window_cursor_] = 0.0;
      delta_v_window_history_[joint][delta_v_window_cursor_] = 0.0;
      window_sum_[joint] = std::max(0.0, window_sum_[joint]);
      delta_v_window_sum_[joint] =
          std::max(0.0, delta_v_window_sum_[joint]);
      StepJoint(joint, desired[joint], measured[joint],
                measured_velocity[joint], accept_desired, new_target, output);
    }
    window_cursor_ = (window_cursor_ + 1) % window_samples_;
    delta_v_window_cursor_ =
        (delta_v_window_cursor_ + 1) % delta_v_window_samples_;
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
  [[nodiscard]] std::size_t delta_v_window_samples() const {
    return delta_v_window_samples_;
  }
  [[nodiscard]] std::size_t measured_velocity_samples() const {
    return measured_velocity_samples_;
  }
  [[nodiscard]] double measured_acceleration_filter_delay() const {
    return 0.5 * static_cast<double>(measured_velocity_samples_ - 1) *
           limits_.writer_dt;
  }

 private:
  static constexpr double kTolerance = 1.0e-12;
  static constexpr double kConstraintTolerance = 1.0e-8;

  static double SmoothTaper(double value, double threshold) {
    if (!(threshold > 0.0)) return 0.0;
    const double x = std::clamp(value / threshold, 0.0, 1.0);
    const double smoothstep = x * x * (3.0 - 2.0 * x);
    return 1.0 - smoothstep;
  }

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
    if (HasReason(reasons, LimitReason::kTargetOffset))
      ++stats.target_offset_ticks;
    if (HasReason(reasons, LimitReason::kMechanicalPosition))
      ++stats.mechanical_limit_ticks;
    if (HasReason(reasons, LimitReason::kTrackingEnvelope))
      ++stats.tracking_limit_ticks;
    if (HasReason(reasons, LimitReason::kInstantVelocity))
      ++stats.velocity_limit_ticks;
    if (HasReason(reasons, LimitReason::kPeakAcceleration) ||
        HasReason(reasons, LimitReason::kSustainedAcceleration) ||
        HasReason(reasons, LimitReason::kBrakingAcceleration))
      ++stats.acceleration_limit_ticks;
    if (HasReason(reasons, LimitReason::kPeakAcceleration))
      ++stats.peak_acceleration_limit_ticks;
    if (HasReason(reasons, LimitReason::kSustainedAcceleration))
      ++stats.sustained_acceleration_limit_ticks;
    if (HasReason(reasons, LimitReason::kBrakingAcceleration))
      ++stats.braking_acceleration_limit_ticks;
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
                 Output& output) {
    auto& stats = stats_[joint];
    ++stats.ticks;
    const double dt = limits_.writer_dt;
    const double previous_position = position_[joint];
    const double previous_velocity = velocity_[joint];
    const double previous_acceleration = acceleration_[joint];
    output.raw_target[joint] = desired;

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
    // First cap the new SONIC goal relative to the measured joint state. This
    // is the per-joint instantaneous-target limit: target_ may jump at 50 Hz,
    // but position_/velocity_/acceleration_ remain continuous at 500 Hz.
    const double lower_target_bound = std::max(
        limits_.min_position[joint],
        measured - limits_.max_target_offset[joint]);
    const double upper_target_bound = std::min(
        limits_.max_position[joint],
        measured + limits_.max_target_offset[joint]);
    const double target_bounded =
        std::clamp(target, lower_target_bound, upper_target_bound);
    if (std::abs(target_bounded - target) > kTolerance) {
      AddReason(output, joint, LimitReason::kTargetOffset);
      output.tracking_limited[joint] = true;
    }
    target = target_bounded;
    output.target_position[joint] = target;

    // The emitted reference has its own measured-relative hard envelope. Keep
    // this distinct from the target cap so both stored command energy and the
    // accepted SONIC goal remain explicit configuration choices.
    const double lower_safe_bound = std::max(
        limits_.min_position[joint],
        measured - limits_.max_tracking_error[joint]);
    const double upper_safe_bound = std::min(
        limits_.max_position[joint],
        measured + limits_.max_tracking_error[joint]);

    const double error = target - previous_position;
    const double max_velocity = limits_.max_instant_velocity[joint];
    const double peak_acceleration = limits_.peak_acceleration[joint];
    const double sustained_acceleration =
        limits_.sustained_acceleration[joint];
    const double braking_acceleration = limits_.braking_acceleration[joint];
    const double max_acceleration = limits_.max_acceleration[joint];
    const double max_jerk = limits_.max_jerk[joint];
    const double distance_factor =
        SmoothTaper(std::abs(error), limits_.burst_distance[joint]);
    const double velocity_factor =
        SmoothTaper(std::abs(previous_velocity), limits_.burst_velocity[joint]);
    const double drive_acceleration_limit = sustained_acceleration +
        (peak_acceleration - sustained_acceleration) * distance_factor *
            velocity_factor;
    output.burst_distance_factor[joint] = distance_factor;
    output.burst_velocity_factor[joint] = velocity_factor;
    output.drive_acceleration_limit[joint] = drive_acceleration_limit;

    // SONIC's held 50 Hz sample is a causal goal, never an endpoint deadline.
    // The unconstrained request is recorded for telemetry; q_out remains a
    // continuous 500 Hz state and approaches it through the governor below.
    const double requested_velocity = error / dt;
    output.requested_velocity[joint] = requested_velocity;

    const double full_position_budget =
        limits_.max_window_velocity[joint] * limits_.window_duration;
    const double normal_position_budget =
        full_position_budget * limits_.normal_window_fraction;
    const double full_position_remaining =
        std::max(0.0, full_position_budget - window_sum_[joint]);
    const double normal_position_remaining =
        std::max(0.0, normal_position_budget - window_sum_[joint]);
    const double full_delta_v_budget =
        limits_.max_window_acceleration[joint] *
        limits_.delta_v_window_duration;
    const double braking_delta_v_reserve =
        std::min(2.0 * max_velocity, full_delta_v_budget);
    const double normal_delta_v_budget =
        std::max(0.0, full_delta_v_budget - braking_delta_v_reserve);
    const double full_delta_v_remaining =
        std::max(0.0, full_delta_v_budget - delta_v_window_sum_[joint]);
    const double normal_delta_v_remaining =
        std::max(0.0, normal_delta_v_budget - delta_v_window_sum_[joint]);

    // Restrict speed to one from which the reference can stop at the target.
    // The continuous bound is followed by a conservative jerk-aware viability
    // check below, including a one-writer-tick reaction margin.
    double stopping_room = std::abs(error);
    if (error > 0.0)
      stopping_room = std::min(stopping_room,
                               upper_safe_bound - previous_position);
    else if (error < 0.0)
      stopping_room = std::min(stopping_room,
                               previous_position - lower_safe_bound);
    stopping_room = std::min(stopping_room, full_position_remaining);
    stopping_room = std::max(0.0, stopping_room -
        std::abs(previous_velocity) * dt -
        0.5 * std::abs(previous_acceleration) * dt * dt);
    const double stopping_safe_velocity =
        std::sqrt(2.0 * braking_acceleration * stopping_room);
    double permitted_velocity =
        std::min(max_velocity, stopping_safe_velocity);
    double velocity_goal = std::abs(error) > kTolerance
        ? std::copysign(permitted_velocity, error)
        : 0.0;
    output.permitted_velocity[joint] = velocity_goal;
    if (std::abs(requested_velocity) > max_velocity + kTolerance) {
      AddReason(output, joint, LimitReason::kCommandVelocity);
      output.step_limited[joint] = true;
    }
    if (stopping_safe_velocity < max_velocity - kTolerance &&
        std::abs(requested_velocity) > stopping_safe_velocity + kTolerance) {
      AddReason(output, joint, LimitReason::kStoppingDistance);
    }

    bool target_stopping =
        (normal_delta_v_remaining <= kTolerance ||
         normal_position_remaining <= kTolerance) &&
        (std::abs(previous_velocity) > kTolerance ||
         std::abs(previous_acceleration) > kTolerance);
    target_stopping = target_stopping ||
        normal_position_remaining <=
            std::abs(previous_velocity) * dt +
                0.5 * std::abs(previous_acceleration) * dt * dt ||
        normal_delta_v_remaining <=
            std::abs(previous_acceleration) * dt;
    if (target_stopping) {
      velocity_goal = 0.0;
      AddReason(output, joint, LimitReason::kStoppingDistance);
    }
    if (!target_stopping && std::abs(previous_velocity) > kTolerance) {
      const double motion_direction = std::copysign(1.0, previous_velocity);
      const double directed_velocity = std::abs(previous_velocity);
      const double directed_acceleration =
          previous_acceleration * motion_direction;
      double room = motion_direction > 0.0
                        ? upper_safe_bound - previous_position
                        : previous_position - lower_safe_bound;
      // Begin braking before the normal drive allowance is exhausted. Once
      // braking is selected below, the full position and delta-v reserves are
      // available; this prevents an empty drive budget from becoming an
      // infeasible acceleration intersection and a false joint fault.
      room = std::min(room, normal_position_remaining);
      if (error * motion_direction > 0.0)
        room = std::min(room, std::abs(error));
      const double stopping_distance = JerkLimitedStoppingDistance(
          directed_velocity, directed_acceleration, braking_acceleration,
          max_jerk);
      const double stopping_delta_v =
          directed_velocity +
          std::max(0.0, directed_acceleration) *
              (directed_acceleration + braking_acceleration) / max_jerk;
      if (error * motion_direction <= 0.0 ||
          stopping_distance >= std::max(0.0, room) ||
          normal_delta_v_remaining <= kTolerance ||
          stopping_delta_v >= full_delta_v_remaining) {
        velocity_goal = 0.0;
        target_stopping = true;
        AddReason(output, joint, LimitReason::kStoppingDistance);
      }
    }

    // stopping_room above already reserves one writer tick. Re-evaluating a
    // second provisional tick here creates a finite dead band around the
    // target (especially during termination), so the next real 500 Hz tick is
    // the only look-ahead used by this causal governor.
    output.permitted_velocity[joint] = velocity_goal;

    double acceleration_goal = (velocity_goal - previous_velocity) / dt;
    output.requested_acceleration[joint] = acceleration_goal;
    const bool command_braking = target_stopping ||
        (std::abs(previous_velocity) > kTolerance &&
         previous_velocity * acceleration_goal < 0.0);
    const double acceleration_limit =
        command_braking ? braking_acceleration : drive_acceleration_limit;
    const double bounded_acceleration_goal =
        std::clamp(acceleration_goal, -acceleration_limit, acceleration_limit);
    if (std::abs(bounded_acceleration_goal - acceleration_goal) > kTolerance) {
      if (command_braking) {
        AddReason(output, joint, LimitReason::kBrakingAcceleration);
      } else {
        if (distance_factor * velocity_factor > kTolerance)
          AddReason(output, joint, LimitReason::kPeakAcceleration);
        if (distance_factor * velocity_factor < 1.0 - kTolerance)
          AddReason(output, joint, LimitReason::kSustainedAcceleration);
      }
      output.acceleration_limited[joint] = true;
    }
    acceleration_goal = bounded_acceleration_goal;

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
    if (acceleration_goal <
            (-max_velocity - previous_velocity) / dt - kTolerance ||
        acceleration_goal >
            (max_velocity - previous_velocity) / dt + kTolerance) {
      AddReason(output, joint, LimitReason::kCommandVelocity);
      output.step_limited[joint] = true;
    }
    const double delta_v_acceleration_cap = delta_v_remaining / dt;
    Intersect(-delta_v_acceleration_cap, delta_v_acceleration_cap,
              allowed_acceleration_lower, allowed_acceleration_upper);
    if (std::abs(acceleration_goal) >
        delta_v_acceleration_cap + kTolerance) {
      AddReason(output, joint, LimitReason::kDeltaVWindow);
      output.acceleration_window_limited[joint] = true;
    }

    const double velocity_from_position_budget = position_remaining / dt;
    Intersect((-velocity_from_position_budget - previous_velocity) / dt,
              (velocity_from_position_budget - previous_velocity) / dt,
              allowed_acceleration_lower, allowed_acceleration_upper);
    if (acceleration_goal <
            (-velocity_from_position_budget - previous_velocity) / dt -
                kTolerance ||
        acceleration_goal >
            (velocity_from_position_budget - previous_velocity) / dt +
                kTolerance) {
      AddReason(output, joint, LimitReason::kPositionWindow);
      output.window_limited[joint] = true;
    }

    // When the reference is already travelling toward an unchanged target,
    // land on it rather than stepping through it. If a fresh target moved
    // behind an in-flight reference, stopping remains gradual and this bound
    // is deliberately not applied.
    if (error > kTolerance && previous_velocity >= -kTolerance) {
      const double target_acceleration_upper =
          (target - previous_position - previous_velocity * dt) / (dt * dt);
      if (allowed_acceleration_lower <=
          target_acceleration_upper + kConstraintTolerance) {
        allowed_acceleration_upper =
            std::min(allowed_acceleration_upper, target_acceleration_upper);
        if (acceleration_goal > target_acceleration_upper + kTolerance)
          AddReason(output, joint, LimitReason::kStoppingDistance);
      }
    } else if (error < -kTolerance && previous_velocity <= kTolerance) {
      const double target_acceleration_lower =
          (target - previous_position - previous_velocity * dt) / (dt * dt);
      if (target_acceleration_lower <=
          allowed_acceleration_upper + kConstraintTolerance) {
        allowed_acceleration_lower =
            std::max(allowed_acceleration_lower, target_acceleration_lower);
        if (acceleration_goal < target_acceleration_lower - kTolerance)
          AddReason(output, joint, LimitReason::kStoppingDistance);
      }
    }

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
    if (acceleration_goal <
            (next_lower_position - previous_position -
             previous_velocity * dt) /
                    (dt * dt) -
                kTolerance ||
        acceleration_goal >
            (next_upper_position - previous_position -
             previous_velocity * dt) /
                    (dt * dt) +
                kTolerance) {
      AddReason(output, joint, LimitReason::kTrackingEnvelope);
      output.tracking_limited[joint] = true;
    }

    if (allowed_acceleration_lower > allowed_acceleration_upper &&
        allowed_acceleration_lower <=
            allowed_acceleration_upper + kConstraintTolerance) {
      const double numerical_boundary =
          0.5 * (allowed_acceleration_lower + allowed_acceleration_upper);
      allowed_acceleration_lower = numerical_boundary;
      allowed_acceleration_upper = numerical_boundary;
    }
    if (allowed_acceleration_lower >
        allowed_acceleration_upper + kConstraintTolerance) {
      // A moving measured-position envelope can become unreachable without a
      // command discontinuity. Preserve the last bounded reference and damp
      // only this joint instead of snapping the target or damping the body.
      LatchFault(joint, JointFaultReason::kEmergencyTrackingCorrection);
      ++stats.emergency_tracking_corrections;
      output.tracking_limited[joint] = true;
      AddReason(output, joint, LimitReason::kTrackingEnvelope);
      AddReason(output, joint, LimitReason::kJointFault);
      output.local_damping[joint] = true;
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
      CountReasons(joint, output.reasons[joint]);
      return;
    }

    double next_acceleration = std::clamp(
        acceleration_goal, allowed_acceleration_lower,
        allowed_acceleration_upper);
    (void)velocity_from_position_budget;

    const double next_velocity =
        previous_velocity + next_acceleration * dt;
    const double next_position = previous_position + next_velocity * dt;
    const double position_motion = std::abs(next_position - previous_position);
    const double delta_v = std::abs(next_velocity - previous_velocity);

    position_[joint] = next_position;
    velocity_[joint] = next_velocity;
    acceleration_[joint] = next_acceleration;
    window_history_[joint][window_cursor_] = position_motion;
    delta_v_window_history_[joint][delta_v_window_cursor_] = delta_v;
    window_sum_[joint] += position_motion;
    delta_v_window_sum_[joint] += delta_v;

    const std::uint32_t command_reason_mask =
        ReasonBit(LimitReason::kMechanicalPosition) |
        ReasonBit(LimitReason::kTargetOffset) |
        ReasonBit(LimitReason::kTrackingEnvelope) |
        ReasonBit(LimitReason::kCommandVelocity) |
        ReasonBit(LimitReason::kPeakAcceleration) |
        ReasonBit(LimitReason::kSustainedAcceleration) |
        ReasonBit(LimitReason::kBrakingAcceleration) |
        ReasonBit(LimitReason::kAccelerationSlew) |
        ReasonBit(LimitReason::kPositionWindow) |
        ReasonBit(LimitReason::kDeltaVWindow) |
        ReasonBit(LimitReason::kStoppingDistance);
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
  std::size_t delta_v_window_samples_ = 1;
  std::size_t delta_v_window_cursor_ = 0;
  std::size_t measured_velocity_samples_ = 2;
  std::size_t measured_velocity_cursor_ = 0;
  std::size_t measured_velocity_population_ = 0;
  double measured_velocity_regression_denominator_ = 0.0;
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
