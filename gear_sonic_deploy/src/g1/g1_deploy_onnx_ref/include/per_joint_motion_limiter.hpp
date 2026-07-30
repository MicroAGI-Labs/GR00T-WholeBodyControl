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
  kInvalidDesiredCommand,
  kMeasuredOverspeed,
  kMeasuredOveracceleration,
};

enum class LimitReason : std::uint32_t {
  kNone = 0,
  kMechanicalPosition = 1u << 0,
  kStoppingDistance = 1u << 1,
  kMeasuredVelocity = 1u << 2,
  kMeasuredAcceleration = 1u << 3,
  kStaleTarget = 1u << 4,
  kTermination = 1u << 5,
  kJointFault = 1u << 6,
  kServoTorque = 1u << 7,
  kTorqueSlew = 1u << 8,
  kMechanicalBarrier = 1u << 9,
  kVelocityEnergy = 1u << 10,
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
  std::array<double, kJointCount> mechanical_slowdown_distance{};
  std::array<double, kJointCount> braking_acceleration{};
  std::array<double, kJointCount> measured_velocity_brake{};
  std::array<double, kJointCount> measured_velocity_fault{};
  std::array<double, kJointCount> measured_acceleration_brake{};
  std::array<double, kJointCount> measured_acceleration_fault{};
  std::array<double, kJointCount> brake_target_error{};
  std::array<double, kJointCount> max_servo_torque{};
  std::array<double, kJointCount> free_torque_step{};
  std::array<double, kJointCount> free_measured_velocity{};
  std::array<double, kJointCount> smoothing_measured_velocity{};
  std::array<double, kJointCount> low_speed_torque_ramp_time{};
  std::array<double, kJointCount> high_speed_torque_ramp_time{};
  std::array<double, kJointCount> protective_braking_ramp_time{};
  double writer_dt = 0.002;
  double nominal_target_interval = 0.020;
  double min_target_interval = 0.005;
  double max_target_interval = 0.100;
  double measured_acceleration_window = 0.020;
  int overspeed_samples_to_fault = 3;
  int overacceleration_samples_to_fault = 3;
};

struct JointStats {
  std::uint64_t ticks = 0;
  std::uint64_t command_limited_ticks = 0;
  std::uint64_t measured_motion_braking_ticks = 0;
  std::uint64_t lifecycle_hold_ticks = 0;
  std::uint64_t fault_ticks = 0;
  std::uint64_t fault_events = 0;
  std::uint64_t local_damping_ticks = 0;
  std::uint64_t max_command_limited_run = 0;
  std::uint64_t current_command_limited_run = 0;
  std::uint64_t mechanical_limit_ticks = 0;
  std::uint64_t measured_velocity_brake_ticks = 0;
  std::uint64_t measured_acceleration_brake_ticks = 0;
  std::uint64_t stale_target_ticks = 0;
  std::uint64_t termination_ticks = 0;
  std::uint64_t servo_torque_limit_ticks = 0;
  std::uint64_t torque_slew_ticks = 0;
  std::uint64_t mechanical_barrier_ticks = 0;
  std::uint64_t velocity_energy_ticks = 0;
  double max_measured_velocity = 0.0;
  double max_measured_acceleration = 0.0;
  double max_tracking_error = 0.0;
  double max_command_position_step = 0.0;
  double max_requested_servo_torque = 0.0;
  double max_emitted_servo_torque = 0.0;
  double max_servo_torque_step = 0.0;
  double max_ordinary_servo_torque_step = 0.0;
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
  std::array<double, kJointCount> measured_acceleration{};
  std::array<double, kJointCount> raw_target{};
  std::array<double, kJointCount> target_position{};
  std::array<double, kJointCount> requested_servo_torque{};
  std::array<double, kJointCount> emitted_servo_torque{};
  std::array<double, kJointCount> emitted_velocity_target{};
  std::array<double, kJointCount> emitted_feedforward_torque{};
  std::array<double, kJointCount> torque_smoothing_factor{};
  std::array<JointState, kJointCount> state{};
  std::array<std::uint32_t, kJointCount> reasons{};
  std::array<bool, kJointCount> local_damping{};
  std::array<bool, kJointCount> measured_acceleration_populated{};
  bool accepting_desired = false;
  bool new_target = false;
  double target_interval = 0.0;
};

inline Limits HardwareSafetyLimits() {
  Limits limits;

  // These are the deployed G1 mechanical envelopes. Wrist roll uses the
  // minimal symmetric range which contains the -1.9778 rad position observed
  // on the Thor-connected physical robot. Values outside these bounds reject
  // initialization rather than being silently clamped.
  limits.min_position = {
      -2.4807, -0.4736, -2.7076, -0.037267, -0.82267, -0.2118,
      -2.4807, -2.9171, -2.7076, -0.037267, -0.82267, -0.2118,
      -2.5680, -0.4700, -0.4700,
      -3.0392, -1.5382, -2.5680, -0.9972, -1.98000, -1.56443, -1.56443,
      -3.0392, -2.2015, -2.5680, -1.0472, -1.98000, -1.56443, -1.56443,
  };
  limits.max_position = {
      2.8298, 2.9171, 2.7076, 2.8298, 0.4736, 0.2118,
      2.8298, 0.4736, 2.7076, 2.8298, 0.4736, 0.2118,
      2.5680, 0.4700, 0.4700,
      2.6204, 2.2015, 2.5680, 2.0444, 1.98000, 1.56443, 1.56443,
      2.6204, 1.5382, 2.5680, 2.0944, 1.98000, 1.56443, 1.56443,
  };
  limits.brake_target_error = {
      .04, .04, .04, .04, .03, .03, .04, .04, .04, .04, .03, .03,
      .03, .03, .03,
      .03, .03, .03, .03, .02, .02, .02,
      .03, .03, .03, .03, .02, .02, .02,
  };

  // Exact state-dependent-torque-projection-v5 profile used by the validated
  // simulator controller. Keep the mandatory/fail-closed deployment wrapper,
  // but do not substitute more restrictive thresholds: doing so changes the
  // closed-loop policy response and destroys sim/deployment parity.
  limits.max_servo_torque = {
      139.0, 139.0, 88.0, 139.0, 25.0, 25.0,
      139.0, 139.0, 88.0, 139.0, 25.0, 25.0,
      88.0, 25.0, 25.0,
      25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
      25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
  };

  for (std::size_t joint = 0; joint < 12; ++joint) {
    limits.free_torque_step[joint] = 0.10 * limits.max_servo_torque[joint];
    limits.free_measured_velocity[joint] = 2.0;
    limits.smoothing_measured_velocity[joint] = 8.0;
    limits.measured_velocity_brake[joint] = 12.0;
    limits.measured_velocity_fault[joint] = 18.0;
    limits.measured_acceleration_brake[joint] = 1000.0;
    limits.measured_acceleration_fault[joint] = 1600.0;
    limits.braking_acceleration[joint] = 150.0;
    limits.mechanical_slowdown_distance[joint] = 0.15;
    limits.low_speed_torque_ramp_time[joint] = 0.020;
    limits.high_speed_torque_ramp_time[joint] = 0.040;
    limits.protective_braking_ramp_time[joint] = 0.020;
  }
  for (std::size_t joint = 12; joint <= 14; ++joint) {
    limits.free_torque_step[joint] = 0.10 * limits.max_servo_torque[joint];
    limits.free_measured_velocity[joint] = 1.0;
    limits.smoothing_measured_velocity[joint] = 4.0;
    limits.measured_velocity_brake[joint] = 6.0;
    limits.measured_velocity_fault[joint] = 10.0;
    limits.measured_acceleration_brake[joint] = 300.0;
    limits.measured_acceleration_fault[joint] = 500.0;
    limits.braking_acceleration[joint] = 80.0;
    limits.mechanical_slowdown_distance[joint] = 0.12;
    limits.low_speed_torque_ramp_time[joint] = 0.030;
    limits.high_speed_torque_ramp_time[joint] = 0.060;
    limits.protective_braking_ramp_time[joint] = 0.025;
  }
  for (std::size_t joint = 15; joint < kJointCount; ++joint) {
    const bool wrist = (joint >= 19 && joint <= 21) || joint >= 26;
    limits.free_torque_step[joint] = 0.10 * limits.max_servo_torque[joint];
    limits.free_measured_velocity[joint] = 0.30;
    limits.smoothing_measured_velocity[joint] = wrist ? 1.0 : 1.5;
    limits.measured_velocity_brake[joint] = wrist ? 2.0 : 4.0;
    limits.measured_velocity_fault[joint] = wrist ? 5.0 : 7.0;
    limits.measured_acceleration_brake[joint] = 150.0;
    limits.measured_acceleration_fault[joint] = 300.0;
    limits.braking_acceleration[joint] = 30.0;
    limits.mechanical_slowdown_distance[joint] = wrist ? 0.08 : 0.10;
    limits.low_speed_torque_ramp_time[joint] = wrist ? 0.100 : 0.080;
    limits.high_speed_torque_ramp_time[joint] = wrist ? 0.150 : 0.120;
    limits.protective_braking_ramp_time[joint] = wrist ? 0.050 : 0.040;
  }
  return limits;
}

class PerJointMotionLimiter {
 public:
  explicit PerJointMotionLimiter(Limits limits) : limits_(std::move(limits)) {
    const auto samples = static_cast<std::size_t>(
        std::llround(limits_.measured_acceleration_window /
                     limits_.writer_dt));
    measured_velocity_samples_ = std::clamp<std::size_t>(
        samples, 2, kMaxMeasuredVelocitySamples);
    const double mean =
        0.5 * static_cast<double>(measured_velocity_samples_ - 1);
    for (std::size_t sample = 0; sample < measured_velocity_samples_; ++sample) {
      const double centered = static_cast<double>(sample) - mean;
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
    target_ = measured;
    position_ = measured;
    emitted_servo_torque_.fill(0.0);
    torque_smoothing_active_.fill(false);
    for (auto& history : measured_velocity_history_) history.fill(0.0);
    measured_velocity_sum_.fill(0.0);
    measured_velocity_weighted_sum_.fill(0.0);
    overspeed_counts_.fill(0);
    overacceleration_counts_.fill(0);
    states_.fill(JointState::kFollowing);
    fault_reasons_.fill(JointFaultReason::kNone);
    stats_.fill(JointStats{});
    target_stats_ = TargetStats{};
    measured_velocity_cursor_ = 0;
    measured_velocity_population_ = 0;
    target_interval_ = limits_.nominal_target_interval;
    seeded_ = true;
    return true;
  }

  Output StepWithServo(
      const std::array<double, kJointCount>& desired,
      const std::array<double, kJointCount>& desired_velocity,
      const std::array<double, kJointCount>& servo_kp,
      const std::array<double, kJointCount>& servo_kd,
      const std::array<double, kJointCount>& feedforward_torque,
      const std::array<double, kJointCount>& measured,
      const std::array<double, kJointCount>& measured_velocity,
      bool accept_desired, bool new_target = true,
      double target_interval = 0.020, bool terminating = false) {
    Output output;
    output.accepting_desired = accept_desired;
    output.new_target = new_target;
    if (!seeded_) {
      output.position = position_;
      output.state = states_;
      return output;
    }

    if (accept_desired && new_target) {
      const double bounded_interval = std::isfinite(target_interval)
          ? std::clamp(target_interval, limits_.min_target_interval,
                       limits_.max_target_interval)
          : limits_.nominal_target_interval;
      target_interval_ = bounded_interval;
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

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      StepJoint(joint, desired[joint], desired_velocity[joint],
                servo_kp[joint], servo_kd[joint], feedforward_torque[joint],
                measured[joint], measured_velocity[joint], accept_desired,
                new_target, terminating, output);
    }
    measured_velocity_cursor_ =
        (measured_velocity_cursor_ + 1) % measured_velocity_samples_;
    measured_velocity_population_ =
        std::min(measured_velocity_population_ + 1,
                 measured_velocity_samples_);
    output.position = position_;
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
  [[nodiscard]] std::size_t measured_velocity_samples() const {
    return measured_velocity_samples_;
  }
  [[nodiscard]] double measured_acceleration_filter_delay() const {
    return 0.5 * static_cast<double>(measured_velocity_samples_ - 1) *
           limits_.writer_dt;
  }

 private:
  static constexpr double kTolerance = 1.0e-12;

  static double SmoothTaper(double value, double threshold) {
    if (!(threshold > 0.0)) return 0.0;
    const double x = std::clamp(value / threshold, 0.0, 1.0);
    const double smoothstep = x * x * (3.0 - 2.0 * x);
    return 1.0 - smoothstep;
  }

  static double SmoothRise(double value, double lower, double upper) {
    if (!(upper > lower)) return value > lower ? 1.0 : 0.0;
    const double x = std::clamp((value - lower) / (upper - lower), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
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
    const double mean = 0.5 * static_cast<double>(count - 1);
    const double numerator = measured_velocity_weighted_sum_[joint] -
        mean * measured_velocity_sum_[joint];
    return numerator /
        (limits_.writer_dt * measured_velocity_regression_denominator_);
  }

  void CountReasons(std::size_t joint, std::uint32_t reasons) {
    auto& stats = stats_[joint];
    if (HasReason(reasons, LimitReason::kMechanicalPosition))
      ++stats.mechanical_limit_ticks;
    if (HasReason(reasons, LimitReason::kMeasuredVelocity))
      ++stats.measured_velocity_brake_ticks;
    if (HasReason(reasons, LimitReason::kMeasuredAcceleration))
      ++stats.measured_acceleration_brake_ticks;
    if (HasReason(reasons, LimitReason::kStaleTarget))
      ++stats.stale_target_ticks;
    if (HasReason(reasons, LimitReason::kTermination))
      ++stats.termination_ticks;
    if (HasReason(reasons, LimitReason::kServoTorque))
      ++stats.servo_torque_limit_ticks;
    if (HasReason(reasons, LimitReason::kTorqueSlew))
      ++stats.torque_slew_ticks;
    if (HasReason(reasons, LimitReason::kMechanicalBarrier))
      ++stats.mechanical_barrier_ticks;
    if (HasReason(reasons, LimitReason::kVelocityEnergy))
      ++stats.velocity_energy_ticks;
  }

  void MarkFaultOutput(std::size_t joint, Output& output) {
    auto& stats = stats_[joint];
    states_[joint] = JointState::kJointFault;
    output.local_damping[joint] = true;
    AddReason(output, joint, LimitReason::kJointFault);
    ++stats.local_damping_ticks;
    ++stats.fault_ticks;
    CountReasons(joint, output.reasons[joint]);
  }

  void ProjectServoTorque(
      std::size_t joint, double target, double desired_velocity,
      double servo_kp, double servo_kd, double feedforward_torque,
      double measured, double measured_velocity, bool accept_desired,
      bool new_target, bool measured_motion_braking, Output& output) {
    auto& stats = stats_[joint];
    const double dt = limits_.writer_dt;
    if (!std::isfinite(desired_velocity) || !std::isfinite(servo_kp) ||
        !std::isfinite(servo_kd) || !std::isfinite(feedforward_torque) ||
        servo_kp <= kTolerance || servo_kd < 0.0) {
      LatchFault(joint, JointFaultReason::kInvalidDesiredCommand);
      MarkFaultOutput(joint, output);
      return;
    }

    output.target_position[joint] = target;
    output.emitted_velocity_target[joint] = desired_velocity;
    const double damping_and_feedforward =
        servo_kd * (desired_velocity - measured_velocity) +
        feedforward_torque;
    const double raw_torque =
        servo_kp * (target - measured) + damping_and_feedforward;
    output.requested_servo_torque[joint] = raw_torque;
    stats.max_requested_servo_torque =
        std::max(stats.max_requested_servo_torque, std::abs(raw_torque));

    const double torque_limit = limits_.max_servo_torque[joint];
    double torque_goal = std::clamp(raw_torque, -torque_limit, torque_limit);
    if (std::abs(torque_goal - raw_torque) > kTolerance)
      AddReason(output, joint, LimitReason::kServoTorque);

    const double speed_factor = SmoothRise(
        std::abs(measured_velocity),
        limits_.free_measured_velocity[joint],
        limits_.smoothing_measured_velocity[joint]);
    const double normal_ramp_time =
        limits_.low_speed_torque_ramp_time[joint] +
        (limits_.high_speed_torque_ramp_time[joint] -
         limits_.low_speed_torque_ramp_time[joint]) * speed_factor;

    const double velocity_energy_factor = SmoothRise(
        std::abs(measured_velocity),
        limits_.free_measured_velocity[joint],
        limits_.measured_velocity_brake[joint]);
    if (torque_goal * measured_velocity > kTolerance &&
        velocity_energy_factor > kTolerance) {
      const double drive_limit =
          torque_limit * (1.0 - velocity_energy_factor);
      const double bounded = std::copysign(
          std::min(std::abs(torque_goal), drive_limit), torque_goal);
      if (std::abs(bounded - torque_goal) > kTolerance) {
        torque_goal = bounded;
        AddReason(output, joint, LimitReason::kVelocityEnergy);
      }
    }

    bool mechanical_barrier = false;
    for (double direction : {-1.0, 1.0}) {
      const double room = direction > 0.0
          ? limits_.max_position[joint] - measured
          : measured - limits_.min_position[joint];
      const double zone = limits_.mechanical_slowdown_distance[joint];
      const double inward_fraction = zone > 0.0
          ? 1.0 - SmoothTaper(std::max(0.0, room), zone)
          : 1.0;
      const double inward_limit = torque_limit * inward_fraction;
      const double before = torque_goal;
      if (direction > 0.0)
        torque_goal = std::min(torque_goal, inward_limit);
      else
        torque_goal = std::max(torque_goal, -inward_limit);
      mechanical_barrier |= std::abs(torque_goal - before) > kTolerance;

      const double inward_velocity =
          std::max(0.0, direction * measured_velocity);
      const double reaction_room = std::max(
          0.0, room - inward_velocity *
              limits_.protective_braking_ramp_time[joint]);
      const double safe_velocity = std::sqrt(
          2.0 * limits_.braking_acceleration[joint] * reaction_room);
      if (inward_velocity > safe_velocity + kTolerance) {
        const double braking_torque = std::min(
            torque_limit,
            servo_kp * limits_.brake_target_error[joint] +
                servo_kd * inward_velocity);
        const double requested_brake = -direction * braking_torque;
        if (direction > 0.0)
          torque_goal = std::min(torque_goal, requested_brake);
        else
          torque_goal = std::max(torque_goal, requested_brake);
        mechanical_barrier = true;
      }
    }
    if (mechanical_barrier) {
      AddReason(output, joint, LimitReason::kMechanicalPosition);
      AddReason(output, joint, LimitReason::kMechanicalBarrier);
      AddReason(output, joint, LimitReason::kStoppingDistance);
      torque_smoothing_active_[joint] = true;
    }

    const double ramp_time = (measured_motion_braking || mechanical_barrier)
        ? std::min(normal_ramp_time,
                   limits_.protective_braking_ramp_time[joint])
        : normal_ramp_time;
    const double previous_torque = emitted_servo_torque_[joint];
    const double requested_step = torque_goal - previous_torque;
    const double free_step = limits_.free_torque_step[joint] *
        (1.0 - 0.75 * speed_factor);
    output.torque_smoothing_factor[joint] = SmoothRise(
        std::abs(requested_step), free_step,
        std::max(free_step + kTolerance, 3.0 * free_step));

    if (std::abs(requested_step) > free_step || !accept_desired ||
        measured_motion_braking || mechanical_barrier) {
      torque_smoothing_active_[joint] = true;
    }

    double emitted_torque = torque_goal;
    if (torque_smoothing_active_[joint]) {
      const double maximum_step =
          torque_limit * dt / std::max(ramp_time, dt);
      emitted_torque = previous_torque +
          std::clamp(requested_step, -maximum_step, maximum_step);
      if (!new_target &&
          std::abs(requested_step) <= free_step + kTolerance &&
          !measured_motion_braking && !mechanical_barrier) {
        emitted_torque = torque_goal;
        torque_smoothing_active_[joint] = false;
      }
      if (std::abs(emitted_torque - torque_goal) > kTolerance)
        AddReason(output, joint, LimitReason::kTorqueSlew);
    }

    double next_position = measured +
        (emitted_torque - damping_and_feedforward) / servo_kp;
    const double bounded_position = std::clamp(
        next_position, limits_.min_position[joint],
        limits_.max_position[joint]);
    if (std::abs(bounded_position - next_position) > kTolerance) {
      next_position = bounded_position;
      AddReason(output, joint, LimitReason::kMechanicalPosition);
      AddReason(output, joint, LimitReason::kMechanicalBarrier);
    }
    // Preserve the exact projected total torque even if the position target
    // had to be clamped at a mechanical bound. The writer publishes these
    // projected velocity/feed-forward fields, not the unverified raw fields.
    const double projected_feedforward = emitted_torque -
        servo_kp * (next_position - measured) -
        servo_kd * (desired_velocity - measured_velocity);
    output.emitted_feedforward_torque[joint] = projected_feedforward;

    const double position_step = next_position - position_[joint];
    position_[joint] = next_position;
    emitted_servo_torque_[joint] = emitted_torque;
    output.emitted_servo_torque[joint] = emitted_torque;

    const std::uint32_t command_reason_mask =
        ReasonBit(LimitReason::kMechanicalPosition) |
        ReasonBit(LimitReason::kStoppingDistance) |
        ReasonBit(LimitReason::kServoTorque) |
        ReasonBit(LimitReason::kTorqueSlew) |
        ReasonBit(LimitReason::kMechanicalBarrier) |
        ReasonBit(LimitReason::kVelocityEnergy);
    const bool command_limited =
        (output.reasons[joint] & command_reason_mask) != 0;
    if (states_[joint] == JointState::kJointFault) {
      MarkFaultOutput(joint, output);
      return;
    }
    if (!accept_desired) {
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
    stats.max_tracking_error = std::max(
        stats.max_tracking_error, std::abs(next_position - measured));
    stats.max_command_position_step = std::max(
        stats.max_command_position_step, std::abs(position_step));
    stats.max_emitted_servo_torque = std::max(
        stats.max_emitted_servo_torque, std::abs(emitted_torque));
    stats.max_servo_torque_step = std::max(
        stats.max_servo_torque_step,
        std::abs(emitted_torque - previous_torque));
    if (!mechanical_barrier && !measured_motion_braking && accept_desired) {
      stats.max_ordinary_servo_torque_step = std::max(
          stats.max_ordinary_servo_torque_step,
          std::abs(emitted_torque - previous_torque));
    }
  }

  void StepJoint(std::size_t joint, double desired, double desired_velocity,
                 double servo_kp, double servo_kd,
                 double feedforward_torque, double measured,
                 double measured_velocity, bool accept_desired,
                 bool new_target, bool terminating, Output& output) {
    auto& stats = stats_[joint];
    ++stats.ticks;
    output.raw_target[joint] = desired;

    if (!std::isfinite(measured)) {
      LatchFault(joint, JointFaultReason::kInvalidMeasuredPosition);
      MarkFaultOutput(joint, output);
      return;
    }
    if (!std::isfinite(measured_velocity)) {
      LatchFault(joint, JointFaultReason::kInvalidMeasuredVelocity);
      MarkFaultOutput(joint, output);
      return;
    }
    if (!std::isfinite(desired)) {
      LatchFault(joint, JointFaultReason::kInvalidDesiredCommand);
      MarkFaultOutput(joint, output);
      return;
    }
    stats.max_measured_velocity =
        std::max(stats.max_measured_velocity, std::abs(measured_velocity));

    bool acceleration_populated = false;
    const double measured_acceleration = EstimateMeasuredAcceleration(
        joint, measured_velocity, acceleration_populated);
    output.measured_acceleration[joint] = measured_acceleration;
    output.measured_acceleration_populated[joint] = acceleration_populated;
    if (acceleration_populated) {
      stats.max_measured_acceleration = std::max(
          stats.max_measured_acceleration, std::abs(measured_acceleration));
    }

    if (std::abs(measured_velocity) > limits_.measured_velocity_fault[joint]) {
      ++overspeed_counts_[joint];
    } else {
      overspeed_counts_[joint] = 0;
    }
    if (acceleration_populated &&
        std::abs(measured_acceleration) >
            limits_.measured_acceleration_fault[joint]) {
      ++overacceleration_counts_[joint];
    } else {
      overacceleration_counts_[joint] = 0;
    }

    const bool velocity_braking =
        std::abs(measured_velocity) > limits_.measured_velocity_brake[joint];
    const bool acceleration_braking = acceleration_populated &&
        std::abs(measured_acceleration) >
            limits_.measured_acceleration_brake[joint];
    const bool measured_motion_braking =
        velocity_braking || acceleration_braking;
    if (velocity_braking)
      AddReason(output, joint, LimitReason::kMeasuredVelocity);
    if (acceleration_braking)
      AddReason(output, joint, LimitReason::kMeasuredAcceleration);

    if (accept_desired && new_target) {
      target_[joint] = desired;
    } else if (!accept_desired) {
      target_[joint] = measured;
      AddReason(output, joint, terminating
          ? LimitReason::kTermination
          : LimitReason::kStaleTarget);
    }

    double target = target_[joint];
    if (states_[joint] == JointState::kJointFault ||
        measured_motion_braking) {
      const double motion = std::abs(measured_velocity) > kTolerance
          ? measured_velocity
          : measured_acceleration;
      const double direction =
          motion > 0.0 ? 1.0 : motion < 0.0 ? -1.0 : 0.0;
      target = measured - direction * limits_.brake_target_error[joint];
    }

    const double bounded_target = std::clamp(
        target, limits_.min_position[joint], limits_.max_position[joint]);
    if (std::abs(bounded_target - target) > kTolerance)
      AddReason(output, joint, LimitReason::kMechanicalPosition);

    // Stale, terminating, braking, and faulted joints must not retain a stale
    // velocity target or feed-forward term which could oppose the safe hold.
    const bool protective_override = !accept_desired ||
        measured_motion_braking ||
        states_[joint] == JointState::kJointFault;
    ProjectServoTorque(
        joint, bounded_target,
        protective_override ? 0.0 : desired_velocity,
        servo_kp, servo_kd,
        protective_override ? 0.0 : feedforward_torque,
        measured, measured_velocity, accept_desired, new_target,
        measured_motion_braking, output);
  }

  Limits limits_;
  bool seeded_ = false;
  std::size_t measured_velocity_samples_ = 2;
  std::size_t measured_velocity_cursor_ = 0;
  std::size_t measured_velocity_population_ = 0;
  double measured_velocity_regression_denominator_ = 0.0;
  double target_interval_ = 0.020;
  std::array<double, kJointCount> target_{};
  std::array<double, kJointCount> position_{};
  std::array<double, kJointCount> emitted_servo_torque_{};
  std::array<bool, kJointCount> torque_smoothing_active_{};
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
    case JointState::kMeasuredMotionBraking:
      return "measured_motion_braking";
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
    case JointFaultReason::kInvalidDesiredCommand:
      return "invalid_desired_command";
    case JointFaultReason::kMeasuredOverspeed: return "measured_overspeed";
    case JointFaultReason::kMeasuredOveracceleration:
      return "measured_overacceleration";
  }
  return "unknown";
}

}  // namespace sonic::safety
