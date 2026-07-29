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
  kServoTorque = 1u << 16,
  kTorqueSlew = 1u << 17,
  kMechanicalBarrier = 1u << 18,
  kVelocityEnergy = 1u << 19,

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
  // Maximum measured-to-reference error permitted before initialization may
  // release external support. This is tighter than the runtime braking
  // envelope and is configured per joint so load-bearing leg error is not
  // treated as equivalent to stored end-effector energy.
  std::array<double, kJointCount> prime_tracking_error{};
  // Distance from the actual modeled hard stop over which commanded velocity
  // is smoothly tapered. This does not inset or alter the mechanical range.
  std::array<double, kJointCount> mechanical_slowdown_distance{};
  // Minimum causal time constant for converging to a held target. This avoids
  // treating each 50 Hz sample as a 2 ms endpoint while remaining independent
  // of future samples (no interpolation).
  std::array<double, kJointCount> minimum_convergence_time{};
  std::array<double, kJointCount> measured_velocity_brake{};
  std::array<double, kJointCount> measured_velocity_fault{};
  std::array<double, kJointCount> measured_acceleration_brake{};
  std::array<double, kJointCount> measured_acceleration_fault{};
  std::array<double, kJointCount> brake_target_error{};
  // Parameters for the phase-preserving servo safety projection.  Small
  // changes in equivalent PD torque pass through exactly. Larger changes, or
  // changes made while the measured joint is moving quickly, are slewed in
  // torque space. This bounds the physical effect of a position-target jump
  // without continuously filtering every SONIC target.
  std::array<double, kJointCount> max_servo_torque{};
  std::array<double, kJointCount> free_torque_step{};
  std::array<double, kJointCount> free_measured_velocity{};
  std::array<double, kJointCount> smoothing_measured_velocity{};
  std::array<double, kJointCount> low_speed_torque_ramp_time{};
  std::array<double, kJointCount> high_speed_torque_ramp_time{};
  std::array<double, kJointCount> protective_braking_ramp_time{};
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
  // The absolute-position and absolute-delta-v command windows were retired
  // after the shared simulator ablation showed that they prevented stable
  // SONIC balance. Histories remain populated so telemetry can still report
  // the unconstrained activity, and either window can be explicitly re-enabled
  // for an A/B diagnostic without introducing another control implementation.
  bool enforce_position_window = false;
  bool enforce_delta_v_window = false;
  bool enforce_measured_acceleration = true;
  bool transparent_torque_projection = false;
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
  std::uint64_t servo_torque_limit_ticks = 0;
  std::uint64_t torque_slew_ticks = 0;
  std::uint64_t mechanical_barrier_ticks = 0;
  std::uint64_t velocity_energy_ticks = 0;
  double max_command_velocity = 0.0;
  double max_command_acceleration = 0.0;
  double max_command_jerk = 0.0;
  double max_window_motion = 0.0;
  double max_window_delta_v = 0.0;
  double max_measured_velocity = 0.0;
  double max_measured_acceleration = 0.0;
  double max_tracking_error = 0.0;
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
  std::array<double, kJointCount> requested_servo_torque{};
  std::array<double, kJointCount> emitted_servo_torque{};
  std::array<double, kJointCount> torque_smoothing_factor{};
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
  // These are modeled mechanical ranges, not inset software safety envelopes.
  // In particular, both the MuJoCo G1 model and Unitree's 29-DoF URDF define
  // right_elbow_joint (index 25) as [-1.0472, 2.0944] rad.  Do not silently
  // narrow that range here; any desired soft-limit behavior should be modeled
  // explicitly and reported separately from MECHANICAL_POSITION.
  // The physical Thor-connected G1 reports wrist roll down to -1.9778 rad at
  // rest, slightly beyond the MuJoCo +/-1.97222 range. Indices 19 and 26 use
  // the minimal symmetric physical envelope of +/-1.98 rad.
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
  // Always-on energy envelope. Small, low-speed corrections use an acceleration
  // ceiling which can be unwound inside the burst velocity/distance region;
  // larger corrections may use the sustained balance authority. Per-joint
  // values reflect the very
  // different balance authority and end-effector risk of legs and arms; the
  // algorithm and lifecycle behavior are identical for every joint.
  for (std::size_t joint = 0; joint < 12; ++joint) {
    limits.max_instant_velocity[joint] = 30.0;
    limits.peak_acceleration[joint] = 150.0;
    limits.sustained_acceleration[joint] = 1500.0;
    limits.braking_acceleration[joint] = 150.0;
    limits.max_acceleration[joint] = 1500.0;
    limits.burst_distance[joint] = 0.03;
    limits.burst_velocity[joint] = 0.30;
    limits.max_target_offset[joint] = 0.25;
    limits.max_tracking_error[joint] = 0.40;
    limits.prime_tracking_error[joint] = 0.30;
    limits.measured_velocity_brake[joint] = 12.0;
    limits.measured_velocity_fault[joint] = 18.0;
    limits.measured_acceleration_brake[joint] = 1000.0;
    limits.measured_acceleration_fault[joint] = 1600.0;
    limits.max_jerk[joint] = 75000.0;
    limits.mechanical_slowdown_distance[joint] = 0.15;
    limits.minimum_convergence_time[joint] = 0.030;
  }
  for (std::size_t joint = 12; joint <= 14; ++joint) {
    limits.max_instant_velocity[joint] = 15.0;
    limits.peak_acceleration[joint] = 80.0;
    limits.sustained_acceleration[joint] = 600.0;
    limits.braking_acceleration[joint] = 80.0;
    limits.max_acceleration[joint] = 600.0;
    limits.burst_distance[joint] = 0.05;
    limits.burst_velocity[joint] = 0.35;
    limits.max_target_offset[joint] = 0.15;
    limits.max_tracking_error[joint] = 0.50;
    limits.prime_tracking_error[joint] = 0.20;
    limits.measured_velocity_brake[joint] = 6.0;
    limits.measured_velocity_fault[joint] = 10.0;
    limits.measured_acceleration_brake[joint] = 300.0;
    limits.measured_acceleration_fault[joint] = 500.0;
    limits.max_jerk[joint] = 20000.0;
    limits.mechanical_slowdown_distance[joint] = 0.12;
    limits.minimum_convergence_time[joint] = 0.040;
  }
  for (std::size_t joint = 15; joint < kJointCount; ++joint) {
    const bool wrist = (joint >= 19 && joint <= 21) || joint >= 26;
    limits.max_instant_velocity[joint] = wrist ? 2.0 : 3.0;
    limits.peak_acceleration[joint] = 30.0;
    limits.sustained_acceleration[joint] = 20.0;
    limits.braking_acceleration[joint] = 30.0;
    limits.max_acceleration[joint] = 30.0;
    limits.burst_distance[joint] = 0.08;
    limits.burst_velocity[joint] = 0.30;
    limits.max_target_offset[joint] = wrist ? 0.08 : 0.10;
    limits.max_tracking_error[joint] = wrist ? 0.20 : 0.25;
    limits.prime_tracking_error[joint] = wrist ? 0.12 : 0.15;
    limits.measured_velocity_brake[joint] = wrist ? 2.0 : 4.0;
    limits.measured_velocity_fault[joint] = wrist ? 5.0 : 7.0;
    limits.measured_acceleration_brake[joint] = 150.0;
    limits.measured_acceleration_fault[joint] = 300.0;
    limits.max_jerk[joint] = 2000.0;
    limits.mechanical_slowdown_distance[joint] = wrist ? 0.08 : 0.10;
    limits.minimum_convergence_time[joint] = 0.080;
  }
  // Experimental phase-preserving envelope. These effort values match the
  // motor families used by policy_parameters.hpp. Ramp times are simulation
  // study values, not approved hardware limits.
  limits.max_servo_torque = {
      139., 139., 88., 139., 25., 25.,
      139., 139., 88., 139., 25., 25.,
      88., 25., 25.,
      25., 25., 25., 25., 25., 5., 5.,
      25., 25., 25., 25., 25., 5., 5.,
  };
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
    limits.free_torque_step[joint] =
        0.10 * limits.max_servo_torque[joint];
  for (std::size_t joint = 0; joint < 12; ++joint) {
    limits.free_measured_velocity[joint] = 2.0;
    limits.smoothing_measured_velocity[joint] = 8.0;
    limits.low_speed_torque_ramp_time[joint] = 0.020;
    limits.high_speed_torque_ramp_time[joint] = 0.040;
    limits.protective_braking_ramp_time[joint] = 0.020;
  }
  for (std::size_t joint = 12; joint <= 14; ++joint) {
    limits.free_measured_velocity[joint] = 1.0;
    limits.smoothing_measured_velocity[joint] = 4.0;
    limits.low_speed_torque_ramp_time[joint] = 0.030;
    limits.high_speed_torque_ramp_time[joint] = 0.060;
    limits.protective_braking_ramp_time[joint] = 0.025;
  }
  for (std::size_t joint = 15; joint < kJointCount; ++joint) {
    const bool wrist = (joint >= 19 && joint <= 21) || joint >= 26;
    limits.free_measured_velocity[joint] = 0.30;
    limits.smoothing_measured_velocity[joint] = wrist ? 1.0 : 1.5;
    limits.low_speed_torque_ramp_time[joint] = wrist ? 0.100 : 0.080;
    limits.high_speed_torque_ramp_time[joint] = wrist ? 0.150 : 0.120;
    limits.protective_braking_ramp_time[joint] = wrist ? 0.050 : 0.040;
  }
  limits.transparent_torque_projection = true;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
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

// Level zero remains an explicit simulator-only diagnostic bypass. Every
// positive level selects the exact same always-on envelope: safety ceilings
// must not become weaker because an operator asks for more task authority.
inline Limits SafetyLimitsForLevel(double level) {
  (void)level;
  return SharedSafetyLimits();
}

// Deprecated compatibility functions. Historical task-authority settings may
// still be present in scripts, but they can no longer weaken any safety value.
inline void ApplyTaskAuthorityLevels(Limits& limits, double base_level,
                                     double waist_authority_level,
                                     double right_arm_authority_level) {
  (void)limits;
  (void)base_level;
  (void)waist_authority_level;
  (void)right_arm_authority_level;
}

inline void ApplyTaskAuthorityLevel(Limits& limits, double base_level,
                                    double task_authority_level) {
  ApplyTaskAuthorityLevels(limits, base_level, task_authority_level,
                           task_authority_level);
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
    emitted_servo_torque_.fill(0.0);
    torque_smoothing_active_.fill(false);
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
    std::array<double, kJointCount> desired_velocity{};
    std::array<double, kJointCount> servo_kp{};
    std::array<double, kJointCount> servo_kd{};
    std::array<double, kJointCount> feedforward_torque{};
    servo_kp.fill(1.0);
    return StepWithServo(desired, desired_velocity, servo_kp, servo_kd,
                         feedforward_torque, measured, measured_velocity,
                         accept_desired, new_target, target_interval);
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
      StepJoint(joint, desired[joint], desired_velocity[joint],
                servo_kp[joint], servo_kd[joint], feedforward_torque[joint],
                measured[joint], measured_velocity[joint], accept_desired,
                new_target, output);
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

    const double mean_index = 0.5 * static_cast<double>(count - 1);
    const double numerator = measured_velocity_weighted_sum_[joint] -
        mean_index * measured_velocity_sum_[joint];
    return numerator /
        (limits_.writer_dt * measured_velocity_regression_denominator_);
  }

  // Distance travelled in a chosen direction while applying maximum opposing
  // jerk until either velocity reaches zero or -max_acceleration is reached,
  // followed by constant braking. A negative directed velocity is treated as
  // zero: this conservatively covers acceleration that can reverse a reference
  // which is currently moving away from the boundary.
  static double JerkLimitedStoppingDistance(double velocity,
                                            double directed_acceleration,
                                            double max_acceleration,
                                            double max_jerk) {
    velocity = std::max(0.0, velocity);
    const double acceleration =
        std::clamp(directed_acceleration, -max_acceleration, max_acceleration);
    if (velocity <= kTolerance && acceleration <= 0.0) return 0.0;
    const double velocity_zero_time =
        (acceleration +
         std::sqrt(acceleration * acceleration + 2.0 * max_jerk * velocity)) /
        max_jerk;
    const double acceleration_limit_time =
        std::max(0.0, (acceleration + max_acceleration) / max_jerk);
    const double ramp_time =
        std::min(velocity_zero_time, acceleration_limit_time);
    const double ramp_distance = std::max(
        0.0, velocity * ramp_time +
                 0.5 * acceleration * ramp_time * ramp_time -
                 max_jerk * ramp_time * ramp_time * ramp_time / 6.0);
    if (velocity_zero_time <= acceleration_limit_time + kTolerance)
      return ramp_distance;
    const double ramp_velocity = std::max(
        0.0, velocity + acceleration * ramp_time -
                 0.5 * max_jerk * ramp_time * ramp_time);
    return ramp_distance +
        ramp_velocity * ramp_velocity / (2.0 * max_acceleration);
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
    if (HasReason(reasons, LimitReason::kServoTorque))
      ++stats.servo_torque_limit_ticks;
    if (HasReason(reasons, LimitReason::kTorqueSlew))
      ++stats.torque_slew_ticks;
    if (HasReason(reasons, LimitReason::kMechanicalBarrier))
      ++stats.mechanical_barrier_ticks;
    if (HasReason(reasons, LimitReason::kVelocityEnergy))
      ++stats.velocity_energy_ticks;
  }

  void StepJointTorqueProjection(
      std::size_t joint, double target, double desired_velocity,
      double servo_kp, double servo_kd, double feedforward_torque,
      double measured, double measured_velocity, double measured_acceleration,
      bool accept_desired, bool new_target, bool faulted,
      bool measured_motion_braking, double previous_position,
      double previous_velocity, double previous_acceleration, Output& output) {
    auto& stats = stats_[joint];
    const double dt = limits_.writer_dt;
    if (!std::isfinite(desired_velocity) || !std::isfinite(servo_kp) ||
        !std::isfinite(servo_kd) || !std::isfinite(feedforward_torque) ||
        servo_kp <= kTolerance || servo_kd < 0.0) {
      LatchFault(joint, JointFaultReason::kInvalidDesiredPosition);
      output.local_damping[joint] = true;
      AddReason(output, joint, LimitReason::kJointFault);
      ++stats.local_damping_ticks;
      ++stats.fault_ticks;
      CountReasons(joint, output.reasons[joint]);
      return;
    }

    output.target_position[joint] = target;
    const double damping_and_feedforward =
        servo_kd * (desired_velocity - measured_velocity) +
        feedforward_torque;
    const double raw_servo_torque =
        servo_kp * (target - measured) + damping_and_feedforward;
    output.requested_servo_torque[joint] = raw_servo_torque;
    stats.max_requested_servo_torque = std::max(
        stats.max_requested_servo_torque, std::abs(raw_servo_torque));

    const double torque_limit = limits_.max_servo_torque[joint];
    double torque_goal =
        std::clamp(raw_servo_torque, -torque_limit, torque_limit);
    if (std::abs(torque_goal - raw_servo_torque) > kTolerance) {
      AddReason(output, joint, LimitReason::kServoTorque);
      output.acceleration_limited[joint] = true;
    }

    const double speed_factor = SmoothRise(
        std::abs(measured_velocity),
        limits_.free_measured_velocity[joint],
        limits_.smoothing_measured_velocity[joint]);
    const double normal_ramp_time =
        limits_.low_speed_torque_ramp_time[joint] +
        (limits_.high_speed_torque_ramp_time[joint] -
         limits_.low_speed_torque_ramp_time[joint]) * speed_factor;

    // Prevent a command from continuing to add mechanical energy as measured
    // speed approaches the protective-braking threshold. Low-speed commands
    // remain exact; only torque with the same sign as measured velocity is
    // tapered. Opposing torque is never reduced by this drive-energy barrier.
    const double velocity_energy_factor = SmoothRise(
        std::abs(measured_velocity),
        limits_.free_measured_velocity[joint],
        limits_.measured_velocity_brake[joint]);
    if (torque_goal * measured_velocity > kTolerance &&
        velocity_energy_factor > kTolerance) {
      const double drive_torque_limit =
          torque_limit * (1.0 - velocity_energy_factor);
      const double energy_bounded_torque = std::copysign(
          std::min(std::abs(torque_goal), drive_torque_limit), torque_goal);
      if (std::abs(energy_bounded_torque - torque_goal) > kTolerance) {
        torque_goal = energy_bounded_torque;
        AddReason(output, joint, LimitReason::kVelocityEnergy);
        output.acceleration_limited[joint] = true;
      }
    }

    // Mechanical protection is a barrier on the equivalent servo torque, not
    // a second target trajectory. Inward authority tapers smoothly to zero at
    // either modeled hard stop. If measured speed has exhausted the
    // jerk/ramp-aware stopping room, request bounded outward braking early.
    bool mechanical_barrier = false;
    for (double direction : {-1.0, 1.0}) {
      const double room = direction > 0.0
          ? limits_.max_position[joint] - measured
          : measured - limits_.min_position[joint];
      const double zone = limits_.mechanical_slowdown_distance[joint];
      const double inward_fraction = zone > 0.0
          ? 1.0 - SmoothTaper(std::max(0.0, room), zone)
          : 1.0;
      const double inward_torque_limit = torque_limit * inward_fraction;
      const double before_taper = torque_goal;
      if (direction > 0.0)
        torque_goal = std::min(torque_goal, inward_torque_limit);
      else
        torque_goal = std::max(torque_goal, -inward_torque_limit);
      mechanical_barrier |=
          std::abs(torque_goal - before_taper) > kTolerance;

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
      output.tracking_limited[joint] = true;
      torque_smoothing_active_[joint] = true;
    }
    const double ramp_time =
        (measured_motion_braking || mechanical_barrier)
            ? std::min(normal_ramp_time,
                       limits_.protective_braking_ramp_time[joint])
            : normal_ramp_time;

    const double previous_torque = emitted_servo_torque_[joint];
    const double requested_torque_step = torque_goal - previous_torque;
    const double free_step = limits_.free_torque_step[joint] *
        (1.0 - 0.75 * speed_factor);
    const double torque_step_factor = SmoothRise(
        std::abs(requested_torque_step), free_step,
        std::max(free_step + kTolerance, 3.0 * free_step));
    const double smoothing_factor =
        std::max(speed_factor * torque_step_factor, torque_step_factor);
    output.torque_smoothing_factor[joint] = smoothing_factor;

    if (std::abs(requested_torque_step) > free_step ||
        !accept_desired || measured_motion_braking || mechanical_barrier) {
      torque_smoothing_active_[joint] = true;
    }

    double emitted_torque = torque_goal;
    if (torque_smoothing_active_[joint]) {
      const double maximum_torque_step =
          torque_limit * dt / std::max(ramp_time, dt);
      emitted_torque = previous_torque + std::clamp(
          requested_torque_step, -maximum_torque_step,
          maximum_torque_step);
      if (!new_target &&
          std::abs(requested_torque_step) <= free_step + kTolerance) {
        // The final small discontinuity is explicitly inside the transparent
        // low-energy allowance; landing exactly removes residual phase lag.
        emitted_torque = torque_goal;
        torque_smoothing_active_[joint] = false;
      }
      if (std::abs(emitted_torque - torque_goal) > kTolerance) {
        AddReason(output, joint, LimitReason::kTorqueSlew);
        output.jerk_limited[joint] = true;
      }
    }

    // Reconstruct the position target which produces the projected equivalent
    // torque under the exact Kp/Kd/feed-forward command sent to the servo.
    double next_position = measured +
        (emitted_torque - damping_and_feedforward) / servo_kp;
    const double mechanically_bounded = std::clamp(
        next_position, limits_.min_position[joint],
        limits_.max_position[joint]);
    if (std::abs(mechanically_bounded - next_position) > kTolerance) {
      next_position = mechanically_bounded;
      AddReason(output, joint, LimitReason::kMechanicalPosition);
      AddReason(output, joint, LimitReason::kMechanicalBarrier);
      output.tracking_limited[joint] = true;
    }
    emitted_torque = servo_kp * (next_position - measured) +
        damping_and_feedforward;

    const double next_velocity =
        (next_position - previous_position) / dt;
    const double next_acceleration =
        (next_velocity - previous_velocity) / dt;
    const double position_motion =
        std::abs(next_position - previous_position);
    const double delta_v = std::abs(next_velocity - previous_velocity);
    position_[joint] = next_position;
    velocity_[joint] = next_velocity;
    acceleration_[joint] = next_acceleration;
    emitted_servo_torque_[joint] = emitted_torque;
    output.emitted_servo_torque[joint] = emitted_torque;
    window_history_[joint][window_cursor_] = position_motion;
    delta_v_window_history_[joint][delta_v_window_cursor_] = delta_v;
    window_sum_[joint] += position_motion;
    delta_v_window_sum_[joint] += delta_v;

    const std::uint32_t command_reason_mask =
        ReasonBit(LimitReason::kMechanicalPosition) |
        ReasonBit(LimitReason::kServoTorque) |
        ReasonBit(LimitReason::kTorqueSlew) |
        ReasonBit(LimitReason::kMechanicalBarrier) |
        ReasonBit(LimitReason::kVelocityEnergy) |
        ReasonBit(LimitReason::kStoppingDistance);
    const bool command_limited =
        (output.reasons[joint] & command_reason_mask) != 0;
    if (faulted || states_[joint] == JointState::kJointFault) {
      states_[joint] = JointState::kJointFault;
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
    stats.max_command_velocity = std::max(
        stats.max_command_velocity, std::abs(next_velocity));
    stats.max_command_acceleration = std::max(
        stats.max_command_acceleration, std::abs(next_acceleration));
    stats.max_command_jerk = std::max(
        stats.max_command_jerk,
        std::abs((next_acceleration - previous_acceleration) / dt));
    stats.max_window_motion = std::max(
        stats.max_window_motion, window_sum_[joint]);
    stats.max_window_delta_v = std::max(
        stats.max_window_delta_v, delta_v_window_sum_[joint]);
    stats.max_tracking_error = std::max(
        stats.max_tracking_error, std::abs(next_position - measured));
    stats.max_emitted_servo_torque = std::max(
        stats.max_emitted_servo_torque, std::abs(emitted_torque));
    stats.max_servo_torque_step = std::max(
        stats.max_servo_torque_step,
        std::abs(emitted_torque - previous_torque));
    if (!mechanical_barrier && !measured_motion_braking && accept_desired &&
        !faulted &&
        !HasReason(output.reasons[joint], LimitReason::kMechanicalPosition)) {
      stats.max_ordinary_servo_torque_step = std::max(
          stats.max_ordinary_servo_torque_step,
          std::abs(emitted_torque - previous_torque));
    }
    (void)measured_acceleration;
  }

  void StepJoint(std::size_t joint, double desired, double desired_velocity,
                 double servo_kp, double servo_kd,
                 double feedforward_torque, double measured,
                 double measured_velocity, bool accept_desired,
                 bool new_target, Output& output) {
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
    if (limits_.transparent_torque_projection) {
      StepJointTorqueProjection(
          joint, target, desired_velocity, servo_kp, servo_kd,
          feedforward_torque, measured, measured_velocity,
          measured_acceleration, accept_desired, new_target, faulted,
          measured_motion_braking, previous_position, previous_velocity,
          previous_acceleration, output);
      return;
    }
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
    const double configured_braking_acceleration =
        limits_.braking_acceleration[joint];
    const double max_acceleration = limits_.max_acceleration[joint];
    const double configured_peak_jerk = limits_.max_jerk[joint];
    const double distance_factor =
        SmoothTaper(std::abs(error), limits_.burst_distance[joint]);
    const double velocity_factor =
        SmoothTaper(std::abs(previous_velocity), limits_.burst_velocity[joint]);
    const double low_energy_factor = distance_factor * velocity_factor;
    double drive_acceleration_limit = sustained_acceleration +
        (peak_acceleration - sustained_acceleration) * low_energy_factor;
    // Drive and ordinary braking share the same energy taper. A large target
    // jump cannot command a high-energy acceleration, and a reversal at speed
    // cannot command a much harsher deceleration. The stopping-distance logic
    // sees this reduced braking authority and therefore begins braking sooner.
    // A measured-motion violation may use the configured emergency authority
    // because the unsafe physical energy already exists and must be arrested.
    const double energy_aware_braking_acceleration = sustained_acceleration +
        (configured_braking_acceleration - sustained_acceleration) *
            low_energy_factor;
    const double braking_acceleration =
        (faulted || measured_motion_braking)
            ? configured_braking_acceleration
            : energy_aware_braking_acceleration;
    // Load-bearing legs reach sustained acceleration within one 50 Hz SONIC
    // interval, the waist within 30 ms, and arms within 100 ms.
    const double sustained_ramp_time =
        joint < 12 ? 0.020 : joint <= 14 ? 0.030 : 0.100;
    const double sustained_jerk = std::min(
        configured_peak_jerk,
        std::max(1.0, sustained_acceleration / sustained_ramp_time));
    const double max_jerk =
        (faulted || measured_motion_braking)
            ? configured_peak_jerk
            : sustained_jerk +
                  (configured_peak_jerk - sustained_jerk) * low_energy_factor;
    output.burst_distance_factor[joint] = distance_factor;
    output.burst_velocity_factor[joint] = velocity_factor;

    // SONIC's held 50 Hz sample is a causal goal, never an endpoint deadline.
    // The unconstrained request is recorded for telemetry; q_out remains a
    // continuous 500 Hz state and approaches it through the governor below.
    const double requested_velocity = error / dt;
    output.requested_velocity[joint] = requested_velocity;

    const double full_position_budget = limits_.enforce_position_window
        ? limits_.max_window_velocity[joint] * limits_.window_duration
        : std::numeric_limits<double>::infinity();
    const double normal_position_budget =
        full_position_budget * limits_.normal_window_fraction;
    const double full_position_remaining =
        std::max(0.0, full_position_budget - window_sum_[joint]);
    const double normal_position_remaining =
        std::max(0.0, normal_position_budget - window_sum_[joint]);
    const double full_delta_v_budget = limits_.enforce_delta_v_window
        ? limits_.max_window_acceleration[joint] *
              limits_.delta_v_window_duration
        : std::numeric_limits<double>::infinity();
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
    // Never credit burst braking when deciding that a velocity is safe. Burst
    // authority disappears as speed rises; viability must remain valid under
    // the sustained deceleration and sustained jerk that are guaranteed at
    // high energy.
    const double stopping_safe_velocity =
        std::sqrt(2.0 * sustained_acceleration * stopping_room);
    // A held 50 Hz target is not a request to eliminate all reference error in
    // one 2 ms writer tick. Use the observed target cadence as the fastest
    // causal convergence horizon. This is a first-order governor over the held
    // target, not interpolation between current and future SONIC samples.
    const double cadence_safe_velocity =
        std::abs(error) /
        std::max({target_interval_, limits_.minimum_convergence_time[joint],
                  dt});
    const double mechanical_room_raw =
        error > 0.0 ? limits_.max_position[joint] - previous_position
                    : previous_position - limits_.min_position[joint];
    const double mechanical_room = std::max(
        0.0, mechanical_room_raw - std::abs(previous_velocity) * dt -
                 0.5 * std::abs(previous_acceleration) * dt * dt);
    const double mechanical_zone =
        limits_.mechanical_slowdown_distance[joint];
    const double mechanical_profile_factor = mechanical_zone > 0.0
        ? 1.0 - SmoothTaper(mechanical_room, mechanical_zone)
        : 1.0;
    const double mechanical_profile_velocity =
        max_velocity * mechanical_profile_factor;
    const double mechanical_stopping_velocity =
        std::sqrt(2.0 * sustained_acceleration * mechanical_room);
    const double mechanical_safe_velocity =
        std::min(mechanical_profile_velocity, mechanical_stopping_velocity);
    // Reduce inward drive authority together with velocity as the reference
    // enters the managed hard-stop approach zone. Without this coupling, a
    // small velocity goal can still accumulate a large acceleration state
    // which takes too long to unwind under the jerk bound.
    drive_acceleration_limit *= mechanical_profile_factor;
    output.drive_acceleration_limit[joint] = drive_acceleration_limit;
    double permitted_velocity =
        std::min({max_velocity, stopping_safe_velocity,
                  cadence_safe_velocity, mechanical_safe_velocity});
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
    if (cadence_safe_velocity < max_velocity - kTolerance &&
        std::abs(requested_velocity) > cadence_safe_velocity + kTolerance) {
      AddReason(output, joint, LimitReason::kCommandVelocity);
    }
    if (mechanical_safe_velocity < max_velocity - kTolerance &&
        std::abs(requested_velocity) > mechanical_safe_velocity + kTolerance) {
      AddReason(output, joint, LimitReason::kMechanicalPosition);
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
    double barrier_acceleration_goal =
        std::numeric_limits<double>::quiet_NaN();
    const auto stopping_acceleration_goal =
        [&](double boundary_direction) {
          const double inward_velocity =
              std::max(0.0, boundary_direction * previous_velocity);
          const double magnitude = std::min(
              braking_acceleration,
              std::sqrt(2.0 * sustained_jerk * inward_velocity));
          return -boundary_direction * magnitude;
        };
    if (!target_stopping && std::abs(error) > kTolerance) {
      const double target_direction = std::copysign(1.0, error);
      const double target_stopping_distance = JerkLimitedStoppingDistance(
          target_direction * previous_velocity,
          target_direction * previous_acceleration,
          sustained_acceleration, sustained_jerk);
      const double target_room = std::min(
          std::abs(error), normal_position_remaining);
      // Acceleration itself contains motion energy even at zero velocity. If
      // the jerk-limited excursion would consume the remaining target room,
      // unwind it now. This prevents a small reversal from acquiring a large
      // acceleration state and subsequently running toward a hard stop.
      if (2.0 * target_stopping_distance >=
          std::max(0.0, target_room)) {
        velocity_goal = 0.0;
        target_stopping = true;
        barrier_acceleration_goal =
            stopping_acceleration_goal(target_direction);
        AddReason(output, joint, LimitReason::kStoppingDistance);
      }
    }

    // Evaluate both actual mechanical boundaries independently of the target
    // and current velocity direction. This catches the important case where a
    // joint is still moving away from a stop but its acceleration has already
    // accumulated enough inward energy to reverse and hit it. A 4x distance
    // reserve absorbs writer discretization, target reversals, and changes in
    // the state-dependent jerk ceiling; it does not change the modeled range.
    double most_critical_mechanical_ratio = 0.0;
    double mechanical_brake_direction = 0.0;
    for (double boundary_direction : {-1.0, 1.0}) {
      const double hard_stop_room = boundary_direction > 0.0
          ? limits_.max_position[joint] - previous_position
          : previous_position - limits_.min_position[joint];
      const double hard_stop_distance = JerkLimitedStoppingDistance(
          boundary_direction * previous_velocity,
          boundary_direction * previous_acceleration,
          sustained_acceleration, sustained_jerk);
      const double ratio = hard_stop_distance /
          std::max(hard_stop_room, kTolerance);
      if (4.0 * hard_stop_distance >= std::max(0.0, hard_stop_room) &&
          ratio > most_critical_mechanical_ratio) {
        most_critical_mechanical_ratio = ratio;
        mechanical_brake_direction = -boundary_direction;
      }
    }
    if (mechanical_brake_direction != 0.0) {
      velocity_goal = 0.0;
      target_stopping = true;
      barrier_acceleration_goal =
          stopping_acceleration_goal(-mechanical_brake_direction);
      AddReason(output, joint, LimitReason::kStoppingDistance);
      AddReason(output, joint, LimitReason::kMechanicalPosition);
    }

    // stopping_room above already reserves one writer tick. Re-evaluating a
    // second provisional tick here creates a finite dead band around the
    // target (especially during termination), so the next real 500 Hz tick is
    // the only look-ahead used by this causal governor.
    output.permitted_velocity[joint] = velocity_goal;

    double acceleration_goal = (velocity_goal - previous_velocity) / dt;
    if (std::isfinite(barrier_acceleration_goal))
      acceleration_goal = barrier_acceleration_goal;
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
  std::array<double, kJointCount> emitted_servo_torque_{};
  std::array<bool, kJointCount> torque_smoothing_active_{};
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
