#include "per_joint_motion_limiter.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>

namespace {

using sonic::safety::HasReason;
using sonic::safety::ApplyTaskAuthorityLevel;
using sonic::safety::ApplyTaskAuthorityLevels;
using sonic::safety::JointFaultReason;
using sonic::safety::JointState;
using sonic::safety::LimitReason;
using sonic::safety::Limits;
using sonic::safety::Output;
using sonic::safety::PerJointMotionLimiter;
using sonic::safety::SafetyLimitsForLevel;
using sonic::safety::SharedSafetyLimits;
using sonic::safety::kJointCount;

void Check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
  }
}
#define CHECK(expression) Check(static_cast<bool>(expression), #expression, __LINE__)

std::array<double, kJointCount> Zeros() { return {}; }

Limits LegacySafetyLimits() {
  auto limits = SharedSafetyLimits();
  limits.transparent_torque_projection = false;
  return limits;
}

Limits LegacySafetyLimitsForLevel(double level) {
  auto limits = SafetyLimitsForLevel(level);
  limits.transparent_torque_projection = false;
  return limits;
}

void CheckInvariants(const Limits& limits, const Output& output,
                     const std::array<double, kJointCount>& measured,
                     const std::array<double, kJointCount>& previous_acceleration,
                     double tolerance = 2.0e-9) {
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    CHECK(std::isfinite(output.position[joint]));
    CHECK(std::isfinite(output.velocity[joint]));
    CHECK(std::isfinite(output.acceleration[joint]));
    CHECK(std::abs(output.velocity[joint]) <=
          limits.max_instant_velocity[joint] + tolerance);
    CHECK(std::abs(output.acceleration[joint]) <=
          limits.max_acceleration[joint] + tolerance);
    CHECK(std::abs(output.acceleration[joint] - previous_acceleration[joint]) <=
          limits.max_jerk[joint] * limits.writer_dt + tolerance);
    if (limits.enforce_position_window) {
      CHECK(output.window_motion[joint] <=
            limits.max_window_velocity[joint] * limits.window_duration +
                tolerance);
    }
    if (limits.enforce_delta_v_window) {
      CHECK(output.window_delta_v[joint] <=
            limits.max_window_acceleration[joint] *
                    limits.delta_v_window_duration +
                tolerance);
    }
    CHECK(output.position[joint] >= limits.min_position[joint] - tolerance);
    CHECK(output.position[joint] <= limits.max_position[joint] + tolerance);
    if (output.state[joint] != JointState::kJointFault) {
      CHECK(std::abs(output.position[joint] - measured[joint]) <=
            limits.max_tracking_error[joint] + tolerance);
    }
  }
}

Output StepWithUnitServo(
    PerJointMotionLimiter& limiter,
    const std::array<double, kJointCount>& desired,
    const std::array<double, kJointCount>& measured,
    const std::array<double, kJointCount>& measured_velocity,
    const std::array<double, kJointCount>& kp,
    bool new_target = true) {
  return limiter.StepWithServo(
      desired, Zeros(), kp, Zeros(), Zeros(), measured, measured_velocity,
      true, new_target, 0.02);
}

void TestTransparentProjectionPassesSmallLowSpeedTargetExactly() {
  const auto limits = SharedSafetyLimits();
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto desired = Zeros();
  auto kp = Zeros();
  kp.fill(10.0);
  desired[22] = 0.10;  // 1 Nm: inside the 2.5 Nm free step.
  CHECK(limiter.Seed(measured));
  auto output =
      StepWithUnitServo(limiter, desired, measured, Zeros(), kp);
  CHECK(std::abs(output.position[22] - desired[22]) < 1.0e-12);
  CHECK(std::abs(output.emitted_servo_torque[22] - 1.0) < 1.0e-12);
  CHECK(!HasReason(output.reasons[22], LimitReason::kTorqueSlew));
  CHECK(output.state[22] == JointState::kFollowing);
}

void TestTransparentProjectionSmoothsLargeRestSnap() {
  const auto limits = SharedSafetyLimits();
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto desired = Zeros();
  auto kp = Zeros();
  kp.fill(10.0);
  desired[22] = 1.0;
  CHECK(limiter.Seed(measured));
  auto output =
      StepWithUnitServo(limiter, desired, measured, Zeros(), kp);
  const double maximum_first_step =
      limits.max_servo_torque[22] * limits.writer_dt /
      limits.low_speed_torque_ramp_time[22];
  CHECK(std::abs(output.emitted_servo_torque[22]) <=
        maximum_first_step + 1.0e-12);
  CHECK(output.position[22] > measured[22]);
  CHECK(output.position[22] < desired[22]);
  CHECK(HasReason(output.reasons[22], LimitReason::kTorqueSlew));
  CHECK(output.state[22] == JointState::kCommandLimited);
  double previous_torque = output.emitted_servo_torque[22];
  bool reached_target = false;
  for (int tick = 0; tick < 100; ++tick) {
    output = StepWithUnitServo(
        limiter, desired, measured, Zeros(), kp, false);
    const double torque_step =
        std::abs(output.emitted_servo_torque[22] - previous_torque);
    CHECK(torque_step <= std::max(
        limits.free_torque_step[22], maximum_first_step) + 1.0e-12);
    previous_torque = output.emitted_servo_torque[22];
    reached_target |=
        std::abs(output.position[22] - desired[22]) < 1.0e-12;
  }
  CHECK(reached_target);
}

void TestTransparentProjectionSmoothsSmallStepOnlyAtHighSpeed() {
  const auto limits = SharedSafetyLimits();
  auto measured = Zeros();
  auto desired = Zeros();
  auto kp = Zeros();
  kp.fill(10.0);
  desired[22] = 0.10;  // 1 Nm.

  PerJointMotionLimiter low_speed(limits);
  CHECK(low_speed.Seed(measured));
  const auto low =
      StepWithUnitServo(low_speed, desired, measured, Zeros(), kp);
  CHECK(std::abs(low.position[22] - desired[22]) < 1.0e-12);

  auto high_velocity = Zeros();
  high_velocity[22] = limits.smoothing_measured_velocity[22];
  PerJointMotionLimiter high_speed(limits);
  CHECK(high_speed.Seed(measured));
  const auto high = StepWithUnitServo(
      high_speed, desired, measured, high_velocity, kp);
  CHECK(high.position[22] < desired[22]);
  CHECK(std::abs(high.emitted_servo_torque[22]) <
        std::abs(low.emitted_servo_torque[22]));
  CHECK(HasReason(high.reasons[22], LimitReason::kTorqueSlew));
}

void TestTransparentProjectionBrakesBeforeMechanicalStop() {
  const auto limits = SharedSafetyLimits();
  constexpr std::size_t joint = 25;
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto kp = Zeros();
  kp.fill(10.0);
  measured[joint] = limits.max_position[joint] - 0.01;
  measured_velocity[joint] = 1.0;
  desired = measured;
  desired[joint] = limits.max_position[joint];
  PerJointMotionLimiter limiter(limits);
  CHECK(limiter.Seed(measured));
  const auto output = StepWithUnitServo(
      limiter, desired, measured, measured_velocity, kp);
  CHECK(output.emitted_servo_torque[joint] < 0.0);
  CHECK(HasReason(output.reasons[joint], LimitReason::kMechanicalBarrier));
  CHECK(HasReason(output.reasons[joint], LimitReason::kStoppingDistance));
  CHECK(output.position[joint] <= limits.max_position[joint]);
}

void TestTransparentProjectionBrakesWristBeforeFaultSpeed() {
  const auto limits = SharedSafetyLimits();
  constexpr std::size_t joint = 27;
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto kp = Zeros();
  kp.fill(10.0);
  measured_velocity[joint] = 2.2;
  PerJointMotionLimiter limiter(limits);
  CHECK(limiter.Seed(measured));
  const auto output = StepWithUnitServo(
      limiter, desired, measured, measured_velocity, kp);
  CHECK(output.emitted_servo_torque[joint] < 0.0);
  CHECK(output.state[joint] == JointState::kMeasuredMotionBraking);
  CHECK(HasReason(output.reasons[joint], LimitReason::kMeasuredVelocity));
  CHECK(limiter.fault_reasons()[joint] == JointFaultReason::kNone);
}

void TestTransparentProjectionTapersTorqueBeforeWristBrakeSpeed() {
  const auto limits = SharedSafetyLimits();
  constexpr std::size_t joint = 27;
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto kp = Zeros();
  kp.fill(10.0);
  measured_velocity[joint] = 1.9;
  desired[joint] = 0.20;
  PerJointMotionLimiter limiter(limits);
  CHECK(limiter.Seed(measured));
  const auto output = StepWithUnitServo(
      limiter, desired, measured, measured_velocity, kp);
  CHECK(output.requested_servo_torque[joint] > 1.9);
  CHECK(output.emitted_servo_torque[joint] >= 0.0);
  CHECK(output.emitted_servo_torque[joint] < 0.2);
  CHECK(HasReason(output.reasons[joint], LimitReason::kVelocityEnergy));
  CHECK(output.state[joint] == JointState::kCommandLimited);
}

void TestSeedsContinuousStateFromMeasuredPosition() {
  const auto limits = LegacySafetyLimits();
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  measured[3] = 0.7;
  CHECK(limiter.Seed(measured));
  auto desired = measured;
  desired[3] = 1.0;
  const auto output =
      limiter.Step(desired, measured, Zeros(), true, true, 0.02);
  CHECK(output.position[3] >= measured[3]);
  CHECK(output.position[3] != desired[3]);
  CHECK(std::abs(output.velocity[3]) <=
        limits.max_acceleration[3] * limits.writer_dt + 1.0e-12);
  CHECK(std::abs(output.acceleration[3]) <=
        limits.max_jerk[3] * limits.writer_dt + 1.0e-12);
}

void TestWristRollSeedUsesPhysicalMechanicalRange() {
  const auto limits = SharedSafetyLimits();
  CHECK(std::abs(limits.min_position[19] + 1.98) < 1.0e-12);
  CHECK(std::abs(limits.max_position[19] - 1.98) < 1.0e-12);
  CHECK(std::abs(limits.min_position[26] + 1.98) < 1.0e-12);
  CHECK(std::abs(limits.max_position[26] - 1.98) < 1.0e-12);

  auto measured = Zeros();
  measured[19] = limits.min_position[19];
  measured[26] = limits.max_position[26];
  PerJointMotionLimiter at_limits(limits);
  CHECK(at_limits.Seed(measured));

  measured[19] = limits.min_position[19] - 1.0e-6;
  PerJointMotionLimiter beyond_limit(limits);
  CHECK(!beyond_limit.Seed(measured));
}

void TestMeasuredRelativeTargetCapDoesNotJumpOutput() {
  auto limits = LegacySafetyLimits();
  limits.max_target_offset.fill(1.0);
  limits.max_target_offset[0] = 0.05;
  limits.max_tracking_error.fill(1.0);
  limits.max_instant_velocity.fill(100.0);
  limits.max_acceleration.fill(2.0);
  limits.peak_acceleration.fill(2.0);
  limits.sustained_acceleration.fill(2.0);
  limits.braking_acceleration.fill(2.0);
  limits.max_jerk.fill(1.0e9);
  limits.burst_distance.fill(10.0);
  limits.burst_velocity.fill(10.0);
  limits.max_window_velocity.fill(100.0);
  limits.max_window_acceleration.fill(10100.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto desired = Zeros();
  desired[0] = 1.0;
  CHECK(limiter.Seed(measured));

  auto output = limiter.Step(desired, measured, Zeros(), true, true, 0.02);
  CHECK(std::abs(output.target_position[0] - 0.05) < 1.0e-12);
  CHECK(HasReason(output.reasons[0], LimitReason::kTrackingDistance));
  CHECK(std::abs(output.acceleration[0] - 2.0) < 1.0e-12);
  CHECK(std::abs(output.velocity[0] - 0.004) < 1.0e-12);
  CHECK(std::abs(output.position[0] - 0.000008) < 1.0e-12);

  // The accepted goal follows the measured-position envelope on every writer
  // tick, while the emitted q/v/a state remains continuous.
  measured[0] = 0.02;
  output = limiter.Step(desired, measured, Zeros(), true, false, 0.02);
  CHECK(std::abs(output.target_position[0] - 0.07) < 1.0e-12);
  CHECK(output.position[0] >= 0.000008);
  CHECK(output.position[0] < output.target_position[0]);
}

void TestNewTargetsPreserveReferenceVelocityAndAcceleration() {
  auto limits = LegacySafetyLimits();
  limits.max_target_offset.fill(1.0);
  limits.max_tracking_error.fill(1.0);
  limits.max_instant_velocity.fill(100.0);
  limits.max_acceleration.fill(4.0);
  limits.peak_acceleration.fill(4.0);
  limits.sustained_acceleration.fill(4.0);
  limits.braking_acceleration.fill(4.0);
  limits.max_jerk.fill(100.0);
  limits.max_window_velocity.fill(100.0);
  limits.max_window_acceleration.fill(10100.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  desired[1] = 0.5;
  CHECK(limiter.Seed(measured));

  Output output;
  for (int tick = 0; tick < 10; ++tick) {
    output = limiter.Step(desired, measured, measured_velocity, true,
                          tick == 0, 0.02);
  }
  const double velocity_before_update = output.velocity[1];
  const double acceleration_before_update = output.acceleration[1];
  CHECK(velocity_before_update > 0.0);
  CHECK(acceleration_before_update > 0.0);

  desired[1] = -0.5;
  output = limiter.Step(desired, measured, measured_velocity, true, true, 0.02);
  CHECK(output.velocity[1] != 0.0);
  CHECK(std::abs(output.velocity[1] - velocity_before_update) <=
        limits.max_acceleration[1] * limits.writer_dt + 1.0e-12);
  CHECK(std::abs(output.acceleration[1] - acceleration_before_update) <=
        limits.max_jerk[1] * limits.writer_dt + 1.0e-12);
}

void TestLowEnergyAndSustainedAcceleration() {
  const auto limits = LegacySafetyLimitsForLevel(1.0);
  auto measured = Zeros();
  auto desired = Zeros();

  PerJointMotionLimiter small_motion(limits);
  CHECK(small_motion.Seed(measured));
  desired[0] = 0.01;
  const auto small = small_motion.Step(
      desired, measured, Zeros(), true, true, 0.02);
  CHECK(small.drive_acceleration_limit[0] <
        limits.sustained_acceleration[0]);
  CHECK(small.drive_acceleration_limit[0] >=
        limits.peak_acceleration[0] - 1.0e-9);

  PerJointMotionLimiter large_motion(limits);
  CHECK(large_motion.Seed(measured));
  desired[0] = 0.10;
  const auto large = large_motion.Step(
      desired, measured, Zeros(), true, true, 0.02);
  CHECK(std::abs(large.drive_acceleration_limit[0] - 1500.0) < 1.0e-9);
  CHECK(std::abs(large.acceleration[0] - 150.0) < 1.0e-9);
  CHECK(HasReason(large.reasons[0], LimitReason::kCommandVelocity));
  CHECK(HasReason(large.reasons[0], LimitReason::kSustainedAcceleration));
  CHECK(HasReason(large.reasons[0], LimitReason::kAccelerationSlew));
  CHECK(!HasReason(large.reasons[0], LimitReason::kPeakAcceleration));
}

void TestEmptyDriveBudgetTransitionsToBrakingWithoutFault() {
  auto limits = LegacySafetyLimitsForLevel(1.0);
  limits.enforce_delta_v_window = true;
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto desired = Zeros();
  desired[8] = 0.10;
  CHECK(limiter.Seed(measured));
  for (int tick = 0; tick < 500; ++tick) {
    const auto output = limiter.Step(
        desired, measured, Zeros(), true, tick % 10 == 0, 0.02);
    CHECK(output.state[8] != JointState::kJointFault);
    CHECK(std::abs(output.velocity[8]) <=
          limits.max_instant_velocity[8] + 1.0e-9);
    CHECK(output.window_delta_v[8] <=
          limits.max_window_acceleration[8] *
                  limits.delta_v_window_duration +
              1.0e-9);
  }
}

void TestRepeatedWriterReadsDoNotCreateTargetImpulses() {
  auto limits = LegacySafetyLimits();
  limits.max_tracking_error.fill(1.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  desired[0] = 0.08;
  CHECK(limiter.Seed(measured));
  auto previous_acceleration = Zeros();
  double previous_position = 0.0;
  double maximum_position = 0.0;
  for (int tick = 0; tick < 120; ++tick) {
    const bool new_target = tick == 0;
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     new_target, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(output.position[0] >= -0.005);
    CHECK(output.position[0] <= desired[0] + 0.005);
    maximum_position = std::max(maximum_position, output.position[0]);
    previous_position = output.position[0];
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  CHECK(limiter.target_stats().new_targets == 1);
  CHECK(limiter.target_stats().repeated_writer_ticks == 119);
  CHECK(maximum_position > 0.95 * desired[0]);
  CHECK(limiter.stats()[0].max_command_jerk <= limits.max_jerk[0] + 1.0e-9);
}

void ExerciseTargetCadence(const std::array<int, 8>& update_ticks) {
  auto limits = LegacySafetyLimits();
  limits.max_tracking_error.fill(1.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto previous_acceleration = Zeros();
  CHECK(limiter.Seed(measured));
  int cadence_index = 0;
  int ticks_to_update = 0;
  double logical_time = 0.0;
  for (int tick = 0; tick < 1200; ++tick) {
    bool new_target = false;
    double target_interval = 0.02;
    if (ticks_to_update == 0) {
      const int cadence = update_ticks[cadence_index++ % update_ticks.size()];
      ticks_to_update = cadence;
      target_interval = cadence * limits.writer_dt;
      desired[0] = 0.04 * std::sin(2.0 * logical_time);
      desired[1] = 0.03 * std::cos(1.3 * logical_time);
      new_target = true;
    }
    --ticks_to_update;
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     new_target, target_interval);
    CheckInvariants(limits, output, measured, previous_acceleration);
    if (output.state[0] == JointState::kJointFault ||
        output.state[1] == JointState::kJointFault) {
      std::cerr << "cadence fault tick=" << tick
                << " cadence=" << update_ticks[0]
                << " q0=" << output.position[0]
                << " v0=" << output.velocity[0]
                << " a0=" << output.acceleration[0]
                << " reason0=" << static_cast<int>(limiter.fault_reasons()[0])
                << " q1=" << output.position[1]
                << " v1=" << output.velocity[1]
                << " a1=" << output.acceleration[1]
                << " reason1=" << static_cast<int>(limiter.fault_reasons()[1])
                << '\n';
    }
    CHECK(output.state[0] != JointState::kJointFault);
    CHECK(output.state[1] != JointState::kJointFault);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
    logical_time += limits.writer_dt;
  }
}

void TestTargetCadencesAndTrajectories() {
  ExerciseTargetCadence({50, 50, 50, 50, 50, 50, 50, 50});  // 10 Hz
  ExerciseTargetCadence({10, 10, 10, 10, 10, 10, 10, 10});  // 50 Hz
  ExerciseTargetCadence({5, 5, 5, 5, 5, 5, 5, 5});           // 100 Hz
  ExerciseTargetCadence({7, 13, 9, 12, 8, 11, 10, 14});      // jitter
}

void TestNoTargetOvershootOnStepAndReversal() {
  auto limits = LegacySafetyLimits();
  limits.max_tracking_error.fill(1.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto previous_acceleration = Zeros();
  CHECK(limiter.Seed(measured));
  desired[2] = 0.06;
  for (int tick = 0; tick < 500; ++tick) {
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     tick == 0, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(output.position[2] <= 0.065);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  CHECK(std::abs(measured[2] - 0.06) < 0.005);
  desired[2] = -0.05;
  for (int tick = 0; tick < 700; ++tick) {
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     tick == 0, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(output.position[2] >= -0.055);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  CHECK(std::abs(measured[2] - (-0.05)) < 0.005);
}

void TestEveryPositiveLevelUsesTheSameAlwaysOnEnvelope() {
  const auto full = SafetyLimitsForLevel(1.0);
  const auto sixty = SafetyLimitsForLevel(0.6);
  const auto forty = SafetyLimitsForLevel(0.4);
  constexpr std::size_t kRightElbowJoint = 25;
  CHECK(std::abs(full.min_position[kRightElbowJoint] - (-1.0472)) < 1.0e-12);
  CHECK(std::abs(full.max_position[kRightElbowJoint] - 2.0944) < 1.0e-12);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    CHECK(sixty.min_position[joint] == full.min_position[joint]);
    CHECK(sixty.max_position[joint] == full.max_position[joint]);
    CHECK(sixty.max_instant_velocity[joint] ==
          full.max_instant_velocity[joint]);
    CHECK(forty.max_instant_velocity[joint] ==
          full.max_instant_velocity[joint]);
    CHECK(sixty.max_acceleration[joint] == full.max_acceleration[joint]);
    CHECK(sixty.peak_acceleration[joint] == full.peak_acceleration[joint]);
    CHECK(sixty.sustained_acceleration[joint] ==
          full.sustained_acceleration[joint]);
    CHECK(sixty.braking_acceleration[joint] ==
          full.braking_acceleration[joint]);
    CHECK(sixty.max_jerk[joint] == full.max_jerk[joint]);
    CHECK(sixty.max_target_offset[joint] == full.max_target_offset[joint]);
    CHECK(sixty.max_tracking_error[joint] == full.max_tracking_error[joint]);
    CHECK(sixty.mechanical_slowdown_distance[joint] ==
          full.mechanical_slowdown_distance[joint]);
    CHECK(full.max_tracking_error[joint] > full.max_target_offset[joint]);
    CHECK(full.prime_tracking_error[joint] > 0.0);
    CHECK(full.prime_tracking_error[joint] < full.max_tracking_error[joint]);
    if (joint < 12) {
      CHECK(std::abs(full.max_instant_velocity[joint] - 30.0) < 1.0e-12);
      CHECK(std::abs(full.peak_acceleration[joint] - 150.0) < 1.0e-12);
      CHECK(std::abs(full.sustained_acceleration[joint] - 1500.0) < 1.0e-12);
      CHECK(std::abs(full.braking_acceleration[joint] - 150.0) < 1.0e-12);
    } else if (joint <= 14) {
      CHECK(std::abs(full.max_instant_velocity[joint] - 15.0) < 1.0e-12);
      CHECK(std::abs(full.peak_acceleration[joint] - 80.0) < 1.0e-12);
      CHECK(std::abs(full.sustained_acceleration[joint] - 600.0) < 1.0e-12);
      CHECK(std::abs(full.braking_acceleration[joint] - 80.0) < 1.0e-12);
    } else {
      const bool wrist = (joint >= 19 && joint <= 21) || joint >= 26;
      CHECK(std::abs(full.max_instant_velocity[joint] -
                     (wrist ? 2.0 : 3.0)) < 1.0e-12);
      CHECK(std::abs(full.peak_acceleration[joint] - 30.0) < 1.0e-12);
      CHECK(std::abs(full.sustained_acceleration[joint] - 20.0) < 1.0e-12);
      CHECK(std::abs(full.braking_acceleration[joint] - 30.0) < 1.0e-12);
    }
  }
}

void TestFormerTaskAuthorityOverridesCannotWeakenSafety() {
  const auto base = SafetyLimitsForLevel(0.6);
  auto targeted = base;
  ApplyTaskAuthorityLevel(targeted, 0.6, 0.4);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    CHECK(targeted.max_instant_velocity[joint] ==
          base.max_instant_velocity[joint]);
    CHECK(targeted.peak_acceleration[joint] == base.peak_acceleration[joint]);
    CHECK(targeted.sustained_acceleration[joint] ==
          base.sustained_acceleration[joint]);
    CHECK(targeted.braking_acceleration[joint] ==
          base.braking_acceleration[joint]);
    CHECK(targeted.max_jerk[joint] == base.max_jerk[joint]);
    CHECK(targeted.max_target_offset[joint] == base.max_target_offset[joint]);
    CHECK(targeted.max_tracking_error[joint] ==
          base.max_tracking_error[joint]);
    CHECK(targeted.measured_velocity_brake[joint] ==
          base.measured_velocity_brake[joint]);
    CHECK(targeted.measured_acceleration_fault[joint] ==
          base.measured_acceleration_fault[joint]);
    CHECK(targeted.min_position[joint] == base.min_position[joint]);
    CHECK(targeted.max_position[joint] == base.max_position[joint]);
  }
}

void TestIndependentTaskAuthorityOverridesCannotWeakenSafety() {
  const auto base = SafetyLimitsForLevel(0.6);
  auto targeted = base;
  ApplyTaskAuthorityLevels(targeted, 0.6, 0.4, 0.3);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    CHECK(targeted.max_instant_velocity[joint] ==
          base.max_instant_velocity[joint]);
    CHECK(targeted.max_acceleration[joint] == base.max_acceleration[joint]);
    CHECK(targeted.max_jerk[joint] == base.max_jerk[joint]);
    CHECK(targeted.measured_velocity_fault[joint] ==
          base.measured_velocity_fault[joint]);
  }
}

void TestPositionAndDeltaVWindows() {
  auto limits = LegacySafetyLimits();
  limits.enforce_position_window = true;
  limits.enforce_delta_v_window = true;
  limits.max_tracking_error.fill(1.0);
  limits.max_window_velocity[0] = 0.02;
  limits.max_window_acceleration[1] = 0.22;
  limits.max_instant_velocity[1] = 0.02;
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  desired[0] = 0.2;
  desired[1] = 0.2;
  auto previous_acceleration = Zeros();
  CHECK(limiter.Seed(measured));
  bool position_window_seen = false;
  bool delta_v_window_seen = false;
  for (int tick = 0; tick < 1000; ++tick) {
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     tick == 0, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    position_window_seen |=
        HasReason(output.reasons[0], LimitReason::kPositionWindow) ||
        HasReason(output.reasons[0], LimitReason::kTargetStopping);
    delta_v_window_seen |=
        HasReason(output.reasons[1], LimitReason::kDeltaVWindow) ||
        HasReason(output.reasons[1], LimitReason::kTargetStopping);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  CHECK(position_window_seen);
  CHECK(delta_v_window_seen);
}

void TestCommandWindowsCanBeDisabledIndependently() {
  auto run = [](bool enforce_position, bool enforce_delta_v) {
    auto limits = LegacySafetyLimits();
    limits.min_position.fill(-10.0);
    limits.max_position.fill(10.0);
    limits.max_target_offset.fill(10.0);
    limits.max_tracking_error.fill(10.0);
    limits.max_instant_velocity.fill(10.0);
    limits.peak_acceleration.fill(1000.0);
    limits.sustained_acceleration.fill(1000.0);
    limits.braking_acceleration.fill(1000.0);
    limits.max_acceleration.fill(1000.0);
    limits.max_jerk.fill(1.0e9);
    limits.max_window_velocity.fill(enforce_position ? 1.0e6 : 0.001);
    limits.max_window_acceleration.fill(enforce_delta_v ? 1.0e6 : 0.001);
    limits.enforce_position_window = enforce_position;
    limits.enforce_delta_v_window = enforce_delta_v;
    PerJointMotionLimiter limiter(limits);
    auto measured = Zeros();
    auto measured_velocity = Zeros();
    auto desired = Zeros();
    desired[0] = 0.2;
    CHECK(limiter.Seed(measured));
    bool saw_position_reason = false;
    bool saw_delta_v_reason = false;
    bool exceeded_position_budget = false;
    bool exceeded_delta_v_budget = false;
    for (int tick = 0; tick < 100; ++tick) {
      const auto output = limiter.Step(desired, measured, measured_velocity,
                                       true, tick == 0, 0.02);
      saw_position_reason |=
          HasReason(output.reasons[0], LimitReason::kPositionWindow);
      saw_delta_v_reason |=
          HasReason(output.reasons[0], LimitReason::kDeltaVWindow);
      exceeded_position_budget |= output.window_motion[0] >
          limits.max_window_velocity[0] * limits.window_duration + 1.0e-9;
      exceeded_delta_v_budget |= output.window_delta_v[0] >
          limits.max_window_acceleration[0] *
              limits.delta_v_window_duration + 1.0e-9;
      measured = output.position;
      measured_velocity = output.velocity;
    }
    if (!enforce_position) {
      CHECK(!saw_position_reason);
      CHECK(exceeded_position_budget);
    }
    if (!enforce_delta_v) {
      CHECK(!saw_delta_v_reason);
      CHECK(exceeded_delta_v_budget);
    }
    CHECK(limits.enforce_measured_acceleration);
  };

  run(false, true);
  run(true, false);
  run(false, false);
}

void TestOneJointNeverChangesAnotherJoint() {
  auto limits = LegacySafetyLimits();
  limits.enforce_position_window = true;
  limits.max_tracking_error.fill(1.0);
  limits.max_window_velocity[0] = 0.005;
  PerJointMotionLimiter baseline(limits);
  PerJointMotionLimiter stressed(limits);
  auto baseline_measured = Zeros();
  auto stressed_measured = Zeros();
  auto baseline_velocity = Zeros();
  auto stressed_velocity = Zeros();
  auto baseline_desired = Zeros();
  auto stressed_desired = Zeros();
  baseline_desired[1] = 0.04;
  stressed_desired[0] = 1.0;
  stressed_desired[1] = baseline_desired[1];
  CHECK(baseline.Seed(baseline_measured));
  CHECK(stressed.Seed(stressed_measured));
  for (int tick = 0; tick < 400; ++tick) {
    const auto baseline_output = baseline.Step(
        baseline_desired, baseline_measured, baseline_velocity, true,
        tick == 0, 0.02);
    const auto stressed_output = stressed.Step(
        stressed_desired, stressed_measured, stressed_velocity, true,
        tick == 0, 0.02);
    CHECK(baseline_output.position[1] == stressed_output.position[1]);
    CHECK(baseline_output.velocity[1] == stressed_output.velocity[1]);
    CHECK(baseline_output.acceleration[1] == stressed_output.acceleration[1]);
    CHECK(baseline_output.state[1] == stressed_output.state[1]);
    CHECK(baseline_output.reasons[1] == stressed_output.reasons[1]);
    baseline_measured = baseline_output.position;
    stressed_measured = stressed_output.position;
    baseline_velocity = baseline_output.velocity;
    stressed_velocity = stressed_output.velocity;
  }
  CHECK(stressed.stats()[0].command_limited_ticks > 0);
}

Limits MeasurementTestLimits() {
  auto limits = LegacySafetyLimits();
  limits.min_position.fill(-100.0);
  limits.max_position.fill(100.0);
  limits.max_tracking_error.fill(100.0);
  limits.measured_velocity_brake.fill(1.0e6);
  limits.measured_velocity_fault.fill(1.0e6);
  limits.measured_acceleration_brake.fill(1.0e6);
  limits.measured_acceleration_fault.fill(1.0e6);
  return limits;
}

void TestMeasuredAccelerationFilter() {
  auto limits = MeasurementTestLimits();
  limits.enforce_measured_acceleration = false;
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  CHECK(limiter.Seed(measured));
  Output output;
  for (int tick = 0; tick < 20; ++tick) {
    measured_velocity[0] = 0.4;
    output = limiter.Step(measured, measured, measured_velocity, false, false,
                          0.02);
  }
  CHECK(output.measured_acceleration_populated[0]);
  CHECK(std::abs(output.measured_acceleration[0]) < 1.0e-9);

  PerJointMotionLimiter ramp_limiter(limits);
  measured = Zeros();
  measured_velocity = Zeros();
  CHECK(ramp_limiter.Seed(measured));
  constexpr double acceleration = 3.0;
  for (int tick = 0; tick < 30; ++tick) {
    measured_velocity[0] = acceleration * tick * limits.writer_dt;
    measured[0] += measured_velocity[0] * limits.writer_dt;
    output = ramp_limiter.Step(measured, measured, measured_velocity, false,
                               false, 0.02);
  }
  CHECK(std::abs(output.measured_acceleration[0] - acceleration) < 1.0e-9);

  PerJointMotionLimiter noise_limiter(limits);
  measured = Zeros();
  measured_velocity = Zeros();
  CHECK(noise_limiter.Seed(measured));
  for (int tick = 0; tick < 30; ++tick) {
    measured_velocity[0] = tick % 2 ? 0.01 : -0.01;
    output = noise_limiter.Step(measured, measured, measured_velocity, false,
                                false, 0.02);
  }
  CHECK(std::abs(output.measured_acceleration[0]) < 1.0);
}

void TestMeasuredMotionPersistenceAndLocality() {
  auto limits = MeasurementTestLimits();
  limits.measured_velocity_brake[4] = 0.2;
  limits.measured_velocity_fault[4] = 0.4;
  limits.overspeed_samples_to_fault = 3;
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  CHECK(limiter.Seed(measured));
  measured_velocity[4] = 0.5;
  Output output;
  for (int tick = 0; tick < 2; ++tick) {
    output = limiter.Step(measured, measured, measured_velocity, true,
                          tick == 0, 0.02);
    CHECK(output.state[4] != JointState::kJointFault);
    CHECK(output.state[4] == JointState::kMeasuredMotionBraking);
  }
  output = limiter.Step(measured, measured, measured_velocity, true, false, 0.02);
  CHECK(output.state[4] == JointState::kJointFault);
  CHECK(output.local_damping[4]);
  CHECK(limiter.fault_reasons()[4] == JointFaultReason::kMeasuredOverspeed);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (joint == 4) continue;
    CHECK(output.state[joint] != JointState::kJointFault);
    CHECK(!output.local_damping[joint]);
  }

  limits.measured_acceleration_window = 0.004;
  limits.measured_acceleration_brake[6] = 1.0;
  limits.measured_acceleration_fault[6] = 2.0;
  limits.overacceleration_samples_to_fault = 3;
  limits.enforce_measured_acceleration = true;
  PerJointMotionLimiter acceleration_limiter(limits);
  measured = Zeros();
  measured_velocity = Zeros();
  CHECK(acceleration_limiter.Seed(measured));
  bool acceleration_faulted = false;
  for (int tick = 0; tick < 12; ++tick) {
    measured_velocity[6] += 0.02;
    measured[6] += measured_velocity[6] * limits.writer_dt;
    output = acceleration_limiter.Step(measured, measured, measured_velocity,
                                       true, tick == 0, 0.02);
    acceleration_faulted |= output.state[6] == JointState::kJointFault;
    if (acceleration_faulted) break;
  }
  CHECK(acceleration_faulted);
  CHECK(acceleration_limiter.fault_reasons()[6] ==
        JointFaultReason::kMeasuredOveracceleration);
  CHECK(!output.local_damping[5]);
}

void TestInvalidMeasurementUsesLocalDampingOnly() {
  PerJointMotionLimiter limiter(LegacySafetyLimits());
  auto measured = Zeros();
  CHECK(limiter.Seed(measured));
  measured[7] = std::numeric_limits<double>::quiet_NaN();
  const auto output =
      limiter.Step(Zeros(), measured, Zeros(), true, true, 0.02);
  CHECK(output.local_damping[7]);
  CHECK(output.state[7] == JointState::kJointFault);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (joint != 7) CHECK(!output.local_damping[joint]);
  }
}

void TestTrackingEnvelopeCannotForceCommandJump() {
  auto limits = LegacySafetyLimits();
  limits.max_tracking_error[2] = 0.10;
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  CHECK(limiter.Seed(measured));
  measured[2] = 0.5;
  const auto output =
      limiter.Step(Zeros(), measured, Zeros(), true, true, 0.02);
  CHECK(output.state[2] == JointState::kJointFault);
  CHECK(output.local_damping[2]);
  CHECK(limiter.fault_reasons()[2] ==
        JointFaultReason::kEmergencyTrackingCorrection);
  CHECK(output.position[2] == 0.0);
  CHECK(output.velocity[2] == 0.0);
  CHECK(output.acceleration[2] == 0.0);
}

void TestTerminationRejectsTargetsAndConvergesToHold() {
  auto limits = LegacySafetyLimits();
  limits.max_tracking_error.fill(1.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  desired[4] = 0.05;
  CHECK(limiter.Seed(measured));
  Output output;
  for (int tick = 0; tick < 100; ++tick) {
    output = limiter.Step(desired, measured, measured_velocity, true,
                          tick == 0, 0.02);
    measured = output.position;
    measured_velocity = output.velocity;
  }
  const double before = output.position[4];
  measured_velocity.fill(0.0);
  measured[4] = before - 0.02;
  for (int tick = 0; tick < 300; ++tick) {
    output = limiter.Step(desired, measured, measured_velocity, false, false,
                          0.02);
    CHECK(!output.accepting_desired);
    CHECK(output.state[4] == JointState::kLifecycleHold ||
          output.state[4] == JointState::kJointFault);
    if (output.state[4] == JointState::kJointFault) break;
  }
  CHECK(output.state[4] != JointState::kJointFault);
  if (!(std::abs(output.position[4] - measured[4]) < 0.01)) {
    std::cerr << "termination q=" << output.position[4]
              << " measured=" << measured[4]
              << " v=" << output.velocity[4]
              << " a=" << output.acceleration[4]
              << " reasons=" << output.reasons[4] << '\n';
  }
  CHECK(std::abs(output.position[4] - measured[4]) < 0.01);
  CHECK(limiter.stats()[4].lifecycle_hold_ticks > 0);
}

void TestArmSnapAndReversalRemainEnergyBounded() {
  const auto limits = LegacySafetyLimits();
  constexpr std::size_t joint = 22;  // right shoulder pitch
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto previous_acceleration = Zeros();
  CHECK(limiter.Seed(measured));

  desired[joint] = limits.max_position[joint];
  for (int tick = 0; tick < 300; ++tick) {
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     tick == 0, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(std::abs(output.velocity[joint]) <= 3.0 + 1.0e-9);
    CHECK(std::abs(output.acceleration[joint]) <= 30.0 + 1.0e-9);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }

  desired[joint] = limits.min_position[joint];
  for (int tick = 0; tick < 500; ++tick) {
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     tick == 0, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(std::abs(output.acceleration[joint] -
                   previous_acceleration[joint]) <=
          limits.max_jerk[joint] * limits.writer_dt + 1.0e-9);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  CHECK(limiter.stats()[joint].fault_events == 0);
}

void TestMechanicalApproachSlowsBeforeBothHardStops() {
  const auto limits = LegacySafetyLimits();
  constexpr std::size_t joint = 25;  // right elbow
  const auto exercise = [&](double start, double requested,
                            double boundary_direction) {
    PerJointMotionLimiter limiter(limits);
    auto measured = Zeros();
    auto measured_velocity = Zeros();
    auto desired = Zeros();
    auto previous_acceleration = Zeros();
    measured[joint] = start;
    desired = measured;
    desired[joint] = requested;
    CHECK(limiter.Seed(measured));
    bool mechanical_reason_seen = false;
    double maximum_near_stop_speed = 0.0;
    for (int tick = 0; tick < 1500; ++tick) {
      const auto output = limiter.Step(
          desired, measured, measured_velocity, true, tick == 0, 0.02);
      CheckInvariants(limits, output, measured, previous_acceleration);
      mechanical_reason_seen |=
          HasReason(output.reasons[joint], LimitReason::kMechanicalPosition);
      const double room = boundary_direction > 0.0
          ? limits.max_position[joint] - output.position[joint]
          : output.position[joint] - limits.min_position[joint];
      if (room < 0.02)
        maximum_near_stop_speed = std::max(
            maximum_near_stop_speed,
            boundary_direction * output.velocity[joint]);
      CHECK(output.state[joint] != JointState::kJointFault);
      previous_acceleration = output.acceleration;
      measured = output.position;
      measured_velocity = output.velocity;
    }
    CHECK(mechanical_reason_seen);
    CHECK(maximum_near_stop_speed < 0.25);
  };

  exercise(limits.max_position[joint] - 0.12,
           limits.max_position[joint] + 1.0, 1.0);
  exercise(limits.min_position[joint] + 0.12,
           limits.min_position[joint] - 1.0, -1.0);
}

void TestRandomizedMillionTickInvariants() {
  auto limits = LegacySafetyLimitsForLevel(0.30);
  limits.max_tracking_error.fill(2.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  auto previous_acceleration = Zeros();
  CHECK(limiter.Seed(measured));
  std::mt19937_64 random(20260726);
  std::uniform_real_distribution<double> target(-0.2, 0.2);
  std::uniform_int_distribution<int> cadence(5, 50);
  int ticks_to_update = 0;
  double interval = 0.02;
  for (int tick = 0; tick < 1000000; ++tick) {
    bool new_target = false;
    if (ticks_to_update == 0) {
      ticks_to_update = cadence(random);
      interval = ticks_to_update * limits.writer_dt;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        desired[joint] = std::clamp(
            target(random), limits.min_position[joint],
            limits.max_position[joint]);
      }
      new_target = true;
    }
    --ticks_to_update;
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     new_target, interval);
    CheckInvariants(limits, output, measured, previous_acceleration, 1.0e-7);
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (output.state[joint] == JointState::kJointFault) {
        std::cerr << "random fault tick=" << tick << " joint=" << joint
                  << " reason="
                  << static_cast<int>(limiter.fault_reasons()[joint])
                  << " q=" << output.position[joint]
                  << " measured=" << measured[joint]
                  << " v=" << output.velocity[joint]
                  << " a=" << output.acceleration[joint]
                  << " desired=" << desired[joint]
                  << " window=" << output.window_motion[joint]
                  << " dv_window=" << output.window_delta_v[joint] << '\n';
      }
      CHECK(output.state[joint] != JointState::kJointFault);
    }
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
}

}  // namespace

int main() {
  TestTransparentProjectionPassesSmallLowSpeedTargetExactly();
  TestTransparentProjectionSmoothsLargeRestSnap();
  TestTransparentProjectionSmoothsSmallStepOnlyAtHighSpeed();
  TestTransparentProjectionBrakesBeforeMechanicalStop();
  TestTransparentProjectionBrakesWristBeforeFaultSpeed();
  TestTransparentProjectionTapersTorqueBeforeWristBrakeSpeed();
  TestSeedsContinuousStateFromMeasuredPosition();
  TestWristRollSeedUsesPhysicalMechanicalRange();
  TestMeasuredRelativeTargetCapDoesNotJumpOutput();
  TestNewTargetsPreserveReferenceVelocityAndAcceleration();
  TestLowEnergyAndSustainedAcceleration();
  TestEmptyDriveBudgetTransitionsToBrakingWithoutFault();
  TestRepeatedWriterReadsDoNotCreateTargetImpulses();
  TestTargetCadencesAndTrajectories();
  TestNoTargetOvershootOnStepAndReversal();
  TestEveryPositiveLevelUsesTheSameAlwaysOnEnvelope();
  TestFormerTaskAuthorityOverridesCannotWeakenSafety();
  TestIndependentTaskAuthorityOverridesCannotWeakenSafety();
  TestPositionAndDeltaVWindows();
  TestCommandWindowsCanBeDisabledIndependently();
  TestOneJointNeverChangesAnotherJoint();
  TestMeasuredAccelerationFilter();
  TestMeasuredMotionPersistenceAndLocality();
  TestInvalidMeasurementUsesLocalDampingOnly();
  TestTrackingEnvelopeCannotForceCommandJump();
  TestTerminationRejectsTargetsAndConvergesToHold();
  TestArmSnapAndReversalRemainEnergyBounded();
  TestMechanicalApproachSlowsBeforeBothHardStops();
  TestRandomizedMillionTickInvariants();
  std::cout << "per_joint_motion_limiter: all tests passed\n";
}
