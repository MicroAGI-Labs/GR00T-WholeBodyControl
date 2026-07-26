#include "per_joint_motion_limiter.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>

namespace {

using sonic::safety::HasReason;
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
    CHECK(output.window_motion[joint] <=
          limits.max_window_velocity[joint] * limits.window_duration +
              tolerance);
    CHECK(output.window_delta_v[joint] <=
          limits.max_window_acceleration[joint] * limits.window_duration +
              tolerance);
    CHECK(output.position[joint] >= limits.min_position[joint] - tolerance);
    CHECK(output.position[joint] <= limits.max_position[joint] + tolerance);
    if (output.state[joint] != JointState::kJointFault) {
      CHECK(std::abs(output.position[joint] - measured[joint]) <=
            limits.max_tracking_error[joint] + tolerance);
    }
  }
}

void TestSeedsContinuousStateFromMeasuredPosition() {
  const auto limits = SharedSafetyLimits();
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

void TestRepeatedWriterReadsDoNotCreateTargetImpulses() {
  auto limits = SharedSafetyLimits();
  limits.max_tracking_error.fill(1.0);
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto measured_velocity = Zeros();
  auto desired = Zeros();
  desired[0] = 0.08;
  CHECK(limiter.Seed(measured));
  auto previous_acceleration = Zeros();
  double previous_position = 0.0;
  for (int tick = 0; tick < 120; ++tick) {
    const bool new_target = tick == 0;
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     new_target, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(output.position[0] >= previous_position - 1.0e-12);
    previous_position = output.position[0];
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  CHECK(limiter.target_stats().new_targets == 1);
  CHECK(limiter.target_stats().repeated_writer_ticks == 119);
  CHECK(limiter.stats()[0].max_command_jerk <= limits.max_jerk[0] + 1.0e-9);
}

void ExerciseTargetCadence(const std::array<int, 8>& update_ticks) {
  auto limits = SharedSafetyLimits();
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
  auto limits = SharedSafetyLimits();
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
    if (output.position[2] > 0.06 + 1.0e-9) {
      std::cerr << "step overshoot tick=" << tick
                << " q=" << output.position[2]
                << " v=" << output.velocity[2]
                << " a=" << output.acceleration[2]
                << " reasons=" << output.reasons[2] << '\n';
    }
    CHECK(output.position[2] <= 0.06 + 1.0e-9);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
  desired[2] = -0.05;
  for (int tick = 0; tick < 700; ++tick) {
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     tick == 0, 0.02);
    CheckInvariants(limits, output, measured, previous_acceleration);
    CHECK(output.position[2] >= -0.05 - 1.0e-9);
    previous_acceleration = output.acceleration;
    measured = output.position;
    measured_velocity = output.velocity;
  }
}

void TestLimitingLevelScalesEveryDynamicCeiling() {
  const auto full = SafetyLimitsForLevel(1.0);
  const auto half = SafetyLimitsForLevel(0.5);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    CHECK(half.min_position[joint] == full.min_position[joint]);
    CHECK(half.max_position[joint] == full.max_position[joint]);
    const auto scaled = [](double half_value, double full_value) {
      return std::abs(half_value - 64.0 * full_value) < 1.0e-10;
    };
    CHECK(scaled(half.max_instant_velocity[joint],
                 full.max_instant_velocity[joint]));
    CHECK(scaled(half.max_acceleration[joint], full.max_acceleration[joint]));
    CHECK(scaled(half.max_jerk[joint], full.max_jerk[joint]));
    CHECK(scaled(half.max_window_velocity[joint],
                 full.max_window_velocity[joint]));
    CHECK(scaled(half.max_window_acceleration[joint],
                 full.max_window_acceleration[joint]));
    CHECK(scaled(half.max_tracking_error[joint],
                 full.max_tracking_error[joint]));
    CHECK(scaled(half.measured_velocity_brake[joint],
                 full.measured_velocity_brake[joint]));
    CHECK(scaled(half.measured_velocity_fault[joint],
                 full.measured_velocity_fault[joint]));
    CHECK(scaled(half.measured_acceleration_brake[joint],
                 full.measured_acceleration_brake[joint]));
    CHECK(scaled(half.measured_acceleration_fault[joint],
                 full.measured_acceleration_fault[joint]));
  }
}

void TestPositionAndDeltaVWindows() {
  auto limits = SharedSafetyLimits();
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

void TestOneJointNeverChangesAnotherJoint() {
  auto limits = SharedSafetyLimits();
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
  auto limits = SharedSafetyLimits();
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
  PerJointMotionLimiter limiter(SharedSafetyLimits());
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
  const auto limits = SharedSafetyLimits();
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
  auto limits = SharedSafetyLimits();
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
  CHECK(std::abs(output.position[4] - measured[4]) < 0.01);
  CHECK(limiter.stats()[4].lifecycle_hold_ticks > 0);
}

void TestRandomizedMillionTickInvariants() {
  auto limits = SafetyLimitsForLevel(0.30);
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
  TestSeedsContinuousStateFromMeasuredPosition();
  TestRepeatedWriterReadsDoNotCreateTargetImpulses();
  TestTargetCadencesAndTrajectories();
  TestNoTargetOvershootOnStepAndReversal();
  TestLimitingLevelScalesEveryDynamicCeiling();
  TestPositionAndDeltaVWindows();
  TestOneJointNeverChangesAnotherJoint();
  TestMeasuredAccelerationFilter();
  TestMeasuredMotionPersistenceAndLocality();
  TestInvalidMeasurementUsesLocalDampingOnly();
  TestTrackingEnvelopeCannotForceCommandJump();
  TestTerminationRejectsTargetsAndConvergesToHold();
  TestRandomizedMillionTickInvariants();
  std::cout << "per_joint_motion_limiter: all tests passed\n";
}
