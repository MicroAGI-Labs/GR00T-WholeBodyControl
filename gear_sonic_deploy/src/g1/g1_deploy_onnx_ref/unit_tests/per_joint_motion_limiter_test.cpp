#include "per_joint_motion_limiter.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>

namespace {

using sonic::safety::JointFaultReason;
using sonic::safety::JointState;
using sonic::safety::PerJointMotionLimiter;
using sonic::safety::SimulationQualificationLimits;
using sonic::safety::SupportedCommissioningLimits;
using sonic::safety::kJointCount;

void Check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
  }
}
#define CHECK(expression) Check(static_cast<bool>(expression), #expression, __LINE__)

std::array<double, kJointCount> Zeros() { return {}; }

void TestSeedsFromMeasuredPosition() {
  const auto limits = SimulationQualificationLimits();
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  measured[3] = 0.7;
  CHECK(limiter.Seed(measured));
  auto desired = measured;
  desired[3] = 2.0;
  const auto output = limiter.Step(desired, measured, Zeros(), true);
  const double maximum_first_step = std::min(
      limits.max_instant_velocity[3] * limits.writer_dt,
      limits.max_acceleration[3] * limits.writer_dt * limits.writer_dt);
  CHECK(std::abs(output.position[3] - measured[3]) <=
        maximum_first_step + 1.0e-9);
  CHECK(output.position[3] != desired[3]);
}

void TestRandomizedIndependentInvariants(const sonic::safety::Limits& limits) {
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  CHECK(limiter.Seed(measured));
  std::mt19937_64 random(20260724);
  std::uniform_real_distribution<double> target(-10.0, 10.0);
  auto previous_velocity = Zeros();
  for (int tick = 0; tick < 100000; ++tick) {
    auto desired = Zeros();
    for (double& value : desired) value = target(random);
    const auto output = limiter.Step(desired, measured, Zeros(), true);
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      CHECK(std::abs(output.velocity[joint]) <=
            limits.max_instant_velocity[joint] + 1.0e-9);
      if (std::abs(output.velocity[joint] - previous_velocity[joint]) >
          limits.max_acceleration[joint] * limits.writer_dt + 1.0e-9) {
        std::cerr << "acceleration detail tick=" << tick << " joint=" << joint
                  << " previous_v=" << previous_velocity[joint]
                  << " next_v=" << output.velocity[joint]
                  << " window=" << output.window_motion[joint]
                  << " q=" << output.position[joint] << '\n';
        CHECK(false);
      }
      CHECK(output.window_motion[joint] <=
            limits.max_window_velocity[joint] * limits.window_duration + 1.0e-9);
      CHECK(std::abs(output.position[joint] - measured[joint]) <=
            limits.max_tracking_error[joint] + 1.0e-9);
      previous_velocity[joint] = output.velocity[joint];
    }
  }
}

void TestOneJointBudgetNeverLimitsAnother() {
  auto limits = SimulationQualificationLimits();
  limits.max_window_velocity[0] = 0.01;
  limits.max_window_velocity[1] = 10.0;
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  CHECK(limiter.Seed(measured));
  auto desired = Zeros();
  desired[0] = 1.0;
  desired[1] = 1.0;
  bool joint_zero_window_limited = false;
  for (int tick = 0; tick < 60; ++tick) {
    const auto output = limiter.Step(desired, measured, Zeros(), true);
    joint_zero_window_limited |= output.window_limited[0];
  }
  const auto output = limiter.Step(desired, measured, Zeros(), true);
  CHECK(joint_zero_window_limited);
  CHECK(output.position[1] > output.position[0] + 0.05);
  CHECK(output.state[1] != JointState::kJointFault);
}

void TestOverspeedBrakesOnlyAffectedJoint() {
  const auto limits = SimulationQualificationLimits();
  PerJointMotionLimiter limiter(limits);
  auto measured = Zeros();
  auto velocity = Zeros();
  auto desired = Zeros();
  desired.fill(0.05);
  CHECK(limiter.Seed(measured));
  velocity[26] = limits.measured_velocity_fault[26] + 0.5;
  sonic::safety::Output output;
  for (int sample = 0; sample < limits.overspeed_samples_to_fault; ++sample) {
    output = limiter.Step(desired, measured, velocity, true);
  }
  CHECK(output.state[26] == JointState::kJointFault);
  CHECK(limiter.fault_reasons()[26] == JointFaultReason::kMeasuredOverspeed);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (joint == 26) continue;
    CHECK(output.state[joint] != JointState::kJointFault);
    CHECK(!output.local_damping[joint]);
  }
}

void TestInvalidMeasurementUsesLocalDampingOnly() {
  PerJointMotionLimiter limiter(SimulationQualificationLimits());
  auto measured = Zeros();
  CHECK(limiter.Seed(measured));
  measured[7] = std::numeric_limits<double>::quiet_NaN();
  const auto output = limiter.Step(Zeros(), measured, Zeros(), true);
  CHECK(output.local_damping[7]);
  CHECK(output.state[7] == JointState::kJointFault);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (joint != 7) CHECK(!output.local_damping[joint]);
  }
}

void TestTerminationRejectsTargetsAndMovesTowardMeasuredHold() {
  PerJointMotionLimiter limiter(SimulationQualificationLimits());
  auto measured = Zeros();
  auto desired = Zeros();
  desired[4] = 0.08;
  CHECK(limiter.Seed(measured));
  for (int tick = 0; tick < 20; ++tick) {
    limiter.Step(desired, measured, Zeros(), true);
  }
  const auto before = limiter.Step(desired, measured, Zeros(), true);
  const auto after = limiter.Step(desired, measured, Zeros(), false);
  CHECK(!after.accepting_desired);
  CHECK(std::abs(after.position[4] - measured[4]) <=
        std::abs(before.position[4] - measured[4]));
  CHECK(after.state[4] == JointState::kBraking ||
        after.state[4] == JointState::kRateLimited);
  CHECK(limiter.stats()[4].lifecycle_hold_ticks > 0);
}

}  // namespace

int main() {
  TestSeedsFromMeasuredPosition();
  TestRandomizedIndependentInvariants(SimulationQualificationLimits());
  TestRandomizedIndependentInvariants(SupportedCommissioningLimits());
  TestOneJointBudgetNeverLimitsAnother();
  TestOverspeedBrakesOnlyAffectedJoint();
  TestInvalidMeasurementUsesLocalDampingOnly();
  TestTerminationRejectsTargetsAndMovesTowardMeasuredHold();
  std::cout << "per_joint_motion_limiter: all tests passed\n";
}
