#include <gtest/gtest.h>

#include "../include/walk_to_idle_transition.hpp"

namespace {

WalkToIdleCommand Command(int mode, double speed,
                          std::array<double, 3> direction = {1.0, 0.0, 0.0}) {
  WalkToIdleCommand command;
  command.locomotion_mode = mode;
  command.movement_speed = speed;
  command.movement_direction = direction;
  command.facing_direction = {0.0, 1.0, 0.0};
  command.height = 0.74;
  return command;
}

TEST(WalkToIdleTransition, IdleWithoutPriorWalkPassesThrough) {
  WalkToIdleTransition transition;
  const auto idle = Command(0, -1.0, {0.0, 0.0, 0.0});

  const auto result = transition.Update(idle, idle, 0.1);

  EXPECT_EQ(result.locomotion_mode, 0);
  EXPECT_FALSE(transition.active());
}

TEST(WalkToIdleTransition, WalkBrakesThroughSlowWalkBeforeIdle) {
  WalkToIdleTransition transition;
  const auto walk = Command(2, -1.0, {0.6, 0.8, 0.0});
  const auto idle = Command(0, -1.0, {0.0, 0.0, 0.0});

  EXPECT_EQ(transition.Update(walk, Command(0, -1.0), 0.1).locomotion_mode, 2);

  auto result = transition.Update(idle, walk, 0.1);
  ASSERT_TRUE(transition.active());
  EXPECT_EQ(result.locomotion_mode, 2);
  EXPECT_DOUBLE_EQ(result.movement_speed, 0.8);
  EXPECT_EQ(result.movement_direction, walk.movement_direction);
  EXPECT_EQ(result.facing_direction, walk.facing_direction);
  EXPECT_DOUBLE_EQ(result.height, walk.height);

  bool saw_slow_walk = false;
  for (int i = 0; i < 30 && transition.active(); ++i) {
    result = transition.Update(idle, result, 0.1);
    if (result.locomotion_mode == 1) {
      saw_slow_walk = true;
      EXPECT_DOUBLE_EQ(result.movement_speed, 0.1);
      EXPECT_EQ(result.movement_direction, walk.movement_direction);
    }
  }

  EXPECT_TRUE(saw_slow_walk);
  EXPECT_FALSE(transition.active());
  EXPECT_EQ(result.locomotion_mode, 0);
  EXPECT_EQ(result.movement_direction, idle.movement_direction);

  // A held IDLE request must not restart the stop sequence.
  result = transition.Update(idle, result, 0.1);
  EXPECT_EQ(result.locomotion_mode, 0);
  EXPECT_FALSE(transition.active());
}

TEST(WalkToIdleTransition, SlowWalkSkipsFastBrakePhase) {
  WalkToIdleTransition transition;
  const auto slow = Command(1, 0.4);
  const auto idle = Command(0, -1.0, {0.0, 0.0, 0.0});

  transition.Update(slow, Command(0, -1.0), 0.1);
  const auto result = transition.Update(idle, slow, 0.1);

  EXPECT_TRUE(transition.active());
  EXPECT_EQ(transition.phase(), WalkToIdleTransition::Phase::BRAKE_SLOW);
  EXPECT_EQ(result.locomotion_mode, 1);
  EXPECT_DOUBLE_EQ(result.movement_speed, 0.1);
}

TEST(WalkToIdleTransition, NewWalkCommandCancelsPendingStop) {
  WalkToIdleTransition transition;
  const auto walk = Command(2, -1.0);
  const auto idle = Command(0, -1.0, {0.0, 0.0, 0.0});

  transition.Update(walk, Command(0, -1.0), 0.1);
  auto result = transition.Update(idle, walk, 0.1);
  ASSERT_TRUE(transition.active());

  const auto resumed = Command(2, 1.2, {0.0, -1.0, 0.0});
  result = transition.Update(resumed, result, 0.1);

  EXPECT_FALSE(transition.active());
  EXPECT_EQ(result.locomotion_mode, resumed.locomotion_mode);
  EXPECT_EQ(result.movement_direction, resumed.movement_direction);
  EXPECT_DOUBLE_EQ(result.movement_speed, resumed.movement_speed);
}

}  // namespace
