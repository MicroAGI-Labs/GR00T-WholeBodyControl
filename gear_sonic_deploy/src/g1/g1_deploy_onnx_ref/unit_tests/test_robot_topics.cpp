#include <cstdlib>

#include <gtest/gtest.h>

#include "robot_parameters.hpp"

namespace {

class SonicDdsTopicTest : public ::testing::Test {
 protected:
  void TearDown() override { unsetenv("SONIC_TOPIC_PREFIX"); }
};

TEST_F(SonicDdsTopicTest, EmptyPrefixPreservesPhysicalRobotTopics) {
  unsetenv("SONIC_TOPIC_PREFIX");
  EXPECT_EQ(SonicDdsTopic(HG_STATE_TOPIC), "rt/lowstate");
  EXPECT_EQ(SonicDdsTopic(HG_CMD_TOPIC), "rt/lowcmd");
  EXPECT_EQ(SonicDdsTopic(HG_IMU_TORSO), "rt/secondary_imu");
  EXPECT_EQ(SonicDdsTopic("rt/dex3/left/cmd"), "rt/dex3/left/cmd");
}

TEST_F(SonicDdsTopicTest, PrefixIsolatesEveryUnitreeTopic) {
  setenv("SONIC_TOPIC_PREFIX", "rt/sim/g1/1", 1);
  EXPECT_EQ(SonicDdsTopic(HG_STATE_TOPIC), "rt/sim/g1/1/lowstate");
  EXPECT_EQ(SonicDdsTopic(HG_CMD_TOPIC), "rt/sim/g1/1/lowcmd");
  EXPECT_EQ(SonicDdsTopic(HG_IMU_TORSO), "rt/sim/g1/1/secondary_imu");
  EXPECT_EQ(SonicDdsTopic("rt/dex3/right/state"),
            "rt/sim/g1/1/dex3/right/state");
}

TEST_F(SonicDdsTopicTest, TrailingSlashesAreNormalized) {
  setenv("SONIC_TOPIC_PREFIX", "rt/sim/g1/0///", 1);
  EXPECT_EQ(SonicDdsTopic(HG_STATE_TOPIC), "rt/sim/g1/0/lowstate");
}

}  // namespace
