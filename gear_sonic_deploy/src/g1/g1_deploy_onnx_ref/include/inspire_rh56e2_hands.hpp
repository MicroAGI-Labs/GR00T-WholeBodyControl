/**
 * @file inspire_rh56e2_hands.hpp
 * @brief SONIC adapter for bilateral Inspire Robotics RH56E2 hands.
 *
 * This class talks to Unitree's official Inspire service boundary:
 *   command: rt/inspire/cmd   (unitree_go::msg::dds_::MotorCmds_)
 *   state:   rt/inspire/state (unitree_go::msg::dds_::MotorStates_)
 *
 * The service, rather than SONIC, owns the physical serial/CAN/Modbus link.
 * Only q is defined by the public DDS contract. Commands are right hand then
 * left hand, and each hand is little, ring, middle, index, thumb bend, thumb
 * rotation. DDS q is normalized with 0=closed and 1=open.
 */

#ifndef INSPIRE_RH56E2_HANDS_HPP
#define INSPIRE_RH56E2_HANDS_HPP

#include "rh56e2_mapping.hpp"
#include "robot_parameters.hpp"
#include "utils.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class InspireRh56e2Hands {
 public:
  static constexpr int kPolicyChannels =
      static_cast<int>(sonic::hands::rh56e2::kPolicyChannelsPerHand);
  static constexpr int kActuators =
      static_cast<int>(sonic::hands::rh56e2::kActuatorsPerHand);

  void initialize(const std::string& network_interface) {
    if (!network_interface.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(
          0, network_interface.c_str());
    }

    command_publisher_.reset(
        new unitree::robot::ChannelPublisher<
            unitree_go::msg::dds_::MotorCmds_>(
            SonicDdsTopic("rt/inspire/cmd")));
    state_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::MotorStates_>(
            SonicDdsTopic("rt/inspire/state")));
    command_publisher_->InitChannel();
    state_subscriber_->InitChannel(
        [this](const void* message) { onState(message); }, 1);

    left_policy_.fill(0.0);
    right_policy_.fill(0.0);
    std::cout
        << "[RH56E2] backend=inspire-rh56e2 command_topic="
        << SonicDdsTopic("rt/inspire/cmd")
        << " state_topic=" << SonicDdsTopic("rt/inspire/state")
        << " order=right-then-left/little,ring,middle,index,thumb-bend,thumb-rotation"
        << " policy_q=0-open-1-closed dds_q=0-closed-1-open"
        << std::endl;
  }

  void SetMaxCloseRatio(double ratio) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    max_close_ratio_ = std::clamp(ratio, 0.2, 1.0);
  }

  double GetMaxCloseRatio() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return max_close_ratio_;
  }

  void writeOnce() {
    if (!command_publisher_) return;
    std::array<double, kPolicyChannels> right_policy;
    std::array<double, kPolicyChannels> left_policy;
    double max_close_ratio;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      right_policy = right_policy_;
      left_policy = left_policy_;
      max_close_ratio = max_close_ratio_;
    }
    unitree_go::msg::dds_::MotorCmds_ command;
    command.cmds().resize(sonic::hands::rh56e2::kBilateralActuators);
    const auto right = sonic::hands::rh56e2::PolicyToInspire(
        right_policy, max_close_ratio);
    const auto left = sonic::hands::rh56e2::PolicyToInspire(
        left_policy, max_close_ratio);
    for (int i = 0; i < kActuators; ++i) {
      command.cmds()[i].q() = static_cast<float>(right[i]);
      command.cmds()[i + kActuators].q() = static_cast<float>(left[i]);
    }
    command_publisher_->Write(command);
  }

  void setAllJointsCommand(
      bool is_left, const std::array<double, kPolicyChannels>& q) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    (is_left ? left_policy_ : right_policy_) = q;
  }

  void open(bool is_left, double = 0.0, double = 0.0) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    auto& policy = is_left ? left_policy_ : right_policy_;
    policy.fill(0.0);
  }

  void close(bool is_left, double = 0.0, double = 0.0) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    auto& policy = is_left ? left_policy_ : right_policy_;
    policy.fill(max_close_ratio_);
    policy[6] = 0.0;
  }

  void hold(bool is_left, double = 0.0, double = 0.0) {
    const auto state = getState(is_left);
    if (!state || state->motor_state().size() != kPolicyChannels) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    auto& policy = is_left ? left_policy_ : right_policy_;
    for (int i = 0; i < kPolicyChannels; ++i) {
      policy[i] = state->motor_state()[i].q();
    }
  }

  // The public Inspire q-only service has no relax/disable field. Holding the
  // measured pose is the least-surprising stop behavior at this boundary.
  void stop(bool is_left) { hold(is_left); }

  std::shared_ptr<const unitree_hg::msg::dds_::HandState_> getState(
      bool is_left) const {
    const auto state = state_buffer_.GetDataWithTime().data;
    if (!state || state->states().size() <
                      sonic::hands::rh56e2::kBilateralActuators) {
      return nullptr;
    }

    std::array<double, sonic::hands::rh56e2::kActuatorsPerHand> inspire{};
    const int offset = is_left ? kActuators : 0;
    for (int i = 0; i < kActuators; ++i) {
      inspire[i] = state->states()[i + offset].q();
    }
    const auto policy = sonic::hands::rh56e2::InspireToPolicy(inspire);

    auto converted = std::make_shared<unitree_hg::msg::dds_::HandState_>();
    converted->motor_state().resize(kPolicyChannels);
    for (int i = 0; i < kPolicyChannels; ++i) {
      converted->motor_state()[i].q() = static_cast<float>(policy[i]);
      converted->motor_state()[i].dq() = 0.0F;
    }
    return converted;
  }

 private:
  void onState(const void* message) {
    const auto* incoming =
        static_cast<const unitree_go::msg::dds_::MotorStates_*>(message);
    state_buffer_.SetData(*incoming);
  }

  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_>
      command_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_>
      state_subscriber_;
  DataBuffer<unitree_go::msg::dds_::MotorStates_> state_buffer_;
  std::array<double, kPolicyChannels> left_policy_{};
  std::array<double, kPolicyChannels> right_policy_{};
  mutable std::mutex command_mutex_;
  double max_close_ratio_ = 1.0;
};

#endif  // INSPIRE_RH56E2_HANDS_HPP
