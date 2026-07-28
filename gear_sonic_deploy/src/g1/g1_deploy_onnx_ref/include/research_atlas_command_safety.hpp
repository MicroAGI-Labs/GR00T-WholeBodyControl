#pragma once

// Monorepo-owned final-writer adapter for Sonic's G1 deploy executable.
//
// This header deliberately has no dependency on Sonic's MotorCommand or Unitree
// DDS types.  The deploy writer copies those types into fixed arrays, calls
// Step(), and then fills LowCmd from the returned FinalCommand.  That keeps the
// safety boundary small enough to test separately.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <msgpack.hpp>
#include <zmq.hpp>

#include "per_joint_motion_limiter.hpp"

namespace research_atlas::sonic {

constexpr std::size_t kBodyJointCount = ::sonic::safety::kJointCount;

struct FinalCommand {
  std::array<double, kBodyJointCount> raw_q{};
  std::array<double, kBodyJointCount> command_q{};
  std::array<double, kBodyJointCount> measured_q{};
  std::array<bool, kBodyJointCount> local_damping{};
  bool ready = false;
  bool accepting_desired = false;
};

class CommandSafety {
 public:
  CommandSafety(std::string profile, bool actuating, int telemetry_port)
      : profile_(std::move(profile)), actuating_(actuating) {
    ::sonic::safety::Limits limits;
    if (profile_ == "upstream-direct") {
      // Preserve Sonic's original robot-facing joint positions. The overlay
      // remains only to mirror the exact final command and to keep hardware
      // shadow incapable of constructing a publisher.
    } else if (profile_ == "simulation") {
      limits = ::sonic::safety::SimulationQualificationLimits();
    } else if (profile_ == "supported-commissioning") {
      limits = ::sonic::safety::SupportedCommissioningLimits();
    } else {
      throw std::invalid_argument(
          "unknown joint limiter profile '" + profile_ +
          "'; expected upstream-direct, simulation, or "
          "supported-commissioning");
    }
    if (profile_ != "upstream-direct") {
      limiter_ = std::make_unique<::sonic::safety::PerJointMotionLimiter>(
          std::move(limits));
    }
    if (telemetry_port != 0) {
      if (telemetry_port < 1 || telemetry_port > 65535) {
        throw std::invalid_argument("command telemetry port must be in 1..65535");
      }
      context_ = std::make_unique<zmq::context_t>(1);
      publisher_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::pub);
      publisher_->set(zmq::sockopt::sndhwm, 4);
      publisher_->set(zmq::sockopt::linger, 0);
      publisher_->bind("tcp://*:" + std::to_string(telemetry_port));
    }
  }

  FinalCommand Step(
      const std::array<double, kBodyJointCount>& raw_q,
      const std::array<double, kBodyJointCount>& measured_q,
      const std::array<double, kBodyJointCount>& measured_dq,
      bool accept_desired) {
    FinalCommand result;
    result.raw_q = raw_q;
    result.measured_q = measured_q;
    result.accepting_desired = accept_desired;
    if (!limiter_) {
      for (const double value : raw_q) {
        if (!std::isfinite(value)) return result;
      }
      result.command_q = raw_q;
      result.ready = true;
      return result;
    }
    if (!limiter_->seeded() && !limiter_->Seed(measured_q)) {
      return result;
    }
    const auto limited =
        limiter_->Step(raw_q, measured_q, measured_dq, accept_desired);
    result.command_q = limited.position;
    result.local_damping = limited.local_damping;
    result.ready = true;
    result.accepting_desired = limited.accepting_desired;
    return result;
  }

  void Publish(const FinalCommand& command) {
    if (!publisher_ || !command.ready || (++writer_ticks_ % 10) != 0) return;

    msgpack::sbuffer payload;
    msgpack::packer<msgpack::sbuffer> packer(&payload);
    packer.pack_map(10);
    PackField(packer, "schema_version", 1);
    PackField(packer, "sequence", sequence_++);
    PackField(packer, "actuating", actuating_);
    PackField(packer, "limiter_enabled", limiter_enabled());
    PackField(packer, "limiter_profile", profile_);
    PackField(packer, "accepting_desired", command.accepting_desired);
    PackArray(packer, "raw_q", command.raw_q);
    PackArray(packer, "command_q", command.command_q);
    PackArray(packer, "measured_q", command.measured_q);
    PackArray(packer, "local_damping", command.local_damping);

    static constexpr char kTopic[] = "g1_command";
    zmq::message_t message(sizeof(kTopic) - 1 + payload.size());
    std::memcpy(message.data(), kTopic, sizeof(kTopic) - 1);
    std::memcpy(
        static_cast<char*>(message.data()) + sizeof(kTopic) - 1,
        payload.data(),
        payload.size());
    static_cast<void>(publisher_->send(message, zmq::send_flags::dontwait));
  }

  [[nodiscard]] bool actuating() const { return actuating_; }
  [[nodiscard]] bool limiter_enabled() const {
    return static_cast<bool>(limiter_);
  }
  [[nodiscard]] const std::string& profile() const { return profile_; }

 private:
  template <typename T>
  static void PackField(
      msgpack::packer<msgpack::sbuffer>& packer, const char* name, const T& value) {
    packer.pack(name);
    packer.pack(value);
  }

  template <typename T, std::size_t N>
  static void PackArray(
      msgpack::packer<msgpack::sbuffer>& packer,
      const char* name,
      const std::array<T, N>& values) {
    packer.pack(name);
    packer.pack_array(N);
    for (const auto& value : values) packer.pack(value);
  }

  std::string profile_;
  bool actuating_;
  std::unique_ptr<::sonic::safety::PerJointMotionLimiter> limiter_;
  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> publisher_;
  std::uint64_t writer_ticks_ = 0;
  std::uint64_t sequence_ = 0;
};

}  // namespace research_atlas::sonic
