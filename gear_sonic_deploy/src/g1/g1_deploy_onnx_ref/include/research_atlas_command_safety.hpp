#pragma once

// Monorepo-owned final-writer adapter for Sonic's G1 deploy executable.
//
// This header deliberately has no dependency on Sonic's MotorCommand or Unitree
// DDS types.  The deploy writer copies those types into fixed arrays, calls
// Step(), and then fills LowCmd from the returned FinalCommand.  That keeps the
// safety boundary small enough to test separately.
//
// The limiter underneath is the state-dependent torque-projection profile, and
// that changes what this adapter has to carry.  A kinematic limiter bounds the
// position target, so returning the limited position was enough.  A torque
// limiter reconstructs the servo torque implied by kp, kd, q, dq and tau_ff,
// projects THAT into a per-joint envelope, and then emits the q, dq and tau_ff
// triple that reproduces the approved torque at the servo.  Taking only the
// position back would publish a command whose torque is not the one the limiter
// checked, which is the failure this boundary exists to prevent.  So Step()
// takes the gains and the feed-forward torque, and FinalCommand carries all
// three emitted fields.

#include <array>
#include <chrono>
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

// The profile the limiter header defines, named as the deploy binary reports it
// so a manifest entry and a log line cannot disagree.  There is exactly one:
// substituting thresholds changes the closed-loop policy response and destroys
// sim/deployment parity, which is why the header ships a single envelope rather
// than a simulation/commissioning pair.
inline constexpr char kLimiterProfile[] = "simulation-parity-v5";

// Not a fourth profile: the limiter is absent entirely.  This is how the eval
// asks what the controller does with no protection in the path, and how real
// teleop preserves Sonic's original writer fields.
inline constexpr char kUpstreamDirect[] = "upstream-direct";

struct FinalCommand {
  std::array<double, kBodyJointCount> raw_q{};
  std::array<double, kBodyJointCount> raw_dq{};
  std::array<double, kBodyJointCount> raw_tau_ff{};
  std::array<double, kBodyJointCount> command_q{};
  std::array<double, kBodyJointCount> command_dq{};
  std::array<double, kBodyJointCount> command_tau_ff{};
  std::array<double, kBodyJointCount> command_kp{};
  std::array<double, kBodyJointCount> command_kd{};
  std::array<double, kBodyJointCount> measured_q{};
  std::array<double, kBodyJointCount> measured_dq{};
  std::array<double, kBodyJointCount> requested_servo_torque{};
  std::array<double, kBodyJointCount> emitted_servo_torque{};
  std::array<bool, kBodyJointCount> local_damping{};
  bool ready = false;
  bool accepting_desired = false;
};

class CommandSafety {
 public:
  CommandSafety(std::string profile, bool actuating, int telemetry_port)
      : profile_(std::move(profile)), actuating_(actuating) {
    if (profile_ == kUpstreamDirect) {
      // Preserve Sonic's original robot-facing fields. The overlay remains only
      // to mirror the exact final command and to keep hardware shadow incapable
      // of constructing a publisher.
    } else if (profile_ == kLimiterProfile) {
      limiter_ = std::make_unique<::sonic::safety::PerJointMotionLimiter>(
          ::sonic::safety::HardwareSafetyLimits());
    } else {
      throw std::invalid_argument(
          "unknown joint limiter profile '" + profile_ + "'; expected " +
          kUpstreamDirect + " or " + kLimiterProfile);
    }
    if (telemetry_port != 0) {
      if (telemetry_port < 1 || telemetry_port > 65535) {
        throw std::invalid_argument("command telemetry port must be in 1..65535");
      }
      context_ = std::make_unique<zmq::context_t>(1);
      publisher_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::pub);
      publisher_->set(zmq::sockopt::sndhwm, 32);
      publisher_->set(zmq::sockopt::linger, 0);
      publisher_->bind("tcp://*:" + std::to_string(telemetry_port));
    }
  }

  FinalCommand Step(
      const std::array<double, kBodyJointCount>& raw_q,
      const std::array<double, kBodyJointCount>& raw_dq,
      const std::array<double, kBodyJointCount>& raw_tau_ff,
      const std::array<double, kBodyJointCount>& kp,
      const std::array<double, kBodyJointCount>& kd,
      const std::array<double, kBodyJointCount>& measured_q,
      const std::array<double, kBodyJointCount>& measured_dq,
      bool accept_desired,
      bool new_target = true,
      double target_interval = 0.020,
      bool terminating = false) {
    FinalCommand result;
    result.raw_q = raw_q;
    result.raw_dq = raw_dq;
    result.raw_tau_ff = raw_tau_ff;
    result.measured_q = measured_q;
    result.measured_dq = measured_dq;
    result.accepting_desired = accept_desired;

    // The five hybrid command fields and the state used to approve them must
    // all be finite. Telemetry validation is not a substitute for withholding
    // a malformed LowCmd at the writer.
    for (std::size_t i = 0; i < kBodyJointCount; ++i) {
      if (!std::isfinite(raw_q[i]) || !std::isfinite(raw_dq[i]) ||
          !std::isfinite(raw_tau_ff[i]) || !std::isfinite(kp[i]) ||
          !std::isfinite(kd[i]) || !std::isfinite(measured_q[i]) ||
          !std::isfinite(measured_dq[i])) {
        return result;
      }
    }

    if (!limiter_) {
      result.command_q = raw_q;
      result.command_dq = raw_dq;
      result.command_tau_ff = raw_tau_ff;
      result.command_kp = kp;
      result.command_kd = kd;
      result.ready = true;
      return result;
    }

    if (!limiter_->seeded() && !limiter_->Seed(measured_q)) {
      return result;
    }

    const auto limited = limiter_->StepWithServo(
        raw_q, raw_dq, kp, kd, raw_tau_ff, measured_q, measured_dq,
        accept_desired, new_target, target_interval, terminating);

    // All three, together. These reconstruct the torque the limiter approved;
    // publishing the position alone would put an unchecked torque on the wire.
    result.command_q = limited.target_position;
    result.command_dq = limited.emitted_velocity_target;
    result.command_tau_ff = limited.emitted_feedforward_torque;
    result.command_kp = kp;
    result.command_kd = kd;
    result.requested_servo_torque = limited.requested_servo_torque;
    result.emitted_servo_torque = limited.emitted_servo_torque;
    result.local_damping = limited.local_damping;
    for (std::size_t i = 0; i < kBodyJointCount; ++i) {
      if (result.local_damping[i]) {
        result.command_kp[i] = 0.0;
        result.command_kd[i] = 8.0;
      }
    }
    result.ready = true;
    result.accepting_desired = limited.accepting_desired;
    return result;
  }

  void Publish(
      const FinalCommand& command,
      std::uint64_t measured_state_sequence) {
    const std::uint64_t writer_sequence = writer_ticks_++;
    if (!publisher_ || !command.ready) return;

    const auto writer_monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    const std::string limiter_decision = LimiterDecision(command);

    msgpack::sbuffer payload;
    msgpack::packer<msgpack::sbuffer> packer(&payload);
    packer.pack_map(25);
    PackField(packer, "schema_version", 3);
    PackField(packer, "writer_sequence", writer_sequence);
    PackField(packer, "writer_monotonic_ns", writer_monotonic_ns);
    PackField(packer, "measured_state_sequence", measured_state_sequence);
    PackField(packer, "actuating", actuating_);
    // Shadow mode still computes a complete packet, but never constructs the
    // DDS publisher.  would_send names that distinction without claiming an
    // acknowledgement from the actuator bus.
    PackField(packer, "would_send", true);
    // This overlay does not yet own command-buffer expiry or a writer
    // watchdog.  Publishing "unsupported" is deliberately weaker than
    // inventing a healthy state.
    PackField(packer, "expiry_state", std::string("unsupported"));
    PackField(packer, "watchdog_state", std::string("unsupported"));
    PackField(packer, "limiter_enabled", limiter_enabled());
    PackField(packer, "limiter_profile", profile_);
    PackField(packer, "limiter_decision", limiter_decision);
    PackField(packer, "accepting_desired", command.accepting_desired);
    PackArray(packer, "raw_q", command.raw_q);
    PackArray(packer, "raw_dq", command.raw_dq);
    PackArray(packer, "raw_tau_ff", command.raw_tau_ff);
    PackArray(packer, "command_q", command.command_q);
    PackArray(packer, "command_dq", command.command_dq);
    PackArray(packer, "command_tau_ff", command.command_tau_ff);
    PackArray(packer, "command_kp", command.command_kp);
    PackArray(packer, "command_kd", command.command_kd);
    PackArray(packer, "measured_q", command.measured_q);
    PackArray(packer, "measured_dq", command.measured_dq);
    PackArray(packer, "requested_servo_torque", command.requested_servo_torque);
    PackArray(packer, "emitted_servo_torque", command.emitted_servo_torque);
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
  [[nodiscard]] std::string LimiterDecision(
      const FinalCommand& command) const {
    if (!limiter_) return "upstream_direct";
    bool projected = false;
    for (std::size_t i = 0; i < kBodyJointCount; ++i) {
      if (command.local_damping[i]) return "local_damping";
      projected =
          projected || command.command_q[i] != command.raw_q[i] ||
          command.command_dq[i] != command.raw_dq[i] ||
          command.command_tau_ff[i] != command.raw_tau_ff[i];
    }
    return projected ? "projected" : "accepted";
  }

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
};

}  // namespace research_atlas::sonic
