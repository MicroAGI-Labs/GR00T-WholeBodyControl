#ifndef WALK_TO_IDLE_TRANSITION_HPP
#define WALK_TO_IDLE_TRANSITION_HPP

#include <algorithm>
#include <array>

/**
 * A lightweight command type used by WalkToIdleTransition.
 *
 * It deliberately does not depend on the planner/TensorRT headers, which keeps
 * the transition state machine independently unit-testable.
 */
struct WalkToIdleCommand {
  int locomotion_mode = 0;
  std::array<double, 3> movement_direction{0.0, 0.0, 0.0};
  std::array<double, 3> facing_direction{1.0, 0.0, 0.0};
  double movement_speed = -1.0;
  double height = -1.0;
};

/**
 * Convert an abrupt WALK -> IDLE request into a short model-native stop.
 *
 * The motion planner has separate WALK, SLOW_WALK and IDLE modes. Jumping
 * directly from a full-speed gait to IDLE asks one generated trajectory and an
 * eight-frame cross-fade to remove all forward momentum. Instead, retain the
 * last direction/facing command, ask WALK for its lowest valid speed, then ask
 * SLOW_WALK for a short creep, and only then issue IDLE once.
 *
 * Time is supplied by the caller in planner/simulation seconds. Consequently
 * the behavior is unchanged by CONTROL_WALL_SCALE or slow-motion playback.
 */
class WalkToIdleTransition {
 public:
  enum class Phase { NONE, BRAKE_WALK, BRAKE_SLOW };

  struct Config {
    int idle_mode = 0;
    int slow_walk_mode = 1;
    int walk_mode = 2;
    int run_mode = 3;
    double walk_brake_seconds = 0.6;
    double slow_brake_seconds = 0.8;
    double walk_brake_speed = 0.8;  // lower edge of WALK's trained range
    double slow_brake_speed = 0.1;  // lower edge of SLOW_WALK's trained range
  };

  WalkToIdleTransition() = default;
  explicit WalkToIdleTransition(Config config) : config_(config) {}

  WalkToIdleCommand Update(const WalkToIdleCommand& requested,
                           const WalkToIdleCommand& last_planner_command,
                           double dt_seconds) {
    const bool idle_requested = requested.locomotion_mode == config_.idle_mode;
    const bool idle_edge = idle_requested && previous_requested_mode_ != config_.idle_mode;

    // A non-idle command always cancels a pending stop immediately. This makes
    // operator overrides responsive and prevents a stale transition completing
    // after walking has resumed.
    if (!idle_requested) {
      Reset();
      previous_requested_mode_ = requested.locomotion_mode;
      return requested;
    }

    if (!active_ && idle_edge && IsWalking(last_planner_command.locomotion_mode)) {
      active_ = true;
      elapsed_seconds_ = 0.0;
      source_ = last_planner_command;
      // A robot already in SLOW_WALK does not need the WALK braking stage.
      phase_ = source_.locomotion_mode == config_.slow_walk_mode
                   ? Phase::BRAKE_SLOW
                   : Phase::BRAKE_WALK;
      if (phase_ == Phase::BRAKE_SLOW) {
        elapsed_seconds_ = config_.walk_brake_seconds;
      }
    }

    previous_requested_mode_ = requested.locomotion_mode;
    if (!active_) {
      return requested;
    }

    WalkToIdleCommand effective = source_;
    if (elapsed_seconds_ < config_.walk_brake_seconds) {
      phase_ = Phase::BRAKE_WALK;
      effective.locomotion_mode = config_.walk_mode;
      effective.movement_speed = config_.walk_brake_speed;
    } else if (elapsed_seconds_ < config_.walk_brake_seconds + config_.slow_brake_seconds) {
      phase_ = Phase::BRAKE_SLOW;
      effective.locomotion_mode = config_.slow_walk_mode;
      effective.movement_speed = config_.slow_brake_speed;
    } else {
      Reset();
      return requested;
    }

    elapsed_seconds_ += std::max(0.0, dt_seconds);
    return effective;
  }

  bool active() const { return active_; }
  Phase phase() const { return phase_; }
  double elapsed_seconds() const { return elapsed_seconds_; }

  void Reset() {
    active_ = false;
    phase_ = Phase::NONE;
    elapsed_seconds_ = 0.0;
  }

 private:
  bool IsWalking(int mode) const {
    return mode == config_.slow_walk_mode || mode == config_.walk_mode ||
           mode == config_.run_mode;
  }

  Config config_;
  bool active_ = false;
  int previous_requested_mode_ = 0;
  double elapsed_seconds_ = 0.0;
  Phase phase_ = Phase::NONE;
  WalkToIdleCommand source_{};
};

#endif  // WALK_TO_IDLE_TRANSITION_HPP
