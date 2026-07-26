#ifndef RH56E2_MAPPING_HPP
#define RH56E2_MAPPING_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace sonic::hands::rh56e2 {

inline constexpr std::size_t kPolicyChannelsPerHand = 7;
inline constexpr std::size_t kActuatorsPerHand = 6;
inline constexpr std::size_t kBilateralActuators = 12;

// Both orders are little, ring, middle, index, thumb bend, thumb rotation.
// CloudWalk/SONIC uses 0=open, 1=closed and has a seventh unused channel.
// Unitree's rt/inspire service uses 0=closed, 1=open and six channels.
inline std::array<double, kActuatorsPerHand> PolicyToInspire(
    const std::array<double, kPolicyChannelsPerHand>& policy,
    double max_close_ratio = 1.0) {
  const double close_limit = std::clamp(max_close_ratio, 0.0, 1.0);
  std::array<double, kActuatorsPerHand> inspire{};
  for (std::size_t i = 0; i < inspire.size(); ++i) {
    const double close = std::isfinite(policy[i])
        ? std::clamp(policy[i], 0.0, close_limit)
        : 0.0;
    inspire[i] = 1.0 - close;
  }
  return inspire;
}

inline std::array<double, kPolicyChannelsPerHand> InspireToPolicy(
    const std::array<double, kActuatorsPerHand>& inspire) {
  std::array<double, kPolicyChannelsPerHand> policy{};
  for (std::size_t i = 0; i < inspire.size(); ++i) {
    const double open = std::isfinite(inspire[i])
        ? std::clamp(inspire[i], 0.0, 1.0)
        : 1.0;
    policy[i] = 1.0 - open;
  }
  policy[6] = 0.0;
  return policy;
}

}  // namespace sonic::hands::rh56e2

#endif  // RH56E2_MAPPING_HPP
