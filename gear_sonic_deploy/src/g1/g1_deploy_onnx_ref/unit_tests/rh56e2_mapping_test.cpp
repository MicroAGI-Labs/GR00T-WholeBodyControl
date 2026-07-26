#include "rh56e2_mapping.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool Near(double a, double b) { return std::abs(a - b) < 1e-12; }

}  // namespace

int main() {
  using sonic::hands::rh56e2::InspireToPolicy;
  using sonic::hands::rh56e2::PolicyToInspire;

  const std::array<double, 7> policy = {0.0, 0.25, 0.5, 0.75, 1.0, 1.2, 9.0};
  const auto inspire = PolicyToInspire(policy);
  assert(Near(inspire[0], 1.0));
  assert(Near(inspire[1], 0.75));
  assert(Near(inspire[2], 0.5));
  assert(Near(inspire[3], 0.25));
  assert(Near(inspire[4], 0.0));
  assert(Near(inspire[5], 0.0));

  const auto limited = PolicyToInspire(policy, 0.6);
  assert(Near(limited[3], 0.4));
  assert(Near(limited[4], 0.4));

  auto invalid = policy;
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  assert(Near(PolicyToInspire(invalid)[0], 1.0));

  const auto round_trip = InspireToPolicy(inspire);
  assert(Near(round_trip[0], 0.0));
  assert(Near(round_trip[1], 0.25));
  assert(Near(round_trip[2], 0.5));
  assert(Near(round_trip[3], 0.75));
  assert(Near(round_trip[4], 1.0));
  assert(Near(round_trip[5], 1.0));
  assert(Near(round_trip[6], 0.0));

  std::cout << "rh56e2_mapping: all tests passed\n";
  return 0;
}
