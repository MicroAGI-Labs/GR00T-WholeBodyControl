#include "per_joint_motion_limiter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using sonic::safety::PerJointMotionLimiter;
using sonic::safety::SharedSafetyLimits;
using sonic::safety::kJointCount;

std::uint64_t Percentile(const std::vector<std::uint64_t>& sorted,
                         double fraction) {
  const auto index = static_cast<std::size_t>(
      fraction * static_cast<double>(sorted.size() - 1));
  return sorted[index];
}

std::string CpuModel() {
  std::ifstream input("/proc/cpuinfo");
  std::string line;
  while (std::getline(input, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const auto key = line.substr(0, colon);
    if (key.find("model name") != std::string::npos ||
        key.find("Hardware") != std::string::npos ||
        key.find("Processor") != std::string::npos) {
      auto value = line.substr(colon + 1);
      value.erase(0, value.find_first_not_of(" \t"));
      if (!value.empty()) return value;
    }
  }
  for (const char* path : {"/proc/device-tree/model",
                           "/sys/devices/virtual/dmi/id/product_name"}) {
    std::ifstream model_input(path, std::ios::binary);
    if (!model_input) continue;
    std::string model((std::istreambuf_iterator<char>(model_input)),
                      std::istreambuf_iterator<char>());
    while (!model.empty() &&
           (model.back() == '\0' || model.back() == '\n' ||
            model.back() == '\r')) {
      model.pop_back();
    }
    if (!model.empty()) return model;
  }
  return "unknown-aarch64-" +
         std::to_string(std::thread::hardware_concurrency()) + "-logical-cpus";
}

std::string JsonEscape(const std::string& text) {
  std::string escaped;
  for (const char character : text) {
    if (character == '\\' || character == '"') escaped.push_back('\\');
    escaped.push_back(character);
  }
  return escaped;
}

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t iterations = 5000000;
  std::string json_path;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--iterations" && index + 1 < argc) {
      iterations = std::stoull(argv[++index]);
    } else if (argument == "--json" && index + 1 < argc) {
      json_path = argv[++index];
    } else {
      std::cerr << "usage: " << argv[0]
                << " [--iterations N] [--json PATH]\n";
      return 2;
    }
  }
  if (iterations == 0) return 2;

  auto limits = SharedSafetyLimits();
  limits.max_tracking_error.fill(1.0);
  limits.measured_acceleration_brake.fill(1.0e6);
  limits.measured_acceleration_fault.fill(1.0e6);
  PerJointMotionLimiter limiter(limits);
  std::array<double, kJointCount> desired{};
  std::array<double, kJointCount> measured{};
  std::array<double, kJointCount> measured_velocity{};
  if (!limiter.Seed(measured)) return 3;

  std::vector<std::uint64_t> durations;
  durations.reserve(iterations);
  std::uint64_t deadline_misses = 0;
  std::uint64_t command_updates = 0;
  volatile double checksum = 0.0;
  for (std::uint64_t tick = 0; tick < iterations; ++tick) {
    const bool new_target = tick % 10 == 0;
    if (new_target) {
      ++command_updates;
      const std::uint64_t phase = (tick / 10) % 400;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const double amplitude = 0.02 + 0.0002 * static_cast<double>(joint);
        if (phase < 100) {
          desired[joint] = amplitude;
        } else if (phase < 200) {
          desired[joint] = -amplitude;
        } else if (phase < 300) {
          desired[joint] = amplitude *
              std::sin(0.02 * static_cast<double>(phase + joint));
        } else {
          desired[joint] = 0.0;
        }
      }
    }
    const auto before = Clock::now();
    const auto output = limiter.Step(desired, measured, measured_velocity, true,
                                     new_target, 0.02);
    const auto after = Clock::now();
    const auto nanoseconds = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(after - before)
            .count());
    durations.push_back(nanoseconds);
    if (nanoseconds >= 2000000) ++deadline_misses;
    measured = output.position;
    measured_velocity = output.velocity;
    checksum = checksum + output.position[tick % kJointCount];
  }
  std::sort(durations.begin(), durations.end());
  std::uint64_t fault_events = 0;
  for (const auto& stats : limiter.stats()) fault_events += stats.fault_events;

#if defined(__clang__)
  const std::string compiler = std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
  const std::string compiler = std::string("gcc ") + __VERSION__;
#else
  const std::string compiler = "unknown";
#endif

  const auto p50 = Percentile(durations, 0.50);
  const auto p95 = Percentile(durations, 0.95);
  const auto p99 = Percentile(durations, 0.99);
  const auto p999 = Percentile(durations, 0.999);
  const auto maximum = durations.back();
  std::string json =
      "{\n"
      "  \"algorithm\": \"continuous-qva-jerk-window-v1\",\n"
      "  \"iterations\": " + std::to_string(iterations) + ",\n" +
      "  \"command_updates\": " + std::to_string(command_updates) + ",\n" +
      "  \"p50_ns\": " + std::to_string(p50) + ",\n" +
      "  \"p95_ns\": " + std::to_string(p95) + ",\n" +
      "  \"p99_ns\": " + std::to_string(p99) + ",\n" +
      "  \"p999_ns\": " + std::to_string(p999) + ",\n" +
      "  \"max_ns\": " + std::to_string(maximum) + ",\n" +
      "  \"deadline_misses\": " + std::to_string(deadline_misses) + ",\n" +
      "  \"fault_events\": " + std::to_string(fault_events) + ",\n" +
      "  \"cpu\": \"" + JsonEscape(CpuModel()) + "\",\n" +
      "  \"compiler\": \"" + JsonEscape(compiler) + "\",\n" +
      "  \"compile_options\": \"-O3 -fno-fast-math\",\n" +
      "  \"checksum\": " + std::to_string(checksum) + "\n"
      "}\n";
  std::cout << json;
  if (!json_path.empty()) {
    std::ofstream output(json_path);
    if (!output) return 4;
    output << json;
  }
  return deadline_misses == 0 && fault_events == 0 && p999 < 100000 ? 0 : 1;
}
