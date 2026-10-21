#pragma once
#include <cstdint>
#include <cstddef>
#include <map>
#include <optional>
#include <stdexcept>

namespace kart_e2e {
// Metadata only. No image copies and no subtraction between ROS and steady clocks.
class LatencyTracker {
 public:
  struct Sample { double ms; bool first; bool warmup; };
  explicit LatencyTracker(size_t maximum) : maximum_(maximum) {
    if (!maximum) throw std::invalid_argument("max_pending must be positive");
  }
  bool input(int64_t stamp, int64_t steady, bool warmup) {
    if (stamp <= 0 || (last_stamp_ && stamp <= last_stamp_)) {
      // A reused stamp also invalidates the earlier candidate. Never pick one arbitrarily.
      pending_.erase(stamp);
      return false;
    }
    last_stamp_ = stamp;
    pending_[stamp] = Entry{steady, warmup, false, false};
    if (pending_.size() > maximum_) pending_.erase(pending_.begin());
    return true;
  }
  std::optional<Sample> observe(int64_t stamp, int64_t steady, bool command) {
    auto it = pending_.find(stamp);
    if (it == pending_.end() || steady < it->second.steady)
      return std::nullopt;
    auto &entry = it->second;
    bool &seen = command ? entry.command : entry.decoded;
    const bool first = !seen;
    seen = true;
    return Sample{double(steady-entry.steady)/1e6, first, entry.warmup};
  }
 private:
  struct Entry { int64_t steady; bool warmup, decoded, command; };
  size_t maximum_;
  int64_t last_stamp_{0};
  std::map<int64_t, Entry> pending_;
};
}  // namespace kart_e2e
