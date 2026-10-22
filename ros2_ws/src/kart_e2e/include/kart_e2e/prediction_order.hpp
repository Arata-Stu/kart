#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>

namespace kart_e2e {
// Metadata only. Decoder serializes access with its existing mutex.
class PredictionOrder {
public:
  struct Check {
    const char *reason;
    int64_t previous;
    bool keep;
  };
  explicit PredictionOrder(const std::string &policy = "observe")
      : drop_(policy == "drop") {
    if (policy != "observe" && policy != "drop")
      throw std::invalid_argument("prediction_order_policy must be observe or drop");
  }
  Check inspect(int64_t stamp) {
    const auto previous = high_water_;
    const char *reason = nullptr;
    if (stamp <= 0) { reason = "invalid_stamp"; ++invalid_; }
    else if (stamp < high_water_) { reason = "out_of_order"; ++backwards_; }
    else if (stamp == high_water_) { reason = "duplicate_stamp"; ++duplicates_; }
    else high_water_ = stamp;
    return {reason, previous, !reason || !drop_};
  }
  // New mode/clock epoch; lifetime counters remain available in warnings.
  void reset() { high_water_ = 0; }
  uint64_t backwards() const { return backwards_; }
  uint64_t duplicates() const { return duplicates_; }
  uint64_t invalid() const { return invalid_; }
private:
  bool drop_;
  int64_t high_water_{0};
  uint64_t backwards_{0}, duplicates_{0}, invalid_{0};
};
}  // namespace kart_e2e
