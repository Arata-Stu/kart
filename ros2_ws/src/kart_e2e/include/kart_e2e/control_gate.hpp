#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace kart_e2e {
struct Decision {
  double steering{0}, throttle{0};
  const char *status;
  bool stop{false};
};
class ControlGate {
public:
  ControlGate(bool steer_only, double fixed, double maximum, double timeout,
              double mode_timeout)
      : steer_only_(steer_only), fixed_(fixed), maximum_(maximum),
        timeout_(timeout), mode_timeout_(mode_timeout) {
    if (!std::isfinite(fixed) || !std::isfinite(maximum) ||
        !std::isfinite(timeout) || !std::isfinite(mode_timeout) || fixed < 0 ||
        fixed > maximum || maximum > 1 || timeout <= 0 || mode_timeout <= 0)
      throw std::invalid_argument("Invalid E2E throttle limits/timeouts");
  }
  void transition() { entered_ = -1; }
  void fault() { latched_ = true; }
  Decision step(double now, int mode, double mode_stamp,
                const std::vector<float> &prediction, double image_stamp,
                bool competing = false) {
    bool fresh = mode_stamp > 0 && now >= mode_stamp &&
                 now - mode_stamp <= mode_timeout_;
    if (fresh && (mode == 2 || mode == 3 || mode == 4)) {
      entered_ = -1;
      latched_ = false;
      return {0, 0, "inactive", false};
    }
    if (!fresh || mode != 1)
      return fail("mode_unavailable");
    if (latched_)
      return fail("fault_latched");
    if (entered_ < 0)
      entered_ = now;
    if (competing)
      return fail("competing_publisher");
    if (now - entered_ < .1)
      return {0, 0, "auto_entry_neutral", false};
    const auto expected = steer_only_ ? 1u : 2u;
    if (prediction.size() != expected || image_stamp < entered_ ||
        now < image_stamp || now - image_stamp > timeout_ ||
        !std::all_of(
            prediction.begin(), prediction.end(),
            [](float v) { return std::isfinite(v); }) ||
        prediction[0] < -1 || prediction[0] > 1 ||
        (!steer_only_ && (prediction[1] < 0 || prediction[1] > 1)))
      return fail("invalid_or_expired_prediction");
    return {prediction[0],
            std::min(maximum_, steer_only_ ? fixed_ : double(prediction[1])),
            "active", false};
  }

private:
  Decision fail(const char *status) {
    latched_ = true;
    return {0, 0, status, true};
  }
  bool steer_only_, latched_{false};
  double fixed_, maximum_, timeout_, mode_timeout_, entered_{-1};
};
} // namespace kart_e2e
