#pragma once
#include <map>
#include <cstdint>
#include <utility>
#include <optional>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
namespace kart_e2e {
// Image-stamped decisions; never substitute a newer section for an old image.
class HeadRouting {
public:
  HeadRouting(int heads, std::string sha, double timeout)
      : heads_(heads), sha_(std::move(sha)), timeout_(timeout) {
    if (heads < 1 || heads > 64 || sha_.empty() || !std::isfinite(timeout) || timeout <= 0)
      throw std::invalid_argument("Invalid head routing contract");
  }
  bool update(int64_t stamp, int index, bool valid, const std::string &sha, double receipt) {
    valid_ = valid && stamp > 0 && index >= 0 && index < heads_ && sha == sha_;
    received_ = receipt;
    states_[stamp] = valid_ ? index : -1;
    while (states_.size() > 256) states_.erase(states_.begin());
    return valid_;
  }
  bool fresh(double now) const {
    return valid_ && now >= received_ && now-received_ <= timeout_;
  }
  std::optional<std::vector<float>> select(int64_t stamp, const std::vector<float> &values,
                                          int width, double now) const {
    if (!fresh(now)) return std::vector<float>{};
    auto found=states_.find(stamp);
    if (found==states_.end()) return std::nullopt;
    if (width<1 || found->second<0 || values.size()!=static_cast<size_t>(heads_*width))
      return std::vector<float>{};
    const auto offset=found->second*width;
    return std::vector<float>(values.begin()+offset,values.begin()+offset+width);
  }
  void reset() { states_.clear(); valid_=false; }
private:
  int heads_;
  std::string sha_;
  double timeout_,received_{0};
  bool valid_{false};
  std::map<int64_t,int> states_;
};
}
