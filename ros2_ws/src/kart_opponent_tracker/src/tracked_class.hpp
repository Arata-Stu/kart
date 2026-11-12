#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace kart_opponent_tracker
{
// A ByteTrack instance associates one class only. Keep the detector's external
// string ID here; ByteTrack's internal numeric label is independent of it.
class TrackedClass
{
public:
  explicit TrackedClass(std::string id = "0", bool enabled = true)
  : id_(std::move(id)), enabled_(enabled)
  {
    if (id_.empty()) {
      throw std::invalid_argument("tracked_class_id must not be empty");
    }
  }

  const std::string & id() const {return id_;}
  bool enabled() const {return enabled_;}

  bool accepts(const std::string & id, double score, double low_score) const
  {
    return enabled_ && id == id_ && std::isfinite(score) && score >= low_score && score <= 1.0;
  }

private:
  std::string id_;
  bool enabled_;
};
}  // namespace kart_opponent_tracker
