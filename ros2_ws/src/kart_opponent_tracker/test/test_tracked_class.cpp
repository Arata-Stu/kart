#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "tracked_class.hpp"

namespace
{
void check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
}  // namespace

int main()
{
  using kart_opponent_tracker::TrackedClass;
  const TrackedClass opponent;
  const TrackedClass car("2");
  const TrackedClass detection_only("2", false);
  check(opponent.enabled() && car.enabled(), "tracking remains enabled by default");
  check(!detection_only.enabled(), "detection-only mode must disable ByteTrack updates");
  check(detection_only.id() == "2", "disabling tracking must retain the selected class contract");
  check(opponent.id() == "0", "custom opponent class ID must be preserved");
  check(car.id() == "2", "COCO car class ID must be preserved");

  // A scene contains high-confidence people and cars; the selected car policy
  // must feed only cars into its single ByteTrack instance.
  const std::vector<std::pair<std::string, double>> scene = {
    {"0", .99}, {"2", .9}, {"7", .98}, {"2", .1}, {"2", .09}, {"02", .9}};
  std::vector<std::string> selected;
  for (const auto & detection : scene) {
    check(!detection_only.accepts(detection.first, detection.second, .1),
      "detection-only mode must not send any class into ByteTrack");
    if (car.accepts(detection.first, detection.second, .1)) {
      selected.push_back(car.id());
    }
  }
  check(selected == std::vector<std::string>({"2", "2"}), "only exact car IDs above the score floor are accepted");
  check(opponent.accepts("0", .6, .1), "custom opponent class remains supported");
  check(!opponent.accepts("2", .9, .1), "custom opponent policy must reject other classes");
  check(car.accepts("2", 1., .1), "score upper boundary is inclusive");
  for (double score : {-1., 1.1, std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    check(!car.accepts("2", score, .1), "invalid confidence must not enter ByteTrack");
  }
  const TrackedClass named("opponent");
  check(named.accepts("opponent", .8, .1), "string class IDs are preserved without numeric remapping");
  try {
    const TrackedClass invalid("");
    (void)invalid;
  } catch (const std::invalid_argument &) {
    return 0;
  }
  throw std::runtime_error("empty tracked_class_id must be rejected");
}
