#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace kart_e2e {
struct LiveLatencySample {
  int64_t source_ns{}, output_ros_ns{}, received_ns{}, output_ns{};
  bool active{}, first{};
  double steering{}, throttle{};
};
// Bounded metadata queue. Disk I/O never executes in the control callback.
class LiveLatencyCsv {
 public:
  LiveLatencyCsv(const std::string &path, size_t capacity) : capacity_(capacity) {
    if (!capacity) throw std::invalid_argument("latency_queue_capacity must be positive");
    file_ = std::fopen(path.c_str(), "wx");
    if (!file_) throw std::runtime_error("Cannot create new latency CSV: " + path);
    std::fprintf(file_, "source_stamp_ns,output_ros_ns,prediction_received_steady_ns,output_steady_ns,active,first_output,source_age_ms,prediction_to_output_ms,steering,throttle,log_dropped_total\n");
    try { worker_ = std::thread([this] { run(); }); }
    catch (...) { std::fclose(file_); throw; }
  }
  ~LiveLatencyCsv() {
    {std::lock_guard<std::mutex> lock(mutex_); stop_ = true;}
    cv_.notify_one(); worker_.join();
    if (dropped_) std::fprintf(stderr, "E2E latency CSV dropped %llu records\n", (unsigned long long)dropped_);
    std::fclose(file_);
  }
  void push(const LiveLatencySample &sample) {
    {std::lock_guard<std::mutex> lock(mutex_);
      if (stop_ || queue_.size() >= capacity_) {++dropped_; return;}
      queue_.push_back(sample);
    }
    cv_.notify_one();
  }
 private:
  void run() {
    for (;;) {
      std::deque<LiveLatencySample> batch; uint64_t dropped;
      {std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(250), [this] {return stop_ || !queue_.empty();});
        batch.swap(queue_); dropped = dropped_;
        if (stop_ && batch.empty()) break;
      }
      for (const auto &s : batch) {
        std::fprintf(file_, "%lld,%lld,%lld,%lld,%d,%d,", (long long)s.source_ns,
          (long long)s.output_ros_ns, (long long)s.received_ns, (long long)s.output_ns, s.active, s.first);
        // ROS timestamp age is separate from a duration measured with steady_clock.
        if (s.active && s.source_ns > 0 && s.output_ros_ns >= s.source_ns)
          std::fprintf(file_, "%.9f", double(s.output_ros_ns-s.source_ns)/1e6);
        std::fputc(',', file_);
        if (s.active && s.received_ns > 0 && s.output_ns >= s.received_ns)
          std::fprintf(file_, "%.9f", double(s.output_ns-s.received_ns)/1e6);
        std::fprintf(file_, ",%.9g,%.9g,%llu\n", s.steering, s.throttle, (unsigned long long)dropped);
      }
      if (std::fflush(file_) != 0 || std::ferror(file_)) {
        // A failed disk must not block control or produce a false complete log.
        std::fprintf(stderr, "E2E latency CSV write failed; logging stopped\n");
        std::lock_guard<std::mutex> lock(mutex_); queue_.clear(); stop_ = true; return;
      }
    }
  }
  size_t capacity_; FILE *file_{}; std::thread worker_;
  std::mutex mutex_; std::condition_variable cv_; std::deque<LiveLatencySample> queue_;
  bool stop_{}; uint64_t dropped_{};
};
}  // namespace kart_e2e
