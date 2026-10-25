#include "kart_e2e/live_latency.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>
int main() {
  const auto path = (std::filesystem::temp_directory_path() / ("kart-live-latency-"+std::to_string(getpid())+".csv")).string();
  std::filesystem::remove(path);
  { kart_e2e::LiveLatencyCsv log(path, 8192);
    log.push({1000000, 4000000, 10000000, 12000000, true, true, .1, .2});
    log.push({1000000, 5000000, 10000000, 13000000, true, false, .1, .2});
    log.push({1000000, 5000000, 10000000, 13000000, false, false, 0, 0});
    log.push({9000000, 5000000, 10000000, 13000000, true, true, 0, 0});
  }
  std::ifstream file(path); std::stringstream stream;stream<<file.rdbuf();const auto text=stream.str();
  assert(text.find(",1,1,3.000000000,2.000000000,") != std::string::npos);
  assert(text.find(",1,0,4.000000000,3.000000000,") != std::string::npos);
  assert(text.find(",0,0,,,") != std::string::npos);
  assert(text.find(",1,1,,3.000000000,") != std::string::npos);
  bool refused=false;try {kart_e2e::LiveLatencyCsv log(path, 8);}catch(const std::runtime_error &){refused=true;}
  assert(refused);std::filesystem::remove(path);
}
