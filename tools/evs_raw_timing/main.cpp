#include <metavision/hal/device/device.h>
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/hal/facilities/i_events_stream_decoder.h>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv) {
  if (argc != 2) {std::cerr << "Usage: evs_raw_timing FILE.raw\n"; return 2;}
  try {
    auto device = Metavision::DeviceDiscovery::open_raw_file(argv[1]);
    if (!device) {throw std::runtime_error("Cannot open RAW file");}
    auto *stream = device->get_facility<Metavision::I_EventsStream>();
    auto *decoder = device->get_facility<Metavision::I_EventsStreamDecoder>();
    if (!stream || !decoder || !decoder->is_time_shifting_enabled()) {
      throw std::runtime_error("RAW decoder with timestamp shifting required");
    }
    stream->start();
    Metavision::timestamp shift = 0;
    bool found = false;
    while (stream->wait_next_buffer() >= 0) {
      const auto buffer = stream->get_latest_raw_data();
      decoder->decode(buffer.begin(), buffer.end());
      if (decoder->get_timestamp_shift(shift)) {found=true; break;}
    }
    stream->stop();
    if (!found) {throw std::runtime_error("RAW timestamp shift not available (empty or unsupported stream)");}
    std::cout << "{\"schema\":\"openeb.raw_time_shift.v1\",\"raw_sensor_shift_us\":" << shift << "}\n";
    return 0;
  } catch (const std::exception &error) {std::cerr << error.what() << '\n'; return 1;}
}
