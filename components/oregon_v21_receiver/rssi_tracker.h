#pragma once

// Signal strength samples from a CC1101, kept for the last few frames so a decoded
// message can be given the strongest sample taken while it was received.
// Plain C++ without ESPHome dependencies, covered by the host tests.

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace oregon_v21_receiver {

class RssiTracker {
 public:
  static constexpr size_t SAMPLES = 32;

  /// A sample taken during a frame at `time_ms` (uptime). `raw` is the CC1101 RSSI
  /// status register value (two's complement, 0.5 dB steps).
  void add(uint32_t time_ms, uint8_t raw);
  /// Strongest sample between `from_ms` and `to_ms` (inclusive, wrap safe), in whole dBm.
  bool peak(uint32_t from_ms, uint32_t to_ms, int16_t &dbm) const;
  void clear();

  /// CC1101 datasheet conversion; the 74 dB offset matches the ESPHome cc1101 component.
  static float raw_to_dbm(uint8_t raw);

 protected:
  uint32_t times_[SAMPLES]{};
  uint8_t raws_[SAMPLES]{};
  size_t next_{0};
  size_t count_{0};
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
