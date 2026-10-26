#include "rssi_tracker.h"

namespace esphome {
namespace oregon_v21_receiver {

float RssiTracker::raw_to_dbm(uint8_t raw) { return static_cast<int8_t>(raw) * 0.5f - 74.0f; }

void RssiTracker::add(uint32_t time_ms, uint8_t raw) {
  this->times_[this->next_] = time_ms;
  this->raws_[this->next_] = raw;
  this->next_ = (this->next_ + 1) % SAMPLES;
  if (this->count_ < SAMPLES)
    this->count_++;
}

bool RssiTracker::peak(uint32_t from_ms, uint32_t to_ms, int16_t &dbm) const {
  // Offsets from `from_ms` in 32 bits keep the comparison valid across a millis() wrap.
  const uint32_t width = to_ms - from_ms;
  bool found = false;
  int best = 0;
  for (size_t i = 0; i < this->count_; i++) {
    if (static_cast<uint32_t>(this->times_[i] - from_ms) > width)
      continue;
    const int v = static_cast<int8_t>(this->raws_[i]);
    if (!found || v > best) {
      best = v;
      found = true;
    }
  }
  if (found) {
    const float f = raw_to_dbm(static_cast<uint8_t>(static_cast<int8_t>(best)));
    dbm = static_cast<int16_t>(f >= 0 ? f + 0.5f : f - 0.5f);
  }
  return found;
}

void RssiTracker::clear() {
  this->next_ = 0;
  this->count_ = 0;
}

}  // namespace oregon_v21_receiver
}  // namespace esphome
