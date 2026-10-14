#pragma once
// Minimal host stub of esphome/core/hal.h: the test controls the clock.
#include <cstdint>
namespace esphome {
inline uint32_t &test_millis() {
  static uint32_t now = 0;
  return now;
}
inline uint32_t millis() { return test_millis(); }
}  // namespace esphome
