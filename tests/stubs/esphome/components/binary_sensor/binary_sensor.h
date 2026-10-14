#pragma once
// Minimal host stub of a binary sensor: records published states.
#include <vector>
namespace esphome {
namespace binary_sensor {
class BinarySensor {
 public:
  void publish_state(bool state) { this->states.push_back(state); }
  std::vector<bool> states;
};
}  // namespace binary_sensor
}  // namespace esphome
