#pragma once
// Minimal host stub of a sensor: records published states.
#include <vector>
namespace esphome {
namespace sensor {
class Sensor {
 public:
  void publish_state(float state) { this->states.push_back(state); }
  std::vector<float> states;
};
}  // namespace sensor
}  // namespace esphome
