// esphome/components/oregon_v21_receiver/oregon_v21_receiver.h

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <cmath>

namespace esphome {
namespace oregon_v21_receiver {

using remote_receiver::RemoteReceiveData;

class OregonV21Receiver : public Component, public remote_receiver::RemoteReceiverListener {
 public:
  // Sensor pointers - will be created and registered via Python code
  sensor::Sensor *temperature_sensor{nullptr};
  sensor::Sensor *humidity_sensor{nullptr};
  sensor::Sensor *battery_sensor{nullptr};

  // Setters for remote receiver
  void set_remote_receiver(remote_receiver::RemoteReceiverBase *rx) {
    this->remote_rx_ = rx;
    if (rx != nullptr) {
      rx->add_listener(this);
    }
  }

  void setup() override { ESP_LOGI("oregon", "Oregon v2.1 Receiver initialized"); }
  void loop() override {}

  // Called when remote_receiver has data
  void on_receive(const RemoteReceiveData &data) override;

 protected:
  remote_receiver::RemoteReceiverBase *remote_rx_{nullptr};

  // Decoding state
  std::vector<int32_t> raw_data_;
  std::vector<uint8_t> bits_;

  // Decoded values
  float decoded_temp_{0.0f};
  float decoded_hum_{0.0f};
  float decoded_batt_{0.0f};

  // Thresholds
  static constexpr int MIN_PULSES = 100;
  static constexpr int MAX_PULSES = 400;
  static constexpr float BASE_US = 1000.0f;
  static constexpr float MANCHESTER_MIN = 0.75f;
  static constexpr float MANCHESTER_MAX = 1.25f;
  static constexpr int MANCHESTER_QUALITY_THRESHOLD = 50;

  // Core algorithm
  bool decode_frame_();
  bool extract_bits_manchester_();
  bool find_sync_and_extract_payload_(std::vector<uint8_t> &payload);
  bool decode_payload_(const std::vector<uint8_t> &payload);

  // Helper
  static float calculate_manchester_quality_(const std::vector<int32_t> &raw);
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
