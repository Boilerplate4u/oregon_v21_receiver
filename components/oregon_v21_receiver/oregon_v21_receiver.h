#pragma once

#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "manchester_decoder.h"
#include "oregon_protocol.h"

namespace esphome {
namespace oregon_v21_receiver {

/// One configured Oregon sensor, matched on channel and optionally rolling code.
class OregonDevice {
 public:
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_rolling_code(uint8_t rolling_code) {
    this->rolling_code_ = rolling_code;
    this->match_rolling_code_ = true;
  }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { this->humidity_sensor_ = s; }
  void set_battery_low_binary_sensor(binary_sensor::BinarySensor *s) { this->battery_low_binary_sensor_ = s; }

  bool matches(const OregonReading &reading) const;
  void publish(const OregonReading &reading);
  void dump_config(const char *tag) const;

 protected:
  uint8_t channel_{0};
  uint8_t rolling_code_{0};
  bool match_rolling_code_{false};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  binary_sensor::BinarySensor *battery_low_binary_sensor_{nullptr};
};

/// Listens to a remote_receiver and decodes Oregon Scientific v2.1 messages.
class OregonV21Receiver : public Component, public remote_base::RemoteReceiverListener {
 public:
  void add_device(OregonDevice *device) { this->devices_.push_back(device); }
  void set_timing(const ManchesterDecoder::Timing &timing) { this->decoder_.set_timing(timing); }

  bool on_receive(remote_base::RemoteReceiveData data) override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  bool handle_message_();
  bool is_repeat_(const uint8_t *data, size_t len);

  ManchesterDecoder decoder_;
  std::vector<OregonDevice *> devices_;

  // Oregon sensors send every message twice; skip the identical copy.
  uint8_t last_message_[ManchesterDecoder::MAX_BYTES]{};
  size_t last_message_len_{0};
  uint32_t last_message_time_{0};
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
