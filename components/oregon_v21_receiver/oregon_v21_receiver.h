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

  /// Channel matches and, if configured, the rolling code too.
  bool matches(const OregonReading &reading) const;
  bool has_rolling_code() const { return this->match_rolling_code_; }
  uint8_t get_channel() const { return this->channel_; }
  void publish(const OregonReading &reading);
  void dump_config() const;

  /// For devices without a rolling code: returns the previous rolling code when a
  /// different one is heard within `window_ms`, once per device. -1 otherwise.
  int16_t check_rolling_code_change(uint8_t rolling_code, uint32_t now, uint32_t window_ms);

 protected:
  uint8_t channel_{0};
  uint8_t rolling_code_{0};
  bool match_rolling_code_{false};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  binary_sensor::BinarySensor *battery_low_binary_sensor_{nullptr};

  int16_t last_rolling_code_{-1};
  uint32_t last_rolling_code_time_{0};
  bool rolling_code_warned_{false};
};

/// Listens to a remote_receiver and decodes Oregon Scientific v2.1 messages.
class OregonV21Receiver : public PollingComponent, public remote_base::RemoteReceiverListener {
 public:
  void add_device(OregonDevice *device) { this->devices_.push_back(device); }
  void set_timing(const ManchesterDecoder::Timing &timing) { this->decoder_.set_timing(timing); }
  void set_verify_bit_pairs(bool verify) { this->verify_bit_pairs_ = verify; }

  void set_messages_sensor(sensor::Sensor *s) { this->messages_sensor_ = s; }
  void set_checksum_errors_sensor(sensor::Sensor *s) { this->checksum_errors_sensor_ = s; }
  void set_bit_pair_errors_sensor(sensor::Sensor *s) { this->bit_pair_errors_sensor_ = s; }
  void set_rejected_sensor(sensor::Sensor *s) { this->rejected_sensor_ = s; }
  void set_aborted_sensor(sensor::Sensor *s) { this->aborted_sensor_ = s; }

  void setup() override;
  void update() override;
  bool on_receive(remote_base::RemoteReceiveData data) override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  bool handle_message_();
  void handle_abort_();
  bool is_repeat_(const uint8_t *data, size_t len);
  bool first_unmatched_(uint8_t channel, uint8_t rolling_code);
  void dump_message_verbose_(const char *hex);

  ManchesterDecoder decoder_;
  std::vector<OregonDevice *> devices_;
  bool verify_bit_pairs_{true};

  // Oregon sensors send every message twice; skip the identical copy.
  uint8_t last_message_[ManchesterDecoder::MAX_BYTES]{};
  size_t last_message_len_{0};
  uint32_t last_message_time_{0};

  // Sensors heard without a configured device, logged at INFO only the first time.
  static constexpr size_t MAX_UNMATCHED = 8;
  uint16_t unmatched_seen_[MAX_UNMATCHED]{};
  uint8_t unmatched_count_{0};

  // Diagnostics, counted since boot.
  uint32_t messages_{0};
  uint32_t checksum_errors_{0};
  uint32_t bit_pair_errors_{0};
  uint32_t rejected_{0};
  uint32_t aborted_{0};
  sensor::Sensor *messages_sensor_{nullptr};
  sensor::Sensor *checksum_errors_sensor_{nullptr};
  sensor::Sensor *bit_pair_errors_sensor_{nullptr};
  sensor::Sensor *rejected_sensor_{nullptr};
  sensor::Sensor *aborted_sensor_{nullptr};
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
