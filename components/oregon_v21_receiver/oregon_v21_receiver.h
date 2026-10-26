#pragma once

#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_OREGON_V21_RSSI
#include "esphome/components/cc1101/cc1101.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#endif

#include "manchester_decoder.h"
#include "oregon_protocol.h"
#include "rssi_tracker.h"

namespace esphome {
namespace oregon_v21_receiver {

/// One configured Oregon remote sensor (the wireless thermo-hygro unit), matched on
/// channel and optionally unit_id.
class RemoteSensor {
 public:
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_unit_id(uint8_t unit_id) {
    this->unit_id_ = unit_id;
    this->match_unit_id_ = true;
  }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { this->humidity_sensor_ = s; }
  void set_battery_low_binary_sensor(binary_sensor::BinarySensor *s) { this->battery_low_binary_sensor_ = s; }
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_sensor_ = s; }

  /// Channel matches and, if configured, the unit_id too.
  bool matches(const OregonReading &reading) const;
  bool has_unit_id() const { return this->match_unit_id_; }
  uint8_t get_channel() const { return this->channel_; }
  /// Publishes the reading; `rssi_dbm` only when `has_rssi` (CC1101 signal strength).
  void publish(const OregonReading &reading, bool has_rssi, int16_t rssi_dbm);
  void dump_config() const;

  /// For remote sensors without a unit_id: returns the previous unit_id when a
  /// different one is heard within `window_ms`, once per remote sensor. -1 otherwise.
  int16_t check_unit_id_change(uint8_t unit_id, uint32_t now, uint32_t window_ms);

 protected:
  uint8_t channel_{0};
  uint8_t unit_id_{0};
  bool match_unit_id_{false};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  binary_sensor::BinarySensor *battery_low_binary_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};

  int16_t last_unit_id_{-1};
  uint32_t last_unit_id_time_{0};
  bool unit_id_warned_{false};
};

/// Listens to a remote_receiver and decodes Oregon Scientific v2.1 messages.
class OregonV21Receiver : public PollingComponent, public remote_base::RemoteReceiverListener {
 public:
  void add_remote_sensor(RemoteSensor *remote_sensor) { this->remote_sensors_.push_back(remote_sensor); }
  void set_timing(const ManchesterDecoder::Timing &timing) { this->decoder_.set_timing(timing); }
  void set_verify_bit_pairs(bool verify) { this->verify_bit_pairs_ = verify; }

  void set_messages_sensor(sensor::Sensor *s) { this->messages_sensor_ = s; }
  void set_checksum_errors_sensor(sensor::Sensor *s) { this->checksum_errors_sensor_ = s; }
  void set_bit_pair_errors_sensor(sensor::Sensor *s) { this->bit_pair_errors_sensor_ = s; }
  void set_rejected_sensor(sensor::Sensor *s) { this->rejected_sensor_ = s; }
  void set_aborted_sensor(sensor::Sensor *s) { this->aborted_sensor_ = s; }

  /// A CC1101 RSSI register sample taken at `time_ms` (uptime) while a frame was received.
  /// Filled by the sampling in loop() when a CC1101 is configured; public for the host tests.
  void add_rssi_sample(uint32_t time_ms, uint8_t raw) { this->rssi_.add(time_ms, raw); }

#ifdef USE_OREGON_V21_RSSI
  void set_radio(cc1101::CC1101Component *radio) { this->radio_ = radio; }
  void set_activity_pin(InternalGPIOPin *pin) { this->activity_pin_ = pin; }
  void loop() override;
#endif

  void setup() override;
  void update() override;
  bool on_receive(remote_base::RemoteReceiveData data) override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  bool handle_message_(uint32_t capture_start_ms, uint32_t capture_end_ms);
  bool handle_unknown_type_(const uint8_t *data, size_t len, const char *hex);
  void handle_abort_();
  void log_abandoned_(uint16_t raw_bits, const uint8_t *data, size_t size, uint32_t pulse_width_us);
  bool is_repeat_(const uint8_t *data, size_t len);
  bool first_unlisted_(uint8_t channel, uint8_t unit_id);
  void dump_message_verbose_(const char *hex);

  ManchesterDecoder decoder_;
  RssiTracker rssi_;
  std::vector<RemoteSensor *> remote_sensors_;
  bool verify_bit_pairs_{true};

  // Oregon sensors send every message twice; skip the identical copy.
  uint8_t last_message_[ManchesterDecoder::MAX_BYTES]{};
  size_t last_message_len_{0};
  uint32_t last_message_time_{0};

  // Remote sensors heard but not listed, logged at INFO only the first time.
  static constexpr size_t MAX_UNLISTED = 8;
  uint16_t unlisted_seen_[MAX_UNLISTED]{};
  uint8_t unlisted_count_{0};

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

#ifdef USE_OREGON_V21_RSSI
  static void gpio_intr_(OregonV21Receiver *arg);
  void sample_rssi_();
  cc1101::CC1101Component *radio_{nullptr};
  InternalGPIOPin *activity_pin_{nullptr};
  ISRInternalGPIOPin activity_isr_pin_;
  // Written by the interrupt on every edge of the data pin.
  volatile uint32_t last_edge_us_{0};
  volatile bool frame_started_{false};
  uint32_t last_rssi_us_{0};
  uint8_t frame_reads_{0};
  // Runs the loop continuously only while a frame is being received.
  HighFrequencyLoopRequester high_freq_;
#endif
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
