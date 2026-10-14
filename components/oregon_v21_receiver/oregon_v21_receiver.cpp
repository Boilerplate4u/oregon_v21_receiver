#include "oregon_v21_receiver.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace oregon_v21_receiver {

static const char *const TAG = "oregon_v21";

// Messages identical to the previous one within this window are treated as the repeat.
static const uint32_t REPEAT_WINDOW_MS = 1500;

bool OregonDevice::matches(const OregonReading &reading) const {
  if (reading.channel != this->channel_)
    return false;
  if (this->match_rolling_code_ && reading.rolling_code != this->rolling_code_)
    return false;
  return true;
}

void OregonDevice::publish(const OregonReading &reading) {
  if (this->temperature_sensor_ != nullptr && reading.has_temperature)
    this->temperature_sensor_->publish_state(reading.temperature);
  if (this->humidity_sensor_ != nullptr && reading.has_humidity)
    this->humidity_sensor_->publish_state(reading.humidity);
  if (this->battery_low_binary_sensor_ != nullptr)
    this->battery_low_binary_sensor_->publish_state(reading.battery_low);
}

void OregonDevice::dump_config(const char *tag) const {
  if (this->match_rolling_code_) {
    ESP_LOGCONFIG(tag, "  Device: channel %u, rolling code %u", this->channel_, this->rolling_code_);
  } else {
    ESP_LOGCONFIG(tag, "  Device: channel %u, any rolling code", this->channel_);
  }
  LOG_SENSOR("    ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("    ", "Humidity", this->humidity_sensor_);
  LOG_BINARY_SENSOR("    ", "Battery low", this->battery_low_binary_sensor_);
}

bool OregonV21Receiver::on_receive(remote_base::RemoteReceiveData data) {
  const auto &raw = data.get_raw_data();
  bool decoded = false;

  this->decoder_.reset();
  for (int32_t value : raw) {
    const uint32_t width = static_cast<uint32_t>(value < 0 ? -value : value);
    if (this->decoder_.feed(width)) {
      decoded |= this->handle_message_();
      this->decoder_.reset();
    }
  }
  if (this->decoder_.finish())
    decoded |= this->handle_message_();
  this->decoder_.reset();

  // Returning false lets ESPHome treat the frame as not handled by this listener.
  return decoded;
}

bool OregonV21Receiver::is_repeat_(const uint8_t *data, size_t len) {
  const uint32_t now = millis();
  const bool repeat = len == this->last_message_len_ && std::memcmp(data, this->last_message_, len) == 0 &&
                      (now - this->last_message_time_) < REPEAT_WINDOW_MS;
  std::memcpy(this->last_message_, data, len);
  this->last_message_len_ = len;
  this->last_message_time_ = now;
  return repeat;
}

bool OregonV21Receiver::handle_message_() {
  const uint8_t *data = this->decoder_.data();
  const size_t len = this->decoder_.size();

  char hex[ManchesterDecoder::MAX_BYTES * 3 + 1];
  size_t pos = 0;
  for (size_t i = 0; i < len && pos + 4 <= sizeof(hex); i++)
    pos += snprintf(hex + pos, sizeof(hex) - pos, "%s%02X", i == 0 ? "" : " ", data[i]);
  hex[pos] = '\0';

  OregonReading reading;
  const ParseResult result = parse_oregon_v21(data, len, reading);
  if (result != ParseResult::OK) {
    ESP_LOGD(TAG, "Rejected message (%s, sensor id 0x%04X): %s", parse_result_to_string(result), reading.sensor_id,
             hex);
    return false;
  }

  if (this->is_repeat_(data, len)) {
    ESP_LOGV(TAG, "Repeat of previous message ignored: %s", hex);
    return true;
  }

  ESP_LOGD(TAG, "Message: %s", hex);
  if (reading.has_humidity) {
    ESP_LOGI(TAG, "%s channel %u (raw %u) rolling code %u: %.1f °C, %u %%, battery %s", reading.model,
             reading.channel, reading.channel_raw, reading.rolling_code, reading.temperature, reading.humidity,
             reading.battery_low ? "low" : "ok");
  } else {
    ESP_LOGI(TAG, "%s channel %u (raw %u) rolling code %u: %.1f °C, battery %s", reading.model, reading.channel,
             reading.channel_raw, reading.rolling_code, reading.temperature, reading.battery_low ? "low" : "ok");
  }

  bool matched = false;
  for (auto *device : this->devices_) {
    if (device->matches(reading)) {
      device->publish(reading);
      matched = true;
    }
  }
  if (!matched) {
    ESP_LOGI(TAG, "No device configured for channel %u rolling code %u (add it under 'devices:' to use it)",
             reading.channel, reading.rolling_code);
  }
  return true;
}

void OregonV21Receiver::dump_config() {
  const auto &t = this->decoder_.get_timing();
  ESP_LOGCONFIG(TAG, "Oregon Scientific v2.1 receiver:");
  ESP_LOGCONFIG(TAG, "  Pulse window: %u-%u us, long pulse from %u us, end gap %u us",
                static_cast<unsigned>(t.min_pulse_us), static_cast<unsigned>(t.max_pulse_us),
                static_cast<unsigned>(t.long_pulse_us), static_cast<unsigned>(t.end_gap_us));
  for (auto *device : this->devices_)
    device->dump_config(TAG);
}

}  // namespace oregon_v21_receiver
}  // namespace esphome
