#include "oregon_v21_receiver.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace oregon_v21_receiver {

static const char *const TAG = "oregon_v21";

// Messages identical to the previous one within this window are treated as the repeat.
static const uint32_t REPEAT_WINDOW_MS = 1500;
// Two rolling codes on one channel within this window means two sensors share the channel.
static const uint32_t ROLLING_CODE_CONFLICT_WINDOW_MS = 10 * 60 * 1000;

// Room for MAX_BYTES as "XX " plus the terminator.
static const size_t HEX_BUFFER_SIZE = ManchesterDecoder::MAX_BYTES * 3 + 1;

static void format_hex(const uint8_t *data, size_t len, char *out, size_t out_size) {
  size_t pos = 0;
  out[0] = '\0';
  for (size_t i = 0; i < len && pos + 4 <= out_size; i++) {
    int written = snprintf(out + pos, out_size - pos, "%s%02X", i == 0 ? "" : " ", data[i]);
    if (written <= 0)
      break;
    pos += static_cast<size_t>(written);
  }
}

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

int16_t OregonDevice::check_rolling_code_change(uint8_t rolling_code, uint32_t now, uint32_t window_ms) {
  int16_t previous = -1;
  if (this->last_rolling_code_ >= 0 && this->last_rolling_code_ != rolling_code &&
      (now - this->last_rolling_code_time_) < window_ms && !this->rolling_code_warned_) {
    previous = this->last_rolling_code_;
    this->rolling_code_warned_ = true;
  }
  this->last_rolling_code_ = rolling_code;
  this->last_rolling_code_time_ = now;
  return previous;
}

void OregonDevice::dump_config() const {
  if (this->match_rolling_code_) {
    ESP_LOGCONFIG(TAG, "  Device: channel %u, rolling code %u", this->channel_, this->rolling_code_);
  } else {
    ESP_LOGCONFIG(TAG, "  Device: channel %u, any rolling code", this->channel_);
  }
  LOG_SENSOR("    ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("    ", "Humidity", this->humidity_sensor_);
  LOG_BINARY_SENSOR("    ", "Battery low", this->battery_low_binary_sensor_);
}

void OregonV21Receiver::setup() { this->decoder_.set_record_aborts(true); }

void OregonV21Receiver::update() {
  if (this->messages_sensor_ != nullptr)
    this->messages_sensor_->publish_state(this->messages_);
  if (this->checksum_errors_sensor_ != nullptr)
    this->checksum_errors_sensor_->publish_state(this->checksum_errors_);
  if (this->bit_pair_errors_sensor_ != nullptr)
    this->bit_pair_errors_sensor_->publish_state(this->bit_pair_errors_);
  if (this->rejected_sensor_ != nullptr)
    this->rejected_sensor_->publish_state(this->rejected_);
  if (this->aborted_sensor_ != nullptr)
    this->aborted_sensor_->publish_state(this->aborted_);
}

bool OregonV21Receiver::on_receive(remote_base::RemoteReceiveData data) {
  const auto &raw = data.get_raw_data();
  bool decoded = false;

  this->decoder_.reset();
  for (int32_t value : raw) {
    // Widths are bounded by the receiver; clamp before negating to avoid INT32_MIN overflow.
    const uint32_t width = value < 0 ? uint32_t(-(int64_t) value) : uint32_t(value);
    const bool done = this->decoder_.feed(width);
    this->handle_abort_();
    if (done) {
      decoded |= this->handle_message_();
      this->decoder_.reset();
    }
  }
  if (this->decoder_.finish())
    decoded |= this->handle_message_();
  this->handle_abort_();
  this->decoder_.reset();

  // Returning false lets ESPHome treat the frame as not handled by this listener.
  return decoded;
}

void OregonV21Receiver::handle_abort_() {
  ManchesterDecoder::AbortInfo info;
  if (!this->decoder_.take_abort(info))
    return;
  this->aborted_++;
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  char hex[HEX_BUFFER_SIZE];
  format_hex(info.data, info.size, hex, sizeof(hex));
  if (info.pulse_width_us == 0) {
    ESP_LOGV(TAG, "Candidate abandoned at end of capture after %u raw bits (%u bytes): %s", info.raw_bits, info.size,
             hex);
  } else {
    ESP_LOGV(TAG, "Candidate abandoned after %u raw bits (%u bytes) at a %" PRIu32 " us pulse: %s", info.raw_bits,
             info.size, info.pulse_width_us, hex);
  }
#endif
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

bool OregonV21Receiver::first_unmatched_(uint8_t channel, uint8_t rolling_code) {
  const uint16_t key = uint16_t(channel) << 8 | rolling_code;
  for (uint8_t i = 0; i < this->unmatched_count_; i++) {
    if (this->unmatched_seen_[i] == key)
      return false;
  }
  if (this->unmatched_count_ < MAX_UNMATCHED) {
    this->unmatched_seen_[this->unmatched_count_++] = key;
    return true;
  }
  return false;  // table full: stay quiet rather than flood the log
}

void OregonV21Receiver::dump_message_verbose_(const char *hex) {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  const auto &stats = this->decoder_.pulse_stats();
  ESP_LOGV(TAG,
           "Manchester message: %u bytes, %u bit pair errors, short %" PRIu32 " us (n=%u), long %" PRIu32 " us (n=%u)",
           static_cast<unsigned>(this->decoder_.size()), this->decoder_.pair_errors(), stats.short_avg_us(),
           stats.short_count, stats.long_avg_us(), stats.long_count);
  ESP_LOGV(TAG, "  bytes: %s", hex);
  // Raw bits in pairs (data bit, complement), one group per nibble, 8 nibbles per line.
  const size_t raw_bits = this->decoder_.raw_bit_count();
  char line[8 * 9 + 1];
  for (size_t nibble_start = 0; nibble_start < raw_bits; nibble_start += 8 * 8) {
    size_t pos = 0;
    for (size_t i = nibble_start; i < raw_bits && i < nibble_start + 8 * 8; i++) {
      if (i != nibble_start && (i % 8) == 0)
        line[pos++] = ' ';
      line[pos++] = this->decoder_.raw_bit(i) ? '1' : '0';
    }
    line[pos] = '\0';
    ESP_LOGV(TAG, "  raw nibbles %2u-%2u: %s", static_cast<unsigned>(nibble_start / 8),
             static_cast<unsigned>(std::min(raw_bits, nibble_start + 64) / 8 - 1), line);
  }
#else
  (void) hex;
#endif
}

bool OregonV21Receiver::handle_message_() {
  const uint8_t *data = this->decoder_.data();
  const size_t len = this->decoder_.size();

  char hex[HEX_BUFFER_SIZE];
  format_hex(data, len, hex, sizeof(hex));
  this->dump_message_verbose_(hex);

  OregonReading reading;
  const ParseResult result = parse_oregon_v21(data, len, reading);

  // Only the bytes covered by the checksum matter; some sensors send an unused tail.
  const size_t checked_bytes = reading.used_bytes != 0 ? reading.used_bytes : len;
  const uint16_t pair_errors = this->decoder_.pair_errors(checked_bytes);
  if (pair_errors > 0) {
    if (this->verify_bit_pairs_) {
      this->bit_pair_errors_++;
      ESP_LOGD(TAG, "Rejected message (%u bit pair errors): %s", pair_errors, hex);
      return false;
    }
    ESP_LOGD(TAG, "Message has %u bit pair errors (verification disabled): %s", pair_errors, hex);
  }

  if (result != ParseResult::OK) {
    if (result == ParseResult::BAD_CHECKSUM) {
      this->checksum_errors_++;
    } else {
      this->rejected_++;
    }
    ESP_LOGD(TAG, "Rejected message (%s, sensor id 0x%04X): %s", parse_result_to_string(result), reading.sensor_id,
             hex);
    return false;
  }

  if (this->is_repeat_(data, reading.used_bytes)) {
    ESP_LOGV(TAG, "Repeat of previous message ignored: %s", hex);
    return true;
  }
  this->messages_++;

  ESP_LOGD(TAG, "Message: %s", hex);
  if (reading.has_humidity) {
    ESP_LOGD(TAG, "%s channel %u (raw %u) rolling code %u: %.1f °C, %u %%, battery %s", reading.model, reading.channel,
             reading.channel_raw, reading.rolling_code, reading.temperature, reading.humidity,
             reading.battery_low ? "low" : "ok");
  } else {
    ESP_LOGD(TAG, "%s channel %u (raw %u) rolling code %u: %.1f °C, battery %s", reading.model, reading.channel,
             reading.channel_raw, reading.rolling_code, reading.temperature, reading.battery_low ? "low" : "ok");
  }

  // Devices with a matching rolling code take precedence over devices for the whole channel.
  bool matched = false;
  for (auto *device : this->devices_) {
    if (device->has_rolling_code() && device->matches(reading)) {
      device->publish(reading);
      matched = true;
    }
  }
  if (!matched) {
    const uint32_t now = millis();
    for (auto *device : this->devices_) {
      if (!device->has_rolling_code() && device->matches(reading)) {
        const int16_t previous =
            device->check_rolling_code_change(reading.rolling_code, now, ROLLING_CODE_CONFLICT_WINDOW_MS);
        if (previous >= 0) {
          ESP_LOGW(TAG,
                   "Channel %u received from rolling codes %d and %u within 10 minutes. If a neighbour's sensor "
                   "uses the same channel, set 'rolling_code: %u' on your device. Ignore this after a battery change.",
                   reading.channel, previous, reading.rolling_code, reading.rolling_code);
        }
        device->publish(reading);
        matched = true;
      }
    }
  }
  if (!matched) {
    if (this->first_unmatched_(reading.channel, reading.rolling_code)) {
      ESP_LOGI(TAG,
               "%s on channel %u with rolling code %u is not configured; add it under 'devices:' to use it "
               "(further messages from it are logged at DEBUG)",
               reading.model, reading.channel, reading.rolling_code);
    } else {
      ESP_LOGD(TAG, "No device configured for channel %u rolling code %u", reading.channel, reading.rolling_code);
    }
  }
  return true;
}

void OregonV21Receiver::dump_config() {
  const auto &t = this->decoder_.get_timing();
  ESP_LOGCONFIG(TAG, "Oregon Scientific v2.1 receiver:");
  ESP_LOGCONFIG(TAG,
                "  Pulse window: %" PRIu32 "-%" PRIu32 " us, long pulse from %" PRIu32 " us, end gap %" PRIu32
                " us, preamble %u",
                t.min_pulse_us, t.max_pulse_us, t.long_pulse_us, t.end_gap_us, t.min_preamble);
  ESP_LOGCONFIG(TAG, "  Verify bit pairs: %s", YESNO(this->verify_bit_pairs_));
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Messages", this->messages_sensor_);
  LOG_SENSOR("  ", "Checksum errors", this->checksum_errors_sensor_);
  LOG_SENSOR("  ", "Bit pair errors", this->bit_pair_errors_sensor_);
  LOG_SENSOR("  ", "Rejected", this->rejected_sensor_);
  LOG_SENSOR("  ", "Aborted", this->aborted_sensor_);
  for (auto *device : this->devices_)
    device->dump_config();
}

}  // namespace oregon_v21_receiver
}  // namespace esphome
