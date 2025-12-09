// esphome/components/oregon_v21_receiver/oregon_v21_receiver.cpp

#include "oregon_v21_receiver.h"
#include "esphome/core/log.h"

namespace esphome {
namespace oregon_v21_receiver {

static const char *TAG = "oregon_v21";

void OregonV21Receiver::on_receive(const RemoteReceiveData &data) {
  // Copy raw timing data
  raw_data_.clear();
  raw_data_.reserve(data.raw_length);
  for (size_t i = 0; i < data.raw_length; i++) {
    raw_data_.push_back(data.raw[i]);
  }

  ESP_LOGD(TAG, "Received frame: %d pulses", (int)raw_data_.size());

  if (!decode_frame_()) {
    ESP_LOGD(TAG, "Frame decode failed");
    return;
  }

  // Publish to sensors
  if (temperature_sensor != nullptr) {
    temperature_sensor->publish_state(decoded_temp_);
    ESP_LOGI(TAG, "Temp: %.1f°C", decoded_temp_);
  }
  if (humidity_sensor != nullptr) {
    humidity_sensor->publish_state(decoded_hum_);
    ESP_LOGI(TAG, "Humidity: %.0f%%", decoded_hum_);
  }
  if (battery_sensor != nullptr) {
    battery_sensor->publish_state(decoded_batt_);
    ESP_LOGI(TAG, "Battery: %.0f%%", decoded_batt_);
  }
}

bool OregonV21Receiver::decode_frame_() {
  if (raw_data_.size() < MIN_PULSES || raw_data_.size() > MAX_PULSES) {
    ESP_LOGV(TAG, "Invalid pulse count: %d (min=%d, max=%d)", (int)raw_data_.size(),
             MIN_PULSES, MAX_PULSES);
    return false;
  }

  // Step 1: Check Manchester quality
  float quality = calculate_manchester_quality_(raw_data_);
  if (quality < MANCHESTER_QUALITY_THRESHOLD) {
    ESP_LOGD(TAG, "Manchester quality too low: %.1f%%", quality);
    return false;
  }
  ESP_LOGD(TAG, "Manchester quality: %.1f%%", quality);

  // Step 2: Extract Manchester bits
  if (!extract_bits_manchester_()) {
    ESP_LOGD(TAG, "Failed to extract bits");
    return false;
  }

  // Step 3: Find sync and extract payload
  std::vector<uint8_t> payload;
  if (!find_sync_and_extract_payload_(payload)) {
    ESP_LOGD(TAG, "Failed to find sync/payload");
    return false;
  }

  // Step 4: Decode payload
  if (!decode_payload_(payload)) {
    ESP_LOGD(TAG, "Failed to decode payload");
    return false;
  }

  return true;
}

float OregonV21Receiver::calculate_manchester_quality_(
    const std::vector<int32_t> &raw) {
  if (raw.size() < 2)
    return 0.0f;

  int manchester_count = 0;
  int total = 0;

  // Process pairs: (HIGH, GAP)
  for (size_t i = 0; i + 1 < raw.size(); i += 2) {
    float high = std::abs(static_cast<float>(raw[i]));
    float gap = std::abs(static_cast<float>(raw[i + 1]));

    if (high > 0.0f) {
      float ratio = gap / high;
      if (ratio >= MANCHESTER_MIN && ratio <= MANCHESTER_MAX) {
        manchester_count++;
      }
      total++;
    }
  }

  if (total == 0)
    return 0.0f;
  return (100.0f * manchester_count) / total;
}

bool OregonV21Receiver::extract_bits_manchester_() {
  bits_.clear();
  bits_.reserve(raw_data_.size() / 2);

  for (size_t i = 0; i + 1 < raw_data_.size(); i += 2) {
    float high = std::abs(static_cast<float>(raw_data_[i]));
    float gap = std::abs(static_cast<float>(raw_data_[i + 1]));

    if (high > 0.0f) {
      float ratio = gap / high;

      if (ratio >= MANCHESTER_MIN && ratio <= MANCHESTER_MAX) {
        // Manchester bit: ratio >= 1.0 → '1', else → '0'
        if (ratio >= 1.0f) {
          bits_.push_back(1);
        } else {
          bits_.push_back(0);
        }
      }
    }
  }

  ESP_LOGD(TAG, "Extracted %d Manchester bits", (int)bits_.size());
  if (bits_.size() < 48) {
    ESP_LOGD(TAG, "Insufficient bits: %d (min 48)", (int)bits_.size());
    return false;
  }

  return true;
}

bool OregonV21Receiver::find_sync_and_extract_payload_(
    std::vector<uint8_t> &payload) {
  // Find sync word 0xA (1010 binary)
  const uint8_t SYNC_WORD = 0xA;
  int sync_pos = -1;

  for (size_t i = 0; i + 3 < bits_.size(); i++) {
    uint8_t nibble = (bits_[i] << 3) | (bits_[i + 1] << 2) | (bits_[i + 2] << 1) |
                     bits_[i + 3];

    if (nibble == SYNC_WORD) {
      sync_pos = i;
      ESP_LOGD(TAG, "Found sync at position %d", sync_pos);
      break;
    }
  }

  if (sync_pos < 0) {
    ESP_LOGD(TAG, "Sync word not found");
    return false;
  }

  // Extract 40 bits after sync
  int payload_start = sync_pos + 4;
  if (payload_start + 40 > (int)bits_.size()) {
    ESP_LOGD(TAG, "Insufficient payload bits");
    return false;
  }

  payload.assign(bits_.begin() + payload_start,
                 bits_.begin() + payload_start + 40);

  return true;
}

bool OregonV21Receiver::decode_payload_(const std::vector<uint8_t> &payload) {
  if (payload.size() < 40) {
    return false;
  }

  // Temperature: bits 8-19 (12 bits), binary coded, /10 for 0.1°C resolution
  uint16_t temp_raw = 0;
  for (int i = 0; i < 12; i++) {
    if (payload[8 + i] == 1) {
      temp_raw += (1 << i);
    }
  }
  decoded_temp_ = temp_raw / 10.0f;

  // Humidity: bits 20-27 (8 bits), BCD encoded
  uint8_t hum_lo = 0, hum_hi = 0;
  for (int i = 0; i < 4; i++) {
    if (payload[20 + i] == 1)
      hum_lo += (1 << i);
    if (payload[24 + i] == 1)
      hum_hi += (1 << i);
  }
  decoded_hum_ = (hum_hi * 10) + hum_lo;

  // Battery: simplified - check bit 28 or flag in byte
  // (adjust based on your sensor model)
  uint8_t flags = 0;
  for (int i = 0; i < 4; i++) {
    if (payload[28 + i] == 1)
      flags += (1 << i);
  }
  decoded_batt_ = (flags & 0x04) ? 100.0f : 20.0f;

  ESP_LOGI(TAG, "Decoded: T=%.1f°C, H=%.0f%%, Batt=%.0f%%", decoded_temp_,
           decoded_hum_, decoded_batt_);

  return true;
}

}  // namespace oregon_v21_receiver
}  // namespace esphome
