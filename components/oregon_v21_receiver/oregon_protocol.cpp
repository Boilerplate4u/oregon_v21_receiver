#include "oregon_protocol.h"

namespace esphome {
namespace oregon_v21_receiver {

namespace {

struct SensorType {
  uint16_t id;
  const char *model;
  bool has_humidity;
  uint8_t last_checksummed_nibble;  // checksum covers nibbles 1..N
  bool bit_coded_channel;
};

// Types and checksum ranges from rtl_433 src/devices/oregon_scientific.c.
// The checksum index there excludes the sync nibble, hence N = rtl_433 index.
const SensorType SENSOR_TYPES[] = {
    {0x1D20, "THGR122N/THGR228N", true, 15, true},
    {0x1D30, "THGR968", true, 15, false},
    {0xEC40, "THN132N/THR228N", false, 12, true},
};

const SensorType *find_sensor_type(uint16_t id) {
  for (const auto &type : SENSOR_TYPES) {
    if (type.id == id)
      return &type;
  }
  return nullptr;
}

bool is_bcd(uint8_t nibble) { return nibble <= 9; }

}  // namespace

uint16_t oregon_type(const uint8_t *data, size_t len) {
  if (len < 3)
    return 0;
  return (uint16_t(oregon_nibble(data, 1)) << 12) | (uint16_t(oregon_nibble(data, 2)) << 8) |
         (uint16_t(oregon_nibble(data, 3)) << 4) | uint16_t(oregon_nibble(data, 4));
}

ParseResult parse_oregon_v21(const uint8_t *data, size_t len, OregonReading &out) {
  if (len < 3)
    return ParseResult::TOO_SHORT;
  if (oregon_nibble(data, 0) != 0xA)
    return ParseResult::BAD_SYNC;

  const uint16_t id = oregon_type(data, len);
  out.type = id;
  const SensorType *type = find_sensor_type(id);
  if (type == nullptr)
    return ParseResult::UNKNOWN_TYPE;

  // Checksum occupies the two nibbles after the last checksummed one.
  const size_t checksum_nibble = type->last_checksummed_nibble + 1;
  const size_t used_bytes = (checksum_nibble + 1) / 2 + 1;
  out.used_bytes = static_cast<uint8_t>(used_bytes);
  if (used_bytes > len)
    return ParseResult::TOO_SHORT;

  uint8_t sum = 0;
  for (size_t i = 1; i <= type->last_checksummed_nibble; i++)
    sum += oregon_nibble(data, i);
  const uint8_t expected = oregon_nibble(data, checksum_nibble) | (oregon_nibble(data, checksum_nibble + 1) << 4);
  if (sum != expected)
    return ParseResult::BAD_CHECKSUM;

  const uint8_t t_tenths = oregon_nibble(data, 9);
  const uint8_t t_units = oregon_nibble(data, 10);
  const uint8_t t_tens = oregon_nibble(data, 11);
  if (!is_bcd(t_tenths) || !is_bcd(t_units) || !is_bcd(t_tens))
    return ParseResult::BAD_VALUE;
  const uint8_t sign = oregon_nibble(data, 12);
  if (sign & 0x07)
    return ParseResult::BAD_VALUE;  // rtl_433 reads these bits as hundreds (AWR129 only)
  float temperature = t_tens * 10.0f + t_units + t_tenths / 10.0f;
  if (sign & 0x08)
    temperature = -temperature;
  if (temperature < -50.0f || temperature > 70.0f)
    return ParseResult::BAD_VALUE;

  uint8_t humidity = 0;
  if (type->has_humidity) {
    const uint8_t h_units = oregon_nibble(data, 13);
    const uint8_t h_tens = oregon_nibble(data, 14);
    if (!is_bcd(h_units) || !is_bcd(h_tens))
      return ParseResult::BAD_VALUE;
    humidity = h_tens * 10 + h_units;
    if (humidity > 100)
      return ParseResult::BAD_VALUE;
  }

  const uint8_t channel_raw = oregon_nibble(data, 5);
  uint8_t channel = channel_raw;
  if (type->bit_coded_channel) {
    switch (channel_raw) {
      case 1:
        channel = 1;
        break;
      case 2:
        channel = 2;
        break;
      case 4:
        channel = 3;
        break;
      default:
        return ParseResult::BAD_VALUE;
    }
  }

  out.model = type->model;
  out.channel_raw = channel_raw;
  out.channel = channel;
  out.unit_id = oregon_nibble(data, 6) | (oregon_nibble(data, 7) << 4);
  out.battery_low = (oregon_nibble(data, 8) & 0x04) != 0;
  out.has_temperature = true;
  out.temperature = temperature;
  out.has_humidity = type->has_humidity;
  out.humidity = humidity;
  return ParseResult::OK;
}

bool parse_unknown_oregon_v21(const uint8_t *data, size_t len, UnknownOregonMessage &out) {
  out = UnknownOregonMessage{};
  if (len < 3 || oregon_nibble(data, 0) != 0xA)
    return false;
  const size_t nibbles = len * 2;

  // Try every checksum position; keep the largest match, it covers the most data.
  uint8_t sum = 0;
  for (size_t n = 1; n <= UNKNOWN_MAX_CHECKSUMMED_NIBBLE && n + 2 < nibbles; n++) {
    sum += oregon_nibble(data, n);
    if (n < UNKNOWN_MIN_CHECKSUMMED_NIBBLE)
      continue;
    const uint8_t expected = oregon_nibble(data, n + 1) | (oregon_nibble(data, n + 2) << 4);
    if (sum == expected) {
      out.last_checksummed_nibble = static_cast<uint8_t>(n);
      out.checksum_matches++;
    }
  }
  if (out.checksum_matches == 0)
    return false;

  const size_t n = out.last_checksummed_nibble;
  out.used_bytes = static_cast<uint8_t>((n + 2) / 2 + 1);
  out.type = oregon_type(data, len);
  out.channel_raw = oregon_nibble(data, 5);
  out.unit_id = oregon_nibble(data, 6) | (oregon_nibble(data, 7) << 4);
  out.flags = oregon_nibble(data, 8);

  // THGR122N layout: temperature in nibbles 9..12, humidity in 13..14.
  if (n >= 12) {
    const uint8_t t_tenths = oregon_nibble(data, 9);
    const uint8_t t_units = oregon_nibble(data, 10);
    const uint8_t t_tens = oregon_nibble(data, 11);
    const uint8_t sign = oregon_nibble(data, 12);
    if (is_bcd(t_tenths) && is_bcd(t_units) && is_bcd(t_tens) && (sign & 0x07) == 0) {
      float temperature = t_tens * 10.0f + t_units + t_tenths / 10.0f;
      if (sign & 0x08)
        temperature = -temperature;
      if (temperature >= -50.0f && temperature <= 70.0f) {
        out.has_guessed_temperature = true;
        out.guessed_temperature = temperature;
      }
    }
  }
  if (n >= 14) {
    const uint8_t h_units = oregon_nibble(data, 13);
    const uint8_t h_tens = oregon_nibble(data, 14);
    if (is_bcd(h_units) && is_bcd(h_tens) && h_tens * 10 + h_units <= 100) {
      out.has_guessed_humidity = true;
      out.guessed_humidity = h_tens * 10 + h_units;
    }
  }
  return true;
}

const char *parse_result_to_string(ParseResult result) {
  switch (result) {
    case ParseResult::OK:
      return "ok";
    case ParseResult::TOO_SHORT:
      return "message too short";
    case ParseResult::BAD_SYNC:
      return "bad sync nibble";
    case ParseResult::UNKNOWN_TYPE:
      return "unknown type";
    case ParseResult::BAD_CHECKSUM:
      return "bad checksum";
    case ParseResult::BAD_VALUE:
      return "value out of range";
  }
  return "unknown";
}

}  // namespace oregon_v21_receiver
}  // namespace esphome
