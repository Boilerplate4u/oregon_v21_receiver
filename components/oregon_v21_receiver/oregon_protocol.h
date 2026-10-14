#pragma once

// Oregon Scientific v2.1 message parser.
//
// Pure C++ without ESPHome dependencies so it can be unit tested on a host.
//
// Input is the byte buffer from ManchesterDecoder ("nibble order": nibble n is
// the low nibble of byte n/2 when n is even and the high nibble when n is odd).
//
// Field layout (nibble index) for the supported sensors, cross checked against
// rene-d/oregon (include/decode.h) and rtl_433 (src/devices/oregon_scientific.c):
//
//   0       sync nibble, always 0xA
//   1..4    sensor id, read in this order (THGR122N = 1 D 2 0 = 0x1D20)
//   5       channel (bit coded on 3 channel sensors: 1, 2, 4)
//   6..7    rolling code (changes when the batteries are replaced)
//   8       flags, bit 2 = low battery
//   9..11   temperature BCD: tenths, units, tens
//   12      temperature sign (bit 3)
//   13..14  humidity BCD: units, tens        (temperature/humidity sensors)
//   15      unknown (comfort level)          (temperature/humidity sensors)
//   next 2  checksum = sum of nibbles 1..N, low nibble first

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace oregon_v21_receiver {

enum class ParseResult : uint8_t {
  OK,
  TOO_SHORT,
  BAD_SYNC,
  UNKNOWN_SENSOR,
  BAD_CHECKSUM,
  BAD_VALUE,
};

struct OregonReading {
  uint16_t sensor_id{0};
  const char *model{""};
  uint8_t channel_raw{0};  // channel nibble as transmitted
  uint8_t channel{0};      // channel as shown on the base station (1..3 for bit coded sensors)
  uint8_t rolling_code{0};
  bool battery_low{false};
  bool has_temperature{false};
  float temperature{0.0f};  // degrees Celsius, outdoor sensor air temperature
  bool has_humidity{false};
  uint8_t humidity{0};      // percent relative humidity
};

/// Nibble n of an Oregon message in nibble order.
inline uint8_t oregon_nibble(const uint8_t *data, size_t n) {
  return (n & 1) ? (data[n / 2] >> 4) : (data[n / 2] & 0x0F);
}

/// Sensor id in rtl_433 notation (nibbles 1..4), or 0 if the buffer is too short.
uint16_t oregon_sensor_id(const uint8_t *data, size_t len);

/// Parse and validate a message. Only fills `out` completely when the result is OK.
ParseResult parse_oregon_v21(const uint8_t *data, size_t len, OregonReading &out);

const char *parse_result_to_string(ParseResult result);

}  // namespace oregon_v21_receiver
}  // namespace esphome
