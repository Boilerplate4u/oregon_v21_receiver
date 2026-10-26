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
//   1..4    type, read in this order (THGR122N = 1 D 2 0 = 0x1D20)
//   5       channel (bit coded on 3 channel sensors: 1, 2, 4; other values are rejected)
//   6..7    unit_id, identifies the remote sensor (changes when the batteries are replaced)
//   8       flags, bit 2 = low battery
//   9..11   temperature BCD: tenths, units, tens
//   12      temperature sign (bit 3); the other bits must be 0 on the supported sensors
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
  UNKNOWN_TYPE,
  BAD_CHECKSUM,
  BAD_VALUE,
};

struct OregonReading {
  uint16_t type{0};
  const char *model{""};
  uint8_t channel_raw{0};  // channel nibble as transmitted
  uint8_t channel{0};      // channel as shown on the base station (1..3 for bit coded sensors)
  uint8_t unit_id{0};
  bool battery_low{false};
  bool has_temperature{false};
  float temperature{0.0f};  // degrees Celsius, outdoor sensor air temperature
  bool has_humidity{false};
  uint8_t humidity{0};    // percent relative humidity
  uint8_t used_bytes{0};  // bytes covered by the checksum, set once the sensor type is known
};

/// Nibble n of an Oregon message in nibble order.
inline uint8_t oregon_nibble(const uint8_t *data, size_t n) {
  return (n & 1) ? (data[n / 2] >> 4) : (data[n / 2] & 0x0F);
}

/// Type in rtl_433 notation (nibbles 1..4), or 0 if the buffer is too short.
uint16_t oregon_type(const uint8_t *data, size_t len);

/// Parse and validate a message. Only fills `out` completely when the result is OK.
ParseResult parse_oregon_v21(const uint8_t *data, size_t len, OregonReading &out);

const char *parse_result_to_string(ParseResult result);

/// A message with an unknown type that still has the Oregon v2.1 framing: sync
/// nibble 0xA and a nibble checksum at some position. Only for logging, the field
/// layout of an unknown type is not known.
struct UnknownOregonMessage {
  uint16_t type{0};
  uint8_t channel_raw{0};  // nibble 5
  uint8_t unit_id{0};      // nibbles 6..7
  uint8_t flags{0};        // nibble 8
  // The checksum covers nibbles 1..last_checksummed_nibble.
  uint8_t last_checksummed_nibble{0};
  uint8_t used_bytes{0};  // bytes up to and including the checksum
  // Several positions can match by chance; the largest one is used.
  uint8_t checksum_matches{0};
  // Values read with the THGR122N layout, only set when the nibbles are valid there.
  bool has_guessed_temperature{false};
  float guessed_temperature{0.0f};
  bool has_guessed_humidity{false};
  uint8_t guessed_humidity{0};
};

/// Shortest and longest checksummed range tried for an unknown type (nibbles 1..N).
static constexpr uint8_t UNKNOWN_MIN_CHECKSUMMED_NIBBLE = 8;
static constexpr uint8_t UNKNOWN_MAX_CHECKSUMMED_NIBBLE = 20;

/// Look for the Oregon v2.1 framing in a message whose type is not known.
/// Returns true when the sync nibble is 0xA and a checksum position matches.
bool parse_unknown_oregon_v21(const uint8_t *data, size_t len, UnknownOregonMessage &out);

}  // namespace oregon_v21_receiver
}  // namespace esphome
