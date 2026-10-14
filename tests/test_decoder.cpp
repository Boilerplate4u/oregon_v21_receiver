// Host unit tests for the ESPHome independent parts of oregon_v21_receiver.
// Build and run with tests/run_tests.sh (Debian/WSL, Linux or macOS).

#include <cmath>
#include <cstdio>
#include <string>

#include "test_helpers.h"

using namespace esphome::oregon_v21_receiver;
using namespace test;

static int failures = 0;
#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      failures++; \
    } \
  } while (0)

static void test_known_message() {
  // Printed by rene-d/oregon on the same ESP32 + RXB6 (2025-12-06):
  // "RAW: 1A 2D 40 F5 60 08 50 C7 4E A6 id: 245 channel: 4 temperature: 8.60 humidity: 75 bat: 0 ok"
  auto m = hex_bytes("1A 2D 40 F5 60 08 50 C7 4E A6");
  OregonReading r;
  CHECK(parse_oregon_v21(m.data(), m.size(), r) == ParseResult::OK);
  CHECK(r.sensor_id == 0x1D20);
  CHECK(r.channel_raw == 4);
  CHECK(r.channel == 3);
  CHECK(r.rolling_code == 245);
  CHECK(std::fabs(r.temperature - 8.6f) < 0.01f);
  CHECK(r.has_humidity && r.humidity == 75);
  CHECK(!r.battery_low);

  auto bad = m;
  bad[8] ^= 0x01;
  CHECK(parse_oregon_v21(bad.data(), bad.size(), r) == ParseResult::BAD_CHECKSUM);
  auto short_msg = hex_bytes("1A 2D 40 F5 60 08");
  CHECK(parse_oregon_v21(short_msg.data(), short_msg.size(), r) == ParseResult::TOO_SHORT);
  auto bad_sync = m;
  bad_sync[0] = 0x1B;
  CHECK(parse_oregon_v21(bad_sync.data(), bad_sync.size(), r) == ParseResult::BAD_SYNC);
  auto unknown = m;
  unknown[1] = 0x2E;  // sensor id 0x1E20
  CHECK(parse_oregon_v21(unknown.data(), unknown.size(), r) == ParseResult::UNKNOWN_SENSOR);
  CHECK(r.sensor_id == 0x1E20);
  // Checksum still valid but a non BCD temperature digit.
  auto non_bcd = m;
  set_nibble(non_bcd, 10, 0xC);
  fix_checksum(non_bcd, 15);
  CHECK(parse_oregon_v21(non_bcd.data(), non_bcd.size(), r) == ParseResult::BAD_VALUE);
  // Bit coded channel values other than 1, 2 and 4 are rejected (review F2).
  auto ch3raw = m;
  set_nibble(ch3raw, 5, 3);
  fix_checksum(ch3raw, 15);
  CHECK(parse_oregon_v21(ch3raw.data(), ch3raw.size(), r) == ParseResult::BAD_VALUE);
  // Unexpected bits in the sign nibble are rejected (review F3).
  auto sign_bits = m;
  set_nibble(sign_bits, 12, 0x9);
  fix_checksum(sign_bits, 15);
  CHECK(parse_oregon_v21(sign_bits.data(), sign_bits.size(), r) == ParseResult::BAD_VALUE);
  // used_bytes reports the checksummed part, also when the parse fails later.
  CHECK(parse_oregon_v21(m.data(), m.size(), r) == ParseResult::OK && r.used_bytes == 9);
  // Exactly long enough: 9 bytes holds nibbles 0..17.
  CHECK(parse_oregon_v21(m.data(), 9, r) == ParseResult::OK);
  CHECK(parse_oregon_v21(m.data(), 8, r) == ParseResult::TOO_SHORT);
}

static void test_round_trips() {
  {
    auto m = hex_bytes("1A 2D 20 7B 00 00 00 00 00 00");
    set_nibble(m, 8, 0x4);  // low battery
    set_nibble(m, 9, 5);    // -12.5 C
    set_nibble(m, 10, 2);
    set_nibble(m, 11, 1);
    set_nibble(m, 12, 0x8);
    set_nibble(m, 13, 3);  // 43 %
    set_nibble(m, 14, 4);
    fix_checksum(m, 15);
    auto msgs = decode_capture(encode_v21(m, 1));
    CHECK(msgs.size() == 1);
    OregonReading r;
    CHECK(!msgs.empty() && msgs[0].pair_errors == 0);
    CHECK(!msgs.empty() && parse_oregon_v21(msgs[0].bytes.data(), msgs[0].bytes.size(), r) == ParseResult::OK);
    CHECK(std::fabs(r.temperature + 12.5f) < 0.01f);
    CHECK(r.humidity == 43);
    CHECK(r.battery_low);
    CHECK(r.channel == 2);
    CHECK(r.rolling_code == 0x7B);
  }
  {
    auto m = hex_bytes("EA 4C 10 33 00 00 00 00");  // THN132N, id 0xEC40, channel 1
    set_nibble(m, 9, 7);
    set_nibble(m, 10, 1);
    set_nibble(m, 11, 2);  // 21.7 C
    fix_checksum(m, 12);
    auto msgs = decode_capture(encode_v21(m, 2));
    OregonReading r;
    CHECK(msgs.size() == 1);
    CHECK(!msgs.empty() && parse_oregon_v21(msgs[0].bytes.data(), msgs[0].bytes.size(), r) == ParseResult::OK);
    CHECK(r.sensor_id == 0xEC40);
    CHECK(!r.has_humidity);
    CHECK(std::fabs(r.temperature - 21.7f) < 0.01f);
  }
}

static void test_bit_pairs() {
  auto m = hex_bytes("1A 2D 40 F5 60 08 50 C7 4E A6");
  // A complement bit that equals its data bit inside the message is counted.
  auto raw = raw_bits_v21(m);
  raw[2 * 37 + 1] = raw[2 * 37];
  auto msgs = decode_capture(encode_raw(raw, 3));
  CHECK(msgs.size() == 1);
  CHECK(!msgs.empty() && msgs[0].pair_errors == 1);
  CHECK(!msgs.empty() && msgs[0].bytes == m);  // the data bits themselves are intact
  // An error in the unused tail byte after the checksum is outside the checked bytes (review F4).
  auto raw_unused = raw_bits_v21(m);
  raw_unused[2 * 76 + 1] = raw_unused[2 * 76];  // byte 9, after the checksum byte 8
  msgs = decode_capture(encode_raw(raw_unused, 9));
  CHECK(msgs.size() == 1);
  {
    ManchesterDecoder dec;
    auto pulses = encode_raw(raw_unused, 9);
    for (int32_t v : pulses)
      dec.feed(uint32_t(v < 0 ? -v : v));
    CHECK(dec.is_done());
    CHECK(dec.pair_errors() == 1);
    CHECK(dec.pair_errors(9) == 0);
    CHECK(dec.pair_errors(10) == 1);
  }
  // The same error in a trailing incomplete byte is not counted.
  auto raw_tail = raw_bits_v21(m);
  raw_tail.push_back(0);
  raw_tail.push_back(0);
  msgs = decode_capture(encode_raw(raw_tail, 4));
  CHECK(msgs.size() == 1);
  CHECK(!msgs.empty() && msgs[0].pair_errors == 0);
}

static void test_raw_bits_and_stats() {
  auto m = hex_bytes("1A 2D 40 F5 60 08 50 C7 4E A6");
  ManchesterDecoder dec;
  auto pulses = encode_v21(m, 5);
  bool done = false;
  for (int32_t v : pulses)
    done = dec.feed(uint32_t(v < 0 ? -v : v)) || done;
  CHECK(done);
  CHECK(dec.raw_bit_count() == m.size() * 16);
  auto expected = raw_bits_v21(m);
  bool same = true;
  for (size_t i = 0; i < dec.raw_bit_count(); i++)
    same = same && dec.raw_bit(i) == expected[i];
  CHECK(same);
  const auto &st = dec.pulse_stats();
  CHECK(st.short_count > 0 && st.long_count > 0);
  CHECK(st.short_avg_us() > 400 && st.short_avg_us() < 580);
  CHECK(st.long_avg_us() > 900 && st.long_avg_us() < 1060);
}

static void test_abort_snapshot() {
  auto m = hex_bytes("1A 2D 40 F5 60 08 50 C7 4E A6");
  auto pulses = encode_v21(m, 6);
  pulses.resize(pulses.size() / 2);
  pulses.push_back(5);  // noise pulse aborts the message half way
  ManchesterDecoder dec;
  dec.set_record_aborts(true);
  for (int32_t v : pulses)
    dec.feed(uint32_t(v < 0 ? -v : v));
  ManchesterDecoder::AbortInfo info;
  CHECK(dec.take_abort(info));
  CHECK(info.size >= 2 && info.size < m.size());
  CHECK(info.pulse_width_us == 5);
  CHECK(info.data[0] == 0x1A && info.data[1] == 0x2D);
  CHECK(!dec.take_abort(info));  // reported once
  // Without recording nothing is reported.
  ManchesterDecoder quiet;
  for (int32_t v : pulses)
    quiet.feed(uint32_t(v < 0 ? -v : v));
  CHECK(!quiet.take_abort(info));
}

static void test_two_messages_in_one_capture() {
  auto a = hex_bytes("1A 2D 40 F5 60 08 50 C7 4E A6");
  auto b = hex_bytes("1A 2D 10 FE 40 06 80 C9 55 EE");
  auto pa = encode_v21(a, 7);
  auto pb = encode_v21(b, 8);
  pa.insert(pa.end(), pb.begin(), pb.end());
  auto msgs = decode_capture(pa);
  CHECK(msgs.size() == 2);
  CHECK(msgs.size() == 2 && msgs[0].bytes == a && msgs[1].bytes == b);
}

static void test_real_captures(const std::string &path) {
  auto captures = load_captures(path);
  int oregon_lines = 0, decoded = 0, noise_lines = 0, noise_hits = 0, pair_errors = 0;
  for (const auto &cap : captures) {
    auto msgs = decode_capture(cap.raw);
    if (!cap.oregon) {
      noise_lines++;
      noise_hits += static_cast<int>(msgs.size());
      continue;
    }
    oregon_lines++;
    for (auto &msg : msgs) {
      OregonReading r;
      pair_errors += msg.pair_errors;
      ParseResult res = parse_oregon_v21(msg.bytes.data(), msg.bytes.size(), r);
      std::printf("  frame:");
      for (uint8_t b : msg.bytes)
        std::printf(" %02X", b);
      if (res == ParseResult::OK) {
        decoded++;
        std::printf("  -> %s ch %u (raw %u) rc %u %.1f C %u %% battery %s, pair errors %u\n", r.model, r.channel,
                    r.channel_raw, r.rolling_code, r.temperature, r.humidity, r.battery_low ? "low" : "ok",
                    msg.pair_errors);
      } else {
        std::printf("  -> %s\n", parse_result_to_string(res));
      }
    }
  }
  std::printf("real captures: %d oregon lines, %d decoded, %d pair errors, %d noise lines, %d noise hits\n",
              oregon_lines, decoded, pair_errors, noise_lines, noise_hits);
  CHECK(oregon_lines > 0);
  CHECK(decoded == oregon_lines);
  CHECK(pair_errors == 0);
  CHECK(noise_hits == 0);
}

int main(int argc, char **argv) {
  test_known_message();
  test_round_trips();
  test_bit_pairs();
  test_raw_bits_and_stats();
  test_abort_snapshot();
  test_two_messages_in_one_capture();
  test_real_captures(argc > 1 ? argv[1] : "data/rxb6_raw_frames.txt");
  std::printf(failures == 0 ? "ALL TESTS PASSED\n" : "%d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
