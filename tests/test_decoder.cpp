// Host unit tests for the ESPHome independent parts of oregon_v21_receiver.
//
// Build and run from the tests directory (Linux, macOS or WSL), see README.md:
//   g++ -std=c++17 -Wall -Wextra -I../components/oregon_v21_receiver test_decoder.cpp
//       ../components/oregon_v21_receiver/manchester_decoder.cpp
//       ../components/oregon_v21_receiver/oregon_protocol.cpp -o test_decoder
//   ./test_decoder

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "manchester_decoder.h"
#include "oregon_protocol.h"

using namespace esphome::oregon_v21_receiver;

static int failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
      failures++;                                                       \
    }                                                                   \
  } while (0)

static std::vector<uint8_t> hex_bytes(const char *hex) {
  std::vector<uint8_t> out;
  std::string s(hex);
  for (size_t i = 0; i + 1 < s.size(); i += 3)
    out.push_back(static_cast<uint8_t>(std::strtoul(s.substr(i, 2).c_str(), nullptr, 16)));
  return out;
}

// Decode a whole capture the same way OregonV21Receiver::on_receive does.
static std::vector<std::vector<uint8_t>> decode_capture(const std::vector<int32_t> &raw) {
  std::vector<std::vector<uint8_t>> messages;
  ManchesterDecoder dec;
  for (int32_t v : raw) {
    if (dec.feed(static_cast<uint32_t>(v < 0 ? -v : v))) {
      messages.emplace_back(dec.data(), dec.data() + dec.size());
      dec.reset();
    }
  }
  if (dec.finish())
    messages.emplace_back(dec.data(), dec.data() + dec.size());
  return messages;
}

// Build a pulse train for a v2.1 message: every bit followed by its inverse,
// a long pulse where the raw bit stream changes and two short pulses where it repeats.
static std::vector<int32_t> encode_v21(const std::vector<uint8_t> &msg, unsigned seed) {
  std::vector<uint8_t> raw;
  for (uint8_t byte : msg) {
    for (int bit = 0; bit < 8; bit++) {
      uint8_t b = (byte >> bit) & 1;
      raw.push_back(b);
      raw.push_back(b ^ 1);
    }
  }
  std::srand(seed);
  auto jitter = [](int32_t w) { return w + (std::rand() % 121) - 60; };
  std::vector<int32_t> pulses;
  int sign = 1;
  auto emit = [&](int32_t w) {
    pulses.push_back(sign * jitter(w));
    sign = -sign;
  };
  for (int i = 0; i < 32; i++)
    emit(976);  // preamble
  emit(488);    // first raw bit must be 0 (sync nibble A starts with 0)
  emit(488);
  for (size_t i = 1; i < raw.size(); i++) {
    if (raw[i] == raw[i - 1]) {
      emit(488);
      emit(488);
    } else {
      emit(976);
    }
  }
  pulses.push_back(-6000);
  return pulses;
}

static void set_nibble(std::vector<uint8_t> &m, size_t n, uint8_t v) {
  if (n & 1)
    m[n / 2] = (m[n / 2] & 0x0F) | (v << 4);
  else
    m[n / 2] = (m[n / 2] & 0xF0) | (v & 0x0F);
}

static void fix_checksum(std::vector<uint8_t> &m, size_t last) {
  uint8_t sum = 0;
  for (size_t i = 1; i <= last; i++)
    sum += oregon_nibble(m.data(), i);
  set_nibble(m, last + 1, sum & 0x0F);
  set_nibble(m, last + 2, sum >> 4);
}

int main(int argc, char **argv) {
  // 1. Message printed by rene-d/oregon on the same ESP32 + RXB6 (Perplexity thread, 2025-12-06):
  //    "RAW: 1A 2D 40 F5 60 08 50 C7 4E A6 id: 245 channel: 4 temperature: 8.60 humidity: 75 bat: 0 ok"
  {
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
  }

  // 2. Synthetic round trips: negative temperature, low battery, temperature only sensor.
  {
    auto m = hex_bytes("1A 2D 20 7B 00 00 00 00 00 00");
    set_nibble(m, 8, 0x4);   // low battery
    set_nibble(m, 9, 5);     // -12.5 C
    set_nibble(m, 10, 2);
    set_nibble(m, 11, 1);
    set_nibble(m, 12, 0x8);
    set_nibble(m, 13, 3);    // 43 %
    set_nibble(m, 14, 4);
    fix_checksum(m, 15);
    auto msgs = decode_capture(encode_v21(m, 1));
    CHECK(msgs.size() == 1);
    OregonReading r;
    CHECK(!msgs.empty() && parse_oregon_v21(msgs[0].data(), msgs[0].size(), r) == ParseResult::OK);
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
    CHECK(!msgs.empty() && parse_oregon_v21(msgs[0].data(), msgs[0].size(), r) == ParseResult::OK);
    CHECK(r.sensor_id == 0xEC40);
    CHECK(!r.has_humidity);
    CHECK(std::fabs(r.temperature - 21.7f) < 0.01f);
  }

  // 3. Real captures from ESPHome remote_receiver on the RXB6.
  {
    const char *path = argc > 1 ? argv[1] : "data/rxb6_raw_frames.txt";
    std::ifstream in(path);
    CHECK(in.good());
    std::string line;
    int oregon_lines = 0, decoded = 0, noise_lines = 0, noise_hits = 0;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::istringstream ls(line);
      std::string kind, csv;
      ls >> kind >> csv;
      std::vector<int32_t> raw;
      std::stringstream cs(csv);
      std::string tok;
      while (std::getline(cs, tok, ','))
        raw.push_back(std::stoi(tok));
      auto msgs = decode_capture(raw);
      if (kind == "noise") {
        noise_lines++;
        noise_hits += static_cast<int>(msgs.size());
        continue;
      }
      oregon_lines++;
      for (auto &msg : msgs) {
        OregonReading r;
        ParseResult res = parse_oregon_v21(msg.data(), msg.size(), r);
        std::printf("  frame:");
        for (uint8_t b : msg)
          std::printf(" %02X", b);
        if (res == ParseResult::OK) {
          decoded++;
          std::printf("  -> %s ch %u (raw %u) rc %u %.1f C %u %% battery %s\n", r.model, r.channel,
                      r.channel_raw, r.rolling_code, r.temperature, r.humidity,
                      r.battery_low ? "low" : "ok");
        } else {
          std::printf("  -> %s\n", parse_result_to_string(res));
        }
      }
    }
    std::printf("real captures: %d oregon lines, %d decoded, %d noise lines, %d noise hits\n",
                oregon_lines, decoded, noise_lines, noise_hits);
    CHECK(oregon_lines > 0);
    CHECK(decoded == oregon_lines);
    CHECK(noise_hits == 0);
  }

  std::printf(failures == 0 ? "ALL TESTS PASSED\n" : "%d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
