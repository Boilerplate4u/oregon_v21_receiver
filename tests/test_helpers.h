#pragma once

// Shared helpers for the host tests.

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "manchester_decoder.h"
#include "oregon_protocol.h"

namespace test {

using esphome::oregon_v21_receiver::ManchesterDecoder;
using esphome::oregon_v21_receiver::oregon_nibble;

struct Message {
  std::vector<uint8_t> bytes;
  uint16_t pair_errors{0};
  uint16_t data_bits{0};
  std::vector<uint16_t> pair_errors_upto;  // pair_errors_upto[n] = errors in the first n bytes
  uint16_t pair_errors_in(size_t n) const {
    return pair_errors_upto.empty() ? 0
                                    : pair_errors_upto[n < pair_errors_upto.size() ? n : pair_errors_upto.size() - 1];
  }
};

inline std::vector<uint8_t> hex_bytes(const std::string &s) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < s.size(); i += 3)
    out.push_back(static_cast<uint8_t>(std::strtoul(s.substr(i, 2).c_str(), nullptr, 16)));
  return out;
}

// Decode a whole capture the same way OregonV21Receiver::on_receive does.
inline std::vector<Message> decode_capture(const std::vector<int32_t> &raw, ManchesterDecoder &dec) {
  std::vector<Message> messages;
  auto take = [&]() {
    Message m;
    m.bytes.assign(dec.data(), dec.data() + dec.size());
    m.pair_errors = dec.pair_errors();
    m.data_bits = dec.data_bit_count();
    for (size_t n = 0; n <= dec.size(); n++)
      m.pair_errors_upto.push_back(dec.pair_errors(n));
    messages.push_back(m);
  };
  dec.reset();
  for (int32_t v : raw) {
    const uint32_t width = v < 0 ? uint32_t(-(int64_t) v) : uint32_t(v);
    if (dec.feed(width)) {
      take();
      dec.reset();
    }
  }
  if (dec.finish())
    take();
  dec.reset();
  return messages;
}

inline std::vector<Message> decode_capture(const std::vector<int32_t> &raw) {
  ManchesterDecoder dec;
  return decode_capture(raw, dec);
}

// Raw v2.1 bit stream for a message: every data bit followed by its complement.
inline std::vector<uint8_t> raw_bits_v21(const std::vector<uint8_t> &msg) {
  std::vector<uint8_t> raw;
  for (uint8_t byte : msg) {
    for (int bit = 0; bit < 8; bit++) {
      uint8_t b = (byte >> bit) & 1;
      raw.push_back(b);
      raw.push_back(b ^ 1);
    }
  }
  return raw;
}

// Pulse train for a raw bit stream: a long pulse where the stream changes and two
// short pulses where it repeats, preceded by a preamble of long pulses.
inline std::vector<int32_t> encode_raw(const std::vector<uint8_t> &raw, uint32_t seed, int32_t jitter_us = 60) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int32_t> jitter(-jitter_us, jitter_us);
  std::vector<int32_t> pulses;
  int sign = 1;
  auto emit = [&](int32_t w) {
    pulses.push_back(sign * (w + jitter(rng)));
    sign = -sign;
  };
  for (int i = 0; i < 32; i++)
    emit(976);  // preamble
  emit(488);    // the first raw bit must be 0 (the sync nibble A starts with 0)
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

inline std::vector<int32_t> encode_v21(const std::vector<uint8_t> &msg, uint32_t seed) {
  return encode_raw(raw_bits_v21(msg), seed);
}

inline void set_nibble(std::vector<uint8_t> &m, size_t n, uint8_t v) {
  if (n & 1)
    m[n / 2] = uint8_t((m[n / 2] & 0x0F) | (v << 4));
  else
    m[n / 2] = uint8_t((m[n / 2] & 0xF0) | (v & 0x0F));
}

inline void fix_checksum(std::vector<uint8_t> &m, size_t last) {
  uint8_t sum = 0;
  for (size_t i = 1; i <= last; i++)
    sum += oregon_nibble(m.data(), i);
  set_nibble(m, last + 1, sum & 0x0F);
  set_nibble(m, last + 2, sum >> 4);
}

struct Capture {
  bool oregon;
  std::vector<int32_t> raw;
};

inline std::vector<Capture> load_captures(const std::string &path) {
  std::vector<Capture> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream ls(line);
    std::string kind, csv;
    ls >> kind >> csv;
    Capture c;
    c.oregon = kind == "oregon";
    std::stringstream cs(csv);
    std::string tok;
    while (std::getline(cs, tok, ','))
      c.raw.push_back(std::stoi(tok));
    out.push_back(c);
  }
  return out;
}

}  // namespace test
