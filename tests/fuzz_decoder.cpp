// Randomised robustness test for ManchesterDecoder and parse_oregon_v21.
// Build and run with tests/run_tests.sh, which enables AddressSanitizer and UBSan.
//
// Three parts, all with a fixed seed so every run is reproducible:
//   1. Random pulse trains: the decoder must never misbehave and every accepted
//      message must pass an independent re-check.
//   2. Random byte buffers straight into the parser.
//   3. Mutated real and synthetic captures (pulses dropped, duplicated, split,
//      merged, stretched or replaced): counts how often a damaged capture is
//      still accepted with values that differ from the original ("false
//      accepts"), with and without the bit pair check.
//
// Set FUZZ_VERBOSE=1 to print every false accept that passes the pair check.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>

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

// Independent re-implementation of the checks parse_oregon_v21 promises.
static bool recheck_accepted(const std::vector<uint8_t> &b, const OregonReading &r) {
  if (b.empty() || (b[0] & 0x0F) != 0xA)
    return false;
  size_t last = r.has_humidity ? 15 : 12;
  if (b.size() * 2 < last + 3)
    return false;
  unsigned sum = 0;
  for (size_t i = 1; i <= last; i++)
    sum += oregon_nibble(b.data(), i);
  unsigned expected = oregon_nibble(b.data(), last + 1) | (oregon_nibble(b.data(), last + 2) << 4);
  if ((sum & 0xFF) != expected)
    return false;
  if (!(r.temperature >= -50.0f && r.temperature <= 70.0f))
    return false;
  if (r.has_humidity && r.humidity > 100)
    return false;
  return r.channel >= 1 && r.channel <= 15;
}

static void fuzz_random_pulses(uint32_t iterations, std::mt19937 &rng) {
  ManchesterDecoder dec;
  dec.set_record_aborts(true);
  std::uniform_int_distribution<int> kind(0, 9);
  std::uniform_int_distribution<uint32_t> len_dist(0, 1500);
  uint64_t full_buffer = 0;
  uint64_t messages = 0, accepted = 0, unknown_shown = 0;
  for (uint32_t it = 0; it < iterations; it++) {
    std::vector<int32_t> raw;
    if (it & 1) {
      // Valid Manchester timing for a random raw bit stream (up to beyond the
      // buffer limit of 400 raw bits), with rare noise pulses mixed in.
      std::vector<uint8_t> bits(std::uniform_int_distribution<size_t>(1, 480)(rng));
      for (auto &b : bits)
        b = rng() & 1;
      bits[0] = 0;
      raw = encode_raw(bits, rng(), 120);
      for (auto &v : raw) {
        if ((rng() % 400) == 0)
          v = ((rng() & 1) ? 1 : -1) * std::uniform_int_distribution<int32_t>(0, 4000)(rng);
      }
      if (rng() & 1)
        raw.pop_back();  // sometimes no end gap, the capture just ends
    } else {
      raw.resize(len_dist(rng));
      for (auto &v : raw) {
        int32_t w;
        switch (kind(rng)) {
          case 0:
            w = std::uniform_int_distribution<int32_t>(0, 100000)(rng);
            break;
          case 1:
            w = std::numeric_limits<int32_t>::min();  // must not overflow when negated
            break;
          case 2:
            w = std::numeric_limits<int32_t>::max();
            break;
          case 3:
          case 4:
          case 5:
            w = int32_t(std::normal_distribution<float>(488.0f, 120.0f)(rng));
            break;
          default:
            w = int32_t(std::normal_distribution<float>(976.0f, 150.0f)(rng));
            break;
        }
        v = ((rng() & 1) || w == std::numeric_limits<int32_t>::min()) ? w : -w;
      }
    }
    auto msgs = decode_capture(raw, dec);
    ManchesterDecoder::AbortInfo info;
    while (dec.take_abort(info))
      CHECK(info.size <= ManchesterDecoder::MAX_BYTES);
    for (auto &m : msgs) {
      messages++;
      if (m.bytes.size() + 1 >= ManchesterDecoder::MAX_BYTES)
        full_buffer++;
      CHECK(m.bytes.size() <= ManchesterDecoder::MAX_BYTES);
      CHECK(m.bytes.size() >= dec.get_timing().min_bytes);
      OregonReading r;
      const ParseResult res = parse_oregon_v21(m.bytes.data(), m.bytes.size(), r);
      if (res == ParseResult::OK) {
        accepted++;
        CHECK(recheck_accepted(m.bytes, r));
      }
      // Same rule as OregonV21Receiver: an unknown type is only logged when it has the
      // framing and its checksummed bytes pass the bit pair check.
      UnknownOregonMessage u;
      if (res == ParseResult::UNKNOWN_TYPE && parse_unknown_oregon_v21(m.bytes.data(), m.bytes.size(), u) &&
          m.pair_errors_in(u.used_bytes) == 0)
        unknown_shown++;
    }
  }
  std::printf("random pulses: %u captures, %llu candidate messages (%llu near the buffer limit), %llu accepted, "
              "%llu logged as unknown type\n",
              iterations, (unsigned long long) messages, (unsigned long long) full_buffer,
              (unsigned long long) accepted, (unsigned long long) unknown_shown);
  CHECK(messages > 0);
  CHECK(full_buffer > 0);
}

static void fuzz_random_bytes(uint32_t iterations, std::mt19937 &rng) {
  uint64_t ok = 0;
  std::uniform_int_distribution<size_t> len_dist(0, ManchesterDecoder::MAX_BYTES);
  for (uint32_t it = 0; it < iterations; it++) {
    std::vector<uint8_t> b(len_dist(rng));
    for (auto &x : b)
      x = uint8_t(rng());
    if (!b.empty() && (rng() & 1)) {
      // Bias towards plausible headers so the deeper code paths are exercised.
      b[0] = uint8_t((b[0] & 0xF0) | 0xA);
      if (b.size() > 2) {
        const uint8_t ids[][2] = {{0x1A, 0x2D}, {0x3A, 0x1D}, {0xEA, 0x4C}};
        auto &id = ids[rng() % 3];
        b[0] = id[0];
        b[1] = id[1];
      }
    }
    OregonReading r;
    ParseResult res = parse_oregon_v21(b.data(), b.size(), r);
    CHECK(parse_result_to_string(res) != nullptr);
    if (res == ParseResult::OK) {
      ok++;
      CHECK(recheck_accepted(b, r));
    }
  }
  std::printf("random bytes: %u buffers, %llu accepted (all re-checked)\n", iterations, (unsigned long long) ok);
}

static std::vector<int32_t> mutate(const std::vector<int32_t> &in, std::mt19937 &rng, int mutations) {
  std::vector<int32_t> out = in;
  for (int m = 0; m < mutations && out.size() > 2; m++) {
    size_t i = std::uniform_int_distribution<size_t>(0, out.size() - 2)(rng);
    switch (rng() % 6) {
      case 0:  // drop
        out.erase(out.begin() + i);
        break;
      case 1:  // duplicate
        out.insert(out.begin() + i, out[i]);
        break;
      case 2: {  // split into two
        int32_t a = out[i] / 2;
        out[i] -= a;
        out.insert(out.begin() + i + 1, -a);
        break;
      }
      case 3:  // merge with the next
        out[i] += (out[i] < 0 ? -1 : 1) * std::abs(out[i + 1]);
        out.erase(out.begin() + i + 1);
        break;
      case 4:  // stretch or shrink
        out[i] = int32_t(out[i] * std::uniform_real_distribution<float>(0.5f, 1.6f)(rng));
        break;
      default:  // replace
        out[i] = (out[i] < 0 ? -1 : 1) * std::uniform_int_distribution<int32_t>(1, 3000)(rng);
        break;
    }
  }
  return out;
}

struct Reference {
  std::vector<int32_t> pulses;
  std::vector<uint8_t> bytes;
};

static void fuzz_mutations(uint32_t iterations, std::mt19937 &rng, const std::vector<Reference> &refs) {
  uint64_t decoded_ok = 0, false_accept_checksum_only = 0, false_accept_with_pairs = 0, rejected_by_pairs = 0;
  uint64_t unknown_shown = 0;
  for (uint32_t it = 0; it < iterations; it++) {
    const auto &ref = refs[it % refs.size()];
    auto msgs = decode_capture(mutate(ref.pulses, rng, 1 + int(rng() % 3)));
    for (auto &m : msgs) {
      OregonReading r;
      const ParseResult res = parse_oregon_v21(m.bytes.data(), m.bytes.size(), r);
      if (res == ParseResult::UNKNOWN_TYPE) {
        UnknownOregonMessage u;
        if (parse_unknown_oregon_v21(m.bytes.data(), m.bytes.size(), u) && m.pair_errors_in(u.used_bytes) == 0)
          unknown_shown++;
      }
      if (res != ParseResult::OK)
        continue;
      CHECK(recheck_accepted(m.bytes, r));
      // Compare the checksummed part only; the tail after the checksum is unused.
      const size_t used = r.has_humidity ? 9 : 8;
      const bool same = m.bytes.size() >= used && ref.bytes.size() >= used &&
                        std::equal(ref.bytes.begin(), ref.bytes.begin() + used, m.bytes.begin());
      if (same) {
        decoded_ok++;
        continue;
      }
      false_accept_checksum_only++;
      // Same rule as OregonV21Receiver: only the checksummed bytes are verified.
      if (m.pair_errors_in(r.used_bytes) == 0) {
        false_accept_with_pairs++;
        if (std::getenv("FUZZ_VERBOSE")) {
          std::printf("  FALSE ACCEPT data_bits=%u ref:", m.data_bits);
          for (uint8_t x : ref.bytes)
            std::printf(" %02X", x);
          std::printf("  got:");
          for (uint8_t x : m.bytes)
            std::printf(" %02X", x);
          std::printf("\n");
        }
      } else
        rejected_by_pairs++;
    }
  }
  std::printf("mutations: %u damaged captures, %llu still decoded correctly\n", iterations,
              (unsigned long long) decoded_ok);
  std::printf("  wrong values accepted with checksum only: %llu\n", (unsigned long long) false_accept_checksum_only);
  std::printf("  wrong values accepted with bit pair check: %llu (%llu caught by the pair check)\n",
              (unsigned long long) false_accept_with_pairs, (unsigned long long) rejected_by_pairs);
  // Damage in the type nibbles turns a known message into an unknown type. It is only
  // logged at DEBUG and never published.
  std::printf("  damaged messages logged as unknown type: %llu\n", (unsigned long long) unknown_shown);
  // The pair check cannot catch damage that keeps every pair complementary (for example a
  // section with inverted polarity); then only the 8 bit checksum protects. Require that the
  // pair check removes at least 90 % of the wrong values that pass the checksum.
  CHECK(false_accept_checksum_only > 0);
  CHECK(false_accept_with_pairs * 10 <= false_accept_checksum_only);
}

int main(int argc, char **argv) {
  const std::string data = argc > 1 ? argv[1] : "data/rxb6_raw_frames.txt";
  const uint32_t scale = argc > 2 ? uint32_t(std::strtoul(argv[2], nullptr, 10)) : 1;
  std::mt19937 rng(20251209);

  std::vector<Reference> refs;
  for (const auto &cap : load_captures(data)) {
    if (!cap.oregon)
      continue;
    auto msgs = decode_capture(cap.raw);
    if (!msgs.empty())
      refs.push_back({cap.raw, msgs[0].bytes});
  }
  for (uint32_t s = 0; s < 8; s++) {
    auto m = hex_bytes("1A 2D 20 7B 00 00 00 00 00 00");
    for (size_t n = 5; n <= 14; n++)
      set_nibble(m, n, n == 5 ? uint8_t(1u << (s % 3)) : n == 12 ? uint8_t((s & 1) * 8) : uint8_t(rng() % 10));
    fix_checksum(m, 15);
    refs.push_back({encode_v21(m, 100 + s), m});
  }
  CHECK(!refs.empty());

  fuzz_random_pulses(20000 * scale, rng);
  fuzz_random_bytes(500000 * scale, rng);
  fuzz_mutations(200000 * scale, rng, refs);

  std::printf(failures == 0 ? "FUZZ PASSED\n" : "%d FUZZ FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
