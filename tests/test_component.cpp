// Host test of the ESPHome glue (OregonV21Receiver / OregonDevice) against minimal
// stubs of the ESPHome headers in tests/stubs. Build and run with tests/run_tests.sh.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "oregon_v21_receiver.h"
#include "test_helpers.h"

using namespace esphome;
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

static bool near(float a, float b) { return std::fabs(a - b) < 0.01f; }

static size_t count_log(char level, const char *needle) {
  size_t n = 0;
  for (const auto &l : test_log())
    if (l.level == level && l.text.find(needle) != std::string::npos)
      n++;
  return n;
}

static void receive(OregonV21Receiver &rx, const std::vector<int32_t> &raw) {
  rx.on_receive(remote_base::RemoteReceiveData(raw));
}

// THGR122N message on a given raw channel nibble with a given rolling code and temperature.
static std::vector<int32_t> thgr_capture(uint8_t channel_raw, uint8_t rolling_code, uint8_t tens, uint8_t units,
                                         uint32_t seed, bool pair_error = false, bool bad_checksum = false) {
  auto m = hex_bytes("1A 2D 00 00 00 00 00 00 00 00");
  set_nibble(m, 5, channel_raw);
  set_nibble(m, 6, rolling_code & 0x0F);
  set_nibble(m, 7, rolling_code >> 4);
  set_nibble(m, 10, units);
  set_nibble(m, 11, tens);
  set_nibble(m, 13, 5);
  set_nibble(m, 14, 4);  // 45 %
  fix_checksum(m, 15);
  if (bad_checksum)
    m[8] ^= 0x10;
  auto raw = raw_bits_v21(m);
  if (pair_error)
    raw[2 * 40 + 1] = raw[2 * 40];
  return encode_raw(raw, seed);
}

struct Rig {
  OregonV21Receiver rx;
  OregonDevice dev[3];
  sensor::Sensor temp[3], hum[3];
  binary_sensor::BinarySensor batt[3];
  sensor::Sensor messages, checksum, pairs, rejected, aborted;
  Rig() {
    test_log().clear();
    test_millis() = 1000;
    rx.set_messages_sensor(&messages);
    rx.set_checksum_errors_sensor(&checksum);
    rx.set_bit_pair_errors_sensor(&pairs);
    rx.set_rejected_sensor(&rejected);
    rx.set_aborted_sensor(&aborted);
  }
  void add(int i, uint8_t channel, int rolling_code = -1) {
    dev[i].set_channel(channel);
    if (rolling_code >= 0)
      dev[i].set_rolling_code(uint8_t(rolling_code));
    dev[i].set_temperature_sensor(&temp[i]);
    dev[i].set_humidity_sensor(&hum[i]);
    dev[i].set_battery_low_binary_sensor(&batt[i]);
    rx.add_device(&dev[i]);
  }
  void tick(uint32_t ms) { test_millis() += ms; }
};

static void test_real_captures(const std::string &path) {
  Rig r;
  r.add(0, 1);       // channel 1, any rolling code
  r.add(1, 3, 245);  // channel 3, rolling code 245
  r.rx.setup();
  int n = 0;
  for (const auto &cap : load_captures(path)) {
    if (!cap.oregon)
      continue;
    receive(r.rx, cap.raw);
    r.tick(20000);
    n++;
  }
  CHECK(n == 12);
  CHECK(r.temp[0].states.size() == 6);
  CHECK(r.temp[1].states.size() == 6);
  CHECK(!r.temp[0].states.empty() && near(r.temp[0].states.front(), 6.4f));
  CHECK(!r.temp[0].states.empty() && near(r.temp[0].states.back(), 6.3f));  // last channel 1 capture in the file
  CHECK(!r.hum[0].states.empty() && near(r.hum[0].states.front(), 98.0f));
  CHECK(!r.temp[1].states.empty() && near(r.temp[1].states.front(), 8.1f));
  CHECK(!r.hum[1].states.empty() && near(r.hum[1].states.front(), 75.0f));
  CHECK(!r.batt[0].states.empty() && r.batt[0].states.front() == false);
  bool saw_63 = false;
  for (float t : r.temp[0].states)
    saw_63 = saw_63 || near(t, 6.3f);
  CHECK(saw_63);
  r.rx.update();
  CHECK(!r.messages.states.empty() && near(r.messages.states.back(), 12.0f));
  CHECK(!r.checksum.states.empty() && near(r.checksum.states.back(), 0.0f));
  CHECK(count_log('W', "rolling codes") == 0);
  // VERBOSE dump is produced for every message.
  CHECK(count_log('V', "Manchester message: 10 bytes, 0 bit pair errors") == 12);
  CHECK(count_log('V', "raw nibbles  0- 7:") == 12);
}

static void test_repeat_is_ignored() {
  Rig r;
  r.add(0, 1);
  auto cap = thgr_capture(1, 0x42, 2, 1, 1);
  receive(r.rx, cap);
  r.tick(100);
  receive(r.rx, cap);  // the sensor's own repeat
  CHECK(r.temp[0].states.size() == 1);
  r.tick(40000);
  receive(r.rx, cap);  // next transmission, same value
  CHECK(r.temp[0].states.size() == 2);
  r.rx.update();
  CHECK(near(r.messages.states.back(), 2.0f));
}

static void test_rolling_code_precedence() {
  Rig r;
  r.add(0, 1);      // whole channel 1
  r.add(1, 1, 77);  // rolling code 77 on channel 1
  receive(r.rx, thgr_capture(1, 77, 1, 5, 2));
  CHECK(r.temp[1].states.size() == 1 && near(r.temp[1].states[0], 15.0f));
  CHECK(r.temp[0].states.empty());  // exact match wins over the channel wide device
  r.tick(5000);
  receive(r.rx, thgr_capture(1, 12, 1, 6, 3));
  CHECK(r.temp[0].states.size() == 1 && near(r.temp[0].states[0], 16.0f));
  CHECK(r.temp[1].states.size() == 1);
}

static void test_conflict_warning() {
  Rig r;
  r.add(0, 1);
  receive(r.rx, thgr_capture(1, 254, 0, 6, 4));
  r.tick(39000);
  receive(r.rx, thgr_capture(1, 77, 2, 2, 5));  // neighbour on the same channel
  CHECK(count_log('W', "rolling codes 254 and 77") == 1);
  r.tick(39000);
  receive(r.rx, thgr_capture(1, 254, 0, 7, 6));
  CHECK(count_log('W', "rolling codes") == 1);  // warned once
  // A change after more than 10 minutes (battery change) does not warn.
  Rig q;
  q.add(0, 2);
  receive(q.rx, thgr_capture(2, 10, 0, 6, 7));
  q.tick(11 * 60 * 1000);
  receive(q.rx, thgr_capture(2, 99, 0, 6, 8));
  CHECK(count_log('W', "rolling codes") == 0);
}

static void test_unmatched_logging() {
  Rig r;
  r.add(0, 1);
  receive(r.rx, thgr_capture(4, 245, 0, 8, 9));  // channel 3, not configured
  r.tick(43000);
  receive(r.rx, thgr_capture(4, 245, 0, 9, 10));
  CHECK(count_log('I', "on channel 3 with rolling code 245 is not configured") == 1);
  CHECK(count_log('D', "No device configured for channel 3 rolling code 245") == 1);
  CHECK(r.temp[0].states.empty());
}

static void test_rejections_and_counters() {
  Rig r;
  r.add(0, 1);
  r.rx.setup();
  receive(r.rx, thgr_capture(1, 5, 1, 1, 11, true));  // bit pair error
  CHECK(r.temp[0].states.empty());
  receive(r.rx, thgr_capture(1, 5, 1, 1, 12, false, true));  // bad checksum
  CHECK(r.temp[0].states.empty());
  auto truncated = thgr_capture(1, 5, 1, 1, 13);
  truncated.resize(truncated.size() / 2);
  truncated.push_back(5);  // noise ends it half way
  receive(r.rx, truncated);
  r.rx.update();
  CHECK(near(r.pairs.states.back(), 1.0f));
  CHECK(near(r.checksum.states.back(), 1.0f));
  CHECK(near(r.aborted.states.back(), 1.0f));
  CHECK(near(r.messages.states.back(), 0.0f));
  CHECK(count_log('V', "Candidate abandoned") == 1);

  // With verification disabled the same message is accepted and the errors are logged.
  Rig q;
  q.add(0, 1);
  q.rx.set_verify_bit_pairs(false);
  receive(q.rx, thgr_capture(1, 5, 1, 1, 14, true));
  CHECK(q.temp[0].states.size() == 1);
  CHECK(count_log('D', "bit pair errors (verification disabled)") == 1);
}

// The THGR122N message from the 12:47:26 log, cut off after `raw_bits` raw bits by `cut_us`.
static std::vector<int32_t> cut_thgr_capture(size_t raw_bits, int32_t cut_us, uint32_t seed) {
  auto raw = raw_bits_v21(hex_bytes("1A 2D 40 F5 81 09 30 C7 50 D9"));
  raw.resize(raw_bits);
  auto pulses = encode_raw(raw, seed);
  pulses.back() = pulses[pulses.size() - 2] < 0 ? cut_us : -cut_us;
  pulses.push_back(-6000);
  return pulses;
}

static void test_truncated_candidates() {
  Rig r;
  r.add(0, 3);
  r.rx.setup();
  receive(r.rx, cut_thgr_capture(144, 1944, 40));  // complete up to the checksum
  CHECK(r.temp[0].states.size() == 1 && near(r.temp[0].states[0], 9.8f));
  CHECK(r.hum[0].states.size() == 1 && near(r.hum[0].states[0], 73.0f));
  CHECK(count_log('V', "cut off by a 1944 us pulse after 144 raw bits") == 1);
  r.tick(200);
  receive(r.rx, encode_v21(hex_bytes("1A 2D 40 F5 81 09 30 C7 50 D9"), 41));  // its complete repeat
  CHECK(r.temp[0].states.size() == 1);
  r.tick(43000);
  receive(r.rx, cut_thgr_capture(131, 1366, 42));  // checksum not complete
  CHECK(r.temp[0].states.size() == 1);
  CHECK(count_log('V', "Candidate abandoned after 131 raw bits (8 bytes) at a 1366 us pulse") == 1);
  CHECK(count_log('D', "Rejected") == 0);
  r.rx.update();
  CHECK(near(r.messages.states.back(), 1.0f));
  CHECK(near(r.aborted.states.back(), 1.0f));
  CHECK(near(r.rejected.states.back(), 0.0f));
  CHECK(near(r.checksum.states.back(), 0.0f));
  CHECK(near(r.pairs.states.back(), 0.0f));
}

int main(int argc, char **argv) {
  test_real_captures(argc > 1 ? argv[1] : "data/rxb6_raw_frames.txt");
  test_repeat_is_ignored();
  test_rolling_code_precedence();
  test_conflict_warning();
  test_unmatched_logging();
  test_rejections_and_counters();
  test_truncated_candidates();
  std::printf(failures == 0 ? "COMPONENT TESTS PASSED\n" : "%d COMPONENT FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
