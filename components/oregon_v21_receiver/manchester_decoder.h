#pragma once

// Manchester decoder for Oregon Scientific v2.1 OOK pulse trains.
//
// Pure C++ without ESPHome dependencies so it can be unit tested on a host.
//
// The state machine is a port of OregonDecoderV2 from rene-d/oregon
// (https://github.com/rene-d/oregon, include/Oregon.h), which in turn is based
// on ookDecoder by Jean-Claude Wippler (MIT license) with Oregon v2 additions by
// Dominique Pierre and Olivier Lebrun.
//
// Input is the sequence of pulse widths in microseconds, both marks and spaces,
// exactly as ESPHome remote_receiver delivers them (sign removed).
//
// Oregon v2.1 sends every data bit twice (the bit followed by its inverse).
// Only every second decoded bit is stored, so the output bytes contain the
// real payload. Within each byte the first received bit ends up in bit 0, which
// gives "nibble order": the low nibble of byte 0 is nibble 0 (the sync nibble A).

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace oregon_v21_receiver {

class ManchesterDecoder {
 public:
  static constexpr size_t MAX_BYTES = 25;

  struct Timing {
    uint32_t min_pulse_us{200};   // shorter pulses are treated as noise
    uint32_t long_pulse_us{700};  // pulses at or above this are "long"
    uint32_t max_pulse_us{1200};  // longer pulses are treated as noise
    uint32_t end_gap_us{2500};    // a gap this long ends a message
    uint8_t min_preamble{24};     // long pulses required before the first short pulse
    uint8_t min_bytes{8};         // shortest accepted message
  };

  ManchesterDecoder() { this->reset(); }
  explicit ManchesterDecoder(const Timing &timing) : timing_(timing) { this->reset(); }

  void set_timing(const Timing &timing) { this->timing_ = timing; }
  const Timing &get_timing() const { return this->timing_; }

  /// Clear all state and wait for a new preamble.
  void reset();

  /// Feed one pulse width. Returns true when a complete message is available.
  bool feed(uint32_t width_us);

  /// Call when the capture buffer ends. Returns true if a message of at least
  /// min_bytes was in progress, which is then made available.
  bool finish();

  bool is_done() const { return this->state_ == DONE; }
  const uint8_t *data() const { return this->data_; }
  size_t size() const { return this->pos_; }

 protected:
  enum State : uint8_t { UNKNOWN, T0, OK, DONE };

  // Returns -1 to reset, 1 when the message is complete, 0 to continue.
  int8_t decode_(uint32_t width_us);
  void got_bit_(uint8_t value);
  void manchester_(uint8_t value);

  Timing timing_{};
  uint8_t data_[MAX_BYTES]{};
  uint16_t total_bits_{0};
  uint8_t pos_{0};
  uint8_t flip_{0};
  State state_{UNKNOWN};
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
