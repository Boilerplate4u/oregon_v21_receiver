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
// Oregon v2.1 sends every data bit as a pair: the bit and its complement. Only
// the first bit of each pair is stored, so the output bytes contain the real
// payload. The second bit is compared with the first and every pair where they
// are equal is counted as a pair error (see pair_errors()).
//
// Within each byte the first received bit ends up in bit 0, which gives
// "nibble order": the low nibble of byte 0 is nibble 0 (the sync nibble A).

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace oregon_v21_receiver {

class ManchesterDecoder {
 public:
  static constexpr size_t MAX_BYTES = 25;
  static constexpr size_t MAX_RAW_BITS = MAX_BYTES * 16;

  struct Timing {
    uint32_t min_pulse_us{200};   // shorter pulses are treated as noise
    uint32_t long_pulse_us{700};  // pulses at or above this are "long"
    uint32_t max_pulse_us{1200};  // longer pulses are treated as noise
    uint32_t end_gap_us{2500};    // a gap this long ends a message
    uint8_t min_preamble{24};     // long pulses required before the first short pulse
    uint8_t min_bytes{8};         // shortest accepted message
  };

  /// Pulse width statistics for the data part of the current message.
  struct PulseStats {
    uint32_t short_sum_us{0};
    uint32_t long_sum_us{0};
    uint16_t short_count{0};
    uint16_t long_count{0};
    uint32_t short_avg_us() const { return short_count ? short_sum_us / short_count : 0; }
    uint32_t long_avg_us() const { return long_count ? long_sum_us / long_count : 0; }
  };

  /// Snapshot of a message that was abandoned after the preamble.
  struct AbortInfo {
    uint8_t data[MAX_BYTES]{};
    uint8_t size{0};             // complete bytes received
    uint16_t raw_bits{0};        // raw bits (both halves of each pair) received
    uint32_t pulse_width_us{0};  // the pulse that caused the abort
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

  /// Bit pairs within the complete bytes where both halves were equal.
  uint16_t pair_errors() const { return this->pair_errors_; }
  /// Bit pair errors within the first `bytes` complete bytes.
  uint16_t pair_errors(size_t bytes) const;

  /// Raw bit i (0 based, both halves of each pair) of the current message.
  uint8_t raw_bit(size_t i) const { return (this->raw_bits_[i / 8] >> (i % 8)) & 1; }
  /// Raw bits belonging to the complete bytes of the current message.
  size_t raw_bit_count() const { return size_t(this->pos_) * 16; }

  /// Data bits received (raw bits / 2), including a trailing incomplete byte.
  uint16_t data_bit_count() const { return this->total_bits_ / 2; }

  const PulseStats &pulse_stats() const { return this->stats_; }

  /// True once after a message was abandoned with at least `min_size` bytes;
  /// copies the snapshot to `out`. Recording is enabled with set_record_aborts().
  bool take_abort(AbortInfo &out);
  void set_record_aborts(bool record, uint8_t min_size = 2) {
    this->record_aborts_ = record;
    this->abort_min_size_ = min_size;
  }

 protected:
  enum State : uint8_t { UNKNOWN, T0, OK, DONE };

  // Returns -1 to reset, 1 when the message is complete, 0 to continue.
  int8_t decode_(uint32_t width_us);
  void got_bit_(uint8_t value);
  void manchester_(uint8_t value);
  void abort_(uint32_t width_us);

  Timing timing_{};
  uint8_t data_[MAX_BYTES]{};
  uint8_t raw_bits_[MAX_RAW_BITS / 8]{};
  PulseStats stats_{};
  uint16_t total_bits_{0};
  uint16_t pair_errors_{0};
  uint8_t pending_pair_errors_{0};  // pair errors in the byte being received
  uint8_t byte_pair_errors_[MAX_BYTES]{};
  uint8_t pos_{0};
  uint8_t flip_{0};
  uint8_t last_first_half_{0};
  State state_{UNKNOWN};

  bool record_aborts_{false};
  uint8_t abort_min_size_{2};
  bool abort_pending_{false};
  AbortInfo abort_info_{};
};

}  // namespace oregon_v21_receiver
}  // namespace esphome
