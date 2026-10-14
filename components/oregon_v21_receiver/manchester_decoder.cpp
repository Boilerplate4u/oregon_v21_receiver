#include "manchester_decoder.h"

#include <cstring>

namespace esphome {
namespace oregon_v21_receiver {

void ManchesterDecoder::reset() {
  std::memset(this->data_, 0, sizeof(this->data_));
  this->total_bits_ = 0;
  this->pos_ = 0;
  this->flip_ = 0;
  this->state_ = UNKNOWN;
}

bool ManchesterDecoder::feed(uint32_t width_us) {
  if (this->state_ != DONE) {
    switch (this->decode_(width_us)) {
      case -1:
        this->reset();
        break;
      case 1:
        this->state_ = DONE;
        break;
      default:
        break;
    }
  }
  return this->state_ == DONE;
}

bool ManchesterDecoder::finish() {
  if (this->state_ == DONE)
    return true;
  if (this->state_ != UNKNOWN && this->pos_ >= this->timing_.min_bytes) {
    this->state_ = DONE;
    return true;
  }
  return false;
}

void ManchesterDecoder::got_bit_(uint8_t value) {
  // Oregon v2.1 sends each bit followed by its inverse: keep only the even ones.
  if ((this->total_bits_ & 0x01) == 0) {
    this->data_[this->pos_] = (this->data_[this->pos_] >> 1) | (value ? 0x80 : 0x00);
  }
  this->total_bits_++;
  this->pos_ = this->total_bits_ >> 4;
  if (this->pos_ >= MAX_BYTES) {
    this->reset();
    return;
  }
  this->state_ = OK;
}

void ManchesterDecoder::manchester_(uint8_t value) {
  // A long pulse flips the current bit value, two short pulses keep it.
  this->flip_ ^= value;
  this->got_bit_(this->flip_);
}

int8_t ManchesterDecoder::decode_(uint32_t width_us) {
  const Timing &t = this->timing_;
  if (width_us >= t.min_pulse_us && width_us < t.max_pulse_us) {
    const bool is_long = width_us >= t.long_pulse_us;
    switch (this->state_) {
      case UNKNOWN:
        if (is_long) {
          // Preamble: count long pulses (flip_ is reused as the counter here).
          if (this->flip_ < 0xFF)
            this->flip_++;
        } else if (this->flip_ >= t.min_preamble) {
          // First short pulse after a long enough preamble: start of data.
          this->flip_ = 0;
          this->state_ = T0;
        } else {
          return -1;
        }
        break;
      case OK:
        if (is_long) {
          this->manchester_(1);
        } else {
          this->state_ = T0;
        }
        break;
      case T0:
        if (is_long)
          return -1;
        this->manchester_(0);
        break;
      default:
        break;
    }
    return 0;
  }
  if (width_us >= t.end_gap_us && this->pos_ >= t.min_bytes)
    return 1;
  return -1;
}

}  // namespace oregon_v21_receiver
}  // namespace esphome
