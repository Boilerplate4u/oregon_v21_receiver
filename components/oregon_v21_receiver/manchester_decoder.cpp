#include "manchester_decoder.h"

#include <cstring>

namespace esphome {
namespace oregon_v21_receiver {

void ManchesterDecoder::reset() {
  // Only the parts written since the last reset need clearing; reset() runs for
  // every noise pulse, so this keeps it cheap.
  const size_t raw_bytes = (size_t(this->total_bits_) + 7) / 8;
  std::memset(this->raw_bits_, 0, raw_bytes < sizeof(this->raw_bits_) ? raw_bytes : sizeof(this->raw_bits_));
  const size_t bytes = size_t(this->pos_) + 1;
  std::memset(this->data_, 0, bytes < sizeof(this->data_) ? bytes : sizeof(this->data_));
  std::memset(this->byte_pair_errors_, 0,
              bytes < sizeof(this->byte_pair_errors_) ? bytes : sizeof(this->byte_pair_errors_));
  this->stats_ = PulseStats{};
  this->total_bits_ = 0;
  this->pair_errors_ = 0;
  this->pending_pair_errors_ = 0;
  this->pos_ = 0;
  this->flip_ = 0;
  this->last_first_half_ = 0;
  this->state_ = UNKNOWN;
}

bool ManchesterDecoder::feed(uint32_t width_us) {
  if (this->state_ != DONE) {
    switch (this->decode_(width_us)) {
      case -1:
        this->abort_(width_us);
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
  if (this->state_ != UNKNOWN)
    this->abort_(0);
  return false;
}

uint16_t ManchesterDecoder::pair_errors(size_t bytes) const {
  uint16_t sum = 0;
  for (size_t i = 0; i < bytes && i < this->pos_; i++)
    sum += this->byte_pair_errors_[i];
  return sum;
}

bool ManchesterDecoder::take_abort(AbortInfo &out) {
  if (!this->abort_pending_)
    return false;
  out = this->abort_info_;
  this->abort_pending_ = false;
  return true;
}

void ManchesterDecoder::abort_(uint32_t width_us) {
  if (!this->record_aborts_ || this->state_ == UNKNOWN || this->pos_ < this->abort_min_size_)
    return;
  std::memcpy(this->abort_info_.data, this->data_, this->pos_);
  this->abort_info_.size = this->pos_;
  this->abort_info_.raw_bits = this->total_bits_;
  this->abort_info_.pulse_width_us = width_us;
  this->abort_pending_ = true;
}

void ManchesterDecoder::got_bit_(uint8_t value) {
  const uint16_t index = this->total_bits_;
  if (index >= MAX_RAW_BITS) {
    this->reset();
    return;
  }
  if (value)
    this->raw_bits_[index / 8] |= uint8_t(1u << (index % 8));

  if ((index & 0x01) == 0) {
    // First half of a pair: this is the data bit.
    this->data_[this->pos_] = (this->data_[this->pos_] >> 1) | (value ? 0x80 : 0x00);
    this->last_first_half_ = value;
  } else if (value == this->last_first_half_) {
    // Second half must be the complement of the first.
    this->pending_pair_errors_++;
  }

  this->total_bits_ = index + 1;
  const uint8_t new_pos = this->total_bits_ >> 4;
  if (new_pos != this->pos_) {
    // A byte (16 raw bits) is complete.
    this->byte_pair_errors_[this->pos_] = this->pending_pair_errors_;
    this->pair_errors_ += this->pending_pair_errors_;
    this->pending_pair_errors_ = 0;
  }
  this->pos_ = new_pos;
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
    if (this->state_ != UNKNOWN) {
      if (is_long) {
        this->stats_.long_sum_us += width_us;
        this->stats_.long_count++;
      } else {
        this->stats_.short_sum_us += width_us;
        this->stats_.short_count++;
      }
    }
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
