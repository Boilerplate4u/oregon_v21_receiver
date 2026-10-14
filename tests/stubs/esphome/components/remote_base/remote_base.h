#pragma once
// Minimal host stub of the remote_base receiver interface used by the component.
// The real API is verified by the ESPHome compile in CI.
#include <cstdint>
#include <vector>
namespace esphome {
namespace remote_base {
using RawTimings = std::vector<int32_t>;
class RemoteReceiveData {
 public:
  explicit RemoteReceiveData(const RawTimings &data) : data_(data) {}
  const RawTimings &get_raw_data() const { return this->data_; }

 protected:
  const RawTimings &data_;
};
class RemoteReceiverListener {
 public:
  virtual ~RemoteReceiverListener() = default;
  virtual bool on_receive(RemoteReceiveData data) = 0;
};
}  // namespace remote_base
}  // namespace esphome
