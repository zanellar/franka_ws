#pragma once

#include <cmath>
#include <cstdint>

namespace franka_example_controllers {

// Called under the controller mutex. Only an explicit experiment request
// clears an abort; parameter edits and recurring pose messages do not.
class DirectionalCbfTaskState {
 public:
  void requestStart() { start_requested_ = true; }
  bool consumeStartRequest() {
    if (!start_requested_) return false;
    start_requested_ = false;
    error_ = 0;
    return true;
  }
  void fail(uint8_t error) { if (error_ == 0) error_ = error; }
  bool aborted() const { return error_ != 0; }
  uint8_t error() const { return error_; }

  static bool acceptsSolution(bool optimal, double residual, double tolerance) {
    return optimal && std::isfinite(residual) && std::isfinite(tolerance) &&
           tolerance >= 0.0 && residual >= -tolerance;
  }

 private:
  bool start_requested_{false};
  uint8_t error_{0};
};
}  // namespace franka_example_controllers
