#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace franka_trajectory {
struct JointBounds {
  double lower;
  double upper;
  double velocity;
};

inline void validateJointTarget(const std::array<double, 7>& q,
                                const std::array<JointBounds, 7>& bounds,
                                double margin, double duration) {
  if (!std::isfinite(duration) || duration < 2.0)
    throw std::runtime_error("duration must be finite and at least 2 seconds");
  if (!std::isfinite(margin) || margin < 0.0)
    throw std::runtime_error("initialization_joint_margin must be finite and nonnegative");
  for (size_t i = 0; i < q.size(); ++i) {
    const auto& b = bounds[i];
    if (!std::isfinite(b.lower) || !std::isfinite(b.upper) ||
        !std::isfinite(b.velocity) || b.velocity <= 0.0 || b.lower + margin >= b.upper - margin)
      throw std::runtime_error("Invalid URDF bounds or margin for joint " + std::to_string(i+1));
    if (!std::isfinite(q[i]) || q[i] < b.lower + margin || q[i] > b.upper - margin)
      throw std::runtime_error("Target violates URDF position margin for joint " + std::to_string(i+1));
  }
}

inline double minimumJointDuration(const std::array<double, 7>& start,
                                   const std::array<double, 7>& target,
                                   const std::array<JointBounds, 7>& bounds,
                                   double velocity_scale, double velocity_cap) {
  if (!std::isfinite(velocity_scale) || velocity_scale <= 0.0 || velocity_scale > 1.0 ||
      !std::isfinite(velocity_cap) || velocity_cap <= 0.0)
    throw std::runtime_error("Invalid initialization velocity parameters");
  double duration = 2.0;
  for (size_t i = 0; i < start.size(); ++i) {
    if (!std::isfinite(start[i])) throw std::runtime_error("Nonfinite measured position");
    // Quintic zero-velocity/zero-acceleration endpoints: peak ds/dt = 1.875/T.
    const double speed = std::min(velocity_cap, velocity_scale * bounds[i].velocity);
    duration = std::max(duration, 1.875 * std::abs(target[i]-start[i]) / speed);
  }
  return duration;
}

// Quintic rest-to-rest profile: max |s''|=10/sqrt(3), max |s'''|=60.
inline double hardwareJointDuration(const std::array<double, 7>& start,
                                    const std::array<double, 7>& target,
                                    double acceleration, double jerk) {
  if (!std::isfinite(acceleration) || acceleration<=0 ||
      !std::isfinite(jerk) || jerk<=0)
    throw std::runtime_error("Invalid initialization acceleration/jerk caps");
  double duration=2.0;
  for (size_t i=0;i<7;++i) {
    const double distance=std::abs(target[i]-start[i]);
    if (!std::isfinite(distance)) throw std::runtime_error("Invalid joint displacement");
    duration=std::max(duration,std::sqrt((10.0/std::sqrt(3.0))*distance/acceleration));
    duration=std::max(duration,std::cbrt(60.0*distance/jerk));
  }
  return duration;
}

inline bool atRest(const std::array<double, 7>& dq, double tolerance) {
  for (double v : dq) if (!std::isfinite(v) || std::abs(v) > tolerance) return false;
  return true;
}

inline void addCartesianDisplacement(double& x, double& y, double& z,
                                     double dx, double dy, double dz) {
  const double next_x = x+dx, next_y = y+dy, next_z = z+dz;
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
      !std::isfinite(next_x) || !std::isfinite(next_y) || !std::isfinite(next_z))
    throw std::runtime_error("Nonfinite Cartesian displacement or resulting target");
  x=next_x; y=next_y; z=next_z;
}

// No rollback to the old Cartesian target after a failed initialization.
// Operations throw on failure. Only the last step may enable Cartesian commands.
template<class Operations>
void performJointInitialization(Operations& op) {
  op.preflight();
  op.invalidateReference();
  op.acquireJointController();
  op.moveAndSettle();
  op.prepareCartesian();
  op.resumeCartesian();
  op.synchronizeReference();
}
}  // namespace franka_trajectory
