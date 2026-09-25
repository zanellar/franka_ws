#pragma once
#include <array>
#include <cstddef>

namespace franka_example_controllers {
// Coordinates: tau_command = u + coriolis; gravity is compensated by the plant.
// Rigid-body identity: 0.5*dq^T*Mdot*dq = dq^T*coriolis.
// Thus hdot = a*u + b for h=Kmax-0.5*dq^T*M*dq.
struct TotalEnergyBarrier {
  std::array<double, 7> a{};
  double b{0.0};
};
inline TotalEnergyBarrier totalEnergyBarrier(const std::array<double, 7>& dq,
                                           const std::array<double, 7>& coriolis) {
  TotalEnergyBarrier result;
  for (std::size_t i=0; i<7; ++i) {
    result.a[i] = -dq[i];
    result.b -= dq[i]*coriolis[i];
  }
  return result;
}
}  // namespace franka_example_controllers
