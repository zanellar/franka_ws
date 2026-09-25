// Total-energy variant of the shared B controller implementation.
#pragma once
#include <franka_example_controllers/cartesian_impedance_directional_kinetic_energy_cbf_controller.h>
namespace franka_example_controllers {
class CartesianImpedanceCBFController
    : public CartesianImpedanceDirectionalKineticEnergyCBFController {
 protected:
  bool limitsTotalEnergy() const override;
};
}  // namespace franka_example_controllers
