#include <franka_example_controllers/cartesian_impedance_cbf_controller.h>
#include <pluginlib/class_list_macros.h>
namespace franka_example_controllers {
bool CartesianImpedanceCBFController::limitsTotalEnergy() const { return true; }
}  // namespace franka_example_controllers
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceCBFController,
                      controller_interface::ControllerBase)
