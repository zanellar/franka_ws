// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp++.h>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_msgs/Cbf.h>

namespace franka_example_controllers {

class CartesianImpedanceDirectionalKineticEnergyCBFController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector8d = Eigen::Matrix<double, 8, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
  using Matrix3x7d = Eigen::Matrix<double, 3, 7>;
  using RowVector7d = Eigen::Matrix<double, 1, 7>;
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, osqp::c_int>;

  struct CbfResult {
    // Bias-free control returned by the CBF. The Coriolis bias is added only
    // afterwards in update(): tau_command = u_safe + u_bias.
    Vector7d u_safe{Vector7d::Zero()};
    double h{0.0};
    double directional_kinetic_energy{0.0};
    uint8_t solver_status{0};
  };

  bool readParameters(ros::NodeHandle& node_handle);
  bool initializeQpSolver();

  CbfResult directionalKineticEnergyCbf(
      const Vector7d& u_nominal,
      const Vector7d& u_bias,
      const Matrix7d& mass,
      const Matrix6x7d& jacobian,
      const Vector7d& dq,
      const Vector7d& tau_J_d,
      double dt);

  Vector7d saturateTorqueRate(const Vector7d& tau_d_calculated,
                              const Vector7d& tau_J_d) const;

  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1000.0};

  const Vector7d tau_max_{
        (Vector7d() << 1000.0, 1000.0, 1000.0, 1000.0,
                    1000.0, 1000.0, 1000.0).finished()};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_{
      Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_{
      Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_damping_{
      Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_{
      Eigen::Matrix<double, 6, 6>::Zero()};

  Vector7d q_d_nullspace_{Vector7d::Zero()};
  Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_target_{Eigen::Quaterniond::Identity()};

  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

  ros::Subscriber sub_equilibrium_pose_;
  ros::Publisher cbf_publisher_;
  franka_msgs::Cbf cbf_info_;

  double Kmax_{1.5};
  double alpha_{1.0};
  double damping_ratio_{1.0};
  double mobility_epsilon_{1.0e-8};
  bool cbf_active_{true};
  Eigen::Vector3d direction_{Eigen::Vector3d::UnitX()};

  // Persistent history used only for numerical derivatives, matching the
  // reference pseudocode:
  //   J_dot[k] ~= (J[k] - J[k-1]) / dt
  //   M_dot[k] ~= (M[k] - M[k-2]) / (2 dt)
  Matrix6x7d previous_jacobian_{Matrix6x7d::Zero()};
  Matrix7d previous_mass_{Matrix7d::Zero()};
  Matrix7d previous_mass_2_{Matrix7d::Zero()};
  int derivative_sample_count_{0};

  SparseMatrix objective_matrix_{7, 7};
  SparseMatrix constraint_matrix_{8, 7};
  Vector8d lower_bounds_{Vector8d::Zero()};
  Vector8d upper_bounds_{Vector8d::Zero()};
  osqp::OsqpInstance qp_instance_;
  osqp::OsqpSolver qp_solver_;
  osqp::OsqpSettings qp_settings_;

  const double kInfinity_{std::numeric_limits<double>::infinity()};
};

}  // namespace franka_example_controllers
