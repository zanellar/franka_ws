// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

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
#include <realtime_tools/realtime_publisher.h>

#include <franka_msgs/Cbf.h>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class CartesianImpedanceDZCBFController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  Eigen::Matrix<double, 7, 1> saturateQdotRate(
    const Eigen::Matrix<double, 7, 1>& qdot_measured,
    const Eigen::Matrix<double, 7, 1>& qdot_prev);
      
  //Eigen::Matrix<double, 7, 1> cbfCompute(
  //    const Eigen::Matrix<double, 7, 1>& tau_d_calculated);  // NOLINT (readability-identifier-naming)

  std::tuple<Eigen::Matrix<double, 7, 1>, double> cbfCompute(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Map<Eigen::Matrix<double, 7, 7>>& D,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& G,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq);

  Eigen::Matrix<double, 7, 1> EMA(
    Eigen::Matrix<double, 7, 1> dq,
    Eigen::Matrix<double, 7, 1> dq_prev,
    double alpha);

  std::tuple<Eigen::Matrix<double, 7, 1>, double> cbfAnalytical(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Map<Eigen::Matrix<double, 7, 7>>& D,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& G,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq);

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  // joint acceleration limits plus some margin (20%): https://frankaemika.github.io/docs/control_parameters.html
  const double delta_qdot_max_[7] = {18.0/1000, 9.0/1000, 12.0/1000, 15.0/1000, 18.0/1000, 24.0/1000, 24.0/1000};
  const Eigen::Matrix<double, 7, 1> tau_max_ = (Eigen::Matrix<double, 7, 1>() << 87, 87, 87, 87, 12, 12, 12).finished();
  // const double delta_qdot_max_[7] = {30.0/1000, 30.0/1000, 30.0/1000, 30.0/1000, 30.0/1000, 30.0/1000, 30.0/1000};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  
  // Send controller data
  // ros::init("cbf_val");
  ros::NodeHandle cbf_info_node_;
  franka_msgs::Cbf cbf_info;
  ros::Publisher cbf_publisher = cbf_info_node_.advertise<franka_msgs::Cbf>("cbf_info", 10000);
  
  // EMA setup (basically a discrete first-order low-pass filter)
  // time constant (inverse cutoff) of EMA filter is -dt / ln(1-alpha)
  // cutoff frequency at alpha=0.01 is approx 10 Hz. Delay of approx 100 samples or 0.1s below cutoff frequency.
  double EMAlpha = 0.05; // range (0,1]

  Eigen::Matrix<double, 7, 1> dq_saturated;
  Eigen::Matrix<double, 7, 1> dq_filtered;

  // cbf solver setup (fetched as ROS parameters)
  double Kmax;
  double alpha;
  double damping_ratio;
  bool cbf_active;
  double deadzone;
  double power;
  
  ros::Time starttime;

  const double kInfinity = std::numeric_limits<double>::infinity();

  // Eigen::SparseMatrix<double> hp(1, 1);
  // Eigen::SparseMatrix<double> objective_matrix(7, 7); // P = I

  Eigen::Matrix<double, 1, 1> h;
  Eigen::SparseMatrix<double> hp{1, 1};
  Eigen::SparseMatrix<double> objective_matrix{7,7}; // P = I
  Eigen::Matrix<double, 1+7, 7> constraint_matrix;
  Eigen::Matrix<double, 1+7, 1> lower_bounds;
  Eigen::Matrix<double, 1+7, 1> upper_bounds;

  Eigen::VectorXd optimal_solution;

  osqp::OsqpInstance QPinstance;
  osqp::OsqpSolver QPsolver;
  osqp::OsqpSettings QPsettings; // TO DO: SOLVER SHOULD ONLY BE SET UP ONCE
  // Edit settings if appropriate.

};

}  // namespace franka_example_controllers
