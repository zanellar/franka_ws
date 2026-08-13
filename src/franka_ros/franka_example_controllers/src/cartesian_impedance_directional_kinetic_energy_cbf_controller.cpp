// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_example_controllers/cartesian_impedance_directional_kinetic_energy_cbf_controller.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceDirectionalKineticEnergyCBFController::readParameters(
    ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("/alpha", alpha_)) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Could not read parameter alpha");
    return false;
  }
  if (!node_handle.getParam("/cbf_active", cbf_active_)) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Could not read parameter "
        "cbf_active");
    return false;
  }
  if (!node_handle.getParam("/damping_ratio", damping_ratio_)) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Could not read parameter "
        "damping_ratio");
    return false;
  }
  if (!node_handle.getParam("/Kmax", Kmax_)) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Could not read parameter Kmax");
    return false;
  }

  double direction_x = 1.0;
  double direction_y = 0.0;
  double direction_z = 0.0;
  node_handle.param("/direction_x", direction_x, 1.0);
  node_handle.param("/direction_y", direction_y, 0.0);
  node_handle.param("/direction_z", direction_z, 0.0);
  node_handle.param("/mobility_epsilon", mobility_epsilon_, 1.0e-8);
  node_handle.param("/derivative_filter_alpha", derivative_filter_alpha_, 0.05);

  direction_ << direction_x, direction_y, direction_z;
  const double direction_norm = direction_.norm();

  if (!std::isfinite(direction_norm) || direction_norm < 1.0e-9) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Direction must be finite "
        "and non-zero");
    return false;
  }
  direction_ /= direction_norm;

  if (!std::isfinite(Kmax_) || Kmax_ <= 0.0) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Kmax must be positive");
    return false;
  }
  if (!std::isfinite(alpha_) || alpha_ < 0.0) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: alpha must be non-negative");
    return false;
  }
  if (!std::isfinite(mobility_epsilon_) || mobility_epsilon_ <= 0.0) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: mobility_epsilon must be "
        "positive");
    return false;
  }
  if (!std::isfinite(derivative_filter_alpha_) || derivative_filter_alpha_ < 0.0 ||
      derivative_filter_alpha_ > 1.0) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: derivative_filter_alpha "
        "must be in [0, 1]");
    return false;
  }

  return true;
}

bool CartesianImpedanceDirectionalKineticEnergyCBFController::initializeQpSolver() {
  objective_matrix_.setIdentity();
  objective_matrix_.makeCompressed();

  // Fixed sparsity: one CBF row and seven identity rows for torque/rate bounds.
  std::vector<Eigen::Triplet<double, osqp::c_int>> constraint_triplets;
  constraint_triplets.reserve(14);
  for (osqp::c_int joint = 0; joint < 7; ++joint) {
    constraint_triplets.emplace_back(0, joint, 1.0);
    constraint_triplets.emplace_back(joint + 1, joint, 1.0);
  }
  constraint_matrix_.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());
  constraint_matrix_.makeCompressed();

  qp_instance_.objective_matrix = objective_matrix_;
  qp_instance_.objective_vector = Vector7d::Zero();
  qp_instance_.constraint_matrix = constraint_matrix_;

  lower_bounds_.head<1>() << -kInfinity_;
  lower_bounds_.tail<7>() = -tau_max_;
  upper_bounds_.head<1>() << kInfinity_;
  upper_bounds_.tail<7>() = tau_max_;
  qp_instance_.lower_bounds = lower_bounds_;
  qp_instance_.upper_bounds = upper_bounds_;

  qp_settings_.verbose = false;
  qp_settings_.max_iter = 8000;
  qp_settings_.time_limit = 0.9e-3;
  qp_settings_.rho = 0.1;
  qp_settings_.sigma = 1.0e-6;
  qp_settings_.alpha = 1.6;
  qp_settings_.warm_start = true;
  qp_settings_.polish = false;

  const absl::Status status = qp_solver_.Init(qp_instance_, qp_settings_);
  if (!status.ok()) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Failed to initialize OSQP");
    return false;
  }

  return true;
}

bool CartesianImpedanceDirectionalKineticEnergyCBFController::init(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& node_handle) {
  sub_equilibrium_pose_ = node_handle.subscribe(
      "/trajectory_publisher/equilibrium_pose", 20,
      &CartesianImpedanceDirectionalKineticEnergyCBFController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  cbf_publisher_ = node_handle.advertise<franka_msgs::Cbf>("/cbf_info", 1000);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Could not read parameter "
        "arm_id");
    return false;
  }

  if (!readParameters(node_handle)) {
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Invalid or missing "
        "joint_names parameter");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Error getting model "
        "interface");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Exception getting model "
        "handle: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Error getting state "
        "interface");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Exception getting state "
        "handle: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Error getting effort joint "
        "interface");
    return false;
  }

  joint_handles_.reserve(7);
  for (size_t joint = 0; joint < 7; ++joint) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[joint]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceDirectionalKineticEnergyCBFController: Exception getting joint "
          "handle: "
          << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(boost::bind(
      &CartesianImpedanceDirectionalKineticEnergyCBFController::complianceParamCallback, this, _1,
      _2));

  position_d_.setZero();
  orientation_d_.setIdentity();
  position_d_target_.setZero();
  orientation_d_target_.setIdentity();
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  if (!initializeQpSolver()) {
    return false;
  }

  ROS_INFO_STREAM(
      "CartesianImpedanceDirectionalKineticEnergyCBFController initialized with direction ["
      << direction_.transpose() << "], Kmax=" << Kmax_ << ", alpha=" << alpha_);

  return true;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::starting(
    const ros::Time& /*time*/) {
  const franka::RobotState initial_state = state_handle_->getRobotState();
  const std::array<double, 49> mass_array = model_handle_->getMass();
  const std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  const Vector7d q_initial = Eigen::Map<const Vector7d>(initial_state.q.data());
  const Matrix7d mass = Eigen::Map<const Matrix7d>(mass_array.data());
  const Matrix6x7d jacobian = Eigen::Map<const Matrix6x7d>(jacobian_array.data());
  const Eigen::Affine3d initial_transform(
      Eigen::Map<const Eigen::Matrix4d>(initial_state.O_T_EE.data()));

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = position_d_;
  orientation_d_target_ = orientation_d_;
  q_d_nullspace_ = q_initial;

  previous_directional_jacobian_ = direction_.transpose() * jacobian.topRows<3>();
  directional_jacobian_dot_filtered_.setZero();
  effective_mass_dot_filtered_ = 0.0;
  derivative_history_initialized_ = false;

  Eigen::LDLT<Matrix7d> mass_ldlt(mass);
  if (mass_ldlt.info() == Eigen::Success) {
    const Vector7d mass_inverse_jacobian_transpose =
        mass_ldlt.solve(previous_directional_jacobian_.transpose());
    const double mobility =
        (previous_directional_jacobian_ * mass_inverse_jacobian_transpose)(0, 0);
    if (std::isfinite(mobility) && mobility > 0.0) {
      previous_effective_mass_ = 1.0 / std::max(mobility, mobility_epsilon_);
      derivative_history_initialized_ = true;
    }
  }
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::update(
    const ros::Time& time,
    const ros::Duration& period) {
  const franka::RobotState robot_state = state_handle_->getRobotState();
  const std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  const std::array<double, 49> mass_array = model_handle_->getMass();
  const std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  const Vector7d coriolis = Eigen::Map<const Vector7d>(coriolis_array.data());
  const Matrix7d mass = Eigen::Map<const Matrix7d>(mass_array.data());
  const Matrix6x7d jacobian = Eigen::Map<const Matrix6x7d>(jacobian_array.data());
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const Vector7d tau_J_d = Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());
  const Vector7d tau_J = Eigen::Map<const Vector7d>(robot_state.tau_J.data());

  const double kinetic_energy = 0.5 * dq.transpose() * mass * dq;

  const Eigen::Affine3d transform(
      Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE.data()));
  const Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = position - position_d_;

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail<3>() << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail<3>() = -transform.rotation() * error.tail<3>();

  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  const Vector7d tau_task =
      jacobian.transpose() *
      (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  const Vector7d tau_nullspace =
      (Matrix7d::Identity() - jacobian.transpose() * jacobian_transpose_pinv) *
      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
       2.0 * std::sqrt(nullspace_stiffness_) * dq);
  const Vector7d tau_nominal = tau_task + tau_nullspace + coriolis;

  const CbfResult cbf_result = directionalKineticEnergyCbf(
      tau_nominal, coriolis, mass, jacobian, dq, tau_J_d, period.toSec());

  Vector7d tau_command = cbf_active_ ? cbf_result.tau_safe
                                     : saturateTorqueRate(tau_nominal, tau_J_d);

  for (size_t joint = 0; joint < 7; ++joint) {
    joint_handles_[joint].setCommand(tau_command(joint));
  }

  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ +
                         (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ +
                       (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ +
                         (1.0 - filter_params_) * nullspace_stiffness_;

  {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_ =
        filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  Eigen::Map<Vector7d> u_des_map(&cbf_info_.u_des[0]);
  Eigen::Map<Vector7d> u_cbf_map(&cbf_info_.u_cbf[0]);
  Eigen::Map<Vector7d> u_measured_map(&cbf_info_.u_measured[0]);
  Eigen::Map<Vector7d> u_saturated_map(&cbf_info_.u_saturated[0]);
  Eigen::Map<Vector7d> u_ext_map(&cbf_info_.u_ext[0]);
  u_des_map = tau_nominal;
  u_cbf_map = cbf_result.tau_safe;
  u_measured_map = tau_J;
  u_saturated_map = tau_command;
  u_ext_map.setZero();

  cbf_info_.h = cbf_result.h;
  cbf_info_.kinetic_energy = kinetic_energy;
  cbf_info_.directional_kinetic_energy = cbf_result.directional_kinetic_energy;
  cbf_info_.Kmax = Kmax_;
  cbf_info_.solver_status = cbf_active_ ? cbf_result.solver_status : 0;
  cbf_info_.header.stamp = time;
  cbf_publisher_.publish(cbf_info_);
}

CartesianImpedanceDirectionalKineticEnergyCBFController::CbfResult
CartesianImpedanceDirectionalKineticEnergyCBFController::directionalKineticEnergyCbf(
    const Vector7d& tau_nominal,
    const Vector7d& coriolis,
    const Matrix7d& mass,
    const Matrix6x7d& jacobian,
    const Vector7d& dq,
    const Vector7d& tau_J_d,
    double dt) {
  CbfResult result;
  result.tau_safe = saturateTorqueRate(tau_nominal, tau_J_d);

  if (!std::isfinite(dt) || dt <= 1.0e-6) {
    dt = 1.0e-3;
  }

  const Matrix3x7d translational_jacobian = jacobian.topRows<3>();
  const RowVector7d directional_jacobian =
      direction_.transpose() * translational_jacobian;
  const double directional_velocity = (directional_jacobian * dq)(0, 0);

  Eigen::LDLT<Matrix7d> mass_ldlt(mass);
  if (mass_ldlt.info() != Eigen::Success) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Mass matrix factorization "
        "failed");
    result.solver_status = 3;
    return result;
  }

  const Vector7d mass_inverse_jacobian_transpose =
      mass_ldlt.solve(directional_jacobian.transpose());
  if (!mass_inverse_jacobian_transpose.allFinite()) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Invalid inverse-dynamics "
        "solution");
    result.solver_status = 3;
    return result;
  }

  const double mobility =
      (directional_jacobian * mass_inverse_jacobian_transpose)(0, 0);
  if (!std::isfinite(mobility) || mobility <= 0.0) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Directional mobility is "
        "non-positive");
    result.solver_status = 3;
    return result;
  }

  const double effective_mass = 1.0 / std::max(mobility, mobility_epsilon_);
  const double directional_energy =
      0.5 * effective_mass * directional_velocity * directional_velocity;
  const double h = Kmax_ - directional_energy;
  result.h = h;
  result.directional_kinetic_energy = directional_energy;

  RowVector7d directional_jacobian_dot = RowVector7d::Zero();
  double effective_mass_dot = 0.0;
  if (derivative_history_initialized_) {
    const RowVector7d raw_jacobian_dot =
        (directional_jacobian - previous_directional_jacobian_) / dt;
    const double raw_effective_mass_dot =
        (effective_mass - previous_effective_mass_) / dt;

    directional_jacobian_dot_filtered_ =
        derivative_filter_alpha_ * raw_jacobian_dot +
        (1.0 - derivative_filter_alpha_) * directional_jacobian_dot_filtered_;
    effective_mass_dot_filtered_ =
        derivative_filter_alpha_ * raw_effective_mass_dot +
        (1.0 - derivative_filter_alpha_) * effective_mass_dot_filtered_;

    directional_jacobian_dot = directional_jacobian_dot_filtered_;
    effective_mass_dot = effective_mass_dot_filtered_;
  }

  previous_directional_jacobian_ = directional_jacobian;
  previous_effective_mass_ = effective_mass;
  derivative_history_initialized_ = true;

  // With gravity compensated internally by Franka and Coriolis added to tau_nominal:
  //   M*qdd = tau - coriolis
  //   h_dot = a*tau + b
  const RowVector7d a =
      -effective_mass * directional_velocity * mass_inverse_jacobian_transpose.transpose();
  const double b =
      -effective_mass * directional_velocity * (directional_jacobian_dot * dq)(0, 0) -
      0.5 * effective_mass_dot * directional_velocity * directional_velocity -
      (a * coriolis)(0, 0);
  const double cbf_lower_bound = -alpha_ * h - b;

  for (int joint = 0; joint < 7; ++joint) {
    constraint_matrix_.coeffRef(0, joint) = a(joint);
  }
  constraint_matrix_.makeCompressed();

  lower_bounds_(0) = cbf_lower_bound;
  upper_bounds_(0) = kInfinity_;
  for (int joint = 0; joint < 7; ++joint) {
    lower_bounds_(joint + 1) =
        std::max(-tau_max_(joint), tau_J_d(joint) - delta_tau_max_);
    upper_bounds_(joint + 1) =
        std::min(tau_max_(joint), tau_J_d(joint) + delta_tau_max_);
  }

  const Vector7d objective_vector = -tau_nominal;
  const absl::Status matrix_status = qp_solver_.UpdateConstraintMatrix(constraint_matrix_);
  const absl::Status objective_status = qp_solver_.SetObjectiveVector(objective_vector);
  const absl::Status bounds_status = qp_solver_.SetBounds(lower_bounds_, upper_bounds_);

  if (!matrix_status.ok() || !objective_status.ok() || !bounds_status.ok()) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Failed to update OSQP data");
    result.solver_status = 4;
    return result;
  }

  const osqp::OsqpExitCode exit_code = qp_solver_.Solve();
  if (exit_code != osqp::OsqpExitCode::kOptimal &&
      exit_code != osqp::OsqpExitCode::kOptimalInaccurate) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: OSQP failed with status %s",
        osqp::ToString(exit_code).c_str());
    result.solver_status = 5;
    return result;
  }

  const Eigen::VectorXd solution = qp_solver_.primal_solution();
  if (solution.size() != 7 || !solution.allFinite()) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Invalid OSQP solution");
    result.solver_status = 5;
    return result;
  }

  result.tau_safe = solution;
  result.solver_status =
      exit_code == osqp::OsqpExitCode::kOptimal ? static_cast<uint8_t>(1)
                                                 : static_cast<uint8_t>(2);

  const double safe_constraint =
      (a * result.tau_safe)(0, 0) + b + alpha_ * h;
  if (safe_constraint < -1.0e-5) {
    ROS_WARN_THROTTLE(
        1.0,
        "CartesianImpedanceDirectionalKineticEnergyCBFController: CBF constraint violation: "
        "%f",
        safe_constraint);
  }

  return result;
}

CartesianImpedanceDirectionalKineticEnergyCBFController::Vector7d
CartesianImpedanceDirectionalKineticEnergyCBFController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_J_d) const {
  Vector7d tau_d_saturated;
  for (size_t joint = 0; joint < 7; ++joint) {
    const double difference = tau_d_calculated(joint) - tau_J_d(joint);
    tau_d_saturated(joint) =
        tau_J_d(joint) + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner<3, 3>() =
      config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner<3, 3>() =
      config.rotational_stiffness * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner<3, 3>() =
      damping_ratio_ * 2.0 * std::sqrt(config.translational_stiffness) *
      Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner<3, 3>() =
      damping_ratio_ * 2.0 * std::sqrt(config.rotational_stiffness) *
      Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  cbf_active_ = config.cbf_active;
  Kmax_ = config.Kmax;
  alpha_ = config.alpha;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  const Eigen::Quaterniond previous_orientation_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

  if (orientation_d_target_.norm() < 1.0e-12) {
    orientation_d_target_ = previous_orientation_target;
    return;
  }
  orientation_d_target_.normalize();

  if (previous_orientation_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() *= -1.0;
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::CartesianImpedanceDirectionalKineticEnergyCBFController,
    controller_interface::ControllerBase)
