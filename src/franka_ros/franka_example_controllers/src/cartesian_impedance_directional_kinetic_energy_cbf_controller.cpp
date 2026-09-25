// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_example_controllers/cartesian_impedance_directional_kinetic_energy_cbf_controller.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceDirectionalKineticEnergyCBFController::readParameters(
    ros::NodeHandle& node_handle) {
  std::string environment;
  node_handle.param<std::string>("runtime_environment", environment, "gazebo");
  if (environment != "gazebo" && environment != "real") {
    ROS_ERROR("runtime_environment must be gazebo or real"); return false;
  }
  real_robot_ = environment == "real";
  std::string energy_mode;
  node_handle.param<std::string>("energy_mode", energy_mode, "directional");
  if (energy_mode == "directional") energy_mode_ = EnergyMode::kDirectional;
  else if (energy_mode == "total") energy_mode_ = EnergyMode::kTotal;
  else if (energy_mode == "none") energy_mode_ = EnergyMode::kNone;
  else {
    ROS_ERROR("energy_mode must be directional, total, or none");
    return false;
  }
  if (real_robot_) {
    if (std::string(DIRECTIONAL_CBF_LIBFRANKA_VERSION)!="0.13.3") {
      ROS_ERROR("Real CBF's actuator prediction is validated against libfranka 0.13.3 source only");
      return false;
    }
    bool sim_time=false, rate_enabled=false;
    node_handle.getParam("/use_sim_time", sim_time);
    std::string realtime;
    if (sim_time || !node_handle.getParam("/franka_control/rate_limiting", rate_enabled) ||
        !rate_enabled || !node_handle.getParam("/franka_control/cutoff_frequency", hardware_cutoff_) ||
        !node_handle.getParam("/franka_control/realtime_config", realtime) || realtime != "enforce") {
      ROS_ERROR("Real mode requires wall time and /franka_control with rate limiter and realtime enforce");
      return false;
    }
    node_handle.param("hardware_torque_rate", hardware_rate_, 999.0);
    node_handle.param("derivative_filter_alpha", derivative_filter_alpha_, 0.05);
    node_handle.param("max_update_seconds", max_update_seconds_, 0.0009);
    node_handle.param("max_period_seconds", max_period_seconds_, 0.002);
    if (!std::isfinite(hardware_cutoff_) || hardware_cutoff_<=0 ||
        !std::isfinite(hardware_rate_) || hardware_rate_<=0 || hardware_rate_>999.999 ||
        !std::isfinite(derivative_filter_alpha_) || derivative_filter_alpha_<=0 || derivative_filter_alpha_>1 ||
        !std::isfinite(max_update_seconds_) || max_update_seconds_<=0 ||
        !std::isfinite(max_period_seconds_) || max_period_seconds_<0.001) return false;
  }
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
  if (energy_mode_ == EnergyMode::kNone && cbf_active_) {
    ROS_ERROR("energy_mode=none requires cbf_active=false");
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

  node_handle.param("abort_damping", abort_damping_, 20.0);
  node_handle.param("cbf_residual_tolerance", cbf_residual_tolerance_, 1.0e-5);
  node_handle.param("torque_limit_tolerance", torque_limit_tolerance_, 1.0e-5);
  const auto& min_config = compliance_paramConfig::__getMin__();
  const auto& max_config = compliance_paramConfig::__getMax__();
  if (!std::isfinite(abort_damping_) || abort_damping_ <= 0.0 ||
      !std::isfinite(cbf_residual_tolerance_) || cbf_residual_tolerance_ < 0.0 ||
      !std::isfinite(torque_limit_tolerance_) || torque_limit_tolerance_ < 0.0 ||
      Kmax_ < min_config.Kmax || Kmax_ > max_config.Kmax ||
      alpha_ < min_config.alpha || alpha_ > max_config.alpha) {
    ROS_ERROR("Invalid abort damping, residual/torque tolerance or CBF configuration range");
    return false;
  }

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

  return true;
}

bool CartesianImpedanceDirectionalKineticEnergyCBFController::readTorqueLimits(
    ros::NodeHandle& node_handle, const std::vector<std::string>& joint_names) {
  std::string description_param;
  std::string description;
  urdf::Model model;
  if (!node_handle.searchParam("robot_description", description_param) ||
      !node_handle.getParam(description_param, description) || !model.initString(description)) {
    ROS_ERROR("Directional CBF: cannot load robot_description for effort limits");
    return false;
  }
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto joint = model.getJoint(joint_names[i]);
    if (!joint || !joint->limits || !std::isfinite(joint->limits->effort) ||
        joint->limits->effort <= 0.0) {
      ROS_ERROR_STREAM("Directional CBF: missing or invalid effort limit for " << joint_names[i]);
      return false;
    }
    torque_limits_(i) = joint->limits->effort;
    if (real_robot_) {
      if (!joint->safety || joint->type != urdf::Joint::REVOLUTE) {
        ROS_ERROR("Real CBF requires URDF revolute joints with safety_controller limits"); return false;
      }
      hardware_limits_[i] = {joint->safety->soft_lower_limit, joint->safety->soft_upper_limit,
          joint->limits->velocity, joint->limits->effort,
          joint->safety->k_position, joint->safety->k_velocity};
      const auto& l=hardware_limits_[i];
      if (!std::isfinite(l.lower) || !std::isfinite(l.upper) || l.lower>=l.upper ||
          !std::isfinite(l.velocity) || l.velocity<=0 || !std::isfinite(l.k_position) ||
          l.k_position<0 || !std::isfinite(l.k_velocity) || l.k_velocity<0) return false;
    }
  }
  ROS_INFO_STREAM("Directional CBF: URDF total-effort limits [Nm]: "
                  << torque_limits_.transpose());
  return true;
}

CartesianImpedanceDirectionalKineticEnergyCBFController::Vector7d
CartesianImpedanceDirectionalKineticEnergyCBFController::clampTorqueCommand(
    const Vector7d& command, const Vector7d& gravity) const {
  if (real_robot_) {
    Vector7d bounded = Vector7d::Zero();
    for (int i=0;i<7;++i) {
      const double value=std::isfinite(command(i)) ? command(i) : 0.0;
      // If the rate/soft intersection is empty, hardware protections take
      // precedence; the fallback is explicitly NOT a CBF solution.
      const double lo=hardware_envelope_.valid ? hardware_envelope_.lower[i] : hardware_envelope_.soft_lower[i];
      const double hi=hardware_envelope_.valid ? hardware_envelope_.upper[i] : hardware_envelope_.soft_upper[i];
      bounded(i)=hardware_cbf::clip(value,lo,hi);
    }
    return bounded;
  }
  // FrankaHWSim adds gravity AFTER receiving this command. These bounds apply
  // to command + gravity, not to the gravity-free command alone.
  // An invalid gravity model prevents predicting total effort. Send no active
  // torque in that case and leave the independent simulator limits enabled.
  if (!gravity.allFinite()) {
    return Vector7d::Zero();
  }
  Vector7d bounded;
  for (int i = 0; i < 7; ++i) {
    const double requested = std::isfinite(command(i)) ? command(i) : 0.0;
    bounded(i) = std::max(-torque_limits_(i) - gravity(i),
                          std::min(torque_limits_(i) - gravity(i), requested));
  }
  return bounded;
}

bool CartesianImpedanceDirectionalKineticEnergyCBFController::initializeQpSolver() {
  // OSQP uses 0.5*u^T*P*u + q^T*u. P=2I and q=-2*u_nominal
  // therefore reproduce ||u-u_nominal||^2 up to an additive constant.
  objective_matrix_.setIdentity();
  objective_matrix_ *= 2.0;
  objective_matrix_.makeCompressed();

  // Fixed sparsity: one CBF half-space followed by seven bilateral torque bounds.
  std::vector<Eigen::Triplet<double, osqp::c_int>> constraint_triplets;
  constraint_triplets.reserve(14);
  for (osqp::c_int joint = 0; joint < 7; ++joint) {
    constraint_triplets.emplace_back(0, joint, 1.0);
    constraint_triplets.emplace_back(1 + joint, joint, 1.0);
  }
  constraint_matrix_.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());
  constraint_matrix_.makeCompressed();

  qp_instance_.objective_matrix = objective_matrix_;
  qp_instance_.objective_vector = Vector7d::Zero();
  qp_instance_.constraint_matrix = constraint_matrix_;

  lower_bounds_(0) = -kInfinity_;
  upper_bounds_(0) = kInfinity_;
  // Placeholder zero-bias box. All bounds are updated before every Solve().
  lower_bounds_.tail<7>() = -torque_limits_;
  upper_bounds_.tail<7>() = torque_limits_;
  qp_instance_.lower_bounds = lower_bounds_;
  qp_instance_.upper_bounds = upper_bounds_;

  qp_settings_.verbose = false;
  qp_settings_.max_iter = 8000;
  // Keep OSQP's default: no wall-clock timeout. max_iter remains a finite gate.
  qp_settings_.eps_abs = 1.0e-7;
  qp_settings_.eps_rel = 1.0e-7;
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

  base_frame_ = arm_id + "_link0";
  diagnostics_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<
      franka_msgs::DirectionalCbfDiagnostics>>(node_handle, "/directional_cbf/diagnostics", 1000);

  if (!readParameters(node_handle)) {
    return false;
  }
  if (real_robot_) {
    realtime_cbf_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<franka_msgs::Cbf>>(
        node_handle, "/cbf_info", 100);
    diagnostics_publisher_->msg_.header.frame_id = base_frame_;
    hardware_buffer_.initRT(hardware_request_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceDirectionalKineticEnergyCBFController: Invalid or missing "
        "joint_names parameter");
    return false;
  }

  if (!readTorqueLimits(node_handle, joint_names)) {
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
  // Seed the server namespace before constructing it: setCallback immediately
  // invokes the callback with this initial configuration.
  dynamic_reconfigure_compliance_param_node_.setParam("Kmax", Kmax_);
  dynamic_reconfigure_compliance_param_node_.setParam("alpha", alpha_);
  dynamic_reconfigure_compliance_param_node_.setParam("cbf_active", cbf_active_);
  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
      dynamic_config_mutex_, dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(boost::bind(
      &CartesianImpedanceDirectionalKineticEnergyCBFController::complianceParamCallback, this, _1,
      _2));

  position_d_.setZero();
  orientation_d_.setIdentity();
  position_d_target_.setZero();
  orientation_d_target_.setIdentity();
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  if (!real_robot_ && !initializeQpSolver()) {
    task_state_.fail(4);
    ROS_ERROR("Directional task aborted during QP setup; experiment service remains available");
  }

  start_experiment_service_ = node_handle.advertiseService(
      "start_experiment",
      &CartesianImpedanceDirectionalKineticEnergyCBFController::startExperimentCallback, this);

  ROS_INFO_STREAM(
      "CartesianImpedanceDirectionalKineticEnergyCBFController initialized with direction ["
      << direction_.transpose() << "], Kmax=" << Kmax_ << ", alpha=" << alpha_);

  return true;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::starting(
    const ros::Time& /*time*/) {
  std::unique_lock<std::recursive_mutex> lock(control_mutex_, std::defer_lock);
  if (!real_robot_) lock.lock();
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

  // Reset the persistent derivative history. The first J_dot sample and the
  // first two M_dot samples are intentionally zero, matching the pseudocode.
  previous_jacobian_ = jacobian;
  previous_mass_ = mass;
  previous_mass_2_ = mass;
  derivative_sample_count_ = 0; previous_dt_=0.001;
  filtered_jacobian_dot_.setZero(); filtered_mass_dot_.setZero();
  have_prediction_ = false;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::update(
    const ros::Time& time,
    const ros::Duration& period) {
  const auto update_begin = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(control_mutex_, std::defer_lock);
  if (!real_robot_) lock.lock();
  qp_time_us_ = 0.0;
  if (real_robot_) {
    // Copy a fixed-size snapshot via a nonblocking RT read.
    const HardwareRequest request = *hardware_buffer_.readFromRT();
    applyCompliance(request.translation, request.rotation, request.nullspace);
    Kmax_=request.Kmax; alpha_=request.alpha; cbf_active_=request.cbf_active;
    if (request.experiment_sequence != hardware_sequence_) {
      hardware_sequence_=request.experiment_sequence;
      requested_displacement_ = request.displacement;
      position_d_target_=Eigen::Map<const Eigen::Vector3d>(request.position.data());
      orientation_d_target_=Eigen::Quaterniond(request.orientation[0],request.orientation[1],
                                               request.orientation[2],request.orientation[3]);
      experiment_service_mode_=true;
      task_state_.requestStart();
    }
    if (period.toSec() == 0.0) {
      for (auto& joint : joint_handles_) joint.setCommand(0.0);
      return;  // libfranka first-callback requirement; no synthetic sample.
    }
  }
  const franka::RobotState robot_state = state_handle_->getRobotState();
  const std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  const std::array<double, 7> gravity_array = model_handle_->getGravity();
  const std::array<double, 49> mass_array = model_handle_->getMass();
  const std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  const Vector7d coriolis = Eigen::Map<const Vector7d>(coriolis_array.data());
  const Vector7d gravity = Eigen::Map<const Vector7d>(gravity_array.data());
  const Matrix7d mass = Eigen::Map<const Matrix7d>(mass_array.data());
  const Matrix6x7d jacobian = Eigen::Map<const Matrix6x7d>(jacobian_array.data());
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const Vector7d tau_J = Eigen::Map<const Vector7d>(robot_state.tau_J.data());

  // Total joint-space kinetic energy, published for diagnostics.
  const double kinetic_energy =
      0.5 * (dq.transpose() * mass * dq)(0, 0);

  const Eigen::Affine3d transform(
      Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE.data()));
  const Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  if (task_state_.consumeStartRequest()) {
    ++experiment_id_;
    experiment_initial_q_ = q;
    // Restart from the measured pose; keep the new requested target.
    position_d_ = position;
    orientation_d_ = orientation;
    q_d_nullspace_ = q;
    previous_jacobian_ = jacobian;
    previous_mass_ = mass;
    previous_mass_2_ = mass;
    derivative_sample_count_ = 0; previous_dt_=0.001;
    filtered_jacobian_dot_.setZero(); filtered_mass_dot_.setZero();
    if (!real_robot_ && cbf_active_ && !initializeQpSolver()) {
      task_state_.fail(4);
      if (!real_robot_) ROS_ERROR("Directional experiment aborted: could not reinitialize OSQP");
    }
  }

  // Snapshot diagnostics before updating the filtered reference below.
  Eigen::Vector3d target_position_sample;
  {
    std::unique_lock<std::mutex> lock(position_and_orientation_d_target_mutex_, std::defer_lock);
    if (!real_robot_) lock.lock();
    target_position_sample = position_d_target_;
  }
  const double ee_target_distance = (position - target_position_sample).norm();
  const double ee_reference_distance = (position - position_d_).norm();

  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = position - position_d_;

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail<3>() << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail<3>() = -transform.rotation() * error.tail<3>();

  Matrix6x7d jacobian_transpose_pinv = Matrix6x7d::Zero();
  if (jacobian.allFinite()) {
    if (real_robot_) {
      // Same damped pseudoinverse (lambda=0.2), fixed-size storage.
      const Eigen::Matrix<double,6,6> gram = jacobian*jacobian.transpose() +
          0.04*Eigen::Matrix<double,6,6>::Identity();
      jacobian_transpose_pinv=gram.ldlt().solve(jacobian);
    } else {
      Eigen::MatrixXd legacy_pinv;
      pseudoInverse(jacobian.transpose(), legacy_pinv);
      jacobian_transpose_pinv=legacy_pinv;
    }
  }

  const Vector7d tau_task =
      jacobian.transpose() *
      (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  const Vector7d tau_nullspace =
      (Matrix7d::Identity() - jacobian.transpose() * jacobian_transpose_pinv) *
      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
       2.0 * std::sqrt(nullspace_stiffness_) * dq);

  // Bias-free nominal control used by the CBF, matching the notes/pseudocode.
  const Vector7d u_nominal = tau_task + tau_nullspace;
  const double translation_gain_sample = cartesian_stiffness_(0, 0);
  const double rotation_gain_sample = cartesian_stiffness_(3, 3);
  const double nullspace_gain_sample = nullspace_stiffness_;

  // Franka gravity compensation is handled by the robot. The model Coriolis
  // term is used as the additive bias outside the CBF:
  //   tau_command = u_safe + u_bias.
  const Vector7d u_bias = coriolis;
  const Vector7d tau_nominal = u_nominal + u_bias;
  // Total effort in Gazebo is u + coriolis + gravity. Gravity is used only
  // to translate the QP bounds; it must NOT be added again to tau_command.
  const Vector7d torque_offset = coriolis + gravity;

  if (real_robot_) {
    hardware_coriolis_=coriolis;
    hardware_envelope_=hardware_cbf::envelope(hardware_limits_,robot_state.q,robot_state.dq,
                                             robot_state.tau_J_d,hardware_cutoff_,hardware_rate_);
    prediction_error_=have_prediction_ ?
        (Eigen::Map<const Vector7d>(robot_state.tau_J_d.data())-previous_prediction_).norm() : 0.0;
    const double control_period = period.toSec();
    const bool invalid_envelope = !hardware_envelope_.valid;
    const bool invalid_period =
        !std::isfinite(control_period) || control_period <= 0.0;
    const bool late_period =
        !invalid_period && control_period > max_period_seconds_;

    if (invalid_envelope && (invalid_period || late_period)) {
      task_state_.fail(11);  // Both hardware envelope and timing invalid.
    } else if (invalid_envelope) {
      task_state_.fail(7);   // Hardware torque envelope invalid.
    } else if (invalid_period) {
      task_state_.fail(9);   // Nonfinite or nonpositive control period.
    } else if (late_period) {
      task_state_.fail(10);  // Control period exceeds max_period_seconds.
    }
  }
  // Continue diagnostics while aborted, but do not solve or resume on a
  // periodic pose message. Only start_experiment can request another attempt.
  const CbfResult cbf_result = directionalKineticEnergyCbf(
      u_nominal, torque_offset, mass, jacobian, dq, period.toSec(),
      energy_mode_ != EnergyMode::kNone && cbf_active_ && !task_state_.aborted());
  if (!task_state_.aborted() && cbf_result.solver_status >= 3) {
    task_state_.fail(cbf_result.solver_status);
    if (!real_robot_) ROS_ERROR_STREAM("Directional experiment ABORTED (status "
                     << static_cast<int>(task_state_.error())
                     << "). Joint braking active. Send a new set_experiment_command to retry.");
  }

  Vector7d tau_command = cbf_result.u_safe + u_bias;
  if (!tau_command.allFinite() || !gravity.allFinite()) {
    if (!real_robot_) ROS_ERROR_THROTTLE(1.0, "Directional experiment aborted: non-finite commanded torque");
    task_state_.fail(3);
  }
  if (!task_state_.aborted()) {
    // Enforce the same effort box with CBF disabled. With CBF enabled this
    // should only remove numerical roundoff; recheck the actual command.
    tau_command = clampTorqueCommand(tau_command, gravity);
    if (energy_mode_ != EnergyMode::kNone && cbf_active_ && cbf_result.solver_status == 1) {
      Vector7d residual_command=tau_command;
      if (real_robot_) {
        hardware_cbf::Seven raw{};
        std::copy(tau_command.data(),tau_command.data()+7,raw.begin());
        const auto predicted=hardware_cbf::predict(hardware_envelope_,raw);
        residual_command=Eigen::Map<const Vector7d>(predicted.data());
      }
      const double applied_residual =
          (cbf_result.barrier_a * (residual_command - coriolis))(0, 0) +
          cbf_result.barrier_b + alpha_ * cbf_result.h;
      if (!DirectionalCbfTaskState::acceptsSolution(true, applied_residual,
                                                   cbf_residual_tolerance_)) {
        task_state_.fail(6);
        if (!real_robot_) ROS_ERROR_THROTTLE(1.0, "Directional CBF: bounded command violates CBF; aborting");
      }
    }
  }
  if (real_robot_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-update_begin).count()
                       > max_update_seconds_) task_state_.fail(8);
  if (task_state_.aborted()) {
    // Gravity is added by FrankaHWSim. Do not add Coriolis here. A positive
    // inertia-weighted viscous brake dissipates total energy without the very
    // large deceleration of a fixed damping gain on low-inertia wrist joints.
    // The requested brake is subsequently torque-limited. After limiting,
    // neither directional-CBF satisfaction nor total-energy dissipation is
    // guaranteed.
    tau_command.setZero();
    if (dq.allFinite()) {
      Eigen::LDLT<Matrix7d> brake_mass;
      if (mass.allFinite()) brake_mass.compute(mass);
      if (mass.allFinite() && brake_mass.info() == Eigen::Success && brake_mass.isPositive()) {
        tau_command = -abort_damping_ * mass * dq;
      } else {
        tau_command = -dq;  // Model-independent viscous braking if M is invalid.
      }
    }
  }

  // Also bound the fallback brake; never label it as a valid CBF solution.
  tau_command = clampTorqueCommand(tau_command, gravity);

  if (real_robot_) {
    hardware_cbf::Seven raw{};
    std::copy(tau_command.data(),tau_command.data()+7,raw.begin());
    const auto predicted=hardware_cbf::predict(hardware_envelope_,raw);
    predicted_torque_=Eigen::Map<const Vector7d>(predicted.data());
    previous_prediction_=predicted_torque_; have_prediction_=true;
  } else predicted_torque_=tau_command;
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
    std::unique_lock<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_, std::defer_lock);
    if (!real_robot_) position_d_target_mutex_lock.lock();
    if (!task_state_.aborted()) {
      position_d_ =
          filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
      orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
    }
  }

  Eigen::Map<Vector7d> u_des_map(&cbf_info_.u_des[0]);
  Eigen::Map<Vector7d> u_cbf_map(&cbf_info_.u_cbf[0]);
  Eigen::Map<Vector7d> u_measured_map(&cbf_info_.u_measured[0]);
  Eigen::Map<Vector7d> u_saturated_map(&cbf_info_.u_saturated[0]);
  Eigen::Map<Vector7d> u_ext_map(&cbf_info_.u_ext[0]);
  // Raw CBF coordinates: Coriolis excluded, before final command limiting.
  // On abort the CBF result is not the applied braking command.
  Eigen::Map<Vector7d>(&cbf_info_.u_nom[0]) = u_nominal;
  Eigen::Map<Vector7d>(&cbf_info_.u_safe[0]) = cbf_result.u_safe;
  Eigen::Map<Vector7d>(&cbf_info_.coriolis[0]) = coriolis;
  u_des_map = tau_nominal;
  u_cbf_map = tau_command;
  u_measured_map = tau_J;
  u_saturated_map = tau_command;
  u_ext_map.setZero();

  cbf_info_.h = cbf_result.h;
  cbf_info_.kinetic_energy = kinetic_energy;
  cbf_info_.directional_kinetic_energy = cbf_result.directional_kinetic_energy;
  cbf_info_.Kmax = Kmax_;
  cbf_info_.solver_status = task_state_.aborted() ? task_state_.error() : cbf_result.solver_status;
  cbf_info_.header.stamp = time;
  if (real_robot_) {
    if (realtime_cbf_publisher_->trylock()) {
      realtime_cbf_publisher_->msg_=cbf_info_;
      realtime_cbf_publisher_->unlockAndPublish();
    }
  } else cbf_publisher_.publish(cbf_info_);

  // No file I/O here. sample_index exposes missed trylocks/transport samples.
  const uint64_t sample_index = diagnostic_sample_index_++;
  if (diagnostics_publisher_->trylock()) {
    auto& diagnostic = diagnostics_publisher_->msg_;
    diagnostic.header.stamp = time;
    diagnostic.header.frame_id = base_frame_;
    diagnostic.sample_index = sample_index;
    diagnostic.experiment_id = experiment_id_;
    diagnostic.energy_mode = static_cast<uint8_t>(energy_mode_);
    for (int i = 0; i < 3; ++i)
      diagnostic.requested_displacement[i] = requested_displacement_[i];
    for (int i = 0; i < 7; ++i)
      diagnostic.experiment_initial_q[i] = experiment_initial_q_(i);
    diagnostic.dt = period.toSec();
    diagnostic.cbf_h = cbf_result.h;
    diagnostic.kinetic_energy_dir = cbf_result.directional_kinetic_energy;
    diagnostic.cbf_constraint_safe = std::numeric_limits<double>::quiet_NaN();
    if (cbf_result.barrier_model_valid && tau_command.allFinite() && coriolis.allFinite()) {
      diagnostic.cbf_constraint_safe =
          (cbf_result.barrier_a * (predicted_torque_ - coriolis))(0, 0) +
          cbf_result.barrier_b + alpha_ * cbf_result.h;
    }
    diagnostic.cbf_constraint_qp = cbf_result.constraint_qp;
    diagnostic.Kmax = Kmax_;
    diagnostic.alpha = alpha_;
    diagnostic.cbf_residual_tolerance = cbf_residual_tolerance_;
    for (int i = 0; i < 3; ++i) diagnostic.direction[i] = direction_(i);
    for (int i = 0; i < 6; ++i)
      diagnostic.svd_jacobian[i] = std::numeric_limits<double>::quiet_NaN();
    if (jacobian.allFinite()) {
      const Eigen::JacobiSVD<Matrix6x7d> svd(jacobian);
      for (int i = 0; i < 6; ++i) diagnostic.svd_jacobian[i] = svd.singularValues()(i);
    }
    diagnostic.kinetic_energy_total = kinetic_energy;
    for (int i = 0; i < 3; ++i) {
      diagnostic.ee_position[i] = position(i);
      diagnostic.target_position[i] = target_position_sample(i);
    }
    diagnostic.ee_target_distance = ee_target_distance;
    diagnostic.ee_reference_distance = ee_reference_distance;
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 7; ++i) {
      diagnostic.q[i] = (real_robot_ || cbf_active_) ? q(i) : unavailable;
      diagnostic.dq[i] = (real_robot_ || cbf_active_) ? dq(i) : unavailable;
      diagnostic.tau_command[i] = (real_robot_ || cbf_active_) ? tau_command(i) : unavailable;
      diagnostic.u_nom[i] = u_nominal(i);
      diagnostic.u_safe[i] = cbf_result.u_safe(i);
    }
    diagnostic.translational_stiffness = translation_gain_sample;
    diagnostic.rotational_stiffness = rotation_gain_sample;
    diagnostic.nullspace_stiffness = nullspace_gain_sample;
    diagnostic.robot_mode = (real_robot_ || cbf_active_) ? static_cast<uint8_t>(robot_state.robot_mode) : 0;
    diagnostic.runtime_environment = real_robot_ ? 1 : 0;
    diagnostic.debug_valid = real_robot_ || cbf_active_;
    diagnostic.qp_time_us=qp_time_us_;
    diagnostic.update_time_us=std::chrono::duration<double,std::micro>(
        std::chrono::steady_clock::now()-update_begin).count();
    diagnostic.torque_prediction_error=prediction_error_;
    for (int i=0;i<7;++i) {
      diagnostic.tau_J_d[i]=robot_state.tau_J_d[i];
      diagnostic.tau_J[i]=robot_state.tau_J[i];
      diagnostic.tau_predicted[i]=predicted_torque_(i);
    }
    diagnostic.control_command_success_rate=robot_state.control_command_success_rate;
    diagnostic.cbf_active = energy_mode_ != EnergyMode::kNone && cbf_active_;
    diagnostic.task_aborted = task_state_.aborted();
    diagnostic.cbf_solution_applied = !task_state_.aborted() && cbf_result.solver_status == 1;
    diagnostic.solver_status = cbf_info_.solver_status;
    diagnostics_publisher_->unlockAndPublish();
  }
}

CartesianImpedanceDirectionalKineticEnergyCBFController::CbfResult
CartesianImpedanceDirectionalKineticEnergyCBFController::directionalKineticEnergyCbf(
    const Vector7d& u_nominal,
    const Vector7d& torque_offset,
    const Matrix7d& mass,
    const Matrix6x7d& jacobian,
    const Vector7d& dq,
    double dt, bool enforce_cbf) {
  CbfResult result;

  // update() gates every failure before sending commands to the joints.
  // The nominal input is returned only for the intentional CBF bypass.
  result.u_safe = u_nominal;
  if (!mass.allFinite() || !jacobian.allFinite() || !dq.allFinite() ||
      !u_nominal.allFinite() || !torque_offset.allFinite()) {
    if (!real_robot_) ROS_ERROR_THROTTLE(1.0, "Directional CBF: non-finite model/state/control");
    result.solver_status = 3;
    return result;
  }

  if (!std::isfinite(dt) || dt <= 1.0e-6) {
    dt = 1.0e-3;
  }

  // -------------------------------------------------------------------------
  // Numerical derivatives of the raw model quantities, matching the
  // pseudocode. The derivative of Lambda_dir itself is then obtained
  // analytically below.
  // -------------------------------------------------------------------------
  Matrix6x7d jacobian_dot = Matrix6x7d::Zero();
  Matrix7d mass_dot = Matrix7d::Zero();

  if (derivative_sample_count_ >= 1) {
    jacobian_dot = (jacobian - previous_jacobian_) / dt;
  }
  if (derivative_sample_count_ >= 2) {
    mass_dot = (mass - previous_mass_2_) / (real_robot_ ? dt+previous_dt_ : 2.0*dt);
  }

  previous_jacobian_ = jacobian;
  previous_mass_2_ = previous_mass_;
  previous_mass_ = mass;
  ++derivative_sample_count_;
  previous_dt_=dt;

  if (real_robot_) {
    filtered_jacobian_dot_=derivative_filter_alpha_*jacobian_dot+
                          (1-derivative_filter_alpha_)*filtered_jacobian_dot_;
    filtered_mass_dot_=derivative_filter_alpha_*mass_dot+(1-derivative_filter_alpha_)*filtered_mass_dot_;
    jacobian_dot=filtered_jacobian_dot_; mass_dot=filtered_mass_dot_;
  }
  // -------------------------------------------------------------------------
  // 1-D directional operational space.
  // direction_ is constant, therefore d_ext_dot=0 and
  // J_dir_dot = direction^T * J_pos_dot.
  // -------------------------------------------------------------------------
  const Matrix3x7d translational_jacobian = jacobian.topRows<3>();
  const Matrix3x7d translational_jacobian_dot = jacobian_dot.topRows<3>();
  const RowVector7d directional_jacobian =
      direction_.transpose() * translational_jacobian;
  const RowVector7d directional_jacobian_dot =
      direction_.transpose() * translational_jacobian_dot;
  const double directional_velocity = (directional_jacobian * dq)(0, 0);

  // -------------------------------------------------------------------------
  // M^{-1} and Lambda_dir.
  // Use an LDLT solve instead of forming an explicit algebraic inverse, but
  // the resulting matrix is exactly the M^{-1} required by the derivation.
  // -------------------------------------------------------------------------
  Eigen::LDLT<Matrix7d> mass_ldlt(mass);
  if (mass_ldlt.info() != Eigen::Success || !mass_ldlt.isPositive()) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "\n================ CBF ERROR ================\n"
        "Directional kinetic-energy CBF: mass-matrix factorization failed.\n"
        "ABORT: nominal control will not be applied.\n"
        "===========================================");
    result.solver_status = 3;
    return result;
  }

  const Matrix7d mass_inverse = mass_ldlt.solve(Matrix7d::Identity());
  if (!mass_inverse.allFinite()) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "\n================ CBF ERROR ================\n"
        "Directional kinetic-energy CBF: invalid M^{-1}.\n"
        "ABORT: nominal control will not be applied.\n"
        "===========================================");
    result.solver_status = 3;
    return result;
  }

  const double lambda_dir_inv =
      (directional_jacobian * mass_inverse * directional_jacobian.transpose())(0, 0);
  if (!std::isfinite(lambda_dir_inv) ||
      (energy_mode_ == EnergyMode::kDirectional && lambda_dir_inv <= mobility_epsilon_)) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "\n================ CBF ERROR ================\n"
        "Directional kinetic-energy CBF: directional mobility is singular or too small: "
            << lambda_dir_inv << "\n"
        "ABORT: nominal control will not be applied.\n"
        "===========================================");
    result.solver_status = 3;
    return result;
  }

  // In total/none mode a singular projection only makes the directional
  // diagnostic unavailable; it must not abort a different energy constraint.
  const bool directional_valid = lambda_dir_inv > mobility_epsilon_;
  const double lambda_dir = directional_valid ? 1.0 / lambda_dir_inv : 0.0;

  // -------------------------------------------------------------------------
  // Analytical derivative of Lambda_dir.
  //
  // Lambda_dir^{-1} = J_dir M^{-1} J_dir^T
  //
  // d/dt Lambda_dir^{-1} =
  //     J_dir_dot M^{-1} J_dir^T
  //   + J_dir M^{-1} J_dir_dot^T
  //   - J_dir M^{-1} M_dot M^{-1} J_dir^T
  //
  // Lambda_dir_dot = -Lambda_dir Lambda_dir_inv_dot Lambda_dir.
  // -------------------------------------------------------------------------
  const double lambda_dir_inv_dot =
      (directional_jacobian_dot * mass_inverse * directional_jacobian.transpose())(0, 0) +
      (directional_jacobian * mass_inverse * directional_jacobian_dot.transpose())(0, 0) -
      (directional_jacobian * mass_inverse * mass_dot * mass_inverse *
       directional_jacobian.transpose())(0, 0);
  const double lambda_dir_dot =
      -lambda_dir * lambda_dir_inv_dot * lambda_dir;

  // Directional kinetic energy and barrier function.
  const double directional_energy = directional_valid
      ? 0.5 * lambda_dir * directional_velocity * directional_velocity
      : std::numeric_limits<double>::quiet_NaN();
  // In compensated coordinates M*qdd=u, so
  // d/dt (dq^T M dq/2) = dq^T u + dq^T M_dot dq/2.
  const double total_energy = 0.5 * (dq.transpose() * mass * dq)(0, 0);
  const double protected_energy = energy_mode_ == EnergyMode::kDirectional
      ? directional_energy : total_energy;
  const double h = Kmax_ - protected_energy;
  result.h = h;
  result.directional_kinetic_energy = directional_energy;

  // -------------------------------------------------------------------------
  // CBF derivative in the compensated coordinates used by the notes:
  //
  //   M qdd = u
  //   h_dot = a u + b
  //
  // Coriolis is NOT part of a or b. It is added outside the CBF as u_bias.
  // -------------------------------------------------------------------------
  const RowVector7d directional_a =
      -directional_velocity * lambda_dir * directional_jacobian * mass_inverse;
  const double directional_b =
      -directional_velocity * lambda_dir * (directional_jacobian_dot * dq)(0, 0) -
      0.5 * directional_velocity * lambda_dir_dot * directional_velocity;
  RowVector7d a = directional_a;
  double b = directional_b;
  if (energy_mode_ != EnergyMode::kDirectional) {
    a = -dq.transpose();
    b = -0.5 * (dq.transpose() * mass_dot * dq)(0, 0);
  }

  result.barrier_a = a;
  result.barrier_b = b;
  result.barrier_model_valid = a.allFinite() && std::isfinite(b) && std::isfinite(h);

  // Diagnostics also describe nominal and abort commands. Enforcement is unchanged.
  if (!enforce_cbf) {
    result.solver_status = 0;
    return result;
  }

  // h_dot + alpha*h >= 0  ->  a*u >= -alpha*h - b.
  const double cbf_lower_bound = -alpha_ * h - b;
  if (!a.allFinite() || !std::isfinite(cbf_lower_bound) ||
      !std::isfinite(h) || !std::isfinite(protected_energy)) {
    if (!real_robot_) ROS_ERROR_THROTTLE(1.0, "Directional CBF: non-finite barrier coefficients");
    result.solver_status = 3;
    return result;
  }

  if (real_robot_) {
    const auto begin=std::chrono::steady_clock::now();
    hardware_cbf::Seven nominal{}, coefficients{}, solution{};
    const double beta=hardware_envelope_.gain;
    double rhs=cbf_lower_bound;
    for (int i=0;i<7;++i) {
      nominal[i]=u_nominal(i)+hardware_coriolis_(i);
      coefficients[i]=beta*a(i);
      rhs-=a(i)*((1-beta)*hardware_envelope_.previous[i]-hardware_coriolis_(i));
    }
    const bool solved=hardware_envelope_.valid && hardware_cbf::project(nominal,
        hardware_envelope_.lower,hardware_envelope_.upper,coefficients,rhs,solution);
    qp_time_us_=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
    if (!solved) { result.solver_status=5; return result; }
    const auto predicted=hardware_cbf::predict(hardware_envelope_,solution);
    result.constraint_qp=(a*(Eigen::Map<const Vector7d>(predicted.data())-hardware_coriolis_))(0,0)+b+alpha_*h;
    if (!DirectionalCbfTaskState::acceptsSolution(true,result.constraint_qp,cbf_residual_tolerance_)) {
      result.solver_status=6; return result;
    }
    result.u_safe=Eigen::Map<const Vector7d>(solution.data())-hardware_coriolis_;
    result.solver_status=1; return result;
  }
  for (int joint = 0; joint < 7; ++joint) {
    constraint_matrix_.coeffRef(0, joint) = a(joint);
  }
  constraint_matrix_.makeCompressed();

  lower_bounds_(0) = cbf_lower_bound;
  upper_bounds_(0) = kInfinity_;
  lower_bounds_.tail<7>() = -torque_limits_ - torque_offset;
  upper_bounds_.tail<7>() = torque_limits_ - torque_offset;

  const Vector7d objective_vector = -2.0 * u_nominal;
  const absl::Status matrix_status = qp_solver_.UpdateConstraintMatrix(constraint_matrix_);
  const absl::Status objective_status = qp_solver_.SetObjectiveVector(objective_vector);
  const absl::Status bounds_status = qp_solver_.SetBounds(lower_bounds_, upper_bounds_);

  if (!matrix_status.ok() || !objective_status.ok() || !bounds_status.ok()) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "\n================ CBF ERROR ================\n"
        "Directional kinetic-energy CBF: failed to update OSQP data.\n"
        "ABORT: nominal control will not be applied.\n"
        "===========================================");
    result.solver_status = 4;
    return result;
  }

  const osqp::OsqpExitCode exit_code = qp_solver_.Solve();
  if (exit_code != osqp::OsqpExitCode::kOptimal) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "\n================ CBF QP INFEASIBLE / FAILED ================\n"
        "Directional kinetic-energy CBF: OSQP status: "
            << osqp::ToString(exit_code) << "\n"
        "ABORT: nominal control will not be applied.\n"
        "TASK ABORTED; the node remains available for another experiment.\n"
        "============================================================");
    result.solver_status = 5;
    return result;
  }

  const Eigen::VectorXd solution = qp_solver_.primal_solution();
  if (solution.size() != 7 || !solution.allFinite()) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "\n================ CBF ERROR ================\n"
        "Directional kinetic-energy CBF: invalid OSQP solution.\n"
        "ABORT: nominal control will not be applied.\n"
        "===========================================");
    result.solver_status = 5;
    return result;
  }

  // Reject material box violations. Only project numerical solver tolerance,
  // then check the CBF again on the projected candidate before accepting it.
  const Vector7d torque_lower = lower_bounds_.tail<7>();
  const Vector7d torque_upper = upper_bounds_.tail<7>();
  const Vector7d raw_solution = solution;
  result.constraint_qp = (a * raw_solution)(0, 0) + b + alpha_ * h;
  const double torque_violation = std::max(
      (torque_lower - raw_solution).maxCoeff(),
      (raw_solution - torque_upper).maxCoeff());
  if (torque_violation > torque_limit_tolerance_) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(1.0, "Directional CBF: torque bounds rejected by "
                                      << torque_violation << " Nm; aborting");
    result.solver_status = 6;
    return result;
  }
  const Vector7d bounded_solution = raw_solution.cwiseMax(torque_lower).cwiseMin(torque_upper);
  const double safe_constraint = (a * bounded_solution)(0, 0) + b + alpha_ * h;
  if (!DirectionalCbfTaskState::acceptsSolution(true, safe_constraint,
                                               cbf_residual_tolerance_)) {
    if (!real_robot_) ROS_ERROR_STREAM_THROTTLE(
        1.0, "Directional CBF residual rejected: " << safe_constraint
             << ", tolerance=" << cbf_residual_tolerance_);
    result.solver_status = 6;
    return result;
  }

  result.u_safe = bounded_solution;
  result.solver_status = 1;
  return result;
}

bool CartesianImpedanceDirectionalKineticEnergyCBFController::startExperimentCallback(
    franka_msgs::StartDirectionalExperiment::Request& request,
    franka_msgs::StartDirectionalExperiment::Response& response) {
  const auto& p = request.target.position;
  const auto& o = request.target.orientation;
  Eigen::Quaterniond orientation(o.w, o.x, o.y, o.z);
  const auto& minimum = compliance_paramConfig::__getMin__();
  const auto& maximum = compliance_paramConfig::__getMax__();
  if (energy_mode_ == EnergyMode::kNone && request.cbf_active) {
    response.success = false;
    response.message = "energy_mode=none requires cbf_active=false";
    return true;
  }
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !orientation.coeffs().allFinite() || !std::isfinite(orientation.norm()) ||
      orientation.norm() < 1.0e-12 || !std::isfinite(request.Kmax) ||
      !std::isfinite(request.alpha) || request.Kmax < minimum.Kmax ||
      request.Kmax > maximum.Kmax || request.alpha < minimum.alpha ||
      request.alpha > maximum.alpha ||
      !std::isfinite(request.displacement[0]) ||
      !std::isfinite(request.displacement[1]) ||
      !std::isfinite(request.displacement[2])) {
    response.success = false;
    response.message = "Invalid target or CBF parameters outside dynamic-reconfigure limits";
    return true;
  }
  orientation.normalize();
  boost::recursive_mutex::scoped_lock dynamic_lock(dynamic_config_mutex_);
  std::lock_guard<std::recursive_mutex> control_lock(control_mutex_);
  auto config = current_config_;
  config.cbf_active = request.cbf_active;
  config.Kmax = request.Kmax;
  config.alpha = request.alpha;
  batching_hardware_request_=real_robot_;
  complianceParamCallback(config, 0);
  dynamic_server_compliance_param_->updateConfig(config);
  batching_hardware_request_=false;
  if (real_robot_) {
    hardware_request_.position={{p.x,p.y,p.z}};
    hardware_request_.displacement={{request.displacement[0], request.displacement[1],
                                    request.displacement[2]}};
    hardware_request_.orientation={{orientation.w(),orientation.x(),orientation.y(),orientation.z()}};
    ++hardware_request_.experiment_sequence;
    hardware_buffer_.writeFromNonRT(hardware_request_);
    response.success=true;
    response.message="Experiment queued; inspect directional diagnostics for completion/abort";
    return true;
  }
  position_d_target_ << p.x, p.y, p.z;
  requested_displacement_ = {{request.displacement[0], request.displacement[1],
                              request.displacement[2]}};
  orientation_d_target_ = orientation;
  // After the first service command, poses and restarts are accepted together
  // through this service. Stale queued topic messages cannot replace the goal.
  experiment_service_mode_ = true;
  task_state_.requestStart();
  response.success = true;
  response.message = "Experiment queued; inspect /cbf_info for execution status";
  return true;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  std::lock_guard<std::recursive_mutex> lock(control_mutex_);
  current_config_ = config;
  if (real_robot_) {
    hardware_request_.translation=config.translational_stiffness;
    hardware_request_.rotation=config.rotational_stiffness;
    hardware_request_.nullspace=config.nullspace_stiffness;
    hardware_request_.Kmax=config.Kmax; hardware_request_.alpha=config.alpha;
    hardware_request_.cbf_active=config.cbf_active;
    if (!batching_hardware_request_) hardware_buffer_.writeFromNonRT(hardware_request_);
    return;
  }
  applyCompliance(config.translational_stiffness,config.rotational_stiffness,config.nullspace_stiffness);
  cbf_active_=config.cbf_active; Kmax_=config.Kmax; alpha_=config.alpha;
}

void CartesianImpedanceDirectionalKineticEnergyCBFController::applyCompliance(
    double translation, double rotation, double nullspace) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner<3, 3>() =
      translation * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner<3, 3>() =
      rotation * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner<3, 3>() =
      damping_ratio_ * 2.0 * std::sqrt(translation) *
      Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner<3, 3>() =
      damping_ratio_ * 2.0 * std::sqrt(rotation) *
      Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = nullspace;

}

void CartesianImpedanceDirectionalKineticEnergyCBFController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  if (real_robot_) return;  // Hardware targets are explicit atomic service requests only.
  std::lock_guard<std::recursive_mutex> lock(control_mutex_);
  if (task_state_.aborted() || experiment_service_mode_) return;
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
