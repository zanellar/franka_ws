// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_cbf_controller.h>

#include <cmath>
#include <memory>
#include <tuple>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <eigen_conversions/eigen_msg.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <osqp++.h>

using namespace osqp;
using namespace Eigen;

namespace franka_example_controllers {

bool CartesianImpedanceCBFController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/trajectory_publisher/equilibrium_pose", 20, &CartesianImpedanceCBFController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceCBFController: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("/alpha", alpha)) {
    ROS_ERROR_STREAM("CartesianImpedanceCBFController: Could not read parameter alpha");
    return false;
  }

  if (!node_handle.getParam("/cbf_active", cbf_active)) {
    ROS_ERROR_STREAM("CartesianImpedanceCBFController: Could not read parameter cbf_active");
    return false;
  }

  if (!node_handle.getParam("/damping_ratio", damping_ratio)) {
    ROS_ERROR_STREAM("CartesianImpedanceCBFController: Could not read parameter damping_ratio");
    return false;
  }

  if (!node_handle.getParam("/Kmax", Kmax)) {
    ROS_ERROR_STREAM("CartesianImpedanceCBFController: Could not read parameter Kmax");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceCBFController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceCBFController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceCBFController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceCBFController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceCBFController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceCBFController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceCBFController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceCBFController::complianceParamCallback, this, _1, _2));

  

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // QPsolver setup, see https://osqp.org/docs/interfaces/solver_settings.html
  QPinstance.lower_bounds.resize(1+2*7);
  QPinstance.upper_bounds.resize(1+2*7); 

  QPsettings.verbose = false; // turn off printing of results
  QPsettings.max_iter = 8000; // makes time limit the real limiting factor
  QPsettings.time_limit = 0.9e-3; // loop should always run within a millisecond

  // QPsettings.eps_abs = 1e-15;
  // QPsettings.eps_rel = 1e-15;

  QPsettings.rho = 0.1;
  QPsettings.sigma = 1e-6;
  QPsettings.alpha = 1.6;
  // QPsettings.adaptive_rho = false;

  QPsettings.warm_start = false;
  QPsettings.polish = true;

  return true;
}

void CartesianImpedanceCBFController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;


  // dq EMA setup
  dq_filtered = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.dq.data());
}

void CartesianImpedanceCBFController::update(const ros::Time& time,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_matrix = model_handle_-> getMass();
  std::array<double, 7> gravity_array = model_handle_-> getGravity();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_matrix.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_cbf(7), tau_sat(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = cartesian_damping_
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis; // note that gravity is done by Franka itself, cannot be turned off

  // apply cbf layer
  double h = 0;

  // filter joint velocity (discreate exponential moving average)
  // dq_filtered = EMA(dq, dq_filtered, EMAlpha);

  // filter joint velocity (limit change in joint velocity based on specs)
  dq_saturated = saturateQdotRate(dq, dq_saturated);

  // Compute the diagnostic barrier value from measured joint velocity.
  std::tie(tau_cbf, h) = cbfAnalytical(tau_d, mass, gravity, dq);
  cbf_info.h = h;

  if (cbf_active) {
    ROS_DEBUG_ONCE("cbf active");

    // Compute and apply the CBF-filtered torque.
    std::tie(tau_cbf, h) = cbfCompute(
        tau_d,
        mass,
        gravity,
        Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_saturated.data()));
  } else {
    ROS_DEBUG_ONCE("cbf not active");

    // Bypass the QP completely.
    tau_cbf = tau_d;
  }

  Eigen::Map<Eigen::VectorXd>(&cbf_info.u_cbf[0], 7, 1) = tau_cbf;

  // Saturate torque rate to avoid discontinuities -> handled by franka, this implementation does not work at all. Therefore left out.
  // tau_sat << saturateTorqueRate(tau_cbf, tau_J_d);
  tau_sat = tau_cbf;

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_sat(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  
  Eigen::Map<Eigen::VectorXd>(&cbf_info.u_measured[0], 7, 1) = tau_J;
  Eigen::Map<Eigen::VectorXd>(&cbf_info.u_des[0], 7, 1) = tau_d;
  Eigen::Map<Eigen::VectorXd>(&cbf_info.u_saturated[0], 7, 1) = tau_sat;
  cbf_info.header.stamp = time;
  cbf_publisher.publish(cbf_info);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceCBFController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceCBFController::saturateQdotRate(
    const Eigen::Matrix<double, 7, 1>& qdot_measured,
    const Eigen::Matrix<double, 7, 1>& qdot_prev) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> qdot_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = qdot_measured[i] - qdot_prev[i];
    qdot_saturated[i] =
        qdot_prev[i] + std::max(std::min(difference, delta_qdot_max_[i]), -delta_qdot_max_[i]);
  }
  return qdot_saturated;
}

std::tuple<Eigen::Matrix<double, 7, 1>, double> CartesianImpedanceCBFController::cbfCompute(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Map<Eigen::Matrix<double, 7, 7>>& D,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& G,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq) {
  Eigen::Matrix<double, 7, 1> tau_d_new{};
  // see https://osqp.org/docs/examples/setup-and-solve.html
  // implements the CBF function " Kmax - .5 * qdot.T @ D @ qdot >= 0 " 

  // compute cbf value
  h(0,0) = Kmax -.5 * dq.transpose() * D * dq;

  /* Load problem data */

  // objective matrix P = Identity
  objective_matrix.setIdentity();
  QPinstance.objective_matrix = objective_matrix; //P

  // objective vector q' = -1*desired control input
  QPinstance.objective_vector = -1*tau_d_calculated;

  // contstrain matrix G = dq.T @ B (B is identity)
  constraint_matrix << dq.transpose(), Eigen::Matrix<double, 7,7>::Identity(7,7);
  QPinstance.constraint_matrix = constraint_matrix.sparseView(); 

  // lower bound: not used, so set to -infinity
  lower_bounds << -kInfinity, -tau_max_;
  QPinstance.lower_bounds = lower_bounds.sparseView();

  // upper bound: h' = alpha(Kmax - .5 * dq.T @ D @ dq) + G.T @ dq
  // hp = (alpha*h).sparseView();
  upper_bounds << alpha*h, tau_max_;
  // for (int i = 0; i < 8; i++) {
  //   ROS_INFO("val %i, %f", i, upper_bounds[i]);
  // }
  QPinstance.upper_bounds = upper_bounds.sparseView();

  absl::Status status = QPsolver.Init(QPinstance, QPsettings);
  // check if status.ok, print warning if not
  if (!status.ok()) {
    ROS_ERROR("QPsolver did not return OK status after initialisation"); // TO DO: PRINT ABSL::STATUS MESSAGE
  }

  OsqpExitCode exit_code = QPsolver.Solve();
  // Check if exit_code == OsqpExitCode::kOptimal.
  if(exit_code != OsqpExitCode::kOptimal) {
    ROS_WARN("QPsolver did not find optimal solution, exit code %s", ToString(exit_code).c_str()); // TO DO: print error code (verbose preferably)
  }
  
  optimal_solution = QPsolver.primal_solution();

  for (size_t i = 0; i < 7; i++) {
    tau_d_new[i] =
        optimal_solution[i];
  }

  // verify result
  double left = (-dq.transpose()*tau_d_new)[0,0];
  double right = -alpha*h[0,0];
  if(left-right < -abs(left)/100000){
    ROS_WARN("CBF constraint not met: %f - %f = %f", left, right, left-right);
  }

  return std::make_tuple(tau_d_new, h(0,0));
}

std::tuple<Eigen::Matrix<double, 7, 1>, double> CartesianImpedanceCBFController::cbfAnalytical(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Map<Eigen::Matrix<double, 7, 7>>& D,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& G,
    const Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq) {
  Eigen::Matrix<double, 7, 1> tau_d_new{};
  tau_d_new = tau_d_calculated;
  // implements the CBF function " Kmax - .5 * qdot.T @ D @ qdot >= 0 using the analytical CBF solution" 

  // compute cbf value
  h(0,0) = Kmax -.5 * dq.transpose() * D * dq;

  double Psi = (-dq.transpose() * tau_d_calculated + alpha*h)(0,0);
  if(Psi < 0){
    tau_d_new += dq/(dq.transpose()*dq)(0,0) * Psi;
  }

  // verify result
  double left = (-dq.transpose()*tau_d_new)[0,0];
  double right = -alpha*h[0,0];
  if(left-right < -abs(left)/100000){
    ROS_WARN("condition not met: %f - %f = %f", left, right, left-right);
  }

  return std::make_tuple(tau_d_new, h(0,0));
}

void CartesianImpedanceCBFController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << damping_ratio * 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << damping_ratio * 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceCBFController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceCBFController::EMA(
    Eigen::Matrix<double, 7, 1> dq,
    Eigen::Matrix<double, 7, 1> dq_prev,
    double alpha) {
    // return dq;
    return alpha*dq + (1-alpha)*dq_prev; 
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceCBFController,
                       controller_interface::ControllerBase)
                   