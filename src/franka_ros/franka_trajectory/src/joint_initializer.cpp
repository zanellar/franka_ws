#include <franka_trajectory/joint_initializer.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <franka_msgs/StartDirectionalExperiment.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <urdf/model.h>

namespace franka_trajectory {
namespace {
template<class Values> std::array<double, 7> asArray(const Values& input) {
  std::array<double, 7> result;
  std::copy(input.begin(), input.end(), result.begin());
  return result;
}
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

JointInitializer::JointInitializer(ros::NodeHandle root, ros::NodeHandle private_node,
                                   const std::string& directional_service)
    : root_(root), private_node_(private_node), state_node_(root),
      directional_service_(directional_service) {
  private_node_.param("enable_joint_initialization", enabled_, false);
  std::string environment;
  private_node_.param<std::string>("runtime_environment", environment, "gazebo");
  require(environment=="gazebo" || environment=="real", "Invalid runtime_environment");
  real_robot_=environment=="real";
  joint_controller_type_=real_robot_ ? "position_controllers/JointTrajectoryController" :
                                      "effort_controllers/JointTrajectoryController";
  private_node_.param("initialization_acceleration_cap", acceleration_cap_, 0.5);
  private_node_.param("initialization_jerk_cap", jerk_cap_, 1.0);
  private_node_.param<std::string>("arm_id", arm_id_, "fr3");
  private_node_.param<std::string>("cartesian_controller", cartesian_controller_,
      "cartesian_impedance_directional_kinetic_energy_cbf_controller");
  private_node_.param<std::string>("joint_trajectory_controller", joint_controller_,
      real_robot_ ? "position_joint_trajectory_controller" : "effort_joint_trajectory_controller");
  private_node_.param<std::string>("controller_manager", manager_, "/controller_manager");
  private_node_.param<std::string>("franka_state_topic", state_topic_,
      "/franka_state_controller/franka_states");
  private_node_.param("initialization_joint_margin", margin_, 0.02);
  private_node_.param("initialization_velocity_scale", velocity_scale_, 0.2);
  private_node_.param("initialization_velocity_cap", velocity_cap_, 0.5);
  private_node_.param("initialization_position_tolerance", position_tolerance_, 0.01);
  private_node_.param("initialization_velocity_tolerance", velocity_tolerance_, 0.02);
  private_node_.param("initialization_settle_time", settle_time_, 0.2);
  private_node_.param("initialization_wall_timeout", wall_timeout_, 120.0);
  for (size_t i=0; i<7; ++i) joint_names_.push_back(arm_id_+"_joint"+std::to_string(i+1));
  joint_hold_publisher_ = root_.advertise<trajectory_msgs::JointTrajectory>(
      "/"+joint_controller_+"/command", 1, false);
  // A separate callback queue keeps state reception live inside the blocking service.
  state_node_.setCallbackQueue(&state_queue_);
  state_subscriber_ = state_node_.subscribe<franka_msgs::FrankaState>(
      state_topic_, 1, [this](const franka_msgs::FrankaStateConstPtr& msg) {
        state_ = *msg;
        ++state_sequence_;
      });
}

std::vector<controller_manager_msgs::ControllerState> JointInitializer::controllers() {
  controller_manager_msgs::ListControllers service;
  require(root_.serviceClient<controller_manager_msgs::ListControllers>(manager_+"/list_controllers")
              .call(service), "controller_manager/list_controllers unavailable");
  return service.response.controller;
}

bool JointInitializer::cartesianRunning() {
  for (const auto& controller : controllers())
    if (controller.name == cartesian_controller_) return controller.state == "running";
  return false;
}

void JointInitializer::requireState(const std::string& name, const std::string& expected) {
  for (const auto& controller : controllers()) {
    if (controller.name == name) {
      require(controller.state == expected, name+" is "+controller.state+", expected "+expected);
      return;
    }
  }
  throw std::runtime_error("Controller not loaded: "+name);
}

void JointInitializer::switchControllers(const std::vector<std::string>& start,
                                         const std::vector<std::string>& stop) {
  controller_manager_msgs::SwitchController service;
  service.request.start_controllers = start;
  service.request.stop_controllers = stop;
  service.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
  service.request.start_asap = false;
  service.request.timeout = 5.0;
  require(root_.serviceClient<controller_manager_msgs::SwitchController>(manager_+"/switch_controller")
              .call(service) && service.response.ok, "Strict controller switch failed");
  for (const auto& name : start) requireState(name, "running");
  for (const auto& name : stop) requireState(name, "stopped");
}

franka_msgs::FrankaState JointInitializer::nextState(double wall_timeout) {
  const auto deadline = ros::WallTime::now()+ros::WallDuration(wall_timeout);
  const uint64_t before = state_sequence_;
  while (ros::ok() && ros::WallTime::now() < deadline) {
    state_queue_.callAvailable(ros::WallDuration(0.01));
    if (state_sequence_ == before) continue;
    const double age = (ros::Time::now()-state_.header.stamp).toSec();
    if (!state_.header.stamp.isZero() && age >= -0.01 && age <= 0.2 &&
        (!have_stamp_ || state_.header.stamp > previous_stamp_)) {
      for (double q : state_.q) require(std::isfinite(q), "Nonfinite measured q");
      for (double dq : state_.dq) require(std::isfinite(dq), "Nonfinite measured dq");
      require(state_.robot_mode == franka_msgs::FrankaState::ROBOT_MODE_MOVE ||
              state_.robot_mode == franka_msgs::FrankaState::ROBOT_MODE_IDLE,
              "Robot mode is not MOVE/IDLE; resolve the robot error first");
      previous_stamp_ = state_.header.stamp;
      have_stamp_ = true;
      return state_;
    }
  }
  throw std::runtime_error("No fresh, advancing Franka state (check clock, connection and state controller)");
}

void JointInitializer::waitSettled(bool check_target) {
  const auto deadline = ros::WallTime::now()+ros::WallDuration(check_target ? wall_timeout_ : 5.0);
  ros::Time stable_since;
  while (ros::ok() && ros::WallTime::now() < deadline) {
    auto state = nextState(std::min(2.0, std::max(0.01, (deadline-ros::WallTime::now()).toSec())));
    bool stable = atRest(asArray(state.dq), velocity_tolerance_);
    if (check_target)
      for (size_t i=0; i<7; ++i) stable = stable && std::abs(state.q[i]-target_[i]) <= position_tolerance_;
    if (!stable) {
      stable_since = ros::Time();
    } else {
      if (stable_since.isZero()) stable_since = state.header.stamp;
      if ((state.header.stamp-stable_since).toSec() >= settle_time_) {
        settled_state_ = state;
        return;
      }
    }
  }
  throw std::runtime_error(check_target ? "Joint target did not settle within tolerances" :
                                         "Robot must be stationary before joint initialization");
}

geometry_msgs::Pose JointInitializer::measuredPose(const franka_msgs::FrankaState& state) const {
  const auto& m = state.O_T_EE;
  for (double value : m) require(std::isfinite(value), "Nonfinite measured EE transform");
  tf2::Matrix3x3 rotation(m[0],m[4],m[8],m[1],m[5],m[9],m[2],m[6],m[10]);
  require(std::abs(rotation.determinant()-1.0) < 1e-3, "Invalid EE rotation matrix");
  tf2::Quaternion q;
  rotation.getRotation(q);
  require(std::isfinite(q.length2()) && q.length2() > 1e-12, "Invalid EE quaternion");
  q.normalize();
  geometry_msgs::Pose pose;
  pose.position.x=m[12]; pose.position.y=m[13]; pose.position.z=m[14];
  pose.orientation.x=q.x(); pose.orientation.y=q.y(); pose.orientation.z=q.z(); pose.orientation.w=q.w();
  return pose;
}

void JointInitializer::preflight() {
  stage_ = "preflight";
  require(enabled_, "Joint initialization is disabled in this launch");
  bool sim_time = false;
  root_.getParam("/use_sim_time", sim_time);
  if (real_robot_) {
    require(!sim_time, "Real initialization requires /use_sim_time=false");
  } else {
    require(sim_time && ros::service::exists("/gazebo/get_world_properties", false),
            "Gazebo initialization requires the simulator and its clock");
  }
  require(!directional_service_.empty(), "Directional experiment service is not configured");
  for (double parameter : {position_tolerance_, velocity_tolerance_, settle_time_, wall_timeout_})
    require(std::isfinite(parameter) && parameter > 0.0, "Invalid initialization tolerance/timeout");
  std::string urdf_text;
  require(root_.getParam("/robot_description", urdf_text), "robot_description is unavailable");
  urdf::Model model;
  require(model.initString(urdf_text), "Invalid robot_description");
  for (size_t i=0; i<7; ++i) {
    auto joint = model.getJoint(joint_names_[i]);
    require(joint && joint->type == urdf::Joint::REVOLUTE && joint->limits,
            "Missing bounded revolute URDF joint: "+joint_names_[i]);
    bounds_[i] = {joint->limits->lower, joint->limits->upper, joint->limits->velocity};
    if (real_robot_) {
      require(static_cast<bool>(joint->safety), "Missing URDF soft joint limits");
      bounds_[i].lower=std::max(bounds_[i].lower,joint->safety->soft_lower_limit);
      bounds_[i].upper=std::min(bounds_[i].upper,joint->safety->soft_upper_limit);
    }
  }
  validateJointTarget(target_, bounds_, margin_, duration_);
  bool cart_running=false, joint_running=false, cart_loaded=false;
  for (const auto& controller : controllers()) {
    if (controller.name == cartesian_controller_) {
      cart_loaded=true; cart_running=controller.state=="running";
    }
    if (controller.name == joint_controller_) {
      require(controller.type == joint_controller_type_,
              "Unexpected initialization controller type: "+controller.type);
      joint_running=controller.state=="running";
    }
  }
  require(cart_loaded && (cart_running != joint_running),
          "Expected exactly one of Cartesian or initialization controller running");
  // Ignore previous clock epoch when a new explicit attempt begins.
  have_stamp_=false;
  waitSettled(false);
  start_state_=settled_state_;
  measuredPose(start_state_);  // Validate handoff data before any switch.
  const double minimum = requiredDuration();
  require(duration_ >= minimum, "duration too short; use at least "+std::to_string(minimum)+" s");
}

double JointInitializer::requiredDuration() const {
  double minimum=minimumJointDuration(asArray(start_state_.q),target_,bounds_,velocity_scale_,velocity_cap_);
  if (real_robot_) minimum=std::max(minimum,hardwareJointDuration(
      asArray(start_state_.q),target_,acceleration_cap_,jerk_cap_));
  return minimum;
}

void JointInitializer::invalidateReference() {
  *reference_ready_ = false;
  stage_ = "acquire joint controller";
}

void JointInitializer::acquireJointController() {
  bool loaded=false, running=false;
  for (const auto& controller : controllers()) {
    if (controller.name == joint_controller_) { loaded=true; running=controller.state=="running"; }
  }
  if (!loaded) {
    controller_manager_msgs::LoadController service;
    service.request.name=joint_controller_;
    require(root_.serviceClient<controller_manager_msgs::LoadController>(manager_+"/load_controller")
                .call(service) && service.response.ok, "Could not load "+joint_controller_);
  }
  for (const auto& controller : controllers())
    if (controller.name == joint_controller_)
      require(controller.type == joint_controller_type_, "Wrong joint-controller type");
  if (!running) switchControllers({joint_controller_}, {cartesian_controller_});
  requireState(cartesian_controller_, "stopped");
  requireState(joint_controller_, "running");
  if (!action_) action_.reset(new ActionClient(root_, "/"+joint_controller_+"/follow_joint_trajectory", true));
  const auto deadline=ros::WallTime::now()+ros::WallDuration(5.0);
  while (ros::ok() && !action_->isServerConnected() && ros::WallTime::now() < deadline)
    ros::WallDuration(0.01).sleep();
  require(action_->isServerConnected(), "Joint trajectory action server unavailable");
}

void JointInitializer::moveAndSettle() {
  stage_ = "joint trajectory";
  // Re-read the actual state after switching; the joint controller starts by holding it.
  waitSettled(false);
  start_state_=settled_state_;
  const double minimum=requiredDuration();
  require(duration_ >= minimum, "Robot moved during handoff; retry with a longer duration");
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names=joint_names_;
  trajectory_msgs::JointTrajectoryPoint first, last;
  first.positions.assign(start_state_.q.begin(),start_state_.q.end());
  first.velocities.assign(7,0.0); first.accelerations.assign(7,0.0);
  first.time_from_start=ros::Duration(0.0);
  last.positions.assign(target_.begin(),target_.end());
  last.velocities.assign(7,0.0); last.accelerations.assign(7,0.0);
  last.time_from_start=ros::Duration(duration_);
  goal.trajectory.points={first,last};
  goal.goal_time_tolerance=ros::Duration(3.0);
  for (const auto& name : joint_names_) {
    control_msgs::JointTolerance tolerance;
    tolerance.name=name; tolerance.position=position_tolerance_;
    tolerance.velocity=velocity_tolerance_; tolerance.acceleration=-1.0;
    goal.goal_tolerance.push_back(tolerance);
  }
  action_->sendGoal(goal);
  const auto deadline=ros::WallTime::now()+ros::WallDuration(wall_timeout_);
  ros::WallTime last_progress=ros::WallTime::now();
  ros::Time stamp=start_state_.header.stamp;
  while (ros::ok() && !action_->getState().isDone() && ros::WallTime::now() < deadline) {
    state_queue_.callAvailable(ros::WallDuration(0.01));
    if (state_.header.stamp > stamp) {
      for (double q : state_.q) require(std::isfinite(q), "Nonfinite q during initialization");
      for (double dq : state_.dq) require(std::isfinite(dq), "Nonfinite dq during initialization");
      require(state_.robot_mode == franka_msgs::FrankaState::ROBOT_MODE_MOVE ||
              state_.robot_mode == franka_msgs::FrankaState::ROBOT_MODE_IDLE,
              "Robot error during initialization");
      stamp=state_.header.stamp; last_progress=ros::WallTime::now();
    }
    require((ros::WallTime::now()-last_progress).toSec() < 2.0,
            "Franka state/clock stopped during initialization");
  }
  require(action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED,
          "Joint trajectory failed or exceeded wall timeout: "+action_->getState().toString());
  auto result=action_->getResult();
  require(result && result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
          "Joint trajectory returned an error");
  stage_ = "settling at joint target";
  waitSettled(true);
}

void JointInitializer::prepareCartesian() {
  stage_ = "prepare Cartesian hold";
  requireState(cartesian_controller_, "stopped");
  requireState(joint_controller_, "running");
  franka_msgs::StartDirectionalExperiment command;
  command.request.target=measuredPose(settled_state_);
  command.request.cbf_active=false;
  // Preserve the user's current energy parameters, but explicitly disable CBF.
  const std::string config="/"+cartesian_controller_+"/dynamic_reconfigure_compliance_param_node/";
  require(root_.getParam(config+"Kmax",command.request.Kmax) &&
          root_.getParam(config+"alpha",command.request.alpha), "CBF configuration parameters unavailable");
  require(root_.serviceClient<franka_msgs::StartDirectionalExperiment>(directional_service_).call(command)
          && command.response.success, "Could not prepare Cartesian hold: "+command.response.message);
  // start_experiment disables CBF and clears abort via its pending explicit restart.
  // starting() captures the measured pose, joints and derivative history at switch time.
}

void JointInitializer::resumeCartesian() {
  stage_ = "resume Cartesian controller";
  switchControllers({cartesian_controller_},{joint_controller_});
}

void JointInitializer::synchronizeReference() {
  stage_ = "synchronize linear target";
  requireState(cartesian_controller_, "running");
  requireState(joint_controller_, "stopped");
  const auto state=nextState(2.0);
  *reference_=measuredPose(state);
  *reference_ready_=true;
}

bool JointInitializer::initialize(const InitializeJointPose::Request& request,
                                  InitializeJointPose::Response& response,
                                  geometry_msgs::Pose& reference, bool& reference_ready) {
  reference_=&reference; reference_ready_=&reference_ready;
  target_=asArray(request.q); duration_=request.duration;
  try {
    performJointInitialization(*this);
    response.success=true;
    response.applied_pose=reference;
    response.message="Joint initialization complete; Cartesian target synchronized to measured pose; CBF disabled";
  } catch (const std::exception& error) {
    if (action_ && !action_->getState().isDone()) action_->cancelGoal();
    if (!reference_ready) {
      // A terminal ABORTED action may leave its trajectory installed. Explicitly
      // replace it with JTC's empty-trajectory hold, even when cancellation is moot.
      // A stopped JTC ignores this command after a completed switch to Cartesian.
      joint_hold_publisher_.publish(trajectory_msgs::JointTrajectory());
    }
    response.success=false;
    response.message=stage_+": "+error.what();
    if (!reference_ready)
      response.message += ". Cartesian experiment commands remain blocked; fix the cause and retry initialize_joint_pose. "
                          "A joint-controller hold was requested. Inspect controller_manager/list_controllers; "
                          "no automatic return to an old target.";
    ROS_ERROR_STREAM(response.message);
  }
  return true;
}
}  // namespace franka_trajectory
