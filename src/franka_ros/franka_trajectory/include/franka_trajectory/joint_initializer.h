#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager_msgs/ControllerState.h>
#include <franka_msgs/FrankaState.h>
#include <franka_trajectory/InitializeJointPose.h>
#include <franka_trajectory/initialization_policy.h>
#include <geometry_msgs/Pose.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace franka_trajectory {
class JointInitializer {
 public:
  JointInitializer(ros::NodeHandle root, ros::NodeHandle private_node,
                   const std::string& directional_service);
  bool initialize(const InitializeJointPose::Request& request,
                  InitializeJointPose::Response& response,
                  geometry_msgs::Pose& reference, bool& reference_ready);
  bool cartesianRunning();

  // Operations used by the tested transaction policy.
  void preflight();
  void invalidateReference();
  void acquireJointController();
  void moveAndSettle();
  void prepareCartesian();
  void resumeCartesian();
  void synchronizeReference();

 private:
  using ActionClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  std::vector<controller_manager_msgs::ControllerState> controllers();
  void requireState(const std::string& name, const std::string& state);
  void switchControllers(const std::vector<std::string>& start,
                         const std::vector<std::string>& stop);
  franka_msgs::FrankaState nextState(double wall_timeout);
  void waitSettled(bool check_target);
  geometry_msgs::Pose measuredPose(const franka_msgs::FrankaState& state) const;

  ros::NodeHandle root_, private_node_;
  ros::CallbackQueue state_queue_;
  ros::NodeHandle state_node_;
  ros::Subscriber state_subscriber_;
  ros::Publisher joint_hold_publisher_;
  franka_msgs::FrankaState state_;
  uint64_t state_sequence_{0};
  ros::Time previous_stamp_;
  bool have_stamp_{false};
  std::string directional_service_, cartesian_controller_, joint_controller_, manager_;
  std::string arm_id_, state_topic_, stage_;
  std::vector<std::string> joint_names_;
  std::array<JointBounds, 7> bounds_{};
  std::array<double, 7> target_{};
  franka_msgs::FrankaState start_state_, settled_state_;
  geometry_msgs::Pose* reference_{nullptr};
  bool* reference_ready_{nullptr};
  double duration_{0.0}, margin_, velocity_scale_, velocity_cap_;
  double position_tolerance_, velocity_tolerance_, settle_time_, wall_timeout_;
  bool enabled_{false};
  bool real_robot_{false};
  std::string joint_controller_type_;
  double acceleration_cap_{0.5}, jerk_cap_{1.0};
  double requiredDuration() const;
  std::unique_ptr<ActionClient> action_;
};
}  // namespace franka_trajectory
