#include <cmath>
#include <mutex>
#include <string>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <franka_trajectory/SetLinearCommand.h>
#include <franka_msgs/StartDirectionalExperiment.h>

class LinearTrajectory {
 public:
  LinearTrajectory()
      : private_node_handle_("~") {
    // Same initial target used by hold.cpp.
    target_pose_.pose.position.x = 0.307;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = 0.59;

    target_pose_.pose.orientation.w = 0.0;
    target_pose_.pose.orientation.x = 0.9238795;
    target_pose_.pose.orientation.y = -0.3826834;
    target_pose_.pose.orientation.z = 0.0;

    private_node_handle_.param("publish_rate", publish_rate_, 100.0);

    pose_publisher_ =
        private_node_handle_.advertise<geometry_msgs::PoseStamped>(
            "equilibrium_pose", 20);

    command_service_ =
        private_node_handle_.advertiseService(
            "set_experiment_command",
            &LinearTrajectory::commandCallback,
            this);

    private_node_handle_.param<std::string>(
        "cbf_dynamic_reconfigure_service",
        cbf_dynamic_reconfigure_service_,
        "/cartesian_impedance_cbf_controller/"
        "dynamic_reconfigure_compliance_param_node/set_parameters");

    dynamic_reconfigure_client_ =
        root_node_handle_.serviceClient<dynamic_reconfigure::Reconfigure>(
            cbf_dynamic_reconfigure_service_);

    private_node_handle_.param<std::string>("directional_experiment_service",
                                           directional_experiment_service_, "");
    if (!directional_experiment_service_.empty()) {
      directional_experiment_client_ = root_node_handle_.serviceClient<
          franka_msgs::StartDirectionalExperiment>(directional_experiment_service_);
    }

    private_node_handle_.param<std::string>("recording_start_service",
                                           recording_start_service_, "");
    if (!recording_start_service_.empty()) {
      recording_client_ = root_node_handle_.serviceClient<std_srvs::Trigger>(
          recording_start_service_);
    }

    ROS_INFO_STREAM(
        "Linear trajectory ready. Command service: "
        << private_node_handle_.resolveName("set_experiment_command"));

    ROS_INFO_STREAM(
        "CBF dynamic-reconfigure service: "
        << cbf_dynamic_reconfigure_service_);
  }

  void run() {
    ros::Rate rate(publish_rate_);

    while (ros::ok()) {
      geometry_msgs::PoseStamped pose_to_publish;

      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_pose_.header.stamp = ros::Time::now();
        target_pose_.header.frame_id = "fr3_link0";
        pose_to_publish = target_pose_;
      }

      pose_publisher_.publish(pose_to_publish);

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  bool commandCallback(
      franka_trajectory::SetLinearCommand::Request& request,
      franka_trajectory::SetLinearCommand::Response& response) {
    if (!std::isfinite(request.x_move) ||
        !std::isfinite(request.Kmax) ||
        !std::isfinite(request.alpha)) {
      response.success = false;
      response.message = "Request contains a non-finite value.";
      return true;
    }

    if (request.Kmax <= 0.0) {
      response.success = false;
      response.message = "Kmax must be strictly positive.";
      return true;
    }

    if (request.alpha <= 0.0) {
      response.success = false;
      response.message = "alpha must be strictly positive.";
      return true;
    }

    // The recorder is already subscribed. Wait for its synchronous ACK before
    // forwarding any motion; subsequent calls reuse the same recording session.
    if (!recording_start_service_.empty()) {
      std_srvs::Trigger recording;
      if (!recording_client_.call(recording) || !recording.response.success) {
        response.success = false;
        response.message = "Experiment not sent: recorder unavailable or not ready. " +
                           recording.response.message;
        ROS_ERROR_STREAM(response.message);
        return true;
      }
    }

    // The directional controller receives the target and parameters atomically.
    // Its abort latch is cleared only by this explicit experiment request.
    if (!directional_experiment_service_.empty()) {
      franka_msgs::StartDirectionalExperiment command;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        command.request.target = target_pose_.pose;
      }
      command.request.target.position.x += request.x_move;
      command.request.cbf_active = request.cbf_active;
      command.request.Kmax = request.Kmax;
      command.request.alpha = request.alpha;
      if (!directional_experiment_client_.call(command)) {
        response.success = false;
        response.message = "Directional experiment service unavailable; target unchanged";
        return true;
      }
      response.success = command.response.success;
      response.message = command.response.message;
      if (response.success) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_pose_.pose = command.request.target;
        response.applied_pose = target_pose_.pose;
      }
      return true;
    }

    // Legacy total-energy controller: update parameters through dynamic reconfigure.
    dynamic_reconfigure::Reconfigure reconfigure_request;

    dynamic_reconfigure::BoolParameter cbf_active_parameter;
    cbf_active_parameter.name = "cbf_active";
    cbf_active_parameter.value = request.cbf_active;

    dynamic_reconfigure::DoubleParameter kmax_parameter;
    kmax_parameter.name = "Kmax";
    kmax_parameter.value = request.Kmax;

    dynamic_reconfigure::DoubleParameter alpha_parameter;
    alpha_parameter.name = "alpha";
    alpha_parameter.value = request.alpha;

    reconfigure_request.request.config.bools.push_back(
        cbf_active_parameter);

    reconfigure_request.request.config.doubles.push_back(
        kmax_parameter);

    reconfigure_request.request.config.doubles.push_back(
        alpha_parameter);

    if (!dynamic_reconfigure_client_.call(reconfigure_request)) {
      response.success = false;
      response.message =
          "Failed to update the controller dynamic-reconfigure service.";
      return true;
    }

    // Only update the pose after the controller accepted its parameters.
    {
      std::lock_guard<std::mutex> lock(target_mutex_);

      target_pose_.pose.position.x += request.x_move;
      target_pose_.header.stamp = ros::Time::now();

      response.applied_pose = target_pose_.pose;
    }

    response.success = true;
    response.message = "Pose and CBF parameters updated.";

    ROS_INFO_STREAM(
        "Experiment command applied:"
        << " x_move=" << request.x_move
        << ", target_x=" << response.applied_pose.position.x
        << ", cbf_active=" << std::boolalpha << request.cbf_active
        << ", Kmax=" << request.Kmax
        << ", alpha=" << request.alpha);

    return true;
  }

  ros::NodeHandle root_node_handle_;
  ros::NodeHandle private_node_handle_;

  ros::Publisher pose_publisher_;
  ros::ServiceServer command_service_;
  ros::ServiceClient dynamic_reconfigure_client_;
  ros::ServiceClient directional_experiment_client_;
  ros::ServiceClient recording_client_;
  std::string recording_start_service_;
  std::string directional_experiment_service_;

  std::mutex target_mutex_;
  geometry_msgs::PoseStamped target_pose_;

  double publish_rate_{100.0};
  std::string cbf_dynamic_reconfigure_service_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_publisher");

  LinearTrajectory trajectory;
  trajectory.run();

  return 0;
}