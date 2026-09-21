// Forward Gazebo Classic contacts, including empty frames, to the ROS recorder.
#include <cstdint>
#include <string>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ContactsState.h>
#include <ros/ros.h>

class ContactDiagnosticsBridge {
 public:
  ContactDiagnosticsBridge() : private_nh_("~") {
    std::string world;
    private_nh_.param<std::string>("world", world, "default");
    publisher_ = nh_.advertise<gazebo_msgs::ContactsState>("/directional_cbf/contacts", 1000);
    node_.reset(new gazebo::transport::Node());
    node_->Init(world);
    subscriber_ = node_->Subscribe("~/physics/contacts",
                                   &ContactDiagnosticsBridge::receive, this);
    ROS_INFO_STREAM("Contact diagnostics: /gazebo/" << world
                    << "/physics/contacts -> /directional_cbf/contacts");
  }

 private:
  void receive(const boost::shared_ptr<const gazebo::msgs::Contacts>& message) {
    gazebo_msgs::ContactsState output;
    output.header.seq = sequence_++;
    output.header.stamp = ros::Time(message->time().sec(), message->time().nsec());
    // Positions/normals and wrenches are copied from Gazebo without a transform.
    output.header.frame_id = "world";
    for (int i = 0; i < message->contact_size(); ++i) {
      const auto& contact = message->contact(i);
      gazebo_msgs::ContactState state;
      state.collision1_name = contact.collision1();
      state.collision2_name = contact.collision2();
      for (int j = 0; j < contact.position_size(); ++j) {
        geometry_msgs::Vector3 value;
        value.x = contact.position(j).x();
        value.y = contact.position(j).y();
        value.z = contact.position(j).z();
        state.contact_positions.push_back(value);
      }
      for (int j = 0; j < contact.normal_size(); ++j) {
        geometry_msgs::Vector3 value;
        value.x = contact.normal(j).x();
        value.y = contact.normal(j).y();
        value.z = contact.normal(j).z();
        state.contact_normals.push_back(value);
      }
      for (int j = 0; j < contact.depth_size(); ++j) {
        state.depths.push_back(contact.depth(j));
      }
      for (int j = 0; j < contact.wrench_size(); ++j) {
        const auto& source = contact.wrench(j).body_1_wrench();
        geometry_msgs::Wrench wrench;
        wrench.force.x = source.force().x();
        wrench.force.y = source.force().y();
        wrench.force.z = source.force().z();
        wrench.torque.x = source.torque().x();
        wrench.torque.y = source.torque().y();
        wrench.torque.z = source.torque().z();
        state.wrenches.push_back(wrench);
        state.total_wrench.force.x += wrench.force.x;
        state.total_wrench.force.y += wrench.force.y;
        state.total_wrench.force.z += wrench.force.z;
        state.total_wrench.torque.x += wrench.torque.x;
        state.total_wrench.torque.y += wrench.torque.y;
        state.total_wrench.torque.z += wrench.torque.z;
      }
      output.states.push_back(state);
    }
    // Empty messages distinguish 'no contacts' from a missing contact stream.
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  gazebo::transport::NodePtr node_;
  gazebo::transport::SubscriberPtr subscriber_;
  uint32_t sequence_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "contact_diagnostics_bridge");
  if (!gazebo::client::setup()) {
    ROS_FATAL("Could not initialize Gazebo contact transport");
    return 1;
  }
  {
    ContactDiagnosticsBridge bridge;
    ros::spin();
  }
  gazebo::client::shutdown();
  return 0;
}
