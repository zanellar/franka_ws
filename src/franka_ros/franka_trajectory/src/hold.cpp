// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>

using namespace std;

namespace tpclock{
// put in namespace to avoid conflicting 'time' variable

ros::Time time;
}

geometry_msgs::PoseStamped trajectory(ros::Time starttime, ros::Time rostime) {
    double t = (rostime-starttime).toSec();
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.307;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.59;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.x = 0.9238795;
    pose.pose.orientation.y = -0.3826834;
    pose.pose.orientation.z = 0;
    pose.header.stamp = rostime;

    return pose;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle node_handle("~");

  // (gazebo) clock subscriber (always keep most recent time signal)
  //ros::Subscriber clk = node_handle.subscribe("/clock", 0, tpclock::clockUpdate);

  // publisher setup
  double publish_rate;
  node_handle.getParam("publish_rate", publish_rate);
  ros::Rate rate(publish_rate);

  geometry_msgs::PoseStamped pose;

  ros::Publisher publisher = node_handle.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 20);
  // TO DO LATER: ADD TRAJECTORY FROM INITIAL POSITION TO DESIRED POSITION

  const ros::Time Tstart = ros::Time::now();

  while(ros::ok()){
  	tpclock::time = ros::Time::now();
    pose = trajectory(Tstart, tpclock::time);

    publisher.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
