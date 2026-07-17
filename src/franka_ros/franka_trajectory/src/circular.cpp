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

// whenever clock signal is received, store value (no need to send new position signal every time a clock message is received)
/*
void clockUpdate(const rosgraph_msgs::Clock::ConstPtr& rostime) {
  time = rostime->clock;
}
*/
}

const double pi = 3.14159265358979323846;
const double rotfrequency = 0.25;

geometry_msgs::PoseStamped trajectory(ros::Time starttime, ros::Time rostime) {
    double t = rostime.toSec();
    // ROS_INFO("ros time %f seconds", t);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.5;
    pose.pose.position.y = .3*sin(2*pi*rotfrequency*t);
    pose.pose.position.z = .7+.3*cos(2*pi*rotfrequency*t);
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
  // create ' clock ' that measures time since start
  // put time into function (with message pointer) that fills message with a time-based position
  // publish this message
  const ros::Time Tstart = ros::Time::now();

  while(ros::ok()){
  	tpclock::time = ros::Time::now();
    pose = trajectory(Tstart, tpclock::time);

    publisher.publish(pose);
    ros::spinOnce();
    rate.sleep();
    // ROS_INFO("sent message");

    // TO DO: SET UP TO USE SIM TIME SOMEHOW (LAUNCH FILE?)
  }
  
  
  // return ros::ok();

  return 0;
}
