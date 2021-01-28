#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// sensor_msgs
#include <sensor_msgs/JointState.h>
// industrial_msgs
#include <industrial_msgs/TriState.h>
#include <industrial_msgs/RobotStatus.h>
// dh_controller
#include "dh_controller/gripper_controller.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dh_controller");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto freq = node.param<double>("publish_frequency", 10);


  std::string gripper_model;
  if (!node.getParam("gripper/model", gripper_model))
  {
    std::string param_name = node.resolveName("gripper/model");
    ROS_ERROR("Failed to get '%s'", param_name.c_str());
    return 1;
  }

  std::vector<std::string> joints;
  if (!node.getParam("gripper/joints", joints))
  {
    std::string param_name = node.resolveName("gripper/joints");
    ROS_ERROR("Failed to get '%s'", param_name.c_str());
    return 1;
  }

  //
  ros::AsyncSpinner spinner(1);
  spinner.start();

  dh::GripperController controller(node, "gripper_cmd");

  // Service Servers
  auto start_srv = node.advertiseService("start", &dh::GripperController::start, &controller);
  auto close_srv = node.advertiseService("close", &dh::GripperController::close_device, &controller);

  // Published Topics
  // auto joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 10, true);
  auto status_pub = node.advertise<industrial_msgs::RobotStatus>("status", 10);

  if (!controller.init(gripper_model, joints))
  {
    ROS_FATAL("Failed to initialize Gripper Controller!");
    return 1;
  }

  if (!controller.start())
  {
    ROS_FATAL("Failed to start Gripper Controller!");
  }

  if (controller.setObjectDroppedFeedback(false))
  {
    ROS_INFO("Disabled 'Object Dropped Feedback' function.");
  }

  ros::Rate rate(freq);
  while (ros::ok())
  {
    rate.sleep();
    const ros::Time now = ros::Time::now();

    industrial_msgs::RobotStatus status_msg;
    status_msg.header.stamp = now;
    status_msg.mode.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.e_stopped.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.drives_powered.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.motion_possible.val = (controller.status.initialized) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
    status_msg.in_error.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.error_code = 0;
    status_pub.publish(status_msg);
  }

  // ros::MultiThreadedSpinner spinner(1);
  // spinner.spin();

  controller.shutdown();
  return 0;
}
