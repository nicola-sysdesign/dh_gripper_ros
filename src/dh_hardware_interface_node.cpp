// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller_manager
#include <controller_manager/controller_manager.h>

// dh_hardware_interface
#include "dh_gripper/hardware_interface/dh_hardware_interface.h"


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "dh_hardware_interface");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto tcp_host = node.param<std::string>("tcp/host", "192.168.1.29");
  auto tcp_port = node.param<int>("tcp/port", 8888);
  auto tcp_timeout = node.param<double>("tcp/timeout", 5.0);

  double loop_hz;
  if (!node.getParam("hardware_interface/loop_hz", loop_hz))
  {
    std::string param_name = node.resolveName("hardware_interface/loop_hz");
    ROS_ERROR("Failed to retrieve '%s' parameter.", param_name.c_str());
  }

  std::vector<std::string> joints;
  if (!node.getParam("hardware_interface/joints", joints))
  {
    std::string param_name = node.resolveName("hardware_interface/joints");
    ROS_ERROR("Failed to retrieve '%s' parameter.", param_name.c_str());
  }


  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Hardware Interface
  dh::GripperHW gripper_hw(node);
  if (!gripper_hw.init(joints))
  {
    ROS_FATAL("Failed to initialize Hardware Interface!");
    return 1;
  }

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&gripper_hw, node);
  if (!gripper_hw.start(tcp_host, tcp_port))
  {
    ROS_FATAL("Failed to start Hardware Interface!");
    return 1;
  }

  // Loop
  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(loop_hz);
  while (ros::ok())
  {
    rate.sleep();
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    //ROS_DEBUG_THROTTLE(0, "Period: %fs", period.toSec());

    gripper_hw.read(time, period);
    controller_manager.update(time, period);
    gripper_hw.write(time, period);

    prev_time = time;
  }


  if (!gripper_hw.stop())
  {
    ROS_FATAL("Failed to stop Hardware Interface!");
    return 1;
  }

  return 0;
}
