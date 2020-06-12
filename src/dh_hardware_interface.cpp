// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// dh_gripper
#include "dh_gripper/hardware_interface/AG95_hardware_interface.h"


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "dh_hardware_interface");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto tcp_ip = node.param<std::string>("tcp/ip", "192.168.1.29");
  auto tcp_port = node.param<int>("tcp/port", 8888);
  auto tcp_timeout = node.param<double>("tcp/timeout", 5.0);


  ros::AsyncSpinner spinner(2);
  spinner.start();


  // Hardware Interface
  dh::AG95_HW gripper_hw(node);

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&hardware_interface);

  // Loop
  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(hardware_interface.loop_hz);
  while (ros::ok())
  {
    rate.sleep();
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    //ROS_DEBUG_THROTTLE(0, "Period: %fs", period.toSec());

    gripper_hw.read();
    controller_manager.update(time, period);
    gripper_hw.write();

    prev_time = time;
  }

  hardware_interface.close();
  return 0;
}
