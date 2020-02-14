// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// dh_gripper
#include "dh_gripper/gripper_controller.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dh_controller");

  // Node
  ros::NodeHandle node("~");

  //
  dh::GripperController controller(node, "gripper_command");

  if (!controller.init())
  {
    ROS_FATAL("Failed to initialize Gripper Controller!");
    return 1;
  }

  if (!controller.start())
  {
    ROS_FATAL("Failed to start Gripper Controller!");
    return 1;
  }

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
