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

  dh::GripperController controller(node, "gripper_command");

  ros::spin();

  return 0;
}
