// roxcpp
#include <ros/ros.h>
// dh_gripper
#include "dh_gripper/gripper_controller.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hand_controller");

  // Node
  ros::NodeHandle node("~");

  dh::GripperController controller(node, "gripper_command");

  ros::spin();

  return 0;
}
