#include <ros/ros.h>
#include <dh_hand_driver/hand_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_controller");
  ros::NodeHandle node("~");
  HandController controller(node, "gripper_command");
  ros::spin();

  return 0;
}
