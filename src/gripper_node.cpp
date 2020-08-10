#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// dh_gripper
#include "dh_controller/gripper_controller.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dh_controller");

  // Node
  ros::NodeHandle node("~");

  // Parameters
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
  dh::GripperController controller(node, "gripper_cmd");

  if (!controller.init(gripper_model, joints))
  {
    ROS_FATAL("Failed to initialize Gripper Controller!");
    return 1;
  }

  if (!controller.start())
  {
    ROS_FATAL("Failed to start Gripper Controller!");
    return 1;
  }

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();

  controller.shutdown();
  return 0;
}
