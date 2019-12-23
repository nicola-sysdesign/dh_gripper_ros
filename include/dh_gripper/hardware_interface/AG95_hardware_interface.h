#ifndef AG95_HARDWARE_INTERFACE_H
#define AG95_HARDWARE_INTERFACE_H
// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// hardware_interface
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// dh_hand_driver
#include <dh_hand_driver/hand_controller.h>
// Boost
#include <boost/shared_ptr.hpp>


namespace dh {

class AG95_HardwareInterface : public hardware_interface::RobotHW {
protected:
  ros::NodeHandle node;
  boost::shared_ptr<HandController> hand_controller;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface pos_joint_interface;
  hardware_interface::EffortJointInterface eff_joint_interface;

  std::vector<double> joint_pos, joint_pos_cmd;
  std::vector<double> joint_vel, joint_vel_cmd;
  std::vector<double> joint_eff, joint_eff_cmd;

  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

public:
  double loop_hz;
  std::vector<std::string> joints;


  AG95_HardwareInterface(ros::NodeHandle &node) : node(node) {

    if (!node.getParam("/AG95/hardware_interface/loop_hz", loop_hz))
      ROS_WARN("Hardware Interface: 'loop_hz' parameter not defined.");

    if (!node.getParam("/AG95/hardware_interface/joints", joints))
      ROS_WARN("Hardware Interface: 'joints' parameter not defined.");

    int n_joints = joints.size();

    joint_pos.resize(n_joints, 0);
    joint_vel.resize(n_joints, 0);
    joint_eff.resize(n_joints, 0);

    joint_pos_cmd.resize(n_joints, 0);
    joint_vel_cmd.resize(n_joints, 0);
    joint_eff_cmd.resize(n_joints, 0);

    for (int i=0; i < n_joints; i++) {
      hardware_interface::JointStateHandle joint_state_handle(joints[i], &joint_pos[i], &joint_vel[i], &joint_eff[i]);
      joint_state_interface.registerHandle(joint_state_handle);

      hardware_interface::JointHandle pos_joint_handle(joint_state_handle, &joint_pos_cmd[i]);
      pos_joint_interface.registerHandle(pos_joint_handle);

      hardware_interface::JointHandle eff_joint_handle(joint_state_handle, &joint_eff_cmd[i]);
      eff_joint_interface.registerHandle(eff_joint_handle);
    }

    registerInterface(&joint_state_interface);
    registerInterface(&pos_joint_interface);
    registerInterface(&eff_joint_interface);
  }


  bool init() {

    // Hand Controller
    hand_controller.reset(new HandController(node, "AG95"));

    if (!hand_controller->initHand()) {
      ROS_ERROR("Hardware Interface: failed to initialize gripper.");
    }

    return true;
  }


  void read() {

  }

  bool position_reached = true;
  void write() {

    if (joint_pos[0] != joint_pos_cmd[0] && position_reached == true) {

      if (!hand_controller->setGrippingForce(90.0)) {
        ROS_ERROR("Failed to set gripper force.");
      }

      if (!hand_controller->moveHand(1, joint_pos_cmd[0], position_reached)) {
        ROS_ERROR("Failed to move the gripper.");
      }
    }
  }


  void close() {
    hand_controller->closeDevice();
    ROS_INFO("Connection closed.");
  }

};

}  // namespace
#endif
