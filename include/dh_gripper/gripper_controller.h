#ifndef DH_GRIPPER_CONTROLLER_H
#define DH_GRIPPER_CONTROLLER_H
#include <string>
#include <vector>
#include <mutex>

#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <serial/serial.h>
#include <control_msgs/GripperCommandAction.h>

#include "dh_gripper/definition.h"
#include "dh_gripper/gripper_driver.h"
#include "dh_gripper/GripperState.h"


namespace dh {

class GripperController {
private:
  //connect mode
  int connect_mode;

  /// Serial ports instance
  serial::Serial serial;

  //socket id
  int sockfd;

  /// controllers for the individual servo motors
  std::string hand_port_name_;

  /// DH Hand Model
  std::string gripper_model;

  // Wait data time;
  double WaitDataTime_;


  /// actionlib Server
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;

  std::vector<double> joint_pos;
  std::vector<double> joint_eff;

  std::vector<bool> stalled;
  std::vector<bool> reached_goal;


public:
  GripperController(ros::NodeHandle node, const std::string &name);
  ~GripperController();

  std::mutex R_mutex;
  std::mutex W_mutex;

  DH_Driver driver;

  DH_Robotics::DH_DataStream readtempdata;


  /**
   * @brief Callback for the action.
   *
   * @param goal
   */
  void actuateHandCB(const control_msgs::GripperCommandGoalConstPtr &goal);

  /**
   * @brief Open DH_Hand communication
   *
   * @return true on success; otherwise returns false.
   */
  bool build_conn();

  /**
   * @brief Initialize the Hand
   *
   */
  bool init();

  /**
   * @brief Close DH_Hand communication
   *
   * @return true on success; otherwise returns false.
   */
  void close_device();

  /**
   * @brief Move individual joint
   *
   * @param MotorID
   * @param target_position
   * @return true on success; otherwise returns false.
   */
  bool moveHand(int motor_id, int target_position, bool &feedback);

  /**
   * @brief Get feedback of individual joint
   *
   * @param MotorID
   * @return true on success; otherwise returns false.
   */
  bool getHandFeedback(int motor_id, bool &feedback);

  /**
   * @brief Get the Gripping Position
   *
   * @param motor_id
   * @param position
   * @return true on success; otherwise returns false.
   */
  bool getGrippingPosition(int motor_id, int &position);

  /**
   * @brief Set the Gripping Force
   *
   * @param gripping_force
   * @return true on success; otherwise returns false.
   */
  bool setGrippingForce(int gripping_force);

  /**
   * @brief Get the Gripping Force
   *
   * @param gripping_force
   * @return true on success; otherwise returns false.
   */
  bool getGrippingForce(int &gripping_force);

  /**
   * @brief Write data to serial
   *
   * @param data
   * @return true
   * @return false
   */
  bool write_data(std::vector<uint8_t> data);

  /**
   * @brief read recevied data ,save to tempdatastream
   *
   * @return true on recevied success;
   * @return false on recevied overtime;
   */
  bool read_data(uint8_t waitconunt = 5);

  bool ensure_set_command(std::vector<uint8_t> data);
  bool ensure_get_command(std::vector<uint8_t> data);
  bool ensure_run_end(std::vector<uint8_t> data);
  bool chacke_data(uint8_t *data);

  /**
   * @brief service callback function
   *
   * @param req
   * @param res
   * @return true on command is vaild;otherwise return false
   */
  bool jointValueCB(dh_gripper::GripperState::Request  &req, dh_gripper::GripperState::Response &res);

};

} // namespace
#endif
