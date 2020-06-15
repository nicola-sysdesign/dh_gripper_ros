#ifndef DH_GRIPPER_CONTROLLER_H
#define DH_GRIPPER_CONTROLLER_H
#include <string>
#include <vector>
#include <mutex>

#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// actionlib
#include <actionlib/server/simple_action_server.h>
// sensor_msgs
#include <sensor_msgs/JointState.h>
// control_msgs
#include <control_msgs/GripperCommandAction.h>
// serial
#include <serial/serial.h>

// Boost
#include <boost/bind.hpp>

#include "dh_gripper/definition.h"
#include "dh_gripper/dh_driver.h"


namespace dh {

const double MIN_POSITION_LIMIT = 0, MAX_POSITION_LIMIT = 100;
const double MIN_EFFORT_LIMIT = 20, MAX_EFFORT_LIMIT = 100;

class GripperController {
private:

  ros::NodeHandle node;

  //
  serial::Serial serial;
  //
  int sockfd;

  /// DH Hand Model
  std::string gripper_model;

  //
  int connect_mode = 0;
  //
  std::string usb_port;
  //
  std::string tcp_ip; int tcp_port;
  //
  ros::Duration data_timeout;

  //
  ros::Timer timer;

  // Topics
  ros::Publisher joint_state_pub;

  // Action Server
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_command_asrv;


  std::vector<double> joint_pos;
  std::vector<double> joint_eff;
  std::vector<bool> stalled;
  std::vector<bool> reached_goal;


  DH_Driver driver;

  std::mutex R_mutex;
  std::mutex W_mutex;

  DH_Robotics::DH_DataStream readtempdata;

public:

  GripperController(const ros::NodeHandle &node = ros::NodeHandle(), const std::string &action_ns = "gripper_command");

  ~GripperController();


  bool init();


  bool start();


  /**
   * @brief service callback function
   *
   * @param req
   * @param res
   * @return true on command is vaild;otherwise return false
   */
  void timer_cb(const ros::TimerEvent &ev);


  /**
   * @brief Callback for the action.
   *
   * @param goal
   */
  void execute_cb(const control_msgs::GripperCommandGoalConstPtr &goal);

  /**
   * @brief Open DH_Hand communication
   *
   * @return true on success; otherwise returns false.
   */
  bool usb_connect(const std::string &usb);

  bool tcp_connect(const std::string &ip, const int port);

  /**
   * @brief Initialize the Hand
   *
   */
  bool init_device();

  /**
   * @brief Close DH_Hand communication
   *
   * @return true on success; otherwise returns false.
   */
  void close_device();

  /**
   * @brief Move individual joint
   *
   * @param motor_id
   * @param target_position
   * @param feedback
   * @return true on success; otherwise returns false.
   */
  bool actuate_gripper(int motor_id, int target_position, bool &feedback);

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
  bool read_data(uint8_t wait_count = 5);

  bool check_data(uint8_t *data);

  bool ensure_set_command(std::vector<uint8_t> data);
  bool ensure_get_command(std::vector<uint8_t> data);

  bool ensure_run_end(std::vector<uint8_t> data);

};

} // namespace
#endif
