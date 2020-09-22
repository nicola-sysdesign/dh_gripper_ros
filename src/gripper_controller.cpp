/**
This is the control interface for the DH Hand
Author:   Jie Sun
Email:    jie.sun@dh-robotics.com
Date:     2019 July 1

Version 2.0
Copyright @ DH-Robotics Ltd.
**/

#include "dh_controller/gripper_controller.h"


dh::GripperController::GripperController(const ros::NodeHandle &node, const std::string &action_ns) :
  node(node),
  data_timeout(0.05),
  gripper_command_asrv(node, action_ns, boost::bind(&dh::GripperController::execute_cb, this, _1), false)
{

}


dh::GripperController::~GripperController()
{
  close_device();
}


bool dh::GripperController::init(const std::string &gripper_model, const std::vector<std::string> &joints)
{
  this->gripper_model = gripper_model;
  this->joints = joints;

  const int n_joints = joints.size();

  j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
  j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

  joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 10, true);

  ROS_INFO("DH %s Controller initialized successfully.", gripper_model.c_str());
  return true;
}


bool dh::GripperController::start()
{
  if (node.getParam("usb/port", usb_port))
  {
    connect_mode = 1;

    if (!usb_connect(usb_port))
    {
      ROS_FATAL("Failed to connect to %s", usb_port.c_str());
      return false;
    }
  }
  else
  {
    connect_mode = 2;

    auto host = node.param<std::string>("tcp/host", "192.168.1.29");
    auto port = node.param<int>("tcp/port", 8888);

    if (!tcp_connect(host, port))
    {
      ROS_FATAL("Failed to connect to %s:%d", host.c_str(), port);
      return false;
    }
  }

  if (!init_device())
  {
    ROS_FATAL("Failed to initialize DH %s", gripper_model.c_str());
    return false;
  }

  //
  int motor_id = 1;

  int pos;
  int eff;

  getGrippingPosition(motor_id, pos);
  getGrippingForce(eff);

  for (int i = 0; i < joints.size(); i++)
  {
    j_pos[i] = -0.03 * (pos / 100.0);
    j_eff[i] = eff;
  }

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = joints;
  joint_state.position = j_pos;
  joint_state.effort = j_eff;
  joint_state_pub.publish(joint_state);

  //
  gripper_command_asrv.start();

  ROS_INFO("DH %s Controller started successfully.", gripper_model.c_str());
  return true;
}


void dh::GripperController::shutdown()
{
  gripper_command_asrv.shutdown();
  close_device();
}


void dh::GripperController::timer_cb(const ros::TimerEvent &ev)
{
  int motor_id = 1;

  int pos;
  int eff;

  if (!getGrippingPosition(motor_id, pos))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to get gripping position.");
    return;
  }

  if(!getGrippingForce(eff))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to get max gripping force.");
    return;
  }

  double j_pos = (-0.03 + 0.03 * (pos / 100.0));
  double j_eff = eff;

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(1);
  joint_state.name[0] = "ag95_finger_joint";
  joint_state.position.resize(1);
  joint_state.position[0] = j_pos;
  joint_state.effort.resize(1);
  joint_state.effort[0] = j_eff;
  joint_state_pub.publish(joint_state);
}


void dh::GripperController::execute_cb(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  int motor_id = 1;

  int pos; double pos_cmd;
  int eff; double eff_cmd;

  pos_cmd = goal->command.position;
  eff_cmd = goal->command.max_effort;

  getGrippingPosition(motor_id, pos);
  getGrippingForce(eff);


  control_msgs::GripperCommandFeedback feedback;
  feedback.position = pos;
  feedback.effort = eff;
  feedback.stalled = false;
  feedback.reached_goal = false;

  control_msgs::GripperCommandResult result;
  result.position = pos;
  result.effort = eff;
  result.stalled = false;
  result.reached_goal = false;


  if (pos_cmd < dh::MIN_POSITION_LIMIT || dh::MAX_POSITION_LIMIT < pos_cmd)
  {
    ROS_ERROR("Gripper Command 'position' = %.1f, out of range.", pos_cmd);
    gripper_command_asrv.setAborted(result);
    return;
  }

  if (eff_cmd < dh::MIN_EFFORT_LIMIT || eff_cmd > dh::MAX_EFFORT_LIMIT)
  {
    ROS_ERROR("Gripper Command 'max_effort' = %.1f, out of range.", eff_cmd);
    gripper_command_asrv.setAborted(result);
    return;
  }

  //
  setGrippingForce(eff_cmd);

  bool not_stalled;
  if (!actuate_gripper(motor_id, pos_cmd, not_stalled))
  {
    ROS_WARN("Failed to actuate the Gripper!");
    control_msgs::GripperCommandResult result;
    gripper_command_asrv.setAborted(result);
    return;
  }

  getGrippingPosition(motor_id, pos);
  getGrippingForce(eff);

  feedback.position = pos;
  feedback.effort = eff;
  feedback.stalled = !not_stalled;
  feedback.reached_goal = (pos == pos_cmd) ? true : false;
  gripper_command_asrv.publishFeedback(feedback);

  // wait
  data_timeout.sleep();


  getGrippingPosition(motor_id, pos);
  getGrippingForce(eff);

  result.position = pos;
  result.effort = eff;
  result.stalled = !not_stalled;
  result.reached_goal = (pos == pos_cmd) ? true : false;
  gripper_command_asrv.setSucceeded(result);


  // joint state
  for (int i = 0; i < joints.size(); i++)
  {
    j_pos[i] = -0.03 * (pos / 100.0);
    j_eff[i] = eff;
  }

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = joints;
  joint_state.position = j_pos;
  joint_state.effort = j_eff;
  joint_state_pub.publish(joint_state);
}


bool dh::GripperController::usb_connect(const std::string &usb_port)
{
  try
  {
    serial.setPort(usb_port.c_str());
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial.setTimeout(timeout);
    serial.open();
  }
  catch (serial::IOException &ex)
  {
    return false;
  }

  if (!serial.isOpen())
  {
    ROS_ERROR("Failed to open USB port: %s", usb_port.c_str());
    return false;
  }

  ROS_INFO("Connected to USB: %s", usb_port.c_str());
  return true;
}


bool dh::GripperController::tcp_connect(const std::string &ip, const int port)
{
  // socket
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
  {
    ROS_ERROR("Socket failed!");
    return false;
  }

  ROS_DEBUG("Socket ID: %d", sockfd);

  // sockaddr_in
  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);

  inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);
  bzero(&(serv_addr.sin_zero), 8);

  // connect
  if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
  {
    ROS_ERROR("Failed to connect to %s:%d", ip.c_str(), port);
    return false;
  }

  ROS_INFO("Connected to %s:%d", ip.c_str(), port);
  return true;
}


bool dh::GripperController::init_device()
{
  auto ret = false;
  auto iter = 0;
  do
  {
    driver.setInitialize();

    if (!write_data(driver.getStream()))
    {
      ROS_WARN("setInitialize write data error");
      continue;
    }

    if (!ensure_set_command(driver.getStream()))
    {
      ROS_WARN("setInitialize wait response timeout");
      continue;
    }

    auto wait_iter = 0;
    do
    {
      data_timeout.sleep();

      if (!ensure_get_command(driver.getStream()))
      {
        continue;
      }

      if (readtempdata.data[0] == 0x01)
      {
        ret = true;
      }

    } while (ret == false && wait_iter++ < 20);

  } while (ret == false && iter++ < 2);

  return ret;
}


void dh::GripperController::close_device()
{
  if (connect_mode == 1)
  {
    serial.close();
  }
  else if (connect_mode == 2)
  {
    close(sockfd);
  }
}


bool dh::GripperController::actuate_gripper(int motor_id, int target_position, bool &feedback)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    driver.setMotorPosition(motor_id, target_position);

    if (!write_data(driver.getStream()))
    {
      ROS_WARN("setMotorPosition write data error");
      continue;
    }

    if (!ensure_set_command(driver.getStream()))
    {
      ROS_WARN("setMotorPosition wait response timeout");
      continue;
    }

    driver.getFeedback(motor_id);

    auto wait_iter = 0;
    do
    {
      if (!write_data(driver.getStream()))
      {
        ROS_WARN("getFeedback wait response timeout");
        continue;
      }

      if (!ensure_run_end(driver.getStream()))
      {
        ROS_WARN("getFeedback ensure_run_end wait response timeout");
        continue;
      }

      switch (readtempdata.data[0])
      {
        case DH_Robotics::FeedbackType::ARRIVED:
          feedback = true;
          ret = true;
          break;

        case DH_Robotics::FeedbackType::CATCHED:
          feedback = false;
          ret = true;
          break;
      }

    } while (ret == false && wait_iter++ < 100);

  } while (ret == false && iter++ < 3);

  return ret;
}


bool dh::GripperController::getHandFeedback(int motor_id, bool &feedback)
{
  auto ret = false;
  auto iter = 0;
  driver.getFeedback(motor_id);
  do
  {
    if (!write_data(driver.getStream()))
    {
      ROS_WARN("getFeedback wait response timeout");
      continue;
    }

    if (!ensure_run_end(driver.getStream()))
    {
      ROS_WARN("getFeedback ensure_run_end wait response timeout");
      continue;
    }

    switch (readtempdata.data[0])
    {
      case DH_Robotics::FeedbackType::ARRIVED:
        feedback = true;
        ret = true;
        break;

      case DH_Robotics::FeedbackType::CATCHED:
        feedback = false;
        ret = true;
        break;
    }

  } while (ret == false && iter++ < 100);

  return ret;
}


bool dh::GripperController::getGrippingPosition(int motor_id, int &position)
{
  auto ret = false;
  auto iter = 0;
  driver.getMotorPosition(motor_id);
  do
  {
    if (!write_data(driver.getStream()))
    {
      ROS_WARN("getMotorPosition write data error");
      continue;
    }

    if (!ensure_get_command(driver.getStream()))
    {
      ROS_WARN("getMotorPosition ensure_get_command wait response timeout");
      continue;
    }

    position = readtempdata.data[0];
    ret = true;

  } while (ret == false && iter++ < 3);

  return ret;
}


bool dh::GripperController::setGrippingForce(int gripping_force)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    driver.setMotorForce(gripping_force);

    if (!write_data(driver.getStream()))
    {
      ROS_WARN("setMotorForce write data error");
      continue;
    }

    if (!ensure_set_command(driver.getStream()))
    {
      ROS_WARN("setMotorForce wait timeout");
      continue;
    }

    ret = true;

  } while (ret == false && iter++ < 3);

  return ret;
}


bool dh::GripperController::getGrippingForce(int &gripping_force)
{
  auto ret = false;
  auto iter = 0;
  driver.getMotorForce();
  do
  {
    if (!write_data(driver.getStream()))
    {
      ROS_WARN("getMotorForce write data error");
      continue;
    }

    if (!ensure_get_command(driver.getStream()))
    {
      ROS_WARN("getMotorForce wait timeout");
      continue;
    }

    gripping_force = readtempdata.data[0];
    ret = true;

  } while (ret == false && iter++ < 3);

  return ret;
}


bool dh::GripperController::setObjectDroppedFeedback(bool enabled)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    driver.setObjectDroppedFeedback(enabled);

    if (!write_data(driver.getStream()))
    {
      ROS_WARN("setObjectDroppedFeedback write data error");
      continue;
    }

    if (!ensure_set_command(driver.getStream()))
    {
      ROS_WARN("setObjectDroppedFeedback wait timeout");
      continue;
    }

    ret = true;

  } while (ret == false && iter++ < 3);

  return ret;
}


bool dh::GripperController::read_data(uint8_t wait_count)
{
  auto ret = false;
  auto iter = 0;
  auto getframe = false;
  uint8_t buf[14];

  do
  {
    data_timeout.sleep();

    if (connect_mode == 1)
    {
      uint8_t count = serial.available() / 14;
      uint8_t remain = serial.available() % 14;
      if (count >= 1 && remain == 0)
      {
        for (; count > 1; count--)
        {
          serial.read(buf, 14);
        }
        serial.read(buf, 14);
        readtempdata.DatafromStream(buf, 14);
        getframe = true;
      }
    }
    else if (connect_mode == 2)
    {
      std::vector<uint8_t> temp;
      unsigned char tempbuf[140] = { 0 };

      int get_num = recv(sockfd, tempbuf, 140, MSG_DONTWAIT);
      for (int i = 0; i < get_num; i++)
      {
        temp.push_back(tempbuf[i]);
      }

      uint8_t count = temp.size() / 14;
      uint8_t remain = temp.size() % 14;
      if (count >= 1 && remain == 0)
      {
        for (int i = 0; i < (count - 1) * 14; i++)
        {
          temp.erase(temp.begin());
        }
        for (int i = 0; i < 14; i++)
        {
          buf[i] = temp.at(0);
          temp.erase(temp.begin());
        }
        readtempdata.DatafromStream(buf, 14);
        getframe = true;
      }
    }

    if (getframe)
    {
      if (check_data(buf))
      {
        ROS_DEBUG("Read: %X %X %X %X %X %X %X %X %X %X %X %X %X %X", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]);
        ret = true;
      }
    }
  } while (ret == false && iter++ < wait_count);

  // if (iter >= waitconunt)
  //   ROS_ERROR_STREAM("Read Overtime you can increase 'WaitDataTime' in launch file ");

  return ret;
}


bool dh::GripperController::write_data(std::vector<uint8_t> data)
{
  auto ret = false;

  if (connect_mode == 1)
  {
    if (serial.write(data) == data.size())
    {
      ret = true;
    }
  }
  else if (connect_mode == 2)
  {
    if (write(sockfd, data.data(), data.size()) == (unsigned int)data.size())
    {
      ret = true;
    }
  }
  return ret;
}


bool dh::GripperController::check_data(uint8_t *data)
{
  if (0xFF == data[0] &&
      0xFE == data[1] &&
      0xFD == data[2] &&
      0xFC == data[3] &&
      0xFB == data[13])
  {
    return true;
  }
  else
  {
    ROS_WARN("get data structure false");
    return false;
  }
}


bool dh::GripperController::ensure_set_command(std::vector<uint8_t> data)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    if (read_data(3))
    {
      if (data.at(0) == readtempdata.frame_head[0] &&
          data.at(1) == readtempdata.frame_head[1] &&
          data.at(2) == readtempdata.frame_head[2] &&
          data.at(3) == readtempdata.frame_head[3] &&
          data.at(4) == readtempdata.DeviceID &&
          data.at(5) == readtempdata.Register[0] &&
          data.at(6) == readtempdata.Register[1] &&
          data.at(7) == readtempdata.option &&
          data.at(8) == readtempdata.reserve &&
          data.at(9) == readtempdata.data[0] &&
          data.at(10) == readtempdata.data[1] &&
          data.at(11) == readtempdata.data[2] &&
          data.at(12) == readtempdata.data[3] &&
          data.at(13) == readtempdata.frame_end)
      {
        ret = true;
        break;
      }
    }
  } while (iter++ < 1);

  return ret;
}


bool dh::GripperController::ensure_get_command(std::vector<uint8_t> data)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    if (read_data(3))
    {
      if (data.at(0) == readtempdata.frame_head[0] &&
          data.at(1) == readtempdata.frame_head[1] &&
          data.at(2) == readtempdata.frame_head[2] &&
          data.at(3) == readtempdata.frame_head[3] &&
          data.at(4) == readtempdata.DeviceID &&
          data.at(5) == readtempdata.Register[0] &&
          data.at(6) == readtempdata.Register[1] &&
          // data.at(7) == (uint8_t)(0) &&
          data.at(8) == readtempdata.reserve &&
          // data.at(9) == readtempdata.data[0] &&
          data.at(10) == readtempdata.data[1] &&
          data.at(11) == readtempdata.data[2] &&
          data.at(12) == readtempdata.data[3] &&
          data.at(13) == readtempdata.frame_end)
      {
        ret = true;
        break;
      }
    }
  } while (iter++ < 1);

  return ret;
}


bool dh::GripperController::ensure_run_end(std::vector<uint8_t> data)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    if (read_data(3))
    {
      if (data.at(0) == readtempdata.frame_head[0] &&
          data.at(1) == readtempdata.frame_head[1] &&
          data.at(2) == readtempdata.frame_head[2] &&
          data.at(3) == readtempdata.frame_head[3] &&
          data.at(4) == readtempdata.DeviceID &&
          data.at(5) == readtempdata.Register[0] &&
          data.at(6) == readtempdata.Register[1] &&
          data.at(7) == readtempdata.option &&
          data.at(8) == readtempdata.reserve &&
          // 0 != readtempdata.data[0] &&
          data.at(10) == readtempdata.data[1] &&
          data.at(11) == readtempdata.data[2] &&
          data.at(12) == readtempdata.data[3] &&
          data.at(13) == readtempdata.frame_end)
      {
        ret = true;
      }
    }
  } while (ret == false && iter++ < 1);

  return ret;
}
