/**
This is the control interface for the DH Hand
Author:   Jie Sun
Email:    jie.sun@dh-robotics.com
Date:     2019 July 1

Version 2.0
Copyright @ DH-Robotics Ltd.
**/

#include "dh_gripper/gripper_controller.h"


dh::GripperController::GripperController(ros::NodeHandle node, const std::string &name) :
  as_(node, name, boost::bind(&GripperController::actuateHandCB, this, _1), false)
{
  // Parameters
  node.param<std::string>("gripper_model", gripper_model, "AG-2E");
  ROS_INFO("Gripper Model: %s", gripper_model.c_str());

  node.param<std::string>("connect_port", hand_port_name_, "/dev/DH_hand");
  ROS_INFO("Connection port: %s", hand_port_name_.c_str());

  node.param<double>("data_timeout", WaitDataTime_, 0.5);


  connect_mode = 0;
  if (hand_port_name_.find('/') != std::string::npos)
  {
    connect_mode = 1;
  }
  if (hand_port_name_.find(':') != std::string::npos)
  {
    connect_mode = 2;
  }
  ROS_INFO("connect_mode : %d", connect_mode);
  if (!build_conn())
  {
    return;
  }
  if (init())
  {
    ROS_INFO("Initialized");
  }

  joint_pos.resize(1, 0.0);
  joint_eff.resize(1, 0.0);

  stalled.resize(1, false);
  reached_goal.resize(1, false);

  ros::ServiceServer service = node.advertiseService("gripper_state", &dh::GripperController::jointValueCB, this);
  as_.start();

  ROS_INFO("server started");
}


dh::GripperController::~GripperController()
{
  close_device();
}


bool dh::GripperController::jointValueCB(dh_gripper::GripperState::Request &req, dh_gripper::GripperState::Response &res)
{
  auto ret = false;
  auto iter = 0;
  do
  {
    readtempdata.DataStream_clear();
    switch (req.get_target)
    {
    case 0:
      ROS_INFO("request: getMotorForce");
      driver.getMotorForce();
      break;
    case 1:
      ROS_INFO("request: getMotor1Position");
      driver.getMotorPosition(1);
      break;
    case 2:
      ROS_INFO("request: getMotor2Position");
      if (gripper_model == "AG-3E")
      {
        driver.getMotorPosition(2);
        break;
      }
    default:
      ROS_ERROR("invalid read command");
      return false;
      break;
    }

    if (write_data(driver.getStream()))
    {
      if (ensure_get_command(driver.getStream()))
      {
        ret = true;
        break;
      }
      else
      {
        ROS_WARN("jointValueCB wait timeout");
      }
    }
    else
    {
      ROS_WARN("jointValueCB write command");
    }
  } while (iter++ < 2);

  res.return_data = readtempdata.data[0];
  return ret;
}


void dh::GripperController::actuateHandCB(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  ROS_INFO("Start to move the DH %s Gripper", gripper_model.c_str());

  int motor_id = 1;

  control_msgs::GripperCommandFeedback feedback;
  control_msgs::GripperCommandResult result;
  bool succeeded = false;
  bool bFeedback = false;

  // move Motor
  // if (Hand_Model_ == "AG-2E" && goal->MotorID == 2)
  // {
  //   ROS_ERROR("invalid AG-2E command");
  //   as_.setAborted(result);
  //   return;
  // }


  if (goal->command.position < 0 || goal->command.position > 100)
  {
    ROS_ERROR("command position: %.1f, out of range.", goal->command.position);
    as_.setAborted(result);
    return;
  }

  if (goal->command.max_effort < 15 || goal->command.max_effort > 100)
  {
    ROS_ERROR("command max_effort: %.1f, out of range.", goal->command.max_effort);
    as_.setAborted(result);
    return;
  }


  setGrippingForce(goal->command.max_effort);
  succeeded = moveHand(motor_id, goal->command.position, bFeedback);

  int position;
  int effort;

  getGrippingPosition(motor_id, position);
  getGrippingForce(effort);

  feedback.position = position;
  feedback.effort = effort;
  feedback.stalled = !bFeedback;
  feedback.reached_goal = bFeedback;
  as_.publishFeedback(feedback);

  ros::Duration(WaitDataTime_).sleep();

  getGrippingPosition(motor_id, position);
  getGrippingForce(effort);

  result.position = position;
  result.effort = effort;
  result.stalled = !bFeedback;
  result.reached_goal = bFeedback;

  if (succeeded)
    as_.setSucceeded(result);
  else
    as_.setAborted(result);
}


bool dh::GripperController::build_conn()
{
  bool hand_connected = false;

  if (connect_mode == 1)
  {
    try
    {
      serial.setPort(hand_port_name_);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      serial.setTimeout(timeout);
      serial.open();
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR("Unable to open port of the hand");
      return hand_connected;
    }

    if (serial.isOpen())
    {
      ROS_INFO("Serial Port for hand initialized");
      hand_connected = true;
    }
    else
    {
      return hand_connected;
    }
  }
  else if (connect_mode == 2)
  {

    std::string servInetAddr = hand_port_name_.substr(0, hand_port_name_.find(":"));
    int PORT = atoi(hand_port_name_.substr(hand_port_name_.find(":") + 1, hand_port_name_.size() - hand_port_name_.find(":") - 1).c_str());

    /*创建socket*/
    struct sockaddr_in serv_addr;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) != -1)
    {
      ROS_INFO("Socket id = %d", sockfd);
      /*设置sockaddr_in 结构体中相关参数*/
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(PORT);
      inet_pton(AF_INET, servInetAddr.c_str(), &serv_addr.sin_addr);
      bzero(&(serv_addr.sin_zero), 8);
      /*调用connect 函数主动发起对服务器端的连接*/
      if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
      {
        ROS_ERROR("Connect failed!\n");
        hand_connected = false;
      }
      else
      {
        ROS_INFO("connected");
        hand_connected = true;
      }
    }
    else
    {
      ROS_ERROR("Socket failed!\n");
      hand_connected = false;
    }
  }
  return hand_connected;
}


bool dh::GripperController::init()
{
  auto ret = false;
  auto iter = 0;
  auto wait_iter = 0;
  driver.setInitialize();
  do
  {
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

    ROS_INFO("init ensure_set_command");

    driver.setInitialize();
    do
    {
      ros::Duration(0.5).sleep();

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
  // stop communication
  if (connect_mode == 1)
  {
    serial.close();
  }
  else if (connect_mode == 2)
  {
    close(sockfd);
  }
}


bool dh::GripperController::moveHand(int motor_id, int target_position, bool &feedback)
{
  auto ret = false;
  auto iter = 0;
  auto wait_iter = 0;
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

      if (readtempdata.data[0] == DH_Robotics::FeedbackType::ARRIVED)
      {
        feedback = true;
        ret = true;
      }
      else if (readtempdata.data[0] == DH_Robotics::FeedbackType::CATCHED)
      {
        feedback = false;
        ret = true;
      }

    } while (ret == false && wait_iter++ < 100);

  } while (ret == false && iter++ < 3);

  return ret;
}


bool dh::GripperController::getHandFeedback(int motor_id, bool &feedback) {
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

    if (readtempdata.data[0] == DH_Robotics::FeedbackType::ARRIVED)
    {
      feedback = true;
      ret = true;
    }
    else if (readtempdata.data[0] == DH_Robotics::FeedbackType::CATCHED)
    {
      feedback = false;
      ret = true;
    }

  } while (ret == false && iter++ < 100);

  return ret;
}


bool dh::GripperController::getGrippingPosition(int motor_id, int &position) {
  auto ret = false;
  auto iter = 0;
  driver.getMotorPosition(motor_id);
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
    if (write_data(driver.getStream()))
    {
      if (ensure_set_command(driver.getStream()))
      {
        ret = true;
        break;
      }
      else
      {
        ROS_WARN("setMotorForce wait timeout");
      }
    }
    else
    {
      ROS_WARN("setMotorForce write data error");
    }
  } while (iter++ < 3);

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


bool dh::GripperController::write_data(std::vector<uint8_t> data)
{
  bool ret = false;
  // ROS_INFO("send x");
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
        break;
      }
    }
  } while (iter++ < 1);
  return ret;
}


bool dh::GripperController::chacke_data(uint8_t *data)
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


bool dh::GripperController::read_data(uint8_t waitconunt)
{
  auto ret = false;
  auto iter = 0;
  auto getframe = false;
  uint8_t buf[14];

  do
  {
    ros::Duration(WaitDataTime_).sleep();
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
      unsigned char tempbuf[140] = {0};
      int get_num = recv(sockfd, tempbuf, 140, MSG_DONTWAIT);
      for (int i = 0; i < get_num; i++)
        temp.push_back(tempbuf[i]);
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
      if (chacke_data(buf))
      {
        // ROS_INFO("Read: %X %X %X %X %X %X %X %X %X %X %X %X %X %X", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]);
        ret = true;
        break;
      }
    }
  } while (iter++ < waitconunt);

  // if (iter >= waitconunt)
  //   ROS_ERROR_STREAM("Read Overtime you can increase 'WaitDataTime' in launch file ");

  return ret;
}
