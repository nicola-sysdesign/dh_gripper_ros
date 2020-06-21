#ifndef AG95_HARDWARE_INTERFACE_H
#define AG95_HARDWARE_INTERFACE_H
// STL
#include <string>
#include <vector>

#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// transmission_interface
#include <transmission_interface/transmission.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
// hardware_interface
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// serial
#include <serial/serial.h>

// Boost
#include <boost/shared_ptr.hpp>

//
#include "dh_gripper/definition.h"
#include "dh_gripper/DH_datastream.h"
#include "dh_gripper/dh_driver.h"


namespace dh {

const double MIN_POSITION_LIMIT = 0, MAX_POSITION_LIMIT = 100;
const double MIN_EFFORT_LIMIT = 20, MAX_EFFORT_LIMIT = 100;

class GripperHW : public hardware_interface::RobotHW {

  int n_joints = 0;

  int connect_mode = 0;

  serial::Serial serial;

  int sockfd;
  ros::Duration data_timeout;

  dh::DH_Driver driver;
  DH_Robotics::DH_DataStream readtempdata;


  bool init_device()
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
        //
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


  bool read_data(uint8_t wait_count)
  {
    auto ret = false;
    auto iter = 0;
    auto getframe = false;
    uint8_t buf[14];

    do
    {
      ros::Duration(data_timeout).sleep();
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

  bool write_data(std::vector<uint8_t> data)
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
      if (::write(sockfd, data.data(), data.size()) == data.size())
      {
        ret = true;
      }
    }

    return ret;
  }

  bool check_data(uint8_t *data)
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


  bool ensure_set_command(std::vector<uint8_t> data)
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

  bool ensure_get_command(std::vector<uint8_t> data)
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

  bool ensure_run_end(std::vector<uint8_t> data)
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


  bool getGrippingPosition(int motor_id, int &position)
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

  bool getGrippingForce(int &gripping_force)
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

  bool setGrippingForce(int gripping_force)
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


  bool actuate_gripper(int motor_id, int target_position, bool &feedback)
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


protected:

  ros::NodeHandle node;

  transmission_interface::SimpleTransmission finger_transmission;
  transmission_interface::ActuatorToJointStateInterface act_to_jnt_state_interface;
  transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos_interface;

  transmission_interface::ActuatorData a_data, a_data_cmd;
  transmission_interface::JointData j_data, j_data_cmd;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface pos_jnt_interface;

  std::vector<double> a_pos, a_pos_cmd;
  std::vector<double> a_vel, a_vel_cmd;
  std::vector<double> a_eff, a_eff_cmd;

  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;

  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

public:

  GripperHW(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    data_timeout(0.2),
    finger_transmission(-100/0.03)
  {

  }


  bool init(const std::vector<std::string> &joints)
  {
    n_joints = joints.size();

    a_pos.resize(n_joints, 0.0); a_pos_cmd.resize(n_joints, 0.0);
    a_vel.resize(n_joints, 0.0); a_vel_cmd.resize(n_joints, 0.0);
    a_eff.resize(n_joints, 0.0); a_eff_cmd.resize(n_joints, 0.0);

    j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0); j_vel_cmd.resize(n_joints, 0.0);
    j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

    for (int i = 0; i < n_joints; i++)
    {
      a_data.position.push_back(&a_pos[i]);
      a_data.velocity.push_back(&a_vel[i]);
      a_data.effort.push_back(&a_eff[i]);

      a_data_cmd.position.push_back(&a_pos_cmd[i]);
      a_data_cmd.velocity.push_back(&a_vel_cmd[i]);
      a_data_cmd.effort.push_back(&a_eff_cmd[i]);

      j_data.position.push_back(&j_pos[i]);
      j_data.velocity.push_back(&j_vel[i]);
      j_data.effort.push_back(&j_eff[i]);

      j_data_cmd.position.push_back(&j_pos_cmd[i]);
      j_data_cmd.velocity.push_back(&j_vel_cmd[i]);
      j_data_cmd.effort.push_back(&j_eff_cmd[i]);


      transmission_interface::ActuatorToJointStateHandle act_to_jnt_state_handle("finger_trans", &finger_transmission, a_data, j_data);
      act_to_jnt_state_interface.registerHandle(act_to_jnt_state_handle);

      transmission_interface::JointToActuatorPositionHandle jnt_to_act_pos_handle("finger_trans", &finger_transmission, a_data_cmd, j_data_cmd);
      jnt_to_act_pos_interface.registerHandle(jnt_to_act_pos_handle);


      hardware_interface::JointStateHandle jnt_state_handle(joints[i], &j_pos[i], &j_vel[i], &j_eff[i]);
      jnt_state_interface.registerHandle(jnt_state_handle);

      hardware_interface::JointHandle pos_jnt_handle(jnt_state_handle, &j_pos_cmd[i]);
      pos_jnt_interface.registerHandle(pos_jnt_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&pos_jnt_interface);
    return true;
  }


  bool start(const std::string &host, unsigned int port)
  {
    std::string usb_port;
    if (node.getParam("usb/port", usb_port))
    {
      connect_mode = 1;
      if (!usb_connect(usb_port))
      {
        ROS_WARN("Failed to connect to %s", usb_port.c_str());
        return false;
      }
    }
    else
    {
      connect_mode = 2;
      if (!tcp_connect(host, port))
      {
        ROS_WARN("Failed to connect to %s:%d", host.c_str(), port);
        return false;
      }
    }

    if (!init_device())
    {
      ROS_FATAL("Failed to initialize %s", "DH AG-95");
      return false;
    }

    ROS_INFO("Hardware Interface started successfully.");
    return true;
  }


  bool usb_connect(const std::string &usb_port)
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
      ROS_FATAL("USB failure!");
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


  bool tcp_connect(const std::string &host, const int port)
  {
    // socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
      ROS_FATAL("TCP Socket failure!");
      return false;
    }

    ROS_DEBUG("TCP Socket ID: %d", sockfd);

    // sockaddr_in
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    inet_pton(AF_INET, host.c_str(), &serv_addr.sin_addr);
    bzero(&(serv_addr.sin_zero), 8);

    // connect
    if (connect(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) == -1)
    {
      ROS_ERROR("Failed to connect to %s:%d", host.c_str(), port);
      return false;
    }

    ROS_INFO("Connected to %s:%d", host.c_str(), port);
    return true;
  }


  void read(const ros::Time &time, const ros::Duration &period)
  {
    for (int i = 0; i < n_joints; i++)
    {
      int pos;
      int eff;

      if (!getGrippingPosition(1+i, pos))
      {
        ROS_WARN("Failed to get Gripping Position.");
        return;
      }
      if (!getGrippingForce(eff))
      {
        ROS_WARN("Failed to get Max Gripping Force.");
        return;
      }

      a_pos[i] = pos;
      a_eff[i] = eff;

      ROS_DEBUG("Gripping Position: %d", pos);
      ROS_DEBUG("Max Gripping Force: %d", eff);
    }

    act_to_jnt_state_interface.propagate();
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    jnt_to_act_pos_interface.propagate();

    for (int i = 0; i < n_joints; i++)
    {
      int pos_cmd = a_pos_cmd[i];
      int eff_cmd = 30.0;

      if (!setGrippingForce(eff_cmd))
      {
        ROS_WARN("Failed to set Max Gripping Force.");
        return;
      }

      bool not_stalled;
      if (!actuate_gripper(1+i, pos_cmd, not_stalled))
      {
        ROS_WARN("Failed to actuate the Gripper.");
        return;
      }
    }
  }


  bool stop()
  {
    if (connect_mode == 1)
    {
      serial.close();
    }
    else if (connect_mode == 2)
    {
      close(sockfd);
    }

    ROS_INFO("Hardware Interface stopped.");
    return true;
  }

};

}  // namespace
#endif
