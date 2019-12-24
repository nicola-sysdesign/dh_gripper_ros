#include <vector>
#include "dh_hand_driver/hand_driver.h"
DH_Hand_Base::DH_Hand_Base(): velocity_(0),position_1(0),target_reached_1(false)
{
}

DH_Hand_Base::~DH_Hand_Base()
{
}

void DH_Hand_Base::reset()
{

}

std::vector<uint8_t> DH_Hand_Base::getStream()
{
  std::vector<uint8_t> temp_data;
  uint8_t buf[14];
  mDatastream.DatatoStream(buf,mDatastream.size);
  for (int i=0; i < 14; i++)
    temp_data.push_back(buf[i]);
  return temp_data;
}

void DH_Hand_Base::SetOperation(int reg, int data, int write, int submode)
{
    mDatastream.DataStream_clear();
    mDatastream.Register[0] = reg & 0xFF;
    mDatastream.Register[1] = submode & 0xFF;
    mDatastream.option = write & 0xFF;
    mDatastream.data[0] = (data >> 0) & 0xFF;
    mDatastream.data[1] = (data >> 8) & 0xFF;
    mDatastream.data[2] = (data >> 16) & 0xFF;
    mDatastream.data[3] = (data >> 24) & 0xFF;
}

void DH_Hand_Base::setInitialize()
{
    //FF FE FD FC 01 08 02 01 00 00 00 00 00 FB
    SetOperation();
}

void DH_Hand_Base::setMotorPosition(int motor_id, const int &target_position)
{
    if (motor_id == 1)
        SetOperation(DH_Robotics::R_Posistion_1, target_position, DH_Robotics::Write);
    else if (motor_id == 2)
        SetOperation(DH_Robotics::R_Posistion_2, target_position, DH_Robotics::Write);
}

void DH_Hand_Base::setMotorForce(const int &target_force)
{
    //FF FE FD FC 01 05 02 01 00 5A 00 00 00 FB
    SetOperation(DH_Robotics::R_Force, target_force, DH_Robotics::Write);
}

void DH_Hand_Base::getMotorPosition(const int &motor_id)
{
    //FF FE FD FC 01 06 02 01 00 5A 00 00 00 FB
    if (motor_id == 1)
        SetOperation(DH_Robotics::R_Posistion_1, 0, DH_Robotics::Read);
    else if (motor_id == 2)
        SetOperation(DH_Robotics::R_Posistion_2, 0, DH_Robotics::Read);
}

void DH_Hand_Base::getMotorForce()
{
    //FF FE FD FC 01 05 02 01 00 5A 00 00 00 FB
    SetOperation(DH_Robotics::R_Force, 0, DH_Robotics::Read);
}

void DH_Hand_Base::getFeedback(const int &motor_id)
{
    SetOperation(DH_Robotics::R_Feedback, 0, DH_Robotics::Read, motor_id);
}
