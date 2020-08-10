#include "dh_controller/dh_driver.h"


dh::DH_Driver::DH_Driver()
{

}


dh::DH_Driver::~DH_Driver()
{

}


void dh::DH_Driver::reset()
{

}


std::vector<uint8_t> dh::DH_Driver::getStream()
{
  std::vector<uint8_t> temp_data;

  uint8_t buf[14];
  mDatastream.DatatoStream(buf, mDatastream.size);
  for (int i = 0; i < 14; i++)
  {
    temp_data.push_back(buf[i]);
  }

  return temp_data;
}


void dh::DH_Driver::SetOperation(int reg, int data, int write, int submode)
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


void dh::DH_Driver::setInitialize()
{
  // FF FE FD FC 01 08 02 01 00 00 00 00 00 FB
  SetOperation();
}


void dh::DH_Driver::setMotorPosition(int motor_id, const int &target_position)
{
  if (motor_id == 1)
  {
    SetOperation(DH_Robotics::R_Posistion_1, target_position, DH_Robotics::Write);
  }
  else if (motor_id == 2)
  {
    SetOperation(DH_Robotics::R_Posistion_2, target_position, DH_Robotics::Write);
  }
}


void dh::DH_Driver::getMotorPosition(int motor_id)
{
  // FF FE FD FC 01 06 02 01 00 5A 00 00 00 FB
  if (motor_id == 1)
  {
    SetOperation(DH_Robotics::R_Posistion_1, 0, DH_Robotics::Read);
  }
  else if (motor_id == 2)
  {
    SetOperation(DH_Robotics::R_Posistion_2, 0, DH_Robotics::Read);
  }
}


void dh::DH_Driver::setMotorForce(const int &target_force)
{
  // FF FE FD FC 01 05 02 01 00 5A 00 00 00 FB
  SetOperation(DH_Robotics::R_Force, target_force, DH_Robotics::Write);
}


void dh::DH_Driver::getMotorForce()
{
  // FF FE FD FC 01 05 02 01 00 5A 00 00 00 FB
  SetOperation(DH_Robotics::R_Force, 0, DH_Robotics::Read);
}


void dh::DH_Driver::getFeedback(int motor_id)
{
  SetOperation(DH_Robotics::R_Feedback, 0, DH_Robotics::Read, motor_id);
}
