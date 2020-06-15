#ifndef DH_GRIPPER_DRIVER_H
#define DH_GRIPPER_DRIVER_H
#include <cstdint>
#include <vector>

#include "dh_gripper/definition.h"
#include "dh_gripper/DH_datastream.h"

/**
 * A controller for an individual joint
 */

namespace dh {

class DH_Driver {
protected:

   /// save the Data Stream
  DH_Robotics::DH_DataStream mDatastream;

public:

  DH_Driver();

  ~DH_Driver();

  /**
   * @brief this function is reserve , do not use it
   *
   */
  void reset();

  /**
   * @brief Get the vector Steam
   *
   * @return std::vector<uint8_t>
   */
  std::vector<uint8_t> getStream();

  /**
   * @brief  Base Function , all setting functions call it
   *
   * @param Reg       RegisterType
   * @param data
   * @param Write     OPTION
   * @param submode   sub_RegisterType, such SlowModeType
   */
  void SetOperation(int Reg = DH_Robotics::R_Initialization, int data = 0, int Write = DH_Robotics::Write, int submode = 0x02);

  /**
   * @brief set the Initialize
   *
   */
  void setInitialize();

  /**
  * @brief Set the Motor 1 Position
  *
  * @param target_position
  */
  void setMotorPosition(int motor_id , const int &target_position);

  /**
   * @brief Get the Motor Position
   *
   * @param MotorID
   */
  void getMotorPosition(int motor_id);

  /**
   * @brief Set the Motor Force
   *
   * @param target_force
   */
  void setMotorForce(const int &target_force);

  /**
   * @brief Get the Motor Force
   *
   */
  void getMotorForce();

  /**
   * @brief Get the Feedback
   *
   * @param MotorID
   */
  void getFeedback(int motor_id);
};

} // namespace
#endif
