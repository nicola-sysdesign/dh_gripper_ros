/**
 *
 * auther: Jie Sun
 * email: jie.sun@kcl.ac.uk
 *
 * date: 2018 April 23
*/

#ifndef DH_GRIPPER_DRIVER_H
#define DH_GRIPPER_DRIVER_H
#include <vector>

#include "dh_gripper/definition.h"
#include "dh_gripper/DH_datastream.h"

/**
 * A controller for an individual joint
 */

namespace dh {

class DH_Driver {
protected:
    /// default velocity for each joint
    double velocity_;

    /// current joint position
    double position_1;

    /// true if current target has been reached
    bool target_reached_1;

     /// save the Data Stream
    DH_Robotics::DH_DataStream mDatastream;

public:
    DH_Driver();
    ~DH_Driver();

    /**
     * @brief Return  motot 1 current position
     *
     */
    double position1() { return position_1; }

    /**
     * @brief Return Velocity.
     *
     */
    double velocity() { return velocity_; }

    /**
     * @brief this function is reserve , do not use it
     *
     */
    void reset();

    /**
     * @brief this function is reserve , do not use it
     *
     */
    void setMotorVelocity(double p) { velocity_ = (int)p; }

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
    void SetOperation(int Reg=DH_Robotics::R_Initialization, int data=0, int Write = DH_Robotics::Write, int submode=0x02);

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
     * @brief Get the Motor Position
     *
     * @param MotorID
     */
    void getMotorPosition(const int &motor_id);

    /**
     * @brief Get the Feedback
     *
     * @param MotorID
     */
    void getFeedback(const int &motor_id);
};

} // namespace
#endif
