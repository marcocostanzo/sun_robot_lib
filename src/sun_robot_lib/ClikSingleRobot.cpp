
#include "sun_robot_lib/ClikSingleRobot.h"

namespace sun
{

    /*=========CONSTRUCTORS=========*/

    ClikSingleRobot::ClikSingleRobot(const std::shared_ptr<Robot> &robot, const TooN::Vector<> &qDH0) : robot_(robot), Clik(qDH0) {}

    /*========GETTERS============*/

    TooN::Vector<> ClikSingleRobot::getJointsRobot() const
    {
        robot_->joints_DH2Robot(qDH_k_);
    }

    TooN::Vector<> ClikSingleRobot::getJointsVelRobot() const
    {
        robot_->jointsvel_DH2Robot(qDHdot_k_);
    }

    /*========SETTERS============*/

    /*========CLIK=========*/

    bool ClikSingleRobot::checkHardJointLimits() const
    {
        return robot_->exceededHardJointLimits(getJointsRobot());
    }

    bool ClikSingleRobot::checkHardJointVelLimits() const
    {
        return robot_->exceededHardVelocityLimits(getJointsRobot());
    }

    bool ClikSingleRobot::checkSoftJointLimits() const
    {
        return robot_->exceededSoftJointLimits(getJointsRobot());
    }

    bool ClikSingleRobot::checkSoftJointVelLimits() const
    {
        return robot_->exceededSoftVelocityLimits(getJointsRobot());
    }

}
