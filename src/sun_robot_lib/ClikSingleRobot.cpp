
#include "sun_robot_lib/ClikSingleRobot.h"

namespace sun {

/*=========CONSTRUCTORS=========*/

ClikSingleRobot::ClikSingleRobot(const std::shared_ptr<Robot> &robot)
    : robot_(robot) {}

/*========GETTERS============*/

/*========SETTERS============*/

/*========CLIK=========*/

bool ClikSingleRobot::exceededHardJointLimits(const TooN::Vector<> &qDH) const {
  return robot_->exceededHardJointLimits(robot_->joints_DH2Robot(qDH));
}

bool ClikSingleRobot::exceededHardJointVelLimits(
    const TooN::Vector<> &qDH_dot) const {
  return robot_->exceededHardVelocityLimits(
      robot_->jointsvel_DH2Robot(qDH_dot));
}

bool ClikSingleRobot::exceededSoftJointLimits(const TooN::Vector<> &qDH) const {
  return robot_->exceededSoftJointLimits(robot_->joints_DH2Robot(qDH));
}

bool ClikSingleRobot::exceededSoftJointVelLimits(
    const TooN::Vector<> &qDH_dot) const {
  return robot_->exceededSoftVelocityLimits(
      robot_->jointsvel_DH2Robot(qDH_dot));
}

} // namespace sun
