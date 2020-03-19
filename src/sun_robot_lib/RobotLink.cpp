/*

    Robot Class

    Copyright 2018-2020 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "sun_robot_lib/RobotLink.h"

using namespace TooN;
using namespace std;

namespace sun
{
/*======CONSTRUCTORS======*/

// Full Constructor
RobotLink::RobotLink(double a, double alpha, double d, double theta, double robot2dh_offset, bool robot2dh_flip,
                     double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double Joint_Soft_limit_lower,
                     double Joint_Soft_limit_higher, double hard_velocity_limit, double soft_velocity_limit,
                     std::string name)
{
  _a = a;
  _alpha = alpha;
  _d = d;
  _theta = theta;
  _robot2dh_offset = robot2dh_offset;
  _robot2dh_flip = robot2dh_flip;
  setHardJointLimits(Joint_Hard_limit_lower, Joint_Hard_limit_higher);
  setSoftJointLimits(Joint_Soft_limit_lower, Joint_Soft_limit_higher);
  setHardVelocityLimit(hard_velocity_limit);
  setSoftVelocityLimit(soft_velocity_limit);
  _name = name;
}

RobotLink::RobotLink(double a, double alpha, double d, double theta, double robot2dh_offset, bool robot2dh_flip,
                     double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double hard_velocity_limit,
                     string name)
  : RobotLink(a, alpha, d, theta, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
              Joint_Hard_limit_lower, Joint_Hard_limit_higher, hard_velocity_limit, hard_velocity_limit, name)
{
}

/*======END CONSTRUCTORS======*/

//========Varie=======//

void RobotLink::checkLowerHigher(double lower, double higher)
{
  if (lower > higher)
  {
    cout << ROBOT_ERROR_COLOR "[RobotLink] Error in checkLowerHigher( double lower, double higher ): lower > "
                              "higher " ROBOT_CRESET
         << endl;
    exit(-1);
  }
}

Matrix<4, 4> RobotLink::A_internal(double theta, double d) const
{
  double sa = sin(_alpha);
  double ca = cos(_alpha);

  double st = sin(theta);
  double ct = cos(theta);

  // standard DH
  return Data(ct, -st * ca, st * sa, _a * ct, st, ct * ca, -ct * sa, _a * st, 0.0, sa, ca, d, 0.0, 0.0, 0.0, 1.0);
}

//===================//

//========GETTERS==============//

/*
    return the link length
*/
double RobotLink::getDH_a() const
{
  return _a;
}

/*
    return the link twist
*/
double RobotLink::getDH_alpha() const
{
  return _alpha;
}

/*
    return the link offset
    Note:
        - For prismatic link this is the joint variable,
          in that case this function returns NaN
*/
double RobotLink::getDH_d() const
{
  return _d;
}

/*
    return the link angle
    Note:
        - For revolute link this is the joint variable,
          in that case this function returns NaN
*/
double RobotLink::getDH_theta() const
{
  return _theta;
}

/*
    Return the offset between the robot and DH convention
*/
double RobotLink::getRobot2DH_offset() const
{
  return _robot2dh_offset;
}

/*
    Return the sign between robot and DH conventions
*/
bool RobotLink::getRobot2DH_flip() const
{
  return _robot2dh_flip;
}

/*
    TODO
*/
Vector<2> RobotLink::getSoftJointLimits() const
{
  return makeVector(_Joint_Soft_limit_lower, _Joint_Soft_limit_higher);
}

/*
    TODO
*/
Vector<2> RobotLink::getHardJointLimits() const
{
  return makeVector(_Joint_Hard_limit_lower, _Joint_Hard_limit_higher);
}

/*
    TODO
*/
double RobotLink::getSoftVelocityLimit() const
{
  return _soft_velocity_limit;
}

/*
    TODO
*/
double RobotLink::getHardVelocityLimit() const
{
  return _hard_velocity_limit;
}

/*
    Return the Joint Name
*/
string RobotLink::getName() const
{
  return _name;
}

//======END GETTERS===========//

//========SETTERS==============//

/*
    TODO
*/
void RobotLink::setDH_a(double a)
{
  _a = a;
}

/*
    TODO
*/
void RobotLink::setDH_alpha(double alpha)
{
  _alpha = alpha;
}

/*
    TODO
    ERROR IF LINK IS PRISMATIC
*/
void RobotLink::setDH_d(double d)
{
  _d = d;
}

/*
    TODO
    ERROR IF LINK IS REVOLUTE
*/
void RobotLink::setDH_theta(double theta)
{
  _theta = theta;
}

/*
    TODO
*/
void RobotLink::setRobot2DH_offset(double offset)
{
  _robot2dh_offset = offset;
}

/*
    TODO
*/
void RobotLink::setRobot2DH_flip(bool flip)
{
  _robot2dh_flip = flip;
}

/*
    TODO
*/
void RobotLink::setSoftJointLimits(double lower, double higher)
{
  checkLowerHigher(lower, higher);
  _Joint_Soft_limit_lower = lower;
  _Joint_Soft_limit_higher = higher;
}

/*
    TODO
*/
void RobotLink::setSoftJointLimits(const TooN::Vector<2>& limits)
{
  setSoftJointLimits(limits[0], limits[1]);
}

/*
    TODO
*/
void RobotLink::setHardJointLimits(double lower, double higher)
{
  checkLowerHigher(lower, higher);
  _Joint_Hard_limit_lower = lower;
  _Joint_Hard_limit_higher = higher;
}

/*
    TODO
*/
void RobotLink::setHardJointLimits(const TooN::Vector<2>& limits)
{
  setHardJointLimits(limits[0], limits[1]);
}

/*
    TODO
*/
void RobotLink::setHardVelocityLimit(double velocity_limit)
{
  if (velocity_limit < 0.0)
  {
    cout << ROBOT_ERROR_COLOR "[RobotLink] Error in setHardVelocityLimit( double velocity_limit): "
                              "velocity_limit<0.0" ROBOT_CRESET
         << endl;
    exit(-1);
  }
  _hard_velocity_limit = velocity_limit;
}

/*
    TODO
*/
void RobotLink::setSoftVelocityLimit(double velocity_limit)
{
  if (velocity_limit < 0.0)
  {
    cout << ROBOT_ERROR_COLOR "[RobotLink] Error in setSoftVelocityLimit( double velocity_limit): "
                              "velocity_limit<0.0" ROBOT_CRESET
         << endl;
    exit(-1);
  }
  _soft_velocity_limit = velocity_limit;
}

/*
    TODO
*/
void RobotLink::setName(const std::string& name)
{
  _name = name;
}

//======END SETTERS===========//

/*
    TODO
*/
void RobotLink::display() const
{
  cout <<

      "RobotLink [" << _name << "]" << endl
       <<

      "Type: " << type() << endl
       <<

      "Kinematic parameters (DH):" << endl
       << "a = " << _a << endl
       << "alpha = " << _alpha << endl
       << "theta = " << _theta << endl
       <<

      "Robot2DH Conversion:" << endl
       << "offset = " << _robot2dh_offset << " | flip = " << (_robot2dh_flip ? "yes" : "no") << endl
       <<

      "SoftLimits:" << endl
       << "[" << _Joint_Soft_limit_lower << " | " << _Joint_Soft_limit_higher << "]" << endl
       <<

      "HardLimits:" << endl
       << "[" << _Joint_Hard_limit_lower << " | " << _Joint_Hard_limit_higher << "]" << endl
       <<

      "Soft Velocity Limit: " << _soft_velocity_limit << endl
       << "Hard Velocity Limit: " << _hard_velocity_limit

      ;  // End COUT
}

/*
    return True if the input q_R (in Robot convention) exceeds the softLimits
*/
bool RobotLink::exceededSoftJointLimits(double q_R) const
{
  return (q_R <= _Joint_Soft_limit_lower || q_R >= _Joint_Soft_limit_higher);
}

/*
    return True if the input q_Robot (in Robot convention) exceeds the HardRobotLimits
*/
bool RobotLink::exceededHardJointLimits(double q_R) const
{
  return (q_R <= _Joint_Hard_limit_lower || q_R >= _Joint_Hard_limit_higher);
}

/*
    return True if the input q_vel exceeds the velocity soft limit
*/
bool RobotLink::exceededSoftVelocityLimit(double q_vel) const
{
  return (abs(q_vel) >= _soft_velocity_limit);
}

/*
    return True if the input q_vel exceeds the velocity hard limit
*/
bool RobotLink::exceededHardVelocityLimit(double q_vel) const
{
  return (abs(q_vel) >= _hard_velocity_limit);
}

/*
    TODO
*/
double RobotLink::joint_Robot2DH(double q_Robot) const
{
  if (_robot2dh_flip)
    return (-q_Robot + _robot2dh_offset);
  else
    return (q_Robot + _robot2dh_offset);
}

/*
    TODO
*/
double RobotLink::joint_DH2Robot(double q_DH) const
{
  if (_robot2dh_flip)
    return -(q_DH - _robot2dh_offset);
  else
    return (q_DH - _robot2dh_offset);
}

/*
    TODO
*/
double RobotLink::jointvel_Robot2DH(double q_vel_Robot) const
{
  if (_robot2dh_flip)
    return -q_vel_Robot;
  else
    return q_vel_Robot;
}

/*
    TODO
*/
double RobotLink::jointvel_DH2Robot(double q_vel_DH) const
{
  if (_robot2dh_flip)
    return -q_vel_DH;
  else
    return q_vel_DH;
}

}  // namespace sun