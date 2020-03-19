/*

    Robot Prismatic Link

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

#include "sun_robot_lib/RobotLinkPrismatic.h"

using namespace TooN;
using namespace std;

namespace sun
{
/*=============CONSTRUCTORS===========*/

// Full Constructor
RobotLinkPrismatic::RobotLinkPrismatic(double a, double alpha, double theta, double robot2dh_offset, bool robot2dh_flip,
                                       double Joint_Hard_limit_lower, double Joint_Hard_limit_higher,
                                       double Joint_Soft_limit_lower, double Joint_Soft_limit_higher,
                                       double hard_velocity_limit, double soft_velocity_limit, string name)
  : RobotLink(a, alpha, NAN, theta, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
              Joint_Soft_limit_lower, Joint_Soft_limit_higher, hard_velocity_limit, soft_velocity_limit, name)
{
}

RobotLinkPrismatic::RobotLinkPrismatic(double a, double alpha, double theta, double robot2dh_offset, bool robot2dh_flip,
                                       double Joint_Hard_limit_lower, double Joint_Hard_limit_higher,
                                       double hard_velocity_limit, string name)
  : RobotLink(a, alpha, NAN, theta, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
              hard_velocity_limit, name)
{
}

/*=======END CONSTRUCTORS===========*/

/*
    Clone the object
*/
RobotLinkPrismatic* RobotLinkPrismatic::clone() const
{
  return new RobotLinkPrismatic(*this);
}

/*
        return the link angle
        Note:
            - For revolute link this is the joint variable,
            in that case this function returns NaN
*/
double RobotLinkPrismatic::getDH_d() const
{
  return NAN;
}

/*
        TODO
        ERROR IF LINK IS REVOLUTE
*/
void RobotLinkPrismatic::setDH_d(double d)
{
  cout << ROBOT_ERROR_COLOR "[RobotLinkPrismatic] Error in setDH_theta( double d ): Cannot set d for "
                            "RobotLinkPrismatic" ROBOT_CRESET
       << endl;
  exit(-1);
}

/*
    TODO
*/
void RobotLinkPrismatic::display() const
{
  RobotLink::display();
}

/*
        Retrun the joint type
        'p' = prismatic
        'r' = revolute
*/
char RobotLinkPrismatic::type() const
{
  return 'p';
}

/*
    Compute the link transform matrix
    input q_DH in DH convention
*/
Matrix<4, 4> RobotLinkPrismatic::A(double q_DH) const
{
  return A_internal(_theta, q_DH);
}

bool isPrismatic(const RobotLink& l)
{
  return (l.type() == 'p');
}

}  // namespace sun