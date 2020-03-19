/*

    Robot Revolute Link

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

#ifndef ROBOTLINKREVOLUTE_H
#define ROBOTLINKREVOLUTE_H

#include "sun_robot_lib/RobotLink.h"

namespace sun
{
//! Revolute Link
class RobotLinkRevolute : public RobotLink
{
private:
  RobotLinkRevolute();  // No Default Constructor

protected:
public:
  /*=============CONSTRUCTORS===========*/

  //! Full Constructor
  RobotLinkRevolute(double a, double alpha, double d, double robot2dh_offset, bool robot2dh_flip,
                    double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double Joint_Soft_limit_lower,
                    double Joint_Soft_limit_higher, double hard_velocity_limit, double soft_velocity_limit,
                    std::string name = "joint_no_name");

  RobotLinkRevolute(double a, double alpha, double d, double robot2dh_offset = 0.0, bool robot2dh_flip = false,
                    double Joint_Hard_limit_lower = -INFINITY, double Joint_Hard_limit_higher = INFINITY,
                    double hard_velocity_limit = INFINITY, std::string name = "joint_no_name");

  /*=======END CONSTRUCTORS===========*/

  /*!
      Clone the object
  */
  virtual RobotLinkRevolute* clone() const override;

  /*!
          return the link angle
          Note:
              - For revolute link this is the joint variable,
              in that case this function returns NaN
  */
  virtual double getDH_theta() const override;

  /*!
          TODO
          ERROR IF LINK IS REVOLUTE
  */
  virtual void setDH_theta(double theta) override;

  /*!
      TODO
  */
  virtual void display() const override;

  /*!
          Retrun the joint type
          'p' = prismatic
          'r' = revolute
  */
  virtual char type() const override;

  /*!
      Compute the link transform matrix
      input qR in robot convention
  */
  virtual TooN::Matrix<4, 4> A(double q_R) const override;

};  // end class

using RobotLinkRevolutePtr = std::unique_ptr<RobotLinkRevolute>;

bool isRevolute(const RobotLink& l);

}  // namespace sun

#endif
