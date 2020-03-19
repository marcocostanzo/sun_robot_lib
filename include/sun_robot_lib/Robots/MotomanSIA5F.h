/*

    Robot Class for the Motoman SIA5F

    Copyright 2019-2020 Università della Campania Luigi Vanvitelli

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

#ifndef ROBOTMOTOMANSIA5F_H
#define ROBOTMOTOMANSIA5F_H

#include "sun_robot_lib/Robot.h"

#define MOTOMANSIA5F_MODEL_STR "MotomanSIA5F"

namespace sun
{
class MotomanSIA5F : public Robot
{
public:
  /*!
      Full constructor
  */
  MotomanSIA5F(const TooN::Matrix<4, 4>& n_T_e, double dls_joint_speed_saturation, const std::string& name);

  /*!
      Constructor with name only
  */
  MotomanSIA5F(const std::string& name);

  /*!
      Empty constructor
  */
  MotomanSIA5F();
};

}  // namespace sun

#endif