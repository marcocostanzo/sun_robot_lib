/*

    Position Line Trajectory Generator Class
    This class is generates a line trajectory in the cartesian space

    Copyright 2018 Università della Campania Luigi Vanvitelli

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

#ifndef LINE_SEGMENT_TRAJ_H
#define LINE_SEGMENT_TRAJ_H

#include "Traj_Generators/Position_Traj_Interface.h"
#include "Traj_Generators/Scalar_Traj_Interface.h"

class Line_Segment_Traj : public Position_Traj_Interface {


private:

/*
    No default Constructor
*/
Line_Segment_Traj();

//These vars now are taken from _traj_s
double _duration, _initial_time;

protected:
        
/*
    Initial Position
*/
TooN::Vector<3> _pi;

/*
    Final Position
*/
TooN::Vector<3> _pf;

/*
    Trajectory for the scalar s variable should be a traj from 0 to 1
    s = 0 -> _pi   &   s = 1 -> _pf
*/
Scalar_Traj_Interface_Ptr _traj_s;

public:

/*======CONSTRUCTORS========*/

/*
    Full Constructor
*/
Line_Segment_Traj(const TooN::Vector<3>& pi, const TooN::Vector<3>& pf, const Scalar_Traj_Interface& traj_s);

/*
    Copy Constructor
*/
Line_Segment_Traj( const Line_Segment_Traj& traj );

/*
    Clone the object in the heap
*/
virtual Line_Segment_Traj* clone() const override;

/*======END CONSTRUCTORS========*/

/*====== GETTERS ========*/

/*
    TODO
*/
virtual TooN::Vector<3> getInitialPoint() const;

/*
    TODO
*/
virtual TooN::Vector<3> getFinalPoint() const;

/*
    Get the final time instant
*/
virtual double getFinalTime() const override;

/*
    Get the initial time instant
*/
virtual double getInitialTime() const override;

/*
    Get the duration [from base class]
*/
//virtual double getDuration() const;

/*
    Get the time left [from base class]
*/
//virtual double getTimeLeft(double secs) const;

/*====== END GETTERS ========*/

/*====== SETTERS =========*/

/*
    Change the initial time instant (translate the trajectory in the time)
*/
virtual void changeInitialTime(double initial_time) override;

/*====== END SETTERS =========*/

/*====== RUNNERS =========*/

/*
    Get Position at time secs
*/
virtual TooN::Vector<3> getPosition(double secs) const override;

/*
    Get Velocity at time secs
*/
virtual TooN::Vector<3> getVelocity(double secs) const override;

/*
    Get Acceleration at time secs
*/
virtual TooN::Vector<3> getAcceleration(double secs) const override;

/*====== END RUNNERS =========*/

};

using Line_Segment_Traj_Ptr = std::unique_ptr<Line_Segment_Traj>;

#endif