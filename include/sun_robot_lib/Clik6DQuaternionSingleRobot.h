#ifndef SUN_CLIK_6D_WITH_QUATERNION_SINGLE_ROBOT_H
#define SUN_CLIK_6D_WITH_QUATERNION_SINGLE_ROBOT_H

#include "sun_robot_lib/ClikSingleRobot.h"

namespace sun
{

    /////////////////////////////////////////////////
    class Clik6DQuaternionSingleRobot : public ClikSingleRobot
    {

    private:
    protected:
        //! For the continuity
        UnitQuaternion oldQuaternion_;

    public:
        /*=========CONSTRUCTORS=========*/

        Clik6DQuaternionSingleRobot(const std::shared_ptr<Robot> &robot, const TooN::Vector<> &qDH0);

        /*========GETTERS============*/

        /*========SETTERS============*/

        /*========CLIK=========*/

        void exec_single_step(const TooN::Vector<3> &pd, const UnitQuaternion &quatd, const TooN::Vector<3> &dpd, const TooN::Vector<3> &omegad, const TooN::Vector<> &qDHdot_secondary_obj);
    };

}
#endif