#include "sun_robot_lib/Clik6DQuaternionSingleRobot.h"

namespace sun
{

    Clik6DQuaternionSingleRobot::Clik6DQuaternionSingleRobot(const std::shared_ptr<Robot> &robot, const TooN::Vector<> &qDH0) : ClikSingleRobot(robot, qDH0) {}

    /*========GETTERS============*/

    /*========SETTERS============*/

    /*========CLIK=========*/

    void Clik6DQuaternionSingleRobot::exec_single_step(const TooN::Vector<3> &pd, const UnitQuaternion &quatd, const TooN::Vector<3> &dpd, const TooN::Vector<3> &omegad, const TooN::Vector<> &qDHdot_secondary_obj)
    {
        // Compute Error
        TooN::Vector<> error = TooN::Zeros(6);
        // fkine
        TooN::Matrix<4, 4> b_T_e = robot_->fkine(qDH_k_);
        TooN::Vector<3> position = b_T_e.T()[3].slice<0, 3>();
        UnitQuaternion quat = UnitQuaternion(b_T_e, oldQuaternion_);
        // positionError
        error.slice<0, 3>() = pd - position;
        // orientationError
        UnitQuaternion deltaQ = quatd / quat;
        error.slice<3, 3>() = deltaQ.getV();

        // Use the geometric Jacobian
        TooN::Matrix<> jacob = robot_->jacob_geometric(qDH_k_);

        // Construct veld
        TooN::Vector<> veld = TooN::Zeros(6);
        veld.slice<0, 3>() = dpd;
        veld.slice<3, 3>() = omegad;

        ClikSingleRobot::exec_single_step(error, jacob, veld,
                                          qDHdot_secondary_obj);
    }

}
