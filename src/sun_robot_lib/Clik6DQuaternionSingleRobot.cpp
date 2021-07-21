#include "sun_robot_lib/Clik6DQuaternionSingleRobot.h"

namespace sun
{

    Clik6DQuaternionSingleRobot::Clik6DQuaternionSingleRobot(const std::shared_ptr<Robot> &robot) : ClikSingleRobot(robot) {}

    void Clik6DQuaternionSingleRobot::resetCurrentQaternion()
    {
        currentQuaternion_ = desiredQuaternion_;
    }

    void Clik6DQuaternionSingleRobot::resetDesiredCartesianTwist()
    {
        desiredLinearVelocity_ = TooN::Zeros;
        desiredAngularVelocity_ = TooN::Zeros;
    }

    /*========GETTERS============*/

    /*========SETTERS============*/

    /*========CLIK=========*/

    TooN::Vector<> Clik6DQuaternionSingleRobot::getClikError(const TooN::Vector<> &q_DH)
    {
        TooN::Vector<> error = TooN::Zeros(6);
        // fkine
        TooN::Matrix<4, 4> b_T_e = robot_->fkine(q_DH);
        TooN::Vector<3> position = transl(b_T_e);
        currentQuaternion_ = UnitQuaternion(b_T_e, currentQuaternion_);
        // positionError
        error.slice<0, 3>() = desiredPosition_ - position;
        // orientationError
        UnitQuaternion deltaQ = desiredQuaternion_ / currentQuaternion_;
        error.slice<3, 3>() = deltaQ.getV();

        return error;
    }

    TooN::Matrix<> Clik6DQuaternionSingleRobot::getClikJacobian(const TooN::Vector<> &q_DH)
    {
        return robot_->jacob_geometric(q_DH);
    }
    TooN::Vector<> Clik6DQuaternionSingleRobot::getDesiredCartesianTwist(const TooN::Vector<> &q_DH)
    {
        TooN::Vector<> desiredCartesianTwist = TooN::Zeros(6);
        desiredCartesianTwist.slice<0, 3>() = desiredLinearVelocity_;
        desiredCartesianTwist.slice<3, 3>() = desiredAngularVelocity_;
        return desiredCartesianTwist;
    }

}
