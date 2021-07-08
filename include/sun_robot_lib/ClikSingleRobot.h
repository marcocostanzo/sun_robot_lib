#ifndef SUN_CLIK_SINGLE_ROBOT_H
#define SUN_CLIK_SINGLE_ROBOT_H

#include "sun_robot_lib/Clik.h"
#include "sun_robot_lib/Robot.h"

namespace sun
{

    /////////////////////////////////////////////////
    class ClikSingleRobot : public Clik
    {

    private:
    protected:
        std::shared_ptr<Robot> robot_;

    public:
        /*=========CONSTRUCTORS=========*/

        ClikSingleRobot(const std::shared_ptr<Robot> &robot, const TooN::Vector<> &qDH0);

        /*========GETTERS============*/

        virtual TooN::Vector<> getJointsRobot() const override;

        virtual TooN::Vector<> getJointsVelRobot() const override;

        /*========SETTERS============*/

        /*========CLIK=========*/

        virtual bool checkHardJointLimits() const override;

        virtual bool checkHardJointVelLimits() const override;

        virtual bool checkSoftJointLimits() const override;

        virtual bool checkSoftJointVelLimits() const override;
    };

}
#endif