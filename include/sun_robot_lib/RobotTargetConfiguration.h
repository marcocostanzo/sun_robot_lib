#ifndef SUN_ROBOT_LIB_ROBOT_TARGET_CONFIGURATION_H
#define SUN_ROBOT_LIB_ROBOT_TARGET_CONFIGURATION_H

#include "sun_robot_lib/JointVelocityGenerator.h"
#include "sun_robot_lib/Robot.h"

namespace sun
{

    /*!
        Generatore di velocità di giunto a partire dallo stato del robot
    */
    class RobotTargetConfiguration : public JointVelocityGenerator
    {

    private:
    protected:
        std::shared_ptr<Robot> robot_;

        TooN::Vector<> desired_configuration_;
        TooN::Vector<> desired_configuration_joint_weights_;

    public:
        virtual TooN::Vector<> generateJointVelocityDH(const TooN::Vector<> &q_DH) const override
        {
            TooN::Vector<> d_W = TooN::Zeros(robot_->getNumJoints());
            double sum_w = 0.0;
            for (int i = 0; i < robot_->getNumJoints(); i++)
            {
                auto link = robot_->getLink(i);

                TooN::Vector<2> limits = link->getSoftJointLimits();

                // case of infinity limits
                if (std::isinf(limits[0]) || std::isinf(limits[1]))
                {
                    d_W[i] = 0.0;
                }
                else
                {

                    // Check Limits
                    limits[0] = link->joint_Robot2DH(limits[0]);
                    limits[1] = link->joint_Robot2DH(limits[1]);
                    if (limits[0] > limits[1])
                    {
                        double tmp = limits[0];
                        limits[0] = limits[1];
                        limits[1] = tmp;
                    }

                    d_W[i] = (q_DH[i] - desired_configuration_[i]) / (limits[1] - limits[0]) * desired_configuration_joint_weights_[i];
                }

                sum_w += desired_configuration_joint_weights_[i];
            }
            d_W *= -1.0 / sum_w;
            return d_W;
        }
    };

}

#endif