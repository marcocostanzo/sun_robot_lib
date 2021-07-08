
#include "sun_robot_lib/Clik.h"

namespace sun
{
    /*=========CONSTRUCTORS=========*/

    /*!
            Default constructor
        */
    Clik::Clik(const TooN::Vector<> &qDH0) : qDH_k_(qDH0), qDHdot_k_(TooN::Zeros(qDH0.size())){};

    /*========GETTERS============*/

    TooN::Vector<> Clik::getJointsDH() const
    {
        return qDH_k_;
    }

    TooN::Vector<> Clik::getJointsVelDH() const
    {
        return qDHdot_k_;
    }

    bool Clik::allJointsAreActive() const
    {
        return fixed_joints_.empty();
    }

    TooN::Vector<> Clik::jointVelAddZerosForUnusedJoints(const TooN::Vector<> &qVel) const
    {
        std::vector<double> qVel_std = getSTDFromTooN(qVel);

        auto it = qVel_std.begin();
        for (int i = 0; i < fixed_joints_.size(); i++)
        {
            qVel_std.insert(it + fixed_joints_[i], 0.0);
        }
        return getTooNFromSTD(qVel_std);
    }

    TooN::Vector<> Clik::jointVelRemoveUnusedJonts(const TooN::Vector<> &qVel) const
    {
        std::vector<double> qVel_std = getSTDFromTooN(qVel);

        auto it = qVel_std.begin();
        for (int i = 0; i < fixed_joints_.size(); i++)
        {
            qVel_std.erase(it + fixed_joints_[i]);
        }
        return getTooNFromSTD(qVel_std);
    }

    TooN::Matrix<> Clik::jacobianRemoveUnusedJonts(const TooN::Matrix<> &jacob) const
    {
        // very inefficient
        std::vector<std::vector<double>> jacob_std_cols = getSTDClumnsFromTooN(jacob);

        auto it = jacob_std_cols.begin();
        for (int i = 0; i < fixed_joints_.size(); i++)
        {
            jacob_std_cols.erase(it + fixed_joints_[i]);
        }
        return getTooNFromSTDClumns(jacob_std_cols);
    }

    /*========SETTERS============*/

    void Clik::setDLSJointSpeedSaturation(double dls_joint_speed_saturation)
    {
        dls_joint_speed_saturation_ = dls_joint_speed_saturation;
    }

    /*========CLIK=========*/

    TooN::Vector<> Clik::clikDH_core_qdot_internal(const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                                   const TooN::Vector<> &qDHdot_secondary_obj) const
    {

        // Method without the DLS
        // SVD<> J_svd(jacob);
        // qpDH = J_svd.backsub(veld + gain * error, ROBOT_CLIK_NO_DLS_CONDITION_NUMBER);

        // Method with the DLS
        TooN::Vector<> vel_e = (veld + gain_error_ * error);
        double damping = norm(vel_e) / dls_joint_speed_saturation_;
        TooN::Matrix<> J_pinv_dls = pinv_DLS(jacob, damping);
        TooN::Vector<> qpDH = J_pinv_dls * vel_e;

        // Null space
        if (gain_null_space_ != 0.0)
        {
            qpDH += gain_null_space_ * nullSpaceProj(jacob, J_pinv_dls) * qDHdot_secondary_obj;
        }

        return qpDH;
    }

    TooN::Vector<> Clik::clikDH_core_qdot(const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                          const TooN::Vector<> &qDHdot_secondary_obj) const
    {
        if (allJointsAreActive())
        {
            return clikDH_core_qdot_internal(error, jacob, veld,
                                             qDHdot_secondary_obj);
        }
        else
        {
            return jointVelAddZerosForUnusedJoints(clikDH_core_qdot_internal(error, jacobianRemoveUnusedJonts(jacob), veld,
                                                                             jointVelRemoveUnusedJonts(qDHdot_secondary_obj)));
        }
    }

    void Clik::clikDH_core(const TooN::Vector<> qDH_k, const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                           const TooN::Vector<> &qDHdot_secondary_obj, TooN::Vector<> &qDH_k1, TooN::Vector<> &qDHdot_k) const
    {

        qDHdot_k = clikDH_core_qdot(error, jacob, veld,
                                    qDHdot_secondary_obj);

        qDH_k1 = (qDH_k + qDHdot_k *
                              Ts_);
    }

    void Clik::exec_single_step(const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                const TooN::Vector<> &qDHdot_secondary_obj)
    {
        TooN::Vector<> qDH_k1 = qDH_k_;
        TooN::Vector<> qDHdot_k = TooN::Zeros(qDH_k1.size());
        clikDH_core(qDH_k_, error, jacob, veld,
                    qDHdot_secondary_obj, qDH_k1, qDHdot_k);

        qDH_k_ = qDH_k1;
        qDHdot_k_ = qDHdot_k;
    }

    void Clik::safetyCheck() const
    {
        if (b_checkHardJointLimits_ && !checkHardJointLimits())
        {
            throw std::runtime_error("HardJointLimits");
        }
        if (b_checkHardJointVelLimits_ && !checkHardJointVelLimits())
        {
            throw std::runtime_error("HardJointVelLimits");
        }
        if (b_checkSoftJointLimits_ && !checkSoftJointLimits())
        {
            throw std::runtime_error("SoftJointLimits");
        }
        if (b_checkSoftJointVelLimits_ && checkSoftJointVelLimits())
        {
            throw std::runtime_error("SoftJointVelLimits");
        }
    }

}