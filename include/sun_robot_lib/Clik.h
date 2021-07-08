#ifndef SUN_CLIK_H
#define SUN_CLIK_H

#include "TooN/TooN.h"
#include "sun_math_toolbox/PortingFunctions.h"
#include "sun_robot_lib/JointVelocityGenerator.h"
#include <memory>

namespace sun
{

    /////////////////////////////////////////////////
    class Clik : public JointVelocityGenerator
    {

    private:
    protected:
        double dls_joint_speed_saturation_ = 3.0;

        double gain_error_ = 0.0;
        double gain_null_space_ = 0.0;
        double Ts_ = 1E-3;

        bool b_checkHardJointLimits_ = true;
        bool b_checkHardJointVelLimits_ = true;
        bool b_checkSoftJointLimits_ = false;
        bool b_checkSoftJointVelLimits_ = false;

        std::vector<int> fixed_joints_;

        std::shared_ptr<JointVelocityGenerator> secondObjQdotDHgenerator_;

        // TooN::Vector<> qDH_k_;
        // TooN::Vector<> qDHdot_k_;

    public:
        /*=========CONSTRUCTORS=========*/

        /*!
            Default constructor
        */
        Clik(const TooN::Vector<>& qDH0);

        /*========MISC============*/

        template <int Size = TooN::Dynamic, typename Precision = TooN::DefaultPrecision, typename Base = TooN::Internal::VBase>
        static TooN::Vector<Size, Precision, Base> getTooNFromSTD(const std::vector<double> &vec_std)
        {
            TooN::Vector<Size, Precision, Base> v_toon = TooN::Zeros(vec_std.size());
            for (int i = 0; i < vec_std.size(); i++)
            {
                v_toon[i] = vec_std[i];
            }
            return v_toon;
        }

        template <int Size = TooN::Dynamic, typename Precision = TooN::DefaultPrecision, typename Base = TooN::Internal::VBase>
        static std::vector<double> getSTDFromTooN(const TooN::Vector<Size, Precision, Base> &vec_toon)
        {
            std::vector<double> vec_std(vec_toon.size());
            for (int i = 0; i < vec_toon.size(); i++)
                vec_std[i] = vec_toon[i];
            return vec_std;
        }

        template <int Rows = TooN::Dynamic, int Cols = Rows, class Precision = TooN::DefaultPrecision, class Layout = TooN::RowMajor>
        static std::vector<std::vector<double>> getSTDClumnsFromTooN(const TooN::Matrix<Rows, Cols, Precision, Layout> &matrix_toon)
        {
            std::vector<std::vector<double>> matrix_cols_std(matrix_toon.num_cols());
            for (int i = 0; i < matrix_toon.num_cols(); i++)
            {
                matrix_cols_std[i].resize(matrix_toon.num_rows());
                for (int j = 0; j < matrix_toon.num_rows(); j++)
                {
                    matrix_cols_std[i][j] = matrix_toon[j][i];
                }
            }
            return matrix_cols_std;
        }

        template <int Rows = TooN::Dynamic, int Cols = Rows, class Precision = TooN::DefaultPrecision, class Layout = TooN::RowMajor>
        static TooN::Matrix<Rows, Cols, Precision, Layout> getTooNFromSTDClumns(const std::vector<std::vector<double>> &matrix_cols_std)
        {
            TooN::Matrix<Rows, Cols, Precision, Layout> matrix_toon = TooN::Zeros(matrix_cols_std.size(), matrix_cols_std[0].size());
            for (int i = 0; i < matrix_toon.num_cols(); i++)
            {
                for (int j = 0; j < matrix_toon.num_rows(); j++)
                {
                    matrix_toon[i][j] = matrix_cols_std[j][i];
                }
            }
            return matrix_toon;
        }

        /*========GETTERS============*/

        virtual TooN::Vector<> getJointsDH() const;

        virtual TooN::Vector<> getJointsVelDH() const;

        virtual TooN::Vector<> getJointsRobot() const;

        virtual TooN::Vector<> getJointsVelRobot() const;

        virtual bool allJointsAreActive() const;

        virtual TooN::Vector<> jointVelAddZerosForUnusedJoints(const TooN::Vector<> &qVel) const;

        virtual TooN::Vector<> jointVelRemoveUnusedJonts(const TooN::Vector<> &qVel) const;

        virtual TooN::Matrix<> jacobianRemoveUnusedJonts(const TooN::Matrix<> &jacob) const;

        /*========SETTERS============*/

        /*!
      set Joint speed saturation used in dls for clik
        */
        virtual void setDLSJointSpeedSaturation(double dls_joint_speed_saturation);

        /*========CLIK=========*/

        /*
    Very General CLIK
    Implements the general version of the clik
    Inputs:
        - qDH_k: joints at time k
        - error: error vector (use the appropriate error type here)
        - jacob: Jacobian calculated in qDH_k (use appropriate jacob function here)
        - veld: desired velocity
        - gain: CLIK Gain
        - Ts: sampling time
        - gain_null_space: Gain for second objective
        - q0_p: velocity to be projected into the null space
    Outputs:
        return: qDH_k+1 joints at time k+1
        qpDH: joints velocity at time k+1
*/
        virtual TooN::Vector<> clikDH_core_qdot_internal(const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                                const TooN::Vector<> &qDHdot_secondary_obj) const;

        virtual TooN::Vector<> clikDH_core_qdot(const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                                const TooN::Vector<> &qDHdot_secondary_obj) const;

        virtual void clikDH_core(const TooN::Vector<> qDH_k, const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                 const TooN::Vector<> &qDHdot_secondary_obj, TooN::Vector<> &qDH_k1, TooN::Vector<> &qDHdot_k) const;

        virtual void exec_single_step(const TooN::Vector<> &error, const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
                                      const TooN::Vector<> &qDHdot_secondary_obj);

        virtual void safetyCheck() const;

        virtual bool checkHardJointLimits() const;
        virtual bool checkHardJointVelLimits() const;
        virtual bool checkSoftJointLimits() const;
        virtual bool checkSoftJointVelLimits() const;

    };

}
#endif