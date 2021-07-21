#ifndef SUN_CLIK_H
#define SUN_CLIK_H

#include "TooN/TooN.h"
#include "sun_math_toolbox/PortingFunctions.h"
#include "sun_robot_lib/JointVelocityGenerator.h"
#include "sun_robot_lib/exceptions.h"
#include <memory>

namespace sun {

/////////////////////////////////////////////////
class Clik : public JointVelocityGenerator {

private:
protected:
  double dls_joint_speed_saturation_ = 3.0;

  double gain_error_ = 0.0;
  double gain_null_space_ = 0.0;

  std::vector<unsigned int> fixed_joints_;

public:
  bool b_checkHardJointLimits_ = true;
  bool b_checkHardJointVelLimits_ = true;
  bool b_checkSoftJointLimits_ = false;
  bool b_checkSoftJointVelLimits_ = false;

  bool b_desired_twist_only_ = false;

  std::shared_ptr<JointVelocityGenerator> secondObjQdotDHgenerator_;
  /*=========CONSTRUCTORS=========*/

  /*!
      Default constructor
  */
  Clik();

  /*========MISC============*/

  template <int Size = TooN::Dynamic,
            typename Precision = TooN::DefaultPrecision,
            typename Base = TooN::Internal::VBase>
  static bool
  toonAnyIsNaNOrIsInf(const TooN::Vector<Size, Precision, Base> &v) {
    return TooN::isnan(v) || !TooN::isfinite(v);
  }

  template <int Size = TooN::Dynamic,
            typename Precision = TooN::DefaultPrecision,
            typename Base = TooN::Internal::VBase>
  static std::string to_string(const TooN::Vector<Size, Precision, Base> &v) {
    std::string out = "[";
    if (v.size() > 0) {
      out += " " + std::to_string(v[0]);
    }
    for (int i = 1; i < v.size(); i++) {
      out += " ," + std::to_string(v[i]);
    }
    out += " ]";
    return out;
  }

  template <int Size = TooN::Dynamic,
            typename Precision = TooN::DefaultPrecision,
            typename Base = TooN::Internal::VBase>
  static TooN::Vector<Size, Precision, Base>
  getTooNFromSTD(const std::vector<Precision> &vec_std) {
    TooN::Vector<Size, Precision, Base> v_toon = TooN::Zeros(vec_std.size());
    for (int i = 0; i < vec_std.size(); i++) {
      v_toon[i] = vec_std[i];
    }
    return v_toon;
  }

  template <int Size = TooN::Dynamic,
            typename Precision = TooN::DefaultPrecision,
            typename Base = TooN::Internal::VBase>
  static std::vector<Precision>
  getSTDFromTooN(const TooN::Vector<Size, Precision, Base> &vec_toon) {
    std::vector<double> vec_std(vec_toon.size());
    for (int i = 0; i < vec_toon.size(); i++)
      vec_std[i] = vec_toon[i];
    return vec_std;
  }

  template <int Rows = TooN::Dynamic, int Cols = Rows,
            class Precision = TooN::DefaultPrecision,
            class Layout = TooN::RowMajor>
  static std::vector<std::vector<Precision>> getSTDClumnsFromTooN(
      const TooN::Matrix<Rows, Cols, Precision, Layout> &matrix_toon) {

    std::vector<std::vector<Precision>> matrix_cols_std(matrix_toon.num_cols());

    for (int i = 0; i < matrix_toon.num_cols(); i++) {
      matrix_cols_std[i].resize(matrix_toon.num_rows());
      for (int j = 0; j < matrix_toon.num_rows(); j++) {
        matrix_cols_std[i][j] = matrix_toon[j][i];
      }
    }

    return matrix_cols_std;
  }

  template <int Rows = TooN::Dynamic, int Cols = Rows,
            class Precision = TooN::DefaultPrecision,
            class Layout = TooN::RowMajor>
  static TooN::Matrix<Rows, Cols, Precision, Layout> getTooNFromSTDClumns(
      const std::vector<std::vector<Precision>> &matrix_cols_std) {

    TooN::Matrix<Rows, Cols, Precision, Layout> matrix_toon =
        TooN::Zeros(matrix_cols_std[0].size(), matrix_cols_std.size());

    for (int i = 0; i < matrix_toon.num_rows(); i++) {
      for (int j = 0; j < matrix_toon.num_cols(); j++) {
        matrix_toon[i][j] = matrix_cols_std[j][i];
      }
    }

    return matrix_toon;
  }

  /*========GETTERS============*/

  virtual bool allJointsAreActive() const;

  virtual TooN::Vector<>
  jointVelAddZerosForUnusedJoints(const TooN::Vector<> &qVel) const;

  virtual TooN::Vector<>
  jointVelRemoveUnusedJonts(const TooN::Vector<> &qVel) const;

  virtual TooN::Matrix<>
  jacobianRemoveUnusedJonts(const TooN::Matrix<> &jacob) const;

  virtual TooN::Vector<>
  generateJointVelocityDH(const TooN::Vector<> &q_DH) override;

  /*========SETTERS============*/

  /*!
set Joint speed saturation used in dls for clik
  */
  virtual void setDLSJointSpeedSaturation(double dls_joint_speed_saturation);

  virtual void setGainNullSpace(double gain_null_space);

  virtual void setGainError(double gain_error);

  virtual void setFixedJoints(const std::vector<unsigned int> &fixed_joints);

  /*========CLIK=========*/

  /*!
      Versione interna dell'algoritmo
      prende quasi tutto come perametro
      Non considera i fixed_joint e il generatore dell'obj secondario
  */
  virtual TooN::Vector<> clikDH_core_qdot_internal_raw(
      const TooN::Vector<> &error, const TooN::Matrix<> &jacob,
      const TooN::Vector<> &veld,
      const TooN::Vector<> &qDHdot_secondary_obj) const;

  /*!
      Versione interna dell'algoritmo, come clikDH_core_qdot_internal_raw
      ma considera solo la velocità desiderata (errore è implicitamente nullo)
  */
  virtual TooN::Vector<> clikDH_core_qdot_internal_raw_desired_twist_only(
      const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
      const TooN::Vector<> &qDHdot_secondary_obj) const;
  /*!
  Versione interna dell'algoritmo
  prende quasi tutto come perametro
  Non considera il generatore dell'obj secondario
  Implementa i fixed joint
  */
  virtual TooN::Vector<>
  clikDH_core_qdot_internal(const TooN::Vector<> &error,
                            const TooN::Matrix<> &jacob,
                            const TooN::Vector<> &veld,
                            const TooN::Vector<> &qDHdot_secondary_obj) const;

  /*!
      Versione interna dell'algoritmo, come clikDH_core_qdot_internal
      ma considera solo la velocità desiderata (errore è implicitamente nullo)
  */
  virtual TooN::Vector<> clikDH_core_qdot_internal_desired_twist_only(
      const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
      const TooN::Vector<> &qDHdot_secondary_obj) const;

  virtual void safetyCheck(const TooN::Vector<> &qDH,
                           const TooN::Vector<> &qDH_dot) const;

  virtual bool exceededHardJointLimits(const TooN::Vector<> &qDH) const = 0;
  virtual bool
  exceededHardJointVelLimits(const TooN::Vector<> &qDH_dot) const = 0;
  virtual void checkHardJointVelLimits(const TooN::Vector<> &qDH_dot) const = 0;
  virtual bool exceededSoftJointLimits(const TooN::Vector<> &qDH) const = 0;
  virtual bool
  exceededSoftJointVelLimits(const TooN::Vector<> &qDH_dot) const = 0;

  virtual TooN::Vector<> getClikError(const TooN::Vector<> &q_DH) = 0;
  virtual TooN::Matrix<> getClikJacobian(const TooN::Vector<> &q_DH) = 0;
  virtual TooN::Vector<>
  getDesiredCartesianTwist(const TooN::Vector<> &q_DH) = 0;
};

} // namespace sun
#endif