
#include "sun_robot_lib/Clik.h"
#include <algorithm>

namespace sun {
/*=========CONSTRUCTORS=========*/

/*!
        Default constructor
    */
Clik::Clik(){};

/*========GETTERS============*/

bool Clik::allJointsAreActive() const { return fixed_joints_.empty(); }

TooN::Vector<>
Clik::jointVelAddZerosForUnusedJoints(const TooN::Vector<> &qVel) const {
  std::vector<double> qVel_std = getSTDFromTooN(qVel);

  for (int i = 0; i < fixed_joints_.size(); i++) {
    auto it = qVel_std.begin();
    qVel_std.insert(it + fixed_joints_[i], 0.0);
  }

  return getTooNFromSTD(qVel_std);
}

TooN::Vector<>
Clik::jointVelRemoveUnusedJonts(const TooN::Vector<> &qVel) const {
  std::vector<double> qVel_std = getSTDFromTooN(qVel);
  auto it = qVel_std.begin();
  int num_removed = 0;
  for (int i = 0; i < fixed_joints_.size(); i++) {
    qVel_std.erase(it + fixed_joints_[i] - num_removed);
    num_removed++;
  }
  return getTooNFromSTD(qVel_std);
}

TooN::Matrix<>
Clik::jacobianRemoveUnusedJonts(const TooN::Matrix<> &jacob) const {
  // very inefficient
  std::vector<std::vector<double>> jacob_std_cols = getSTDClumnsFromTooN(jacob);

  auto it = jacob_std_cols.begin();
  int num_removed = 0;
  for (int i = 0; i < fixed_joints_.size(); i++) {
    jacob_std_cols.erase(it + fixed_joints_[i] - num_removed);
    num_removed++;
  }
  return getTooNFromSTDClumns(jacob_std_cols);
}

/*========SETTERS============*/

void Clik::setGainNullSpace(double gain_null_space) {
  if (gain_null_space < 0) {
    throw std::runtime_error("gain null space should not be negative");
  }
  gain_null_space_ = gain_null_space;
}

void Clik::setGainError(double gain_error) {
  if (gain_error < 0) {
    throw std::runtime_error("gain_error should not be negative");
  }
  gain_error_ = gain_error;
}

void Clik::setDLSJointSpeedSaturation(double dls_joint_speed_saturation) {
  dls_joint_speed_saturation_ = dls_joint_speed_saturation;
}

void Clik::setFixedJoints(const std::vector<unsigned int> &fixed_joints) {
  fixed_joints_ = fixed_joints;
  std::sort(fixed_joints_.begin(), fixed_joints_.end());
}

/*========CLIK=========*/

TooN::Vector<> Clik::clikDH_core_qdot_internal_raw(
    const TooN::Vector<> &error, const TooN::Matrix<> &jacob,
    const TooN::Vector<> &veld,
    const TooN::Vector<> &qDHdot_secondary_obj) const {

  // Method without the DLS
  // SVD<> J_svd(jacob);
  // qpDH = J_svd.backsub(veld + gain * error,
  // ROBOT_CLIK_NO_DLS_CONDITION_NUMBER);

  // Method with the DLS
  TooN::Vector<> vel_e = (veld + gain_error_ * error);
  double damping = norm(vel_e) / dls_joint_speed_saturation_;
  TooN::Matrix<> J_pinv_dls = pinv_DLS(jacob, damping);
  TooN::Vector<> qpDH = J_pinv_dls * vel_e;

  // Null space
  if (gain_null_space_ != 0.0) {
    qpDH += gain_null_space_ * nullSpaceProj(jacob, J_pinv_dls) *
            qDHdot_secondary_obj;
  }

  return qpDH;
}

TooN::Vector<> Clik::clikDH_core_qdot_internal_raw_desired_twist_only(
    const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
    const TooN::Vector<> &qDHdot_secondary_obj) const {

  // Method without the DLS
  // SVD<> J_svd(jacob);
  // qpDH = J_svd.backsub(veld + gain * error,
  // ROBOT_CLIK_NO_DLS_CONDITION_NUMBER);

  // Method with the DLS
  TooN::Vector<> vel_e = (veld);
  double damping = norm(vel_e) / dls_joint_speed_saturation_;
  TooN::Matrix<> J_pinv_dls = pinv_DLS(jacob, damping);
  TooN::Vector<> qpDH = J_pinv_dls * vel_e;

  // Null space
  if (gain_null_space_ != 0.0) {
    qpDH += gain_null_space_ * nullSpaceProj(jacob, J_pinv_dls) *
            qDHdot_secondary_obj;
  }

  return qpDH;
}

TooN::Vector<> Clik::clikDH_core_qdot_internal(
    const TooN::Vector<> &error, const TooN::Matrix<> &jacob,
    const TooN::Vector<> &veld,
    const TooN::Vector<> &qDHdot_secondary_obj) const {
  if (allJointsAreActive()) {
    return clikDH_core_qdot_internal_raw(error, jacob, veld,
                                         qDHdot_secondary_obj);
  } else {
    // TODO: the following use of std::vector is very inefficient
    return jointVelAddZerosForUnusedJoints(clikDH_core_qdot_internal_raw(
        error, jacobianRemoveUnusedJonts(jacob), veld,
        jointVelRemoveUnusedJonts(qDHdot_secondary_obj)));
  }
}

TooN::Vector<> Clik::clikDH_core_qdot_internal_desired_twist_only(
    const TooN::Matrix<> &jacob, const TooN::Vector<> &veld,
    const TooN::Vector<> &qDHdot_secondary_obj) const {
  if (allJointsAreActive()) {
    return clikDH_core_qdot_internal_raw_desired_twist_only(
        jacob, veld, qDHdot_secondary_obj);
  } else {
    // TODO: the following use of std::vector is very inefficient
    return jointVelAddZerosForUnusedJoints(
        clikDH_core_qdot_internal_raw_desired_twist_only(
            jacobianRemoveUnusedJonts(jacob), veld,
            jointVelRemoveUnusedJonts(qDHdot_secondary_obj)));
  }
}

TooN::Vector<> Clik::generateJointVelocityDH(const TooN::Vector<> &q_DH) {
  if (b_desired_twist_only_) {
    if (secondObjQdotDHgenerator_) {
      return clikDH_core_qdot_internal_desired_twist_only(
          getClikJacobian(q_DH), getDesiredCartesianTwist(q_DH),
          secondObjQdotDHgenerator_->generateJointVelocityDH(q_DH));
    } else {
      return clikDH_core_qdot_internal_desired_twist_only(
          getClikJacobian(q_DH), getDesiredCartesianTwist(q_DH),
          TooN::Zeros(q_DH.size()));
    }
  } else {
    if (secondObjQdotDHgenerator_) {
      return clikDH_core_qdot_internal(
          getClikError(q_DH), getClikJacobian(q_DH),
          getDesiredCartesianTwist(q_DH),
          secondObjQdotDHgenerator_->generateJointVelocityDH(q_DH));
    } else {
      return clikDH_core_qdot_internal(
          getClikError(q_DH), getClikJacobian(q_DH),
          getDesiredCartesianTwist(q_DH), TooN::Zeros(q_DH.size()));
    }
  }
}

void Clik::safetyCheck(const TooN::Vector<> &qDH,
                       const TooN::Vector<> &qDH_dot) const {
  if (TooN::isnan(qDH)) {
    throw robot::ExceededJointLimits("Joint NaN : " + to_string(qDH));
  }
  if (!TooN::isfinite(qDH)) {
    throw robot::ExceededJointLimits("Joint Inf : " + to_string(qDH));
  }
  if (TooN::isnan(qDH_dot)) {
    throw robot::ExceededJointLimits("JointVel NaN : " + to_string(qDH_dot));
  }
  if (!TooN::isfinite(qDH_dot)) {
    throw robot::ExceededJointLimits("JointVel Inf : " + to_string(qDH_dot));
  }
  if (b_checkHardJointLimits_ && exceededHardJointLimits(qDH)) {
    throw robot::ExceededJointLimits("HardJointLimits");
  }
  if (b_checkHardJointVelLimits_ && exceededHardJointVelLimits(qDH_dot)) {
    throw robot::ExceededJointLimits("HardJointVelLimits");
  }
  if (b_checkSoftJointLimits_ && exceededSoftJointLimits(qDH)) {
    throw robot::ExceededJointLimits("SoftJointLimits");
  }
  if (b_checkSoftJointVelLimits_ && exceededSoftJointVelLimits(qDH_dot)) {
    throw robot::ExceededJointLimits("SoftJointVelLimits");
  }
}

} // namespace sun