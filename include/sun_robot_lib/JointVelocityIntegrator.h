#ifndef SUN_ROBOT_LIB_JOINT_VELOCITY_INTEGRATOR_H
#define SUN_ROBOT_LIB_JOINT_VELOCITY_INTEGRATOR_H

#include "TooN/TooN.h"
#include "sun_robot_lib/JointVelocityGenerator.h"
#include <memory>

namespace sun {

/*!
    Generatore di velocità di giunto a partire dallo stato del robot
*/
class JointVelocityIntegrator {

private:
protected:
  TooN::Vector<> qDH_k_;
  TooN::Vector<> qDHdot_k_;

public:
  std::shared_ptr<JointVelocityGenerator> jointVelocityGenerator_;

  JointVelocityIntegrator(const TooN::Vector<> &qDH0)
      : qDH_k_(qDH0), qDHdot_k_(TooN::Zeros(qDH0.size())) {}

  double Ts_ = 1E-3;

  std::shared_ptr<JointVelocityGenerator> &operator->() {
    return jointVelocityGenerator_;
  }

  virtual const TooN::Vector<> &getJointsDH() const { return qDH_k_; }

  virtual const TooN::Vector<> &getJointsVelDH() const { return qDHdot_k_; }

  virtual void setJointsDH(const TooN::Vector<> &qDH) { qDH_k_ = qDH; }

  virtual void setJointsVelDH(const TooN::Vector<> &qDHdot) {
    qDHdot_k_ = qDHdot;
  }

  virtual void resetJointsVelDH() { qDHdot_k_ = TooN::Zeros; }

  virtual void exec_single_step() {
    qDHdot_k_ = jointVelocityGenerator_->generateJointVelocityDH(qDH_k_);

    qDH_k_ = (qDH_k_ + qDHdot_k_ * Ts_);
  }
};

} // namespace sun

#endif