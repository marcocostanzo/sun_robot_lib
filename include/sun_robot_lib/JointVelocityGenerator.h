#ifndef SUN_ROBOT_LIB_JOINT_VELOCITY_GENERATOR_H
#define SUN_ROBOT_LIB_JOINT_VELOCITY_GENERATOR_H

#include "TooN/TooN.h"

namespace sun {

/*!
    Generatore di velocità di giunto a partire dallo stato del robot
*/
class JointVelocityGenerator {

private:
protected:
public:
  virtual TooN::Vector<>
  generateJointVelocityDH(const TooN::Vector<> &q_DH) = 0;
};

} // namespace sun

#endif