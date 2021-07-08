#ifndef SUN_CLIK_6D_WITH_QUATERNION_SINGLE_ROBOT_H
#define SUN_CLIK_6D_WITH_QUATERNION_SINGLE_ROBOT_H

#include "sun_robot_lib/ClikSingleRobot.h"

namespace sun {

/////////////////////////////////////////////////
class Clik6DQuaternionSingleRobot : public ClikSingleRobot {

private:
protected:
  //! For the continuity
  UnitQuaternion currentQuaternion_;

public:
  TooN::Vector<3> desiredPosition_;
  UnitQuaternion desiredQuaternion_;

  TooN::Vector<3> desiredLinearVelocity_;
  TooN::Vector<3> desiredAngularVelocity_;

  void resetCurrentQaternion();

  /*=========CONSTRUCTORS=========*/

  Clik6DQuaternionSingleRobot(const std::shared_ptr<Robot> &robot);

  /*========GETTERS============*/

  /*========SETTERS============*/

  virtual void resetDesiredCartesianTwist();

  /*========CLIK=========*/

  virtual TooN::Vector<> getClikError(const TooN::Vector<> &q_DH) override;
  virtual TooN::Matrix<> getClikJacobian(const TooN::Vector<> &q_DH) override;
  virtual TooN::Vector<>
  getDesiredCartesianTwist(const TooN::Vector<> &q_DH) override;
};

} // namespace sun
#endif