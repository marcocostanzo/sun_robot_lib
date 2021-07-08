#ifndef SUN_CLIK_SINGLE_ROBOT_H
#define SUN_CLIK_SINGLE_ROBOT_H

#include "sun_robot_lib/Clik.h"
#include "sun_robot_lib/Robot.h"

namespace sun {

/////////////////////////////////////////////////
class ClikSingleRobot : public Clik {

private:
protected:
public:
  std::shared_ptr<Robot> robot_;

  /*=========CONSTRUCTORS=========*/

  ClikSingleRobot(const std::shared_ptr<Robot> &robot);

  /*========GETTERS============*/

  /*========SETTERS============*/

  /*========CLIK=========*/

  virtual bool
  exceededHardJointLimits(const TooN::Vector<> &qDH) const override;

  virtual bool
  exceededHardJointVelLimits(const TooN::Vector<> &qDH_dot) const override;

  virtual bool
  exceededSoftJointLimits(const TooN::Vector<> &qDH) const override;

  virtual bool
  exceededSoftJointVelLimits(const TooN::Vector<> &qDH_dot) const override;

  virtual TooN::Vector<> getClikError(const TooN::Vector<> &q_DH) override = 0;
  virtual TooN::Matrix<>
  getClikJacobian(const TooN::Vector<> &q_DH) override = 0;
  virtual TooN::Vector<>
  getDesiredCartesianTwist(const TooN::Vector<> &q_DH) override = 0;
};

} // namespace sun
#endif