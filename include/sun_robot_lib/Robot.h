/*

    Robot Class

    Copyright 2018-2020 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef ROBOT_H
#define ROBOT_H

#include <sun_robot_lib/RobotLinkPrismatic.h>
#include <sun_robot_lib/RobotLinkRevolute.h>
#include <iomanip>
#include "sun_math_toolbox/PortingFunctions.h"
#include "sun_math_toolbox/UnitQuaternion.h"

namespace sun
{
//! The Robot Class
class Robot
{
private:
protected:
  //! Transformation matrix of link_ w.r.t. base frame
  TooN::Matrix<4, 4> _b_T_0;  // T_0^b
  //! Links
  std::vector<RobotLinkPtr> _links;
  //! Transformation matrix of effector w.r.t. link_n frame
  TooN::Matrix<4, 4> _n_T_e;  // T_e^n

  //! Joint speed saturation used in dls for clik
  double _dls_joint_speed_saturation;  // Used in clik

  //! Name of the robot
  std::string _name;

  //! Model of the robot
  std::string _model;

public:
  /*=========CONSTRUCTORS=========*/

  /*!
      Default constructor
      
      Robot with no links
  */
  Robot();

  Robot(const std::string& name);

  /*!
      Full constructor
  */
  Robot(const std::vector<RobotLinkPtr>& links, const TooN::Matrix<4, 4>& b_T_0, const TooN::Matrix<4, 4>& n_T_e,
        double dls_joint_speed_saturation, const std::string& name);

  /*!
      Constuctor without links

      usefull to use robot.push_back_link(...)
  */
  Robot(const TooN::Matrix<4, 4>& b_T_0, const TooN::Matrix<4, 4>& n_T_e, double dls_joint_speed_saturation,
        const std::string& name);
  /*!
      Copy Constructor
  */
  Robot(const Robot& robot);

  /*=====END CONSTRUCTORS=========*/

  /*=======HELPS=========*/

protected:
  /*!
      Internal function that checks if the matrix is Homog and print an error
  */
  static void checkHomog(const TooN::Matrix<4, 4>& M);

public:
  /*!
      Display robot in smart way
  */
  virtual void display();

  /*!
      Display robot position
      Input in DH Convention
  */
  virtual void dispPosition(const TooN::Vector<>& q_DH);

  /*=======END HELPS=====*/

  /*=========GETTERS=========*/

  /*!
      get Joint speed saturation used in dls for clik
  */
  virtual double getDLSJointSpeedSaturation() const;

  /*!
      get number of joints
  */
  virtual int getNumJoints() const;

  /*!
      Get Transformation matrix of link_0 w.r.t. base frame
  */
  virtual TooN::Matrix<4, 4> getbT0() const;

  /*!
      get Vector of links
      this function makes a copy
  */
  virtual std::vector<RobotLinkPtr> getLinks() const;

  /*!
      get reference of link i

      Note: smart_pointer
  */
  virtual RobotLinkPtr& getLink(int i);

  /*!
      Get Transformation matrix of link_0 w.r.t. base frame
  */
  virtual TooN::Matrix<4, 4> getnTe() const;

  /*!
      Get robot name
  */
  virtual std::string getName() const;

  /*!
      Get robot model
  */
  virtual std::string getModel() const;

  /*!
      Get i-th joint name
  */
  virtual std::string getJointName(int i) const;

  /*!
      get a string of joint names given the bitmap
  */
  virtual std::string jointsNameFromBitMask(const std::vector<bool>& jointMask) const;

  /*!
      Clone the object
  */
  virtual Robot* clone() const;

  /*=========END GETTERS=========*/

  /*=========SETTERS=========*/

  /*!
      set Joint speed saturation used in dls for clik
  */
  virtual void setDLSJointSpeedSaturation(double dls_joint_speed_saturation);

  /*!
      Set Transformation matrix of link_0 w.r.t. base frame
  */
  virtual void setbT0(const TooN::Matrix<4, 4>& b_T_0);

  /*!
      Set vector of links
  */
  virtual void setLinks(const std::vector<RobotLinkPtr>& links);

  /*!
      Add a link to the kinematic chain
  */
  virtual void push_back_link(const RobotLink& link);

  /*!
      overloaded operator: Add a link to the kinematic chain
  */
  virtual Robot& operator+=(const RobotLink& link);

  /*!
      overloaded operator: Constuct a new Robot object and add a link to the kinematic chain
  */
  virtual Robot operator+(const RobotLink& link) const;

  /*!
      Remove last link of the chain
  */
  virtual void pop_back_link();

  /*!
      Set Transformation matrix of endeffector w.r.t. link_n frame
  */
  virtual void setnTe(const TooN::Matrix<4, 4>& n_T_e);

  /*!
      Set Robot Name
  */
  virtual void setName(const std::string& name);

  /*!
      Set Robot Model
  */
  virtual void setModel(const std::string& model);

  /*=========END SETTERS=========*/

  /*=========CONVERSIONS=========*/

  /*!
      Transform joints from robot to DH convention
  */
  virtual TooN::Vector<> joints_Robot2DH(const TooN::Vector<>& q_Robot) const;

  /*!
      Transform joints from HD to robot convention
  */
  virtual TooN::Vector<> joints_DH2Robot(const TooN::Vector<>& q_DH) const;

  /*!
      Transform joints velocity from robot to DH convention
  */
  virtual TooN::Vector<> jointsvel_Robot2DH(const TooN::Vector<>& q_dot_Robot) const;

  /*!
      Transform joints from DH to robot convention
  */
  virtual TooN::Vector<> jointsvel_DH2Robot(const TooN::Vector<>& q_dot_DH) const;

  /*=========END CONVERSIONS=========*/

  /*=========SAFETY=========*/

  /*!
      Check Hard Limits
      
      Return a logic vector, if the i-th element is true then the i-th link has violated the limits
  */
  virtual std::vector<bool> checkHardJointLimits(const TooN::Vector<>& q_Robot) const;

  /*!
      Check Hard Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededHardJointLimits(const TooN::Vector<>& q_Robot) const;

  /*!
      Check Soft Limits

      Return a logic vector, if the i-th element is true then the i-th link has violated the limits
  */
  virtual std::vector<bool> checkSoftJointLimits(const TooN::Vector<>& q_Robot) const;

  /*!
      Check Soft Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededSoftJointLimits(const TooN::Vector<>& q_R) const;

  /*!
      Check Hard Velocity Limits

      Return a logic vector, if the i-th element is true then the i-th link has violated the limits
  */
  virtual std::vector<bool> checkHardVelocityLimits(const TooN::Vector<>& q_dot) const;

  /*!
      Check Velocity Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededHardVelocityLimits(const TooN::Vector<>& q_dot) const;

  virtual std::vector<bool> checkSoftVelocityLimits(const TooN::Vector<>& q_dot) const;

  /*!
      Check SOFT Velocity Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededSoftVelocityLimits(const TooN::Vector<>& q_dot) const;

  /*=========END SAFETY=========*/

  /*========FKINE=========*/

protected:
  /*!
      Internal fkine

      This function compute the fkine to joint "n_joint" given the last transformation to joint n_joint-1
      - q_DH_j is the joint position of the i-th link
      - b_T_j_1 is the transformation of the link j-1 w.r.t base frame
  */
  virtual TooN::Matrix<4, 4> fkine_internal(const double& q_DH_j, const TooN::Matrix<4, 4>& b_T_j_1, int n_joint) const;

public:
  /*!
      fkine to n_joint-th link

      j_T_f will be post multiplyed to the result

      if n_joint = NUM_JOINT+1 then the result is b_T_e*j_T_f
  */
  virtual TooN::Matrix<4, 4> fkine(const TooN::Vector<>& q_DH, int n_joint, const TooN::Matrix<4, 4>& j_T_f) const;

  /*!
      fkine to n_joint-th link

      if n_joint = NUM_JOINT+1 then the result is b_T_e
  */
  virtual TooN::Matrix<4, 4> fkine(const TooN::Vector<>& q_DH, int n_joint) const;

  /*!
      fkine to the end-effector
  */
  virtual TooN::Matrix<4, 4> fkine(const TooN::Vector<>& q_DH) const;

  /*!
      The resul of this function is the matrix b_T_f
      where f is a given frame defined by the input e_T_f
      - e_T_f is the transformation of the frame {f} w.r.t. frame {end-effector}
  */
  virtual TooN::Matrix<4, 4> fkine(const TooN::Vector<>& q_DH, const TooN::Matrix<4, 4>& e_T_f) const;

  /*!
      This function return all the transformation up to link "n_joint"

      The return is a vector of size n_joint

      if n_joint=NUM_JOINT+1 then the output will be a vector of size n_joint as well, but the last element is b_T_e
  */
  virtual std::vector<TooN::Matrix<4, 4>> fkine_all(const TooN::Vector<>& q_DH, int n_joint) const;

  /*========END FKINE=========*/

  /*========Jacobians=========*/

protected:
  /*!
      Internal computation of the position part of the jacobian in the frame {f}
      The input is a vector of all transformation the considered joints i.e.
      [ b_T_0 , b_T_1, ... , (b_T_j*j_T_f) ] (size = joints+1)
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_p_internal(const std::vector<TooN::Matrix<4, 4>>& all_T) const;

  /*!
      Internal computation of the orientation part of the geometric jacobian in the frame {f}
      The input is a vector of all transformation the considered joints i.e.
      [ b_T_0 , b_T_1, ... , (b_T_j*j_T_f) ] (size = joints+1)
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_o_geometric_internal(const std::vector<TooN::Matrix<4, 4>>& all_T) const;

public:
  /*!
      Compute the position part of the jacobian in frame {f} w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      The matrix j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t. frame of the joint n_joint
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_p(const TooN::Vector<>& q_DH, int n_joint,
                                                 const TooN::Matrix<4, 4>& j_T_f) const;

  /*!
      Compute the position part of the jacobian in frame of joint n_joint w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_p(const TooN::Vector<>& q_DH, int n_joint) const;

  /*!
      Compute the position part of the jacobian in frame {end-effector} w.r.t. base frame (pag 111)
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_p(const TooN::Vector<>& q_DH) const;

  /*!
      Compute the orientation part of the geometric jacobian in frame {f} w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      The matrix j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t. frame of the joint n_joint
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_o_geometric(const TooN::Vector<>& q_DH, int n_joint,
                                                           const TooN::Matrix<4, 4>& j_T_f) const;

  /*!
      Compute the orientation part of the geometric jacobian in frame of joint n_joint w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_o_geometric(const TooN::Vector<>& q_DH, int n_joint) const;

  /*!
      Compute the orientation part of the geometric jacobian in frame {end-effector} w.r.t. base frame (pag 111)
  */
  virtual TooN::Matrix<3, TooN::Dynamic> jacob_o_geometric(const TooN::Vector<>& q_DH) const;

  /*!
      Compute the geometric jacobian in frame {f} w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      The matrix j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t. frame of the joint n_joint
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual TooN::Matrix<6, TooN::Dynamic> jacob_geometric(const TooN::Vector<>& q_DH, int n_joint,
                                                         const TooN::Matrix<4, 4>& j_T_f) const;

  /*!
      Compute the geometric jacobian in frame of joint n_joint w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual TooN::Matrix<6, TooN::Dynamic> jacob_geometric(const TooN::Vector<>& q_DH, int n_joint) const;

  /*!
      Compute the geometric jacobian in frame {end-effector} w.r.t. base frame (pag 111)
  */
  virtual TooN::Matrix<6, TooN::Dynamic> jacob_geometric(const TooN::Vector<>& q_DH) const;

  /*!
      Ginven the jacobian b_J in frame {b} and the rotation matrix u_R_b of frame {b} w.r.t. frame {u},
      compute the jacobian w.r.t frame {u} (pag 113)
      The jacobian b_J can be the position part (3xQ), the orientation part (3xQ) or the full jacobian (6xQ)
  */
  static TooN::Matrix<> change_jacob_frame(TooN::Matrix<> b_J, const TooN::Matrix<3, 3>& u_R_b);

  /*========END Jacobians=========*/

  /*========CLIK=========*/

  /*!
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
          - q0_dot: velocity to be projected into the null space

      Outputs:
          - return: qDH_k+1 joints at time k+1
          - qpDH: joints velocity at time k+1
  */
  virtual TooN::Vector<> clik(const TooN::Vector<>& qDH_k, const TooN::Vector<6>& error, const TooN::Matrix<>& jacob,
                              const TooN::Vector<6>& veld, double gain, double Ts, double gain_null_space,
                              const TooN::Vector<>& q0_dot,
                              // Return Vars
                              TooN::Vector<>& qpDH);

  /*!
      Clik using Quaternions FULL VERSION
      
      Inputs:
          - qDH_k: joints at time k
          - pd: desired posotion
          - Qd: deisred quaternion
          - oldQ: last quaternion at time k-1 (needed for continuity)
          - dpd: desired position velocity
          - omegad: desired angular velocity
          - mask: bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the
     
     error computation
          - gain: CLIK Gain
          - Ts: sampling time
          - gain_null_space: Gain for second objective
          - q0_p: velocity to be projected into the null space
      
      Outputs:
          - return: qDH_k+1 joints at time k+1
          - qpDH: joints velocity at time k+1
          - error: error vector at time k
          - newQ: Quaternion at time k (usefull for continuity in the next call of these function)
  */
  virtual TooN::Vector<> clik(const TooN::Vector<>& qDH_k, const TooN::Vector<3>& pd, const UnitQuaternion& Qd,
                              const UnitQuaternion& oldQ, const TooN::Vector<3>& dpd, const TooN::Vector<3>& omegad,
                              const TooN::Vector<6, int>& mask, double gain, double Ts, double gain_null_space,
                              const TooN::Vector<>& q0_p,
                              // Return Vars
                              TooN::Vector<>& qpDH, TooN::Vector<6>& error, UnitQuaternion& newQ);

  /*!
      Clik using Quaternions

      Inputs:
          - qDH_k: joints at time k
          - pd: desired posotion
          - Qd: deisred quaternion
          - oldQ: last quaternion at time k-1 (needed for continuity)
          - dpd: desired position velocity
          - omegad: desired angular velocity
          - gain: CLIK Gain
          - Ts: sampling time
          - gain_null_space: Gain for second objective
          - q0_p: velocity to be projected into the null space
      
      Outputs:
          - return: qDH_k+1 joints at time k+1
          - qpDH: joints velocity at time k+1
          - error: error vector at time k
          - newQ: Quaternion at time k (usefull for continuity in the next call of these function)
  */
  virtual TooN::Vector<> clik(const TooN::Vector<>& qDH_k,  // Actual joints positions
                              const TooN::Vector<3>& pd, const UnitQuaternion& Qd,
                              const UnitQuaternion& oldQ,  // For Quaternion Continuity
                              const TooN::Vector<3>& dpd, const TooN::Vector<3>& omegad,
                              double gain,             // CLIK Gain
                              double Ts,               // sampling time
                              double gain_null_space,  // Gain for second objective
                              const TooN::Vector<>& q0_p,
                              // Return Vars
                              TooN::Vector<>& qpDH, TooN::Vector<6>& error, UnitQuaternion& newQ);

  /*!
      Clik using Quaternions, the null space is used to maximize distance from soft joints limits
      
      Inputs:
          - qDH_k: joints at time k
          - pd: desired posotion
          - Qd: deisred quaternion
          - oldQ: last quaternion at time k-1 (needed for continuity)
          - dpd: desired position velocity
          - omegad: desired angular velocity
          - mask: bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the
     
     error computation
          - gain: CLIK Gain
          - Ts: sampling time
          - gain_null_space: Gain for second objective
          - desired_configuration: target for joint position (used into the second objective obj)
          - desired_configuration_joint_weights: weights for joints in the second objective
      
      Outputs:
          - return: qDH_k+1 joints at time k+1
          - qpDH: joints velocity at time k+1
          - error: error vector at time k
          - newQ: Quaternion at time k (usefull for continuity in the next call of these function)
  */
  virtual TooN::Vector<> clik(const TooN::Vector<>& qDH_k, const TooN::Vector<3>& pd, const UnitQuaternion& Qd,
                              const UnitQuaternion& oldQ, const TooN::Vector<3>& dpd, const TooN::Vector<3>& omegad,
                              const TooN::Vector<6, int>& mask, double gain, double Ts, double gain_null_space,
                              const TooN::Vector<>& desired_configuration,
                              const TooN::Vector<>& desired_configuration_joint_weights,
                              // Return Vars
                              TooN::Vector<>& qpDH, TooN::Vector<6>& error, UnitQuaternion& newQ);

  /*!
      Clik using Quaternions, the null space is used to maximize distance from soft joints limits
      
      Inputs:
          - qDH_k: joints at time k
          - pd: desired posotion
          - Qd: deisred quaternion
          - oldQ: last quaternion at time k-1 (needed for continuity)
          - dpd: desired position velocity
          - omegad: desired angular velocity
          - gain: CLIK Gain
          - Ts: sampling time
          - gain_null_space: Gain for second objective
          - desired_configuration: target for joint position (used into the second objective obj)
          - desired_configuration_joint_weights: weights for joints in the second objective
      
      Outputs:
          - return: qDH_k+1 joints at time k+1
          - qpDH: joints velocity at time k+1
          - error: error vector at time k
          - newQ: Quaternion at time k (usefull for continuity in the next call of these function)
  */
  virtual TooN::Vector<> clik(const TooN::Vector<>& qDH_k, const TooN::Vector<3>& pd, const UnitQuaternion& Qd,
                              const UnitQuaternion& oldQ, const TooN::Vector<3>& dpd, const TooN::Vector<3>& omegad,
                              double gain, double Ts, double gain_null_space,
                              const TooN::Vector<>& desired_configuration,
                              const TooN::Vector<>& desired_configuration_joint_weights,
                              // Return Vars
                              TooN::Vector<>& qpDH, TooN::Vector<6>& error, UnitQuaternion& newQ);

  /*========END CLIK=========*/

  /*====== COST FUNCTIONS FOR NULL SPACE ======*/

  /*!
      Gradient of cost function to minimize the distance from joints centers

      Inputs:
          -q_DH: joint positions
          -desired_configuration: center of joints
          -desired_configuration_joint_weights: weigths for the joints
  */
  virtual TooN::Vector<> grad_fcst_target_configuration(const TooN::Vector<>& q_DH,
                                                        const TooN::Vector<>& desired_configuration,
                                                        const TooN::Vector<>& desired_configuration_joint_weights);

  /*====== END COST FUNCTIONS FOR NULL SPACE ======*/

};  // END CLASS

using RobotPtr = std::unique_ptr<Robot>;

/*==========Operators========*/

/*!
  overloaded operator +

  Construct a new Robot object with link1 as the first link and link2 as second link
*/
Robot operator+(const RobotLink& link1, const RobotLink& link2);

}  // namespace sun

#endif