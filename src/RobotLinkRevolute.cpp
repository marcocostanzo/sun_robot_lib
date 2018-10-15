

#include "RobotLinkRevolute.h"

using namespace TooN;
using namespace std;



        /*=============CONSTRUCTORS===========*/

        //Full Constructor
        RobotLinkRevolute::RobotLinkRevolute(   double a, double alpha, double d, double theta, 
                                                double offset, bool flip, 
                                                double robot2dh_offset, double robot2dh_sign, 
                                                double DHJoint_limit_lower, double DHJoint_limit_higher, 
                                                double RobotJoint_limit_lower, double RobotJoint_limit_higher, 
                                                double velocity_limit_lower, double velocity_limit_higher,
                                                string name ):
                                                RobotLink(  a, alpha, d, theta, 
                                                            offset, flip, 
                                                            robot2dh_offset, robot2dh_sign, 
                                                            DHJoint_limit_lower, DHJoint_limit_higher, 
                                                            RobotJoint_limit_lower, RobotJoint_limit_higher, 
                                                            velocity_limit_lower, velocity_limit_higher,
                                                            name )
                                                {}

        RobotLinkRevolute::RobotLinkRevolute( double a, double alpha, double d, double theta, double offset, bool flip):
            RobotLink( a, alpha, d, theta, offset, flip)
            {}

        RobotLinkRevolute::RobotLinkRevolute( double a, double alpha, double d, double theta):
            RobotLink( a, alpha, d, theta)
            {}

        /*=======END CONSTRUCTORS===========*/

        /*
                return the link angle
                Note:
                    - For revolute link this is the joint variable,
                    in that case this function returns NaN
        */
        double RobotLinkRevolute::getDH_theta() const{
            return NAN;
        }

        /*
                TODO
                ERROR IF LINK IS REVOLUTE
        */
        void RobotLinkRevolute::setDH_theta( double theta ){
            cout << ROBOT_ERROR_COLOR "[RobotLinkRevolute] Error in setDH_theta( double theta ): Cannot set theta for RobotLinkRevolute" ROBOT_CRESET << endl;
            exit(-1);
        }

        /*
            TODO
        */
        void RobotLinkRevolute::display() const{
            cout << _name << " RobotLinkRevolute TODO" << endl;
        }

        /*
                Retrun the joint type
                'p' = prismatic
                'r' = revolute
        */
        char RobotLinkRevolute::type() const {
            return 'r';
        }

        /*
            Compute the link transform matrix
        */
        Matrix<4,4> RobotLinkRevolute::A( double q_DH ) const{

             q_DH = DH_revert_offset(q_DH);

             return A_internal( q_DH, _d );

        }


bool isRevolute( const RobotLink& l ){
    return ( l.type() == 'r' );
}
