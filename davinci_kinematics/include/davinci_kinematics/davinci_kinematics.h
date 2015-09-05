/* 
 * File:   davinci_kinematics.h
 * Author: wsn
 *
 * Created Sept 2, 2015
 */



#ifndef DAVINCI_KIN_H
#define	DAVINCI_KIN_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>

using namespace std;


typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
//define D-H frames;
// "base" frame is same as "one_psm_base_link"
//  O_base is at pivot (trocar) point
//  z-axis points "up", y-axis points forward, x-axis is to robot's right

// DH-0 frame: this is a static transform from base frame, but conforms to
// DH convention that z0 is through first joint axis.  
// get freedom to choose orientation of x0, y0 spin about z0
// O_0 = O_base
// + rotation of q[0]==q1 corresponds to "leaning to the left", -->
// z_0 points along -y axis of base frame;

// choose x0 pointing "down", so yaw home will agree with davinci q[0]
// then y0 points along +x_base
// R_{0/base} = [0  1  0
//               0  0 -1
//              -1  0  0 ]

// DH-1 frame:  this frame moves w/ yaw joint (theta1 or q(0))
// construct from z_0 crossed into z_1==> x_1
// z_1 axis points to left (positive rotation of q[1]==q2 ==> pitch "leans forward")
// + rotation is about z-axis pointing to the left, i.e. coincident w/ -x0 (in home position)
// cross z_0 into z_1 ==> x1 axis points "down" in home position
// by construction, alpha1 = +pi/2.
// O_1 = O_0 ==> a1=0, d1=0
// --> need a psi_0 offset = +/- pi/2 so davinci home (in psi coords) has tool shaft parallel to z_base
// psi[0] = theta1_DH- pi/2


// DH-2 frame: construct from z1 (pitch axis) and z2 (prismatic axis)
// choose prismatic axis pointing through tool-shaft centerline, towards gripper
// choose origin O_2 to lie on z1 at intersection w/ wrist-bend joint
// define d2=0 such that O_2 is at O_base = O_1 = O_2
// from home pose, z1 is to the left, and z2 is down, so x2 points inwards, towards robot
// but x1 axis points "down" at Davinci home pose;
// this corresponds to a theta3 of +pi/2...and this value is static;
// by construction, alpha_2 = +pi/2
// a2=0, 
// d2 is a variable (prismatic joint displacement)
// 

// DH-3 frame: z axis is spin about tool shaft, coincident w/ displacement axis z3
// for + rotation, z3 points same direction as z2, so alpha3 = 0
// choose origin coincident: O_3 = O_2, so a3 = d3 = 0
// get freedom to choose frame orientation of x3 (perpendicular to z3)
// define x3 direction such that thetaDH4 = q[3] (no offset correction needed; in agreement at home pose)
// at home pose, x3 points towards robot--coincident w/ x2 of prismatic jnt


// DH-4 frame: construct from tool-shaft spin axis, z3 and wrist-bend z4 axes
// by choice of origin for O3, have O_3 = O_4
// --> a4=0, d4=0
// by construction, x4 = z3 crossed into z4 and alpha4= +pi/2
// for plus wrist bend, at home pose, z4 points to "left"
// so, z3 cross z4 = x4 points FORWARD
// this is pi away from x3, so need offset such that q[3]=0 when thetaDH_4 = pi


// DH-5 frame: z5 is through gripper-jaw rotation axis
// at home pose, z5 points "IN"...s.t. + rotation causes gripper jaws to point to right
// O_5 is on the z5 axis, offset from O_4
// z4 and z5 do not intersect.  Have a non-zero a5 offset
// min dist from z4 to z5 defines x5; points from O4 to O_5
// +rotation direction of jaw rotation--> alpha5 is - pi/2
// d5 = 0
// need to find theta5 (wrist-bend) offset for Davinci home

// DH-6 frame: choose a final gripper frame (don't have two axes to construct)
// try z6 pointing out from z5 axis at +/- pi/2 = alpha6;
// set coincident origin, a6 = d6 = 0; O_6 = O_5 (then use separate transform to gripper tip, if desired)
// find theta6 offset to conform w/ Davinci home

//frames: 
 //world->
 //   one_psm_base_link->
 //     one_outer_yaw_link-->
 //        one_outer_pitch_link-->
 //          one_tool_main_link-->
 //            one_tool_wrist_link-->
 //               one_tool_wrist_shaft_link-->
 //                  one_tool_wrist_sca_link--> z-axis through wrist bend axis, but moves w/ wrist bend
 //                     one_tool_wrist_sca_shaft_link-->: z-axis thru gripper jaw rot axis
 //                       one_tool_tip_link

//frames:  base has x-axis forward, z up, origin at mount plate;
// sequentially, from torso to tip, frames are:
//right_upper_shoulder has z-axis through S0, x-axis at 45-deg (parallel to home pose of outstretched right arm)
//right_lower_shoulder has z-axis through S1, x-axis points forward along humerus axis (outstretched in home pose)
//  origin is "shoulder point"
//right_upper_elbow: z-axis through humerus rotation (E0), and x-axis points DOWN (in home pose);
// origin is offset along (self) z-axis relative to parent frame
//right_lower_elbow: origin at "elbow point" w/ z-axis along elbow bend (E1), w/ x-axis FORWARD (at home)
//right_upper_forearm: origin offset from above (along self z); z-axis along forearm rot (W0)
// right lower_forearm: has origin at "wrist point"; z-axis through wrist bend (W1)
//right_wrist: z-axis points OUT from tool flange--but orgin is offset (btwn wrist pt and flange face); W2 axis
//right_hand: origin on flange face, z-axis points out; x-axis is DOWN (at home)

// at "home" angles (all zeros, w/rt Baxter commands), arms are outstretched w/ first shoulder jnt (S0)
// such that arms are at 45deg from forward (making a 90-deg angle w/rt each other)
// and elbow offsets are BELOW humerus axes



const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis = 0.0091;
const double gripper_jaw_length = 0.0102; 

const double DH_a1=0.0; //origin0 coincident w/ origin1
const double DH_a2=0.0; // axis z1,z2 intersect
const double DH_a3=0.0; //axes z2 (prismatic) and z3 (shaft rot) intersect
const double DH_a4=0.0; //axes z3 (shaft rot) and z4 (wrist bend) intersect
// axes z4 (wrist bend) and z5 (gripper-jaw rot axis) do NOT intersect:
const double DH_a5=dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis;  
const double DH_a6=0.0; // define tool frame on jaw w/ z6 axis intersecting z5 axis

const double DH_a7=0.0; //not sure what to do with this one


const double DH_d1 = 0.0;// 
const double DH_d2 = 0.0;//  THIS IS VARIABLE
const double DH_d3 = 0.0; 
const double DH_d4 = 0.0;
const double DH_d5 = 0.0; 
const double DH_d6 = 0.0;
const double DH_d7 = 0.0; 

//robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
const double DH_alpha1 = M_PI/2.0;
const double DH_alpha2 = M_PI/2.0;
const double DH_alpha3 = 0.0; // prismatic axis is aligned with tool-shaft spin axis 
const double DH_alpha4 = M_PI/2.0;
const double DH_alpha5 = -M_PI/2.0; // offset from wrist bend to jaw bend axis--> alpha is defined -pi/2
const double DH_alpha6 = M_PI/2.0; // choose gripper frame to have z-axis pointing along jaws
const double DH_alpha7 = 0.0; // relevant only if treat jaws as separate links...


// q_Davinci vec: starts counting from 0; 0 displacement at model "home"
//  when model joint_states are all at displacement 0.0
// use these offsets to convert to DH coords

const double insertion_offset=0.0156; //must command this much displacement to get wrist-bend axis to intersect base origin

const double DH_q_offset0 = 0.0;
const double DH_q_offset1 = M_PI/2.0;
const double DH_q_offset2 = -insertion_offset;
const double DH_q_offset3 = M_PI;
const double DH_q_offset4 = M_PI/2;
const double DH_q_offset5 = M_PI/2; //M_PI;
const double DH_q_offset6 = 0.0; //M_PI;

const double deg2rad = M_PI/180.0;

// NEED TO FIND THESE:
const double DH_q_max0 = deg2rad*95; //141; //51;
const double DH_q_max1 = deg2rad*60;
const double DH_q_max2 = deg2rad*173.5;
const double DH_q_max3 = deg2rad*150;
const double DH_q_max4 = deg2rad*175.25;
const double DH_q_max5 = deg2rad*120; //
const double DH_q_max6 = deg2rad*175.25;

//-141, -123, -173.5, -3, -175.25, -90, -175.25
const double DH_q_min0 = -deg2rad*95; //51; //141;
const double DH_q_min1 = -deg2rad*123;
const double DH_q_min2 = -deg2rad*173.5; 
const double DH_q_min3 = -deg2rad*3;
const double DH_q_min4 = -deg2rad*175.25;
const double DH_q_min5 = -deg2rad*90; //
const double DH_q_min6 = -deg2rad*175.25;

const double DH_a_params[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
double DH_d_params[7] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6,DH_d7};
const double DH_alpha_params[7] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6,DH_alpha7};
const double DH_q_offsets[7] = {DH_q_offset0,DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6};
const double q_lower_limits[7] = {DH_q_min0,DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6};
const double q_upper_limits[7] = {DH_q_max0,DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6};


class Davinci_fwd_solver {
public:
    Davinci_fwd_solver(); //constructor
    
    // represent transforms as affine3d objects
    Eigen::Affine3d affine_frame0_wrt_base_; 
    Eigen::Affine3d affine_gripper_wrt_frame6_;    
    Eigen::Affine3d affine_gripper_wrt_base_; 
    vector <Eigen::Affine3d> affines_i_wrt_iminus1_;
    vector <Eigen::Affine3d> affine_products_;
    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;
    Eigen::VectorXd theta_DH_offsets_,dval_DH_offsets_;
    
    Eigen::MatrixXd Jacobian_; //not used/implemented yet    

    tf::TransformListener *tfListener_ptr_; //pointer to a transform listener
    // some translation utilities
    Eigen::Affine3f transformTFToEigen(const tf::Transform &t); 
    Eigen::Affine3f stampedTFToEigen(const tf::StampedTransform &t);
    Eigen::Affine3d transformTFToAffine3d(const tf::Transform &t); 
    Eigen::Affine3d stampedTFToAffine3d(const tf::StampedTransform &t);
    
    geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e);
    geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
    
    //generic: provide DH values, compute transform, expressed as Eigen::Affine3d
    void convert_qvec_to_DH_vecs(const Vectorq7x1& q_vec);
    Vectorq7x1 convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec, const Eigen::VectorXd &dvals_DH_vec);

    Eigen::Affine3d computeAffineOfDH(double a, double d, double alpha, double theta);
    
    // provide joint angles and prismatic displacement (q_vec[2]) w/rt DaVinci coords
    // will get translated to DH coords to solve fwd kin
    // return affine describing gripper pose w/rt base frame
    Eigen::Affine3d fwd_kin_solve(const Vectorq7x1& q_vec);  
    //the following version takes args of DH thetas and d's; used by above fnc
    Eigen::Affine3d fwd_kin_solve_DH(const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& d_vec);

    
    Eigen::Affine3d get_affine_frame(int i) { return affine_products_[i]; }; // return affine of frame i w/rt base
     
};

//IK class derived from FK class:
class Davinci_IK_solver:Davinci_fwd_solver  {
public:
    Davinci_IK_solver(); //constructor; 

    // return the number of valid solutions; actual vector of solutions will require an accessor function
    void get_solns(std::vector<Vectorq7x1> &q_solns);
    bool fit_joints_to_range(Vectorq7x1 &qvec);
    Eigen::Vector3d q123_from_wrist(Eigen::Vector3d wrist_pt);
    Eigen::Vector3d compute_w_from_tip(Eigen::Affine3d affine_gripper_tip, Eigen::Vector3d &zvec_4);
    int  ik_solve(Eigen::Affine3d const& desired_hand_pose);
    Vectorq7x1 get_soln() {return q_vec_soln_;};

private:
    bool fit_q_to_range(double q_min, double q_max, double &q);    
    std::vector<Vectorq7x1> q7dof_solns_;
    std::vector<Vectorq7x1> q_solns_fit_;
    Vectorq7x1 q_vec_soln_;
 
  
    //Eigen::MatrixXd Jacobian_;
};

#endif	/* DAVINCI_KIN_H */

