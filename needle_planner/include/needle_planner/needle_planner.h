/* 
 * File:   needle_planner.h
 * Author: wsn
 *
 * Created Feb 24, 2016
 */

bool debug_needle_print=false;
#ifndef NEEDLE_PLANNER_H
#define	NEEDLE_PLANNER_H
#include <ros/ros.h>
#include <iostream>
#include <fstream>
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
#include <davinci_kinematics/davinci_kinematics.h>
//compute desired gripper frame from desired needle pose w/rt tissue
// use with: int  ik_solve(Eigen::Affine3d const& desired_hand_pose);

using namespace std;

const double DEFAULT_NEEDLE_GRASP_DEPTH = 0.005; //default: grab needle at jaw mid-point
//this computation is with respect to a gripper--does not matter which arm, since
// no IK is done here
const int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y=1;
const int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Y=-1;
const int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z=1; //grab needle w/ needle z-axis parallel to gripper z-axis
const int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z=-1; // needle z antiparallel to gripper z
const double DEFAULT_NEEDLE_RADIUS = 0.0254/2.0; // for 1" diam needle
const double DEFAULT_NEEDLE_AXIS_HT= DEFAULT_NEEDLE_RADIUS/2.0; // height of needle z-axis above tissue
const int NSAMPS_DRIVE_PLAN = 21; // decide how many samples of grasp poses to compute for needle drive over 180 deg
//phi grab at 0.0--> grab in middle of arc
const double DEFAULT_PHI_GRAB = 0.0;// M_PI/2.0; //puts tail of needle in middle of gripper--really not feasible

class NeedlePlanner {
public:
    NeedlePlanner(); //constructor; 
    //geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e);
    //geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);

   void set_needle_radius(double r) {needle_radius_ = r; }
   void set_needle_axis_ht (double h) {needle_axis_ht_ = h; }
   void set_psi_needle_axis_tilt_wrt_tissue(double tilt) {psi_needle_axis_tilt_wrt_tissue_ = tilt; }
    // result depends on how the gripper is grasping the needle.  This has a default
    // grasp transform, changeable with "set" functions
    //the next 4 fncs change params of the default needle grasp transform
    void set_affine_grasp_frame_wrt_gripper_frame(Eigen::Affine3d affine)
         {affine_grasp_frame_wrt_gripper_frame_ = affine; }
    void set_affine_needle_frame_wrt_tissue(Eigen::Affine3d affine)
    { affine_needle_frame_wrt_tissue_ = affine; }
    void set_grasp_depth(double depth) { grasp_depth_ = depth; } // this far from gripper tip
    //two kinematic choices, here referred to as "thumb up" or "thumb down"
    //hmm...maybe redundant with specifying affine_needle_frame_wrt_grasp_frame
    void set_grab_needle_plus_minus_y(int needle_plus_minus_y) {
          if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_) grab_needle_plus_minus_y_= needle_plus_minus_y;
          else if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_) grab_needle_plus_minus_y_= needle_plus_minus_y;
          else ROS_WARN("grasp status not legal; not being changed"); }
    void set_grab_needle_plus_minus_z(int needle_plus_minus_z) {
          if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z == grab_needle_plus_minus_z_) grab_needle_plus_minus_z_= needle_plus_minus_z;
          else if (GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z == grab_needle_plus_minus_z_) grab_needle_plus_minus_z_= needle_plus_minus_z;
          else ROS_WARN("grasp status needle z-axis sign not legal; not being changed"); }
    
    void compute_grasp_transform(); //if use above sets, apply logic to compute grasp transform
    void compute_grasp_transform(double phi_x,double phi_y);
    //the next fnc can override all of the above
    void set_affine_needle_frame_wrt_gripper_frame(Eigen::Affine3d affine)
         {affine_needle_frame_wrt_gripper_frame_ = affine; }

    void compute_tissue_frame_wrt_camera(Eigen::Vector3d entrance_pt,
        Eigen::Vector3d exit_pt, Eigen::Vector3d tissue_normal);
    //main fnc: given tissue entrance pt, exit pt and surface normal (w/rt camera frame)
    // compute a sequence of gripper poses (w/rt camera frame) for needle driving
    void compute_needle_drive_gripper_affines(vector <Eigen::Affine3d> &gripper_affines_wrt_camera);
    
    //test fnc--just computes a simple gripper path:
    void simple_test_gripper_motion(double x, double y, double z, double r,vector <Eigen::Affine3d> &gripper_affines_wrt_camera);

    void write_needle_drive_affines_to_file(vector <Eigen::Affine3d> &gripper_affines_wrt_camera);
    double vers(double phi) { return (1.0-cos(phi)); } 
    
    //some utility functions:
    //for rotations about z and y axes
    Eigen::Matrix3d Rotz(double phi);
    Eigen::Matrix3d Roty(double phi);
    Eigen::Matrix3d Rotx(double phi);
    Eigen::Matrix3d Rot_k_phi(Eigen::Vector3d k_vec,double phi);
    void print_affine(Eigen::Affine3d affine); //print out an affine for debug

private:
    //needle properties: these will be constant for a given operation
    double needle_radius_; 
    double needle_axis_ht_; // height of needle z-axis above tissue surface
    double dist_entrance_to_exit_; // follows from needle height and needle radius
    
    //needle-grasp properties: these will be constant during needle driving
    int grab_needle_plus_minus_y_; //needle origin to lie in + or - gripper-y half space?
    int grab_needle_plus_minus_z_; //bvec_needle = +/- bvec_gripper (for default cases)
    double grasp_depth_; // w/rt gripper bvec, where are needle-grasp contact points on jaws?
    double phi_grab_; //helps define needle grasp; phi_grab_=0--> grab at center, phi_grab_=pi/2
    //--> grab at tail; phi_grab = -pi/2 --> grab at tip
    //specify needle orientation: e.g., needle z-axis typically parallel or antiparallel to gripper bvec
    Eigen::Vector3d bvec_needle_wrt_grasp_frame_; 
    Eigen::Vector3d nvec_needle_wrt_grasp_frame_;  //nvec_needle is from needle center to needle tip 
    Eigen::Vector3d tvec_needle_wrt_grasp_frame_;    
    //next two transforms are fixed during needle driving; they
    // describe how the needle is held by the gripper
    Eigen::Affine3d affine_grasp_frame_wrt_gripper_frame_; 
    Eigen::Vector3d O_needle_frame_wrt_grasp_frame_;
    Eigen::Matrix3d R_needle_frame_wrt_grasp_frame_;
    Eigen::Matrix3d R0_N_wrt_G_; 
    Eigen::Vector3d O0_N_wrt_G_;
    Eigen::Affine3d affine_needle_frame_wrt_grasp_frame_; 
    Eigen::Affine3d affine_needle_frame_wrt_gripper_frame_; 
    
    //these properties require perception: define a frame on the tissue, wrt camera frame
    Eigen::Vector3d nvec_tissue_frame_wrt_camera_,tvec_tissue_frame_wrt_camera_,bvec_tissue_frame_wrt_camera_;
    Eigen::Vector3d desired_needle_entrance_point_; // will be specified interactively
    Eigen::Vector3d repaired_exit_pt_; //this value must be consistent 
          //w/ the entrance pt, needle ht and needle radius.  Can't trust operator to get this exact
    // tissue frame should follow from entrance pt, exit pt and tissue normal
    Eigen::Matrix3d R_tissue_frame_wrt_camera_frame_;
    Eigen::Affine3d affine_tissue_frame_wrt_camera_frame_; 
    
    //needle-driving strategy: follows from 
    //needle_radius_, needle_axis_ht_, tissue normal, specified entrance pt and repaired exit pt,
    // assumes needle-z axis is parallel to the tissue, and needle will rotate about its own z axis
    //bvec_needle must also be perpendicular to tissue-frame x-axis, which points from entrance pt to exit pt
    // implies bvec_needle MUST point antiparallel to tvec_tissue
    // this vector will remain constant during needle driving
    Eigen::Vector3d bvec_needle_wrt_tissue_frame_; //w/ above assumptions, this is (0,-1,0)
    //need to define where needle-tip starts;
    // for simplicity, assume needle tip and needle tail both start at needle_axis_ht_ above the tissue;
    //  (so needle tip is not yet touching the tissue at the entrance point--overly conservative)
    // then, needle x-axis (from needle origin to needle tip) is antiparallel to tissue x-axis:
    Eigen::Vector3d nvec_needle_wrt_tissue_frame_; //initialize to (-1,0,0); will change orientation during driving
    Eigen::Vector3d tvec_needle_wrt_tissue_frame_; // follows from bvec and nvec
    Eigen::Matrix3d R0_needle_wrt_tissue_; //initial orientation of needle frame w/rt tissue;
                                           // follows from above bvec and nvec
    //specify origin of needle frame (center of needle) w/rt tissue frame; this will be
    //  directly above the mid-point between entrance and exit pt; will remain constant during
    // needle driving (in all frames)
    Eigen::Vector3d O_needle_wrt_tissue_; //= (dist_entrance_to_exit_, 0, needle_axis_ht_)
    Eigen::Affine3d affine_init_needle_frame_wrt_tissue_; //starting pose of needle
    // variables that evolve during needle driving:    
    // during driving, insertion angle goes from 0 to pi (could shorten this)
    double phi_insertion_;
    double psi_needle_axis_tilt_wrt_tissue_;
    Eigen::Matrix3d R_needle_wrt_tissue_;
    Eigen::Affine3d affine_needle_frame_wrt_tissue_;  //this varies during needle driving
    Eigen::Affine3d affine_gripper_frame_wrt_tissue_;  // this follows from needle frame and grasp transform
    
    Eigen::Affine3d affine_needle_frame_wrt_camera_;  //this varies during needle driving    
    //this is the desired result: where should the gripper be to drive the needle
    // follows from affine_needle_frame_wrt_camera_ and grasp transforms
    Eigen::Affine3d affine_gripper_frame_wrt_camera_frame_;    
    
    Davinci_fwd_solver davinci_fwd_solver_; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver_;
    //default camera transform: should find actual tf by listening, but this
    // hard-coded default is useful for simple tests
    Eigen::Affine3d default_affine_lcamera_to_psm_one_;    
};

#endif
