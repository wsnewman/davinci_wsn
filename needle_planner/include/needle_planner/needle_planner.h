/* 
 * File:   needle_planner.h
 * Author: wsn
 *
 * Created Feb 24, 2016
 */
#ifndef NEEDLE_PLANNER_H
#define	NEEDLE_PLANNER_H
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

//compute desired gripper frame from desired needle pose w/rt tissue
// use with: int  ik_solve(Eigen::Affine3d const& desired_hand_pose);

using namespace std;

const double default_needle_grasp_depth = 0.005; //default: grab needle at jaw mid-point
const int PSM1=1; //choose which arm to use
const int PSM2=2;
const int THUMB_UP=1;
const int THUMB_DN=2;
const int DEFAULT_NEEDLE_RADIUS = 2.54/2.0; // for 1" diam needle
const int NEEDLE_AXIS_HT= DEFAULT_NEEDLE_RADIUS/2.0; // height of needle z-axis above tissue
const int NSAMPS_DRIVE_PLAN = 21; // decide how many samples of grasp poses to compute for needle drive over 180 deg

class NeedlePlanner {
public:
    NeedlePlanner(); //constructor; 
    //geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e);
    //geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);

   void set_needle_radius(double r) {needle_radius_ = r; }
   void set_needle_axis_ht (double h) {needle_axis_ht_ = h; }

    // result depends on how the gripper is grasping the needle.  This has a default
    // grasp transform, changeable with "set" functions
    void set_affine_grasp_frame_wrt_gripper_frame(Eigen::Affine3d affine)
         {affine_grasp_frame_wrt_gripper_frame_ = affine; }
    void affine_needle_frame_wrt_grasp_frame(Eigen::Affine3d affine)
         {affine_needle_frame_wrt_grasp_frame_ = affine; }
    void set_PSM_choice(int psm_num) {          
          if (PSM1 == psm_num) psm_choice_ = psm_num;
          else if (PSM2  == psm_num) psm_choice_ = psm_num;
          else ROS_WARN("PSM choice not legal; not being changed"); }

    void set_grasp_depth(double depth) { grasp_depth_ = depth; } // this far from gripper tip
    //two kinematic choices, here referred to as "thumb up" or "thumb down"
    //hmm...maybe redundant with specifying affine_needle_frame_wrt_grasp_frame
    void set_thumb_up_or_dn(int thumb_up_dn) {
          if (THUMB_UP == thumb_up_dn) gripper_thumb_= thumb_up_dn;
          else if (THUMB_DN == thumb_up_dn) gripper_thumb_= thumb_up_dn;
          else ROS_WARN("thumb status not legal; not being changed"); }

    //main fnc: given tissue entrance pt, exit pt and surface normal (w/rt camera frame)
    // compute a sequence of gripper poses (w/rt camera frame) for needle driving
    void compute_needle_drive_gripper_affines(Eigen::Vector3d entrance_pt,
        Eigen::Vector3d exit_pt, Eigen::Vector3d tissue_normal, 
        vector <Eigen::Affine3d> &gripper_affines_wrt_camera);
    
    Eigen::Matrix3d Rotz(double phi);

private:
    double needle_radius_; 
    double needle_axis_ht_; // height of needle z-axis above tissue surface
    double dist_entrance_to_exit_; // follows from needle height and needle radius
    //decide which PSM to use.  PSM1 is robot's right arm
    int psm_choice_; //1 or 2
    //next two transforms are fixed during needle driving; they
    // describe how the needle is held by the gripper
    Eigen::Affine3d affine_grasp_frame_wrt_gripper_frame_; 	
    Eigen::Affine3d affine_needle_frame_wrt_grasp_frame_; 
    double grasp_depth_; 
    int gripper_thumb_; //1 or 2 for thumb "up" or "dn" (--> gripper y axis points "dn" or "up")

    Eigen::Matrix3d R_needle_wrt_tissue_;
    Eigen::Matrix3d R0_needle_wrt_tissue_;    
    Eigen::Vector3d O_needle_wrt_tissue_;
    Eigen::Affine3d affine_needle_frame_wrt_tissue_;  //this varies during needle driving
    Eigen::Affine3d affine_needle_frame_wrt_camera_;  //this varies during needle driving
    
    Eigen::Vector3d nvec_tissue_frame_wrt_camera_,tvec_tissue_frame_wrt_camera_,bvec_tissue_frame_wrt_camera_;

    Eigen::Vector3d repaired_exit_pt_; //this value must be consistent w/ the entrance pt and needle ht

    // tissue frame should follow from entrance pt, exit pt and tissue normal
    Eigen::Affine3d affine_tissue_frame_wrt_camera_frame_; 
    // insertion angle goes from 0 to pi while driving the needle
    double phi_insertion_;
    //this is the desired result: where should the gripper be to drive the needle
    Eigen::Affine3d affine_gripper_frame_wrt_camera_frame_; 

    


};

#endif
