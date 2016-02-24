/* 
 * File:   needle_planner.cpp
 * Author: wsn
 *
 * Created Feb 24, 2016
 */
#include <needle_planner/needle_planner.h>
//library to compute gripper poses to perform needle driving
// main fnc is: void compute_needle_drive_gripper_affines(...)
// need to specify tissue properties (w/rt camera frame)
// Also specify grasp transform (or accept the default) and choose which arm (or accept default=PSM1)
// provide tissue entrance pt, exit pt and surface normal (w/rt camera frame)
// Get back a vector of affines of corresponding gripper affines, wrt camera frame
// (really, inputs/outputs w/rt whatever frame is of interest, but presumably camera frame)

// at present, this code assumes rotation of the needle during driving is about an axis parallel to the tissue surface




NeedlePlanner::NeedlePlanner() {
    //decide which PSM to use.  PSM1 is robot's right arm
    psm_choice_ = PSM1; //default
    phi_insertion_ = 0.0; //start drive from here
    needle_radius_ = DEFAULT_NEEDLE_RADIUS;
    needle_axis_ht_ = NEEDLE_AXIS_HT;
    Eigen::Matrix3d R_needle_wrt_tissue;
    grasp_depth_ = default_needle_grasp_depth; 

    //assume an initial pose for the needle relative to the tissue:
    //needle origin is ht needle_axis_ht_ above the tissue, above mid-pt btwn entrance and exit
    // initially, both tip and tail of needle are above the tissue at ht needle_axis_ht_ (w/ tip unnecessarily far from tissue)
    //  (revist this later)
    //at this initial pose, needle x axis is antiparallel to tissue x axis, needle z axis is parallel to tissue y axis
    Eigen::Vector3d z_axis,y_axis,x_axis;
    z_axis<<0,1,0; //needle z is parallel to tissue y
    x_axis<<-1,0,0; // needle x is antiparallel to tissue x
    y_axis<<0,0,-1; // needle y axis is antiparallel to tissue z axis
    R_needle_wrt_tissue_.col(0) = x_axis;
    R_needle_wrt_tissue_.col(1) = y_axis;
    R_needle_wrt_tissue_.col(2) = z_axis;    
    R0_needle_wrt_tissue_ = R_needle_wrt_tissue_;
    affine_needle_frame_wrt_tissue_.linear() = R_needle_wrt_tissue_;
    //compute entrance-to-exit distance, based on needle radius and needle-axis height
    //consider equilateral triangle, needle origin above tissue (base) at ht h
    //sides of triangle are both r
    // two right triangles w/ height h and hypoteneus r--> b_rt (right-triangle base) = sqrt(r^2 - h^2)
    //base of equilateral triangle = 2*b_rt
    double b_rt = sqrt(needle_radius_*needle_radius_ - needle_axis_ht_*needle_axis_ht_);
    dist_entrance_to_exit_ = 2*b_rt;
    ROS_INFO("distance from entrance to exit is %f",dist_entrance_to_exit_);
            
    O_needle_wrt_tissue_<<0.5*dist_entrance_to_exit_,0,needle_axis_ht_;
    affine_needle_frame_wrt_tissue_.translation() = O_needle_wrt_tissue_;
      
   
    //next two transforms are fixed during needle driving; they
    // describe how the needle is held by the gripper
    Eigen::Matrix3d R;
    R << 1, 0, 0,
     0, 1, 0,
     0, 0, 1;
    affine_grasp_frame_wrt_gripper_frame_.linear() = R; // grasp frame aligned with gripper frame
    Eigen::Vector3d O_grasp_frame;
    O_grasp_frame<<0, 0, -grasp_depth_;
    affine_grasp_frame_wrt_gripper_frame_.translation() = O_grasp_frame;
    
    // decide how to grab the needle, w/rt grasp frame; (changeable via "set" fnc)
    
    //affine_needle_frame_wrt_grasp_frame_; 
    
    
    gripper_thumb_ = THUMB_UP; //1 or 2 for thumb "up" or "dn" (--> gripper y axis points "dn" or "up")

    // tissue frame should follow from entrance pt, exit pt and tissue normal
    Eigen::Affine3d affine_tissue_frame_wrt_camera_frame_; 
    // insertion angle goes from 0 to pi while driving the needle
    phi_insertion_ = 0.0; //initial angle for needle rotation
    
};

void NeedlePlanner::compute_needle_drive_gripper_affines(Eigen::Vector3d entrance_pt,
        Eigen::Vector3d exit_pt, Eigen::Vector3d tissue_normal, 
        vector <Eigen::Affine3d> &gripper_affines_wrt_camera) {
    
    //set up tissue frame w/rt camera
    Eigen::Vector3d nvec,tvec,bvec;
    bvec = tissue_normal;
    nvec = (exit_pt - entrance_pt);
    double nvec_norm = nvec.norm();
    if (nvec_norm< 0.001) {
        ROS_WARN("specified entrance and exit points are within 1mm");
        return;
    }
    nvec = nvec/nvec_norm;
    tvec = bvec.cross(nvec);
    Eigen::Matrix3d R_tissue_wrt_camera;
    R_tissue_wrt_camera.col(0)=nvec;
    R_tissue_wrt_camera.col(1)=tvec;
    R_tissue_wrt_camera.col(2)=bvec;
    affine_tissue_frame_wrt_camera_frame_.linear() = R_tissue_wrt_camera;
    affine_tissue_frame_wrt_camera_frame_.translation() = entrance_pt;
    
    //already have the initial needle frame w/rt tissue frame:  
    //rotate needle frame w/rt tissue
    double dphi = M_PI/(NSAMPS_DRIVE_PLAN-1);
    Eigen::Matrix3d Rotz_needle;
    for (int ipose=0;ipose<NSAMPS_DRIVE_PLAN;ipose++) {
        Rotz_needle = Rotz(phi_insertion_);
        R_needle_wrt_tissue_ = Rotz_needle*R0_needle_wrt_tissue_; //update rotation of needle drive
        //need to check these transforms...
        affine_needle_frame_wrt_tissue_.linear() = R_needle_wrt_tissue_;
        affine_needle_frame_wrt_camera_ = affine_tissue_frame_wrt_camera_frame_.inverse()*affine_needle_frame_wrt_tissue_;  
        affine_gripper_frame_wrt_camera_frame_ = affine_grasp_frame_wrt_gripper_frame_*affine_needle_frame_wrt_grasp_frame_*affine_needle_frame_wrt_camera_;
        gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
        
                phi_insertion_+=dphi;
    }
    //compute gripper from from needle frame, in camera coords
    //push this frame on the vector gripper_affines_wrt_camera
    // repeat for angles phi_insertion_ from 0 to pi
}

Eigen::Matrix3d NeedlePlanner::Rotz(double phi) {
    Eigen::Matrix3d Rotz;
    Rotz(0,0) = cos(phi);
    Rotz(0,1) = -sin(phi);
    Rotz(0,2) = 0.0;
    Rotz(1,0) = sin(phi);
    Rotz(1,1) = cos(phi);
    Rotz(1,0) = 0.0;
    Rotz(2,0) = 0.0;
    Rotz(2,1) = 0.0;
    Rotz(2,2) = 1.0;
    return Rotz;
}
