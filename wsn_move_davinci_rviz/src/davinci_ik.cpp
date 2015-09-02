//start developing IK; begin by making validation code: receive transforms
// to compare to IK solution

#include<ros/ros.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
using namespace std;

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

// choose x0 coincident w/ x_base ==> y0 coincident w/ z_base
// R_{0/base} = [1  0  0
//               0  0 -1
//               0  1  0 ]

// DH-1 frame:  
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
// this corresponds to a theta3 of +pi/2...and this value is static
// by construction, alpha_2 = +pi/2
// a2=0, d2 is a variable
// 

// DH-3 frame: z axis is spin about tool shaft, coincident w/ displacement axis z3
// choose origin coincident: O_3 = O_2
// check sign of + spin: alpha3 = 0 or pi
// check home angle of spin: choose x3, y3 to simplify, e.g. thetaDH_3 = psi_3

// DH-4 frame: construct from tool-shaft spin axis, z3 and wrist-bend z4 axes
// by choice of origin for O3, have O_3 = O_4
// --> a4=0, d4=0
// by construction, x4 = z3 crossed into z4 and alpha4 +pi/2
// need to find home angle and need to find positive direction of wrist rotation to define +z4

// DH-5 frame: z5 is through gripper-jaw rotation axis
// O_5 is on the z5 axis, offset from O_4
// z4 and z5 do not intersect.  Have a non-zero a5 offset
// min dist from z4 to z5 defines x5; 
// need to find +rotation direction of jaw rotation--> if alpha5 is +/- pi/2
// d5 = 0
// need to find theta5 (wrist-bend) offset for Davinci home

// DH-6 frame: choose a final gripper frame (don't have two axes to construct)
// try z6 pointing out from z5 axis at +/- pi/2 = alpha6;
// set coincident origin, a6 = d6 = 0; O_6 = O_5 (then use separate transform to gripper tip, if desired)
// find theta6 offset to conform w/ Davinci home

tf::TransformListener *g_tfListener_ptr; //pointer to a global transform listener
void transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &e) {
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
}

 geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e) {
    Eigen::Vector3f Oe;
    Eigen::Matrix3f Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaternionf q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "davinci_ik");
    ros::NodeHandle nh; //standard ros node handle   

	tf::StampedTransform tf_wrist_wrt_base;
    tf::TransformListener tfListener;
    g_tfListener_ptr = &tfListener;
    // wait to start receiving valid tf transforms 
    Eigen::Affine3f affine_wrist_wrt_base;
    bool tferr = true;
    ROS_INFO("waiting for tf between one_psm_base_link and one_tool_wrist_sca_link...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            g_tfListener_ptr->lookupTransform("one_psm_base_link", "one_tool_wrist_sca_link", ros::Time(0), tf_wrist_wrt_base);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    tf::Vector3 tf_Origin = tf_wrist_wrt_base.getOrigin();
    //ROS_INFO("")
    tf::Matrix3x3 tf_R = tf_wrist_wrt_base.getBasis();
    
    tf::Transform tf_temp;
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame tool tip w/rt tool_wrist_sca_shaft_link");
   g_tfListener_ptr->lookupTransform("one_tool_wrist_sca_shaft_link", "one_tool_tip_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   
   ROS_INFO("frame one_tool_wrist_sca_shaft_link w/rt one_tool_wrist_link");
   g_tfListener_ptr->lookupTransform("one_tool_wrist_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;     
   
   ROS_INFO("frame one_psm_base_link to one_tool_tip_link");
   g_tfListener_ptr->lookupTransform("one_psm_base_link", "one_tool_tip_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   
   double gripper_jaw_length = 0.0102;
   Eigen::Vector3f gripper_tip_origin;
   gripper_tip_origin = affine_wrist_wrt_base.translation();
   Eigen::Vector3f z_vec_gripper_tip;
   Eigen::Matrix3f  R_gripper_tip;
   R_gripper_tip = affine_wrist_wrt_base.linear();
   z_vec_gripper_tip = R_gripper_tip.col(2);
   Eigen::Vector3f computed_jaw_axis_origin;
   computed_jaw_axis_origin = gripper_tip_origin - z_vec_gripper_tip*gripper_jaw_length;
   cout<<"computed jaw axis origin: "<<computed_jaw_axis_origin.transpose()<<endl;
   
   //find origin of one_tool_wrist_link relative to one_psm_base_link
  ROS_INFO("frame 6 w/rt frame 0");
   g_tfListener_ptr->lookupTransform("one_psm_base_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   Eigen::Matrix3f R_6_wrt_0;
   Eigen::Vector3f O_6_wrt_0;   
   R_6_wrt_0 = affine_wrist_wrt_base.linear();
   O_6_wrt_0 = affine_wrist_wrt_base.translation();
   
   cout<<"computed jaw axis origin: "<<computed_jaw_axis_origin.transpose()<<endl;
   
   //find origin of one_tool_wrist_link relative to one_psm_base_link
  ROS_INFO("frame one_tool_wrist_shaft_link to one_tool_wrist_sca_shaft_link");
   g_tfListener_ptr->lookupTransform("one_tool_wrist_shaft_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   
   double a_from_wrist_bend_to_jaw_axis =  0.0091;
   
   //   R(q6) =   [sin(q6)  cos(q6)  0
   //             0        0      1
   //           cos(q6) -sin(q6)  0]
//
   // == R_{6/5}
   Eigen::Matrix3f R_6_wrt_5, R_5_wrt_0;
   Eigen::Vector3f O_5_wrt_0;
   // GIVEN O_6_wrt_0 and R_6_wrt_0 as specification of desired gripper pose, w/rt ref frame 0 = one_psm_base_link
   // w/ origin at pivot (trocar) point
   
   // TEST FWD KIN CONSISTENCY; SET MODEL q6 as follows
   double q6 = 1.0; // put q6 value here, or subscribe to it; eventually, solve for it
   Eigen::Vector3f x_vec, y_vec, z_vec;
   double a6 = 0.0091; //distance from jaw-bend axis to wrist-bend axis
   x_vec<<sin(q6),0,cos(q6);
   y_vec<<cos(q6),0,-sin(q6);
   z_vec<<0,1,0;
   R_6_wrt_5.col(0) = x_vec;
   R_6_wrt_5.col(1) = y_vec;   
   R_6_wrt_5.col(2) = z_vec;  
   R_5_wrt_0 = R_6_wrt_5.transpose()*R_6_wrt_0;
   Eigen::Vector3f xvec_5;
   xvec_5 = R_5_wrt_0.col(0);
   O_5_wrt_0 = O_6_wrt_0 - a6*xvec_5; //get to O5 by moving dist a6 backwards along x5 from O6
   
   cout<<"computed O_5_wrt_0: "<<O_5_wrt_0.transpose()<<endl;
   cout<<"computed R_5_wrt_0: "<<endl;
   cout<<R_5_wrt_0<<endl;
   
   // compare this to tf:
   //find origin of one_tool_wrist_link relative to one_psm_base_link
   ROS_INFO("frame 5 w/rt frame 0, per tf: ");
   g_tfListener_ptr->lookupTransform("one_psm_base_link", "one_tool_wrist_sca_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   transformTFToEigen(tf_temp, affine_wrist_wrt_base);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;    
}
