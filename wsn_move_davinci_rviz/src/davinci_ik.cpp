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
