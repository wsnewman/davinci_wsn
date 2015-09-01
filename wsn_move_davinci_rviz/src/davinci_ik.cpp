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

}
