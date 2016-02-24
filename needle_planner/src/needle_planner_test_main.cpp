/* 
 * File:   needle_planner_test_main.cpp
 * Author: wsn
 *
 * Created Feb 24, 2016
 */
#include <needle_planner/needle_planner.h>
//example use of needle-planner library
int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "needle_planner_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Eigen::Vector3d entrance_pt,exit_pt,tissue_normal;
    tissue_normal<<0,0,-1; //antiparallel to optical axis
    entrance_pt<<0,0,0.1; //100mm directly under camera; should be OK
    exit_pt<<0,0.01,0.1; // exit pt is along camera-frame +y axis relative to entrance pt
    vector <Eigen::Affine3d> gripper_affines_wrt_camera;  //put answers here  

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;  
    needlePlanner.compute_needle_drive_gripper_affines(entrance_pt,exit_pt, tissue_normal, gripper_affines_wrt_camera);
    int nposes = gripper_affines_wrt_camera.size();
    ROS_INFO("computed %d gripper poses w/rt camera",nposes);
    Eigen::Affine3d affine_pose;
    for (int i=0;i<nposes;i++) {
        ROS_INFO("pose %d",i);
        cout<<gripper_affines_wrt_camera[i].linear()<<endl;
        cout<<"origin: "<<gripper_affines_wrt_camera[i].translation().transpose()<<endl;
    }
    ROS_WARN("I am broken; fix me!");
    return 0;
} 
