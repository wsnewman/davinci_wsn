/* 
 * File:   needle_planner_test_simple_path.cpp
 * Author: wsn
 *
 * Created March 9, 2016
 */
#include <needle_planner/needle_planner.h>
//example use of needle-planner library
int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "needle_planner_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
        
    double x,y,z,r;
    cout<<"creating test circular motion; enter coords of center of circle, w/rt camera."<<endl;
    cout<<"enter x (e.g. -0.1):";
    cin>>x;
    cout<<"enter y (e.g. 0.0):";
    cin>>y;
    cout<<"enter z (e.g. 0.1):";
    cin>>z;
    cout<<"enter radius of circular path (e.g. 0.01):";
    cin>>r;
    //Eigen::Vector3d entrance_pt,exit_pt,tissue_normal;
    //tissue_normal<<0,0,-1; //antiparallel to optical axis
    //entrance_pt<<0.0,0.1,0.1; //100mm under camera; slightly forward, to avoid jnt lims should be OK
    //exit_pt<<0.01,0.1,0.1; // exit pt is shifted along camera-frame +x axis relative to entrance pt
    vector <Eigen::Affine3d> gripper_affines_wrt_camera;  //put answers here  

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;  
    
    //compute the path:
    //    void simple_test_gripper_motion(double x, double y, double z, double r,vector <Eigen::Affine3d> &gripper_affines_wrt_camera);
    needlePlanner.simple_test_gripper_motion(x,y,z,r,gripper_affines_wrt_camera);
    //write these to file--will be called "gripper_poses_in_camera_coords.csp"
    needlePlanner.write_needle_drive_affines_to_file(gripper_affines_wrt_camera);
    ROS_INFO("try executing this path with: ");
    ROS_INFO("rosrun playfile_reader playfile_cameraspace gripper_poses_in_camera_coords.csp");
    return 0;
} 
