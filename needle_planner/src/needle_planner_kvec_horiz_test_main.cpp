/* 
 * File:   needle_planner_kvec_horiz_test_main.cpp
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
    Eigen::Vector3d O_needle;
    O_needle<< -0.02,0.004,0.1578;
    double r_needle = 0.012; //0.0254/2.0;
    double kvec_yaw = 0.0;
    vector <Eigen::Affine3d> gripper_affines_wrt_camera;  //put answers here  
    

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;  
    //compute the tissue frame in camera coords, based on point-cloud selections:
    
    needlePlanner.simple_horiz_kvec_motion(O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera);
    int nposes = gripper_affines_wrt_camera.size();
    ROS_INFO("computed %d gripper poses w/rt camera",nposes);
    Eigen::Affine3d affine_pose;
    for (int i=0;i<nposes;i++) {
        ROS_INFO("pose %d",i);
        cout<<gripper_affines_wrt_camera[i].linear()<<endl;
        cout<<"origin: "<<gripper_affines_wrt_camera[i].translation().transpose()<<endl;
    }
    needlePlanner.write_needle_drive_affines_to_file(gripper_affines_wrt_camera);
    
    return 0;
} 
