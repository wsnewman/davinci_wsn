/* 
 * File:   needle_planner_kvec_horiz_test_main_v2.cpp
 * Author: wsn
 *
 * Created Mar 17, 2016
 * this version: test needle drive for various kvec angles
 * Extend: accept needle entry point; compute needle drives for pivots about entry point
 *  need to specify height of needle center above tissue
 * This version: subscribes to "thePoint" topic
 */
#include <needle_planner/needle_planner.h>
//example use of needle-planner library
const double r_needle = 0.012; //0.0254/2.0;
const double needle_height_above_tissue = r_needle / sqrt(2.0);
const double d_to_exit_pt = 2 * r_needle / sqrt(2.0); // only for chosen needle ht
const double z_tissue = 0.17; // HARD CODED; USE ACTUAL TISSUE Z, WRT CAMERA
Eigen::Vector3d g_O_entry_point;
bool g_got_new_entry_point = false;

void inPointCallback(const geometry_msgs::Point& pt_msg) {

    g_O_entry_point(0) = pt_msg.x;
    g_O_entry_point(1) = pt_msg.y;
    g_O_entry_point(2) = pt_msg.z;
    g_got_new_entry_point = true;

    cout << "received new entry point = " << g_O_entry_point.transpose() << endl;
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "needle_planner_test_main"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ros::Publisher exit_pt_publisher = nh.advertise<geometry_msgs::Point>("exit_points", 1);
    ros::Subscriber thePoint = nh.subscribe("/thePoint", 1, inPointCallback);

    Eigen::Vector3d O_needle;
    Eigen::Vector3d O_entrance_pt;
    Eigen::Vector3d O_exit_pt;
    double x_entrance, y_entrance;

    /*
    cout<<"using tissue z = "<<z_tissue<<endl; 

    O_needle(2) = z_tissue - needle_height_above_tissue; // closer to camera, so smaller z
    
    cout<<"enter entrance pt x (e.g. -0.02): ";
    cin>>x_entrance;
    O_entrance_pt(0) = x_entrance;
    cout<<"enter entrance pt y (e.g. 0.01): ";
    cin>>y_entrance;
    O_entrance_pt(1) = y_entrance;
    O_entrance_pt(2) = z_tissue;
    cout<<"O_entrance_pt = "<<O_entrance_pt.transpose()<<endl;
   
    //O_needle<< -0.02,0.004,0.1578; //we can hard-code O_needle(2) (z-value) for known tissue height
     */

    vector <Eigen::Affine3d> gripper_affines_wrt_camera; //put answers here 
    vector <geometry_msgs::Point> exit_points;
    geometry_msgs::Point exitPoint;
    double needle_x, needle_y;
    Eigen::Vector3d v_entrance_to_exit, v_entrance_to_exit0;
    v_entrance_to_exit0 << 0, -1, 0; // corresponds to chosen needle kvec along 1,0,0

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;

    while (ros::ok()) {
        if (g_got_new_entry_point) {
            g_got_new_entry_point = false;

            double kvec_yaw = 0.0; // rotation of needle z-axis w/rt camera x-axis

                O_entrance_pt = g_O_entry_point;
            //compute the tissue frame in camera coords, based on point-cloud selections:
            for (kvec_yaw = 0.0; kvec_yaw < 6.28; kvec_yaw += 0.1) {
                v_entrance_to_exit = needlePlanner.Rotz(kvec_yaw) * v_entrance_to_exit0; //rotate the needle axis about camera z-axis
                O_exit_pt = O_entrance_pt + d_to_exit_pt*v_entrance_to_exit;
                O_needle = 0.5 * (O_exit_pt + O_entrance_pt);
                O_needle(2) -= needle_height_above_tissue;
                cout << "O_needle = " << O_needle.transpose() << endl;
                cout << "O_exit_pt = " << O_exit_pt.transpose() << endl;
                gripper_affines_wrt_camera.clear();

                needlePlanner.simple_horiz_kvec_motion(O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera);
                int nposes = gripper_affines_wrt_camera.size();
                ROS_WARN("at kvec_yaw = %f, computed %d needle-drive gripper poses ", kvec_yaw, nposes);
                if (nposes >= 40) {
                    exitPoint.x = O_exit_pt(0);
                    exitPoint.y = O_exit_pt(1);
                    exitPoint.z = O_exit_pt(2);
                    exit_pt_publisher.publish(exitPoint);
                    exit_points.push_back(exitPoint); //not used...
                }
            }
        }
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    /*
    Eigen::Affine3d affine_pose;
    for (int i=0;i<nposes;i++) {
        ROS_INFO("pose %d",i);
        cout<<gripper_affines_wrt_camera[i].linear()<<endl;
        cout<<"origin: "<<gripper_affines_wrt_camera[i].translation().transpose()<<endl;
    }
    needlePlanner.write_needle_drive_affines_to_file(gripper_affines_wrt_camera);
     */
    return 0;
}
