/* 
 * File:   needle_planner_kvec_horiz_test_main_v2.cpp
 * Author: wsn
 *
 * Created Mar 17, 2016
 * this version: test needle drive for various kvec angles
 * Extend: accept needle entry point; compute needle drives for pivots about entry point
 *  need to specify height of needle center above tissue
 * This version: subscribes to "thePoint" topic
 * can manually test with: rostopic pub  /thePoint geometry_msgs/Point  '{x: 0, y: 0, z: 0.12}'
 * NOTE: this depends on positioning of the arms, whether this is reachable
 * 
 */
#include <needle_planner/needle_planner.h>
#include <geometry_msgs/Polygon.h>
const int g_npts_good=32;

Eigen::Affine3d g_affine_lcamera_to_psm_one, g_affine_lcamera_to_psm_two; //, affine_gripper_wrt_base;
Eigen::Affine3d g_psm1_start_pose,g_psm2_start_pose;
Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
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
    return e;
}

void init_poses() {
   ROS_INFO("getting transforms from camera to PSMs");
    tf::TransformListener tfListener;
    tf::StampedTransform tfResult_one, tfResult_two;
    
    bool tferr = true;
    int ntries = 0;
    ROS_INFO("waiting for tf between base and camera...");
    while (tferr) {
        if (ntries > 5) break; //give up and accept default after this many tries
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("left_camera_optical_frame", "one_psm_base_link", ros::Time(0), tfResult_one);
            tfListener.lookupTransform("left_camera_optical_frame", "two_psm_base_link", ros::Time(0), tfResult_two);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
        }
    }
    //default transform: need to match this up to camera calibration!
    if (tferr) {
        g_affine_lcamera_to_psm_one.translation() << -0.155, -0.03265, 0.0;
        Eigen::Vector3d nvec, tvec, bvec;
        nvec << -1, 0, 0;
        tvec << 0, 1, 0;
        bvec << 0, 0, -1;
        Eigen::Matrix3d R;
        R.col(0) = nvec;
        R.col(1) = tvec;
        R.col(2) = bvec;
        g_affine_lcamera_to_psm_one.linear() = R;
        g_affine_lcamera_to_psm_two.linear() = R;
        g_affine_lcamera_to_psm_two.translation() << 0.145, -0.03265, 0.0;
        ROS_WARN("using default transform");
    } else {

        ROS_INFO("tf is good");

        //g_affine_lcamera_to_psm_one is the position/orientation of psm1 base frame w/rt left camera link frame
        // need to extend this to camera optical frame
        g_affine_lcamera_to_psm_one = transformTFToEigen(tfResult_one);
        g_affine_lcamera_to_psm_two = transformTFToEigen(tfResult_two);
    }
    ROS_INFO("transform from left camera to psm one:");
    cout << g_affine_lcamera_to_psm_one.linear() << endl;
    cout << g_affine_lcamera_to_psm_one.translation().transpose() << endl;
    ROS_INFO("transform from left camera to psm two:");
    cout << g_affine_lcamera_to_psm_two.linear() << endl;
    cout << g_affine_lcamera_to_psm_two.translation().transpose() << endl;  
    
    //now get initial poses:
    ROS_INFO("waiting for tf between base and grippers...");
    while (tferr) {
        if (ntries > 5) break; //give up and accept default after this many tries
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("left_camera_optical_frame", "one_tool_tip_link", ros::Time(0), tfResult_one);
            tfListener.lookupTransform("left_camera_optical_frame", "two_tool_tip_link", ros::Time(0), tfResult_two);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
        }
    }
    //default start pose, if can't get tf:
    if (tferr) {
        g_psm1_start_pose.translation() << -0.02, 0, 0.04;
        g_psm2_start_pose.translation() << 0.02, 0, 0.04;
        Eigen::Vector3d nvec, tvec, bvec;
        nvec << -1, 0, 0;
        tvec << 0, 1, 0;
        bvec << 0, 0, -1;
        Eigen::Matrix3d R;
        R.col(0) = nvec;
        R.col(1) = tvec;
        R.col(2) = bvec;
        g_psm1_start_pose.linear() = R;
        g_psm2_start_pose.linear() = R;
        ROS_WARN("using default start poses");
    } else {

        ROS_INFO("tf is good");

        //g_affine_lcamera_to_psm_one is the position/orientation of psm1 base frame w/rt left camera link frame
        // need to extend this to camera optical frame
        g_psm1_start_pose = transformTFToEigen(tfResult_one);
        g_psm2_start_pose = transformTFToEigen(tfResult_two);
    }
    ROS_INFO("psm1 gripper start pose:");
    cout << g_psm1_start_pose.linear() << endl;
    cout << g_psm1_start_pose.translation().transpose() << endl;
    ROS_INFO("psm2 gripper start pose:");
    cout << g_psm2_start_pose.linear() << endl;
    cout << g_psm2_start_pose.translation().transpose() << endl;      
    
}

//example use of needle-planner library
const double r_needle = 0.012; //0.0254/2.0;
const double needle_height_above_tissue = r_needle / sqrt(2.0);
const double d_to_exit_pt = 2 * r_needle / sqrt(2.0); // only for chosen needle ht
const double z_tissue = 0.10; // HARD CODED; USE ACTUAL TISSUE Z, WRT CAMERA
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
    ros::Publisher exit_pt_array_publisher = nh.advertise<geometry_msgs::Polygon>("exit_point_array", 1);    
    ros::Subscriber thePoint = nh.subscribe("/thePoint", 1, inPointCallback);

    Eigen::Vector3d O_needle;
    Eigen::Vector3d O_entrance_pt;
    Eigen::Vector3d O_exit_pt;
    double x_entrance, y_entrance;
    geometry_msgs::Polygon polygon_msg;

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
    geometry_msgs::Point32 p32;
    double needle_x, needle_y;
    Eigen::Vector3d v_entrance_to_exit, v_entrance_to_exit0;
    v_entrance_to_exit0 << 0, -1, 0; // corresponds to chosen needle kvec along 1,0,0

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;
    init_poses();
     // set camera-to-base transforms:
    needlePlanner.set_affine_lcamera_to_psm_one(g_affine_lcamera_to_psm_one);
    needlePlanner.set_affine_lcamera_to_psm_two(g_affine_lcamera_to_psm_two);   
    
    while (ros::ok()) {
        if (g_got_new_entry_point) {
            g_got_new_entry_point = false;
            polygon_msg.points.clear();
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
                if (nposes >= g_npts_good) {
                    exitPoint.x = O_exit_pt(0);
                    exitPoint.y = O_exit_pt(1);
                    exitPoint.z = O_exit_pt(2);
                    p32.x = O_exit_pt(0);
                    p32.y = O_exit_pt(1);
                    p32.z = O_exit_pt(2);
                    exit_pt_publisher.publish(exitPoint);
                    polygon_msg.points.push_back(p32);
                    exit_points.push_back(exitPoint); //not used...
                }
                if (polygon_msg.points.size()>0) {
                    exit_pt_array_publisher.publish(polygon_msg);
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
