/* 
 * File:   needle_plan_horiz_kvec.cpp
 * Author: wsn
 *
 * Created March 19, 2016, wsn
 * This version: listens for a geometry_msgs/Polygon message on topic: entrance_and_exit_pts
 * ASSUMES the tissue is horizontal; constructs corresponding kvec_yaw angle and needle center point
 * Computes a corresponding cps file for gripper1 motion for circular needle drive
 * saves this file to disk as: 
 */
#include <needle_planner/needle_planner.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

//example use of needle-planner library

Eigen::Affine3d g_affine_lcamera_to_psm_one, g_affine_lcamera_to_psm_two; //, affine_gripper_wrt_base;
Eigen::Affine3d g_psm1_start_pose, g_psm2_start_pose;
bool g_got_new_points = false;
//bool g_got_exit_marker_point = false;
tf::TransformListener *g_tfListener_ptr;
Eigen::Vector3d g_O_entry_point, g_O_exit_point;

//helper fnc to transform a pose, given a transform
geometry_msgs::Point xform_point(geometry_msgs::Point in_pnt,tf::Transform tf) {
    tf::Vector3 tf_in_pt,tf_out_pt;
    geometry_msgs::Point out_pnt;
    tf_in_pt.setX(in_pnt.x);
    tf_in_pt.setY(in_pnt.y);
    tf_in_pt.setZ(in_pnt.z);
    tf_out_pt = tf*tf_in_pt;
    out_pnt.x = tf_out_pt.getX();
    out_pnt.y = tf_out_pt.getY();
    out_pnt.z = tf_out_pt.getZ();
    return out_pnt;
}

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
    //tf::TransformListener tfListener;
    tf::StampedTransform tfResult_one, tfResult_two;
    g_tfListener_ptr = new tf::TransformListener;
    bool tferr = true;
    int ntries = 0;
    ROS_INFO("waiting for tf between base and camera...");
    while (tferr) {
        if (ntries > 5) break; //give up and accept default after this many tries
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            g_tfListener_ptr->lookupTransform("left_camera_optical_frame", "one_psm_base_link", ros::Time(0), tfResult_one);
            g_tfListener_ptr->lookupTransform("left_camera_optical_frame", "two_psm_base_link", ros::Time(0), tfResult_two);
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
            g_tfListener_ptr->lookupTransform("left_camera_optical_frame", "one_tool_tip_link", ros::Time(0), tfResult_one);
           g_tfListener_ptr->lookupTransform("left_camera_optical_frame", "two_tool_tip_link", ros::Time(0), tfResult_two);
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
    
    //just to be safe:
    g_O_entry_point(0) = 0.0;
    g_O_entry_point(1) = 0.0;
    g_O_entry_point(2) = 0.12;
    g_O_exit_point = g_O_entry_point;
    g_O_exit_point(0) += 0.02;
    

}



void inPointsCallback(const geometry_msgs::Polygon& pts_array) {
    if (pts_array.points.size() != 2) {
        ROS_ERROR("wrong number of points in pts_array! should be two");
        return;
    }
    //unpack the points and fill in entry and exit pts
    g_O_entry_point(0) = pts_array.points[0].x;
    g_O_entry_point(1) = pts_array.points[0].y;
    g_O_entry_point(2) = pts_array.points[0].z;
    g_O_exit_point(0) = pts_array.points[1].x;
    g_O_exit_point(1) = pts_array.points[1].y;
    g_O_exit_point(2) = pts_array.points[1].z;

    g_got_new_points = true;

    cout << "received new entry point = " << g_O_entry_point.transpose() << endl;
    cout << "and exit point: " << g_O_exit_point.transpose() << endl;
}


void entryPointCallback(const geometry_msgs::Point& pt_msg) {
    g_O_entry_point(0) = pt_msg.x;
    g_O_entry_point(1) = pt_msg.y;
    g_O_entry_point(2) = pt_msg.z;
    cout << "received entry point = " << g_O_entry_point.transpose() << endl;    
}

void exitMarkerCallback(const visualization_msgs::Marker& marker) {
    ROS_INFO("received exit-point marker");
    cout<<"frame_id="<<marker.header.frame_id<<endl;
    ROS_INFO("coors w/rt frame_id: %f, %f, %f",marker.pose.position.x,marker.pose.position.y,marker.pose.position.z);
    tf::StampedTransform stfMarkerWrtCamera;
    //string marker_frame(marker.header.frame_id);
    g_tfListener_ptr->lookupTransform("left_camera_optical_frame", marker.header.frame_id, ros::Time(0), stfMarkerWrtCamera);
    tf::Transform tfMarkerWrtCamera(stfMarkerWrtCamera.getBasis(),stfMarkerWrtCamera.getOrigin()); //get tf from stamped tf    
    geometry_msgs::Point marker_pt = marker.pose.position;
    geometry_msgs::Point marker_pt_wrt_camera = xform_point(marker_pt,tfMarkerWrtCamera);
    ROS_INFO("marker coords w/rt camera: %f, %f, %f",marker_pt_wrt_camera.x,marker_pt_wrt_camera.y,marker_pt_wrt_camera.z);
    g_O_exit_point(0) = marker_pt_wrt_camera.x;
    g_O_exit_point(1) = marker_pt_wrt_camera.y;
    g_O_exit_point(2) = marker_pt_wrt_camera.z;   
    //g_got_new_points = true;
}


int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "needle_planner_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ros::Subscriber pt_subscriber = nh.subscribe("/entrance_and_exit_pts", 1, inPointsCallback);
    ros::Subscriber entry_pt_subscriber = nh.subscribe("/thePoint", 1, entryPointCallback);
   
    ros::Subscriber exit_marker_subscriber = nh.subscribe("/final_point_marker",1,exitMarkerCallback);
//  ros::Publisher finalPointPub = nh.advertise<visualization_msgs::Marker>("/final_point_marker", 0);

    init_poses();

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;
    needlePlanner.set_affine_lcamera_to_psm_one(g_affine_lcamera_to_psm_one);
    needlePlanner.set_affine_lcamera_to_psm_two(g_affine_lcamera_to_psm_two);

    Eigen::Vector3d O_needle;
    double r_needle = DEFAULT_NEEDLE_RADIUS; //0.012; //0.0254/2.0;
    Eigen::Vector3d in_to_out_vec;
    double kvec_yaw;
    


    //cout<<"enter kvec_yaw: (e.g. 0-2pi): ";
    //cin>>kvec_yaw;
    /*
    double needle_x,needle_y,needle_z;
    cout<<"enter needle center x coord (e.g. 0): ";
    cin>>needle_x;
    cout<<"enter needle center y coord (e.g. 0): ";
    cin>>needle_y;  
    cout<<"enter needle center z coord (e.g. 0.112): ";
    cin>>needle_z;      
    O_needle(0) = needle_x;
    O_needle(1) = needle_y;
    O_needle(2) = needle_z;
     */
    Eigen::Affine3d affine_pose;
    vector <Eigen::Affine3d> gripper_affines_wrt_camera;  //put answers here  
    
    ROS_INFO("entering loop...");
    while (ros::ok()) {
        if (g_got_new_points) {
            g_got_new_points = false;
        //compute O_needle from entry and exit points:
        O_needle = 0.5 * (g_O_entry_point + g_O_exit_point);
        O_needle(2) -= DEFAULT_NEEDLE_AXIS_HT;


        in_to_out_vec = g_O_exit_point - g_O_entry_point;
        //vector from entry to exit is 90-deg away from needle z-axis, so add pi/2
        kvec_yaw = atan2(in_to_out_vec(1), in_to_out_vec(0))+M_PI/2.0;
        ROS_INFO("using kvec_yaw = %f",kvec_yaw);

            //compute the tissue frame in camera coords, based on point-cloud selections:

            needlePlanner.simple_horiz_kvec_motion(O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera);
            int nposes = gripper_affines_wrt_camera.size();
            ROS_INFO("computed %d gripper poses w/rt camera", nposes);

            for (int i = 0; i < nposes; i++) {
                ROS_INFO("pose %d", i);
                cout << gripper_affines_wrt_camera[i].linear() << endl;
                cout << "origin: " << gripper_affines_wrt_camera[i].translation().transpose() << endl;
            }
            needlePlanner.write_needle_drive_affines_to_file(gripper_affines_wrt_camera);
        }
        
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
