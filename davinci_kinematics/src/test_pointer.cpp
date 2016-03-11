#include<ros/ros.h> 
#include<geometry_msgs/Point.h> 
#include <davinci_kinematics/davinci_kinematics.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<davinci_traj_streamer/trajAction.h>

bool g_got_new_pose = false;
int g_count = 0;

Eigen::Vector3d g_des_point;

Davinci_fwd_solver g_davinci_fwd_solver; //instantiate a forward-kinematics solver    
Davinci_IK_solver g_ik_solver;
Eigen::Affine3d g_des_gripper_affine1, g_des_gripper_affine2;
Eigen::Affine3d g_default_affine_lcamera_to_psm_one;
Eigen::Affine3d g_des_gripper1_wrt_base; 
Vectorq7x1 g_q_vec1_start, g_q_vec1_goal, g_q_vec2_start, g_q_vec2_goal;

trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory 
trajectory_msgs::JointTrajectoryPoint g_trajectory_point1, g_trajectory_point2;
davinci_traj_streamer::trajGoal g_goal;

// here is a "goal" object compatible with the server, as defined in example_action_server/action
davinci_traj_streamer::trajGoal goal;

void inPointCallback(const geometry_msgs::Point& pt_msg) {
    Eigen::Affine3d des_gripper1_wrt_base;

    g_des_point(0) = pt_msg.x;
    g_des_point(1) = pt_msg.y;
    g_des_point(2) = pt_msg.z;
    cout << "received des point = " << g_des_point.transpose() << endl;
    g_des_gripper_affine1.translation() = g_des_point;
    //convert this to base coords:
    g_des_gripper1_wrt_base = g_default_affine_lcamera_to_psm_one.inverse() * g_des_gripper_affine1;
    //try computing IK:
    if (g_ik_solver.ik_solve(g_des_gripper1_wrt_base)) {
        ROS_INFO("found IK soln");
        g_got_new_pose = true;
        g_q_vec1_start = g_q_vec1_goal;
        g_q_vec1_goal = g_ik_solver.get_soln();
        cout << g_q_vec1_goal.transpose() << endl;
        g_trajectory_point1 = g_trajectory_point2; //former goal is new start

        for (int i = 0; i < 6; i++) {
            g_trajectory_point2.positions[i] = g_q_vec1_goal[i]; //leave psm2 angles alone; just update psm1 goal angles
        }
        //gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
        //    des_trajectory.joint_names.push_back("right_s0"); //yada-yada should fill in names
        g_des_trajectory.header.stamp = ros::Time::now();

        g_des_trajectory.points[0] = g_des_trajectory.points[1]; //former goal is new start
        g_des_trajectory.points[1] = g_trajectory_point2;



        // copy traj to goal:
        g_goal.trajectory = g_des_trajectory;
    } else {
        ROS_WARN("NO IK SOLN FOUND");
    }

}

void doneCb(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

int do_inits() {
    Eigen::Matrix3d R;
    Eigen::Vector3d nvec, tvec, bvec, tip_pos;
    bvec << 0, 0, 1; //default for testing: gripper points "down"
    nvec << 1, 0, 0;
    tvec = bvec.cross(nvec);
    R.col(0) = nvec;
    R.col(1) = tvec;
    R.col(2) = bvec;
    g_des_gripper_affine1.linear() = R;
    tip_pos << -0.15, -0.03, 0.07;
    g_des_gripper_affine1.translation() = tip_pos; //will change this, but start w/ something legal

    //hard-coded camera-to-base transform, useful for simple testing/debugging
    g_default_affine_lcamera_to_psm_one.translation() << -0.155, -0.03265, 0.0;
    nvec << -1, 0, 0;
    tvec << 0, 1, 0;
    bvec << 0, 0, -1;
    //Eigen::Matrix3d R;
    R.col(0) = nvec;
    R.col(1) = tvec;
    R.col(2) = bvec;
    g_default_affine_lcamera_to_psm_one.linear() = R;

    g_q_vec1_start.resize(7);
    g_q_vec1_goal.resize(7);
    g_q_vec2_start.resize(7);
    g_q_vec2_goal.resize(7);

    g_des_gripper1_wrt_base = g_default_affine_lcamera_to_psm_one.inverse() * g_des_gripper_affine1;
    g_ik_solver.ik_solve(g_des_gripper1_wrt_base); // compute IK:
    g_q_vec1_goal = g_ik_solver.get_soln();
    g_q_vec1_start = g_q_vec1_goal;
    g_q_vec2_start << 0, 0, 0, 0, 0, 0, 0; //just put gripper 2 at home position
    g_q_vec2_goal << 0, 0, 0, 0, 0, 0, 0;
    //repackage q's into a trajectory;
    //populate a goal message for action server; 
    // initialize with 2 poses that are identical
    g_trajectory_point1.positions.clear();
    g_trajectory_point2.positions.clear();
    //resize these:
    for (int i=0;i<14;i++)  {
        g_trajectory_point1.positions.push_back(0.0); 
        g_trajectory_point2.positions.push_back(0.0); 
    }
        
    for (int i = 0; i < 7; i++) { 
        g_trajectory_point1.positions[i] = g_q_vec1_start[i];
        g_trajectory_point1.positions[i + 7] = g_q_vec2_start[i];
        //should fix up jaw-opening values...do this later
    }

    g_trajectory_point1.time_from_start = ros::Duration(0.0);
    g_trajectory_point2.time_from_start = ros::Duration(2.0);


    g_des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    g_des_trajectory.joint_names.clear();
    //    des_trajectory.joint_names.push_back("right_s0"); //yada-yada should fill in names
    g_des_trajectory.header.stamp = ros::Time::now();

    g_des_trajectory.points.push_back(g_trajectory_point1); // first point of the trajectory 
    g_des_trajectory.points.push_back(g_trajectory_point2);

    // copy traj to goal:
    g_goal.trajectory = g_des_trajectory;
    g_got_new_pose = true; //send robot to start pose
    ROS_INFO("done w/ inits");

    return 0;
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pointer");
    ros::NodeHandle n;


    ros::Subscriber thePoint = n.subscribe("/thePoint", 1, inPointCallback);
    //could separately subscribe to the normal topic, "/the_plane_normal"
    if (do_inits()!=0) return 1; //poor-man's constructor       
    
     actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> g_action_client("trajActionServer", true);
   // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    int max_tries = 0;
    while (!server_exists) {
        server_exists = g_action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("retrying...");
        max_tries++;
        if (max_tries > 100)
            break;
    }

    if (!server_exists) {
        ROS_WARN("could not connect to server; quitting");
        return 1; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server"); // if here, then we connected to the server;    

 

    while (ros::ok()) {

        ros::spinOnce();
        if (g_got_new_pose) {
            ROS_INFO("sending new goal");
            g_got_new_pose = false;
            // stuff a goal message:
            g_count++;
            g_goal.traj_id = g_count; // this merely sequentially numbers the goals sent
            ROS_INFO("sending traj_id %d", g_count);
            //g_action_client.sendGoal(g_goal, &doneCb);
            g_action_client.sendGoal(g_goal);

            bool finished_before_timeout = g_action_client.waitForResult(ros::Duration(5.0));
            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result for goal number %d", g_count);
                return 0;
            } else {
                ROS_INFO("finished before timeout");
            }
            ros::Duration(0.5).sleep();
        }
    }
    return 0; // should never get here, unless roscore dies 
}
