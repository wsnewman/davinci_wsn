#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>
#include <davinci_traj_streamer/davinci_traj_streamer.h>
#include <davinci_traj_streamer/trajAction.h>

const double MAX_PULL_LENGTH = 1.0;

Eigen::Vector3d thread_start_loc;
Eigen::Vector3d * safety_cube;
Eigen::Affine3d world_to_camera_tf;
Eigen::Affine3d camera_to_world_tf;
Eigen::Affine3d one_to_camera_tf;
Eigen::Affine3d two_to_camera_tf;

double x_delta, y_delta, z_delta;

bool right_gripper_leading;
Eigen::Vector3d right_gripper_position;
Eigen::Vector3d left_gripper_position;
sensor_msgs::JointState lkjs;


bool find_thread_start();
bool find_safety_cube();
Eigen::Vector3d calculate_goal_point();

bool get_limb_position(bool leader, ros::NodeHandle& nh);
bool go_to(
	actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> & action_client,
	const Eigen::Vector3d & point, bool dom
);
bool go_to_idle(bool dom);

bool called_back;
void jointstat_cb(const sensor_msgs::JointState::ConstPtr& incoming);
void doneCb(
	const actionlib::SimpleClientGoalState& state,
	const davinci_traj_streamer::trajResultConstPtr& result
);

trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 

//********************************************NOTE********************************************
/*
	The thread puller assumes that the task is being called with the right gripper
	attached to a thread, and the left gripper in the idle position (or at least 
	out of the way).
*/
//********************************************************************************************

int main(int argc, char** argv){

	//Set up our node, etc.
	ros::init(argc, argv, "hc_string_puller");
	ros::NodeHandle nh;
	DavinciJointPublisher djp(nh);
	
	//Lock into the action server
	actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> action_client("trajActionServer", true);
	ROS_INFO("Waiting for interpolator server:");
	bool server_exists = action_client.waitForServer(ros::Duration(5.0));
	// something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	while (!server_exists && ros::ok()) {
		ROS_WARN("Could not connect to server; retrying...");
		server_exists = action_client.waitForServer(ros::Duration(5.0));
	}
	ROS_INFO("Connected to interpolator server.");

	//These MAY OR MAY NOT always be here- I think that for things like the safety
	//	cube it makes some sense to define them in worldspace, but I'm not sure
	//	it's strictly necessary. I'd like to eliminate these calls if possible,
	//	so try to minimize the use of the world space in general.
	//Get a generally useful thing: the transform between the WORLD and the CAMERA
	tf::StampedTransform tf_raw;
	tf::TransformListener tfl;
	bool tferr=true;
	while(tferr && ros::ok()) {
		tferr=false;
		try {
			tfl.lookupTransform("world","camera",  ros::Time(0), tf_raw);
		}
		catch(tf::TransformException &exception) {
			ROS_WARN("%s", exception.what());
                	ROS_WARN("retrying");
                	tferr=true;
                	ros::Duration(0.5).sleep(); // sleep for half a second
                	ros::spinOnce();                
            	}
    	}
    	world_to_camera_tf = Davinci_fwd_solver::transformTFToAffine3d(tf_raw);
	ROS_INFO("Got transform between world and cameras.");
	//And the reverse.
	tferr=true;
	while(tferr && ros::ok()) {
		tferr=false;
		try {
			tfl.lookupTransform("camera","world",  ros::Time(0), tf_raw);
		}
		catch(tf::TransformException &exception) {
			ROS_WARN("%s", exception.what());
                	ROS_WARN("retrying");
                	tferr=true;
                	ros::Duration(0.5).sleep(); // sleep for half a second
                	ros::spinOnce();                
            	}
    	}
    	camera_to_world_tf = Davinci_fwd_solver::transformTFToAffine3d(tf_raw);
	ROS_INFO("Got transform between cameras and world.");
	
	//I'm pretty sure these ones need to stay:
	tferr=true;
	while(tferr && ros::ok()) {
		tferr=false;
		try {
			tfl.lookupTransform("camera","one_psm_base_link",  ros::Time(0), tf_raw);
		}
		catch(tf::TransformException &exception) {
			ROS_WARN("%s", exception.what());
                	ROS_WARN("retrying");
                	tferr=true;
                	ros::Duration(0.5).sleep(); // sleep for half a second
                	ros::spinOnce();                
            	}
    	}
    	one_to_camera_tf = Davinci_fwd_solver::transformTFToAffine3d(tf_raw);
	ROS_INFO("Got transform between cameras and arm one.");
	tferr=true;
	while(tferr && ros::ok()) {
		tferr=false;
		try {
			tfl.lookupTransform("camera","two_psm_base_link",  ros::Time(0), tf_raw);
		}
		catch(tf::TransformException &exception) {
			ROS_WARN("%s", exception.what());
                	ROS_WARN("retrying");
                	tferr=true;
                	ros::Duration(0.5).sleep(); // sleep for half a second
                	ros::spinOnce();                
            	}
    	}
    	two_to_camera_tf = Davinci_fwd_solver::transformTFToAffine3d(tf_raw);
	ROS_INFO("Got transform between cameras and arm two.");
	
	//Calculate our operational cube in camera space.
	if(!find_safety_cube()){
		ROS_ERROR("Safety cube undefined! WTF?!");
		return 1;
	}
	
	//Go ahead and get the location of or thread (in camera space).
	if(!find_thread_start()){
		ROS_ERROR("String puller could not locate the thread.");
		ROS_ERROR("Make sure there is a thread there or something...\
		 actually, I'm really not sure HOW this can even happen in hardcoded mode.");
		return 1;
	}
	
	//Get the position of the starting (right) gripper
	right_gripper_leading = true;
	get_limb_position(true, nh);
	

	//Calculate a parametric equation for the line between those two points.
	x_delta = right_gripper_position.x() - thread_start_loc.x();
	y_delta = right_gripper_position.y() - thread_start_loc.y();
	z_delta = right_gripper_position.z() - thread_start_loc.z();
	
	ROS_INFO("Finished finding things, beginning loop.");

	//Start pulling
	double pulled_length = 0.0;
	while(pulled_length < MAX_PULL_LENGTH && ros::ok()){
		//Find where we are supposed to pull TO:
		Eigen::Vector3d goalPoint = calculate_goal_point();
	
		Eigen::Vector3d gp_tf = camera_to_world_tf * goalPoint;
		ROS_INFO("We should go to (%f, %f, %f)", gp_tf.x(), gp_tf.y(), gp_tf.z());
		
		go_to(action_client, goalPoint, true);
	}
	
	return 0;
}

bool find_thread_start(){
	//HARDCODED======================================================================================================================
	//Hardcoded location in world space.
	thread_start_loc = Eigen::Vector3d(0.0, -0.261329, 0.142492);
	//Convert it to camera space...
	thread_start_loc = world_to_camera_tf * thread_start_loc;
	//===============================================================================================================================
	
	return true;
}

bool find_safety_cube(){
	//HARDCODED======================================================================================================================
	//Code the corners of the cube in which we operate. (World space)
	safety_cube = new Eigen::Vector3d[2];
	
	safety_cube[0] = Eigen::Vector3d(-0.25, -0.3, 0.1);
	safety_cube[1] = Eigen::Vector3d(0.25, 0.1, 0.5);
	
	safety_cube[0] = world_to_camera_tf * safety_cube[0];
	safety_cube[1] = world_to_camera_tf * safety_cube[1];
	//===============================================================================================================================
	return true;
}

bool get_limb_position(bool leader, ros::NodeHandle& nh){
	//Grab an up-to-date joint state
	ros::Subscriber joint_subscribe = nh.subscribe("/davinci/joint_states", 2, &jointstat_cb);
	called_back = false;
	while(!called_back && ros::ok()){
		ros::spinOnce();
	}
	joint_subscribe.shutdown();

	//Get the joint positions of the target limb
	//Later we might want to make this get names dynamically, since I guess left and right could be switched or something.
	std::string first_part;
	if((leader && right_gripper_leading) || (!leader && !right_gripper_leading)){
		first_part = "two";
	}
	else{
		first_part  = "one";
	}
	
	Vectorq7x1 joints;
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_yaw_joint", lkjs, joints[0]);
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_pitch_joint", lkjs, joints[1]);
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_insertion_joint", lkjs, joints[2]);
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_roll_joint", lkjs, joints[3]);
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_wrist_pitch_joint", lkjs, joints[4]);
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_wrist_yaw_joint", lkjs, joints[5]);
	Davinci_fwd_solver::get_jnt_val_by_name(first_part + "_outer_wrist_open_angle_joint", lkjs, joints[6]);
	//There has GOT to be an easier way to do this...
	
	//ROS_INFO("Joint result is %f, %f, %f, %f, %f, %f, %f", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);
	
	//Calculate the gripper's position therefrom.
	Davinci_fwd_solver f = Davinci_fwd_solver();
	Eigen::Affine3d gp_transform = f.fwd_kin_solve(joints);
	if((leader && right_gripper_leading) || (!leader && !right_gripper_leading)){
		right_gripper_position = /*camera_to_world_tf **/ two_to_camera_tf * gp_transform * Eigen::Vector3d(0.0, 0.0, 0.0);
		//ROS_INFO("Gripper location is (%f, %f, %f)", g_wrt_r.x(), g_wrt_r.y(), g_wrt_r.z());
	}
	else{
		left_gripper_position = /*camera_to_world_tf **/ one_to_camera_tf * gp_transform * Eigen::Vector3d(0.0, 0.0, 0.0);
		//ROS_INFO("Gripper location is (%f, %f, %f)", g_wrt_r.x(), g_wrt_r.y(), g_wrt_r.z());
	}
	return true;
}

Eigen::Vector3d calculate_goal_point(){
	double x_candidate_1 = (safety_cube[0].x() - thread_start_loc.x()) / x_delta;
	double x_candidate_2 = (safety_cube[1].x() - thread_start_loc.x()) / x_delta;
	double x_candidate = max(x_candidate_1, x_candidate_2);
	
	double y_candidate_1 = (safety_cube[0].y() - thread_start_loc.y()) / y_delta;
	double y_candidate_2 = (safety_cube[1].y() - thread_start_loc.y()) / y_delta;
	double y_candidate = max(y_candidate_1, y_candidate_2);
	
	double z_candidate_1 = (safety_cube[0].z() - thread_start_loc.z()) / z_delta;
	double z_candidate_2 = (safety_cube[1].z() - thread_start_loc.z()) / z_delta;
	double z_candidate = max(z_candidate_1, z_candidate_2);
	
	double t_value = min(x_candidate, min(y_candidate, z_candidate));
	
	Eigen::Vector3d gp;
	gp[0] = thread_start_loc.x() + x_delta * t_value;
	gp[1] = thread_start_loc.y() + y_delta * t_value;
	gp[2] = thread_start_loc.z() + z_delta * t_value;
	
	return gp;
}

bool go_to(
	actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> & action_client,
	const Eigen::Vector3d & gp, bool dom
){
	//Set up the message.
	des_trajectory.points.clear();
	des_trajectory.joint_names.clear();
	des_trajectory.header.stamp = ros::Time::now();
	
	trajectory_msgs::JointTrajectoryPoint trajectory_point;
	trajectory_point.positions.resize(14);

	//Figure out which arms we want to do what with.
	Eigen::Vector3d tfgp;
	int joint_offset;
	
	if((dom && right_gripper_leading) || (!dom && !right_gripper_leading)){
		tfgp = two_to_camera_tf.inverse() * gp;
		joint_offset = 7;
	}
	else{
		tfgp = one_to_camera_tf.inverse() * gp;
		joint_offset = 0;
	}
	
	//The claw we are not interested in should get out of the way:
	for(int i  = 0; i < 7; i++){
		trajectory_point.positions[i+joint_offset] = 0.0;
	}
	for(int i  = 6; i >= 0; i--){
		trajectory_point.positions[i-joint_offset] = 0.0;
	}
	trajectory_point.time_from_start = ros::Duration(2.0);
	
	des_trajectory.points.push_back(trajectory_point);
	
	davinci_traj_streamer::trajGoal goal;
	goal.trajectory = des_trajectory;
	
	
	/*//Calculate the joint positions for the limb we care about using inverse kinematics.
	Davinci_IK_solver diks = Davinci_IK_solver();
	Eigen::Vector3d wjoints = diks.q123_from_wrist(tfgp);
	//ROS_INFO("Wjoints contains %f, %f, %f", wjoints[0], wjoints[1], wjoints[2]);
	Eigen::Affine3d affine_form;
	affine_form = Eigen::Translation3d(tfgp);
	int x = diks.ik_solve(affine_form);
	ROS_INFO("Number of available solutions is %i", x);*/
	
	//Sendificate
	action_client.sendGoal(goal, &doneCb);
	bool finished_before_timeout;
	finished_before_timeout = action_client.waitForResult(ros::Duration(4.0));
	if (!finished_before_timeout) {
		ROS_WARN("giving up waiting on result");
		return 0;
	}
	else {
		ROS_INFO("finished before timeout");
	}
	
	return true;
}

void jointstat_cb(const sensor_msgs::JointState::ConstPtr& incoming){
	lkjs = *incoming;
	called_back = true;
}

void doneCb(
	const actionlib::SimpleClientGoalState& state,
	const davinci_traj_streamer::trajResultConstPtr& result
){
	ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
	ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}
