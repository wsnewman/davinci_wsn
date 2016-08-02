#include <ros/ros.h>

#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>

#include <other_utilities/math_utility.h>

const double MAX_PULL_LENGTH = 1.0;

Eigen::Vector3d thread_start_loc;
Eigen::Vector3d * safety_cube;
tf::StampedTransform world_to_camera_tf;

double x_delta, y_delta, z_delta;

bool right_gripper_leading;
Eigen::Vector3d right_gripper_position;
Eigen::Vector3d left_gripper_position;
sensor_msgs::JointState lkjs;


bool find_thread_start();
bool find_safety_cube();
void calculate_goal_point(Eigen::Vector3d & gp);

bool get_limb_position(bool leader, ros::NodeHandle& nh);
bool move_to_point(const Eigen::Vector3d & point, bool dom);

bool called_back;
void jointstat_cb(const sensor_msgs::JointState::ConstPtr& incoming);

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
	
	//Get a generally useful thing: the transform between the WORLD and the CAMERA
	tf::TransformListener tfl;
	bool tferr=true;
	while(tferr && ros::ok()) {
		tferr=false;
		try {
			tfl.lookupTransform("world","left_camera_optical_frame",  ros::Time(0), world_to_camera_tf);
		}
		catch(tf::TransformException &exception) {
			ROS_WARN("%s", exception.what());
                	ROS_WARN("retrying");
                	tferr=true;
                	ros::Duration(0.5).sleep(); // sleep for half a second
                	ros::spinOnce();                
            	}
    	}
	ROS_INFO("Got transform between world and cameras.");
	
	//Calculate our operational cube in camera space.
	/*if(!find_safety_cube()){
		ROS_ERROR("Safety cube undefined! WTF?!");
		return 1;
	}
	
	//Go ahead and get the location of or thread (in camera space).
	if(!find_thread_start()){
		ROS_ERROR("String puller could not locate the thread.");
		ROS_ERROR("Make sure there is a thread there or something... actually, I'm really not sure HOW this can even happen.");
		return 1;
	}
	
	//Get the position of the starting (right) gripper
	right_gripper_leading = true;
	get_limb_position(true, nh);
	
	//Calculate a parametric equation for the line between those two points.
	x_delta = right_gripper_position.x() - thread_start_loc.x();
	y_delta = right_gripper_position.y() - thread_start_loc.y();
	z_delta = right_gripper_position.z() - thread_start_loc.z();
	
	//Find where we are supposed to pull TO:
	Eigen::Vector3d goalPoint;
	calculate_goal_point(goalPoint);
	
	ROS_INFO("Finished finding things, beginning loop.");
	
	//Start pulling
	double pulled_length = 0.0;
	while(pulled_length < MAX_PULL_LENGTH){
		
	}*/
	
	return 0;
}

bool find_thread_start(){
	//HARDCODED======================================================================================================================
	//Hardcoded location in world space.
	thread_start_loc = Eigen::Vector3d(0.0, -0.261329, 0.546016);
	//Convert it to camera space...
	Eigen::Affine3d wctf_a = math_util::transformTFToEigen(world_to_camera_tf);
	thread_start_loc = wctf_a * thread_start_loc;
	//===============================================================================================================================
	
	return true;
}

bool find_safety_cube(){
	//HARDCODED======================================================================================================================
	//Code the corners of the cube in which we operate.
	safety_cube = new Eigen::Vector3d[2];
	
	safety_cube[0] = Eigen::Vector3d(-0.25, -0.3, 0.4);
	safety_cube[1] = Eigen::Vector3d(0.25, 0.1, 0.8);
	
	Eigen::Affine3d wctf_a = math_util::transformTFToEigen(world_to_camera_tf);
	safety_cube[0] = wctf_a * safety_cube[0];
	safety_cube[1] = wctf_a * safety_cube[1];
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
	
	Davinci_fwd_solver f = Davinci_fwd_solver();
	Vectorq7x1 joints;
	f.get_jnt_val_by_name(first_part + "_outer_yaw_joint", lkjs, joints[0]);
	f.get_jnt_val_by_name(first_part + "_outer_pitch_joint", lkjs, joints[1]);
	f.get_jnt_val_by_name(first_part + "_outer_insertion_joint", lkjs, joints[2]);
	f.get_jnt_val_by_name(first_part + "_outer_roll_joint", lkjs, joints[3]);
	f.get_jnt_val_by_name(first_part + "_outer_wrist_pitch_joint", lkjs, joints[4]);
	f.get_jnt_val_by_name(first_part + "_outer_wrist_yaw_joint", lkjs, joints[5]);
	f.get_jnt_val_by_name(first_part + "_outer_wrist_open_angle_joint", lkjs, joints[6]);
	//There has GOT to be an easier way to do this...
	
	//Calculate the gripper's position therefrom.
	Eigen::Affine3d gp_transform = f.fwd_kin_solve(joints);
	Eigen::Vector3d g_wrt_r = gp_transform * Eigen::Vector3d(0.0, 0.0, 0.0);
	Eigen::Affine3d wctf_a = math_util::transformTFToEigen(world_to_camera_tf);
	if((leader && right_gripper_leading) || (!leader && !right_gripper_leading)){
		right_gripper_position = wctf_a * g_wrt_r;
	}
	else{
		left_gripper_position = wctf_a * g_wrt_r;
	}
	return true;
}

void calculate_goal_point(Eigen::Vector3d & gp){
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
	
	gp[0] = thread_start_loc.x() + x_delta * t_value;
	gp[1] = thread_start_loc.y() + y_delta * t_value;
	gp[2] = thread_start_loc.z() + z_delta * t_value;
}

bool move_to_point(const Eigen::Vector3d & point, bool dom){
	Davinci_IK_solver f = Davinci_IK_solver();
	
	return true;
}

void jointstat_cb(const sensor_msgs::JointState::ConstPtr& incoming){
	lkjs = *incoming;
	called_back = true;
}
