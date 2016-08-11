#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <davinci_kinematics/davinci_kinematics.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <davinci_traj_streamer/trajAction.h>

const std::string joint_names[13] = {
	"joint1_position_controller"	,
	"joint2_position_controller"	,
	"joint2_1_position_controller"	,
	"joint2_2_position_controller"	,
	"joint2_3_position_controller"	,
	"joint2_4_position_controller"	,
	"joint2_5_position_controller"	,
	"joint3_position_controller"	,
	"joint4_position_controller"	,
	"joint5_position_controller"	,
	"joint6_position_controller"	,
	"joint7_position_controller"	,
	"joint7_position_controller_mimic"
};
//And of course, despite representing the same exact joints, the joint names
//that come out of the state message are DIFFERENT from the ones the
//controllers answer to...
const std::string feedback_names[13] = {
	"outer_yaw_joint"			,
	"outer_pitch_joint"			,
	"outer_pitch_joint_1" 			,
	"outer_pitch_joint_2" 			,
	"outer_pitch_joint_3" 			,
	"outer_pitch_joint_4"			,
	"outer_pitch_joint_5" 			,
	"outer_insertion_joint"			,
	"outer_roll_joint"			,
	"outer_wrist_pitch_joint"		,
	"outer_wrist_yaw_joint"			,
	"outer_wrist_open_angle_joint"		,
	"outer_wrist_open_angle_joint_mimic"
	
};
const double dt_traj = 0.01;

ros::Publisher publishers[2][13];
actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> * server_gp;
sensor_msgs::JointState states;
bool fresh_pos;

void CB_execute(
	const davinci_traj_streamer::trajGoalConstPtr& goal,
	actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>* server
);

void CB_update(const sensor_msgs::JointState::ConstPtr& incoming);

void CB_goal();

bool exec_position(const trajectory_msgs::JointTrajectoryPoint & in_point, double & elapsed_time);

std::vector<std::vector<double> > expand_joint_list(const std::vector<double> & input);

std::vector<std::vector<double> > get_robot_pos();

int main(int argc, char **argv) {
	//Set up the node.
	ros::init(argc, argv, "traj_interpolator_both_as");
	ros::NodeHandle nh;
	
	//Set up the publishers.
	for(int i = 0; i < 13; i++){
		publishers[0][i] = nh.advertise<std_msgs::Float64>("/davinci/one_" + joint_names[i] + "/command", 1, true); 
		publishers[1][i] = nh.advertise<std_msgs::Float64>("/davinci/two_" + joint_names[i] + "/command", 1, true);
	}
	
	//Set up the subscriber
	ros::Subscriber robot_state_sub = nh.subscribe("/davinci/joint_states", 10, CB_update);
	ROS_INFO("NODE LINKS LATCHED");
	
	//Begin offering the service
	actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> server(nh, "trajActionServer", false);
	server_gp = & server;
	server.registerGoalCallback(&CB_goal);
	server.start();
	
	//Go into spin.
	while (ros::ok()){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}	
	
	return 0;
}

bool exec_position(const trajectory_msgs::JointTrajectoryPoint& in_point, double & elapsed_time){
	double input_gtime = in_point.time_from_start.toSec();
	std::vector<double> input_jnts = in_point.positions;
	
	if(input_jnts.size() != 14){
		ROS_ERROR("DaVinci trajectory points should have 14 joints, and this one has %lu!", input_jnts.size());
		return false;
	}
	
	ROS_INFO("	Go to position %f, %f, %f, %f, %f, %f, %f and %f, %f, %f, %f, %f, %f, %f by %f at %f",
		input_jnts[0],
		input_jnts[1],
		input_jnts[2],
		input_jnts[3],
		input_jnts[4],
		input_jnts[5],
		input_jnts[6],
		input_jnts[7],
		input_jnts[8],
		input_jnts[9],
		input_jnts[10],
		input_jnts[11],
		input_jnts[12],
		input_jnts[13],
		input_gtime,
		elapsed_time
	);
	
	std::vector<std::vector<double> > start_configuration = get_robot_pos();
	std::vector<std::vector<double> > end_configuration = expand_joint_list(input_jnts);
	
	double old_elapsed_time = elapsed_time;
	while(elapsed_time < input_gtime){
		ros::Duration wait_time(dt_traj);
		elapsed_time = elapsed_time + dt_traj;
		std_msgs::Float64 messages[2][13];
		double parametric_value = (elapsed_time - old_elapsed_time) / (input_gtime - old_elapsed_time);
		for(int i = 0; i < 13; i++){
			messages[0][i].data = start_configuration[0][i] + (end_configuration[0][i] - start_configuration[0][i]) * parametric_value;
			messages[1][i].data = start_configuration[1][i] + (end_configuration[1][i] - start_configuration[1][i]) * parametric_value;
		}
		//ROS_ERROR("Shoulder goal is %f", messages[1][0].data);
		for(int i = 0; i < 2; i++){
			for(int j = 0; j < 13; j++){
				publishers[i][j].publish(messages[i][j]);
			}
		}
		ros::spinOnce();
		wait_time.sleep();
	}
		
	return true;
}

std::vector<std::vector<double> > get_robot_pos(){
	fresh_pos = false;
	while(!fresh_pos){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	std::vector<std::vector<double> > rp;
	rp.resize(2);
	rp[0].resize(13);
	rp[1].resize(13);
	
	//Read the robopositions
	for(int i = 0; i < 13; i++){
		Davinci_fwd_solver::get_jnt_val_by_name("one_" + feedback_names[i], states, rp[0][i]);
		Davinci_fwd_solver::get_jnt_val_by_name("two_" + feedback_names[i], states, rp[1][i]);
	}
	
	/*ROS_INFO("Arm two is at: ");
	for(int i  = 0; i < 13; i++){
		ROS_INFO("	%f", rp[1][i]);
	}*/
	
	return rp;
}

//Convert the seven degrees of freedom the trajectories come in into the 13 joints Da Vinci-gazebo actually controls (once for each hand).
std::vector<std::vector<double> > expand_joint_list(const std::vector<double> & input){
	std::vector<std::vector<double> > njl;
	njl.resize(2);
	njl[0].resize(13);
	njl[1].resize(13);
	
	for(int i = 0; i < 2; i++){
		int offset = 7 * i;
		njl[i][0] = input[0 + offset];//joint1_position_controller
 
		njl[i][1] = input[1 + offset];//joint2_position_controller
		njl[i][2] = input[1 + offset];//joint2_1_position_controller
		njl[i][3] = input[1 + offset];//joint2_2_position_controller
		njl[i][6] = input[1 + offset];//joint2_5_position_controller 

		njl[i][4] = -input[1 + offset];//joint2_3_position_controller 
		njl[i][5] = -input[1 + offset];//joint2_4_position_controller
		
		njl[i][7] = input[2 + offset];//joint3_position_controller

		njl[i][8] = input[3 + offset];//joint4_position_controller

		njl[i][9] = input[4 + offset];//joint5_position_controller

		njl[i][10] = input[5 + offset];//joint6_position_controller

		njl[i][11] = input[6 + offset];//joint7_position_controller
		njl[i][12] = -input[6 + offset];//joint7_position_controller_mimic
	}
    
	return njl;
}

void CB_goal(){
	
	//I guess you could just run the whole trajectory in
	//here, but I've kept the execute function.
	CB_execute(server_gp->acceptNewGoal(), server_gp);
	
	//These SUPPOSEDLY don't do anything, but their
	//existance somehow prevents a very serious issue.
	server_gp->isPreemptRequested();
	server_gp->isActive();
	server_gp->isNewGoalAvailable();
}

void CB_execute(
	const davinci_traj_streamer::trajGoalConstPtr& goal,
	actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>* server
){
	ROS_INFO("Got a trajectory request with id %u, %lu positions.", goal->traj_id, goal->trajectory.points.size());
	
	double elapsed_time = 0.0;
	
	for(int i = 0; i < goal->trajectory.points.size(); i++){
		if(!exec_position(goal->trajectory.points[i], elapsed_time)){
			davinci_traj_streamer::trajResult r;
			r.return_val = 0;
			r.traj_id = goal->traj_id;
			server->setAborted();
			return;
		}
	}
	ROS_INFO("Trajectory %u complete.", goal->traj_id);
	davinci_traj_streamer::trajResult r;
	r.return_val = 1;
	r.traj_id = goal->traj_id;
	server->setSucceeded(r);
	
	return;
}

void CB_update(const sensor_msgs::JointState::ConstPtr& incoming){
	fresh_pos = true;
	states = *incoming;
	return;
}
