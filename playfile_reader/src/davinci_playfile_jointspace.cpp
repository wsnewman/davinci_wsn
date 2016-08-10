#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <playfile_reader/playfile_record.h>

#include <davinci_traj_streamer/trajAction.h>

// Read a list of CSV records.
std::istream& operator >> (std::istream & ins, data_t & data){
	// make sure that the returned data only contains the CSV data we read here
	data.clear();

	// For every record we can read from the file, append it to our resulting data
	record_t record;
	while (ins >> record){
		data.push_back(record);
	}

	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;
}

int main(int argc, char **argv) {
	//Set up our node.
	ros::init(argc, argv, "playfile_jointspace");
	ros::NodeHandle nh;
	
	//Locate our file.
	std::string fname;
	if(argc == 2){
		fname = argv[1];
		ROS_INFO("Literal file location: %s", fname.c_str());
	}
	else if(argc == 3){
		std::string packpart = ros::package::getPath(argv[1]).c_str();
		std::string pathpart = argv[2];
		fname = (packpart + pathpart);
		ROS_INFO("Package file location: %s", fname.c_str());
	}
	else{
		ROS_INFO("argc= %d; missing file command-line argument; halting",argc);
		return 0;
	}
	
	//Read the file in.
	std::ifstream infile(fname.c_str());
	if(!infile){//file couldn't be opened
		ROS_ERROR("Error: file %s could not be opened.", fname.c_str());
		exit(1);
	}
	data_t data;
	infile >> data;
	if (!infile.eof()){
		ROS_ERROR("Error: file %s could not be read properly.", fname.c_str());
		exit(1);
	}
	infile.close();
	
	//Perform a few checks...
	unsigned min_record_size = data[0].size();
	unsigned max_record_size = 0;
	for (unsigned n = 0; n < data.size(); n++) {
		if (max_record_size < data[ n ].size())
			max_record_size = data[ n ].size();
		if (min_record_size > data[ n ].size())
			min_record_size = data[ n ].size();
	}
	if (max_record_size>15) {
		ROS_ERROR("Bad file %s: The largest record has %i fields.", fname.c_str(), max_record_size);
		exit(1);
	}
	if (min_record_size<15) {
		ROS_ERROR("Bad file %s: The smallest record has %i fields.", fname.c_str(), min_record_size);
		exit(1);
	}
	
	//Convert into a trajectory message
	trajectory_msgs::JointTrajectory des_trajectory;
	double total_wait_time = 0.0;
	for (unsigned n = 0; n < data.size(); n++){
		trajectory_msgs::JointTrajectoryPoint trajectory_point;
		trajectory_point.positions.resize(14);
		double t_arrival;
		
		//Pack points, one at a time.
		for (int i=0;i<14;i++) {
			trajectory_point.positions[i] = data[n][i];
		}
		t_arrival = data[n][14];
		trajectory_point.time_from_start = ros::Duration(t_arrival);
		
		des_trajectory.points.push_back(trajectory_point);
		
		total_wait_time = t_arrival;
	}
	des_trajectory.header.stamp = ros::Time::now();
	
	//Add an ID.
	davinci_traj_streamer::trajGoal tgoal;
	tgoal.trajectory = des_trajectory;
	srand(time(NULL));
	tgoal.traj_id = rand();
	
	//Locate and lock the action server
	actionlib::SimpleActionClient<
		davinci_traj_streamer::trajAction
	> action_client("trajActionServer", true);
	bool server_exists = action_client.waitForServer(ros::Duration(5.0));
	// something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	ROS_INFO("Waiting for server: ");
	while (!server_exists && ros::ok()) {
		server_exists = action_client.waitForServer(ros::Duration(5.0));
		ROS_WARN("Could not connect to server; retrying...");
	}
	ROS_INFO("SERVER LINK LATCHED");
	
	//Send our message:
	ROS_INFO("Sending trajectory with ID %u", tgoal.traj_id);
	action_client.sendGoal(tgoal);
	
	//In theory, the following function call does absolutely nothing.
	//In practice, it prevents an intermittent bug that rarely makes the
	//action server unusable.
	action_client.getState();
	
	//Wait for it to finish.
	while(!action_client.waitForResult(ros::Duration(total_wait_time + 2.0)) && ros::ok()){
		ROS_WARN("CLIENT TIMED OUT- LET'S TRY AGAIN...");
	}
	
	ROS_INFO(
		"Sevre state is %s, goal state for trajectory %u is %i",
		action_client.getState().toString().c_str(),
		action_client.getResult()->traj_id,
		action_client.getResult()->return_val
	);
	
	//This has to do with the intermittent bug referenced above
	//If you see this appear in your execution, relaunch the entire simulation and everything
	//because it's not recoverable, and contact me immediately- or, better yet, DON'T
	//contact me, because I will have no idea what to do about it.
	if(action_client.getState() ==  actionlib::SimpleClientGoalState::RECALLED){
		ROS_WARN("Server glitch. You may panic now.");
	}

	return 0;
}
