// wsn pgm to receive Davinci trajectories and interpolate them smoothly
// as commands to Davinci;
// starting w/ just 4DOF (no gripper)
#include <davinci_traj_streamer/davinci_traj_streamer.h>
//#include <davinci_kinematics/davinci_joint_publisher.h>
#include <actionlib/server/simple_action_server.h>

//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../baxter_traj_streamer/action/traj.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (traj) and appended name (Action)
#include<davinci_traj_streamer/trajAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;


/* maybe restore this later
bool trajInterpStatusSvc(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
//ROS_INFO("responding to service request: status of trajectory interpolator");
response.resp = working_on_trajectory; // return status of "working on trajectory"
return true;
}
*/


class TrajActionServer {
	private:

		ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

		// this class will own a "SimpleActionServer" called "as_".
		// it will communicate using messages defined in baxter_traj_streamer/action/traj.action
		// the type "trajAction" is auto-generated from our name "traj" and generic name "Action"
		actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> as_;

		// here are some message types to communicate with our client(s)
		davinci_traj_streamer::trajGoal goal_; // goal message, received from client
		davinci_traj_streamer::trajResult result_; // put results here, to be sent back to the client when done w/ goal
		davinci_traj_streamer::trajFeedback feedback_; // not used in this example; 
		// would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
		//baxter_core_msgs::JointCommand right_cmd, left_cmd;
		trajectory_msgs::JointTrajectory new_trajectory; // member var to receive new traj's;
		int g_count; //=0; //just for testing
		bool working_on_trajectory; // = false;
		void command_joints(Eigen::VectorXd q_cmd);
		ros::Publisherj1_pub_,j2_pub_,j2_1_pub_,j2_2_pub_,j2_3_pub_,j2_4_pub_,j2_5_pub_,j3_pub_,j4_pub_,j5_pub_,j6_pub_,j7_pub_;
		void initializePublishers();
		//DavinciJointPublisher davinciJointPublisher; //(&nh_); //:&nh_;//(&nh_);//(nh_); //DavinciJointPublisher davinciJointPublisher(&nh);
	public:
		//TrajActionServer(ros::NodeHandle nh);
		TrajActionServer(ros::NodeHandle &nh); //define the body of the constructor outside of class definition
	
		~TrajActionServer(void) {
		}
		// Action Interface
		void executeCB(const actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>::GoalConstPtr& goal);
		bool update_trajectory(
			double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev, 
			int &isegment, Eigen::VectorXd &qvec_new
		);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//a pointer to the function to be executed upon receipt of a goal.
//
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.the _1 argument says that our executeCB takes one argument
// the final argument"false" says don't start the server yet.(We'll do this in the constructor)
//TrajActionServer::TrajActionServer(ros::NodeHandle nh) :nh_(nh),
TrajActionServer::TrajActionServer(ros::NodeHandle &nh)
	:nh_(nh),as_(nh, "trajActionServer", boost::bind(&TrajActionServer::executeCB, this, _1),false){
	// in the above initialization, we name the server "example_action"
	//clients will need to refer to this name to connect with this server
	ROS_INFO("in constructor of TrajActionServer...");
	// do any other desired initializations here...specific to your implementation
	//initializations:
	/*should not need this: use davinci_joint_publisher object

	left_cmd.mode = 1; // set the command modes to "position"
	right_cmd.mode = 1;
		// define the joint angles 0-6 to be right arm, from shoulder out to wrist;
	right_cmd.names.push_back("right_s0");
	right_cmd.names.push_back("right_s1");
	right_cmd.names.push_back("right_e0");
	right_cmd.names.push_back("right_e1");
	right_cmd.names.push_back("right_w0");
	right_cmd.names.push_back("right_w1");
	right_cmd.names.push_back("right_w2");
	// do push-backs to establish desired vector size with valid joint angles
	for (int i = 0; i < 7; i++) {
		right_cmd.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
		left_cmd.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
	}
	 * */
	//DavinciJointPublisher davinciJointPublisher(&nh_); // ugly...used pointer to davinciJointPublisher to initialize in constructor
	//davinciJointPublisherPtr = &davinciJointPublisher;
	//DavinciJointPublisher davinciJointPublisher;
	initializePublishers();
	g_count = 0;
	working_on_trajectory = false;
	ROS_INFO("starting action server: trajActionServer ");
	as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <baxter_traj_streamer::trajAction> customizes the simple action server to use our own "action" message 
// defined in our package, "baxter_traj_streamer", in the subdirectory "action", called "traj.action"
// The name "traj" is prepended to other message types created automatically during compilation.
// e.g.,"trajAction" is auto-generated from (our) base name "traj" and generic name "Action"

/*
void TrajActionServer::cmd_pose_right(Vectorq7x1 qvec) {
	//member var right_cmd_ already has joint names populated
	for (int i = 0; i < 7; i++) {
		right_cmd.command[i] = qvec[i];
	}
	joint_cmd_pub_right.publish(right_cmd);
}
*/


void TrajActionServer::initializePublishers() {
	//ros::Publisherj1_pub_,j2_pub_,j2_1_pub_,j2_2_pub_,j2_3_pub_,j2_4_pub_,j2_5_pub_,j3_pub_,j4_pub_,j5_pub_,j6_pub_,j7_pub_;

	j1_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint1_position_controller/command", 1, true); 
	j2_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint2_position_controller/command", 1, true);
	j2_1_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint2_1_position_controller/command", 1, true);
	j2_2_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint2_2_position_controller/command", 1, true);
	j2_3_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint2_3_position_controller/command", 1, true);
	j2_4_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint2_4_position_controller/command", 1, true);	
	j2_5_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint2_5_position_controller/command", 1, true);
	j3_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint3_position_controller/command", 1, true);
	j4_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint4_position_controller/command", 1, true);
	j5_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint5_position_controller/command", 1, true);
	j6_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint6_position_controller/command", 1, true);
	j7_pub_ =nh_.advertise<std_msgs::Float64>("/davinci/joint7_position_controller/command", 1, true);
}

void TrajActionServer::command_joints(Eigen::VectorXd q_cmd) {
	int njnts = q_cmd.size();
	std_msgs::Float64 qval_msg;
	qval_msg.data = q_cmd(0);
	j1_pub_.publish(qval_msg);
	if (njnts>1) {
		qval_msg.data=q_cmd(1);
		j2_pub_.publish(qval_msg);
		// mimic this for pitch mechanism joints:
		j2_1_pub_.publish(qval_msg);
		j2_2_pub_.publish(qval_msg);
		j2_5_pub_.publish(qval_msg);
		// the following 2 joints follow w/ negative of j2
		qval_msg.data= -q_cmd(1);
		j2_3_pub_.publish(qval_msg);
		j2_4_pub_.publish(qval_msg);
	}
	if (njnts>2) {
		qval_msg.data=q_cmd(2);
		j3_pub_.publish(qval_msg);
	}
	if (njnts>3) {
		qval_msg.data=q_cmd(3);
		j4_pub_.publish(qval_msg);
	}
	if (njnts>4) {
		qval_msg.data=q_cmd(4);
		j5_pub_.publish(qval_msg);
	}
	if (njnts>5) {
		qval_msg.data=q_cmd(5);
		j6_pub_.publish(qval_msg);
	}
	if (njnts>6) {
		qval_msg.data=q_cmd(6);
		j7_pub_.publish(qval_msg);
	}
}

void TrajActionServer::executeCB(const actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>::GoalConstPtr& goal) {
	double traj_clock, dt_segment, dq_segment, delta_q_segment, traj_final_time;
	int isegment;
	trajectory_msgs::JointTrajectoryPoint trajectory_point0;

	Eigen::VectorXd qvec, qvec0, qvec_prev, qvec_new;
	// TEST TEST TEST
	//Eigen::VectorXd q_vec;
	//q_vec<<0.1,0.2,0.15,0.4,0.5,0.6,0.7;

	ROS_INFO("in executeCB");

	g_count++; // keep track of total number of goals serviced since this server was started
	result_.return_val = g_count; // we'll use the member variable result_, defined in our class
	result_.traj_id = goal->traj_id;
	cout<<"received trajectory w/ "<<goal->trajectory.points.size()<<" points"<<endl;
	// copy trajectory to global var:
	new_trajectory = goal->trajectory; // 
	// insist that a traj have at least 2 pts
	int npts = new_trajectory.points.size();
	if (npts< 2) {
		ROS_WARN("too few points; aborting goal");
		as_.setAborted(result_);
	}
	else { //OK...have a valid trajectory goal; execute it
		//got_new_goal = true;
		//got_new_trajectory = true;
		ROS_INFO("Cb received traj w/ npts = %d",npts);
		//cout << "Cb received traj w/ npts = " << new_trajectory.points.size() << endl;
		//trajectory_msgs::JointTrajectoryPoint trajectory_point0;
		//trajectory_point0 = new_trajectory.points[0];
		//trajectory_point0 =tj_msg.points[0]; 
		//cout<<new_trajectory.points[0].positions.size()<<" =new_trajectory.points[0].positions.size()"<<endl;
		//cout<<"size of positions[]: "<<trajectory_point0.positions.size()<<endl;
		cout << "subgoals: " << endl;
		int njnts; 
		for (int i = 0; i < npts; i++) {
			njnts = new_trajectory.points[i].positions.size();
			cout<<"njnts: "<<njnts<<endl;
			for (int j = 0; j < njnts; j++) { //copy from traj point to 7x1 vector
				cout << new_trajectory.points[i].positions[j] << ", ";
			}
			cout<<endl;
			cout<<"time from start: "<<new_trajectory.points[i].time_from_start.toSec()<<endl;
			cout << endl;
		}

		as_.isActive();

		working_on_trajectory = true;
		//got_new_trajectory=false;
		traj_clock = 0.0; // initialize clock for trajectory;
		isegment = 0;
		trajectory_point0 = new_trajectory.points[0];
		njnts = new_trajectory.points[0].positions.size();
		int njnts_new;
		qvec_prev.resize(njnts);
		qvec_new.resize(njnts);
		ROS_INFO("populating qvec_prev: ");
		for (int i = 0; i < njnts; i++) { //copy from traj point to Eigen type vector
			qvec_prev[i] = trajectory_point0.positions[i];
		}
		//cmd_pose_right(qvec0); //populate and send out first command
		//qvec_prev = qvec0;
		cout << "start pt: " << qvec_prev.transpose() << endl;
	}
	while (working_on_trajectory) {
		traj_clock += dt_traj;
		// update isegment and qvec according to traj_clock; 
		//if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false 
		//ROS_INFO("traj_clock = %f; updating qvec_new",traj_clock);
		working_on_trajectory = update_trajectory(traj_clock, new_trajectory, qvec_prev, isegment, qvec_new);
		//cmd_pose_right(qvec_new); // use qvec to populate object and send it to robot
		//ROS_INFO("publishing qvec_new as command");
		//davinciJointPublisher.pubJointStatesAll(qvec_new);
		command_joints(qvec_new);//map these to all gazebo joints and publish as commands
		qvec_prev = qvec_new;
	
		//cout << "traj_clock: " << traj_clock << "; vec:" << qvec_new.transpose() << endl;
		ros::spinOnce();
		ros::Duration(dt_traj).sleep();
	}
	ROS_INFO("completed execution of a trajectory" );
	ROS_INFO("sending back a result %i %i", result_.return_val, result.traj_id);
	as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message 
}

	// more general version--arbitrary number of joints
bool TrajActionServer::update_trajectory(
	double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev, 
	int &isegment, Eigen::VectorXd &qvec_new
){

	trajectory_msgs::JointTrajectoryPoint trajectory_point_from, trajectory_point_to;
	int njnts = qvec_prev.size();
	cout<<"njnts for qvec_prev: "<<njnts<<endl;
	Eigen::VectorXd qvec, qvec_to, delta_qvec, dqvec;
	int nsegs = trajectory.points.size() - 1;
	ROS_INFO("update_trajectory: nsegs = %d, isegment = %d",nsegs,isegment);
	double t_subgoal;
	//cout<<"traj_clock = "<<traj_clock<<endl;
	if (isegment < nsegs) {
		trajectory_point_to = trajectory.points[isegment + 1];
		t_subgoal = trajectory_point_to.time_from_start.toSec();
		cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
	}
	else {
		cout << "reached end of last segment" << endl;
		trajectory_point_to = trajectory.points[nsegs];
		t_subgoal = trajectory_point_to.time_from_start.toSec();

		for (int i = 0; i < njnts; i++) {
			qvec_new[i] = trajectory_point_to.positions[i];
		}
		cout << "final time: " << t_subgoal << endl;
		return false;
	}

	cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
	while ((t_subgoal < traj_clock)&&(isegment < nsegs)) {
		cout<<"loop: iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
		isegment++;
		if (isegment > nsegs - 1) {
			//last point
			trajectory_point_to = trajectory.points[nsegs];
			cout<<"next traj pt #jnts = "<<trajectory_point_to.positions.size()<<endl;
			for (int i = 0; i < njnts; i++) {
				qvec_new[i] = trajectory_point_to.positions[i];
			}
			cout << "iseg>nsegs" << endl;
			return false;
		}

		trajectory_point_to = trajectory.points[isegment + 1];
		t_subgoal = trajectory_point_to.time_from_start.toSec();
	}
	//cout<<"t_subgoal = "<<t_subgoal<<endl;
	//here if have a valid segment:
	cout<<"njnts of trajectory_point_to: "<<trajectory_point_to.positions.size()<<endl;
	qvec_to.resize(njnts);
	for (int i = 0; i < njnts; i++) {
		qvec_to[i] = trajectory_point_to.positions[i];
	}
	delta_qvec.resize(njnts);
	delta_qvec = qvec_to - qvec_prev; //this far to go until next node;
	double delta_time = t_subgoal - traj_clock;
	if (delta_time < dt_traj) delta_time = dt_traj;
		dqvec.resize(njnts);
	dqvec = delta_qvec * dt_traj / delta_time;
	qvec_new = qvec_prev + dqvec;
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "traj_interpolator_action_server"); // name this node 
	ros::NodeHandle nh;
	//DavinciJointPublisher davinciJointPublisher(&nh);//(nh_); //DavinciJointPublisher davinciJointPublisher(&nh);

	//publisher is global
	//joint_cmd_pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
	
	/* maybe restore this later...
	ROS_INFO("Initializing Services");
	ros::ServiceServer statusService = nh.advertiseService("trajInterpStatusSvc", trajInterpStatusSvc);
	*/
 
	ROS_INFO("instantiating the trajectory interpolator action server: ");
	TrajActionServer as(nh); //(nh); // create an instance of the class "TrajActionServer"

	ROS_INFO("ready to receive/execute trajectories");
	//main loop:
	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(dt_traj).sleep();
	}
}
