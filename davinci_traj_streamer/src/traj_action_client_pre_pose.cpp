// traj_action_client_pre_pose: 
// wsn, Sept, 2015...variation from baxter for testing traj_interpolator_as action server

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <davinci_traj_streamer/davinci_traj_streamer.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_traj_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_traj_streamer in your package.xml,
// and include the header file below
#include<davinci_traj_streamer/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}


int main(int argc, char** argv) {
        ros::init(argc, argv, "traj_action_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        
        int g_count = 0;
                int ans;
    Vectorq7x1 q_pre_pose;
    //q_in << 0, 0, 0, 0, 0, 0, 0;  

    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 qvec_1,qvec_2;
    qvec_1<< 0,0,0,0,0,0,0;
    qvec_2<< -0.5, 0.4,  0.1,    0.6,    -0.7,   0.8,  0;       
  
      // std::vector<Eigen::VectorXd> des_path;
        // cout<<"creating des_path vector; enter 1:";
        //cin>>ans;
        

        trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
        trajectory_msgs::JointTrajectoryPoint trajectory_point1,trajectory_point2; 
        

           
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        davinci_traj_streamer::trajGoal goal; 
 

        trajectory_point1.positions.clear(); 
           trajectory_point2.positions.clear();
       trajectory_point1.time_from_start =    ros::Duration(0.0);
       trajectory_point2.time_from_start =    ros::Duration(1.0);
    for (int i=0;i<7;i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(qvec_1(i));
        trajectory_point2.positions.push_back(qvec_2(i));
    } 

    des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory.joint_names.clear(); 
    //    des_trajectory.joint_names.push_back("right_s0"); //yada-yada should fill in names
    des_trajectory.header.stamp = ros::Time::now(); 
       
    des_trajectory.points.push_back(trajectory_point1); // first point of the trajectory 
    des_trajectory.points.push_back(trajectory_point2);    
        // copy traj to goal:
        goal.trajectory = des_trajectory;
        //cout<<"ready to connect to action server; enter 1: ";
        //cin>>ans;
        // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
        actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> action_client("trajActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


        if (!server_exists) {
            ROS_WARN("could not connect to server; quitting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        //server_exists = action_client.waitForServer(); //wait forever 
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        // stuff a goal message:
        g_count++;
        goal.traj_id = g_count; // this merely sequentially numbers the goals sent
        ROS_INFO("sending traj_id %d",g_count);
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }
        
        //}

    return 0;
}

