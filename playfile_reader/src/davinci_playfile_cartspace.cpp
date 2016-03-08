// example of how to read a cartesian-space trajectory stored line-by-line in CSV file format, 
// perform IK on specified poses, convert to trajectory and play it
//specialized for dual-PSM davinci; assumes fixed order:

//files must be designed to account for relative positions of PSM1 and PSM2 base frames w/rt world

//the following are w/rt to frame ONE_PSM_BASE_LINK
//entries 0-2 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
//entries 3-5 = x-axis direction for PSM1 gripper-tip frame (x-axis points parallel to gripper-jaw rotation axis; y-axis points from fingertip to fingertip)
//entries 6-8 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
//entry 9 = angle of jaw opening (in radians)

// repeat next entries for PSM2:
//the following are w/rt to frame TWO_PSM_BASE_LINK
//entries 10-12 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
//entries 13-15 = x-axis direction for PSM1 gripper-tip frame (x-axis points parallel to gripper-jaw rotation axis; y-axis points from fingertip to fingertip)
//entries 16-18 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
//entry 19 = angle of jaw opening (in radians)

//entry 20 = desired arrival time (from start of trajectory, in seconds)

//each line must contain all 21 values (in fixed order), separated by commas

//the file is read and checked for size consistency (though not for reachability nor speed viability--so WATCH OUT!)
// each gripper pose is converted by IK to a joint-space pose.
// file is packed up as a joint-space "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.
// 

#include<ros/ros.h>
#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>
#include <davinci_traj_streamer/davinci_traj_streamer.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_traj_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_traj_streamer in your package.xml,
// and include the header file below
#include<davinci_traj_streamer/trajAction.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
using namespace std;
typedef vector <double> record_t;
typedef vector <record_t> data_t;

// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.

istream& operator>>(istream& ins, record_t& record) {
    // make sure that the returned record contains only the stuff we read now
    record.clear();

    // read the entire line into a string (a CSV record is terminated by a newline)
    string line;
    getline(ins, line);

    // now we'll use a stringstream to separate the fields out of the line
    stringstream ss(line);
    string field;
    while (getline(ss, field, ',')) {
        // for each field we wish to convert it to a double
        // (since we require that the CSV contains nothing but floating-point values)
        stringstream fs(field);
        double f = 0.0; // (default value is 0.0)
        fs >> f;

        // add the newly-converted field to the end of the record
        record.push_back(f);
    }

    // Now we have read a single line, converted into a list of fields, converted the fields
    // from strings to doubles, and stored the results in the argument record, so
    // we just return the argument stream as required for this kind of input overload function.
    return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator>>(istream& ins, data_t& data) {
    // make sure that the returned data only contains the CSV data we read here
    data.clear();

    // For every record we can read from the file, append it to our resulting data
    record_t record;
    while (ins >> record) {
        data.push_back(record);
    }

    // Again, return the argument stream as required for this kind of input stream overload.
    return ins;
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "playfile_jointspace"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //instantiate a DavinciJointPublisher object and pass in pointer to nodehandle for constructor to use
    DavinciJointPublisher davinciJointPublisher(nh);

    //ROS_INFO("instantiating  forward solver and an ik_solver");
    //Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    //Davinci_IK_solver ik_solver;

    if (argc != 2) {
        ROS_INFO("argc= %d; missing file command-line argument; halting", argc);
        return 0;
    }
    //open the trajectory file:
    ifstream infile(argv[1]);
    if (!infile) // file couldn't be opened
    {
        cerr << "Error: file could not be opened; halting" << endl;
        exit(1);
    }

    // Here is the data we want.
    data_t data;

    // Here is the file containing the data. Read it into data.
    infile >> data;

    // Complain if something went wrong.
    if (!infile.eof()) {
        cout << "error reading file!\n";
        return 1;
    }

    infile.close();

    // Otherwise, list some basic information about the file.
    cout << "CSV file contains " << data.size() << " records.\n";

    unsigned min_record_size = data[0].size();
    unsigned max_record_size = 0;
    for (unsigned n = 0; n < data.size(); n++) {
        if (max_record_size < data[ n ].size())
            max_record_size = data[ n ].size();
        if (min_record_size > data[ n ].size())
            min_record_size = data[ n ].size();
    }
    if (max_record_size > 21) {
        ROS_WARN("bad file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

    }
    if (min_record_size < 21) {
        ROS_WARN("bad file");
        cout << "The smallest record has " << min_record_size << " fields.\n";
        return 1;

    }
    //cout << "The second field in the fourth record contains the value " << data[ 3 ][ 1 ] << ".\n";

    // interpret the data as gripper Cartesian poses:
    //entries 0-2 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
    //entries 3-5 = x-axis direction for PSM1 gripper-tip frame (x-axis points from tip of jaw1 to tip of jaw2)
    //entries 6-8 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
    //entry 9 = angle of jaw opening (in radians)

    vector<Eigen::Vector3d> x_vecs1, y_vecs1, z_vecs1, tip_origins1;
    vector<Eigen::Vector3d> x_vecs2, y_vecs2,z_vecs2, tip_origins2;
    Eigen::Vector3d x_vec1, y_vec1,z_vec1, tip_origin1;
    Eigen::Vector3d x_vec2, y_vec2,z_vec2, tip_origin2;
    vector<double> gripper_angs1, gripper_angs2;
    double gripper_ang1, gripper_ang2;
    vector<double> arrival_times;
    double arrival_time;
    int nposes = data.size();
    for (int n = 0; n < nposes; n++) {
        // pack trajectory points, one at a time:
        for (int i = 0; i < 3; i++) {
            tip_origin1(i) = data[n][i];
            x_vec1(i) = data[n][i + 3];
            z_vec1(i) = data[n][i + 6];
            tip_origin2(i) = data[n][i + 10];
            x_vec2(i) = data[n][i + 13];
            z_vec2(i) = data[n][i + 16];
        }
        gripper_ang1 = data[n][9];
        gripper_ang2 = data[n][19];
        //cout<<"read: gripper_ang2 = "<<gripper_ang2<<endl;
        arrival_time = data[n][20];
        
        x_vecs1.push_back(x_vec1);
        z_vecs1.push_back(z_vec1);
        y_vec1 = z_vec1.cross(x_vec1); //construct consistent y-vec from x and z vecs'
        y_vecs1.push_back(y_vec1);
        tip_origins1.push_back(tip_origin1);
        gripper_angs1.push_back(gripper_ang1);
        
        x_vecs2.push_back(x_vec2);
        z_vecs2.push_back(z_vec2);
        y_vec2 = z_vec2.cross(x_vec2);    
        y_vecs2.push_back(y_vec2);
        tip_origins2.push_back(tip_origin2);
        gripper_angs2.push_back(gripper_ang2);   
        //cout<<"gripper_angs2: "<<gripper_angs2[n]<<endl;
        
        arrival_times.push_back(arrival_time);        
    }
    //organize these as affine poses:
    Eigen::Affine3d des_gripper_affine1,des_gripper_affine2;
    vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
    Eigen::Matrix3d R;
    for (int i=0;i<nposes;i++) {
        R.col(0) = x_vecs1[i];
        R.col(1) = y_vecs1[i];        
        R.col(2) = z_vecs1[i];
        des_gripper_affine1.linear() = R;
        des_gripper_affine1.translation() = tip_origins1[i];
        gripper1_affines.push_back(des_gripper_affine1);
        
        R.col(0) = x_vecs2[i];
        R.col(1) = y_vecs2[i];        
        R.col(2) = z_vecs2[i];
        des_gripper_affine2.linear() = R;
        des_gripper_affine2.translation() = tip_origins2[i];  
        gripper2_affines.push_back(des_gripper_affine2);        
    }
    
    //do IK to convert these to joint angles:
    //Eigen::VectorXd q_vec1,q_vec2;
    Vectorq7x1 q_vec1,q_vec2;
    q_vec1.resize(7);
    q_vec2.resize(7);
    ROS_INFO("instantiating  forward solver and an ik_solver");
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver;
    
    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
    // if using wsn's trajectory streamer action server
    des_trajectory.header.stamp = ros::Time::now();

    trajectory_msgs::JointTrajectoryPoint trajectory_point; //,trajectory_point2; 
    trajectory_point.positions.resize(14);    
    
    for (int ipose=0;ipose<nposes;ipose++) {   
        des_gripper_affine1 = gripper1_affines[ipose];
        ik_solver.ik_solve(des_gripper_affine1); //convert desired pose into equiv joint displacements
        q_vec1 = ik_solver.get_soln(); 
        q_vec1(6) = gripper_angs1[ipose];
        cout<<"qvec1: "<<q_vec1.transpose()<<endl;
        
        des_gripper_affine2 = gripper2_affines[ipose];
        ik_solver.ik_solve(des_gripper_affine2); //convert desired pose into equiv joint displacements
        q_vec2 = ik_solver.get_soln();  
        q_vec2(6) = gripper_angs2[ipose];
        cout<<"qvec2: "<<q_vec2.transpose()<<endl;        
        //repackage q's into a trajectory;
        for (int i=0;i<7;i++) {
            trajectory_point.positions[i] = q_vec1(i);
            trajectory_point.positions[i+7] = q_vec2(i);  
            //should fix up jaw-opening values...do this later
        }
        trajectory_point.time_from_start = ros::Duration(arrival_times[ipose]);
        des_trajectory.points.push_back(trajectory_point);        
        
    }
    

    //now have the data in a trajectory; send it to the trajectory streamer action server to execute

    // here is a "goal" object compatible with the server, as defined in example_action_server/action    
    // copy traj to goal:   
    davinci_traj_streamer::trajGoal goal;
    goal.trajectory = des_trajectory;
    //cout<<"ready to connect to action server; enter 1: ";
    //cin>>ans;
    // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
    actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> action_client("trajActionServer", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        int max_tries = 0;
        while (!server_exists) {
           server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
           // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
           ros::spinOnce();
           ros::Duration(0.1).sleep();
           ROS_INFO("retrying...");
           max_tries++;
           if (max_tries>100)
               break;
        }

    if (!server_exists) {
        ROS_WARN("could not connect to server; quitting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    //server_exists = action_client.waitForServer(); //wait forever 


    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    // stuff a goal message:
    //g_count++;
    //goal.traj_id = g_count; // this merely sequentially numbers the goals sent
    //ROS_INFO("sending traj_id %d",g_count);
    //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
    action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
    //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    double t_timeout=arrival_time+2.0; //wait 2 sec longer than expected duration of move
    
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(t_timeout));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 0;
    } else {
        ROS_INFO("finished before timeout");
    }



    cout << "Good bye!\n";
    return 0;
}


