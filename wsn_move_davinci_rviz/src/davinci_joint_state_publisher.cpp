//write a node to movea davinci PSM in rviz;
// crude joint-state publisher
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <Eigen/Eigen>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

const double insertion_offset=0.0156;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dv_joint_state_publisher"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher dvrk_joint_state_publisher = n.advertise<sensor_msgs::JointState>("dvrk_psm/joint_states", 1);

    
    sensor_msgs::JointState jointState; //create an object of type "jointState", 
    Eigen::VectorXd amp_vec;
    amp_vec.resize(7);
    //amp_vec<<1,0.5,0.0,1,1,1,1;
    //amp_vec<<0.2,0.5,0.0,0.5,0.5,0.5,0;
    //amp_vec<<0,0,0,0.5,0.2,0,0;
    amp_vec<<0,0,0,0,0,0,0;    
    Eigen::VectorXd omega_vec;
        omega_vec.resize(7);
    omega_vec<<1,1,1,1,1,0.5,1;
    Eigen::VectorXd q_vec;
    q_vec.resize(7);
    q_vec<<0,0,0,0,0,0,0;
    Eigen::VectorXd phase_vec;
    phase_vec.resize(7);
    phase_vec= q_vec;

    
 //std_msgs/Header header
//  uint32 seq
//  time stamp
//  string frame_id
//string[] name
//float64[] position
//float64[] velocity
//float64[] effort
    jointState.name.push_back("one_outer_yaw_joint");
    jointState.name.push_back("one_outer_pitch_joint");
    jointState.name.push_back("one_outer_pitch_joint_1");
    jointState.name.push_back("one_outer_pitch_joint_2");
    jointState.name.push_back("one_outer_pitch_joint_3");
    jointState.name.push_back("one_outer_pitch_joint_4");
    jointState.name.push_back("one_outer_pitch_joint_5");
    jointState.name.push_back("one_outer_insertion_joint");    
    jointState.name.push_back("one_outer_roll_joint");    
    jointState.name.push_back("one_outer_wrist_pitch_joint");
    jointState.name.push_back("one_outer_wrist_yaw_joint");
    jointState.name.push_back("one_outer_wrist_open_angle_joint");
    jointState.name.push_back("one_outer_wrist_open_angle_joint_mimic");    
    
    for (int i=0;i<13;i++)
        jointState.position.push_back(0.0); // allocate memory and initialize joint values to 0

   double dt = 0.01;
   ros::Rate naptime(dt);  
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        jointState.header.stamp = ros::Time::now();
        phase_vec+= omega_vec*dt;
        for (int i=0;i<7;i++) {
            if (phase_vec[i]>2.0*M_PI) {
                phase_vec[i] -= 2.0*M_PI; //keep phase from accumulating too much
            }
            q_vec[i] = amp_vec[i]*sin(phase_vec[i]);
        }
        q_vec[6] += amp_vec[6]; // force only positive values for gripper opening

        //joint 1: 
        // DEBUG:
        /* */
         q_vec[0] = 0.0;
        q_vec[1]= 0.0;
        q_vec[2] = insertion_offset+0.0;
        q_vec[3] = 0.0;
        q_vec[4] = 0.0; // wrist bend
        q_vec[5] = 0.0; //1.57; // gripper-jaw rotation
        q_vec[6] = 0.0;
      
        jointState.position[0] = q_vec[0];

        //joint2:
        jointState.position[1] = q_vec[1]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointState.position[2] = q_vec[1];
          jointState.position[3] = q_vec[1];
          jointState.position[4] = -q_vec[1];
          jointState.position[5] = -q_vec[1];
          jointState.position[6] = q_vec[1];          
        
        //insertion:
        jointState.position[7] = q_vec[2];
       
        //rotation about tool shaft:
        jointState.position[8] = q_vec[3]; 

        // wrist bend:
        jointState.position[9] = q_vec[4]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointState.position[10] = q_vec[5]-0.5*q_vec[6];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointState.position[11] = q_vec[6];         
        
        //ROS_INFO("ang1 = %f",ang1);
        dvrk_joint_state_publisher.publish(jointState); // publish the joint states
        //ROS_INFO("sleep");
	ros::Duration(dt).sleep(); 
        //ROS_INFO("awake");
    }
}

