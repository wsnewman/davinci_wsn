//davinci_joint_publisher_test_main.cpp:
//wsn, Sept 2015
// example use of the joint_publisher class


// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include <davinci_kinematics/davinci_joint_publisher.h>


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "davinci_joint_state_publisher"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    //ROS_INFO("main: instantiating an object of type ExampleRosClass");
    DavinciJointPublisher davinciJointPublisher(nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    // define a vector of desired joint displacements...w/o linkage redundancies
    Eigen::VectorXd q_vec;
    q_vec.resize(7);
    q_vec<<0.1,0.2,0.15,0.4,0.5,0.6,0.7;
    
    //use the publisher object to map these correctly and send them to rviz

    
    for (int i=0;i<100;i++) {
     davinciJointPublisher.pubJointStates(q_vec); //publish a bunch of times, to make sure
        ros::spinOnce();   
        ros::Duration(0.1).sleep(); 
    }

    return 0;
} 

