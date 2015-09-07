// example_ros_class.h header file //
// wsn; Feb, 2015

#ifndef DAVINCI_JOINT_PUBLISHER_H_
#define DAVINCI_JOINT_PUBLISHER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <davinci_kinematics/davinci_kinematics.h>
//#include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package

// define a class, including a constructor, member variables and member functions
class DavinciJointPublisher
{
public:
    //DavinciJointPublisher(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    DavinciJointPublisher(ros::NodeHandle &nh); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor

    // may choose to define public methods or public variables, if desired
    void pubJointStates(Eigen::VectorXd q_vec);
    void pubJointStates(Vectorq7x1 q_vec1, Vectorq7x1 q_vec2);

    void pubJointStates(Vectorq7x1 q_vec);
    void pubJointStates2(Eigen::VectorXd q_vec);

private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer minimal_service_;
    ros::Publisher  joint_state_publisher_;
    ros::Publisher  joint_state_publisher2_;    
    //double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    //double val_to_remember_; // member variables will retain their values even as callbacks come and go
     sensor_msgs::JointState jointState_; //create an object of type "jointState", 
     sensor_msgs::JointState jointState2_; // and for 2nd psm
    // member methods as well:
    //void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();
   
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    //prototype for callback for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
