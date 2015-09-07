//davinci_joint_publisher.cpp:
//wsn, Sept 2015
// create a class to simplify publishing jointStates for Davinci rviz model
// this would get replaced by a Gazebo model, when one is available


// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include <davinci_kinematics/davinci_joint_publisher.h>


//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
//DavinciJointPublisher::DavinciJointPublisher(ros::NodeHandle* nodehandle):nh_(*nodehandle)
//DavinciJointPublisher(ros::NodeHandle *nh);
DavinciJointPublisher::DavinciJointPublisher(ros::NodeHandle &nh):nh_(nh)
{ // constructor
    ROS_INFO("in class constructor of DavinciJointPublisher");
    //initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();

    jointState_.name.push_back("one_outer_yaw_joint");
    jointState_.name.push_back("one_outer_pitch_joint");
    jointState_.name.push_back("one_outer_pitch_joint_1");
    jointState_.name.push_back("one_outer_pitch_joint_2");
    jointState_.name.push_back("one_outer_pitch_joint_3");
    jointState_.name.push_back("one_outer_pitch_joint_4");
    jointState_.name.push_back("one_outer_pitch_joint_5");
    jointState_.name.push_back("one_outer_insertion_joint");    
    jointState_.name.push_back("one_outer_roll_joint");    
    jointState_.name.push_back("one_outer_wrist_pitch_joint");
    jointState_.name.push_back("one_outer_wrist_yaw_joint");
    jointState_.name.push_back("one_outer_wrist_open_angle_joint");
    jointState_.name.push_back("one_outer_wrist_open_angle_joint_mimic");    
    
    for (int i=0;i<13;i++)
        jointState_.position.push_back(0.0); // allocate memory and initialize joint values to 0

        jointState2_.name.push_back("two_outer_yaw_joint");
    jointState2_.name.push_back("two_outer_pitch_joint");
    jointState2_.name.push_back("two_outer_pitch_joint_1");
    jointState2_.name.push_back("two_outer_pitch_joint_2");
    jointState2_.name.push_back("two_outer_pitch_joint_3");
    jointState2_.name.push_back("two_outer_pitch_joint_4");
    jointState2_.name.push_back("two_outer_pitch_joint_5");
    jointState2_.name.push_back("two_outer_insertion_joint");    
    jointState2_.name.push_back("two_outer_roll_joint");    
    jointState2_.name.push_back("two_outer_wrist_pitch_joint");
    jointState2_.name.push_back("two_outer_wrist_yaw_joint");
    jointState2_.name.push_back("two_outer_wrist_open_angle_joint");
    jointState2_.name.push_back("two_outer_wrist_open_angle_joint_mimic");    

    jointStatesBoth_.name.push_back("one_outer_yaw_joint");
    jointStatesBoth_.name.push_back("one_outer_pitch_joint");
    jointStatesBoth_.name.push_back("one_outer_pitch_joint_1");
    jointStatesBoth_.name.push_back("one_outer_pitch_joint_2");
    jointStatesBoth_.name.push_back("one_outer_pitch_joint_3");
    jointStatesBoth_.name.push_back("one_outer_pitch_joint_4");
    jointStatesBoth_.name.push_back("one_outer_pitch_joint_5");
    jointStatesBoth_.name.push_back("one_outer_insertion_joint");    
    jointStatesBoth_.name.push_back("one_outer_roll_joint");    
    jointStatesBoth_.name.push_back("one_outer_wrist_pitch_joint");
    jointStatesBoth_.name.push_back("one_outer_wrist_yaw_joint");
    jointStatesBoth_.name.push_back("one_outer_wrist_open_angle_joint");
    jointStatesBoth_.name.push_back("one_outer_wrist_open_angle_joint_mimic");    
    
    jointStatesBoth_.name.push_back("two_outer_yaw_joint");
    jointStatesBoth_.name.push_back("two_outer_pitch_joint");
    jointStatesBoth_.name.push_back("two_outer_pitch_joint_1");
    jointStatesBoth_.name.push_back("two_outer_pitch_joint_2");
    jointStatesBoth_.name.push_back("two_outer_pitch_joint_3");
    jointStatesBoth_.name.push_back("two_outer_pitch_joint_4");
    jointStatesBoth_.name.push_back("two_outer_pitch_joint_5");
    jointStatesBoth_.name.push_back("two_outer_insertion_joint");    
    jointStatesBoth_.name.push_back("two_outer_roll_joint");    
    jointStatesBoth_.name.push_back("two_outer_wrist_pitch_joint");
    jointStatesBoth_.name.push_back("two_outer_wrist_yaw_joint");
    jointStatesBoth_.name.push_back("two_outer_wrist_open_angle_joint");
    jointStatesBoth_.name.push_back("two_outer_wrist_open_angle_joint_mimic");  
   
    for (int i=0;i<26;i++)
        jointStatesBoth_.position.push_back(0.0); // allocate memory and initialize joint values to 0

}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
/*
void DavinciJointPublisher::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &ExampleRosClass::subscriberCallback,this);  
    // add more subscribers here, as needed
}
*/
//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
/*
void DavinciJointPublisher::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh_.advertiseService("exampleMinimalService",
                                                   &ExampleRosClass::serviceCallback,
                                                   this);  
    // add more services here, as needed
}
*/

//member helper function to set up publishers;
void DavinciJointPublisher::initializePublishers()
{
    ROS_INFO("Initializing Publisher");
    joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("dvrk_psm/joint_states", 1);
    //can re-use the above publisher for right, left or both
    joint_state_publisher2_ = nh_.advertise<sensor_msgs::JointState>("dvrk_psm/joint_states", 1); 
    
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

//alt: accept either Vectorq7x1 or Eigen::Vector Xd as args
void DavinciJointPublisher::pubJointStates(Vectorq7x1 q_vec) {
    Eigen::VectorXd q_vec_xd;
    q_vec_xd.resize(7);
    for (int i=0;i<7;i++) q_vec_xd(i) = q_vec(i);
    pubJointStates(q_vec_xd);
    
}

//alt: accept either Vectorq7x1 or Eigen::Vector Xd as args
void DavinciJointPublisher::pubJointStates(Vectorq7x1 q_vec1, Vectorq7x1 q_vec2) {
    Eigen::VectorXd q_vec_xd1, q_vec_xd2;
    q_vec_xd1.resize(7);
    q_vec_xd2.resize(7);    
    for (int i=0;i<7;i++) {
        q_vec_xd1(i) = q_vec1(i);
        q_vec_xd2(i) = q_vec2(i);  
    }
    pubJointStates(q_vec_xd1); //,q_vec_xd2);
    //cout<<"qvec1: "<<q_vec_xd1.transpose()<<endl;
    ros::spinOnce(); //really only need this if have a subscriber or service running
    ros::Duration(0.002).sleep(); //loop timer    
    pubJointStates2(q_vec_xd2);
    //cout<<"qvec2: "<<q_vec_xd2.transpose()<<endl;    
}

void DavinciJointPublisher::pubJointStates(Eigen::VectorXd q_vec) {
    
            jointState_.header.stamp = ros::Time::now();
       jointState_.position[0] = q_vec[0];

        //joint2:
        jointState_.position[1] = q_vec[1]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointState_.position[2] = q_vec[1];
          jointState_.position[3] = q_vec[1];
          jointState_.position[4] = -q_vec[1];
          jointState_.position[5] = -q_vec[1];
          jointState_.position[6] = q_vec[1];          
        
        //insertion:
        jointState_.position[7] = q_vec[2];
       
        //rotation about tool shaft:
        jointState_.position[8] = q_vec[3]; 

        // wrist bend:
        jointState_.position[9] = q_vec[4]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointState_.position[10] = q_vec[5]-0.5*q_vec[6];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointState_.position[11] = q_vec[6]; 
        jointState_.position[12] = -q_vec[6]; //a mimic joint of outer_wrist_open_angle_joint       
        
        //two_outer_wrist_open_angle_joint_mimic: position[12]; need to look up what this does
        //ROS_INFO("ang1 = %f",ang1);
        joint_state_publisher_.publish(jointState_); // publish the joint states
    
}

void DavinciJointPublisher::pubJointStates2(Eigen::VectorXd q_vec) {
    
       jointState2_.header.stamp = ros::Time::now();
       jointState2_.position[0] = q_vec[0];

        //joint2:
        jointState2_.position[1] = q_vec[1]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointState2_.position[2] = q_vec[1];
          jointState2_.position[3] = q_vec[1];
          jointState2_.position[4] = -q_vec[1];
          jointState2_.position[5] = -q_vec[1];
          jointState2_.position[6] = q_vec[1];          
        
        //insertion:
        jointState2_.position[7] = q_vec[2];
       
        //rotation about tool shaft:
        jointState2_.position[8] = q_vec[3]; 

        // wrist bend:
        jointState2_.position[9] = q_vec[4]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointState2_.position[10] = q_vec[5]-0.5*q_vec[6];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointState2_.position[11] = q_vec[6];         
        
        //ROS_INFO("ang1 = %f",ang1);
        joint_state_publisher2_.publish(jointState2_); // publish the joint states
    
}

//given desired angles (7dof) for both left and right arms, combine these and publish
void DavinciJointPublisher::pubJointStatesAll(Eigen::VectorXd q_vec1, Eigen::VectorXd q_vec2) {
    
       jointStatesBoth_.header.stamp = ros::Time::now();
       jointStatesBoth_.position[0] = q_vec1[0];

        //joint2:
        jointStatesBoth_.position[1] = q_vec1[1]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointStatesBoth_.position[2] = q_vec1[1];
          jointStatesBoth_.position[3] = q_vec1[1];
          jointStatesBoth_.position[4] = -q_vec1[1];
          jointStatesBoth_.position[5] = -q_vec1[1];
          jointStatesBoth_.position[6] = q_vec1[1];          
        
        //insertion:
        jointStatesBoth_.position[7] = q_vec1[2];
       
        //rotation about tool shaft:
        jointStatesBoth_.position[8] = q_vec1[3]; 

        // wrist bend:
        jointStatesBoth_.position[9] = q_vec1[4]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointStatesBoth_.position[10] = q_vec1[5]-0.5*q_vec1[6];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointStatesBoth_.position[11] = q_vec1[6];  
        
        // repeat for other arm, appending these values:
       jointStatesBoth_.position[0+13] = q_vec2[0];
        jointStatesBoth_.position[1+13] = q_vec2[1]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointStatesBoth_.position[2+13] = q_vec2[1];
          jointStatesBoth_.position[3+13] = q_vec2[1];
          jointStatesBoth_.position[4+13] = -q_vec2[1];
          jointStatesBoth_.position[5+13] = -q_vec2[1];
          jointStatesBoth_.position[6+13] = q_vec2[1];          
        
        //insertion:
        jointStatesBoth_.position[13+7] = q_vec2[2];
       
        //rotation about tool shaft:
        jointStatesBoth_.position[8+13] = q_vec2[3]; 

        // wrist bend:
        jointStatesBoth_.position[9+13] = q_vec2[4]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointStatesBoth_.position[10+13] = q_vec2[5]-0.5*q_vec2[6];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointStatesBoth_.position[11+13] = q_vec2[6];         
        
        //ROS_INFO("ang1 = %f",ang1);
        joint_state_publisher_.publish(jointStatesBoth_); // publish the joint states               
               
    }

    void DavinciJointPublisher::pubJointStatesAll(Eigen::VectorXd qVecAll) {
       jointStatesBoth_.header.stamp = ros::Time::now();
       
       jointStatesBoth_.position[0] = qVecAll[0];

        //joint2:
        jointStatesBoth_.position[1] = qVecAll[1]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointStatesBoth_.position[2] = qVecAll[1];
          jointStatesBoth_.position[3] = qVecAll[1];
          jointStatesBoth_.position[4] = -qVecAll[1];
          jointStatesBoth_.position[5] = -qVecAll[1];
          jointStatesBoth_.position[6] = qVecAll[1];          
        
        //insertion:
        jointStatesBoth_.position[7] = qVecAll[2];
       
        //rotation about tool shaft:
        jointStatesBoth_.position[8] = qVecAll[3]; 

        // wrist bend:
        jointStatesBoth_.position[9] = qVecAll[4]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointStatesBoth_.position[10] = qVecAll[5]-0.5*qVecAll[6];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointStatesBoth_.position[11] = qVecAll[6];  
        jointState_.position[12] = -qVecAll[6]; //a mimic joint of outer_wrist_open_angle_joint       

        // repeat for other arm, appending these values:
       jointStatesBoth_.position[0+13] = qVecAll[0+7];
        jointStatesBoth_.position[1+13] = qVecAll[1+7]; // main joint for jnt2
        //the following joints mimic jnt2, just for visualization
          jointStatesBoth_.position[2+13] = qVecAll[1+7];
          jointStatesBoth_.position[3+13] = qVecAll[1+7];
          jointStatesBoth_.position[4+13] = -qVecAll[1+7];
          jointStatesBoth_.position[5+13] = -qVecAll[1+7];
          jointStatesBoth_.position[6+13] = qVecAll[1+7];          
        
        //insertion:
        jointStatesBoth_.position[7+13] = qVecAll[2+7];
       
        //rotation about tool shaft:
        jointStatesBoth_.position[8+13] = qVecAll[3+7]; 

        // wrist bend:
        jointStatesBoth_.position[9+13] = qVecAll[4+7]; 
        
        //jaws:
        // q_vec[5] rotates both jaws together
       
        jointStatesBoth_.position[10+13] = qVecAll[5+7]-0.5*qVecAll[6+7];  
        
        // this rotates only 1 jaw; command only positive values,
        // so controls gripper opening; q6=0 --> gripper closed
        
        jointStatesBoth_.position[11+13] = qVecAll[6+7];         
        jointState_.position[12+13] = -qVecAll[6+7]; //a mimic joint of outer_wrist_open_angle_joint       
        
        //ROS_INFO("ang1 = %f",ang1);
        joint_state_publisher_.publish(jointStatesBoth_); // publish the joint states           
    }

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)
/*
void DavinciJointPublisher::subscriberCallback(const std_msgs::Float32& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data= val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); //output the square of the received value; 
}
*/

/*
//member function implementation for a service callback function
bool DavinciJointPublisher::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    return true;
}
*/

// example use:
/*
int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "davinci_joint_state_publisher"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    //ROS_INFO("main: instantiating an object of type ExampleRosClass");
    DavinciJointPublisher davinciJointPublisher(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    // define a vector of desired joint displacements...w/o linkage redundancies
    Eigen::VectorXd q_vec;
    q_vec<<0.1,0.2,0.15,0.4,0.5,0.6,0.7;
    
    //use the publisher object to map these correctly and send them to rviz

    
    for (int i=0;i<10;i++) {
     davinciJointPublisher.pubJointStates(q_vec); //publish a bunch of times, to make sure
        ros::spinOnce();   
        ros::Duration(0.1).sleep(); 
    }

    return 0;
} 
*/
