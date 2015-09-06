//davinci_cart_traj_pub.cpp:
//wsn, Sept 2015
// example code to compute Cartesian trajectories, perform IK, and publish joint angles
// this example keeps gripper jaws pointing forward, parallel to ground,
// and has gripper tip sweep out a circle in the x-z (vertical) plane
// have not dealt yet with gripper open/close control

#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "davinci_joint_state_publisher"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    DavinciJointPublisher davinciJointPublisher(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
 
    ROS_INFO("instantiating  forward solver and an ik_solver");
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver;
    
    
    // define a vector of desired joint displacements...w/o linkage redundancies
    //7'th angle is related to jaw opening--but not well handled yet
    Vectorq7x1 q_vec,q_vec2;
    
    //generate a Cartesian trajectory
    Eigen::Matrix3d R_des;
    Eigen::Vector3d x_vec,y_vec,z_vec, O_tip,O_tip2;
    Eigen::Affine3d affine_gripper_des,affine_gripper_des2;
    //specify the desired orientation as an R matrix:
    x_vec<<1,0,0;
    y_vec<<0,0,-1;
    z_vec<<0,1,0;
    R_des.col(0) = x_vec;
    R_des.col(1) = y_vec;
    R_des.col(2) = z_vec;
    affine_gripper_des.linear() = R_des;
    affine_gripper_des2.linear() = R_des;    
    // the tip coordinates will change with time; sweep out a circular path
    double r_tip, theta_tip, z_offset_tip, omega_tip;
    r_tip = 0.04;
    z_offset_tip = 0.15; //insertion should always be >0 to get gripper through portal
    omega_tip = 1.0;
    theta_tip = 0;
    double x_tip, y_tip, z_tip, x_tip2, y_tip2, z_tip2;
    y_tip = 0.05;
    double dt = 0.01;
    ROS_INFO("starting loop");
    while(ros::ok()) {
        //generate a circular path in space for the gripper tip:
        theta_tip += dt*omega_tip; //
        if (theta_tip>2*M_PI) theta_tip-= 2*M_PI;
        z_tip = r_tip*cos(theta_tip)-z_offset_tip;
        x_tip = -0.13;    
        y_tip = r_tip*sin(theta_tip);
        O_tip(0) = x_tip;
        O_tip(1) = y_tip;
        O_tip(2) = z_tip; //<<x_tip,y_tip,z_tip;
        affine_gripper_des.translation() = O_tip;

        z_tip2 = z_tip; //r_tip*cos(theta_tip)-z_offset_tip;
        x_tip2 = -x_tip;
        
        y_tip2 = y_tip; //r_tip*sin(theta_tip);
        O_tip2(0) = x_tip2;
        O_tip2(1) = y_tip2;
        O_tip2(2) = z_tip2; //<<x_tip,y_tip,z_tip;
        affine_gripper_des2.translation() = O_tip2;        
        
        
        //don't need to change affine.linear() if want to preserve orientation
        //ROS_INFO("doing IK solve: ");
        ik_solver.ik_solve(affine_gripper_des); //convert desired pose into equiv joint displacements
        q_vec = ik_solver.get_soln();   
        ik_solver.ik_solve(affine_gripper_des2); //convert desired pose into equiv joint displacements
        q_vec2 = ik_solver.get_soln();          
        //cout<<"q_vec: "<<q_vec.transpose()<<endl;
        
        //use the publisher object to map these correctly and send them to rviz
        davinciJointPublisher.pubJointStates(q_vec,q_vec2); 
        
        ros::spinOnce(); //really only need this if have a subscriber or service running
        ros::Duration(dt).sleep(); //loop timer

    }

    return 0;
} 

