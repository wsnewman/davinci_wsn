// cart_move_client_svc: 
// another variation...
// in this case, receive pose messages and communicate them to the cartesian-move action server



#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<cwru_action/cart_moveAction.h>
#include<cwru_davinci_srv/arm_poses.h>


using namespace std;

//utility to convert a pose to an Eigen::Affine
Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    
    Eigen::Affine3d e;
    Eigen::Vector3d Oe;
    Oe(0)=pose.position.x;
    Oe(1)=pose.position.y;
    Oe(2)=pose.position.z;

    Eigen::Quaterniond q;
    q.x()=pose.orientation.x;
    q.y()=pose.orientation.y;
    q.z()=pose.orientation.z;
    q.w()=pose.orientation.w;  
    Eigen::Matrix3d Re(q);
    e.linear()=Re;
    e.translation() = Oe;
    return e;
}

//utility fnc to convert an Eigen::Affine3d object into an equivalent geometry_msgs::Pose object
geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const cwru_action::cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    //ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}

//gripper 1 is robot's "right" arm; gripper 2 is left
cwru_action::cart_moveGoal pack_goal(Eigen::Affine3d affine_des_gripper1,
				     Eigen::Affine3d affine_des_gripper2,
				     double jaw1_angle,double jaw2_angle,double move_time) {
        cwru_action::cart_moveGoal goal;

	//pack the goal object:
   	goal.gripper_jaw_angle1 = jaw1_angle;
  	goal.gripper_jaw_angle2 = jaw2_angle;
	goal.move_time = move_time;

	//nuisance...must re-expressed desired poses from Affine to PoseStamped to populate action goal
   	geometry_msgs::PoseStamped des_pose_gripper1;
          des_pose_gripper1.pose = transformEigenAffine3dToPose(affine_des_gripper1);
	  des_pose_gripper1.header.stamp = ros::Time::now();
	geometry_msgs::PoseStamped des_pose_gripper2;
	  des_pose_gripper2.pose = transformEigenAffine3dToPose(affine_des_gripper2);
	  des_pose_gripper2.header.stamp = ros::Time::now();

	goal.des_pose_gripper1 = des_pose_gripper1;
	goal.des_pose_gripper2 = des_pose_gripper2;
 
	return goal;
}

geometry_msgs::PoseStamped g_arm1_des_pose,g_arm2_des_pose;
double g_gripper1_des_ang,g_gripper2_des_ang,g_move_time;
Eigen::Affine3d g_affine_des_gripper1,g_affine_des_gripper2;

bool svcCallback(cwru_davinci_srv::arm_posesRequest& request, cwru_davinci_srv::arm_posesResponse& response)
{
    ROS_INFO("service callback activated");
    g_arm1_des_pose = request.arm1_des_pose;
    g_arm2_des_pose = request.arm2_des_pose;    
    g_gripper1_des_ang = request.gripper1_des_ang;
    g_gripper2_des_ang = request.gripper2_des_ang;    
    g_move_time = request.move_time;
    
    g_affine_des_gripper1 = transformPoseToEigenAffine3d(g_arm1_des_pose.pose);
    g_affine_des_gripper2 = transformPoseToEigenAffine3d(g_arm2_des_pose.pose);    
  return true;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "cart_move_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        
	Eigen::Affine3d affine_des_gripper1,affine_des_gripper2;
        double gripper1_jaw_angle,gripper2_jaw_angle,move_time;
	Eigen::Vector3d tip_origin1,tip_origin2;
	Eigen::Matrix3d R_gripper1,R_gripper2;
	Eigen::Vector3d z_vec1,z_vec2,x_vec1,x_vec2,y_vec1,y_vec2;
        
        cwru_davinci_srv::arm_poses arm_poses_srv;
        ros::ServiceServer service = nh.advertiseService("cart_move_svc", svcCallback);
        ROS_INFO("cart_move_svc is available");

        // hard-code desired gripper poses
        // gripper 1 is DaVinci's right arm (from robot's viewpoint)
        tip_origin1<<0,0,0.14;
	z_vec1<<1,0,0;
	x_vec1<<0,0,-1;
	y_vec1 = z_vec1.cross(x_vec1);
 	R_gripper1.col(0)=x_vec1;
 	R_gripper1.col(1)=y_vec1;        
 	R_gripper1.col(2)=z_vec1;
	affine_des_gripper1.linear()=R_gripper1;
	affine_des_gripper1.translation()=tip_origin1;
	gripper1_jaw_angle=0.3;

        // and specify the "left" gripper pose:
	tip_origin2<<0,0,0.12;
	z_vec2<<-1,0,0;
	x_vec2<<0,0,-1;
	y_vec2 = z_vec2.cross(x_vec2);
 	R_gripper2.col(0)=x_vec2;
 	R_gripper2.col(1)=y_vec2;        
 	R_gripper2.col(2)=z_vec2;
	affine_des_gripper2.linear()=R_gripper2;
	affine_des_gripper2.translation()=tip_origin2; 
	gripper2_jaw_angle=0.3;
	move_time=2.0;
          
        // here is a "goal" object compatible with the server, as defined in cwru_action_server/cart_move
        cwru_action::cart_moveGoal goal; 

	goal = pack_goal(affine_des_gripper1,affine_des_gripper2,gripper1_jaw_angle,gripper2_jaw_angle,move_time);

      /*
	//pack the goal object:
   	goal.gripper_jaw_angle1 = gripper1_jaw_angle;
  	goal.gripper_jaw_angle2 = gripper2_jaw_angle;
	goal.move_time = move_time;

	//nuisance...must re-expressed desired poses from Affine to PoseStamped to populate action goal
   	geometry_msgs::PoseStamped des_pose_gripper1;
          des_pose_gripper1.pose = transformEigenAffine3dToPose(affine_des_gripper1);
	  des_pose_gripper1.header.stamp = ros::Time::now();
	geometry_msgs::PoseStamped des_pose_gripper2;
	  des_pose_gripper2.pose = transformEigenAffine3dToPose(affine_des_gripper2);
	  des_pose_gripper2.header.stamp = ros::Time::now();

	goal.des_pose_gripper1 = des_pose_gripper1;
	goal.des_pose_gripper2 = des_pose_gripper2;
       */

        // use the name of our server, which is: cartMoveActionServer (named in cart_move_as.cpp)
        actionlib::SimpleActionClient<cwru_action::cart_moveAction> action_client("cartMoveActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = false;
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
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        // stuff a goal message:
        //goal.traj_id = g_count; // this merely sequentially numbers the goals sent

        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(move_time+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }

    return 0;
}

