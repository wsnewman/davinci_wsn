// davinci_kinematics_test_main.cpp
// wsn, Sept 2015
// test function for davinci_kinematics library
// use as:  
//FK:     affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);
//IK:       ik_solver.ik_solve(affine_gripper_wrt_base);
//            q_vec_ik = ik_solver.get_soln();

// IK has solns off a bit...due to round-off??

#include <davinci_kinematics/davinci_kinematics.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry> 
#include <sensor_msgs/JointState.h>

using namespace std;

Vectorq7x1 g_q_vec;  //global, filled by callback
bool g_got_callback=false;

void jsCallback(const sensor_msgs::JointState& jointState) 
{ 
        g_q_vec(0) = jointState.position[0];
        g_q_vec(1) = jointState.position[1];
        g_q_vec(2) = jointState.position[7];
        g_q_vec(3) = jointState.position[8];          //rotation about tool shaft:     
        g_q_vec(4) = jointState.position[9];        // wrist bend:
        g_q_vec(5) = jointState.position[10];       // q_vec[5] rotates both jaws together
        g_q_vec(6) = 0.0;  // decide what to do with this--2nd jaw
    g_got_callback = true;
} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "davinci_kinematics_test_main");
    
  ros::NodeHandle n; // need this to establish communications with our new node 
  //create a Subscriber object and have it subscribe to the topic "topic1" 
  // the function "myCallback" will wake up whenever a new message is published to topic1 
  // the real work is done inside the callback function 
  
  ros::Subscriber joint_state_subscriber= n.subscribe("dvrk_psm/joint_states",1,jsCallback);   
  
  // this is what we are testing: FK and IK solvers:
  Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
  Davinci_IK_solver ik_solver;
  
   Eigen::Vector3d w_wrt_base,q123;
   
  ROS_INFO("waiting to receive joint states");
  while (!g_got_callback)   {
      ros::spinOnce();
  }
 
    Vectorq7x1 q_vec, err_vec, q_vec_ik;
    q_vec << 0,0,0,0,0,0,0;
    //NEED THESE TO AGREE W/ JOINT_STATE_PUBLISHER
    /*
        q_vec[0] = 0.0;
        q_vec[1]= 0.0;
        q_vec[2] = insertion_offset+0.0;
        q_vec[3] = 0.0;
        q_vec[4] = 0.0; // wrist bend
        q_vec[5] = 0.0; //1.57; // gripper-jaw rotation
        q_vec[6] = 0.0;
    */    
    // use published values:
        q_vec = g_q_vec;
        cout<<"using q_vec = "<<q_vec.transpose()<<endl;
    //tf::StampedTransform baseToHand;
    tf::StampedTransform tf_wrist_wrt_base, tf_gripper_tip_wrt_base, tf_frame_wrt_base;
    tf::TransformListener tfListener;
    
    Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base, affine_frame_wrt_base;
        // wait to start receiving valid tf transforms 
    /*
    bool tferr = true;
    ROS_INFO("waiting for tf between one_psm_base_link and one_tool_tip_link...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("one_psm_base_link", "one_tool_tip_link", ros::Time(0), tf_gripper_tip_wrt_base);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    
    ROS_INFO("gripper tip frame  one_tool_tip_link per tf:");
   // now get same from tf:
   affine_gripper_wrt_base =  davinci_fwd_solver.stampedTFToAffine3d(tf_gripper_tip_wrt_base);

   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_gripper_wrt_base.translation().transpose()<<endl; 
*/
   Eigen::Vector3d tip_from_FK, tip_from_FK_of_IK, tip_err;
   ROS_INFO("gripper tip frame from FK: ");   
   affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_base.linear()<<endl;
   tip_from_FK = affine_gripper_wrt_base.translation();
   cout<<"origin: "<<tip_from_FK.transpose()<<endl; 
   cout<<endl;
   cout<<endl;
   
   ik_solver.ik_solve(affine_gripper_wrt_base);
   q_vec_ik = ik_solver.get_soln();
   
   ROS_INFO("FK of IK soln: ");
   affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec_ik);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_base.linear()<<endl;
   tip_from_FK_of_IK = affine_gripper_wrt_base.translation();
   cout<<"origin: ";
   cout<<tip_from_FK_of_IK.transpose()<<endl;   
   
   cout<<endl;
   cout<<"q_vec in: "<<q_vec.transpose()<<endl;
   cout<<"q_vec_ik: "<<q_vec_ik.transpose()<<endl;
   err_vec = q_vec - q_vec_ik;
   cout<<"err vec: "<<err_vec.transpose()<<endl;
   cout<<endl;
   cout<<endl;
   tip_err = tip_from_FK-tip_from_FK_of_IK;
   cout<<"tip err: "<<tip_err.transpose()<<endl;
   return 0;
   
   
   
   // test IK function:  find wrist point from gripper-tip frame:
   Eigen::Vector3d z_vec4;
   w_wrt_base = ik_solver.compute_w_from_tip(affine_gripper_wrt_base,z_vec4);
   cout<<"origin4 from IK: "<<w_wrt_base.transpose()<<endl;
   cout<<"z_vec4 from IK: "<<z_vec4.transpose()<<endl;
   q123 = ik_solver.q123_from_wrist(w_wrt_base);
   cout<<"q123: "<<q123.transpose()<<endl;
   // LOOKS LIKE THIS WORKS--RECONFIRM w/ FK:
   Eigen::VectorXd theta_vec,d_vec;
   theta_vec.resize(7); //<<q123(0),q123(1),0,0,0,0,0;
   theta_vec<<0,0,0,0,0,0,0;
   theta_vec(0) = q123(0);
   theta_vec(1) = q123(1);
   cout<<"theta_vec: "<<theta_vec.transpose()<<endl;
   d_vec.resize(7);
   d_vec<<0,0,0,0,0,0,0;
   d_vec(2) = q123(2);
   cout<<"d_vec: "<<d_vec.transpose()<<endl;
   
   //use partial IK soln to compute FK of first three frames:
   ROS_INFO("calling fwd_kin_solve_DH()");
   davinci_fwd_solver.fwd_kin_solve_DH(theta_vec, d_vec);
    ROS_INFO("wrist frame (frame 3) from IK/FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(2); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;  
   ROS_INFO("x-axis from above is reference for theta4");
   //double sval = 
   
   // now compare to home-brew FK:
 
   ROS_INFO("gripper tip frame from FK: ");   
   affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);

   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_gripper_wrt_base.translation().transpose()<<endl;

 
   tfListener.lookupTransform("one_psm_base_link", "one_tool_wrist_sca_link", ros::Time(0), tf_wrist_wrt_base);
   cout<<endl;
    ROS_INFO("wrist bend frame one_tool_wrist_sca_link per tf:");
   // now get same from tf:
   affine_wrist_wrt_base =  davinci_fwd_solver.stampedTFToAffine3d(tf_wrist_wrt_base);

   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;    
   cout<<endl;
   cout<<"WRIST BEND AXIS: "<<affine_wrist_wrt_base.linear().col(2).transpose()<<endl;
   
   /*
   tfListener.lookupTransform("one_psm_base_link", "one_tool_main_link", ros::Time(0), tf_frame_wrt_base);
   cout<<endl;
    ROS_INFO("one_tool_main_link frame per tf:");
   // now get same from tf:
   affine_frame_wrt_base =  davinci_fwd_solver.stampedTFToAffine3d(tf_frame_wrt_base);

   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;    
   cout<<endl;  

   ROS_INFO("wrist frame (frame 3) from FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(2); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;   
 
  tfListener.lookupTransform("one_psm_base_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_frame_wrt_base);
   cout<<endl;
    ROS_INFO("gripper rot frame one_tool_wrist_sca_shaft_link per tf:");
   // now get same from tf:
   affine_frame_wrt_base =  davinci_fwd_solver.stampedTFToAffine3d(tf_frame_wrt_base);

   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;    
   cout<<endl;

   ROS_INFO("gripper frame (frame 4) from FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(3); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;     
   

   w_wrt_base = affine_wrist_wrt_base.translation();
   q123 = ik_solver.q123_from_wrist(w_wrt_base);
   cout<<"q123: "<<q123.transpose()<<endl;
  */
 /*
     ROS_INFO("frame 1 per wsn FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(0); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;
   
     ROS_INFO("frame 2 per wsn FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(1); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;
   */

   /*
   ROS_INFO("frame 4 per wsn FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(3); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame 5 per wsn FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(4); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame 6 per wsn FK: ");
   affine_frame_wrt_base = davinci_fwd_solver.get_affine_frame(5); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_frame_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_frame_wrt_base.translation().transpose()<<endl;
 
   ROS_INFO("gripper tip frame per wsn FK: ");
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_gripper_wrt_base.translation().transpose()<<endl;
*/

 
    return 0;
}
