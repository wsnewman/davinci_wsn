// davinci_kinematics_test_main.cpp
// wsn, Sept 2015
// test function for davinci_kinematics library


#include <davinci_kinematics/davinci_kinematics.h>
 #include <tf/transform_listener.h>
//#include <tf/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry> 
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "davinci_kinematics_test_main");
 
    Vectorq7x1 q_vec;
    q_vec << 0,0,0,0,0,0,0;
    //NEED THESE TO AGREE W/ JOINT_STATE_PUBLISHER
        q_vec[0] = 0.0;
        q_vec[1]= 0.0;
        q_vec[2] = insertion_offset+0.0;
        q_vec[3] = 0.0;
        q_vec[4] = 0.0; // wrist bend
        q_vec[5] = 0.0; //1.57; // gripper-jaw rotation
        q_vec[6] = 0.0;
 
    //tf::StampedTransform baseToHand;
    tf::StampedTransform tf_wrist_wrt_base;
    tf::TransformListener tfListener;
    
    //Davinci_IK_solver ik_solver;
    // bring in a fwd solver object:
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    //tfListener_ptr = &tfListener;
    // wait to start receiving valid tf transforms 
    Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base;
    bool tferr = true;
    ROS_INFO("waiting for tf between one_psm_base_link and one_tool_wrist_sca_link...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("one_psm_base_link", "one_tool_wrist_sca_link", ros::Time(0), tf_wrist_wrt_base);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    
    ROS_INFO("frame 4 (wrist bend) per tf:");
   // now get same from tf:
   affine_wrist_wrt_base =  davinci_fwd_solver.stampedTFToAffine3d(tf_wrist_wrt_base);

   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;    

   // now compare to home-brew FK:
 
   //ROS_INFO("gripper frame from FK: ");   
   affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);
  /*
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_gripper_wrt_base.translation().transpose()<<endl;
   */
     ROS_INFO("frame 1 per wsn FK: ");
   affine_wrist_wrt_base = davinci_fwd_solver.get_affine_frame(0); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
     ROS_INFO("frame 2 per wsn FK: ");
   affine_wrist_wrt_base = davinci_fwd_solver.get_affine_frame(1); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame 3 per wsn FK: ");
   affine_wrist_wrt_base = davinci_fwd_solver.get_affine_frame(2); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame 4 per wsn FK: ");
   affine_wrist_wrt_base = davinci_fwd_solver.get_affine_frame(3); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame 5 per wsn FK: ");
   affine_wrist_wrt_base = davinci_fwd_solver.get_affine_frame(4); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
   ROS_INFO("frame 6 per wsn FK: ");
   affine_wrist_wrt_base = davinci_fwd_solver.get_affine_frame(5); // get frame 4 w/rt base
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;
   
   // this much is consistent...
   // at q[0]=0, q[1]=0, q[2] = insertion_offset, --> origin of frame one_tool_wrist_sca_link = 0,0,0 w/rt
   // one_psm_base_link;
   // also, axes of frame one_tool_wrist_sca_link are correctly described in R matrix
   
   // at q[0] = 1.57, tool leans to "left";
   // no change in origin of frame one_tool_wrist_sca_link
   // and distal frame now has x-axis aligned w/ ref frame, and z axes are antiparallel and y axes are antiparallel
   // i.e., R matrix still makes sense
   // ==> this interpretation of Affine is: distal frame w/rt base frame
   
   
   
   /*
   ROS_INFO("frame tool tip w/rt tool_wrist_sca_shaft_link");
   tfListener.lookupTransform("one_tool_wrist_sca_shaft_link", "one_tool_tip_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base=davinci_fwd_solver.transformTFToEigen(tf_temp);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   
   ROS_INFO("frame one_tool_wrist_sca_shaft_link w/rt one_tool_wrist_link");
   tfListener.lookupTransform("one_tool_wrist_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base= davinci_fwd_solver.transformTFToEigen(tf_temp);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;     
   
   ROS_INFO("frame one_psm_base_link to one_tool_tip_link");
   tfListener.lookupTransform("one_psm_base_link", "one_tool_tip_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base= davinci_fwd_solver.transformTFToEigen(tf_temp);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   
   double gripper_jaw_length = 0.0102;
   Eigen::Vector3f gripper_tip_origin;
   gripper_tip_origin = affine_wrist_wrt_base.translation();
   Eigen::Vector3f z_vec_gripper_tip;
   Eigen::Matrix3f  R_gripper_tip;
   R_gripper_tip = affine_wrist_wrt_base.linear();
   z_vec_gripper_tip = R_gripper_tip.col(2);
   Eigen::Vector3f computed_jaw_axis_origin;
   computed_jaw_axis_origin = gripper_tip_origin - z_vec_gripper_tip*gripper_jaw_length;
   cout<<"computed jaw axis origin: "<<computed_jaw_axis_origin.transpose()<<endl;
   
   //find origin of one_tool_wrist_link relative to one_psm_base_link
  ROS_INFO("frame 6 w/rt frame 0");
   tfListener.lookupTransform("one_psm_base_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base = davinci_fwd_solver.transformTFToEigen(tf_temp);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   Eigen::Matrix3f R_6_wrt_0;
   Eigen::Vector3f O_6_wrt_0;   
   R_6_wrt_0 = affine_wrist_wrt_base.linear();
   O_6_wrt_0 = affine_wrist_wrt_base.translation();
   
   cout<<"computed jaw axis origin: "<<computed_jaw_axis_origin.transpose()<<endl;
   
   //find origin of one_tool_wrist_link relative to one_psm_base_link
  ROS_INFO("frame one_tool_wrist_shaft_link to one_tool_wrist_sca_shaft_link");
   tfListener.lookupTransform("one_tool_wrist_shaft_link", "one_tool_wrist_sca_shaft_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base= davinci_fwd_solver.transformTFToEigen(tf_temp);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;  
   
   double a_from_wrist_bend_to_jaw_axis =  0.0091;
   
   //   R(q6) =   [sin(q6)  cos(q6)  0
   //             0        0      1
   //           cos(q6) -sin(q6)  0]
//
   // == R_{6/5}
   Eigen::Matrix3f R_6_wrt_5, R_5_wrt_0;
   Eigen::Vector3f O_5_wrt_0;
   // GIVEN O_6_wrt_0 and R_6_wrt_0 as specification of desired gripper pose, w/rt ref frame 0 = one_psm_base_link
   // w/ origin at pivot (trocar) point
   
   // TEST FWD KIN CONSISTENCY; SET MODEL q6 as follows
   double q6 = 1.0; // put q6 value here, or subscribe to it; eventually, solve for it
   Eigen::Vector3f x_vec, y_vec, z_vec;
   double a6 = 0.0091; //distance from jaw-bend axis to wrist-bend axis
   x_vec<<sin(q6),0,cos(q6);
   y_vec<<cos(q6),0,-sin(q6);
   z_vec<<0,1,0;
   R_6_wrt_5.col(0) = x_vec;
   R_6_wrt_5.col(1) = y_vec;   
   R_6_wrt_5.col(2) = z_vec;  
   R_5_wrt_0 = R_6_wrt_5.transpose()*R_6_wrt_0;
   Eigen::Vector3f xvec_5;
   xvec_5 = R_5_wrt_0.col(0);
   O_5_wrt_0 = O_6_wrt_0 - a6*xvec_5; //get to O5 by moving dist a6 backwards along x5 from O6
   
   cout<<"computed O_5_wrt_0: "<<O_5_wrt_0.transpose()<<endl;
   cout<<"computed R_5_wrt_0: "<<endl;
   cout<<R_5_wrt_0<<endl;
   
   // compare this to tf:
   //find origin of one_tool_wrist_link relative to one_psm_base_link
   ROS_INFO("frame 5 w/rt frame 0, per tf: ");
   tfListener.lookupTransform("one_psm_base_link", "one_tool_wrist_sca_link", ros::Time(0), tf_wrist_wrt_base);
   tf_Origin = tf_wrist_wrt_base.getOrigin();
   tf_R = tf_wrist_wrt_base.getBasis();
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base= davinci_fwd_solver.transformTFToEigen(tf_temp);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_wrist_wrt_base.linear()<<endl;
   cout<<"origin: ";
   cout<<affine_wrist_wrt_base.translation().transpose()<<endl;    
 */
 
    return 0;
}
