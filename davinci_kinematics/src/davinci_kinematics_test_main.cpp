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
    //ROS_INFO("DH_a_params[1] = %f",DH_a_params[1]);
    //manual check: enter RPY values and q-angles to try to match gazebo to fwd_kin
    double euler_R = 01.2192;
    double euler_P = 0.9412;
    double euler_Y = 0.4226;
    Vectorq7x1 q_in;
    q_in << 0,0,0,0,0,0,0;
    //q_in << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    //tf::TransformListener tfListener;
    tf::StampedTransform baseToHand;
    tf::StampedTransform tf_wrist_wrt_base;
    tf::TransformListener tfListener;
    
    //Davinci_IK_solver ik_solver;
    // bring in a fwd solver object:
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    //g_tfListener_ptr = &tfListener;
    // wait to start receiving valid tf transforms 
    Eigen::Affine3d affine_wrist_wrt_base;
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
    /*
    tf::Vector3 tf_Origin = tf_wrist_wrt_base.getOrigin();
    //ROS_INFO("")
    tf::Matrix3x3 tf_R = tf_wrist_wrt_base.getBasis();
    
    tf::Transform tf_temp;
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
   affine_wrist_wrt_base=davinci_fwd_solver.transformTFToEigen(tf_temp);
   */
   affine_wrist_wrt_base =  davinci_fwd_solver.stampedTFToAffine3d(tf_wrist_wrt_base);

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
   // create a static transform to describe DH frame-0 w/rt our base frame:
   Eigen::Matrix3f R_0_wrt_base;
   Eigen::Vector3f Origin_0_wrt_base;
   Origin_0_wrt_base<<0,0,0;
   Eigen::Vector3f x_axis,y_axis,z_axis;
   z_axis<<0,-1,0; // points IN, so + rotation is consistent leaning to the robot's left
   x_axis<<0,0,-1;  // choose x0 to point down, so will not have a joint-angle offset for pitch
   y_axis<<1,0,0; // consistent triad
   R_0_wrt_base.col(0) = x_axis;
   R_0_wrt_base.col(1) = y_axis;
   R_0_wrt_base.col(2) = z_axis;   
   Eigen::Affine3f affine_frame0_wrt_base;
   affine_frame0_wrt_base.linear() = R_0_wrt_base;
   affine_frame0_wrt_base.translation() = Origin_0_wrt_base;   
   */
   
   
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
   /*
   bool tferr=true;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener.lookupTransform("right_hand", "torso", ros::Time(0), baseToHand);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    //right_upper_shoulder is located at bottom of first shoulder jnt
    // from now on, tfListener will keep track of transforms    
    tfListener.lookupTransform("torso", "right_arm_mount", ros::Time(0), baseToHand);
    tf::Vector3 pos = baseToHand.getOrigin();
    ROS_INFO("right_arm_mount w/rt torso:  x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);  
    //baseToHand.getRotation()
    tf::Quaternion quat = baseToHand.getRotation();
  
    Eigen::Quaterniond e_quat;
    e_quat.x() = quat.x();
    e_quat.y() = quat.y();
    e_quat.z() = quat.z();
    e_quat.w() = quat.w();   
    Eigen::Matrix3d Rmount =  e_quat.toRotationMatrix();
    cout<<"Rmount = "<<endl;
    cout<<Rmount<<endl;
    

    Eigen::Matrix4d A_flange;
    //Eigen::Affine3d Affine_flange = baxter_fwd_solver.fwd_kin_solve_wrt_torso(const Vectorq7x1& q_vec);
            
    tfListener.lookupTransform("right_upper_shoulder", "right_lower_shoulder", ros::Time(0), baseToHand);
    pos = baseToHand.getOrigin();
    ROS_INFO("shoulder x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);

    tfListener.lookupTransform("right_upper_shoulder","right_lower_elbow", ros::Time(0), baseToHand);
    pos = baseToHand.getOrigin();
    ROS_INFO("elbow x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);
    tfListener.lookupTransform("right_upper_shoulder","right_lower_forearm",  ros::Time(0), baseToHand);
    pos = baseToHand.getOrigin();
    ROS_INFO("wrist x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);     
    tfListener.lookupTransform("right_upper_shoulder","right_hand",  ros::Time(0), baseToHand);
    pos = baseToHand.getOrigin();
    ROS_INFO("hand x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);   
    
*/


    std::cout << "==== Test for Davinci kinematics solver ====" << std::endl;
    int ans = 1;
    bool reachable_proposition;
    //while (ans) 
    {
        //pick legal, random values of q:
        double rval;
        /*
        for (int i = 0; i < 7; i++) {
            rval = ((double) rand()) / RAND_MAX;
            //ROS_INFO("randval: %f",rval);
            double qval = -M_PI + 2.0 * M_PI*rval;
            q_in[i] = qval; // assign random values to joint coords; fix range later
        }

        reachable_proposition = ik_solver.fit_joints_to_range(q_in);
        */
        //if (reachable_proposition) 
        /*
        {

            Eigen::Affine3d A_fwd_DH = davinci_fwd_solver.fwd_kin_flange_wrt_r_arm_mount_solve(q_in); //fwd_kin_solve
            // rotate DH frame6 to reconcile with URDF frame7:
            //Eigen::Affine3d A_fwd_URDF = A_fwd_DH*a_tool;
            //std::cout << "q_in: " << q_in.transpose() << std::endl;
            //std::cout << "A rot: " << std::endl;
            //std::cout << A_fwd_URDF.linear() << std::endl;
            //std::cout << "A origin: " << A_fwd_URDF.translation().transpose() << std::endl;
            //Eigen::Matrix3d R_flange = A_fwd_URDF.linear();
            Eigen::Matrix4d A_wrist,A_elbow,A_shoulder;



            Eigen::Matrix3d R_hand;
            //Eigen::Matrix3d R_Y = Eigen::AngleAxisd(euler_Y, Eigen::Vector3d::UnitZ());
            //R_hand = Eigen::AngleAxisd(euler_Y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler_P, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_R, Eigen::Vector3d::UnitX());

            //std::cout<<"R from RPY vals: "<<std::endl;
            //std::cout<<R_hand<<std::endl;
            A_shoulder = davinci_fwd_solver.get_shoulder_frame();
            std::cout << "fwd kin shoulder point: " << A_shoulder(0, 3) << ", " << A_shoulder(1, 3) << ", " << A_shoulder(2, 3) << std::endl;

            A_elbow = davinci_fwd_solver.get_elbow_frame();
            std::cout << "fwd kin elbow point: " << A_elbow(0, 3) << ", " << A_elbow(1, 3) << ", " << A_elbow(2, 3) << std::endl;
 
            A_wrist = davinci_fwd_solver.get_wrist_frame();
            std::cout << "fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
            
            A_flange = davinci_fwd_solver.get_flange_frame();
            std::cout << "fwd kin flange origin: " << A_flange(0, 3) << ", " << A_flange(1, 3) << ", " << A_flange(2, 3) << std::endl;
           */
                    
     /*
            int nsolns = ik_solver.ik_solve(A_fwd_DH);
            std::cout << "number of IK solutions: " << nsolns << std::endl;

            std::vector<Vectorq6x1> q6dof_solns;
            ik_solver.get_solns(q6dof_solns);
            nsolns = q6dof_solns.size();
            double q_err;
            int i_min = -1;
            std::cout << "found " << nsolns << " solutions:" << std::endl;
            for (int i = 0; i < nsolns; i++) {
                Vectorq6x1 q_soln = q6dof_solns[i];
                ik_solver.fit_joints_to_range(q_soln);
                std::cout << q_soln.transpose() << std::endl;
                q6dof_solns[i] = q_soln;
                q_err =(q_in-q_soln).norm();//fabs(q_in[0] - q_soln[0]) + fabs(q_in[1] - q_soln[1]) + fabs(q_in[2] - q_soln[2]);
                if (q_err < 0.000001) {
                    //std::cout<<"precise fit for soln "<<i<<std::endl;
                    i_min = i;
                }

                //std::cout<< "q_err: "<<q_err<<std::endl;
            }
            std::cout << "precise fit for soln " << i_min << std::endl <<std::endl;
            std::cout << "des fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
            std::cout << "fwd kin wrist points from these solutions:" << std::endl;
            for (int i = 0; i < nsolns; i++) {
                A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(q6dof_solns[i]);
                A_wrist = irb120_fwd_solver.get_wrist_frame();
                std::cout << "fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
            }
      * */
            //std::cout << "enter 1 to continue, 0 to stop: ";
            //std::cin >> ans;


    }
    return 0;
}
