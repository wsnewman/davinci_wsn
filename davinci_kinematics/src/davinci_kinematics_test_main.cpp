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
    tf::TransformListener tfListener;
    tf::StampedTransform baseToHand;
    
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
    

    //Davinci_IK_solver ik_solver;
        Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver

    std::cout << "==== Test for Baxter kinematics solver ====" << std::endl;
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

    }
    return 0;
}
