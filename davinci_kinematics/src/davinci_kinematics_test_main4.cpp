// davinci_kinematics_test_main4.cpp
// send out random legal joint values; test resulting single solution

#include <davinci_kinematics/davinci_kinematics.h>
#include <tf/transform_listener.h>
//#include <tf/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry> 
#include <sensor_msgs/JointState.h>

using namespace std;

Vectorq7x1 g_q_vec; //global, filled by callback
bool g_got_callback = false;

void jsCallback(const sensor_msgs::JointState& jointState) {
    g_q_vec(0) = jointState.position[0];
    g_q_vec(1) = jointState.position[1];
    g_q_vec(2) = jointState.position[7];
    g_q_vec(3) = jointState.position[8]; //rotation about tool shaft:     
    g_q_vec(4) = jointState.position[9]; // wrist bend:
    g_q_vec(5) = jointState.position[10]; // q_vec[5] rotates both jaws together
    g_q_vec(6) = 0.0; // decide what to do with this--2nd jaw
    g_got_callback = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "davinci_kinematics_test_main");

    ros::NodeHandle n; // need this to establish communications with our new node  

    // this is what we are testing: FK and IK solvers:
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver;

    Eigen::Vector3d w_wrt_base, q123;

    Vectorq7x1 q_vec, err_vec, q_vec_ik;
    q_vec << 0, 0, 0, 0, 0, 0, 0;
    int err_cnt = 0;

    Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base, affine_frame_wrt_base;
    // wait to start receiving valid tf transforms 


    Eigen::Vector3d tip_from_FK, tip_from_FK_of_IK, tip_err;
    //note: with print-outs, takes about 45sec for 10,000 iterations, and got 0 errors
    for (int itries = 0; itries < 10000; itries++) {
        davinci_fwd_solver.gen_rand_legal_jnt_vals(q_vec);

        cout << "using q_vec = " << q_vec.transpose() << endl;
        ROS_INFO("gripper tip frame from FK: ");
        affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);
        cout << "affine linear (R): " << endl;
        cout << affine_gripper_wrt_base.linear() << endl;
        tip_from_FK = affine_gripper_wrt_base.translation();
        cout << "origin: " << tip_from_FK.transpose() << endl;
        cout << endl;
        cout << endl;

        if (ik_solver.ik_solve(affine_gripper_wrt_base) > 0) { //any legal solns?
            q_vec_ik = ik_solver.get_soln();

            ROS_INFO("FK of IK soln: ");
            affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec_ik);
            cout << "affine linear (R): " << endl;
            cout << affine_gripper_wrt_base.linear() << endl;
            tip_from_FK_of_IK = affine_gripper_wrt_base.translation();
            cout << "origin: ";
            cout << tip_from_FK_of_IK.transpose() << endl;

            cout << endl;
            cout << "q_vec in: " << q_vec.transpose() << endl;
            cout << "q_vec_ik: " << q_vec_ik.transpose() << endl;
            err_vec = q_vec - q_vec_ik;

            cout << "err vec: " << err_vec.transpose() << endl;
            cout << endl;
            cout << endl;
            tip_err = tip_from_FK - tip_from_FK_of_IK;
            cout << "tip err: " << tip_err.transpose() << endl;
            ROS_WARN("jspace errvec norm: %f", err_vec.norm());
            ROS_WARN("tip pos err norm: %f:", tip_err.norm());
            if (err_vec.norm() + tip_err.norm() > 0.0001) {
                ROS_ERROR("excessive error!");
                err_cnt++;
            }
            ROS_INFO("itries = %d; err_cnt = %d", itries,err_cnt);
        }
    }
    ROS_INFO("err_cnt = %d", err_cnt);
    return 0;
}
