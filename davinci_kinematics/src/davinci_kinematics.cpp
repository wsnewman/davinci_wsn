// davinci_kinematics implementation file; start w/ fwd kin

//"include" path--should just be <baxter_kinematics/baxter_kinematics.h>, at least for modules outside this package
#include <davinci_kinematics/davinci_kinematics.h>

using namespace std;


//tf::TransformListener *g_tfListener_ptr; //pointer to a global transform listener


Eigen::Affine3f  Davinci_fwd_solver::transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3f e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

Eigen::Affine3d Davinci_fwd_solver::transformTFToAffine3d(const tf::Transform &t){
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}


Eigen::Affine3d Davinci_fwd_solver::stampedTFToAffine3d(const tf::StampedTransform &t){
    tf::Vector3 tf_Origin = t.getOrigin();
    tf::Matrix3x3 tf_R = t.getBasis();
    
    tf::Transform tf_temp;
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
    Eigen::Affine3d e;
    e=transformTFToAffine3d(tf_temp);    

    return e;
}


Eigen::Affine3f  Davinci_fwd_solver::stampedTFToEigen(const tf::StampedTransform &t) {
    tf::Vector3 tf_Origin = t.getOrigin();
    tf::Matrix3x3 tf_R = t.getBasis();
    
    tf::Transform tf_temp;
    tf_temp.setBasis(tf_R);
    tf_temp.setOrigin(tf_Origin);
    Eigen::Affine3f e;
    e=transformTFToEigen(tf_temp);    

    return e;
}

 //   geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e);
 geometry_msgs::Pose Davinci_fwd_solver::transformEigenAffine3fToPose(Eigen::Affine3f e) {
    Eigen::Vector3f Oe;
    Eigen::Matrix3f Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaternionf q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

 geometry_msgs::Pose Davinci_fwd_solver::transformEigenAffine3dToPose(Eigen::Affine3d e) {
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

 //given a vector of joint states in DaVinci coords, convert these into
 // equivalen DH parameters, theta and d
void Davinci_fwd_solver::convert_qvec_to_DH_vecs(const Vectorq7x1& q_vec) {
    //    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;
    //Eigen::VectorXd theta_DH_offsets_,dval_DH_offsets_;
    thetas_DH_vec_.resize(7);

    thetas_DH_vec_ = theta_DH_offsets_; // +? -?
    for (int i=0;i<2;i++) {
        thetas_DH_vec_(i)+= q_vec(i);
    }

    for (int i=3;i<7;i++) {
        thetas_DH_vec_(i)+= q_vec(i);
    }

    dvals_DH_vec_.resize(7);

    dvals_DH_vec_ = dval_DH_offsets_; //+? -?

    dvals_DH_vec_(2)+=q_vec(2);
    ROS_INFO("q_vec(2), dvals_DH_vec_(2) = %f, %f",q_vec(2),dvals_DH_vec_(2));

}

     
Eigen::Affine3d Davinci_fwd_solver::computeAffineOfDH(double a, double d, double alpha, double theta){
     Eigen::Affine3d affine_DH;
     Eigen::Matrix3d R;
     Eigen::Vector3d p;       

    double cq = cos(theta);
    double sq = sin(theta);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    affine_DH.linear() = R;
    
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    affine_DH.translation() = p;

     return affine_DH;
}

Davinci_fwd_solver::Davinci_fwd_solver() { 
    //affine describing frame0 w/rt base frame--see comments above
   Eigen::Matrix3d R_0_wrt_base;
   Eigen::Vector3d Origin_0_wrt_base;
   Origin_0_wrt_base<<0,0,0;
   Eigen::Vector3d x_axis,y_axis,z_axis;
   // R_{0/base} = [0  1  0
   //               0  0 -1
   //              -1  0  0 ]
   z_axis<<0,-1,0; // points IN, so + rotation is consistent leaning to the robot's left
   x_axis<<0,0,-1;  // choose x0 to point down, so will not have a joint-angle offset for pitch
   y_axis<<1,0,0; // consistent triad
   R_0_wrt_base.col(0) = x_axis;
   R_0_wrt_base.col(1) = y_axis;
   R_0_wrt_base.col(2) = z_axis;   
   affine_frame0_wrt_base_; //member variable
   affine_frame0_wrt_base_.linear() = R_0_wrt_base;
   affine_frame0_wrt_base_.translation() = Origin_0_wrt_base;      

   // fill in a static tool transform from frame6 to a frame of interest on the gripper
   //Eigen::Affine3d affine_gripper_wrt_frame6_;
   Eigen::Matrix3d R_gripper_wrt_frame6;
   Eigen::Vector3d Origin_gripper_wrt_frame6;
   Origin_gripper_wrt_frame6<<0,0,jaw_length;
   z_axis<<0,0,1; // points IN, so + rotation is consistent leaning to the robot's left
   x_axis<<1,0,0;  // choose x0 to point down, so will not have a joint-angle offset for pitch
   y_axis<<0,1,0; // consistent triad
   R_gripper_wrt_frame6.col(0) = x_axis;
   R_gripper_wrt_frame6.col(1) = y_axis;
   R_gripper_wrt_frame6.col(2) = z_axis;     
   affine_gripper_wrt_frame6_.linear() = R_gripper_wrt_frame6;
   affine_gripper_wrt_frame6_.translation() = Origin_gripper_wrt_frame6;   
   
   theta_DH_offsets_.resize(7);
   for (int i=0;i<7;i++) {
       theta_DH_offsets_(i) = DH_q_offsets[i];
   }
   theta_DH_offsets_(2) = 0.0; //don't put prismatic displacement here
   
   dval_DH_offsets_.resize(7);
   dval_DH_offsets_<<0,0,DH_q_offsets[2],0,0,0,0;
   

}

// fwd-kin fnc: computes gripper frame w/rt base frame given q_vec
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_solve(const Vectorq7x1& q_vec) {   
    
    //use:
    //Eigen::Affine3d affine_frame0_wrt_base_;
    //Eigen::Affine3d affine_gripper_wrt_frame6_;    
    //Eigen::Affine3d affine_gripper_wrt_base_; 
    //vector <Eigen::Affine3d> affines_i_wrt_iminus1_;
    //vector <Eigen::Affine3d> affine_products_;    
    // and: 
    
    //Eigen::Affine3d Davinci_fwd_solver::computeAffineOfDH(double a, double d, double alpha, double theta)
    //    vector <Eigen::Affine3d> affines_i_wrt_iminus1_;
    //  vector <Eigen::Affine3d> affine_products_;
    
    //convert q_vec to DH coordinates:
    ROS_INFO("converting q to DH vals");
    convert_qvec_to_DH_vecs(q_vec);
    cout<<"theta_DH: "<<thetas_DH_vec_.transpose()<<endl;
    cout<<"dvals_DH: "<<dvals_DH_vec_.transpose()<<endl;
    
    affines_i_wrt_iminus1_.resize(7);
    ROS_INFO("computing successive frame transforms: ");
    Eigen::Affine3d xform;
    double a,d,theta,alpha;
    for (int i=0;i<7;i++) {
        a = DH_a_params[i];
        d = dvals_DH_vec_(i);
        alpha = DH_alpha_params[i];
        theta = thetas_DH_vec_(i);
        ROS_INFO("i = %d; a,d,alpha,theta = %f %f %f %f",i,a,d,alpha,theta);
        xform= computeAffineOfDH(DH_a_params[i], dvals_DH_vec_(i), DH_alpha_params[i],thetas_DH_vec_(i));
        affines_i_wrt_iminus1_[i]= xform;
    }
    
    ROS_INFO("computing transform products: ");
    affine_products_.resize(7);
    affine_products_[0] =  affine_frame0_wrt_base_*affines_i_wrt_iminus1_[0];
    for (int i=1;i<7;i++) {
        affine_products_[i] = affine_products_[i-1]*affines_i_wrt_iminus1_[i];
    }
    
    affine_gripper_wrt_base_ = affine_products_[6]*affine_gripper_wrt_frame6_;
    
    return affine_gripper_wrt_base_;
}



    // these fncs also include transform from flange to tool frame
//    Eigen::Affine3d fwd_kin_tool_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec); // given vector of q angles, compute fwd kin of tool w/rt right-arm mount 
//    Eigen::Affine3d fwd_kin_tool_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec);//version w/ spherical-wrist approx
//    Eigen::Affine3d fwd_kin_tool_wrt_torso_solve(const Vectorq7x1& q_vec); //rtns pose w/rt torso frame (base frame) 

/*
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_tool_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec) {
    Eigen::Affine3d A_flange_wrt_r_arm_mount;
    Eigen::Affine3d A_tool_wrt_r_arm_mount;
    A_flange_wrt_r_arm_mount = fwd_kin_flange_wrt_r_arm_mount_solve(q_vec);
    A_tool_wrt_r_arm_mount = A_flange_wrt_r_arm_mount*A_tool_wrt_flange_;
    return A_tool_wrt_r_arm_mount;
}
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_tool_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec) {
    Eigen::Affine3d A_flange_wrt_r_arm_mount;
    Eigen::Affine3d A_tool_wrt_r_arm_mount;
    A_flange_wrt_r_arm_mount = fwd_kin_flange_wrt_r_arm_mount_solve_approx(q_vec);
    A_tool_wrt_r_arm_mount = A_flange_wrt_r_arm_mount*A_tool_wrt_flange_;
    return A_tool_wrt_r_arm_mount;
}

Eigen::Affine3d Davinci_fwd_solver::fwd_kin_tool_wrt_torso_solve(const Vectorq7x1& q_vec) {
    Eigen::Affine3d A_flange_wrt_torso;
    Eigen::Affine3d A_tool_wrt_torso;
    A_flange_wrt_torso = fwd_kin_flange_wrt_torso_solve(q_vec);
    A_tool_wrt_torso = A_flange_wrt_torso*A_tool_wrt_flange_;
    return A_tool_wrt_torso;
}


//    Eigen::Affine3d fwd_kin_flange_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec); // given vector of q angles, compute fwd kin
//    Eigen::Affine3d fwd_kin_flange_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec);//version w/ spherical-wrist approx
//    Eigen::Affine3d fwd_kin_flange_wrt_torso_solve(const Vectorq7x1& q_vec); //rtns pose w/rt torso frame (base frame)
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_flange_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d M;
    M = fwd_kin_solve_(q_vec);
    Eigen::Affine3d A(M);
    return A;
}

Eigen::Affine3d Davinci_fwd_solver::fwd_kin_flange_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d M;
    M = fwd_kin_solve_approx_(q_vec);
    Eigen::Affine3d A(M);
    return A;
}

Eigen::Affine3d Davinci_fwd_solver::fwd_kin_flange_wrt_torso_solve(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d M;
    M = fwd_kin_solve_(q_vec);
    M = A_torso_to_rarm_mount_*M;
    Eigen::Affine3d A(M);
    return A;
}



Eigen::Matrix4d Davinci_fwd_solver::get_wrist_frame() {
    return A_mat_products_[5]; // frames 4 and 5 have coincident origins
}

Eigen::Matrix4d Davinci_fwd_solver::get_shoulder_frame() {
    return A_mat_products_[0]; // frame 1 has coincident origin, since a2=d2=0
}

Eigen::Matrix4d Davinci_fwd_solver::get_elbow_frame() {
    return A_mat_products_[3]; // frames 2 and 3 have coincident origins
}

Eigen::Matrix4d Davinci_fwd_solver::get_flange_frame() {
    return A_mat_products_[6];
}


Eigen::Matrix4d Davinci_fwd_solver::get_shoulder_frame_approx() {
    return A_mat_products_approx_[0]; // frame 1 has coincident origin, since a2=d2=0
}

Eigen::Matrix4d Davinci_fwd_solver::get_elbow_frame_approx() {
    return A_mat_products_approx_[3]; // frames 2 and 3 have coincident origins
}

Eigen::Matrix4d Davinci_fwd_solver::get_wrist_frame_approx() {
    Eigen::Matrix4d A_wrist;
    A_wrist = A_mat_products_approx_[5];
    //cout<<"A_wrist from get_wrist: "<<endl;
    //cout<<A_wrist<<endl;
    return A_wrist; // frames 4 and 5 have coincident origins
}

Eigen::Matrix4d Davinci_fwd_solver::get_flange_frame_approx() {
    return A_mat_products_approx_[6];
}

//fwd kin from frame 1 to wrist pt
Eigen::Vector3d Davinci_fwd_solver::get_wrist_coords_wrt_frame1(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d A_shoulder_to_wrist;
    fwd_kin_solve_(q_vec);
    A_shoulder_to_wrist = A_mats_[1]*A_mats_[2]*A_mats_[3]*A_mats_[4];
    Eigen::Vector3d w_wrt_1 = A_shoulder_to_wrist.block<3, 1>(0, 3);
    return w_wrt_1;
}
*/

/* Wrist Jacobian:  somewhat odd; restricted to q_s1, q_humerus and q_elbow
 * return a 3x3 Jacobian relating dq to delta wrist point coords, w/rt q_s1, q_humerus and q_elbow*/
// wrist coords are expressed w/rt frame1
// if q_forearm is known, use it--else set it to 0 or other approx

/*
Eigen::Matrix3d Davinci_fwd_solver::get_wrist_Jacobian_3x3(double q_s1, double q_humerus, double q_elbow, double q_forearm) {
    Vectorq7x1 q_vec;
    for (int i=0;i<7;i++) q_vec(i)=0.0;
    q_vec(1) = q_s1;
    q_vec(2) = q_humerus;
    q_vec(3) = q_elbow;
    q_vec(4) = q_forearm;
    
    Eigen::Matrix4d A_mats_3dof[5];
    Eigen::Matrix4d A_mat_products_3dof[5];

    //Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Ai;
    Eigen::Matrix3d R;
    //Eigen::Vector3d p,t1,t2;;
    Eigen::Matrix3d Jw1_trans;
    Eigen::Matrix3d Jw1_ang;
    Eigen::Matrix3d Origins;
    //Eigen::Matrix3d Rvecs;
    Eigen::Vector3d zvec,rvec,wvec,Oi;
    //populate 5 A matrices and their products; need 5 just to get to wrist point, but can assume q_forearm=0, q_wrist_bend=0
    // note--starting from S1 frame, skipping frame 0
    for (int i=0;i<5;i++) {
        A_mats_3dof[i] = compute_A_of_DH(i+1, q_vec(i+1));
    }
        
    A_mat_products_3dof[0] = A_mats_3dof[0];
    //cout<<"A_mat_products_3dof[0]"<<endl;
    //cout<<A_mat_products_3dof[0]<<endl;
    for (int i=1;i<5;i++) {
        A_mat_products_3dof[i] = A_mat_products_3dof[i-1]*A_mats_3dof[i];
    }
    wvec = A_mat_products_3dof[4].block<3, 1>(0, 3); //strip off wrist coords
    //cout<<"wvec w/rt frame1: "<<wvec.transpose()<<endl;
    
    //compute the angular Jacobian, using z-vecs from each frame; first frame is just [0;0;1]
    zvec<<0,0,1;
    Jw1_ang.block<3, 1>(0, 0) = zvec; // and populate J_ang with them; at present, this is not being returned
    Oi<<0,0,0;
    Origins.block<3, 1>(0, 0) = Oi;
    for (int i=1;i<3;i++) {
        zvec = A_mat_products_3dof[i-1].block<3, 1>(0, 2); //%strip off z axis of each previous frame; note subscript slip  
        Jw1_ang.block<3, 1>(0, i) = zvec; // and populate J_ang with them;
        Oi = A_mat_products_3dof[i-1].block<3, 1>(0, 3); //origin of i'th frame
        Origins.block<3, 1>(0, i) = Oi;
    }    
    //now, use the zvecs to help compute J_trans
    for (int i=0;i<3;i++) {
        zvec = Jw1_ang.block<3, 1>(0, i); //%recall z-vec of current axis     
        Oi =Origins.block<3, 1>(0, i); //origin of i'th frame
        rvec = wvec - Oi; //%vector from origin of i'th frame to wrist pt 
        //Rvecs.block<3, 1>(0, i) = rvec; //save these?
        //t1 = zvecs.block<3, 1>(0, i);
        //t2 = rvecs.block<3, 1>(0, i);
        Jw1_trans.block<3, 1>(0, i) = zvec.cross(rvec);  
        //cout<<"frame "<<i<<": zvec = "<<zvec.transpose()<<"; Oi = "<<Oi.transpose()<<endl;
    }     
    //cout<<"J_ang: "<<endl;
    //cout<<Jw1_ang<<endl;
    return Jw1_trans;
}
*/

// confirmed this function is silly...
// can easily transform Affine frames or A4x4 frames w:  Affine_torso_to_rarm_mount_.inverse()*pose_wrt_torso;

    /*
     * Eigen::Affine3d Davinci_fwd_solver::transform_affine_from_torso_frame_to_arm_mount_frame(Eigen::Affine3d pose_wrt_torso) {
    //convert desired_hand_pose into equiv w/rt right-arm mount frame:

    Eigen::Affine3d desired_pose_wrt_arm_mount,desired_pose_wrt_arm_mount2;
    Eigen::Matrix3d R_hand_des_wrt_torso = pose_wrt_torso.linear();
    Eigen::Vector3d O_hand_des_wrt_torso = pose_wrt_torso.translation();
    Eigen::Vector3d O_hand_des_wrt_arm_mount;
    Eigen::Vector3d O_arm_mount_wrt_torso = A_torso_to_rarm_mount_.col(3).head(3);
    Eigen::Matrix3d R_arm_mount_wrt_torso = A_torso_to_rarm_mount_.block<3, 3>(0, 0);
    
    desired_pose_wrt_arm_mount.linear() = R_arm_mount_wrt_torso.transpose()*R_hand_des_wrt_torso;
 
    O_hand_des_wrt_arm_mount = R_arm_mount_wrt_torso.transpose()* O_hand_des_wrt_torso 
            - R_arm_mount_wrt_torso.transpose()* O_arm_mount_wrt_torso; //desired hand origin w/rt arm_mount frame
           
    desired_pose_wrt_arm_mount.translation() = O_hand_des_wrt_arm_mount;  
    cout<<"input pose w/rt torso: R"<<endl;
    cout<<pose_wrt_torso.linear()<<endl;
    cout<<"origin of des frame w/rt torso: "<<pose_wrt_torso.translation().transpose()<<endl;
    
    cout<<"input pose w/rt arm-mount frame: R"<<endl;
    cout<<desired_pose_wrt_arm_mount.linear()<<endl;    
    cout<<"origin of des frame w/rt arm-mount frame: "<<desired_pose_wrt_arm_mount.translation().transpose()<<endl;

    // now, try easier approach:
    desired_pose_wrt_arm_mount2 = Affine_torso_to_rarm_mount_.inverse()*pose_wrt_torso;
     cout<<"input pose w/rt arm-mount frame, method 2: R"<<endl;
    cout<<desired_pose_wrt_arm_mount2.linear()<<endl;    
    cout<<"origin of des frame w/rt arm-mount frame, method 2: "<<desired_pose_wrt_arm_mount2.translation().transpose()<<endl;   

    
    return desired_pose_wrt_arm_mount;
    return Affine_torso_to_rarm_mount_.inverse()*pose_wrt_torso;
}
*/




//IK methods:
Davinci_IK_solver::Davinci_IK_solver() {
    //constructor: 
    //ROS_INFO("Davinci_IK_solver constructor");
    /*
    L_humerus_ = 0.37082; // diag distance from shoulder to elbow; //DH_d_params[2];
    double L3 = DH_d3;
    double A3 = DH_a3;
    //L_humerus_ = sqrt(A3 * A3 + L3 * L3);
    L_forearm_ = DH_d5; // d-value is approx len, ignoring offset; 0.37442; // diag dist from elbow to wrist; //sqrt(A3 * A3 + L3 * L3);
    
    phi_shoulder_= acos((-A3*A3+L_humerus_*L_humerus_+L3*L3)/(2.0*L3*L_humerus_));
    // the following is redundant w/ fwd_solver instantiation, but repeat here, in case fwd solver
    // is not created
    //A_rarm_mount_to_r_lower_forearm_ = Eigen::Matrix4d::Identity();
    //A_rarm_mount_to_r_lower_forearm_(0,3) = rmount_to_r_lower_forearm_x;
    //A_rarm_mount_to_r_lower_forearm_(1,3) = rmount_to_r_lower_forearm_y;
    //A_rarm_mount_to_r_lower_forearm_(2,3) = rmount_to_r_lower_forearm_z;  
    //cout<<"A_rarm_mount_to_r_lower_forearm"<<endl; // this was populated by inherited fwd_kin constructor
    //cout<<A_rarm_mount_to_r_lower_forearm_<<endl;
     * */
}

//accessor function to get all solutions

void Davinci_IK_solver::get_solns(std::vector<Vectorq7x1> &q_solns) {
    q_solns = q_solns_fit_; //q7dof_solns;
}

/*
Eigen::Vector3d Davinci_IK_solver::wrist_frame0_from_flange_wrt_rarm_mount(Eigen::Affine3d affine_flange_frame) {
    Eigen::Vector3d flange_z_axis_wrt_arm_mount;
    Eigen::Vector3d flange_origin_wrt_arm_mount;
    Eigen::Vector3d wrist_pt_vec_wrt_arm_mount;
    Eigen::Matrix3d R_flange_wrt_arm_mount;
    
    R_flange_wrt_arm_mount = affine_flange_frame.linear();
    flange_origin_wrt_arm_mount = affine_flange_frame.translation(); 
    flange_z_axis_wrt_arm_mount = R_flange_wrt_arm_mount.col(2); 
    wrist_pt_vec_wrt_arm_mount = flange_origin_wrt_arm_mount-flange_z_axis_wrt_arm_mount*DH_d7;
        
    // this much looks correct...deduce wrist point from flange frame, w/rt arm mount frame
    //cout<<"wrist pt w/rt arm mount from flange pose and IK: "<<wrist_pt_vec_wrt_arm_mount.transpose()<<endl;
    return wrist_pt_vec_wrt_arm_mount;
}
*/


bool Davinci_IK_solver::fit_q_to_range(double q_min, double q_max, double &q) {
    while (q<q_min) {
        q+= 2.0*M_PI;
    }
    while (q>q_max) {
        q-= 2.0*M_PI;
    }    
    if (q<q_min)
        return false;
    else
        return true;
}

bool Davinci_IK_solver::fit_joints_to_range(Vectorq7x1 &qvec) {
    bool fits=true;
    bool does_fit;
    double q;
    for (int i=0;i<7;i++) {
        q = qvec[i];
        does_fit = fit_q_to_range(q_lower_limits[i],q_upper_limits[i],q);
        qvec[i] = q;
        fits = fits&&does_fit;
    }
    if (fits)
        return true;
    else
        return false;
}

int Davinci_IK_solver::ik_solve(Eigen::Affine3d const& desired_hand_pose) // solve IK
{ return 0; // dummy
}


