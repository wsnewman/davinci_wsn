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
    //ROS_INFO("q_vec(2), dvals_DH_vec_(2) = %f, %f",q_vec(2),dvals_DH_vec_(2));
    ROS_INFO("using theta1, theta2, d3 = %f %f %f", thetas_DH_vec_(0),thetas_DH_vec_(1),dvals_DH_vec_(2));

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
   /*
   Eigen::Matrix3d R_gripper_wrt_frame6;
   Eigen::Vector3d Origin_gripper_wrt_frame6;
   Origin_gripper_wrt_frame6<<0,0,gripper_jaw_length;
   z_axis<<0,0,1; 
   x_axis<<1,0,0;  
   y_axis<<0,1,0; 
   R_gripper_wrt_frame6.col(0) = x_axis;
   R_gripper_wrt_frame6.col(1) = y_axis;
   R_gripper_wrt_frame6.col(2) = z_axis;     
   affine_gripper_wrt_frame6_.linear() = R_gripper_wrt_frame6;
   affine_gripper_wrt_frame6_.translation() = Origin_gripper_wrt_frame6;   
   */
   affine_gripper_wrt_frame6_ = computeAffineOfDH(0, gripper_jaw_length, 0,-M_PI/2);
   
   theta_DH_offsets_.resize(7);
   for (int i=0;i<7;i++) {
       theta_DH_offsets_(i) = DH_q_offsets[i];
   }
   theta_DH_offsets_(2) = 0.0; //don't put prismatic displacement here
   
   dval_DH_offsets_.resize(7);
   dval_DH_offsets_<<0,0,DH_q_offsets[2],0,0,0,0;
   

}

//provide DH theta and d values, return affine pose of gripper tip w/rt base frame
// also computes all intermediate affine frames, w/rt base frame
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_solve_DH(const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& d_vec) { 
    //use or affect these member variables:
    //Eigen::Affine3d affine_frame0_wrt_base_;
    //Eigen::Affine3d affine_gripper_wrt_frame6_;    
    //Eigen::Affine3d affine_gripper_wrt_base_; 
    //vector <Eigen::Affine3d> affines_i_wrt_iminus1_;
    //vector <Eigen::Affine3d> affine_products_;    
    //    vector <Eigen::Affine3d> affines_i_wrt_iminus1_;
    //  vector <Eigen::Affine3d> affine_products_;
    
    affines_i_wrt_iminus1_.resize(7);
    //ROS_INFO("computing successive frame transforms: ");
    Eigen::Affine3d xform;
    double a,d,theta,alpha;
    for (int i=0;i<7;i++) {
        a = DH_a_params[i];
        d = d_vec(i);
        alpha = DH_alpha_params[i];
        theta = theta_vec(i);
        //ROS_INFO("i = %d; a,d,alpha,theta = %f %f %f %f",i,a,d,alpha,theta);
        xform= computeAffineOfDH(a, d, alpha,theta);
        affines_i_wrt_iminus1_[i]= xform;
    }
    
    //ROS_INFO("computing transform products: ");
    affine_products_.resize(7);
    affine_products_[0] =  affine_frame0_wrt_base_*affines_i_wrt_iminus1_[0];
    for (int i=1;i<7;i++) {
        affine_products_[i] = affine_products_[i-1]*affines_i_wrt_iminus1_[i];
    }
    
    affine_gripper_wrt_base_ = affine_products_[6]*affine_gripper_wrt_frame6_;
    
    return affine_gripper_wrt_base_;    
}


// fwd-kin fnc: computes gripper frame w/rt base frame given q_vec
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_solve(const Vectorq7x1& q_vec) {   
    //convert q_vec to DH coordinates:
    //ROS_INFO("converting q to DH vals");
    convert_qvec_to_DH_vecs(q_vec);
    //cout<<"theta_DH: "<<thetas_DH_vec_.transpose()<<endl;
    //cout<<"dvals_DH: "<<dvals_DH_vec_.transpose()<<endl;
    affine_gripper_wrt_base_ = fwd_kin_solve_DH(thetas_DH_vec_, dvals_DH_vec_);     
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


//given a 3-D wrist point w/rt base frame (at portal origin), solve for theta1, theta2 and d3;
// return these in a vector (in that order)
// "wrist point" is ambiguous.  meaning here is O_3 = O_4 = intersection of tool-shaft rotation and
// (first) wrist-bend axis
// THIS LOOKS GOOD--though should be more thoroughly tested
Eigen::Vector3d Davinci_IK_solver::q123_from_wrist(Eigen::Vector3d wrist_pt) {
    Eigen::Vector3d q123;
    double theta1, theta2, d3;
    d3 = wrist_pt.norm(); //that was easy enough
    //now, w = R_1/0*R_2/1*[0;0;d3]
    // or, [wx;wy;wz] = [c1*s2;s1*s2; -c2]*d3
    Eigen::Vector3d w_prime; // transform w to w_wrt_frame0, then scale it w/ w/d3;
    //note: in frame0, wrist-point z-value is measured along yaw (jnt1) z-axis;
    // displacement along z0 axis depends on tool-insertion length, d3, and on rotation of pitch mechanism, theta2
    // note that theta2 is pi/2 + q_davinci(1);
    // if range of q_davinci is +/- pi/2, then range of theta2 is 0 to +pi
    w_prime = wrist_pt/d3;
    w_prime = affine_frame0_wrt_base_.inverse()*w_prime;
    //arc cosine of x, in the interval [0,pi] radians...which is interval of interest for theta2, so keep this soln
    theta2 = acos(-w_prime(2));
    // s2 will always be >0 for 0<theta2<pi
    // so atan2 should yield a good answer
    theta1 = atan2(w_prime(1),w_prime(0));
    q123(0) = theta1;
    q123(1) = theta2;
    q123(2) = d3;
    
    return q123;
}

//defined tool-tip frame such that x-axis is anti-parallel to the gripper-jaw rotation axis
// "5" frame is frame w/ z-axis through the last rotation joint--rotation of gripper jaws
// return the wrist point...but also calculate zvec_4

Eigen::Vector3d Davinci_IK_solver::compute_w_from_tip(Eigen::Affine3d affine_gripper_tip, Eigen::Vector3d &zvec_4) {
  // the following are all expressed w/rt the 0 frame
  Eigen::Vector3d zvec_tip_frame,xvec_tip_frame,origin_5, zvec_5, xvec_5,origin_4; // zvec_4;   
  Eigen::Matrix3d R_tip;
  R_tip = affine_gripper_tip.linear();
  zvec_tip_frame= R_tip.col(2);
  xvec_tip_frame= R_tip.col(0);
  zvec_5 = -xvec_tip_frame; // by definition of tip frame
  origin_5 = affine_gripper_tip.translation() - gripper_jaw_length*zvec_tip_frame;
  cout<<"O5: "<<origin_5.transpose()<<endl;
  Eigen::Vector3d z_perp, z_parallel;
  // plane P_perp is perpendicular to z_perp and contains O5
  // plane P_parallel is perpendicular to z_parallel and contains O5, base origin, and z_perp
  z_perp = zvec_5; //used to define a plane perpendicular to jaw-rotation axis
  z_parallel = z_perp.cross(origin_5); // O5 - O_0 is same as O5
  z_parallel = z_parallel/(z_parallel.norm());
  cout<<"z_parallel: "<<z_parallel.transpose()<<endl;
  xvec_5 = z_perp.cross(z_parallel); // could be + or -
  xvec_5 = xvec_5/(xvec_5.norm()); // should not be necessary--already unit length
  cout<<"xvec_5: "<<xvec_5.transpose()<<endl;
  
  Eigen::Vector3d origin_4a,origin_4b;
  origin_4a = origin_5-dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*xvec_5;
  origin_4b = origin_5+dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*xvec_5;
  origin_4 = origin_4a;
  if (origin_4b.norm()<origin_4a.norm()) {
        origin_4 = origin_4b;
  }
  cout<<"origin_4: "<<origin_4.transpose()<<endl;
  zvec_4 = zvec_5.cross(xvec_5);
  cout<<"zvec_4: "<<zvec_4.transpose()<<endl;
  return origin_4;
}


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


