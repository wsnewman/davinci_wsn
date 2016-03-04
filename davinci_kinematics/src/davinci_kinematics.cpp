// davinci_kinematics implementation file; start w/ fwd kin


//"include" path--should just be <davinci_kinematics/davinci_kinematics.h>, at least for modules outside this package
#include <davinci_kinematics/davinci_kinematics.h>
#include <sensor_msgs/JointState.h>

using namespace std;

//fnc to extract a joint value from a JointState message;
// provide the name of interest, as a C++ string, and provide the entire
// jointState message;  will set the value of "qval" arg, if possible;
// will return "true" or "false" to indicate if name was found on list
bool Davinci_fwd_solver::get_jnt_val_by_name(string jnt_name,sensor_msgs::JointState jointState,double &qval) {
    int njnts = jointState.name.size();
    for (int ijnt = 0;ijnt<njnts;ijnt++) {
        if (jnt_name.compare(jointState.name[ijnt])==0) {
            // found a name match!
            qval= jointState.position[ijnt];
            ROS_INFO("found name match for %s at position %d, jnt val = %f",jnt_name.c_str(),ijnt,qval);
            return true;
        }
    }
    //if get to here, did not find a match:
    ROS_WARN("no match for joint name %s",jnt_name.c_str());
    return false;
}

//some utilities to convert data types:
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

// as above, but double instead of float
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

// versions for stamped transforms
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

//and go the other direction:
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
 // equivalent DH parameters, theta and d
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
    ROS_INFO("using DH theta1, theta2, d3 = %f %f %f", thetas_DH_vec_(0),thetas_DH_vec_(1),dvals_DH_vec_(2));

}
//    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;

Vectorq7x1 Davinci_fwd_solver::convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec, 
        const Eigen::VectorXd &dvals_DH_vec) {
    //    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;
    //Eigen::VectorXd theta_DH_offsets_,dval_DH_offsets_;
    Vectorq7x1 q_vec;
    for (int i=0;i<7;i++) {
        q_vec(i) = thetas_DH_vec(i)-theta_DH_offsets_(i);
    }
    q_vec(2) = dvals_DH_vec(2)-dval_DH_offsets_(2);
    return q_vec;

}

//given 4 DH parameters, compute the corresponding transform as an affine3d
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
    //R(3,1) = 0; //% erdem IK bug #1: initialize 0 values in R matrix explicitly otherwise causes numerical issues
    R(2, 0) = 0;
    R(2, 1) = sa;
    R(2, 2) = ca;
    affine_DH.linear() = R;
    
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    affine_DH.translation() = p;

     return affine_DH;
}

// use member fncs to compute and multiply successive transforms
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
// converts q_Vec to DH params, then uses above fnc for fwd kin
Eigen::Affine3d Davinci_fwd_solver::fwd_kin_solve(const Vectorq7x1& q_vec) {   
    //convert q_vec to DH coordinates:
    //ROS_INFO("converting q to DH vals");
    convert_qvec_to_DH_vecs(q_vec);
    //cout<<"theta_DH: "<<thetas_DH_vec_.transpose()<<endl;
    //cout<<"dvals_DH: "<<dvals_DH_vec_.transpose()<<endl;
    affine_gripper_wrt_base_ = fwd_kin_solve_DH(thetas_DH_vec_, dvals_DH_vec_);     
    return affine_gripper_wrt_base_;
}

//gen_rand_legal_jnt_vals: compute random values within legal joint range:
void Davinci_fwd_solver::gen_rand_legal_jnt_vals(Vectorq7x1 &qvec) {
    double drand_val;
    qvec(6)=0;
    for (int i=0;i<6;i++) {
        drand_val = ((double) rand())/((double) RAND_MAX);
        qvec(i) = q_lower_limits[i] + (q_upper_limits[i]-q_lower_limits[i])*drand_val;
           //q_lower_limits[i],q_upper_limits[i]
        }

            //if (q<q_lower_limits[i]) does_fit=false;
            //if (q>q_upper_limits[i]) does_fit=false;
}


//IK methods:
Davinci_IK_solver::Davinci_IK_solver() {
    //constructor: 
    //ROS_INFO("Davinci_IK_solver constructor");
    min_dist_O4_to_gripper_tip_ = sqrt(gripper_jaw_length*gripper_jaw_length 
             + dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis);
    ROS_INFO("min_dist_O4_to_gripper_tip_=%f",min_dist_O4_to_gripper_tip_);
}

//accessor function to get all solutions

void Davinci_IK_solver::get_solns(std::vector<Vectorq7x1> &q_solns) {
    q_solns = q_solns_fit_; //q7dof_solns;
}

//ik_solve uses helper funcs below:
// compute_w_from_tip finds the wrist point (at wrist bend), given desired gripper frame;
// q123_from_wrist solves for first three joint displacements, given wrist point

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
//  zvec_4 has a +/- ambiguity

Eigen::Vector3d Davinci_IK_solver::compute_w_from_tip(Eigen::Affine3d affine_gripper_tip, Eigen::Vector3d &zvec_4,
   Eigen::Vector3d &alt_O4) {
  // the following are all expressed w/rt the 0 frame
  Eigen::Vector3d zvec_tip_frame,xvec_tip_frame,origin_5, zvec_5, xvec_5,origin_4; // zvec_4;   
  Eigen::Matrix3d R_tip;
  R_tip = affine_gripper_tip.linear();
  zvec_tip_frame= R_tip.col(2);
  xvec_tip_frame= R_tip.col(0);
  zvec_5 = -xvec_tip_frame; // by definition of tip frame
  origin_5 = affine_gripper_tip.translation() - gripper_jaw_length*zvec_tip_frame;
  //cout<<"O5: "<<origin_5.transpose()<<endl;
  Eigen::Vector3d z_perp, z_parallel;
  // consider these two planes: 
  // define vector z_perp, which is the same a z5
  // P_perp contains O5 and is perpendicular to z5; claim: P_perp contains 04
  // P_parallel is defined by: contains O_0, contains O_5 and contains z5; claim: P_parallel contains O4
  // given P_parallel, can compute the normal vector to this plane--call it z_parallel
  //
  //  note that plane P_parallel is perpendicular to P_perp; (z_perp is perpendicular to z_parallel)
  //  if both planes contain O4, then O4 lies along the line of intersection of P_perp with P_parallel
  //  this line must be perpendicular to z_perp and to z_parallel, and thus it is +/- z_parallel cross z_perp
  // call this intersect_vec;
  //  intersect_vec is the same as +/- x5: the vector from z4 to z5 (in DH notation); sign is ambiguous at this point
  //  O4 can be found by starting from O5, moving distance "dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis" along x5
  //  to resolve the sign ambiguity, consider two O4 candidates: O4a = O5-dist*intersect_vec, 
  //  and O4b = O5+dist*intersect_vec
  //  the correct solution is the point that is CLOSEST to the origin O_0
  
  
  // plane P_perp is perpendicular to z_perp and contains O5
  // plane P_parallel is perpendicular to z_parallel and contains O5, base origin, and z_perp
  z_perp = zvec_5; //used to define a plane perpendicular to jaw-rotation axis
  
  //z_parallel is a problem if zvec_5 points at the origin (portal).
  //worst case of feasible pose is when q5 is +/- 90 deg.
  Eigen::Vector3d p_hat_O_O5 = origin_5/origin_5.norm();

  z_parallel = z_perp.cross(p_hat_O_O5); // O5 - O_0 is same as O5
  double mag_z_perp_cross_O5 = z_parallel.norm();
  cout<<"zvec_5: "<<zvec_5.transpose()<<endl;
  cout<<"p_hat_O5: "<<p_hat_O_O5.transpose()<<endl;
  ROS_WARN("mag_z_perp_cross_O5=%f",mag_z_perp_cross_O5);
  z_parallel = z_parallel/mag_z_perp_cross_O5;
  //cout<<"z_parallel: "<<z_parallel.transpose()<<endl;
  xvec_5 = -z_perp.cross(z_parallel); // could be + or -  ?
  xvec_5 = xvec_5/(xvec_5.norm()); // should not be necessary--already unit length
  cout<<"xvec_5: "<<xvec_5.transpose()<<endl;
  
  
  //should get gripper-jaw angle from gripper z_des and xvec_5
  //q6 is rotation from xvec_5 to z_gripper_des about zvec_5
  
  
  Eigen::Vector3d origin_4a,origin_4b,O_tip;
  O_tip = affine_gripper_tip.translation();
  origin_4a = origin_5-dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*xvec_5;
  origin_4b = origin_5+dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*xvec_5;
  double dist_O4a_gripper_tip = (O_tip-origin_4a).norm();
  double dist_O4b_gripper_tip = (O_tip-origin_4b).norm();
  cout<<"dist_O4a_gripper_tip="<<dist_O4a_gripper_tip<<"; dist_O4b_gripper_tip="<<dist_O4b_gripper_tip<<endl;
  //if ((dist_O4a_gripper_tip<min_dist_O4_to_gripper_tip_)&&(dist_O4b_gripper_tip<min_dist_O4_to_gripper_tip_)) {
  //    ROS_WARN("BOTH 04 options too close to gripper tip!");
  //}
  origin_4 = origin_4a;
  alt_O4 = origin_4b;
  /*
  if (origin_4b.norm()<origin_4a.norm()) {
        origin_4 = origin_4b;
        alt_O4 = origin_4a;
        //FIX THIS?
        xvec_5 = -xvec_5; //default solution was incorrect, so negate x5 direction
        // but we don't return xvec5 anyway, so no issue?
  }
  */ 
  // possible error here: need to get sign of xvec_5 correct.
  // given O_4 and O_5, should have xvec_5 point from O_4 towards O_5
  //cout<<"origin_4: "<<origin_4.transpose()<<endl;
  // if use CORRECT direction of x5 axis and z5 axis, does CORRECT direction of zvec_4 follow?
  zvec_4 = -(zvec_5.cross(xvec_5)); //ambiguity here: zvec_4 could be +/- along this direction
  //erdem IK bug # 2: correct cross product sign
  //cout<<"zvec_4: "<<zvec_4.transpose()<<endl;
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
        return false; //rtn false if no periodic soln in range
    else
        return true;
}

bool Davinci_IK_solver::fit_joints_to_range(Vectorq7x1 &qvec) {
    bool fits=true;
    bool does_fit;
    double q;
    for (int i=0;i<7;i++) {
        q = qvec[i];
        if (i!=2) { //treat d3 differently
                does_fit = fit_q_to_range(q_lower_limits[i],q_upper_limits[i],q);
        }
        else { //special case for d3...although generic formula also works in this case
            does_fit=true;
            if (q<q_lower_limits[i]) does_fit=false;
            if (q>q_upper_limits[i]) does_fit=false;
        }
        if (!does_fit) {
            ROS_WARN("IK err: jnt %d;  lower lim: %f; upper lim: %f desired val = %f;",
                    i,q_lower_limits[i],q_upper_limits[i],q);
        }
        qvec[i] = q;
        fits = fits&&does_fit;
    }
    if (fits)
        return true;
    else
        return false;
}



int Davinci_IK_solver::ik_solve(Eigen::Affine3d const& desired_hand_pose) // solve IK
{ 
   Eigen::Vector3d z_vec4,z4_wrt_3,O_6_wrt_4,xvec6_wrt_5,O_5_wrt_base,zvec5_wrt_base;
   Eigen::Vector3d w_wrt_base,q123, alt_w_wrt_base, alt_q123,des_tip_origin,zvec_tip_wrt_base;
   Eigen::Vector3d w_fk_test;
   Eigen::VectorXd theta_vec,d_vec;   
   Eigen::Affine3d affine_test_fk;
   Eigen::Matrix3d R_tip_wrt_base;
   double err;
   //disallow a positive tool-tip z-height--> would be above the portal
   // in fact, must insert at least past the wrist joint, z4, so tip must
   // tip_z must be at least 
   des_tip_origin = desired_hand_pose.translation();
   double tool_tip_z_des = des_tip_origin(2);
   if (tool_tip_z_des>0.0) {
       ROS_WARN("requested tool tip height above the portal!");
       cout<<"tool_tip_z_des: "<<tool_tip_z_des<<endl;
       return 0; //no solns
   }
   //check if O5 is above the portal:
   R_tip_wrt_base= desired_hand_pose.linear();
   zvec_tip_wrt_base= R_tip_wrt_base.col(2); 
   O_5_wrt_base = des_tip_origin - zvec_tip_wrt_base*gripper_jaw_length;
   if (O_5_wrt_base(2)>0.0) {
       ROS_WARN("requested jaw axis height above the portal!");
       cout<<"O_5_wrt_base: "<<O_5_wrt_base.transpose()<<endl;
       return 0; //no solns      
   }
   //test if gripper z-axis implies excessive wrist bend:
   // consider vector from portal (origin) to O5 (jaws-axis), and project
   // the desired gripper z-axis onto this vector.  Result must be >0 for
   // wrist bend to be |q5|< pi/2
   // this is a necessary but not sufficient test; 
   //can still violate wrist-bend>pi/2 and pass this test
   double projection_gripper_zvec_onto_O5_vec = zvec_tip_wrt_base.dot(O_5_wrt_base);
   if (projection_gripper_zvec_onto_O5_vec<= 0.0) {
       ROS_WARN("requested gripper z-vec points towards portal--excessive wrist bend required");
       return 0; //no solns   
   }
   //; // by definition of tip frame
   //better: look at cross product of O_5_wrt_base and zvec5_wrt_base
   zvec5_wrt_base = -R_tip_wrt_base.col(0); // gripper x-axis is same as z5 
   double mag_z5xO5 = (zvec5_wrt_base.cross(O_5_wrt_base)).norm();
   if (mag_z5xO5 < dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis) {
       ROS_WARN("mag_z5xO5 =%f < L_w_to_gripperJaw_axis; --> excessive wrist bend",mag_z5xO5);
       return 0;
   }
   
   //first step: get the wrist-bend origin on tool shaft from desired gripper pose:
   //desired_hand_pose input, z_vec4, alt_w_wrt_base output
   w_wrt_base = compute_w_from_tip(desired_hand_pose,z_vec4,alt_w_wrt_base);
   //cout<<"origin4 from IK: "<<w_wrt_base.transpose()<<endl;
   //cout<<"z_vec4 from IK: "<<z_vec4.transpose()<<endl;
   
   //next step: get theta1, theta2, d3 soln from wrist position:
   q123 = q123_from_wrist(w_wrt_base);
   w_fk_test = compute_fk_wrist(q123);
   cout<<"w_wrt_base: "<<w_wrt_base.transpose()<<endl;
   cout<<"w_fk_test :"<<w_fk_test.transpose()<<endl;
   cout<<"w_err: "<<(w_wrt_base-w_fk_test).norm()<<endl;
   //cout<<"q123: "<<q123.transpose()<<endl;
   compute_q456(q123,z_vec4,desired_hand_pose);
   
 
   //DEBUG:
   //test this soln:
   cout<<"SOLN: "<<endl;
   affine_test_fk= fwd_kin_solve(q_vec_soln_);
   cout<<"soln: "<<q_vec_soln_.transpose()<<endl;
   //if (!fit_joints_to_range(q_vec_soln_)) ROS_WARN("joint range violation");   
   //cout<<"fk soln: "<<endl;
   //cout<<"gripper origin: "<<affine_test_fk.translation().transpose()<<endl;
   err=(affine_test_fk.translation()-desired_hand_pose.translation()).norm();
   ROS_WARN("gripper origin err: %f",err);
   //cout<<"Rotation:"<<endl;
   //cout<<affine_test_fk.linear()<<endl;
   //cout<<"Rotation err:"<<endl;
   //cout<<(desired_hand_pose.linear()-affine_test_fk.linear())<<endl;
  
   //compute_q456(q123,-z_vec4,desired_hand_pose);
   //affine_test_fk= fwd_kin_solve(q_vec_soln_);
   // cout<<"soln: "<<q_vec_soln_.transpose()<<endl;
   //if (!fit_joints_to_range(q_vec_soln_)) ROS_WARN("joint range violation"); 
    
   //cout<<"fk soln: "<<endl;
   //cout<<"gripper origin: "<<affine_test_fk.translation().transpose()<<endl;
   //err=(affine_test_fk.translation()-desired_hand_pose.translation()).norm();
   //ROS_WARN("Soln 1 err w/ -zvec:  %f",err);
   //cout<<"soln: "<<q_vec_soln_.transpose()<<endl;
   
   
   //alt soln:
   /*
   alt_q123 = q123_from_wrist(alt_w_wrt_base); //the other possibility...unlikely, and out of jnt range
   w_fk_test = compute_fk_wrist(alt_q123);   
   //cout<<"q123: "<<q123.transpose()<<endl;
   //cout<<"alt w_wrt_base: "<<alt_w_wrt_base.transpose()<<endl;
   //cout<<"w_fk_test :"<<w_fk_test.transpose()<<endl;
   //cout<<"alt soln, w_err: "<<(alt_w_wrt_base-w_fk_test).norm()<<endl;
   
   compute_q456(alt_q123,-z_vec4,desired_hand_pose); //+/- z_vec4?
   cout<<"ALT SOLN: "<<endl;
   affine_test_fk= fwd_kin_solve(q_vec_soln_);
   cout<<"soln: "<<q_vec_soln_.transpose()<<endl;
   //if (!fit_joints_to_range(q_vec_soln_)) ROS_WARN("joint range violation");    
   //cout<<"fk soln: "<<endl;
   //cout<<"gripper origin: "<<affine_test_fk.translation().transpose()<<endl;
   err=(affine_test_fk.translation()-desired_hand_pose.translation()).norm();
   ROS_WARN("alt soln gripper origin err: %f",err);
   //cout<<"Rotation:"<<endl;
   //cout<<affine_test_fk.linear()<<endl;
   //cout<<"Rotation err:"<<endl;
   //cout<<(desired_hand_pose.linear()-affine_test_fk.linear())<<endl;
   
   compute_q456(alt_q123,z_vec4,desired_hand_pose); //+/- z_vec4?
   affine_test_fk= fwd_kin_solve(q_vec_soln_);
   cout<<"soln: "<<q_vec_soln_.transpose()<<endl;
   //if (!fit_joints_to_range(q_vec_soln_)) ROS_WARN("joint range violation");    
   //cout<<"fk soln: "<<endl;
   //cout<<"gripper origin: "<<affine_test_fk.translation().transpose()<<endl;
   err=(affine_test_fk.translation()-desired_hand_pose.translation()).norm();
   ROS_WARN("alt soln gripper origin, -zvec err: %f",err);  
   */
   //fit_joints_to_range(q_vec_soln_); // erdem IK bug #3: check and correct angle periodicity.        
   //cout<<"soln: "<<q_vec_soln_.transpose()<<endl;
   //cout<<"origin: ";
   //cout<<affine_frame_wrt_base.translation().transpose()<<endl;  
   //ROS_INFO("x-axis from above is reference for theta4");
   //double sval = 
   if (fit_joints_to_range(q_vec_soln_)) //rtns true if legal soln
    return 1; // 1 legal soln
   else
       return 0; //0 legal solns
   //must use get_soln() to return q_vec_soln_
}

//debug fnc: compute FK of wrist pt from q123
//could have put this in IK instead...
Eigen::Vector3d Davinci_IK_solver::compute_fk_wrist(Eigen::Vector3d q123)
{
   Eigen::Affine3d affine_frame_wrt_base;
   Eigen::Vector3d wrist_pt;
   
   Eigen::VectorXd theta_vec,d_vec; 
   //compute FK of this soln:
   theta_vec.resize(7); //<<q123(0),q123(1),0,0,0,0,0;
   theta_vec<<0,0,0,0,0,0,0;
   theta_vec(0) = q123(0);
   theta_vec(1) = q123(1);
   //cout<<"theta_vec: "<<theta_vec.transpose()<<endl;
   d_vec.resize(7);
   d_vec<<0,0,0,0,0,0,0;
   d_vec(2) = q123(2);
   //cout<<"d_vec: "<<d_vec.transpose()<<endl;
   
   //use partial IK soln to compute FK of first three frames:
   //ROS_INFO("calling fwd_kin_solve_DH()");
   fwd_kin_solve_DH(theta_vec, d_vec);
   // ROS_INFO("wrist frame (frame 3) from IK/FK: ");
   affine_frame_wrt_base = get_affine_frame(2); // this frame depends only on 1st 3 var's
   wrist_pt = affine_frame_wrt_base.translation();
   return wrist_pt;
}

//populates q_vec_soln_; rtns 0 if not legal solns
int Davinci_IK_solver::compute_q456(Eigen::Vector3d q123,Eigen::Vector3d z_vec4,Eigen::Affine3d desired_hand_pose)
{
   Eigen::Affine3d affine_frame_wrt_base,affine_frame6_wrt_4,affine_frame6_wrt_5,fk_gripper_frame;
   Eigen::Vector3d z4_wrt_3,O_6_wrt_4,xvec6_wrt_5;
   Eigen::VectorXd theta_vec,d_vec; 
   //compute FK of this soln:
   theta_vec.resize(7); //<<q123(0),q123(1),0,0,0,0,0;
   theta_vec<<0,0,0,0,0,0,0;
   theta_vec(0) = q123(0);
   theta_vec(1) = q123(1);
   //cout<<"theta_vec: "<<theta_vec.transpose()<<endl;
   d_vec.resize(7);
   d_vec<<0,0,0,0,0,0,0;
   d_vec(2) = q123(2);
   //cout<<"d_vec: "<<d_vec.transpose()<<endl;
   
   //use partial IK soln to compute FK of first three frames:
   //ROS_INFO("calling fwd_kin_solve_DH()");
   fwd_kin_solve_DH(theta_vec, d_vec);
   // ROS_INFO("wrist frame (frame 3) from IK/FK: ");
   affine_frame_wrt_base = get_affine_frame(2); // this frame depends only on 1st 3 var's
   Eigen::Matrix3d R_3_wrt_base;
   R_3_wrt_base = affine_frame_wrt_base.linear();
   //cout<<"affine linear (R): "<<endl;
   //cout<<R_3_wrt_base<<endl;
   //cout<<endl;
   //use this to express z_vec4 in frame-3 coords.  Expect z-component to be zero
   z4_wrt_3 = R_3_wrt_base.transpose()*z_vec4;
   //cout<<"z4 w/rt frame 3: "<<z4_wrt_3.transpose()<<endl;
   // FIX THIS: given ambiguity of +/- z_vec4, there are 2 solns here, PI apart
   double theta4 = atan2(z4_wrt_3(1),z4_wrt_3(0))+M_PI/2.0;
   //ROS_INFO("theta4 = %f",theta4);
   
   // for the following, it might be easier to use knowledge of O4 and O5 to compute theta5
   // also, given x5_vec, and z6_vec_desired, should be able to get theta6
   
   //recompute FK for 1st 4 variables:
   theta_vec(3) = theta4;   
   fwd_kin_solve_DH(theta_vec, d_vec);  
   // get frame 4, which depends on 1st 4 vars:
   affine_frame_wrt_base = get_affine_frame(3);  
   
   //compute transform frame 6 wrt frame 4:
   // A_{g/base} = A_{4/base}*A_{6/4}*A_{g/6}
   // so, A_{4/base}_inv * A_{g/base} * A_{g/6}_inv = A_{4/base}
   affine_frame6_wrt_4 = affine_frame_wrt_base.inverse()*desired_hand_pose*affine_gripper_wrt_frame6_.inverse();
   O_6_wrt_4 = affine_frame6_wrt_4.translation();
   double theta5 =  atan2(O_6_wrt_4(1),O_6_wrt_4(0));
   //ROS_INFO("theta5 = %f",theta5);   
   
   theta_vec(4) = theta5;   
   fwd_kin_solve_DH(theta_vec, d_vec);  
   // get frame 5, which depends on 1st 5 vars:
   affine_frame_wrt_base = get_affine_frame(4); 
   affine_frame6_wrt_5= affine_frame_wrt_base.inverse()*desired_hand_pose*affine_gripper_wrt_frame6_.inverse();
   //cout<<"origin frame 6 wrt 5: "<<affine_frame6_wrt_5.translation().transpose()<<endl;
   //cout<<"R 6 wrt 5: "<<endl;
   //cout<<affine_frame6_wrt_5.linear()<<endl;
           
   xvec6_wrt_5 = affine_frame6_wrt_5.linear().col(0);
   double theta6 = atan2(xvec6_wrt_5(1),xvec6_wrt_5(0));
   //ROS_INFO("theta6 = %f",theta6); 
   theta_vec(5) = theta6;   
   //fk_gripper_frame = fwd_kin_solve_DH(theta_vec, d_vec);  
   //cout<<"FK gripper frame: ";
   //cout<<"origin: "<<fk_gripper_frame.translation().transpose()<<endl;
   //cout<<"R:"<<endl;
   //cout<<fk_gripper_frame.linear()<<endl;
   
   // pack the solution in to a single vector
   q_vec_soln_ =  convert_DH_vecs_to_qvec(theta_vec, d_vec);
   }
