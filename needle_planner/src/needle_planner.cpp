/* 
 * File:   needle_planner.cpp
 * Author: wsn
 *
 * Created Feb 24, 2016
 */
#include <needle_planner/needle_planner.h>
//library to compute gripper poses to perform needle driving
// main fnc is: void compute_needle_drive_gripper_affines(...)
// need to specify tissue properties (w/rt camera frame)
// Also specify grasp transform (or accept the default) and choose which arm (or accept default=PSM1)
// provide tissue entrance pt, exit pt and surface normal (w/rt camera frame)
// Get back a vector of affines of corresponding gripper affines, wrt camera frame
// (really, inputs/outputs w/rt whatever frame is of interest, but presumably camera frame)


// NEEDLE-DRIVE ASSUMPTIONS:  
// this code assumes needle driving will be in a circular arc about the needle center
// The needle center is defined as the center of the circle that contains the needle semi-circle
//  This is defined as the origin of the needle frame. The z-axis of the needle frame, bvec_needle,
// is defined perpendicular to the needle plane, with positive direction corresponding to
// needle driving with the tip leading (a requirement) corresponds to positive rotation about
// the needle z axis.
// at present, this code assumes rotation of the needle during driving is about 
// an axis parallel to the tissue surface, i.e. needle z-axis is perpendicular to tissue normal
//  (this is not required, but common)

// NEEDLE GRASP ASSUMPTIONS:
// this code does not care which arm is used; all gripper affines are computed to achieve
// a needle-drive path; not arm IK is performed here

// one can choose where on the circumference of the needle to grab it--but pragmatically,
// one will grab the needle as close to the needle tail as possible (i.e., as far from the 
// needle point as possible), so as to minimize requirements for re-grasping during needle driving
// The choice of where along the circumference to grab the needle is phi_grab_
// Define this as: phi_grab_=0 means to grab the needle in the center of its arc;
// phi_grab_ = +pi/2 means to grab the needle at its tail (should make this pi/2- half_gripper_width
//   to get a full grasp).  phi_grab_ = -pi/2 would mean to grab the gripper tip (not useful for
//  needle driving).

// a common grasp strategy is to hold the needle such that its plane is perpendicular to
// the gripper z-axis, i.e. the needle bvec_needle = +/- bvec_gripper; this is not required in general,
// but it is the default grasp pose for this code

// One must choose the contact points on the gripper where the needle will be held.  The
// default assumption for this code is that the needle will be grabbed half-way between the
// gripper tips and the gripper-jaw joint.  This is changeable via a "set" call.

// With the constraint that the needle plane is perpendicular to the gripper bvec_gripper, there
// are still 4 variations: bvec_needle = +/- bvec_gripper, and needle origin along positive or
// negative gripper y-axis (tvec_gripper).  These may by: 1) selecting direction of bvec_needle
//  (e.g., parallel or antiparallel to bvec_gripper), and 2) choosing "needle_plus_y" or "needle_ninus_y".
// For bvec_needle parallel or antiparallel to bvec_gripper, "needle_plus_y" means that the needle 
// origin will lie on the positive tvec_gripper axis; "needle_minus_y" means that the needle lies on
// the negative tvec_gripper axis.
// More generally, the needle z-axis, bvec_needle, does not have to be parallel or antiparallel to
// the gripper bvec_gripper.  In this case, the needle origin will not lie on the gripper tvec_gripper
// axis.  However, the needle origin would lie in either the positive-y half space of the gripper frame
// or the negative-y half-space of the gripper frame, and the designation "needle_plus_y" would
// still be meaningful.  However, this does not cover the special case where the plane of the needle
// contains bvec_gripper.  In this case, the needle origin will lie at y/gripper = 0.  If this case
// This case is not covered here.


// a common grasp is to have the needle surface-tangent parallel or antiparallel to the gripper x axis at 
//  the grasp location ,i.e. the needle extends out perpendicular to the jaws, i.e.
//  needle tangent  is perpendicular to gripper bvec
// it is also common to have the needle z-axis parallel to or antiparallel to the gripper z axis:
// bvec_needle || bvec_gripper, or bvec_needle || -bvec_gripper
//  That is, the needle origin lies along the (plus or minus) gripper y axis.
// Define the needle-frame x-axis as pointing from needle center (origin) towards needle tip
// Approximately, if grab the needle as far as possible from the needle tip, 
// and this grasp has needle bvec_needle || +/- bvec_gripper, then the needle tip 
// tip would also lie along the 


NeedlePlanner::NeedlePlanner() {
    ROS_INFO("needle planner constructor: initializations");
    needle_radius_ = DEFAULT_NEEDLE_RADIUS;
    needle_axis_ht_ = DEFAULT_NEEDLE_AXIS_HT;
    grasp_depth_ = DEFAULT_NEEDLE_GRASP_DEPTH;     
    grab_needle_plus_minus_y_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y;
    phi_grab_ = DEFAULT_PHI_GRAB;   
    ROS_INFO("needle arc radius = %f",needle_radius_);
    ROS_INFO("needle ht above tissue = %f",needle_axis_ht_);
    //compute entrance-to-exit distance, based on needle radius and needle-axis height
    //consider equilateral triangle, needle origin above tissue (base) at ht h
    //sides of triangle are both r
    // two right triangles w/ height h and hypoteneus r--> b_rt (right-triangle base) = sqrt(r^2 - h^2)
    //base of equilateral triangle = 2*b_rt    
    double b_rt = sqrt(needle_radius_*needle_radius_ - needle_axis_ht_*needle_axis_ht_);
    dist_entrance_to_exit_ = 2*b_rt;
    ROS_INFO("at needle ht %f, distance from entrance to exit is %f",needle_axis_ht_,dist_entrance_to_exit_);
          
    // the next 2 vectors are constant during needle driving
    O_needle_wrt_tissue_<<0.5*dist_entrance_to_exit_,0,needle_axis_ht_;
    bvec_needle_wrt_tissue_frame_<<0,-1,0; //needle z-axis points along tissue-frame -y axis
    
    //set initial pose of needle relative to tissue; these values change during insertion
    nvec_needle_wrt_tissue_frame_<<-1,0,0; //start w/ needle x-axis along -tissue-frame -x axis
    tvec_needle_wrt_tissue_frame_ = bvec_needle_wrt_tissue_frame_.cross(nvec_needle_wrt_tissue_frame_);
    //initial orientation of needle frame, w/rt tissue frame:
    R0_needle_wrt_tissue_.col(0) = nvec_needle_wrt_tissue_frame_;
    R0_needle_wrt_tissue_.col(1) = tvec_needle_wrt_tissue_frame_;        
    R0_needle_wrt_tissue_.col(2) = bvec_needle_wrt_tissue_frame_;
    //put O and R0 into an affine for initial pose of needle w/rt tissue frame
    affine_init_needle_frame_wrt_tissue_.linear() = R0_needle_wrt_tissue_;
    affine_init_needle_frame_wrt_tissue_.translation() = O_needle_wrt_tissue_;
    //ROS_INFO("INIT: initial affine_needle_frame_wrt_tissue_");
    //print_affine(affine_init_needle_frame_wrt_tissue_);      
   
    //next two transforms are fixed during needle driving; they
    // describe how the needle is held by the gripper
    
    //grasp frame wrt gripper frame--trivial (by default)--a simple translation along -z
    Eigen::Matrix3d R;
    R << 1, 0, 0,
     0, 1, 0,
     0, 0, 1; //identity; grasp frame is parallel to gripper frame, just offset in z
    affine_grasp_frame_wrt_gripper_frame_.linear() = R; // grasp frame aligned with gripper frame
    Eigen::Vector3d O_grasp_frame;
    O_grasp_frame<<0, 0, -grasp_depth_;
    affine_grasp_frame_wrt_gripper_frame_.translation() = O_grasp_frame;
    ROS_INFO("FIXED: affine_grasp_frame_wrt_gripper_frame_");
    print_affine(affine_grasp_frame_wrt_gripper_frame_); 
    
    // decide how to grab the needle, w/rt grasp frame; (changeable via "set" fnc)
    //default choice: needle origin in +y half space
    grab_needle_plus_minus_y_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y; 
    //and needle z-axis parallel to gripper z axis:
    grab_needle_plus_minus_z_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z;
    // with bvec_needle = +/- bvec_gripper, needle origin is either on +y-axis or -y-axis of grasp frame
 
    compute_grasp_transform();
    R0_N_wrt_G_ = affine_needle_frame_wrt_gripper_frame_.linear(); 
    O0_N_wrt_G_ = affine_needle_frame_wrt_gripper_frame_.translation();
      
};

void NeedlePlanner::compute_grasp_transform() {
    ROS_INFO("computing grasp transform: ");
   // with bvec_needle = +/- bvec_gripper, needle origin is either on +y-axis or -y-axis of grasp frame
    O_needle_frame_wrt_grasp_frame_<<0,grab_needle_plus_minus_y_*needle_radius_,0; 
    bvec_needle_wrt_grasp_frame_<<0,0,grab_needle_plus_minus_z_; // DEFAULT: needle z axis is parallel to gripper z axis
    
    //--> rot gripper about +z gripper axis to insert needle
    // compute the needle x-axis relative to the gripper;
    // needle x-axis is from needle origin to needle tip
    // handle the 4 default case here:
    // gripper and needle z axes are parallel or antiparallel (sign of grab_needle_plus_minus_z_)
    // needle origin is in + or - y half space of gripper frame (sign of grab_needle_plus_minus_y_)
    // grab the needle at phi_grab_, defined as 0 at center of needle, +pi/2 at tail of needle
    nvec_needle_wrt_grasp_frame_(0) = grab_needle_plus_minus_z_*grab_needle_plus_minus_y_*cos(phi_grab_);
    nvec_needle_wrt_grasp_frame_(1) = grab_needle_plus_minus_y_*sin(phi_grab_);
    nvec_needle_wrt_grasp_frame_(2) = 0.0;
    tvec_needle_wrt_grasp_frame_ = bvec_needle_wrt_grasp_frame_.cross(nvec_needle_wrt_grasp_frame_);
    
    R_needle_frame_wrt_grasp_frame_.col(0) = nvec_needle_wrt_grasp_frame_;
    R_needle_frame_wrt_grasp_frame_.col(1) = tvec_needle_wrt_grasp_frame_;
    R_needle_frame_wrt_grasp_frame_.col(2) = bvec_needle_wrt_grasp_frame_;
    affine_needle_frame_wrt_grasp_frame_.linear() = R_needle_frame_wrt_grasp_frame_; 
    affine_needle_frame_wrt_grasp_frame_.translation() = O_needle_frame_wrt_grasp_frame_;
    if(debug_needle_print) ROS_INFO("FIXED: affine_needle_frame_wrt_grasp_frame_");
    if(debug_needle_print) print_affine(affine_needle_frame_wrt_grasp_frame_);    
    affine_needle_frame_wrt_gripper_frame_ = affine_grasp_frame_wrt_gripper_frame_*affine_needle_frame_wrt_grasp_frame_;
    if(debug_needle_print) ROS_INFO("FIXED: affine_needle_frame_wrt_gripper_frame_");
    if(debug_needle_print) print_affine(affine_needle_frame_wrt_gripper_frame_);      
    
}

//deduce needle grasp transform as follows:
// approx--assume grab needle at "tail" w/ tip of tail in middle of gripper jaws (not practical,
//  but simple to visualize and compute)
// assume nominal grasp pose is: O_N/G = origin of needle frame w/rt gripper frame;
// R_N/G = orientation of needle frame w/rt gripper frame
// nominal start pose, phi_x=0, phi_y=0:  O_N/G = [0; r; -grasp_depth_]
// bvec_N/G = [0;0;1]: needle drive axis is parallel to gripper jaws, bvec_gripper
// xvec_N/G = [0;1;0]: vector from needle origin to needle tip is aligned with tvec_gripper,
// which points in jaw "pinch" direction
// want to allow two rotations of needle, but tvec_Needle should remain in the gripper x-z plane
// (i.e., should have no gripper-y component)
// apply phi_x, 0->2pi (though 270deg would present interference w/ needle tip -->
// rotate needle about gripper x-axis: R_N/G = Rotx(phi_x)*R0_N/G
// then rotate needle about gripper y-axis: R_N/G = Roty(phi_y)*

void  NeedlePlanner::compute_grasp_transform(double phi_x,double phi_y) {
    Eigen::Matrix3d R_N_wrt_G,Rx,Ry;
    Eigen::Vector3d O_N_wrt_G;
    Rx =  Rotx(phi_x);
    Ry = Roty(phi_y);
    R_N_wrt_G = Ry*Rx*R0_N_wrt_G_;
    affine_needle_frame_wrt_gripper_frame_.linear() = R_N_wrt_G;
    O_N_wrt_G = R_N_wrt_G*O0_N_wrt_G_;
    affine_needle_frame_wrt_gripper_frame_.translation() = O_N_wrt_G;
    if(debug_needle_print) ROS_INFO("FIXED: affine_needle_frame_wrt_gripper_frame_");
    if(debug_needle_print) print_affine(affine_needle_frame_wrt_gripper_frame_);  
}

//function to compute affine_tissue_frame_wrt_camera_frame_
//must specify entrance_pt as vector w/rt camera frame
// must specify tissue normal vector, also in camera frame
// also provide approximate exit point on tissue, in camera frame
// Exit point will get "repaired" to be consistent distance from entrance point, 
// consistent w/ needle radius and height of needle z-axis above tissue
// tissue-frame x-axis will be inferred as vector from entrance point to "repaired" exit pt 
void NeedlePlanner::compute_tissue_frame_wrt_camera(Eigen::Vector3d entrance_pt,
        Eigen::Vector3d exit_pt, Eigen::Vector3d tissue_normal)  {
    //set up tissue frame w/rt camera
    bvec_tissue_frame_wrt_camera_ = tissue_normal;
    nvec_tissue_frame_wrt_camera_ = (exit_pt - entrance_pt);
    //normalize this vector:
    double nvec_norm = nvec_tissue_frame_wrt_camera_.norm();
    if (nvec_norm< 0.001) {
        ROS_WARN("specified entrance and exit points are within 1mm; no path will be planned");
        return;
    }
    nvec_tissue_frame_wrt_camera_ = nvec_tissue_frame_wrt_camera_/nvec_norm;
    tvec_tissue_frame_wrt_camera_ = bvec_tissue_frame_wrt_camera_.cross(nvec_tissue_frame_wrt_camera_);
    repaired_exit_pt_ = entrance_pt + nvec_tissue_frame_wrt_camera_*dist_entrance_to_exit_;
    R_tissue_frame_wrt_camera_frame_.col(0)=nvec_tissue_frame_wrt_camera_;
    R_tissue_frame_wrt_camera_frame_.col(1)=tvec_tissue_frame_wrt_camera_;
    R_tissue_frame_wrt_camera_frame_.col(2)=bvec_tissue_frame_wrt_camera_;
    affine_tissue_frame_wrt_camera_frame_.linear() = R_tissue_frame_wrt_camera_frame_;
    affine_tissue_frame_wrt_camera_frame_.translation() = entrance_pt;
    if(debug_needle_print) cout<<"FIXED: affine_tissue_frame_wrt_camera_frame_"<<endl;
    if(debug_needle_print) print_affine(affine_tissue_frame_wrt_camera_frame_);    
    
}

//main fnc: 
// Return a vector full of affines describing desired gripper frames w/rt camera frame
void NeedlePlanner::compute_needle_drive_gripper_affines(vector <Eigen::Affine3d> &gripper_affines_wrt_camera) {
    

    //must first compute the tissue frame and its transform w/rt camera frame
    // by calling: compute_tissue_frame_wrt_camera
    
    //next, establish the initial needle frame w/rt tissue frame:
    //per constructor, have a default initial needle frame w/rt tissue frame
    // override this, if desired, via fnc 
    //rotate needle frame about needle z-axis;   
    //sample this path in angular increments dphi
    //given R_needle_frame_wrt_tissue_frame, the needle z-axis IS anti-parallel to the
    //tissue-frame y-axis: bvec_needle_wrt_tissue_frame_<<0,-1,0;
    // take the needle-frame R_needle_wrt_tissue, and rotate it about the tissue-frame y-axis
    // keep the needle origin constant
    
   //next two are variable, as the needle is inserted:
    //initialize consistent insertion angle and initial pose of needle w/rt tissue;
    // these values will change during needle driving:
    phi_insertion_ = 0.0; //start drive from here   
    affine_needle_frame_wrt_tissue_ = affine_init_needle_frame_wrt_tissue_;     
    
    //version for 90-deg drive only ********
    double dphi = M_PI/(2.0*(NSAMPS_DRIVE_PLAN-1));  //M_PI/(NSAMPS_DRIVE_PLAN-1);
    Eigen::Vector3d kvec_needle;
    //kvec_needle = affine_needle_frame_wrt_tissue_.linear().col(2); //z-axis of needle frame
    
    Eigen::Matrix3d Rot_needle;
    Eigen::Matrix3d R0_needle_wrt_tissue_ = affine_needle_frame_wrt_tissue_.linear(); //update this, in case user changed init needle pose
    //rotate the needle about the tissue-frame x-axis to tilt the needle bvec:
    affine_needle_frame_wrt_tissue_.linear() = Rotx(psi_needle_axis_tilt_wrt_tissue_)*R0_needle_wrt_tissue_;
    R0_needle_wrt_tissue_= affine_needle_frame_wrt_tissue_.linear();
    kvec_needle = affine_needle_frame_wrt_tissue_.linear().col(2);
    if(debug_needle_print) cout<<"kvec_needle="<<kvec_needle.transpose()<<endl;
    if(debug_needle_print) cout<<"R0 needle:"<<endl;
    
    Eigen::Vector3d needle_origin;
    needle_origin = affine_needle_frame_wrt_tissue_.translation();
    affine_needle_frame_wrt_tissue_.translation() = Rotx(psi_needle_axis_tilt_wrt_tissue_)*needle_origin;
    if(debug_needle_print) cout<<affine_needle_frame_wrt_tissue_.linear()<<endl;

    for (int ipose=0;ipose<NSAMPS_DRIVE_PLAN;ipose++) {
        //Roty_needle = Roty(-phi_insertion_); //rotate about tissue-frame -y axis
        //more general--allow any needle z axis:
        Rot_needle = Rot_k_phi(kvec_needle,phi_insertion_);
        //R_needle_wrt_tissue_ = Roty_needle*R0_needle_wrt_tissue_; //update rotation of needle drive
        R_needle_wrt_tissue_ = Rot_needle*R0_needle_wrt_tissue_; //update rotation of needle drive
        
        if(debug_needle_print) cout<<"R_needle w/rt tissue:"<<endl;
        if(debug_needle_print) cout<<R_needle_wrt_tissue_<<endl;
        //need to check these transforms...
        affine_needle_frame_wrt_tissue_.linear() = R_needle_wrt_tissue_;
        //ROS_INFO("affine_needle_frame_wrt_tissue_");
        //print_affine(affine_needle_frame_wrt_tissue_);
        
        affine_gripper_frame_wrt_tissue_ = 
                 affine_needle_frame_wrt_tissue_*affine_needle_frame_wrt_gripper_frame_.inverse();
        if(debug_needle_print) ROS_INFO("affine_gripper_frame_wrt_tissue_");
        if(debug_needle_print) print_affine(affine_gripper_frame_wrt_tissue_);
        
        //affine_needle_frame_wrt_camera_ = affine_tissue_frame_wrt_camera_frame_.inverse()*affine_needle_frame_wrt_tissue_;  
        //ROS_INFO("affine_needle_frame_wrt_camera_");
        //print_affine(affine_needle_frame_wrt_camera_); 
        
        affine_gripper_frame_wrt_camera_frame_ = 
                affine_tissue_frame_wrt_camera_frame_*affine_gripper_frame_wrt_tissue_;


        if(debug_needle_print) ROS_INFO("affine_gripper_frame_wrt_camera_frame_");
        if(debug_needle_print) print_affine(affine_gripper_frame_wrt_camera_frame_);        
        gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
        
                phi_insertion_+=dphi; 
    }
    //compute gripper from from needle frame, in camera coords
    //push this frame on the vector gripper_affines_wrt_camera
    // repeat for angles phi_insertion_ from 0 to pi
}
Eigen::Matrix3d NeedlePlanner::Rotx(double phi) {
    Eigen::Matrix3d Rx;
    Rx(0,0) = 1.0; //
    Rx(0,1) = -sin(phi);
    Rx(0,2) = 0.0;
    Rx(1,0) = 0.0;
    Rx(1,1) = cos(phi);
    Rx(1,2) = -sin(phi);
    Rx(2,0) = 0.0;
    Rx(2,1) = sin(phi);
    Rx(2,2) = cos(phi);
    if(debug_needle_print) cout<<"Rotx:"<<endl;
    if(debug_needle_print) cout<<Rx<<endl;
    return Rx;    
}

Eigen::Matrix3d NeedlePlanner::Rotz(double phi) {
    Eigen::Matrix3d Rz;
    Rz(0,0) = cos(phi);
    Rz(0,1) = -sin(phi);
    Rz(0,2) = 0.0;
    Rz(1,0) = sin(phi);
    Rz(1,1) = cos(phi);
    Rz(1,2) = 0.0;
    Rz(2,0) = 0.0;
    Rz(2,1) = 0.0;
    Rz(2,2) = 1.0;
    if(debug_needle_print) cout<<"Rotz:"<<endl;
    if(debug_needle_print) cout<<Rz<<endl;
    return Rz;
}

Eigen::Matrix3d NeedlePlanner::Roty(double phi) {
    Eigen::Matrix3d Roty;
    Roty(0,0) = cos(phi);
    Roty(0,1) = 0;
    Roty(0,2) = sin(phi);
    Roty(1,0) = 0;
    Roty(1,1) = 1.0;
    Roty(1,2) = 0.0;
    Roty(2,0) = -sin(phi);
    Roty(2,1) = 0.0;
    Roty(2,2) = cos(phi);
    //cout<<"Roty:"<<endl;
    //cout<<Roty<<endl;
    return Roty;
}

//fnc to compute a rotation matrix angle phi about vector k_vec
Eigen::Matrix3d NeedlePlanner::Rot_k_phi(Eigen::Vector3d k_vec,double phi) {
    Eigen::Matrix3d R_k_phi;
    double kx = k_vec(0);
    double ky = k_vec(1);
    double kz = k_vec(2);
    R_k_phi(0,0) = kx*kx*vers(phi)+cos(phi);
    R_k_phi(0,1) = ky*kx*vers(phi)-kz*sin(phi);
    R_k_phi(1,0) = R_k_phi(0,1);
    R_k_phi(0,2) = kz*kx*vers(phi)+ky*sin(phi);
    R_k_phi(2,0) = R_k_phi(0,2);
    R_k_phi(1,1) = ky*ky*vers(phi)+cos(phi);
    R_k_phi(1,2) = kz*ky*vers(phi)-kx*sin(phi);
    R_k_phi(2,1) = R_k_phi(1,2);
    R_k_phi(2,2) = kz*kz*vers(phi)+cos(phi);
    return R_k_phi;
}

void NeedlePlanner::print_affine(Eigen::Affine3d affine) {
    cout<<"Rotation: "<<endl;
        cout<<affine.linear()<<endl;
        cout<<"origin: "<<affine.translation().transpose()<<endl;
    
}

//this playfile writer example is specialized for needle drive w/ PSM1
void NeedlePlanner::write_needle_drive_affines_to_file(vector <Eigen::Affine3d> &gripper_affines_wrt_camera) {
  ofstream outfile;
  outfile.open ("gripper_poses_in_camera_coords.csp");
  //the following are w/rt to left camera optical frame
//entries 0-2 = origin of gripper tip (a point mid-way between the gripper jaw tips)
//entries 3-5 = gripper x-axis direction (x-axis points parallel to gripper-jaw rotation axis; 
  //y-axis points from fingertip to fingertip)
//entries 6-8 = z-axis direction for gripper-tip frame (z-axis points from wrist to tip)
   int nposes = gripper_affines_wrt_camera.size();
   if (nposes<1) {
       ROS_WARN("NO POSES TO SAVE");
       return;
   }
    ROS_INFO("saving computed %d gripper poses w/rt camera",nposes);
    Eigen::Affine3d affine_pose;
    
    Eigen::Vector3d Origin,Origin1_last,Origin2_start,Origin2_last;
    Eigen::Vector3d Origin2_offset1,Origin2_offset2;
    Eigen::Matrix3d R,R1_last,R2;
    Eigen::Vector3d nvec,bvec,nvec1_last,bvec1_last;
    Eigen::Vector3d nvec_needle,bvec_needle,origin_needle, tip_of_needle;
    Eigen::Vector3d gripper2_grasp_origin,gripper2_pre_grasp_origin;
    Eigen::Vector3d nvec_gripper2,tvec_gripper2,bvec_gripper2;
    //pose of gripper1 at end of needle drive:
    Eigen::Affine3d affine_gripper1_frame_wrt_camera_last;
    Eigen::Affine3d affine_needle_frame_wrt_camera_last;
    
    //pose of gripper1 at end of needle drive 
    affine_gripper1_frame_wrt_camera_last = gripper_affines_wrt_camera[nposes-1];
    //T_n/c = T_g/c*T_n/g :  compute needle pose at end of needle drive w/rt camera frame
    affine_needle_frame_wrt_camera_last = affine_gripper1_frame_wrt_camera_last*
               affine_needle_frame_wrt_gripper_frame_;
    // get coordinates of needle tip:
    nvec_needle = affine_needle_frame_wrt_camera_last.linear().col(0);
    bvec_needle = affine_needle_frame_wrt_camera_last.linear().col(2);
    origin_needle = affine_needle_frame_wrt_camera_last.translation();
    tip_of_needle = origin_needle+needle_radius_*nvec_needle;
    
    //define gripper2 goal as: origin of gripper tip + grasp_depth_ forward along bvec_needle 
    //puts tip in center of claw--should leave more grasp length along needle body--fix later
    gripper2_grasp_origin = tip_of_needle + grasp_depth_*bvec_needle; 
    //compute approach pose, back up along gripper z-axis to make room for gripper 1 needle drive
    gripper2_pre_grasp_origin = gripper2_grasp_origin-0.03*bvec_needle; 
    //specify the gripper-2 orientation:
    bvec_gripper2 = bvec_needle; //grasp needle w/ gripper z-axis parallel to needle z-axis 
    tvec_gripper2 = nvec_needle; //gripper y-axis is "pinch" direction--align this with needle x-axis
    nvec_gripper2 = tvec_gripper2.cross(bvec_gripper2);
    R2.col(2) =  bvec_gripper2; 
    R2.col(1) =  tvec_gripper2; 
    R2.col(0) = nvec_gripper2; 
    
    
    double t=4;
    double dt= 1.0; //time step between poses
    for (int i=0;i<nposes;i++) {
        Origin = gripper_affines_wrt_camera[i].translation();
        outfile<<Origin(0)<<", "<<Origin(1)<<", "<<Origin(2)<<",     ";
        R = gripper_affines_wrt_camera[i].linear();
        nvec= R.col(0);
        bvec= R.col(2);
        outfile<<nvec(0)<<", "<<nvec(1)<<", "<<nvec(2)<<",     ";
        outfile<<bvec(0)<<", "<<bvec(1)<<", "<<bvec(2)<<",  0,   ";
        //now, gripper 2:
        outfile<<gripper2_pre_grasp_origin(0)<<", "<<gripper2_pre_grasp_origin(1)<<", "<<gripper2_pre_grasp_origin(2)<<",     ";
        outfile<<nvec_gripper2(0)<<", "<<nvec_gripper2(1)<<", "<<nvec_gripper2(2)<<",     ";
        outfile<<bvec_gripper2(0)<<", "<<bvec_gripper2(1)<<", "<<bvec_gripper2(2)<<",  0.5,   "<<t<<endl;  
        t+=dt;
    }  
    t+= 2.0; // move gripper 2 to grasp pose:
        // keep gripper 1 still:
        Origin = gripper_affines_wrt_camera[nposes-1].translation();
        outfile<<Origin(0)<<", "<<Origin(1)<<", "<<Origin(2)<<",     ";
        R = gripper_affines_wrt_camera[nposes-1].linear();
        nvec= R.col(0);
        bvec= R.col(2);
        outfile<<nvec(0)<<", "<<nvec(1)<<", "<<nvec(2)<<",     ";
        outfile<<bvec(0)<<", "<<bvec(1)<<", "<<bvec(2)<<",  0,   ";
        //move gripper 2 forward:
        outfile<<gripper2_grasp_origin(0)<<", "<<gripper2_grasp_origin(1)<<", "<<gripper2_grasp_origin(2)<<",     ";
        outfile<<nvec_gripper2(0)<<", "<<nvec_gripper2(1)<<", "<<nvec_gripper2(2)<<",     ";
        outfile<<bvec_gripper2(0)<<", "<<bvec_gripper2(1)<<", "<<bvec_gripper2(2)<<",  0.5,   "<<t<<endl;      
    t+= 2.0; // close gripper 2:
        Origin = gripper_affines_wrt_camera[nposes-1].translation();
        outfile<<Origin(0)<<", "<<Origin(1)<<", "<<Origin(2)<<",     ";
        R = gripper_affines_wrt_camera[nposes-1].linear();
        nvec= R.col(0);
        bvec= R.col(2);
        outfile<<nvec(0)<<", "<<nvec(1)<<", "<<nvec(2)<<",     ";
        outfile<<bvec(0)<<", "<<bvec(1)<<", "<<bvec(2)<<",  0,   ";
        //move gripper 2 forward:
        outfile<<gripper2_grasp_origin(0)<<", "<<gripper2_grasp_origin(1)<<", "<<gripper2_grasp_origin(2)<<",     ";
        outfile<<nvec_gripper2(0)<<", "<<nvec_gripper2(1)<<", "<<nvec_gripper2(2)<<",     ";
        outfile<<bvec_gripper2(0)<<", "<<bvec_gripper2(1)<<", "<<bvec_gripper2(2)<<",  0.0,   "<<t<<endl; 
        
  outfile.close();
  ROS_INFO("wrote gripper motion plan to file gripper_poses_in_camera_coords.csp");
}


