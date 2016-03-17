/* 
 * File:   needle_planner_test_main_v3.cpp
 * Author: wsn
 *
 * Created March 4, 2016
 */
//this version: loop through options to find a good needle-drive approach

//try to optimize needle driving w/rt needle grasp pose
// start w/ desired needle path has z-axis parallel to tissue;
// also specify entrance/exit/needle ht, or equiv, height and orientation of bvec_needle;
// needle grasp transform: T_needle_wrt_gripper:
// require grasp needle near needle tail, e.g. contact pts on needle at ~ [-r,0,0]/needle frame
// or, from grasp pt, origin of needle frame is at [r,0,0], measured along needle-frame axes
// Need R_N/G (orientation of needle frame w/rt gripper frame)
// Start w/ needle-frame s.t. O_N/G: nvec_N_wrt_G = [0,1,0], i.e. needle x-axis is || pinch direc
//  bvec_N_wrt_G = [0,0,1], i.e. needle drive axis (z-axis) is || gripper z-axis
//  tvec_N follows
//  O_N/G (origin of needle w/rt gripper) = R_N/G*[r;0;0]
//  from R0_N/G, rotate z-axis of needle about gripper-frame x-axis by phi_x
//  then rotate needle about gripper-frame y-axis by phi_y

#include <needle_planner/needle_planner.h>
#include <davinci_kinematics/davinci_kinematics.h>

//example use of needle-planner library
bool test_debug = false;

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
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

int runlength_ones(vector <int> valid_ik_samps) {
    int npts = valid_ik_samps.size();
    int n_ones = 0;
    int n_ones_next = 0;
    for (int i=0;i<npts;i++)
        if (valid_ik_samps[i]) n_ones++;
    if (n_ones==0) return 0;
    //OK--there is at least one run of 1's; score it
    int i_start=0;
    while(valid_ik_samps[i_start]==0) i_start++; 
    //found a start; count 1's from here
    n_ones=0;
    while((valid_ik_samps[i_start]==1)&&(i_start<npts)) 
    {  n_ones++;
       i_start++;
       //ROS_INFO("n_ones, i_start = %d, %d",n_ones,i_start);
    }
    //cout<<"score for first string: "<<n_ones<<endl;
    
    // see if there is another run:
    while (i_start<npts) {
        //search for start of next string:
        while((valid_ik_samps[i_start]==0)&&(i_start<npts)) i_start++; 
        if (i_start>=npts) return n_ones; //did not find another string of 1's
        //if here, found start of another string of 1's; score it
        //cout<<"found a second string"<<endl;
        n_ones_next=0;
        while((valid_ik_samps[i_start]==1)&&(i_start<npts)) {
            i_start++;
            n_ones_next++;
        }
        //cout<<"next string score = "<<n_ones_next<<endl;
        //done w/ next string; is it a winner?
        if (n_ones_next>n_ones) n_ones = n_ones_next;
        
    }
    return n_ones;
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "needle_planner_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Eigen::Vector3d entrance_pt, exit_pt, tissue_normal;
    tissue_normal << 0, 0, -1; //antiparallel to optical axis
    entrance_pt << -0.1, 0.05, 0.1; //100mm under camera; slightly forward, to avoid jnt lims should be OK
    exit_pt << -0.09, 0.05, 0.1; // exit pt is shifted along camera-frame +x axis relative to entrance pt
    vector <Eigen::Affine3d> gripper_affines_wrt_camera; //put answers here 
    vector <Eigen::Affine3d> vetted_gripper_affines_wrt_camera;

    ROS_INFO("instantiating  forward solver and an ik_solver");
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver;
    
    ofstream outfile;    
    outfile.open ("best_poses.dat");

    ROS_INFO("main: instantiating an object of type NeedlePlanner");
    NeedlePlanner needlePlanner;


    //compute the tissue frame in camera coords, based on point-cloud selections:
    needlePlanner.compute_tissue_frame_wrt_camera(entrance_pt, exit_pt, tissue_normal);
    //optional: manually set the needle grasp pose, else accept default from constructor
    //needlePlanner.set_affine_needle_frame_wrt_gripper_frame(affine);
    //optional: set the initial needle frame w/rt tissue frame (or accept default in constructor)
    //needlePlanner.set_affine_needle_frame_wrt_tissue(Eigen::Affine3d affine)
    // given grasp transform, tissue transform and initial needle pose w/rt tissue,
    // compute needle-drive path as rotation about needle-z axis:

    ROS_INFO("getting transforms from camera to PSMs");
    tf::TransformListener tfListener;
    tf::StampedTransform tfResult_one, tfResult_two;
    Eigen::Affine3d affine_lcamera_to_psm_one, affine_lcamera_to_psm_two, affine_gripper_wrt_base;
    bool tferr = true;
    int ntries = 0;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        if (ntries > 5) break; //give up and accept default after this many tries
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("left_camera_optical_frame", "one_psm_base_link", ros::Time(0), tfResult_one);
            tfListener.lookupTransform("left_camera_optical_frame", "two_psm_base_link", ros::Time(0), tfResult_two);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
        }
    }
    //default transform: need to match this up to camera calibration!
    if (tferr) {
        affine_lcamera_to_psm_one.translation() << -0.155, -0.03265, 0.0;
        Eigen::Vector3d nvec, tvec, bvec;
        nvec << -1, 0, 0;
        tvec << 0, 1, 0;
        bvec << 0, 0, -1;
        Eigen::Matrix3d R;
        R.col(0) = nvec;
        R.col(1) = tvec;
        R.col(2) = bvec;
        affine_lcamera_to_psm_one.linear() = R;
        affine_lcamera_to_psm_two.linear() = R;
        affine_lcamera_to_psm_two.translation() << 0.145, -0.03265, 0.0;
        ROS_WARN("using default transform");
    } else {

        ROS_INFO("tf is good");

        //affine_lcamera_to_psm_one is the position/orientation of psm1 base frame w/rt left camera link frame
        // need to extend this to camera optical frame
        affine_lcamera_to_psm_one = transformTFToEigen(tfResult_one);
        affine_lcamera_to_psm_two = transformTFToEigen(tfResult_two);
    }
    ROS_INFO("transform from left camera to psm one:");
    cout << affine_lcamera_to_psm_one.linear() << endl;
    cout << affine_lcamera_to_psm_one.translation().transpose() << endl;
    ROS_INFO("transform from left camera to psm two:");
    cout << affine_lcamera_to_psm_two.linear() << endl;
    cout << affine_lcamera_to_psm_two.translation().transpose() << endl;

    //try grabbing needle w/ bvec_needle = -bvec_gripper
    //needlePlanner.set_grab_needle_plus_minus_z(GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z);
    //needlePlanner.compute_grasp_transform();

    double ang_to_exit, phi_x, phi_y, tilt;
    Eigen::Vector3d nvec_tissue;
    vector <int> valid_ik_samps;
    double best_phi_x = 0.0;
    double best_phi_y = 0.0;
    double best_tilt = 0.0;
    int best_drive_score = 0;
    int drive_score = 0;
    int best_run = 0;
    
    //while (ros::ok()) 
    {
        cout << "entrance point at: " << entrance_pt.transpose() << endl;
        cout << "enter angle to exit point (0-2pi): ";
        cin >> ang_to_exit;

        //cout << "enter needle-grasp phi_x: ";

        //cin >> phi_x;
        for (phi_x = 0.0; phi_x < 6.28; phi_x += 0.1) {
            for (phi_y = 0.0; phi_y < 6.28; phi_y += 0.1) {
                ROS_INFO("phi_x, phi_y = %f, %f",phi_x,phi_y);
                for (tilt = -1.2; tilt < 1.21; tilt += 0.1) {
                    drive_score = 0;

                    //cout << "enter needle-grasp phi_y: ";
                    //cin >> phi_y;
                    needlePlanner.compute_grasp_transform(phi_x, phi_y);
                    //cout<< "enter tilt of needle z-axis w/rt tissue: ";
                    //cin>> tilt;
                    needlePlanner.set_psi_needle_axis_tilt_wrt_tissue(tilt);

                    //for (ang_to_exit = 0; ang_to_exit < 6.28; ang_to_exit += 1.0) 
                    {
                        //cout << "angle to exit pt on tissue surface: " << ang_to_exit << endl;
                        //cin>>ang_to_exit;
                        nvec_tissue << cos(ang_to_exit), sin(ang_to_exit), 0.0;
                        exit_pt = entrance_pt + nvec_tissue;
                        needlePlanner.compute_tissue_frame_wrt_camera(entrance_pt, exit_pt, tissue_normal);
                        gripper_affines_wrt_camera.clear();
                        vetted_gripper_affines_wrt_camera.clear();

                        needlePlanner.compute_needle_drive_gripper_affines(gripper_affines_wrt_camera);
                        int nposes = gripper_affines_wrt_camera.size();
                        //ROS_INFO("computed %d gripper poses w/rt camera", nposes);
                        Eigen::Affine3d affine_pose, affine_gripper_wrt_base_frame, affine_gripper_wrt_base_fk;
                        Eigen::Vector3d origin_err;
                        Eigen::Matrix3d R_err;
                        Vectorq7x1 q_vec1;
                        q_vec1.resize(7);
                        valid_ik_samps.clear();
                        for (int i = 0; i < nposes; i++) {
                            if (test_debug) ROS_INFO("pose %d", i);
                            affine_pose = gripper_affines_wrt_camera[i];
                            if (test_debug) cout << affine_pose.linear() << endl;
                            if (test_debug) cout << "origin: " << affine_pose.translation().transpose() << endl;
                            affine_gripper_wrt_base_frame = affine_lcamera_to_psm_one.inverse() * affine_pose;
                            if (test_debug) ROS_INFO("pose %d w/rt PSM1 base:", i);
                            if (test_debug) cout << affine_gripper_wrt_base_frame.linear() << endl;
                            if (test_debug) cout << "origin: " << affine_gripper_wrt_base_frame.translation().transpose() << endl;
                            if (ik_solver.ik_solve(affine_gripper_wrt_base_frame)) { //convert desired pose into equiv joint displacements
                                valid_ik_samps.push_back(1);
                                drive_score++;
                                vetted_gripper_affines_wrt_camera.push_back(affine_pose); //accept this one
                                q_vec1 = ik_solver.get_soln();
                                q_vec1(6) = 0.0;
                                if (test_debug) cout << "qvec1: " << q_vec1.transpose() << endl;
                                if (test_debug) ROS_INFO("FK of IK soln: ");
                                affine_gripper_wrt_base_fk = davinci_fwd_solver.fwd_kin_solve(q_vec1);
                                if (test_debug) cout << "affine linear (R): " << endl;
                                if (test_debug) cout << affine_gripper_wrt_base_fk.linear() << endl;
                                if (test_debug) cout << "origin: ";
                                if (test_debug) cout << affine_gripper_wrt_base_fk.translation().transpose() << endl;
                                origin_err = affine_gripper_wrt_base_frame.translation() - affine_gripper_wrt_base_fk.translation();
                                R_err = affine_gripper_wrt_base_frame.linear() - affine_gripper_wrt_base_fk.linear();
                                if (test_debug) ROS_WARN("error test: %f", origin_err.norm());
                                if (test_debug) cout << "IK/FK origin err: " << origin_err.transpose() << endl;
                                if (test_debug) cout << "R err: " << endl;
                                if (test_debug) cout << R_err << endl;
                            } else {
                                if (test_debug) ROS_WARN("GRIPPER 1: NO VALID IK SOLN!!");
                                valid_ik_samps.push_back(0);
                            }
                        } //end loop through needle-drive poses
                        drive_score = runlength_ones(valid_ik_samps);
                        ROS_INFO("drive_score: %d",drive_score);
                    } //end loop ang_to_exit
                    if (drive_score == best_drive_score) {
                        best_drive_score = drive_score;
                        best_phi_x = phi_x;
                        best_phi_y = phi_y;
                        best_tilt = tilt;
                        cout << "valid samples status:" << endl;
                        for (int i = 0; i < valid_ik_samps.size(); i++) {
                            cout << valid_ik_samps[i] << "  ";
                        }
                        cout << endl;
                        ROS_INFO("phi_x, phi_y, tilt = %f, %f, %f",phi_x,phi_y,tilt);                        
                        ROS_WARN("tied best strategy; score = %d; enter 1: ",best_drive_score);

                        int ans;
                        //cin>>ans;
                        //needlePlanner.write_needle_drive_affines_to_file(vetted_gripper_affines_wrt_camera);
                    }                               
                    if (drive_score > best_drive_score) {
                        best_drive_score = drive_score;
                        best_phi_x = phi_x;
                        best_phi_y = phi_y;
                        best_tilt = tilt;
                        cout << "valid samples status:" << endl;
                        for (int i = 0; i < valid_ik_samps.size(); i++) {
                            cout << valid_ik_samps[i] << "  ";
                        }
                        cout << endl;
                        ROS_INFO("phi_x, phi_y, tilt = %f, %f, %f",phi_x,phi_y,tilt);                        
                        ROS_WARN("new best strategy; score = %d; enter 1: ",best_drive_score);

                        int ans;
                        //cin>>ans;
                        needlePlanner.write_needle_drive_affines_to_file(vetted_gripper_affines_wrt_camera);
                    }
                    if (drive_score==21) { // save best solns to file
                        outfile<<phi_x<<", "<<phi_y<<", "<<tilt<<endl;
                    }
         
                }


            }
        }
    }
    //needlePlanner.write_needle_drive_affines_to_file(vetted_gripper_affines_wrt_camera);

    ROS_INFO("best drive score: %d", best_drive_score);
    ROS_INFO("best phi_x, phi_y, tilt = %f, %f, %f", best_phi_x, best_phi_y, best_tilt);
    outfile.close();
    return 0;

}
