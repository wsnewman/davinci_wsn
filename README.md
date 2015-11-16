# davinci_wsn
wsn experimental code for DaVinci robot

Startup rviz with: 
`roslaunch dvrk_model wsn_psm_one_rviz.launch`

or for dual arms:
`roslaunch dvrk_model wsn_psm_both_rviz.launch`

start the joint-space interpolator action server:
`rosrun davinci_traj_streamer traj_interpolator_as`

source in: /src/traj_interpolator_as.cpp

then talk to the action server via a client, e.g.:
`rosrun davinci_traj_streamer davinci_traj_action_client_pre_pose` 

This will cause both arms to move together.

traj_interpolator_as action server expects to receive trajectories, type trajectory_msgs::JointTrajectory, in goal specification,
where each trajectory point (trajectory_msgs::JointTrajectoryPoint) specifies 14 joint angles: the first 7 for arm 1, and the next
7 for arm 2.  traj_interpolator_as will interpolate these 14-D joint-space subgoals at specified dt (0.01 sec) and publish increments
at this rate.

At present, these publications are to: /dvrk_psm/joint_states, which drives rviz.  However, rviz contains multiple "mimic" joints
that are dependent (e.g. 4-bar linkages).  Expect that actual system (and Gazebo model) should have only 7 actuator commands per PSM
(Patient-Side Manipulator).

See also davinci_playfiles for a start on reading joint-space trajectories from CSV files (to be merged with equivalent of
davinci_traj_action_pre_pose).  

Did an IK test with cartesian path as well.
Made the gripper tip sweep out a circular path while holding the gripper orientation constant. (see davinci_kinematics cartesian-move code)

Looks encouraging.  Demonstrated Cartesian-space trajectories in rviz--including dual-arm motions.
See libraries in davinci_kinematics for fw, ik and joint publishing.

Still need to find and install actual joint limits.
Need to test assumption that legal IK solutions are unique.

Update Sept 13, 2015:

created Gazebo extensions of urdf models.  Start with:
`roslaunch dvrk_model wsn_davinci_gazebo.launch`

Then start trajectory streamer node (for both arms and both tools):
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`
Note: the above has been recompiled using dual-arm streamer code

Test: cd to /ros_ws/src/davinci_wsn/davinci_playfiles  and run:
`rosrun playfile_reader playfile_cartspace testfile2.csp`

Can move a checkerboard under simulated cameras.  First, get arms out of the way.  From same dir as above, run: 
`rosrun playfile_reader playfile_cartspace retract.csp`

Then:
`rosrun move_gazebo_model move_calibration_checkerboard`

This will cause dual arms to move in a Gazebo simulation.
demonstrated could knock can off block using arms, but self collision checking is not working.
Also, gripper not able to hold objects.

Extensions to manipulation:

start gazebo: 

`roslaunch dvrk_model wsn_davinci_gazebo.launch`

start trajectory server:

`rosrun davinci_traj_streamer davinci_traj_interpolator_as`

move robot to retracted pose w/ grippers open (from /ros_ws/src/davinci_wsn/davinci_playfiles):

`rosrun playfile_reader playfile_cartspace retract.csp`

The world model includes a 1mm-thickness peg.  If desired, this peg model can be inserted manually via Gazebo.  To do so, from the "Insert" tab, choose model 1mm_peg (stored in .gazebo).  This model needs to be stored in the /home/.gazebo directory.  If not present, see the 1mm_bar.tar.gz file in this repo, and store this in your .gazebo dir.

reduce gravity:  From gazebo, "world" tab, "physics" item, select "gravity" and set gravity components to 0.  This will reduce the "gravity droop" of the motor controllers.

Move peg to place it between open fingers:

`rosrun move_gazebo_model move_1mm_bar`

 (see source code in cwru_baxter/move_gazebo_model/src/move_gazebo_model2.cpp).  This will prompt for the model name (enter 1mm_bar)
 
 At zero gravity, this program can be killed and the peg will remain between the gripper fingers.

Close the gripper (still from playfiles dir):

`rosrun playfile_reader playfile_cartspace retracted_close_grippers.csp`

* Peg hand-off in camera space:  note that in camera space, x is left/right and y is front/back

From the directory davinci_playfiles, run the routine:

`rosrun playfile_reader playfile_cameraspace cameraspace_peg_handoff_prepose.csp`

start up rviz:

`rosrun rviz rviz`

and move the view to zoom on the grippers from the viewpoint of camera: davinci/left_camera/image_raw

This will pre-pose the grippers, facing each other, with the "right" (from DaVinci perspective) gripper open.
(This appears as the "left" gripper from the camera view, similar to the teleoperation view)

Reduce the simulated gravity to a small value (e.g. gz = -0.1)

Pre-position the "needle" between the right-arm gripper jaws by running:

`rosrun move_gazebo_model move_1mm_bar_camera_init`

Close the gripper jaws to grab the needle by running (from the davinci_playfiles directory):

`rosrun playfile_reader playfile_cameraspace cameraspace_peg_handoff_prepose_grab.csp`

Kill the needle positioner node, move_1mm_bar_camera_init.  The needle should be grasped by the gripper.

Then run:

`rosrun playfile_reader playfile_cameraspace cameraspace_peg_handoff.csp`

The needle will be handed off to the opposite gripper.











