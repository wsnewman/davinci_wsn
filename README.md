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


