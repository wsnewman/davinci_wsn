# playfile_reader

Node to read a CSV file encoding desired dual-arm joint-space motions, convert to a trajectory, and send as a goal request to the trajectory streamer action server.
Specialized for dual-PSM davinci; assumes fixed order:
entries 0-6 correspond to PSM1, joints 1-7; entries 7-13 are joints 1-7 of PSM2; entry 14 is desired arrival time (from start), in seconds.
each line must contain all 15 values (in fixed order), separated by commas

The file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
The file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.

## Example usage
start rviz:
`roslaunch dvrk_model wsn_psm_both_rviz.launch`

start trajectory streamer action server:
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`

cd to a directory containing joint playfiles, in CSV format (e.g., cd to davinci_playfiles).  Run this node with file argument, e.g.
to execute file testfile2.jsp, do:
`rosrun playfile_reader playfile_jointspace testfile2.jsp`

ALTERNATIVE: Cartesian playfile. Read in a file of sequence of desired gripper poses, in Cartesian space.
Perform IK, pack joint-space solutions into a trajectory, and send as goal to trajectory streamer action server:

`rosrun playfile_reader playfile_cartspace testfile.csp`

//files must be designed to account for relative positions of PSM1 and PSM2 base frames w/rt world

//the following are w/rt to frame ONE_PSM_BASE_LINK
//entries 0-2 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
//entries 3-5 = x-axis direction for PSM1 gripper-tip frame (x-axis points parallel to gripper-jaw rotation axis; y-axis points from fingertip to fingertip)
//entries 6-8 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
//entry 9 = angle of jaw opening (in radians)

// repeat next entries for PSM2:
//the following are w/rt to frame TWO_PSM_BASE_LINK
//entries 10-12 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
//entries 13-15 = x-axis direction for PSM1 gripper-tip frame (x-axis points parallel to gripper-jaw rotation axis; y-axis points from fingertip to fingertip)
//entries 16-18 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
//entry 19 = angle of jaw opening (in radians)

//entry 20 = desired arrival time (from start of trajectory, in seconds)

//each line must contain all 21 values (in fixed order), separated by commas

Modified 9/27/15:
rosrun playfile_reader playfile_cameraspace test_cameraspace.csp
This playfile version assumes desired gripper poses are expressed w/rt left camera optical frame;
Uses tflistener to get transform from optical frame to each PSM base frame, then uses same
IK code;

Looks encouraging.  Gets out of reach for z ~>= 0.16 (along optical axis)
DO get significant gravity droop.  Alignment looks good (red dot on gripper tips
align w/ optical z axis) when gravity is reduced.  (Should improve gains and reduce
masses).

Can start up this way:
`roslaunch dvrk_model wsn_davinci_gazebo.launch`
which starts 2 psm's along with stereo cameras, a table and a 1mm blue bar
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`
to get the trajectory interpolator running
And from playfiles directory, retract grippers with:
`rosrun playfile_reader playfile_cartspace retract.csp`
then can run a camera-space playfile with, e.g.:
`rosrun playfile_reader playfile_cameraspace test_cameraspace.csp`
which interprets a camera-space Cartesian file to drive the grippers.
View in rviz to see grippers from /davinci/left_camera/image_raw






    
