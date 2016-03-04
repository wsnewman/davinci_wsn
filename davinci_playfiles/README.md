# davinci_playfiles
ROS package just to hold playfiles

## Example usage
start davinci in gazebo:
`roslaunch dvrk_model wsn_davinci_gazebo.launch`
start the trajectory action server:
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`
change to this directory:
`roscd davinci_playfiles`
retract the grippers to a viable pose:
`rosrun playfile_reader playfile_cartspace retract.csp`
3 types of playfiles: jointspace, e.g.:
`rosrun playfile_reader playfile_jointspace test_jnt_lims.jsp`
Or move in Cartesian space, w/rt base frame of each manipulator (inconvenient)
`rosrun playfile_reader playfile_cartspace xxx.csp` (file just express gripper frames in respective base frames)
Or, most useful, move in camera space, e.g.:
`rosrun playfile_reader playfile_cameraspace gripper_poses_in_camera_coords.csp`
(*.csp files express desired gripper frames w/rt left-camera optical frame)

    
