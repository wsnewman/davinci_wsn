# needle_planner

Your description goes here

## Example usage

## Running tests/demos
start gazebo: 

`roslaunch dvrk_model wsn_davinci_gazebo.launch`

start trajectory server:

`rosrun davinci_traj_streamer davinci_traj_interpolator_as`

navigate to the "davinci_playfiles" directory:
`roscd davinci_playfiles`

Move the robot to retract the grippers to a reasonable starting position:
`rosrun playfile_reader playfile_cartspace retract.csp`
or
`rosrun playfile_reader playfile_jointspace home.jsp`

Generate a needle-drive plan:
`rosrun needle_planner needle_planner_test_main`
The file "gripper_poses_in_camera_coords.csp" will be created in the current directory

Try running the needle-drive plan:
`rosrun playfile_reader playfile_cameraspace gripper_poses_in_camera_coords.csp`

## needle planner example:
start davinci (or at least a roscore)
`rosrun needle_planner needle_planner_kvec_horiz_test_main_v3`
publish an entry point, e.g.:
`rostopic pub  /thePoint geometry_msgs/Point  '{x: 0, y: 0, z: 0.12}' `
(OR run publish selected points node via rviz w/ tissue surface w/rt DaVinci)
Observe the result with:
rostopic echo exit_points

 

