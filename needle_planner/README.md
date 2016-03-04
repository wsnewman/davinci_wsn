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

Generate a needle-drive plan:
`rosrun needle_planner needle_planner_test_main`
The file "gripper_poses_in_camera_coords.csp" will be created in the current directory

Try running the needle-drive plan:
`rosrun playfile_reader playfile_cameraspace gripper_poses_in_camera_coords.csp`

