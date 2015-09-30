# cart_move_as

Started this package 9/29; trying to make a class that has both an action server and an action client;
Should accept cartesian pose goals, perform IK, package as a joint trajectory, and send this as a goal
to the joint-space interpolator action server for execution.

Compiles, but needs work.
Also need to create an example client for illustration.

## Example usage
Start up:
`roslaunch dvrk_model wsn_davinci_gazebo.launch`
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`
`rosrun cart_move_as cart_move_as`
`rosrun cart_move_as cart_move_ac` (demo action client, to be built)
## Running tests/demos
    
