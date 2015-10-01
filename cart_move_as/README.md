# cart_move_as

Started this package 9/29; cartesian-move class has both an action server and an action client;
Accepts Cartesian pose goals--expressed in left_camera_optical_frame, performs IK (on both arms), packages as a (dual-arm) joint trajectory, and sends this as a goal
to the joint-space interpolator action server for execution.

cart_move_as remembers that previous (most recent) pose commands and uses these as initial commands (time 0) for next trajectory move.
Upon start-up, the "previous command" is set to the current arm poses, so no jumps will occur with incoming Cartesian goals.

At present, this server accepts only a single (dual-arm) goal at a time and executes each goal one at a time.
A simple extension would be to interpolate moves in Cartesian space to pack a joint-space trajectory command for straight-line motions.
(Would need to define what is meant for interpolation of gripper orientations).
Another possible extension is to accept multiple (a vector of) Cartesian goal poses.

The cart_move_client_example has a pair of hard-coded gripper poses, which are sent to the cart_move_as action server.
This example may be extended to invoke motions using computed poses (e.g. vision guided)

## Example usage
Start up:
`roslaunch dvrk_model wsn_davinci_gazebo.launch`
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`
`rosrun cart_move_as cart_move_as`
`rosrun cart_move_as cart_move_client_example` 

    
