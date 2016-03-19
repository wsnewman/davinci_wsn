# davinci_kinematics

wsn DaVinci kinematics, forward and reverse

## Example usage
start simulator with:
`roslaunch dvrk_model wsn_psm_one_rviz.launch`

start a joint-state publisher with:
`rosrun davinci_kinematics davinci_cart_traj_pub`

This will cause the gripper tip to sweep out a circular path while holding the gripper orientation constant.

There are two libraries (classes) created here: davinci_kinematics and davinci_joint_publisher.
Example program davinci_cart_traj_pub.cpp instantiates a joint publisher from class DavinciJointPublisher, as
well as Davinci_fwd_solver and Davinci_IK_solver from davinci_kinematics.

In davinci_cart_traj_pub.cpp, a desired gripper path is generated in Cartesian coordinates (a sequence of affine3d objects),
and these poses are converted to joint coordinates via a Davinci_IK_solver.  These coordinates describe yaw, pitch,
insertion, tool-shaft rotation, wrist-bend and gripper-jaw tilt.  The DavinciJointPublisher encapsulates mapping these
commands to a vector that includes controlling the closed-chain joints.  Values published are illustrated by rviz.

## Running tests/demos
The "test_pointer" code is meant to test hand/eye coordination.  It subscribes to topic "thePoint", computes IK for the right gripper, 
and sends the psm1 gripper to point straight down with its tip at the specified coordinates.  This can be run with:

`roslaunch dvrk_model wsn_davinci_gazebo.launch`

`rosrun davinci_traj_streamer davinci_traj_interpolator_as`

`rosrun davinci_kinematics test_pointer`

Then, you can test points manually, e.g. with:

 `rostopic pub  /thePoint geometry_msgs/Point  '{x: -0.1, y: 0.0, z: 0.1}'`
    
