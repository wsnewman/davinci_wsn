wsn pgm to receive Davinci trajectories and interpolate them smoothly as commands to Davinci;

Sept 2015

`rosrun davinci_traj_streamer traj_interpolator_as`

source in: /src/traj_interpolator_as.cpp

See traj_action_client_pre_pose.cpp for example of how to use this action server
in 2nd terminal, run test client:

`rosrun davinci_traj_streamer davinci_traj_action_client_pre_pose` 

this will send two trajectory points to the action server as a goal, and the action server
  will decompose these into a stream of incremental commands, which it publishes using
  a DavinciJointPublisher object

action message goal includes trajectory_msgs/JointTrajectory
action server breaks this up into incremental commands, spaced at dt_traj
populates and publishes joint commands every dt_traj
complains and exits if have fewer than 2 trajectory points

At this point, the commands are published as a joint-state publisher, which drives Rviz only.
Possible extension to driving a gazebo model with a davinci_cmd message

values for dt_traj and max velocities are set in header, davinci_traj_streamer.h




