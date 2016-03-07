%load file:
clear all
load gripper_poses_in_camera_coords.csp
xvals = gripper_poses_in_camera_coords(:,1);
zvals = gripper_poses_in_camera_coords(:,3);
plot(xvals,zvals)