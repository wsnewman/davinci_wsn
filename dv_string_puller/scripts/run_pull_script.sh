rosrun sticky_fingers finger_control_dummy_node sticky_finger/two_fingertip1_sticky sticky

rosrun gazebo_ros spawn_model -sdf -database long_thin_rod -model ltr -z 0.5

rostopic pub --once /gazebo/set_model_state gazebo_msgs/ModelState '{
model_name: ltr,
pose: { position: { x: 0, y: -0.1, z: 0.247 }, orientation: {x: -0.491983115673, y: 0, z: 0, w: 0.870604813099 } },
twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  },
reference_frame: world
}'

rosrun playfile_reader playfile_jointspace dv_string_puller /play/jsp/pull_rh_prep.jsp

rosrun dv_string_puller pull_string_hardcoded

read X

rosrun playfile_reader playfile_jointspace  dv_string_puller /play/jsp/all_zero.jsp

rosrun sticky_fingers finger_control_dummy_node sticky_finger/one_fingertip1_sticky smooth
rosrun sticky_fingers finger_control_dummy_node sticky_finger/two_fingertip1_sticky smooth

rosservice call gazebo/delete_model '{model_name: ltr}'
