%calibration data; first, in pixels:
u_left = [	396;		
	194;		
	206;		
	412;		
	403;		
	187;		
	195;		
	410]'
	
v_vec = [	106  	
	113	
	317	
	306	
	94	
	101	
	317	
	305]'
	
u_right = [332
	127
	139
	349
	342
 	120
	130
	350]'
U_left = [u_left;v_vec]
U_right=[u_right;v_vec]

in2 = 2.0*0.0254; %2"
%corresponding known locations of fiducials in world frame
P_w = [	 in2   0   0; 	%say x to rt, y dn, z into view, parallel to z_camera
	0      0    0;
	0   in2    0;
	in2 in2   0;
	in2   0   0.005; 	
	0      0     0.005;
	0   in2     0.005;
	in2 in2    0.005]'

%assumes (cx,cy) in center, both left and right
cx_left = 640/2;
cx_right = cx_left;
cy_left = 480/2;
cy_right = cy_left;
c_left = [cx_left;cy_left]
c_right = [cx_right;cy_right]

%need to find these!
focus_in_pixels = 800  %assumes focus same for r and l
baseline = 0.005

origin_w_wrt_left_cam = [0;0;0.1] 	
%assume images are rectified, so rotations are the same...
origin_w_wrt_right_cam = origin_w_wrt_left_cam+ [-baseline;0;0]

theta_x= 0.0;
psi_y = 0.0;
phi_z= 0.0;
R_world_wrt_cam = Rfnc(theta_x,psi_y,phi_z)

Err = 0;
%compute fit errors:
for i=1:8
  u_vec = U_left(:,i)
  p_vec = P_w(:,i)
  e_vec =     	err_vec(focus_in_pixels,u_vec,R_world_wrt_cam,p_vec,origin_w_wrt_left_cam)
  Err = Err + e_vec'*e_vec
  end
  sqrtErr = sqrt(Err)
		
