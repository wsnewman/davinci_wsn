function R = Rfnc(theta_x,psi_y,phi_z)
Rx = [1    	0	0
        0	cos(theta_x) -sin(theta_x)
        0	sin(theta_x), cos(theta_x)]
 Ry = [cos(psi_y)  	0	sin(psi_y)
 	0	1	0
 	-sin(psi_y)	0	cos(psi_y)]
 Rz = [cos(phi_z)	-sin(phi_z)	0
           sin(phi_z)	cos(phi_z)	0
           0		0	1]
           
 R = Rx*Ry*Rz
% return R
 endfunction
 
        